"""
flow_node: dense optical flow between consecutive camera frames, published as FlowFrame messages.

Flow pairs are consecutive RGB frames, so the gap is always one camera period however slow the
machine is. Depth is needed only for the current frame: the depth image nearest in time to it is
attached when the pair is processed (its own stamp is kept in the depth image's header).

The RGB callback only pairs the frames and stores the newest pair in a one-slot mailbox. A worker
thread takes the newest pair, computes the flow and publishes the result, so a slow backend drops
pairs instead of building a queue.
"""

import collections
import threading
import time
import traceback

import cv2
import numpy as np
from rapid_nav_msgs.msg import FlowFrame
from rapid_nav_perception.backends import make_backend, torch_available
from rapid_nav_perception.conversions import array_to_image_fields, image_to_array
from rapid_nav_perception.flowvis import flow_to_rgb
from rapid_nav_perception.geometry import resize_depth_nearest, scale_intrinsics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

REPORT_PERIOD_S = 5.0
DEPTH_HISTORY = 30


def stamp_seconds(stamp):
    """Return a builtin_interfaces stamp in seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class FlowNode(Node):
    """Compute and publish the optical flow of consecutive RGB frames, with the matching depth."""

    def __init__(self):
        """Read the parameters, build the backend and set up the subscriptions and the worker."""
        super().__init__('flow_node')
        self.declare_parameter('backend', 'raft_small')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('width', 320)
        self.declare_parameter('max_rate', 10.0)
        self.declare_parameter('max_pair_gap', 0.2)
        self.declare_parameter('depth_slop', 0.04)
        self.declare_parameter('weights_path', '')
        self.declare_parameter('raft_iterations', 12)
        self.declare_parameter('num_threads', 0)
        self.declare_parameter('publish_vis', True)
        self.target_width = int(self.get_parameter('width').value)
        self.max_rate = float(self.get_parameter('max_rate').value)
        self.max_pair_gap = float(self.get_parameter('max_pair_gap').value)
        self.depth_slop = float(self.get_parameter('depth_slop').value)
        self.publish_vis = bool(self.get_parameter('publish_vis').value)

        requested = str(self.get_parameter('backend').value)
        self.backend, _warning = make_backend(
            requested,
            device=str(self.get_parameter('device').value),
            weights_path=str(self.get_parameter('weights_path').value) or None,
            iterations=int(self.get_parameter('raft_iterations').value),
            num_threads=int(self.get_parameter('num_threads').value),
            logger=self.get_logger(),
        )
        self.device = getattr(self.backend, 'device', 'cpu')
        self.torch_version = None
        if torch_available():
            import torch

            self.torch_version = torch.__version__

        self.info = None
        self.size = None
        self.previous = None
        self.mailbox = None
        self.stop_event = threading.Event()
        self.cond = threading.Condition()
        self.lock = threading.Lock()
        self.received = 0
        self.dropped = 0
        self.no_depth = 0
        self.depths = collections.deque(maxlen=DEPTH_HISTORY)
        self.processed = 0
        self.latencies = []

        sensor_qos = qos_profile_sensor_data
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image', self.on_rgb, sensor_qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.on_depth, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_info, sensor_qos)
        self.flow_pub = self.create_publisher(FlowFrame, '/perception/flow_frame', 10)
        self.vis_pub = self.create_publisher(Image, '/perception/flow_vis', 10)
        self.create_timer(REPORT_PERIOD_S, self.report)

        self.worker = threading.Thread(target=self.work, name='flow_worker', daemon=True)
        self.worker.start()

    def on_info(self, msg):
        """Keep the latest camera intrinsics; the processing size is set from the first one."""
        self.info = msg
        if self.size is None:
            self.choose_size(msg.width, msg.height)

    def choose_size(self, camera_width, camera_height):
        """Set the processing size: the target width, the camera aspect, RAFT multiples of 8."""
        height = round(camera_height * self.target_width / camera_width)
        width = self.target_width
        if self.backend.name.startswith('raft'):
            rounded = (height // 8) * 8, (width // 8) * 8
            if rounded != (height, width):
                self.get_logger().info(
                    'RAFT needs multiples of 8: {}x{} becomes {}x{}'.format(
                        width, height, rounded[1], rounded[0]))
            height, width = rounded
        self.size = (width, height)
        self.get_logger().info('flow_node: backend={} device={} size={}x{} torch={}'.format(
            self.backend.name, self.device, width, height, self.torch_version or 'none'))

    def on_depth(self, msg):
        """Keep the recent depth images; the worker picks the one nearest to its frame."""
        with self.lock:
            self.depths.append((stamp_seconds(msg.header.stamp), msg))

    def nearest_depth(self, stamp):
        """Return the kept depth image nearest in time to stamp, or None beyond depth_slop."""
        with self.lock:
            candidates = list(self.depths)
        if not candidates:
            return None
        depth_stamp, msg = min(candidates, key=lambda item: abs(item[0] - stamp))
        return msg if abs(depth_stamp - stamp) <= self.depth_slop else None

    def on_rgb(self, rgb_msg):
        """Pair the frame with the previous RGB frame and put a close pair into the mailbox."""
        stamp = stamp_seconds(rgb_msg.header.stamp)
        current = (rgb_msg, stamp)
        with self.lock:
            self.received += 1
        if self.previous is None:
            self.previous = current
            return
        previous_stamp = self.previous[1]
        gap = stamp - previous_stamp
        if gap <= 0.0 or gap > self.max_pair_gap:
            with self.lock:
                self.dropped += 1
            self.previous = current
            return
        pair = (self.previous, current)
        with self.cond:
            self.mailbox = pair
            self.cond.notify()
        self.previous = current

    def work(self):
        """Worker thread: take the newest pair, respect max_rate, compute and publish."""
        last_done = None
        while not self.stop_event.is_set():
            with self.cond:
                while self.mailbox is None and not self.stop_event.is_set():
                    self.cond.wait(0.1)
                if self.stop_event.is_set():
                    break
            if self.max_rate > 0.0 and last_done is not None:
                remaining = last_done + 1.0 / self.max_rate - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
            with self.cond:
                pair = self.mailbox
                self.mailbox = None
            if pair is None:
                continue
            info = self.info
            if info is None or self.size is None:
                continue
            try:
                self.process(pair, info)
            except Exception:  # keep the worker alive; log the traceback
                self.get_logger().error('flow worker: {}'.format(traceback.format_exc()))
            last_done = time.monotonic()

    def process(self, pair, info):
        """Compute the flow of one pair and publish the FlowFrame (and the picture)."""
        (prev_rgb_msg, _), (cur_rgb_msg, cur_stamp) = pair
        cur_depth_msg = self.nearest_depth(cur_stamp)
        if cur_depth_msg is None:
            with self.lock:
                self.no_depth += 1
            return
        width, height = self.size
        camera_height, camera_width = int(info.height), int(info.width)
        first = self.resize_rgb(image_to_array(prev_rgb_msg), width, height)
        second = self.resize_rgb(image_to_array(cur_rgb_msg), width, height)

        started = time.perf_counter()
        flow = self.backend.compute(first, second)
        latency_ms = (time.perf_counter() - started) * 1000.0

        depth = image_to_array(cur_depth_msg)
        depth = resize_depth_nearest(depth, width, height).astype(np.float32)
        fx, fy, cx, cy = scale_intrinsics(
            info.k[0], info.k[4], info.k[2], info.k[5],
            width / float(camera_width), height / float(camera_height))
        scaled = CameraInfo()
        scaled.header = cur_rgb_msg.header
        scaled.height = height
        scaled.width = width
        scaled.distortion_model = info.distortion_model
        scaled.d = list(info.d)
        scaled.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        scaled.r = list(info.r)
        scaled.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        scaled.binning_x = info.binning_x
        scaled.binning_y = info.binning_y
        scaled.roi = info.roi

        message = FlowFrame()
        message.header = cur_rgb_msg.header
        message.previous_stamp = prev_rgb_msg.header.stamp
        h, w, encoding, step, data = array_to_image_fields(flow, '32FC2')
        message.flow = self.make_image(cur_rgb_msg.header, h, w, encoding, step, data)
        h, w, encoding, step, data = array_to_image_fields(depth, '32FC1')
        message.depth = self.make_image(cur_depth_msg.header, h, w, encoding, step, data)
        message.camera_info = scaled
        message.backend = self.backend.name
        message.latency_ms = float(latency_ms)
        self.flow_pub.publish(message)

        if self.publish_vis:
            picture = flow_to_rgb(flow)
            h, w, encoding, step, data = array_to_image_fields(picture, 'rgb8')
            self.vis_pub.publish(self.make_image(cur_rgb_msg.header, h, w, encoding, step, data))

        with self.lock:
            self.processed += 1
            self.latencies.append(latency_ms)

    @staticmethod
    def resize_rgb(rgb, width, height):
        """Resize an RGB image to the processing size with area averaging; no-op when equal."""
        if rgb.shape[1] == width and rgb.shape[0] == height:
            return rgb
        return cv2.resize(rgb, (width, height), interpolation=cv2.INTER_AREA)

    @staticmethod
    def make_image(header, height, width, encoding, step, data):
        """Return a sensor_msgs/Image with the given header and fields."""
        image = Image()
        image.header = header
        image.height = height
        image.width = width
        image.encoding = encoding
        image.is_bigendian = 0
        image.step = step
        image.data = data
        return image

    def report(self):
        """Log the counters and the median latency once per period."""
        with self.lock:
            latencies = list(self.latencies)
            self.latencies = []
            received, dropped, processed = self.received, self.dropped, self.processed
            no_depth = self.no_depth
        median = float(np.median(latencies)) if latencies else float('nan')
        self.get_logger().info(
            'flow_node: received={} processed={} dropped_too_far={} no_depth={} '
            'median_latency_ms={:.1f}'.format(received, processed, dropped, no_depth, median))

    def stop(self):
        """Stop the worker thread and wait for it."""
        self.stop_event.set()
        with self.cond:
            self.cond.notify_all()
        if self.worker.is_alive():
            self.worker.join(timeout=5.0)


def main(args=None):
    """Run the flow node until Ctrl-C."""
    rclpy.init(args=args)
    node = FlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
