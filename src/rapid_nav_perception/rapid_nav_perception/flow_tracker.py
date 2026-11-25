"""
Camera-based obstacle tracks: moving regions found in the optical flow, as TrackedObstacleArray.

For each FlowFrame the node removes the robot's own motion (depth at both times and the camera pose
from TF at both times, see egomotion.py), groups the samples that still move into obstacles on the
floor and tracks them over time (tracking.py). It reads only the camera stream and TF: no ground
truth.

With the parameter `uncertainty` true (the fused hazard source) each track also carries the
reliability r of its latest frame as its confidence, and a position and velocity sigma from the
spread of its cluster scaled by 1 / r (tracking.frame_reliability, tracking.track_sigmas). With
false (the default) the tracks are published exactly as before, sigmas 0.
"""

import collections
import math
import sys
import time

import numpy as np
from rapid_nav_msgs.msg import FlowFrame, TrackedObstacle, TrackedObstacleArray
from rapid_nav_perception.conversions import image_to_array
from rapid_nav_perception.egomotion import (
    block_texture, depth_speed_floor, moving_mask, sample_points, texture_mask, world_velocities)
from rapid_nav_perception.tracking import cluster, frame_reliability, track_sigmas, Tracker
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

IMAGE_HISTORY = 12
REPORT_PERIOD_S = 5.0


def stamp_seconds(stamp):
    """Return a builtin_interfaces/Time as seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def matrix_of(transform):
    """Return the 4 x 4 matrix of a geometry_msgs/Transform."""
    q, d = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    m = np.eye(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = [d.x, d.y, d.z]
    return m


class FlowTracker(Node):
    """Turn FlowFrames into tracked obstacles in the map frame."""

    DEFAULTS = {
        'stride': 4, 'depth_block': 4, 'min_depth': 0.3, 'max_depth': 4.0,
        'min_point_speed': 0.20, 'min_speed': 0.20, 'max_point_speed': 3.0, 'min_height': 0.05,
        'max_height': 2.0, 'cell': 0.10,
        'min_samples': 10, 'gate': 0.6, 'alpha': 0.5, 'max_track_age': 1.0, 'confirm': 2,
        'tf_timeout': 0.2, 'min_texture': 6.0, 'gate_k': 0.0, 'flow_sigma_px': 0.5,
    }

    def __init__(self):
        """Declare the parameters, subscribe to the flow frames and set up TF."""
        super().__init__('flow_tracker')
        for name, value in self.DEFAULTS.items():
            self.declare_parameter(name, value)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('topic', '/obstacles/tracked')
        self.declare_parameter('uncertainty', False)
        # 'shared' (default): the TF listener spins this node in a second executor. 'dedicated': it
        # gets a node of its own, so /tf keeps arriving while on_frame waits for the transform at
        # the newest depth stamp (with the shared node the wait sees no new data and times out).
        self.declare_parameter('tf_listener_node', 'shared')
        listener_node = str(self.get_parameter('tf_listener_node').value).strip().lower()
        if listener_node not in ('shared', 'dedicated'):
            raise ValueError("tf_listener_node must be 'shared' or 'dedicated', got '{}'".format(
                listener_node))
        self.uncertainty = bool(self.get_parameter('uncertainty').value)
        self.p = {name: self.get_parameter(name).value for name in self.DEFAULTS}
        self.target = self.get_parameter('target_frame').value
        self.tracker = Tracker(self.p['gate'], self.p['alpha'], self.p['max_track_age'],
                               int(self.p['confirm']))
        self.buffer = Buffer()
        self.listener = TransformListener(
            self.buffer, self if listener_node == 'shared' else None, spin_thread=True)
        self.pub = self.create_publisher(
            TrackedObstacleArray, self.get_parameter('topic').value, 10)
        self.create_subscription(FlowFrame, '/perception/flow_frame', self.on_frame, 2)
        self.images = collections.deque(maxlen=IMAGE_HISTORY)
        if self.p['min_texture'] > 0.0:
            self.create_subscription(
                Image, '/camera/image', self.on_image, qos_profile_sensor_data)
        self.counts = {'frames': 0, 'used': 0, 'no_tf': 0, 'bad_dt': 0, 'few': 0, 'no_image': 0}
        self.moving_samples = 0
        self.reliability = 1.0
        self.last_report = time.monotonic()

    def pose(self, frame, stamp):
        """Return the matrix map <- frame at the stamp, or None when TF cannot give it."""
        try:
            t = self.buffer.lookup_transform(
                self.target, frame, Time(seconds=stamp_seconds(stamp), nanoseconds=0,
                                         clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=float(self.p['tf_timeout'])))
        except TransformException:
            return None
        return matrix_of(t.transform)

    def on_image(self, msg):
        """Keep the grey image (processing size not known yet: the full frame) of recent frames."""
        grey = image_to_array(msg).astype(np.float64).mean(axis=2)
        self.images.append((stamp_seconds(msg.header.stamp), grey))

    def texture_of(self, stamp, shape, tile):
        """Return the structure-tensor texture per tile of the image nearest the stamp, or None."""
        if not self.images:
            return None
        seconds = stamp_seconds(stamp)
        when, grey = min(self.images, key=lambda e: abs(e[0] - seconds))
        if abs(when - seconds) > 0.05:
            return None
        factor = max(1, grey.shape[0] // shape[0])
        if factor > 1:
            h, w = shape[0] * factor, shape[1] * factor
            grey = grey[:h, :w].reshape(shape[0], factor, shape[1], factor).mean(axis=(1, 3))
        return block_texture(grey, tile)

    def on_frame(self, msg):
        """Process one FlowFrame and publish the confirmed tracks."""
        self.counts['frames'] += 1
        # The points come from the depth images, so the poses and dt are those of the depth stamps
        # (they differ from the colour stamps by up to 0.03 s, which is 0.015 rad at 0.5 rad/s).
        s0, s1 = msg.previous_depth.header.stamp, msg.depth.header.stamp
        t0, t1 = stamp_seconds(s0), stamp_seconds(s1)
        dt = t1 - t0
        if dt <= 0.0:
            self.counts['bad_dt'] += 1
            return
        frame = msg.header.frame_id
        pose0 = self.pose(frame, s0)
        pose1 = self.pose(frame, s1)
        if pose0 is None or pose1 is None:
            self.counts['no_tf'] += 1
            return
        k = msg.camera_info.k
        p = self.p
        flow = image_to_array(msg.flow)
        block = int(p['depth_block'])
        pix, x0, x1 = sample_points(
            flow, image_to_array(msg.previous_depth), image_to_array(msg.depth),
            (k[0], k[4], k[2], k[5]), int(p['stride']), p['min_depth'], p['max_depth'], block)
        total = x0.shape[0]
        if p['min_texture'] > 0.0:
            # Where the previous image has no texture in two directions the flow is a guess.
            tile = block if block > 1 else int(p['stride'])
            texture = self.texture_of(msg.previous_stamp, flow.shape[:2], tile)
            if texture is None:
                self.counts['no_image'] += 1
                return
            keep = texture_mask(pix, texture, tile, p['min_texture'])
            x0, x1 = x0[keep], x1[keep]
        self.reliability = frame_reliability(msg.backend, x0.shape[0], total)
        self.counts['used'] += 1
        if x0.shape[0] == 0:
            self.counts['few'] += 1
            clusters = []
        else:
            w1, w = world_velocities(x0, x1, pose0, pose1, dt)
            # Flow error fakes a larger speed the farther the point is: raise the floor with depth.
            floor = np.maximum(p['min_point_speed'], depth_speed_floor(
                x1[:, 2], k[0], dt, p['gate_k'], p['flow_sigma_px']))
            mask = moving_mask(w1, w, floor, p['max_point_speed'],
                               p['min_height'], p['max_height'])
            self.moving_samples += int(mask.sum())
            clusters = cluster(w1[mask, :2], w[mask, :2], p['cell'], int(p['min_samples']))
            # Depth noise gives random velocities; a real mover has a mean velocity of its own.
            clusters = [c for c in clusters if math.hypot(c['vx'], c['vy']) >= p['min_speed']]
        if self.uncertainty:
            for c in clusters:
                c['position_sigma'], c['velocity_sigma'] = track_sigmas(c, self.reliability)
                c['reliability'] = self.reliability
        self.tracker.update(clusters, t1)
        self.publish(s1, t1)
        self.report()

    def publish(self, stamp, t1):
        """Publish the confirmed tracks (an empty array when there are none)."""
        out = TrackedObstacleArray()
        out.header.stamp = stamp
        out.header.frame_id = self.target
        out.source = 'flow'
        for tr in self.tracker.confirmed(t1):
            o = TrackedObstacle()
            o.id = tr['id']
            o.position.x, o.position.y = tr['x'], tr['y']
            o.velocity.x, o.velocity.y = tr['vx'], tr['vy']
            o.radius = tr['radius']
            o.confidence = float(tr['confidence'])
            if self.uncertainty:
                o.confidence = float(tr.get('reliability', self.reliability))
                o.position_sigma = float(tr.get('position_sigma', 0.0))
                o.velocity_sigma = float(tr.get('velocity_sigma', 0.0))
            out.obstacles.append(o)
        self.pub.publish(out)

    def report(self):
        """Log the counters once per period."""
        now = time.monotonic()
        if now - self.last_report < REPORT_PERIOD_S:
            return
        self.last_report = now
        c = self.counts
        self.get_logger().info(
            'flow_tracker: frames={} used={} skipped_no_tf={} skipped_bad_dt={} '
            'no_image={} no_samples={} moving_samples={} reliability={:.2f} tracks={}'.format(
                c['frames'], c['used'], c['no_tf'], c['bad_dt'], c['no_image'], c['few'],
                self.moving_samples, self.reliability,
                len(self.tracker.confirmed())))


def main(args=None):
    """Run the node until shut down."""
    rclpy.init(args=args)
    node = FlowTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
