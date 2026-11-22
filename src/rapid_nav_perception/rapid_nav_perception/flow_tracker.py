"""
Camera-based obstacle tracks: moving regions found in the optical flow, as TrackedObstacleArray.

For each FlowFrame the node removes the robot's own motion (depth at both times and the camera pose
from TF at both times, see egomotion.py), groups the samples that still move into obstacles on the
floor and tracks them over time (tracking.py). It reads only the camera stream and TF: no ground
truth.
"""

import math
import sys
import time

import numpy as np
from rapid_nav_msgs.msg import FlowFrame, TrackedObstacle, TrackedObstacleArray
from rapid_nav_perception.conversions import image_to_array
from rapid_nav_perception.egomotion import moving_mask, sample_points, world_velocities
from rapid_nav_perception.tracking import cluster, Tracker
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

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
        'stride': 4, 'min_depth': 0.3, 'max_depth': 6.0, 'min_point_speed': 0.20,
        'max_point_speed': 3.0, 'min_height': 0.05, 'max_height': 2.0, 'cell': 0.10,
        'min_samples': 6, 'gate': 0.6, 'alpha': 0.5, 'max_track_age': 1.0, 'confirm': 2,
        'tf_timeout': 0.2,
    }

    def __init__(self):
        """Declare the parameters, subscribe to the flow frames and set up TF."""
        super().__init__('flow_tracker')
        for name, value in self.DEFAULTS.items():
            self.declare_parameter(name, value)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('topic', '/obstacles/tracked')
        self.p = {name: self.get_parameter(name).value for name in self.DEFAULTS}
        self.target = self.get_parameter('target_frame').value
        self.tracker = Tracker(self.p['gate'], self.p['alpha'], self.p['max_track_age'],
                               int(self.p['confirm']))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.pub = self.create_publisher(
            TrackedObstacleArray, self.get_parameter('topic').value, 10)
        self.create_subscription(FlowFrame, '/perception/flow_frame', self.on_frame, 2)
        self.counts = {'frames': 0, 'used': 0, 'no_tf': 0, 'bad_dt': 0, 'few': 0}
        self.moving_samples = 0
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

    def on_frame(self, msg):
        """Process one FlowFrame and publish the confirmed tracks."""
        self.counts['frames'] += 1
        t0, t1 = stamp_seconds(msg.previous_stamp), stamp_seconds(msg.header.stamp)
        dt = t1 - t0
        if dt <= 0.0:
            self.counts['bad_dt'] += 1
            return
        frame = msg.header.frame_id
        pose0 = self.pose(frame, msg.previous_stamp)
        pose1 = self.pose(frame, msg.header.stamp)
        if pose0 is None or pose1 is None:
            self.counts['no_tf'] += 1
            return
        k = msg.camera_info.k
        p = self.p
        _, x0, x1 = sample_points(
            image_to_array(msg.flow), image_to_array(msg.previous_depth),
            image_to_array(msg.depth), (k[0], k[4], k[2], k[5]), int(p['stride']),
            p['min_depth'], p['max_depth'])
        self.counts['used'] += 1
        if x0.shape[0] == 0:
            self.counts['few'] += 1
            clusters = []
        else:
            w1, w = world_velocities(x0, x1, pose0, pose1, dt)
            mask = moving_mask(w1, w, p['min_point_speed'], p['max_point_speed'],
                               p['min_height'], p['max_height'])
            self.moving_samples += int(mask.sum())
            clusters = cluster(w1[mask, :2], w[mask, :2], p['cell'], int(p['min_samples']))
        self.tracker.update(clusters, t1)
        self.publish(msg.header.stamp, t1)
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
            'flow_tracker: frames={} used={} skipped_no_tf={} skipped_bad_dt={} no_samples={} '
            'moving_samples={} tracks={}'.format(
                c['frames'], c['used'], c['no_tf'], c['bad_dt'], c['few'], self.moving_samples,
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
