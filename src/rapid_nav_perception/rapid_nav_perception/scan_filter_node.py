"""
The lidar scan without the tracked moving obstacles, for SLAM.

Subscribes to /scan and to the fused tracks (/obstacles/tracked, map frame). Each scan's beams that
end inside a track's predicted disk (scan_filter.py) become NaN, and the scan goes out on
/scan_static with the same header and beam count. Without a fresh track the scan is republished
unchanged, with no TF lookup.

The tracks are moved into the laser frame with the latest map -> odom and odom -> laser at the scan
stamp (lookup_transform_full, fixed frame odom), so the node never waits for the map -> odom that
SLAM computes from this very scan. When that transform is missing the raw scan goes out and is
counted. The raw /scan stays for the costmaps and for fusion.
"""

import math
import sys
import time

import numpy as np
from rapid_nav_msgs.msg import TrackedObstacleArray
from rapid_nav_perception import scan_filter
from rapid_nav_perception.flow_tracker import stamp_seconds
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

REPORT_PERIOD_S = 5.0


def planar(transform):
    """Return (R 2x2, t 2) of the planar part of a geometry_msgs Transform."""
    q = transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    return (np.array([[c, -s], [s, c]]),
            np.array([transform.translation.x, transform.translation.y]))


class ScanFilterNode(Node):
    """Republish /scan with the beams on tracked obstacles removed."""

    DEFAULTS = {
        'margin': scan_filter.MARGIN, 'k_sigma': scan_filter.K_SIGMA,
        'max_radius': scan_filter.MAX_RADIUS, 'min_speed': scan_filter.MIN_SPEED,
        'max_age': scan_filter.MAX_AGE, 'tf_timeout': 0.05,
    }

    def __init__(self):
        """Declare the parameters, set up TF, the subscriptions and the publisher."""
        super().__init__('scan_filter')
        for name, value in self.DEFAULTS.items():
            self.declare_parameter(name, value)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('tracks_topic', '/obstacles/tracked')
        self.declare_parameter('output_topic', '/scan_static')
        self.declare_parameter('fixed_frame', 'odom')
        self.p = {name: float(self.get_parameter(name).value) for name in self.DEFAULTS}
        self.fixed = self.get_parameter('fixed_frame').value
        self.tracks = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(TrackedObstacleArray, self.get_parameter('tracks_topic').value,
                                 self.on_tracks, 10)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self.on_scan,
                                 qos_profile_sensor_data)
        self.counts = {'scans': 0, 'filtered': 0, 'beams': 0, 'no_tf': 0}
        self.last_report = time.monotonic()

    def on_tracks(self, msg):
        """Keep the latest track array."""
        self.tracks = msg

    def map_to_laser(self, scan):
        """Return (R, t) from the tracks' frame to the laser at the scan stamp, or None."""
        try:
            t = self.buffer.lookup_transform_full(
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp, clock_type=self.get_clock().clock_type),
                self.tracks.header.frame_id, Time(clock_type=self.get_clock().clock_type),
                self.fixed, timeout=Duration(seconds=self.p['tf_timeout']))
        except TransformException:
            return None
        return planar(t.transform)

    def on_scan(self, msg):
        """Filter one scan and publish it."""
        self.counts['scans'] += 1
        out = msg
        tracks = []
        dt = None
        if self.tracks is not None and self.tracks.obstacles:
            dt = stamp_seconds(msg.header.stamp) - stamp_seconds(self.tracks.header.stamp)
            tracks = scan_filter.select(
                [{'x': o.position.x, 'y': o.position.y, 'vx': o.velocity.x, 'vy': o.velocity.y,
                  'radius': o.radius, 'position_sigma': o.position_sigma,
                  'velocity_sigma': o.velocity_sigma} for o in self.tracks.obstacles],
                abs(dt), self.p['min_speed'], self.p['max_age'])
        if tracks:
            transform = self.map_to_laser(msg)
            if transform is None:
                self.counts['no_tf'] += 1
            else:
                ranges, mask = scan_filter.filter_scan(
                    msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                    tracks, dt, transform, self.p['margin'], self.p['k_sigma'],
                    self.p['max_radius'])
                removed = int(np.count_nonzero(mask))
                if removed:
                    out = LaserScan()
                    out.header = msg.header
                    out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
                    out.angle_increment = msg.angle_increment
                    out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
                    out.range_min, out.range_max = msg.range_min, msg.range_max
                    out.ranges = ranges.tolist()
                    out.intensities = msg.intensities
                    self.counts['filtered'] += 1
                    self.counts['beams'] += removed
        self.pub.publish(out)
        self.report()

    def report(self):
        """Log the counters once per period."""
        now = time.monotonic()
        if now - self.last_report < REPORT_PERIOD_S:
            return
        self.last_report = now
        c = self.counts
        self.get_logger().info(
            'scan_filter: scans={} filtered={} beams_removed={} no_tf={}'.format(
                c['scans'], c['filtered'], c['beams'], c['no_tf']))


def main(args=None):
    """Run the node until shut down."""
    rclpy.init(args=args)
    node = ScanFilterNode()
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
