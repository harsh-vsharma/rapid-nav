"""
Fused obstacle tracks: lidar clusters and camera tracks in one Kalman filter per obstacle.

Subscribes to /scan (points moved to the map frame with TF at the scan stamp), to the static map
(/map, transient local) and to the camera tracks (/obstacles/flow, from flow_tracker with
uncertainty on). With map_mode 'slam' the map is the one slam_toolbox grows: it is rebuilt at
every update, cells that are not known to be free count as static, and with track_guard the
points around a confirmed moving track are kept even where the map has absorbed it. Each
measurement updates the filter at its own stamp (fusion.py). The confirmed
tracks are published at a fixed rate on /obstacles/tracked with source 'fused', predicted to the
time of the message, with their position and velocity sigmas.
"""

import sys
import time

from nav_msgs.msg import OccupancyGrid
import numpy as np
from rapid_nav_msgs.msg import TrackedObstacle, TrackedObstacleArray
from rapid_nav_perception import fusion
from rapid_nav_perception.flow_tracker import matrix_of, stamp_seconds
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

REPORT_PERIOD_S = 5.0
OCCUPIED = 65  # map value (0..100) from which a cell is occupied (map_server occupied_thresh 0.65)
FREE = 25      # map value below which a known cell is free (free_thresh 0.25); -1 is unknown
MAP_MODES = ('fixed', 'slam')


class FusionNode(Node):
    """Run the fusion filter on the lidar and the camera tracks and publish the fused tracks."""

    DEFAULTS = {
        'rate': 10.0, 'link': fusion.LINK, 'min_points': fusion.MIN_POINTS,
        'static_clearance': fusion.STATIC_CLEARANCE, 'q': fusion.Q_ACCEL,
        'lidar_sigma': fusion.LIDAR_SIGMA, 'gate': fusion.GATE, 'confirm': fusion.CONFIRM,
        'max_age': fusion.MAX_AGE, 'init_velocity_sigma': fusion.INIT_VELOCITY_SIGMA,
        'max_range': 8.0, 'tf_timeout': 0.1, 'guard_min_speed': 0.2,
        'guard_extra': fusion.STATIC_CLEARANCE,
    }

    def __init__(self):
        """Declare the parameters, set up TF, the subscriptions and the publishing timer."""
        super().__init__('fusion_node')
        for name, value in self.DEFAULTS.items():
            self.declare_parameter(name, value)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('topic', '/obstacles/tracked')
        self.declare_parameter('flow_topic', '/obstacles/flow')
        self.declare_parameter('map_mode', 'fixed')
        self.declare_parameter('track_guard', False)
        self.map_mode = str(self.get_parameter('map_mode').value).strip().lower()
        if self.map_mode not in MAP_MODES:
            raise ValueError("map_mode must be 'fixed' or 'slam', got '{}'".format(self.map_mode))
        self.track_guard = bool(self.get_parameter('track_guard').value)
        self.p = {name: self.get_parameter(name).value for name in self.DEFAULTS}
        self.target = self.get_parameter('target_frame').value
        self.fusion = fusion.Fusion(self.p['q'], self.p['lidar_sigma'], self.p['gate'],
                                    int(self.p['confirm']), self.p['max_age'],
                                    self.p['init_velocity_sigma'])
        self.grid = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.pub = self.create_publisher(
            TrackedObstacleArray, self.get_parameter('topic').value, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(TrackedObstacleArray, self.get_parameter('flow_topic').value,
                                 self.on_flow, 10)
        self.create_timer(1.0 / float(self.p['rate']), self.publish)
        self.counts = {'scans': 0, 'no_map': 0, 'no_tf': 0, 'clusters': 0, 'flow': 0,
                       'flow_tracks': 0, 'maps': 0, 'guarded': 0}
        self.last_report = time.monotonic()

    def on_map(self, msg):
        """Keep the map as a mask of the static cells (near occupied; with slam also unknown)."""
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        known_free = None
        if self.map_mode == 'slam':
            known_free = (data >= 0) & (data < FREE)
        self.grid = fusion.StaticGrid(data >= OCCUPIED, info.origin.position.x,
                                      info.origin.position.y, info.resolution,
                                      float(self.p['static_clearance']), known_free)
        self.counts['maps'] += 1
        if self.map_mode == 'fixed' or self.counts['maps'] == 1:
            self.get_logger().info('fusion_node: {} map {} x {} at {} m'.format(
                'static' if self.map_mode == 'fixed' else 'first slam', info.width, info.height,
                info.resolution))

    def scan_points(self, msg):
        """Return the scan's valid points (N, 2) in the target frame, or None without TF."""
        try:
            t = self.buffer.lookup_transform(
                self.target, msg.header.frame_id,
                Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=float(self.p['tf_timeout'])))
        except TransformException:
            return None
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        angles = msg.angle_min + msg.angle_increment * np.arange(ranges.shape[0])
        top = min(float(msg.range_max), float(self.p['max_range']))
        ok = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < top)
        r, a = ranges[ok], angles[ok]
        local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r), np.ones_like(r)])
        return (matrix_of(t.transform) @ local)[:2].T

    def on_scan(self, msg):
        """Cluster the scan points off the static map and update the filter."""
        self.counts['scans'] += 1
        if self.grid is None:
            self.counts['no_map'] += 1
            return
        points = self.scan_points(msg)
        if points is None:
            self.counts['no_tf'] += 1
            return
        t = stamp_seconds(msg.header.stamp)
        keep = ()
        if self.track_guard:
            keep = self.fusion.guard_disks(t, float(self.p['guard_min_speed']),
                                           float(self.p['guard_extra']))
            self.counts['guarded'] += len(keep)
        clusters = fusion.lidar_clusters(points, self.grid, float(self.p['link']),
                                         int(self.p['min_points']), keep)
        self.counts['clusters'] += len(clusters)
        self.fusion.update_lidar(clusters, t)

    def on_flow(self, msg):
        """Update the filter with the camera tracks (map frame) at their stamp."""
        if msg.header.frame_id != self.target:
            self.get_logger().warn(
                'fusion_node: camera tracks in {}, expected {}'.format(
                    msg.header.frame_id, self.target), throttle_duration_sec=5.0)
            return
        self.counts['flow'] += 1
        self.counts['flow_tracks'] += len(msg.obstacles)
        obstacles = [{'x': o.position.x, 'y': o.position.y, 'vx': o.velocity.x,
                      'vy': o.velocity.y, 'position_sigma': o.position_sigma,
                      'velocity_sigma': o.velocity_sigma} for o in msg.obstacles]
        self.fusion.update_flow(obstacles, stamp_seconds(msg.header.stamp))

    def publish(self):
        """Publish the confirmed tracks predicted to now (an empty array when there are none)."""
        now = self.get_clock().now()
        out = TrackedObstacleArray()
        out.header.stamp = now.to_msg()
        out.header.frame_id = self.target
        out.source = 'fused'
        for tr in self.fusion.confirmed(now.nanoseconds * 1e-9):
            o = TrackedObstacle()
            o.id = tr['id']
            o.position.x, o.position.y = tr['x'], tr['y']
            o.velocity.x, o.velocity.y = tr['vx'], tr['vy']
            o.radius = float(tr['radius'])
            o.confidence = float(tr['confidence'])
            o.position_sigma = float(tr['position_sigma'])
            o.velocity_sigma = float(tr['velocity_sigma'])
            out.obstacles.append(o)
        self.pub.publish(out)
        self.report(len(out.obstacles))

    def report(self, published):
        """Log the counters once per period."""
        now = time.monotonic()
        if now - self.last_report < REPORT_PERIOD_S:
            return
        self.last_report = now
        c = self.counts
        self.get_logger().info(
            'fusion_node: scans={} no_map={} no_tf={} lidar_clusters={} flow_msgs={} '
            'flow_tracks={} tracks={} published={} maps={} guarded={}'.format(
                c['scans'], c['no_map'], c['no_tf'], c['clusters'], c['flow'], c['flow_tracks'],
                len(self.fusion.tracks), published, c['maps'], c['guarded']))


def main(args=None):
    """Run the node until shut down."""
    rclpy.init(args=args)
    node = FusionNode()
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
