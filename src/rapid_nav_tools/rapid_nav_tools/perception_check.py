"""
Check the optical flow pipeline on the simulated robot and write a bundle.

Start the simulation with the flow node (ros2 launch rapid_nav_bringup sim.launch.py
perception:=true), then run:

ros2 run rapid_nav_tools perception_check [--out DIR] [--profile native|docker] [--no-drive]

The checks need no flow ground truth. They use the robot's own sensors: the depth image against the
lidar (geometry), the flow of a still scene (at rest), the flow while driving forward (expansion)
and the flow while turning in place against the gyro's prediction (rotation). The bundle in DIR
holds environment.txt, results.json (every per-frame value), summary.md, flow_forward.png,
rgb_forward.png, flow_rotate.png and rgb_rotate.png. The exit status is 0 when no check fails, else
1.
"""

import argparse
import collections
import datetime
import math
import os
import sys
import time

from geometry_msgs.msg import Twist
import numpy as np
from PIL import Image
from rapid_nav_msgs.msg import FlowFrame
from rapid_nav_perception.conversions import image_to_array
from rapid_nav_perception.flowvis import flow_to_rgb
from rapid_nav_perception.geometry import backproject_row
from rapid_nav_tools.bundle import (
    environment_lines,
    format_number,
    Report,
    stamp_seconds,
    summary_text,
    verdict_of,
    write_outputs,
)
from rapid_nav_tools.checks import (
    band,
    central_window,
    predicted_rotation_flow,
    predicted_turn_flow_u,
    scan_range_at,
    transform_points,
)
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Imu, LaserScan
from sensor_msgs.msg import Image as ImageMsg
import tf2_ros

FLOW_WAIT_S = 90.0
CLOCK_WAIT_S = 60.0
SETTLE_SIM_S = 2.0
REST_SIM_S = 2.0
CMD_PERIOD_S = 0.05
DRIVE_SPEED = 0.3
DRIVE_TIME = 8.0  # 2.4 m along the free strip; long enough for slow backends
STOP_TIME = 1.0
TURN_RATE = 0.6
TURN_TIME = 10.472  # one full turn at TURN_RATE
SKIP_FIRST_S = 1.0
MIN_FRAMES = 3
GEOMETRY_COLUMN_STEP = 4
GEOMETRY_MIN_POINTS = 50
GEOMETRY_MEDIAN_M = 0.05
GEOMETRY_WITHIN_M = 0.10
GEOMETRY_WITHIN_FRACTION = 0.80
REST_MEDIAN_PX = 0.25
FORWARD_FLOOR_V_PX = 0.3
ROTATE_RATIO = (0.90, 1.10)
ROTATE_VERTICAL_RATIO = 0.15
IMAGE_KEEP = 60
FLOW_LAG_S = 180.0


class PerceptionCheck(Node):
    """Collect the flow frames, the sensors and the TF tree, and drive the robot."""

    def __init__(self, profile, out_dir):
        """Create the subscriptions, the command publisher and the TF listener."""
        super().__init__(
            'perception_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.profile = profile
        self.out_dir = out_dir
        self.values = {}
        self.clock_pairs = []
        self.flow_records = []
        self.first_flow = None
        self.depth = None
        self.lever = None
        self.info = None
        self.scan = None
        self.imu = []
        self.rgb_frames = collections.deque(maxlen=IMAGE_KEEP)
        self.windows = {}
        self.pictures = {}
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscriptions_kept = [
            self.create_subscription(Clock, '/clock', self.on_clock, qos),
            self.create_subscription(FlowFrame, '/perception/flow_frame', self.on_flow, qos),
            self.create_subscription(ImageMsg, '/camera/image', self.on_rgb, qos),
            self.create_subscription(ImageMsg, '/camera/depth_image', self.on_depth, qos),
            self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos),
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos),
            self.create_subscription(Imu, '/imu', self.on_imu, qos),
        ]
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # --- subscriptions -------------------------------------------------------------------------

    def on_clock(self, msg):
        """Remember the simulated and wall-clock times for the real-time factor."""
        self.clock_pairs.append((stamp_seconds(msg.clock), time.monotonic()))

    def on_rgb(self, msg):
        """Keep the last colour frames, to save the one with the same stamp as a flow frame."""
        self.rgb_frames.append((stamp_seconds(msg.header.stamp), msg))

    def on_depth(self, msg):
        """Keep the latest depth image."""
        self.depth = msg

    def on_info(self, msg):
        """Keep the latest camera intrinsics."""
        self.info = msg

    def on_scan(self, msg):
        """Keep the latest lidar scan."""
        self.scan = msg

    def on_imu(self, msg):
        """Keep the gyro z and the stamp of every IMU sample."""
        self.imu.append((stamp_seconds(msg.header.stamp), msg.angular_velocity.z))

    def on_flow(self, msg):
        """Compute the per-frame values of one flow frame and keep pictures of the windows."""
        if self.first_flow is None:
            self.first_flow = msg
        record = self.frame_record(msg)
        self.flow_records.append(record)
        for name, (start, end) in self.windows.items():
            if start <= record['previous_stamp'] and record['stamp'] < end:
                colour = self.colour_at(record['stamp'])
                self.pictures[name] = (record['stamp'], image_to_array(msg.flow).copy(), colour)

    def colour_at(self, stamp):
        """Return the colour frame with this stamp as an array, or None when not kept."""
        for frame_stamp, msg in self.rgb_frames:
            if abs(frame_stamp - stamp) < 1e-6:
                return image_to_array(msg).copy()
        return None

    def frame_record(self, msg):
        """Return the per-frame values: flow magnitudes in image bands, and the gyro z."""
        flow = image_to_array(msg.flow)
        magnitude = np.hypot(flow[..., 0], flow[..., 1])
        stamp = stamp_seconds(msg.header.stamp)
        previous = stamp_seconds(msg.previous_stamp)
        dt = stamp - previous
        omega = self.mean_gyro(previous, stamp)
        bottom = band(flow, 0.75, 1.0, 0.0, 1.0)
        left = band(flow, 0.0, 1.0, 0.0, 1.0 / 3.0)
        right = band(flow, 0.0, 1.0, 2.0 / 3.0, 1.0)
        centre = central_window(flow, 0.5)
        fx = float(msg.camera_info.k[0])
        cx = float(msg.camera_info.k[2])
        predicted_u = self.predicted_centre_u(msg, fx, cx, omega * dt)
        return {
            'stamp': stamp,
            'previous_stamp': previous,
            'dt': dt,
            'latency_ms': float(msg.latency_ms),
            'backend': msg.backend,
            'fx': fx,
            'median_magnitude': float(np.median(magnitude)),
            'bottom_v': float(np.median(bottom[..., 1])),
            'left_u': float(np.median(left[..., 0])),
            'right_u': float(np.median(right[..., 0])),
            'centre_u': float(np.median(centre[..., 0])),
            'centre_v': float(np.median(centre[..., 1])),
            'gyro_z': omega,
            'predicted_centre_u': predicted_u,
        }

    def camera_lever(self):
        """Return the camera's distance ahead of the turning axis (base_link to camera_link x)."""
        if self.lever is None:
            try:
                transform = self.tf_buffer.lookup_transform('base_link', 'camera_link', Time())
                self.lever = float(transform.transform.translation.x)
            except tf2_ros.TransformException:
                return None
        return self.lever

    def predicted_centre_u(self, msg, fx, cx, theta):
        """Return the median predicted u-flow of a turn over the central window, or NaN."""
        lever = self.camera_lever()
        if lever is None or math.isnan(theta):
            return float('nan')
        depth = image_to_array(msg.depth)
        u_offset = np.broadcast_to(np.arange(depth.shape[1], dtype=np.float64) - cx, depth.shape)
        centre_depth = central_window(depth, 0.5)
        centre_u = central_window(u_offset, 0.5)
        valid = np.isfinite(centre_depth) & (centre_depth > 0.0)
        if not valid.any():
            return float('nan')
        predicted = predicted_turn_flow_u(centre_u[valid], centre_depth[valid], fx, theta, lever)
        return float(np.median(predicted))

    def mean_gyro(self, start, end):
        """Return the mean gyro z between two stamps; NaN when no IMU sample lies between."""
        samples = [value for stamp, value in self.imu if start < stamp <= end]
        return float(np.mean(samples)) if samples else float('nan')

    # --- time and motion -----------------------------------------------------------------------

    def spin_for(self, seconds):
        """Process callbacks for the given wall-clock time."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now(self):
        """Return the simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for_clock(self, timeout):
        """Return True when /clock arrives within the wall-clock timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.clock_pairs:
                return True
        return False

    def wait_for_flow(self, timeout):
        """Return True when the first flow frame arrives within the wall-clock timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.first_flow is not None:
                return True
        return False

    def wait_for_flow_until(self, stamp, timeout):
        """Process callbacks until a flow frame at or after stamp has arrived."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.flow_records and self.flow_records[-1]['stamp'] >= stamp:
                return True
        return False

    def wait_sim_seconds(self, seconds):
        """Process callbacks until the given simulated time has passed."""
        start = self.sim_now()
        while self.sim_now() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)

    def publish_command(self, linear, angular):
        """Publish one velocity command."""
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.cmd_pub.publish(message)

    def run_for_sim(self, linear, angular, seconds):
        """Send the velocity at 20 Hz of simulated time for the given simulated seconds."""
        start = self.sim_now()
        last_sent = None
        while self.sim_now() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = self.sim_now()
            if last_sent is None or now - last_sent >= CMD_PERIOD_S:
                self.publish_command(linear, angular)
                last_sent = now
        self.publish_command(0.0, 0.0)

    def real_time_factor(self):
        """Return simulated seconds per wall-clock second over the clock messages seen so far."""
        if len(self.clock_pairs) < 2:
            return 0.0
        sim_span = self.clock_pairs[-1][0] - self.clock_pairs[0][0]
        wall_span = self.clock_pairs[-1][1] - self.clock_pairs[0][1]
        return sim_span / wall_span if wall_span > 0.0 else 0.0

    # --- geometry ------------------------------------------------------------------------------

    def geometry(self, report):
        """Compare the depth image's centre row, moved into lidar_link, with the lidar's ranges."""
        if self.depth is None or self.info is None or self.scan is None:
            report.add('geometry inputs', 'missing', 'depth, camera_info and scan', 'FAIL')
            return
        depth = image_to_array(self.depth)
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        row = int(round(cy))
        u, x, z = backproject_row(depth, row, fx, cx)
        keep = (u.astype(int) % GEOMETRY_COLUMN_STEP) == 0
        u, x, z = u[keep], x[keep], z[keep]
        y = (row - cy) * z / fy
        points = np.stack([x, y, z], axis=1)
        try:
            transform = self.tf_buffer.lookup_transform('lidar_link', 'camera_optical_frame',
                                                        Time())
        except tf2_ros.TransformException as error:
            report.add('tf lidar_link <- camera_optical_frame', 'missing: {}'.format(error),
                       'available', 'FAIL')
            return
        t = transform.transform.translation
        r = transform.transform.rotation
        in_lidar = transform_points(points, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))
        bearings = np.arctan2(in_lidar[:, 1], in_lidar[:, 0])
        ranges = np.hypot(in_lidar[:, 0], in_lidar[:, 1])
        scan_ranges = np.array([
            scan_range_at(b, self.scan.angle_min, self.scan.angle_increment, self.scan.ranges)
            for b in bearings])
        valid = np.isfinite(scan_ranges) & np.isfinite(ranges)
        differences = np.abs(ranges[valid] - scan_ranges[valid])
        count = int(valid.sum())
        median = float(np.median(differences)) if count else float('nan')
        within = float(np.mean(differences <= GEOMETRY_WITHIN_M)) if count else 0.0
        self.values['geometry'] = {'points': count, 'median_abs_error_m': median,
                                   'fraction_within_0.10m': within}
        report.add('geometry points used', str(count), '>= {}'.format(GEOMETRY_MIN_POINTS),
                   'INFO')
        report.add('geometry median |depth - lidar| (m)', format_number(median, 3),
                   '<= {:.2f}'.format(GEOMETRY_MEDIAN_M),
                   verdict_of(count >= GEOMETRY_MIN_POINTS and median <= GEOMETRY_MEDIAN_M))
        report.add('geometry fraction within 0.10 m', format_number(within, 3),
                   '>= {:.2f}'.format(GEOMETRY_WITHIN_FRACTION),
                   verdict_of(count >= GEOMETRY_MIN_POINTS
                              and within >= GEOMETRY_WITHIN_FRACTION))

    # --- the run -------------------------------------------------------------------------------

    def frames_between(self, previous_from, stamp_before):
        """Return the frame records with previous_stamp >= previous_from, stamp < stamp_before."""
        return [r for r in self.flow_records
                if r['previous_stamp'] >= previous_from and r['stamp'] < stamp_before]


def clean(value):
    """Return the value with NaN replaced by None, so that results.json is plain JSON."""
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean(item) for item in value]
    if isinstance(value, float) and math.isnan(value):
        return None
    return value


def check_flow_arrival(node, report):
    """Wait for the first flow frame and report the backend, the size and the frame id."""
    if not node.wait_for_flow(FLOW_WAIT_S):
        report.add('flow frames received', 'none', 'within {:.0f} s'.format(FLOW_WAIT_S), 'FAIL')
        return False
    first = node.first_flow
    report.add('flow backend', first.backend, 'INFO', 'INFO')
    report.add('flow size', '{}x{}'.format(first.flow.width, first.flow.height),
               '640x480 (default)', 'INFO')
    report.add('flow frame id', first.header.frame_id, 'camera_optical_frame',
               verdict_of(first.header.frame_id == 'camera_optical_frame'))
    return True


def check_rest(node, report, start, end):
    """Median flow magnitude over the still window must be small."""
    window = [r for r in node.flow_records if start <= r['stamp'] <= end]
    if len(window) < 1:
        report.add('at rest: flow frames', '0', '>= 1', 'FAIL')
        return
    median = float(np.median([r['median_magnitude'] for r in window]))
    node.values['rest_median_magnitude_px'] = median
    report.add('at rest: median |flow| (px)', format_number(median, 3),
               '< {:.2f}'.format(REST_MEDIAN_PX), verdict_of(median < REST_MEDIAN_PX))


def check_forward(node, report, start, stop):
    """While driving forward the floor flows down and the sides flow outwards."""
    frames = node.frames_between(start + SKIP_FIRST_S, stop)
    if len(frames) < MIN_FRAMES:
        verdict = 'WARN' if node.profile == 'docker' else 'FAIL'
        report.add('forward frames', str(len(frames)), '>= {}'.format(MIN_FRAMES), verdict)
        return
    bottom = float(np.median([r['bottom_v'] for r in frames]))
    left = float(np.median([r['left_u'] for r in frames]))
    right = float(np.median([r['right_u'] for r in frames]))
    node.values['forward'] = {'frames': len(frames), 'bottom_v_px': bottom, 'left_u_px': left,
                              'right_u_px': right}
    report.add('forward frames', str(len(frames)), '>= {}'.format(MIN_FRAMES), 'INFO')
    report.add('forward floor v (px)', format_number(bottom, 2),
               '> {:.1f}'.format(FORWARD_FLOOR_V_PX),
               verdict_of(bottom > FORWARD_FLOOR_V_PX))
    report.add('forward left third u (px)', format_number(left, 2), '< 0',
               verdict_of(left < 0.0))
    report.add('forward right third u (px)', format_number(right, 2), '> 0',
               verdict_of(right > 0.0))


def check_rotation(node, report, start, stop):
    """
    While turning in place, the central flow matches the gyro's prediction.

    The prediction is per pixel and includes the camera's lever arm (it sits ahead of the turning
    axis, so near points flow more); the rotation-only ratio is kept in results.json.
    """
    frames = node.frames_between(start + SKIP_FIRST_S, stop)
    usable = [r for r in frames if not math.isnan(r['gyro_z']) and r['dt'] > 0.0]
    if len(usable) < MIN_FRAMES:
        verdict = 'WARN' if node.profile == 'docker' else 'FAIL'
        report.add('rotation frames', str(len(usable)), '>= {}'.format(MIN_FRAMES), verdict)
        return
    ratios = []
    verticals = []
    rotation_only = []
    for record in usable:
        predicted = record['predicted_centre_u']
        if math.isnan(predicted) or abs(predicted) < 1e-6:
            continue
        ratios.append(record['centre_u'] / predicted)
        rotation_only.append(record['centre_u'] / predicted_rotation_flow(
            record['fx'], record['gyro_z'], record['dt']))
        verticals.append(abs(record['centre_v']) / abs(predicted))
    if len(ratios) < MIN_FRAMES:
        verdict = 'WARN' if node.profile == 'docker' else 'FAIL'
        report.add('rotation frames with a prediction', str(len(ratios)),
                   '>= {}'.format(MIN_FRAMES), verdict)
        return
    ratio = float(np.median(ratios))
    vertical = float(np.median(verticals))
    node.values['rotation'] = {'frames': len(ratios), 'ratio': ratio, 'vertical': vertical,
                               'ratio_rotation_only': float(np.median(rotation_only)),
                               'lever_m': node.lever}
    report.add('rotation frames', str(len(ratios)), '>= {}'.format(MIN_FRAMES), 'INFO')
    report.add('rotation ratio (flow / gyro prediction)', format_number(ratio, 3),
               '{:.2f} to {:.2f}'.format(*ROTATE_RATIO),
               verdict_of(ROTATE_RATIO[0] <= ratio <= ROTATE_RATIO[1]))
    report.add('rotation vertical share', format_number(vertical, 3),
               '< {:.2f}'.format(ROTATE_VERTICAL_RATIO),
               verdict_of(vertical < ROTATE_VERTICAL_RATIO))


def latency_rows(node, report, wall_start, wall_end):
    """Report the processing latency and the rate of the flow node (information only)."""
    records = node.flow_records
    if not records:
        report.add('flow frames processed', '0', '-', 'INFO')
        return
    latency = np.array([r['latency_ms'] for r in records])
    wall = max(wall_end - wall_start, 1e-6)
    report.add('flow frames processed', str(len(records)), '-', 'INFO')
    report.add('flow latency median (ms)', format_number(float(np.median(latency)), 1),
               '-', 'INFO')
    p95 = float(np.percentile(latency, 95))
    report.add('flow latency 95th percentile (ms)', format_number(p95, 1), '-', 'INFO')
    report.add('flow frames per wall second', format_number(len(records) / wall, 2), '-', 'INFO')
    report.add('real-time factor', format_number(node.real_time_factor(), 3), '-', 'INFO')


def save_pictures(node, report):
    """Save the flow and colour pictures of the forward and rotation windows."""
    for name in ('forward', 'rotate'):
        if name not in node.pictures:
            report.add('picture {}'.format(name), 'none', 'a frame in the window', 'INFO')
            continue
        _stamp, flow, colour = node.pictures[name]
        Image.fromarray(flow_to_rgb(flow)).save(
            os.path.join(node.out_dir, 'flow_{}.png'.format(name)))
        if colour is not None:
            Image.fromarray(colour).save(os.path.join(node.out_dir, 'rgb_{}.png'.format(name)))


def parse_arguments(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description='Check the optical flow pipeline.')
    parser.add_argument('--out', default=None,
                        help='bundle folder (default: perception_check_<time> here)')
    parser.add_argument('--profile', choices=['native', 'docker'], default='native',
                        help='native: strict; docker: missing drive frames are a WARN')
    parser.add_argument('--no-drive', action='store_true', help='skip the forward and turn checks')
    return parser.parse_args(argv)


def main(argv=None):
    """Run every check, write the bundle and return the exit status."""
    raw_argv = sys.argv[1:] if argv is None else list(argv)
    args = parse_arguments(raw_argv)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out_dir = args.out or os.path.join(os.getcwd(), 'perception_check_' + stamp)
    os.makedirs(out_dir, exist_ok=True)
    environment = environment_lines(raw_argv)
    report = Report()
    rtf = 0.0
    values = {}

    rclpy.init()
    node = None
    wall_clock_start = time.monotonic()
    try:
        node = PerceptionCheck(args.profile, out_dir)
        if not node.wait_for_clock(CLOCK_WAIT_S):
            report.add('/clock', 'none', 'messages within {:.0f} s'.format(CLOCK_WAIT_S), 'FAIL')
        elif check_flow_arrival(node, report):
            wall_start = wall_clock_start
            node.wait_sim_seconds(SETTLE_SIM_S)
            node.spin_for(0.5)
            node.geometry(report)
            rest_start = node.sim_now()
            node.wait_sim_seconds(REST_SIM_S)
            rest_end = node.sim_now()
            node.wait_for_flow_until(rest_end, FLOW_LAG_S)
            check_rest(node, report, rest_start, rest_end)
            if not args.no_drive:
                forward_start = node.sim_now()
                node.windows['forward'] = (forward_start + SKIP_FIRST_S,
                                           forward_start + DRIVE_TIME)
                node.run_for_sim(DRIVE_SPEED, 0.0, DRIVE_TIME)
                forward_stop = node.sim_now()
                node.run_for_sim(0.0, 0.0, STOP_TIME)
                node.wait_for_flow_until(forward_stop, FLOW_LAG_S)
                check_forward(node, report, forward_start, forward_stop)
                turn_start = node.sim_now()
                node.windows['rotate'] = (turn_start + SKIP_FIRST_S, turn_start + TURN_TIME)
                node.run_for_sim(0.0, TURN_RATE, TURN_TIME)
                turn_stop = node.sim_now()
                node.run_for_sim(0.0, 0.0, STOP_TIME)
                node.wait_for_flow_until(turn_stop, FLOW_LAG_S)
                check_rotation(node, report, turn_start, turn_stop)
                save_pictures(node, report)
            else:
                report.add('drive', 'skipped', '-', 'INFO')
            latency_rows(node, report, wall_start, time.monotonic())
            values = {'frames': clean(node.flow_records), 'summary': clean(node.values)}
            rtf = node.real_time_factor()
        else:
            report.add('flow', 'no frames', 'frames', 'FAIL')
    except KeyboardInterrupt:
        report.add('run', 'interrupted', 'complete', 'FAIL')
    finally:
        if node is not None:
            for _ in range(5):
                node.publish_command(0.0, 0.0)
                rclpy.spin_once(node, timeout_sec=0.01)
            node.destroy_node()
        rclpy.shutdown()

    write_outputs(out_dir, 'perception_check summary', environment, rtf, report.rows, {}, values)
    print(summary_text('perception_check summary', environment, rtf, report.rows))
    print('bundle: ' + out_dir)
    return 0 if report.count('FAIL') == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
