"""
Score the camera-based obstacle tracks against the truth while a scenario runs.

Start the simulation with hazards:=flow and a scenario, then this recorder (scripts/dev.sh
track-smoke does both and drives the robot with nav_run). The recorder only listens: the tracks
on /obstacles/tracked, the obstacles' true poses (/model/<name>/odometry) and the robot's true pose
(/ground_truth/odom). The truth is used for scoring only. It stops when the stop file appears (or
after --duration simulated seconds) and writes the bundle: summary.md, results.json, a picture.
"""

import argparse
import json
import math
import os
import sys
import time

from nav_msgs.msg import Odometry
import numpy as np
from PIL import Image, ImageDraw
from rapid_nav_msgs.msg import TrackedObstacleArray
from rapid_nav_tools.nav_run import stamp_now
from rapid_nav_tools.obstacle_driver import load_scenarios
from rapid_nav_tools.scenarios import obstacle_topics
from rapid_nav_tools.sim_check import environment_lines
from rapid_nav_tools.track_metrics import score
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

PICTURE_PX = 600
PICTURE_M = (-5.5, -4.5, 5.5, 4.5)  # the arena: x0, y0, x1, y1


def stamp_seconds(stamp):
    """Return a builtin_interfaces/Time as seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    """Return the yaw of a quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class Recorder(Node):
    """Record the tracks, the obstacles' truth and the robot's truth."""

    def __init__(self, obstacles):
        """Subscribe to everything that is scored."""
        super().__init__('track_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.messages = []
        self.truth = {o['name']: ([], []) for o in obstacles}
        self.robot = ([], [], [])
        self.sources = set()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)
        self.clock_first = None
        self.clock_last = None
        self.create_subscription(TrackedObstacleArray, '/obstacles/tracked', self.on_tracks, 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_robot, 50)
        for o in obstacles:
            self.create_subscription(
                Odometry, obstacle_topics(o['name'])[1],
                lambda m, n=o['name']: self.on_obstacle(n, m), 50)

    def now_s(self):
        """Return the simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tracks(self, msg):
        """Record one message of tracks with its arrival time."""
        now = self.now_s()
        if self.clock_first is None:
            self.clock_first = (now, time.monotonic())
        self.clock_last = (now, time.monotonic())
        self.sources.add(msg.source)
        self.messages.append({
            't': stamp_seconds(msg.header.stamp), 'recv': now,
            'tracks': [{'id': o.id, 'x': o.position.x, 'y': o.position.y, 'vx': o.velocity.x,
                        'vy': o.velocity.y, 'radius': o.radius, 'confidence': o.confidence}
                       for o in msg.obstacles]})

    def on_obstacle(self, name, msg):
        """Record an obstacle's true position."""
        times, xy = self.truth[name]
        times.append(stamp_seconds(msg.header.stamp) if msg.header.stamp.sec else self.now_s())
        xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_robot(self, msg):
        """Record the robot's true pose."""
        times, xy, yaw = self.robot
        times.append(stamp_seconds(msg.header.stamp) if msg.header.stamp.sec else self.now_s())
        xy.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
        yaw.append(yaw_of(msg.pose.pose.orientation))

    def camera_mount(self):
        """Return (ox, oy, forward_yaw): the camera in the robot frame, from the static TF."""
        try:
            t = self.buffer.lookup_transform('base_footprint', 'camera_optical_frame', Time(),
                                             timeout=rclpy.duration.Duration(seconds=5.0))
        except TransformException:
            return (0.0, 0.0, 0.0)
        q, d = t.transform.rotation, t.transform.translation
        # the optical axis (z) in the base frame: third column of the rotation matrix
        ax = 2.0 * (q.x * q.z + q.y * q.w)
        ay = 2.0 * (q.y * q.z - q.x * q.w)
        return (d.x, d.y, math.atan2(ay, ax))


def draw_picture(path, recorder, scenario):
    """Draw the true obstacle paths (blue), the robot (grey) and the confirmed tracks (red)."""
    x0, y0, x1, y1 = PICTURE_M
    scale = PICTURE_PX / (x1 - x0)
    image = Image.new('RGB', (PICTURE_PX, int((y1 - y0) * scale)), (255, 255, 255))
    draw = ImageDraw.Draw(image)

    def px(x, y):
        return ((x - x0) * scale, (y1 - y) * scale)

    for name, (times, xy) in recorder.truth.items():
        if len(xy) > 1:
            draw.line([px(*p) for p in xy], fill=(40, 80, 220), width=2)
    if len(recorder.robot[1]) > 1:
        draw.line([px(*p) for p in recorder.robot[1]], fill=(150, 150, 150), width=2)
    for m in recorder.messages:
        for k in m['tracks']:
            cx, cy = px(k['x'], k['y'])
            draw.ellipse([cx - 2, cy - 2, cx + 2, cy + 2], fill=(220, 40, 40))
    draw.text((8, 8), 'track_check: {} (blue: true paths, red: flow tracks, grey: robot)'.format(
        scenario), fill=(0, 0, 0))
    image.save(path)


def fmt(value, digits=3):
    """Format a number for a table; a dash when it is missing."""
    return '-' if value is None else '{:.{}f}'.format(value, digits)


def summary_text(scenario, result, rtf, sources):
    """Return summary.md."""
    rows = [
        ('detection rate (in-view samples matched)', '{} ({} of {} in view)'.format(
            fmt(result['detection_rate']), result['matched_samples'], result['in_view_samples'])),
        ('in-view samples / matched', '{} / {}'.format(result['in_view_samples'],
                                                       result['matched_samples'])),
        ('position error, median (m)', fmt(result['position_error_median_m'])),
        ('position error, 95th percentile (m)', fmt(result['position_error_p95_m'])),
        ('speed error, median (m/s)', fmt(result['speed_error_median_mps'])),
        ('heading error, median (deg)', fmt(result['heading_error_median_deg'], 1)),
        ('confirmed tracks / false tracks', '{} / {}'.format(result['tracks_confirmed'],
                                                             result['false_tracks'])),
        ('false tracks per simulated second', fmt(result['false_tracks_per_s'])),
        ('false tracks per message, mean', fmt(result['false_per_message_mean'])),
        ('share of false tracks while turning > 0.3 rad/s', fmt(result['false_turning_fraction'])),
        ('latency stamp to publish, simulated (s)', fmt(result['latency_sim_median_s'])),
        ('latency stamp to publish, wall (s)',
         fmt(result['latency_sim_median_s'] / rtf) if rtf and result['latency_sim_median_s']
         is not None else '-'),
        ('messages / simulated seconds', '{} / {:.1f}'.format(result['messages'],
                                                              result['duration_s'])),
        ('real-time factor (measured)', fmt(rtf, 2)),
        ('source field of the messages', ', '.join(sorted(sources)) or '-'),
    ]
    lines = ['# track_check: {}'.format(scenario), '', '| metric | value |', '|---|---|']
    lines += ['| {} | {} |'.format(a, b) for a, b in rows]
    return '\n'.join(lines) + '\n'


def parse_arguments(argv):
    """Return the command-line arguments."""
    parser = argparse.ArgumentParser(description='Score camera tracks against the truth.')
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--out', default=None,
                        help='bundle folder (default track_check_<scenario>_<time>)')
    parser.add_argument('--stop-file', default=None, help='stop when this file exists')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='stop after this many simulated seconds (0: only the stop file)')
    return parser.parse_args(argv)


def main(argv=None):
    """Record until the stop file or the duration, then write the bundle."""
    argv_list = list(sys.argv if argv is None else argv)
    args = parse_arguments(argv_list[1:])
    out_dir = args.out or 'track_check_{}_{}'.format(args.scenario, stamp_now())
    os.makedirs(out_dir, exist_ok=True)
    scenarios = load_scenarios()
    if args.scenario not in scenarios:
        print('unknown scenario {!r}'.format(args.scenario), file=sys.stderr)
        return 1
    obstacles = scenarios[args.scenario].get('obstacles') or []
    rclpy.init()
    node = Recorder(obstacles)
    start = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = node.now_s()
            if start is None and node.messages:
                start = now
            if args.stop_file and os.path.exists(args.stop_file):
                break
            if args.duration and start is not None and now - start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    camera = node.camera_mount()
    duration = (node.now_s() - start) if start is not None else 0.0
    rtf = None
    if node.clock_first and node.clock_last and node.clock_last[1] - node.clock_first[1] > 5.0:
        rtf = ((node.clock_last[0] - node.clock_first[0])
               / (node.clock_last[1] - node.clock_first[1]))
    truth = {n: (np.array(t), np.array(xy)) for n, (t, xy) in node.truth.items()}
    robot = tuple(np.array(a) for a in node.robot)
    result = score(node.messages, truth, robot, camera, duration)
    with open(os.path.join(out_dir, 'environment.txt'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(environment_lines(argv_list) + [
            'scenario: ' + args.scenario,
            'camera mount (x, y, axis yaw): {:.3f} {:.3f} {:.3f}'.format(*camera),
            'real-time factor (measured): ' + fmt(rtf, 2)]) + '\n')
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as handle:
        json.dump({'scenario': args.scenario, 'rtf': rtf, 'metrics': result,
                   'sources': sorted(node.sources), 'camera': camera,
                   'first_nonempty_message': next(
                       (m for m in node.messages if m['tracks']), None)},
                  handle, indent=1)
    summary = summary_text(args.scenario, result, rtf, node.sources)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(summary)
    draw_picture(os.path.join(out_dir, 'tracks_{}.png'.format(args.scenario)), node, args.scenario)
    print(summary)
    print('bundle: ' + out_dir)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
