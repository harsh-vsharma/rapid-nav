"""
Run the goals of a scenario through the navigation stack and score them against ground truth.

Start the navigation stack first (ros2 launch rapid_nav_bringup sim.launch.py navigation:=true
controller:=<name>), then run:

    ros2 run rapid_nav_tools nav_run --scenario tour|unmapped [--controller dwa|dwb|rpp]
                                     [--out DIR] [--profile native|docker] [--goal-timeout S]

The scenario's goals (rapid_nav_navigation/config/nav_scenarios.yaml) go to /navigate_to_pose one
after another. Every goal is scored from the simulator's ground truth: success (within 0.25 m and
0.35 rad of the goal), the time in simulated seconds, the travelled length, the mean tracking error
to the last plan, the minimum body clearance and the collision episodes against the map's occupied
cells and the scenario's boxes, the recoveries, and how far the wheel odometry and AMCL drifted.
Nav2's own verdict is recorded beside the ground truth, and every disagreement is counted.

The bundle in DIR holds environment.txt, results.json, summary.md and trajectories_<scenario>.png.
The exit status is 0 when every check of dwa passes (the rows of dwb and rpp are information),
else 1.
"""

import argparse
import datetime
import json
import math
import os
import subprocess
import sys
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
import numpy as np
from PIL import Image, ImageDraw
from rapid_nav_tools.checks import (
    angle_diff,
    body_clearance,
    collision_episodes,
    occupied_cells_xy,
    tracking_error,
    tracking_error_in_force,
    travelled_length,
    yaw_from_quaternion,
)
from rapid_nav_tools.sim_check import environment_lines
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
import yaml

SCENARIOS = ('tour', 'unmapped')
CONTROLLERS = ('dwa', 'dwb', 'rpp')
ROBOT_RADIUS_M = 0.25
GOAL_XY_TOLERANCE_M = 0.25
GOAL_YAW_TOLERANCE_RAD = 0.35
MIN_DWA_CLEARANCE_M = 0.05
SAMPLE_PERIOD_S = 0.1
CLOCK_WAIT_S = 60.0
SETTLE_SIM_S = 2.0
SPAWN_SETTLE_SIM_S = 3.0
SERVER_WAIT_S = 180.0
SPAWN_TIMEOUT_S = 120.0
CELL_M = 0.05
MAP_NAME = 'arena'
PICTURE_PX_PER_M = 60
PICTURE_MARGIN_PX = 40
PICTURE_MAX_SPEED = 0.5


def stamp_now():
    """Return the bundle name stamp, e.g. 20261114-101500."""
    return datetime.datetime.now().strftime('%Y%m%d-%H%M%S')


def finite_or_none(value):
    """Return the value as a float, or None when it is not finite (for the JSON file)."""
    if value is None or not math.isfinite(value):
        return None
    return float(value)


def yaw_quaternion(yaw):
    """Return (z, w) of a yaw rotation: the goal orientation of a planar pose."""
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def load_scenario(name):
    """Return the scenario dictionary of nav_scenarios.yaml."""
    path = os.path.join(get_package_share_directory('rapid_nav_navigation'), 'config',
                        'nav_scenarios.yaml')
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)[name]


def load_map():
    """Return (occupied cell centres in metres, map yaml dictionary, occupancy grid)."""
    share = get_package_share_directory('rapid_nav_navigation')
    yaml_path = os.path.join(share, 'maps', MAP_NAME + '.yaml')
    with open(yaml_path, encoding='utf-8') as handle:
        meta = yaml.safe_load(handle)
    pixels = np.asarray(Image.open(os.path.join(share, 'maps', meta['image'])))
    # Trinary map, negate 0: a cell is occupied when (255 - pixel) / 255 exceeds 0.65.
    grid = np.where(pixels < 89, 100, 0).astype(np.int8)
    # The image's first row is the top of the map; the grid's first row is the bottom.
    grid = np.flipud(grid)
    origin = (float(meta['origin'][0]), float(meta['origin'][1]))
    return occupied_cells_xy(grid, float(meta['resolution']), origin), meta, grid


def box_sdf(name, size):
    """Return the SDF of a static grey box, centred on its origin."""
    return (
        '<sdf version="1.9"><model name="{name}"><static>true</static>'
        '<link name="link">'
        '<collision name="collision"><geometry><box><size>{s} {s} {s}</size></box></geometry>'
        '</collision>'
        '<visual name="visual"><geometry><box><size>{s} {s} {s}</size></box></geometry>'
        '<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>'
        '</visual></link></model></sdf>'
    ).format(name=name, s=size)


def drift(start_truth, end_truth, start_sensor, end_sensor):
    """
    Return how far a sensor's motion over the goal differs from the ground truth's motion.

    Wheel odometry starts from its own origin, so its drift is the difference between the motion
    the odometry reports and the motion of the truth since the goal began. Without a start (AMCL,
    which is in the map frame) it is the distance of the sensor from the truth at the end.
    """
    if end_sensor is None or end_truth is None:
        return None
    if start_sensor is None or start_truth is None:
        return finite_or_none(math.hypot(end_sensor[0] - end_truth[0],
                                         end_sensor[1] - end_truth[1]))
    sensor_motion = (end_sensor[0] - start_sensor[0], end_sensor[1] - start_sensor[1])
    truth_motion = (end_truth[0] - start_truth[0], end_truth[1] - start_truth[1])
    return finite_or_none(math.hypot(sensor_motion[0] - truth_motion[0],
                                     sensor_motion[1] - truth_motion[1]))


def score_goal(index, goal, samples, plan_history, timing, outcome, occupied, boxes):
    """Return the score of one goal from its ground-truth samples and Nav2's outcome."""
    truth = [s['truth'] for s in samples if s['truth'] is not None]
    truth_times = [s['t'] for s in samples if s['truth'] is not None]
    start_truth = truth[0] if truth else None
    end_truth = truth[-1] if truth else None
    truth_xy = np.asarray([t[:2] for t in truth], dtype=np.float64).reshape(-1, 2)
    plan_times = [when for when, _ in plan_history]
    plans = [plan for _, plan in plan_history]
    last_plan = plans[-1] if plans else np.zeros((0, 2))
    clearances = body_clearance(truth_xy, occupied, boxes, ROBOT_RADIUS_M, cell=CELL_M)
    if end_truth is None:
        distance_error = None
        yaw_error = None
        success_truth = False
    else:
        distance_error = math.hypot(goal['x'] - end_truth[0], goal['y'] - end_truth[1])
        yaw_error = abs(angle_diff(goal['yaw'], end_truth[2]))
        success_truth = (distance_error <= GOAL_XY_TOLERANCE_M
                         and yaw_error <= GOAL_YAW_TOLERANCE_RAD)
    success_nav2 = outcome['status'] == GoalStatus.STATUS_SUCCEEDED
    odom = [s['odom'] for s in samples if s['odom'] is not None]
    filtered = [s['filtered'] for s in samples if s['filtered'] is not None]
    amcl = [s['amcl'] for s in samples if s['amcl'] is not None]
    min_clearance = float(np.min(clearances)) if clearances.size else None
    return {
        'goal': index,
        'target': goal,
        'nav2_status': outcome['status_name'],
        'nav2_error_code': outcome['error_code'],
        'nav2_error_msg': outcome['error_msg'],
        'nav2_success': bool(success_nav2),
        'truth_success': bool(success_truth),
        'disagreement': bool(success_nav2 != success_truth),
        'timed_out': bool(outcome['timed_out']),
        'end_truth': list(end_truth) if end_truth is not None else None,
        'distance_error_m': finite_or_none(distance_error),
        'yaw_error_rad': finite_or_none(yaw_error),
        'time_s': finite_or_none(timing['end'] - timing['start']),
        'travelled_m': finite_or_none(travelled_length(truth_xy)),
        'tracking_error_m': finite_or_none(
            tracking_error_in_force(truth_xy, truth_times, plan_times, plans)),
        'tracking_error_last_plan_m': finite_or_none(tracking_error(truth_xy, last_plan)),
        'plans_published': len(plans),
        'plan_last': [[round(float(x), 3), round(float(y), 3)] for x, y in last_plan],
        'min_clearance_m': finite_or_none(min_clearance),
        'collision_episodes': collision_episodes(clearances, 0.0),
        'recoveries': int(outcome['recoveries']),
        'odom_drift_m': drift(start_truth, end_truth,
                              odom[0] if odom else None, odom[-1] if odom else None),
        'filtered_drift_m': drift(start_truth, end_truth,
                                  filtered[0] if filtered else None,
                                  filtered[-1] if filtered else None),
        'amcl_error_m': drift(None, end_truth, None, amcl[-1] if amcl else None),
        'samples': len(samples),
    }


class NavRun(Node):
    """Subscribe to the navigation topics, keep the latest messages and send the goals."""

    def __init__(self):
        """Create the subscriptions and the action client."""
        super().__init__(
            'nav_run',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.latest = {'truth': None, 'odom': None, 'filtered': None, 'amcl': None, 'cmd': None}
        self.plan = None
        self.plan_history = []
        self.clock_seen = False
        self.feedback = {'recoveries': 0, 'distance_remaining': None}
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Clock, '/clock', self._on_clock, sensor_qos)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_odometry('truth'),
                                 sensor_qos)
        self.create_subscription(Odometry, '/odom', self._on_odometry('odom'), sensor_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odometry('filtered'),
                                 sensor_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl,
                                 sensor_qos)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _on_clock(self, _message):
        """Note that the simulated clock is running."""
        self.clock_seen = True

    def _on_odometry(self, key):
        """Return the callback that keeps the (x, y, yaw) of an odometry topic."""
        def keep(message):
            pose = message.pose.pose
            orientation = pose.orientation
            self.latest[key] = (pose.position.x, pose.position.y,
                                yaw_from_quaternion(orientation.x, orientation.y,
                                                    orientation.z, orientation.w))
        return keep

    def _on_amcl(self, message):
        """Keep the AMCL pose as (x, y, yaw)."""
        pose = message.pose.pose
        orientation = pose.orientation
        self.latest['amcl'] = (pose.position.x, pose.position.y,
                               yaw_from_quaternion(orientation.x, orientation.y,
                                                   orientation.z, orientation.w))

    def _on_cmd(self, message):
        """Keep the latest command as (linear, angular)."""
        self.latest['cmd'] = (message.linear.x, message.angular.z)

    def _on_plan(self, message):
        """Keep the latest global plan as an (N, 2) array, and the time it was published."""
        self.plan = np.asarray([(p.pose.position.x, p.pose.position.y) for p in message.poses],
                               dtype=np.float64).reshape(-1, 2)
        self.plan_history.append((self.sim_now(), self.plan))

    def sim_now(self):
        """Return the simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for_sim(self, seconds):
        """Spin until the simulated clock has advanced by the given seconds."""
        start = self.sim_now()
        while self.sim_now() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_until(self, predicate, timeout):
        """Spin until the predicate holds or the wall-clock timeout passes; return the outcome."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def sample(self):
        """Return the latest values of every topic, with the simulated time."""
        return {'t': self.sim_now(), 'truth': self.latest['truth'], 'odom': self.latest['odom'],
                'filtered': self.latest['filtered'], 'amcl': self.latest['amcl'],
                'cmd': self.latest['cmd']}


def make_goal(goal):
    """Return the NavigateToPose goal of a scenario goal (map frame)."""
    message = NavigateToPose.Goal()
    message.pose = PoseStamped()
    message.pose.header.frame_id = 'map'
    message.pose.pose.position.x = float(goal['x'])
    message.pose.pose.position.y = float(goal['y'])
    z, w = yaw_quaternion(float(goal['yaw']))
    message.pose.pose.orientation.z = z
    message.pose.pose.orientation.w = w
    return message


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'unknown',
    GoalStatus.STATUS_ACCEPTED: 'accepted',
    GoalStatus.STATUS_EXECUTING: 'executing',
    GoalStatus.STATUS_CANCELING: 'canceling',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def run_goal(node, goal, goal_timeout):
    """Send one goal, sample while it runs, cancel on timeout; return samples, timing, outcome."""
    outcome = {'status': GoalStatus.STATUS_UNKNOWN, 'status_name': 'unknown', 'error_code': None,
               'error_msg': '', 'recoveries': 0, 'timed_out': False}
    node.plan = None
    node.plan_history = []
    node.feedback = {'recoveries': 0, 'distance_remaining': None}

    def on_feedback(message):
        feedback = message.feedback
        node.feedback['recoveries'] = int(getattr(feedback, 'number_of_recoveries', 0))
        node.feedback['distance_remaining'] = float(getattr(feedback, 'distance_remaining', 0.0))

    samples = []
    timing = {'start': node.sim_now(), 'end': None}
    send = node.client.send_goal_async(make_goal(goal), feedback_callback=on_feedback)
    if not node.wait_until(send.done, 30.0):
        outcome['status_name'] = 'send timed out'
        timing['end'] = node.sim_now()
        return samples, timing, outcome
    handle = send.result()
    if not handle.accepted:
        outcome['status_name'] = 'rejected'
        timing['end'] = node.sim_now()
        return samples, timing, outcome

    result_future = handle.get_result_async()
    wall_deadline = time.monotonic() + 4.0 * goal_timeout + 60.0
    last_sample = None
    while not result_future.done():
        rclpy.spin_once(node, timeout_sec=0.02)
        now = node.sim_now()
        if last_sample is None or now - last_sample >= SAMPLE_PERIOD_S:
            samples.append(node.sample())
            last_sample = now
        if now - timing['start'] > goal_timeout or time.monotonic() > wall_deadline:
            outcome['timed_out'] = True
            cancel = handle.cancel_goal_async()
            node.wait_until(cancel.done, 30.0)
            node.wait_until(result_future.done, 30.0)
            break
    samples.append(node.sample())
    timing['end'] = node.sim_now()
    if result_future.done():
        wrapped = result_future.result()
        outcome['status'] = wrapped.status
        outcome['status_name'] = STATUS_NAMES.get(wrapped.status, str(wrapped.status))
        outcome['error_code'] = int(wrapped.result.error_code)
        outcome['error_msg'] = wrapped.result.error_msg
    outcome['recoveries'] = node.feedback['recoveries']
    return samples, timing, outcome


def spawn_boxes(boxes, size, log_path):
    """Spawn the scenario's boxes into the arena; return their (x, y, half side) for scoring."""
    extra = []
    with open(log_path, 'w', encoding='utf-8') as log:
        for box in boxes:
            command = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'arena',
                       '-name', box['name'], '-x', str(box['x']), '-y', str(box['y']),
                       '-z', str(size / 2.0), '-string', box_sdf(box['name'], size)]
            result = subprocess.run(command, capture_output=True, text=True,
                                    timeout=SPAWN_TIMEOUT_S, check=False)
            log.write('{}: exit {}\n{}{}\n'.format(
                box['name'], result.returncode, result.stdout, result.stderr))
            extra.append((float(box['x']), float(box['y']), size / 2.0))
    return extra


def write_environment(out_dir, argv, scenario, controller, profile, goal_timeout):
    """Write environment.txt: the host, the commands and the scenario."""
    lines = environment_lines(argv)
    lines += [
        'scenario: ' + scenario,
        'controller: {} (recorded only: the launch chose it)'.format(controller),
        'profile: ' + profile,
        'goal timeout (sim s): {}'.format(goal_timeout),
        'robot radius (m): {}'.format(ROBOT_RADIUS_M),
    ]
    with open(os.path.join(out_dir, 'environment.txt'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')


def fmt(value, digits=3):
    """Format a number for a table; a dash when it is missing."""
    if value is None:
        return '-'
    return '{:.{}f}'.format(value, digits)


def build_checks(controller, goals):
    """Return the check rows: dwa's verdicts, and the same rows as information for the others."""
    n = len(goals)
    reached = sum(1 for g in goals if g['truth_success'])
    episodes = sum(g['collision_episodes'] for g in goals)
    clearances = [g['min_clearance_m'] for g in goals if g['min_clearance_m'] is not None]
    minimum = min(clearances) if clearances else None
    disagreements = sum(1 for g in goals if g['disagreement'])

    def verdict(ok):
        """PASS or FAIL for dwa; INFO for the reference controllers."""
        if controller != 'dwa':
            return 'INFO'
        return 'PASS' if ok else 'FAIL'

    return [
        {'check': 'goals reached (ground truth)', 'value': '{}/{}'.format(reached, n),
         'expected': 'all', 'verdict': verdict(reached == n)},
        {'check': 'collision episodes (ground truth)', 'value': str(episodes),
         'expected': '0', 'verdict': verdict(episodes == 0)},
        {'check': 'minimum body clearance (m)', 'value': fmt(minimum),
         'expected': '>= {}'.format(MIN_DWA_CLEARANCE_M),
         'verdict': verdict(minimum is not None and minimum >= MIN_DWA_CLEARANCE_M)},
        {'check': "Nav2's success vs ground truth", 'value': '{} disagreement(s)'.format(
            disagreements), 'expected': 'none', 'verdict': 'INFO'},
    ]


def summary_text(scenario, controller, checks, goals):
    """Return summary.md: the check table and the per-goal table."""
    lines = ['# nav_run: {} with {}'.format(scenario, controller), '',
             '| check | value | expected | verdict |', '|---|---|---|---|']
    for row in checks:
        lines.append('| {} | {} | {} | {} |'.format(
            row['check'], row['value'], row['expected'], row['verdict']))
    lines += ['', '## Goals', '',
              '| goal | target (x, y, yaw) | Nav2 | truth | error (m, rad) | time (s) | '
              'travelled (m) | tracking, plan in force (m) | min clearance (m) | collisions | '
              'recoveries | '
              'odom drift (m) | AMCL error (m) |',
              '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for g in goals:
        target = g['target']
        lines.append('| {} | ({}, {}, {}) | {} | {} | {} / {} | {} | {} | {} | {} | {} | {} | '
                     '{} | {} |'.format(
                         g['goal'] + 1, fmt(target['x'], 2), fmt(target['y'], 2),
                         fmt(target['yaw'], 2), g['nav2_status'],
                         'success' if g['truth_success'] else 'fail',
                         fmt(g['distance_error_m']), fmt(g['yaw_error_rad']),
                         fmt(g['time_s'], 1), fmt(g['travelled_m'], 2),
                         fmt(g['tracking_error_m'], 3), fmt(g['min_clearance_m'], 3),
                         g['collision_episodes'], g['recoveries'], fmt(g['odom_drift_m']),
                         fmt(g['amcl_error_m'])))
    return '\n'.join(lines) + '\n'


def draw_picture(path, scenario, goals, samples_per_goal, plans_per_goal, occupied, boxes,
                 grid_shape):
    """Draw the map, the boxes, the ground-truth track by speed, and the numbered goals."""
    height, width = grid_shape
    xmin, ymin = -5.5, -4.5
    xmax, ymax = xmin + width * CELL_M, ymin + height * CELL_M
    scale = PICTURE_PX_PER_M
    canvas_w = int(round((xmax - xmin) * scale)) + 2 * PICTURE_MARGIN_PX
    canvas_h = int(round((ymax - ymin) * scale)) + 2 * PICTURE_MARGIN_PX
    image = Image.new('RGB', (canvas_w, canvas_h), (250, 250, 250))
    draw = ImageDraw.Draw(image)

    def to_px(x, y):
        return (PICTURE_MARGIN_PX + (x - xmin) * scale, PICTURE_MARGIN_PX + (ymax - y) * scale)

    half = CELL_M / 2.0
    for cx, cy in occupied:
        x0, y0 = to_px(cx - half, cy + half)
        x1, y1 = to_px(cx + half, cy - half)
        draw.rectangle([x0, y0, x1, y1], fill=(60, 60, 60))
    for bx, by, bhalf in boxes:
        x0, y0 = to_px(bx - bhalf, by + bhalf)
        x1, y1 = to_px(bx + bhalf, by - bhalf)
        draw.rectangle([x0, y0, x1, y1], fill=(140, 140, 140), outline=(90, 90, 90))
    for plans in plans_per_goal:
        for plan in plans:
            if plan.shape[0] >= 2:
                draw.line([to_px(x, y) for x, y in plan], fill=(150, 150, 255), width=1)
    for samples in samples_per_goal:
        previous = None
        for sample in samples:
            truth = sample['truth']
            if truth is None:
                previous = None
                continue
            point = to_px(truth[0], truth[1])
            if previous is not None:
                dt = max(sample['t'] - previous[2], 1e-6)
                speed = math.hypot(truth[0] - previous[0], truth[1] - previous[1]) / dt
                t = min(speed / PICTURE_MAX_SPEED, 1.0)
                colour = (int(255 * t), 60, int(255 * (1.0 - t)))
                draw.line([previous[3], point], fill=colour, width=3)
            previous = (truth[0], truth[1], sample['t'], point)
    for index, goal in enumerate(goals):
        gx, gy = to_px(goal['x'], goal['y'])
        radius = 9
        draw.ellipse([gx - radius, gy - radius, gx + radius, gy + radius], fill=(255, 255, 255),
                     outline=(0, 0, 0))
        draw.text((gx - 4, gy - 6), str(index + 1), fill=(0, 0, 0))
    draw.text((PICTURE_MARGIN_PX, 8), 'nav_run: {} (track: blue slow, red 0.5 m/s)'.format(
        scenario), fill=(0, 0, 0))
    image.save(path)


def parse_arguments(argv):
    """Return the command-line arguments."""
    parser = argparse.ArgumentParser(description='Run a scenario and score it on ground truth.')
    parser.add_argument('--scenario', required=True, choices=SCENARIOS)
    parser.add_argument('--controller', default='dwa', choices=CONTROLLERS,
                        help='recorded in the bundle; the launch chose the controller')
    parser.add_argument('--out', default=None,
                        help='bundle folder (default nav_run_<scenario>_<controller>_<time>)')
    parser.add_argument('--profile', default='native', choices=('native', 'docker'))
    parser.add_argument('--goal-timeout', type=float, default=120.0,
                        help='simulated seconds allowed per goal')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the scenario, write the bundle and print the summary; return the exit status."""
    argv_list = list(sys.argv if argv is None else argv)
    args = parse_arguments(argv_list[1:])
    out_dir = args.out or 'nav_run_{}_{}_{}'.format(args.scenario, args.controller, stamp_now())
    os.makedirs(out_dir, exist_ok=True)
    write_environment(out_dir, argv_list, args.scenario, args.controller, args.profile,
                      args.goal_timeout)

    scenario = load_scenario(args.scenario)
    occupied, _, grid = load_map()
    box_size = float(scenario.get('box_size', 0.0))
    boxes = []
    rclpy.init()
    node = NavRun()
    try:
        if not node.wait_until(lambda: node.clock_seen, CLOCK_WAIT_S):
            raise RuntimeError('no /clock within {} s'.format(CLOCK_WAIT_S))
        if not node.wait_until(lambda: node.latest['truth'] is not None, CLOCK_WAIT_S):
            raise RuntimeError('no ground truth within {} s'.format(CLOCK_WAIT_S))
        node.spin_for_sim(SETTLE_SIM_S)
        if not node.client.wait_for_server(timeout_sec=SERVER_WAIT_S):
            raise RuntimeError('/navigate_to_pose is not available within {} s'.format(
                SERVER_WAIT_S))
        if scenario.get('boxes'):
            boxes = spawn_boxes(scenario['boxes'], box_size,
                                os.path.join(out_dir, 'spawn.txt'))
            node.spin_for_sim(SPAWN_SETTLE_SIM_S)

        goals, samples_per_goal, plans = [], [], []
        for index, goal in enumerate(scenario['goals']):
            samples, timing, outcome = run_goal(node, goal, args.goal_timeout)
            scored = score_goal(index, goal, samples, node.plan_history, timing, outcome,
                                occupied, boxes)
            plans.append([plan for _, plan in node.plan_history])
            goals.append(scored)
            samples_per_goal.append(samples)
            print('goal {}: {} truth={} time={} s'.format(
                index + 1, outcome['status_name'],
                'success' if scored['truth_success'] else 'fail', fmt(scored['time_s'], 1)),
                flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    checks = build_checks(args.controller, goals)
    results = {
        'scenario': args.scenario,
        'controller': args.controller,
        'profile': args.profile,
        'goal_timeout_s': args.goal_timeout,
        'checks': checks,
        'boxes': boxes,
        'goals': goals,
        'samples': samples_per_goal,
    }
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as handle:
        json.dump(results, handle, indent=1)
    summary = summary_text(args.scenario, args.controller, checks, goals)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(summary)
    draw_picture(os.path.join(out_dir, 'trajectories_{}.png'.format(args.scenario)),
                 args.scenario, scenario['goals'], samples_per_goal, plans, occupied, boxes,
                 grid.shape)
    print(summary)
    print('bundle: ' + out_dir)
    failed = [row for row in checks if row['verdict'] == 'FAIL']
    return 1 if (args.controller == 'dwa' and failed) else 0


if __name__ == '__main__':
    sys.exit(main())
