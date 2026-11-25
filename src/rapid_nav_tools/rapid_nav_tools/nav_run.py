"""
Run the goals of a scenario through the navigation stack and score them against ground truth.

Start the navigation stack first (ros2 launch rapid_nav_bringup sim.launch.py navigation:=true
controller:=<name>), then run:

    ros2 run rapid_nav_tools nav_run --scenario tour|unmapped|crossing|head_on|crowd
                                     [--controller dwa|dwb|rpp]
                                     [--out DIR] [--profile native|docker] [--goal-timeout S]

The scenario's goals (rapid_nav_navigation/config/nav_scenarios.yaml) go to /navigate_to_pose one
after another. Every goal is scored from the simulator's ground truth: success (within 0.25 m and
0.35 rad of the goal), the time in simulated seconds, the travelled length, the mean tracking error
to the last plan, the minimum body clearance and the collision episodes against the map's occupied
cells and the scenario's boxes, the recoveries, and how far the wheel odometry and AMCL drifted.
Nav2's own verdict is recorded beside the ground truth, and every disagreement is counted.

A scenario with moving obstacles (crossing, head_on, crowd) needs sim.launch.py with
scenario:=<name>. nav_run then also records the obstacles' true tracks, scores the clearance to
them at the same simulated time, and judges whether the scenario played out as scripted (the
validity rows). The navigation rows of a dynamic scenario are information for every controller.

The bundle in DIR holds environment.txt, results.json, summary.md and trajectories_<scenario>.png.
The exit status is 0 when every check of dwa passes (the rows of dwb and rpp are information)
and every validity row passes, else 1; 2 when a dynamic scenario has no obstacle driver.
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
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
import numpy as np
from PIL import Image, ImageDraw
from rapid_nav_tools.checks import (
    angle_diff,
    body_clearance,
    collision_episodes,
    dynamic_clearance,
    interpolate_track,
    occupied_cells_xy,
    tracking_error,
    tracking_error_in_force,
    travelled_length,
    yaw_from_quaternion,
)
from rapid_nav_tools.scenarios import (
    obstacle_topics,
    SHAPES,
    track_spec,
    validate_scenario,
    validity_rows,
)
from rapid_nav_tools.sim_check import environment_lines
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Int32
import yaml

SCENARIOS = ('tour', 'unmapped', 'crossing', 'head_on', 'crowd')
READY_WAIT_S = 120.0
CLOSE_ENCOUNTER_M = 0.30
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
CONTROLLERS = ('dwa', 'dwb', 'rpp')
HAZARDS = ('off', 'oracle', 'oracle_static', 'flow', 'fused')
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


def obstacle_tracks(obstacles, samples):
    """Return the track dicts (for dynamic_clearance) of the recorded obstacle samples."""
    tracks = []
    for obstacle in obstacles:
        rows = samples.get(obstacle['name']) or []
        times = [r[0] for r in rows]
        xy = [[r[1], r[2]] for r in rows]
        track = track_spec(obstacle, times, xy)
        track['shape'] = obstacle['shape']
        if not rows:
            first = obstacle['path'][0]
            track.update(times=[0.0], xy=[[float(first['x']), float(first['y'])]])
        tracks.append(track)
    return tracks


def downsample(track, period=0.1):
    """Return the track at about 10 Hz of simulated time, for results.json."""
    times, xy = [], []
    last = None
    for t, point in zip(track['times'], track['xy']):
        if last is None or t - last >= period - 1e-9:
            times.append(round(t, 3))
            xy.append([round(point[0], 3), round(point[1], 3)])
            last = t
    return {'times': times, 'xy': xy}


def closest_approach(truth_xy, truth_times, tracks):
    """Return the closest centre-to-centre approach to any obstacle: distance, who, when, where."""
    best = {'distance': None, 'obstacle': None, 't': None, 'robot': None, 'obstacle_xy': None,
            'by_obstacle': {}}
    for track in tracks:
        if truth_xy.shape[0] == 0:
            continue
        at = interpolate_track(track['times'], track['xy'], truth_times)
        distance = np.hypot(truth_xy[:, 0] - at[:, 0], truth_xy[:, 1] - at[:, 1])
        k = int(np.argmin(distance))
        best['by_obstacle'][track['name']] = float(distance[k])
        if best['distance'] is None or distance[k] < best['distance']:
            best.update(distance=float(distance[k]), obstacle=track['name'],
                        t=float(truth_times[k]),
                        robot=[float(truth_xy[k, 0]), float(truth_xy[k, 1])],
                        obstacle_xy=[float(at[k, 0]), float(at[k, 1])])
    return best


def score_goal(index, goal, samples, plan_history, timing, outcome, occupied, boxes,
               tracks=()):
    """Return the score of one goal from its ground-truth samples and Nav2's outcome."""
    truth = [s['truth'] for s in samples if s['truth'] is not None]
    truth_times = [s['t'] for s in samples if s['truth'] is not None]
    start_truth = truth[0] if truth else None
    end_truth = truth[-1] if truth else None
    truth_xy = np.asarray([t[:2] for t in truth], dtype=np.float64).reshape(-1, 2)
    plan_times = [when for when, _ in plan_history]
    plans = [plan for _, plan in plan_history]
    last_plan = plans[-1] if plans else np.zeros((0, 2))
    static = body_clearance(truth_xy, occupied, boxes, ROBOT_RADIUS_M, cell=CELL_M)
    dynamic, _ = dynamic_clearance(truth_xy, truth_times, tracks, ROBOT_RADIUS_M)
    clearances = np.minimum(static, dynamic)
    approach = closest_approach(truth_xy, truth_times, tracks)
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
        'min_clearance_static_m': finite_or_none(float(np.min(static)) if static.size else None),
        'min_clearance_dynamic_m': finite_or_none(
            float(np.min(dynamic)) if dynamic.size else None),
        'closest_approach_m': finite_or_none(approach['distance']),
        'closest_approach': approach,
        'close_encounters': collision_episodes(dynamic, CLOSE_ENCOUNTER_M),
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
        self.clock_first = None
        self.clock_last = None
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
        self.navigator_state = self.create_client(GetState, '/bt_navigator/get_state')
        self.goal_pub = self.create_publisher(Int32, '/scenario/goal', LATCHED)
        self.ready = False
        self.create_subscription(Bool, '/scenario/ready', self._on_ready, LATCHED)
        self.obstacle_samples = {}

    def _on_ready(self, message):
        """Note that the obstacle driver has spawned every obstacle."""
        self.ready = self.ready or bool(message.data)

    def record_obstacles(self, names):
        """Record the true (time, x, y) of each named obstacle for the whole run."""
        for name in names:
            self.obstacle_samples[name] = []
            self.create_subscription(
                Odometry, obstacle_topics(name)[1],
                lambda message, key=name: self.obstacle_samples[key].append((
                    message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                    message.pose.pose.position.x, message.pose.pose.position.y)), 50)

    def announce_goal(self, number):
        """Tell the obstacle driver which goal (1-based) is about to be sent."""
        self.goal_pub.publish(Int32(data=number))
        self.spin_for_sim(0.1)

    def _on_clock(self, message):
        """Note that the simulated clock is running; keep its first and last (sim, wall) pair."""
        self.clock_seen = True
        pair = (message.clock.sec + message.clock.nanosec * 1e-9, time.monotonic())
        if self.clock_first is None:
            self.clock_first = pair
        self.clock_last = pair

    def measured_rtf(self):
        """Return the real-time factor of the run (sim seconds per wall second), or None."""
        if self.clock_first is None or self.clock_last is None:
            return None
        wall = self.clock_last[1] - self.clock_first[1]
        if wall < 5.0:
            return None
        return (self.clock_last[0] - self.clock_first[0]) / wall

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

    def navigator_active(self, timeout):
        """Wait until bt_navigator is active; return the outcome."""
        # Nav2 1.5 offers /navigate_to_pose before the lifecycle manager activates bt_navigator,
        # and rejects goals until then; waiting for the action server alone is not enough there.
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.navigator_state.wait_for_service(timeout_sec=1.0):
                future = self.navigator_state.call_async(GetState.Request())
                if self.wait_until(future.done, 5.0) and future.result() is not None and \
                        future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return True
            self.wait_until(lambda: False, 0.5)
        return False

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


def write_environment(out_dir, argv, scenario, controller, profile, goal_timeout, hazards='off'):
    """Write environment.txt: the host, the commands and the scenario."""
    lines = environment_lines(argv)
    lines += [
        'scenario: ' + scenario,
        'controller: {} (recorded only: the launch chose it)'.format(controller),
        'hazards: {} (recorded only: the launch chose it)'.format(hazards),
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


def build_checks(controller, goals, dynamic=False):
    """
    Return the check rows: dwa's verdicts, and the same rows as information for the others.

    In a dynamic scenario the navigation rows are information for every controller: the
    collisions there are the measured baseline.
    """
    n = len(goals)
    reached = sum(1 for g in goals if g['truth_success'])
    episodes = sum(g['collision_episodes'] for g in goals)
    clearances = [g['min_clearance_m'] for g in goals if g['min_clearance_m'] is not None]
    minimum = min(clearances) if clearances else None
    disagreements = sum(1 for g in goals if g['disagreement'])

    def verdict(ok):
        """PASS or FAIL for dwa; INFO for the reference controllers."""
        if controller != 'dwa' or dynamic:
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


def summary_text(scenario, controller, checks, goals, validity=(), hazards='off', rtf=None):
    """Return summary.md: the validity table, the check table and the per-goal table."""
    lines = ['# nav_run: {} with {}'.format(scenario, controller), '',
             'hazards: {}'.format(hazards)]
    if rtf is not None:
        lines.append('real-time factor (measured): {:.2f}'.format(rtf))
    lines.append('')
    if validity:
        lines += ['## Scenario validity (did it play out as scripted)', '',
                  '| check | value | expected | verdict |', '|---|---|---|---|']
        for row in validity:
            lines.append('| {} | {} | {} | {} |'.format(
                row['check'], row['value'], row['expected'], row['verdict']))
        lines += ['']
    lines += ['## Navigation', '', '| check | value | expected | verdict |', '|---|---|---|---|']
    for row in checks:
        lines.append('| {} | {} | {} | {} |'.format(
            row['check'], row['value'], row['expected'], row['verdict']))
    dynamic = bool(validity)
    header = ('| goal | target (x, y, yaw) | Nav2 | truth | error (m, rad) | time (s) | '
              'travelled (m) | tracking, plan in force (m) | min clearance (m) | collisions | '
              'recoveries | odom drift (m) | AMCL error (m) |')
    columns = 13
    if dynamic:
        header += (' min static (m) | min dynamic (m) | closest approach (m) | '
                   'close encounters (< 0.30 m) |')
        columns += 4
    lines += ['', '## Goals', '', header, '|' + '---|' * columns]
    for g in goals:
        target = g['target']
        row = ('| {} | ({}, {}, {}) | {} | {} | {} / {} | {} | {} | {} | {} | {} | {} | '
               '{} | {} |'.format(
                   g['goal'] + 1, fmt(target['x'], 2), fmt(target['y'], 2),
                   fmt(target['yaw'], 2), g['nav2_status'],
                   'success' if g['truth_success'] else 'fail',
                   fmt(g['distance_error_m']), fmt(g['yaw_error_rad']),
                   fmt(g['time_s'], 1), fmt(g['travelled_m'], 2),
                   fmt(g['tracking_error_m'], 3), fmt(g['min_clearance_m'], 3),
                   g['collision_episodes'], g['recoveries'], fmt(g['odom_drift_m']),
                   fmt(g['amcl_error_m'])))
        if dynamic:
            row += ' {} | {} | {} | {} |'.format(
                fmt(g['min_clearance_static_m']), fmt(g['min_clearance_dynamic_m']),
                fmt(g['closest_approach_m']), g['close_encounters'])
        lines.append(row)
    return '\n'.join(lines) + '\n'


def draw_picture(path, scenario, goals, samples_per_goal, plans_per_goal, occupied, boxes,
                 grid_shape, obstacles=(), tracks=None, approaches=()):
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
    tracks = tracks or {}
    shapes = {o['name']: SHAPES[o['shape']] for o in obstacles}
    for name, track in tracks.items():
        points = [to_px(x, y) for x, y in track['xy']]
        if len(points) >= 2:
            draw.line(points, fill=(150, 150, 150), width=2)
        next_dot = 0.0
        for t, point in zip(track['times'], points):
            if t >= next_dot:
                draw.ellipse([point[0] - 3, point[1] - 3, point[0] + 3, point[1] + 3],
                             fill=(110, 110, 110))
                next_dot = t + 2.0
        if points:
            draw.text((points[0][0] + 6, points[0][1] - 6), name, fill=(90, 90, 90))
    for index, detail in enumerate(approaches):
        if not detail or detail.get('robot') is None or detail['distance'] > 3.0:
            continue
        rx, ry = to_px(*detail['robot'])
        draw.ellipse([rx - 7, ry - 7, rx + 7, ry + 7], outline=(0, 140, 0), width=3)
        draw.text((rx + 9, ry - 6), 'g{}'.format(index + 1), fill=(0, 110, 0))
        spec = shapes.get(detail['obstacle'])
        ox, oy = detail['obstacle_xy']
        if spec is None:
            continue
        if spec['kind'] == 'cylinder':
            r = spec['radius']
            x0, y0 = to_px(ox - r, oy + r)
            x1, y1 = to_px(ox + r, oy - r)
            draw.ellipse([x0, y0, x1, y1], outline=(200, 90, 0), width=2)
        else:
            hx, hy = spec['size'][0] / 2.0, spec['size'][1] / 2.0
            x0, y0 = to_px(ox - hx, oy + hy)
            x1, y1 = to_px(ox + hx, oy - hy)
            draw.rectangle([x0, y0, x1, y1], outline=(200, 90, 0), width=2)
        draw.line([(rx, ry), to_px(ox, oy)], fill=(200, 90, 0), width=1)
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
    parser.add_argument('--hazards', default='off', choices=HAZARDS,
                        help='recorded in the bundle; the launch chose the hazard source')
    parser.add_argument('--out', default=None,
                        help='bundle folder (default nav_run_<scenario>_<controller>[_<hazards>]'
                             '_<time>)')
    parser.add_argument('--profile', default='native', choices=('native', 'docker'))
    parser.add_argument('--goal-timeout', type=float, default=120.0,
                        help='simulated seconds allowed per goal')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the scenario, write the bundle and print the summary; return the exit status."""
    argv_list = list(sys.argv if argv is None else argv)
    args = parse_arguments(argv_list[1:])
    tag = '' if args.hazards == 'off' else '_' + args.hazards
    out_dir = args.out or 'nav_run_{}_{}{}_{}'.format(
        args.scenario, args.controller, tag, stamp_now())
    os.makedirs(out_dir, exist_ok=True)
    write_environment(out_dir, argv_list, args.scenario, args.controller, args.profile,
                      args.goal_timeout, args.hazards)

    scenario = load_scenario(args.scenario)
    validate_scenario(args.scenario, scenario)
    obstacles = scenario.get('obstacles') or []
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
        if not node.navigator_active(SERVER_WAIT_S):
            raise RuntimeError('bt_navigator is not active within {} s'.format(SERVER_WAIT_S))
        if obstacles:
            node.record_obstacles([o['name'] for o in obstacles])
            if not node.wait_until(lambda: node.ready, READY_WAIT_S):
                print('no obstacle driver: launch sim.launch.py with scenario:={}'.format(
                    args.scenario), file=sys.stderr)
                return 2
        if scenario.get('boxes'):
            boxes = spawn_boxes(scenario['boxes'], box_size,
                                os.path.join(out_dir, 'spawn.txt'))
            node.spin_for_sim(SPAWN_SETTLE_SIM_S)

        goals, samples_per_goal, plans = [], [], []
        for index, goal in enumerate(scenario['goals']):
            node.announce_goal(index + 1)
            samples, timing, outcome = run_goal(node, goal, args.goal_timeout)
            # score against the obstacles' tracks, which cover the whole run so far
            tracks_so_far = obstacle_tracks(obstacles, node.obstacle_samples)
            scored = score_goal(index, goal, samples, node.plan_history, timing, outcome,
                                occupied, boxes, tracks_so_far)
            plans.append([plan for _, plan in node.plan_history])
            goals.append(scored)
            samples_per_goal.append(samples)
            print('goal {}: {} truth={} time={} s'.format(
                index + 1, outcome['status_name'],
                'success' if scored['truth_success'] else 'fail', fmt(scored['time_s'], 1)),
                flush=True)
    finally:
        node_samples = {name: list(rows) for name, rows in node.obstacle_samples.items()}
        node.destroy_node()
        rclpy.shutdown()

    checks = build_checks(args.controller, goals, dynamic=bool(obstacles))
    final_tracks = obstacle_tracks(obstacles, node_samples)
    validity = []
    if obstacles:
        robot_times = [s['t'] for samples in samples_per_goal for s in samples
                       if s['truth'] is not None]
        robot_xy = [s['truth'][:2] for samples in samples_per_goal for s in samples
                    if s['truth'] is not None]
        encounter = None
        if len(goals) >= 2:
            encounter = goals[1]['closest_approach']['by_obstacle'].get('walker_1')
        validity = validity_rows(args.scenario, scenario,
                                 {t['name']: t for t in final_tracks}, robot_times, robot_xy,
                                 encounter, ROBOT_RADIUS_M)
    rtf = node.measured_rtf()
    if rtf is not None:
        with open(os.path.join(out_dir, 'environment.txt'), 'a', encoding='utf-8') as handle:
            handle.write('real-time factor (measured): {:.2f}\n'.format(rtf))
    results = {
        'scenario': args.scenario,
        'controller': args.controller,
        'hazards': args.hazards,
        'rtf': rtf,
        'profile': args.profile,
        'goal_timeout_s': args.goal_timeout,
        'checks': checks,
        'validity': validity,
        'tracks': {t['name']: downsample(t) for t in final_tracks},
        'boxes': boxes,
        'goals': goals,
        'samples': samples_per_goal,
    }
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as handle:
        json.dump(results, handle, indent=1)
    summary = summary_text(args.scenario, args.controller, checks, goals, validity, args.hazards,
                           rtf)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(summary)
    draw_picture(os.path.join(out_dir, 'trajectories_{}.png'.format(args.scenario)),
                 args.scenario, scenario['goals'], samples_per_goal, plans, occupied, boxes,
                 grid.shape, obstacles, {t['name']: t for t in final_tracks},
                 [g['closest_approach'] for g in goals] if obstacles else ())
    print(summary)
    print('bundle: ' + out_dir)
    failed = [row for row in checks + validity if row['verdict'] == 'FAIL']
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
