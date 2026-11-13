"""
planner_check: check the global planners on the RAPID-NAV maps.

Offline (--mode offline) it asks the planner server, started by planning.launch.py with static
localisation, for a path for every query and every planner. In the simulation (--mode sim) it first
checks AMCL against the ground truth through the scripted drive, then runs the same queries. Every
query goes to every planner id with use_start true. The bundle folder holds summary.md,
table_<map>.md, results.csv, results.json, paths_<map>.png and environment.txt.
"""

import argparse
import bisect
import csv
import datetime
import json
import math
import os
import statistics
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from PIL import Image, ImageDraw
from rapid_nav_tools.checks import (
    clearances,
    heading_change_per_metre,
    max_step,
    occupied_cells_xy,
    path_length,
)
from rapid_nav_tools.sim_check import environment_lines, format_number, Report, verdict_of
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
import yaml

PLANNER_IDS = ('RapidAStar', 'RapidAStarSmooth', 'RapidDijkstra', 'RapidThetaStar',
               'NavFn', 'Smac2D', 'NavThetaStar')
OUR_PLANNERS = ('RapidAStar', 'RapidAStarSmooth', 'RapidDijkstra', 'RapidThetaStar')
PALETTE = ((230, 25, 75), (60, 180, 75), (0, 130, 200), (245, 130, 48),
           (145, 30, 180), (70, 170, 170), (200, 90, 200))
EXPECTED_CODES = {'goal_occupied': 206, 'goal_outside': 204}
ERROR_NAMES = {0: 'none', 200: 'unknown', 201: 'invalid planner', 202: 'tf error',
               203: 'start outside map', 204: 'goal outside map', 205: 'start occupied',
               206: 'goal occupied', 207: 'timeout', 208: 'no valid path'}
CLEARANCE_MIN_M = 0.20
LENGTH_RATIO = 1.02
THETA_SLACK_M = 0.05
TIME_LIMIT_MS = 200.0
OCCUPIED_VALUE = 65
SERVER_WAIT_S = 120.0
MAP_WAIT_S = 120.0
CLOCK_WAIT_S = 120.0
REQUEST_WAIT_S = 60.0
STATS_WAIT_S = 0.2
SETTLE_SIM_S = 20.0
CYCLE_SETTLE_S = 5.0
CMD_PERIOD_S = 0.05
DRIVE_SPEED = 0.3
DRIVE_TIME = 4.0
STOP_TIME = 1.0
TURN_RATE = 0.6
TURN_TIME = 5.236
POSITION_TOLERANCE_M = 0.10
YAW_TOLERANCE_DEG = 3.0


def duration_seconds(duration):
    """Return a builtin_interfaces duration in seconds."""
    return duration.sec + duration.nanosec * 1e-9


def yaw_of(quaternion):
    """Return the yaw angle (rad) of a geometry_msgs quaternion."""
    q = quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Return the angle wrapped to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_stamped(pose):
    """Return a PoseStamped in the map frame from an {x, y, yaw} mapping."""
    message = PoseStamped()
    message.header.frame_id = 'map'
    message.pose.position.x = float(pose['x'])
    message.pose.position.y = float(pose['y'])
    message.pose.orientation.z = math.sin(0.5 * float(pose['yaw']))
    message.pose.orientation.w = math.cos(0.5 * float(pose['yaw']))
    return message


def load_queries(map_name, path=None):
    """Return the queries of one map from planner_queries.yaml (the installed copy by default)."""
    if path is None:
        share = get_package_share_directory('rapid_nav_navigation')
        path = os.path.join(share, 'config', 'planner_queries.yaml')
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if map_name not in data:
        raise ValueError('no queries for map {!r} in {}'.format(map_name, path))
    queries = []
    for item in data[map_name]:
        if item['expect'] not in ('success', 'goal_occupied', 'goal_outside'):
            raise ValueError('bad expect in query {!r}'.format(item['name']))
        queries.append({'name': item['name'], 'start': item['start'], 'goal': item['goal'],
                        'expect': item['expect']})
    return queries


def expected_map_info(map_name):
    """Return the resolution and origin of a built-in map from its YAML file."""
    share = get_package_share_directory('rapid_nav_navigation')
    with open(os.path.join(share, 'maps', map_name + '.yaml'), encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return float(data['resolution']), [float(v) for v in data['origin'][:2]]


class PlannerCheck(Node):
    """Talk to the planner server and, in the simulation, to AMCL and the drive."""

    def __init__(self, sim):
        """Create the action client, the subscriptions and the velocity publisher."""
        super().__init__(
            'planner_check',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, sim)],
        )
        self.map_msg = None
        self.costmap_msg = None
        self.stats = {}
        self.amcl = []
        self.truth = []
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_costmap,
                                 latched)
        for planner_id in OUR_PLANNERS:
            self.create_subscription(String, '/' + planner_id + '/stats',
                                     self._stats_recorder(planner_id), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl.append, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.truth.append, 10)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_map(self, message):
        """Keep the latest map."""
        self.map_msg = message

    def _on_costmap(self, message):
        """Keep the latest global costmap."""
        self.costmap_msg = message

    def costmap_has_map(self):
        """
        Return True once the global costmap marks the map's obstacles.

        The planner server's costmap takes the map a moment after /map is published; a query sent
        before that plans over free space. Lethal cells are 100 in the published costmap; at least
        as many as the map's occupied cells must be marked (inflation only adds to them).
        """
        if self.map_msg is None or self.costmap_msg is None:
            return False
        occupied = int(np.count_nonzero(np.asarray(self.map_msg.data, dtype=np.int16) >= 65))
        lethal = int(np.count_nonzero(np.asarray(self.costmap_msg.data, dtype=np.int16) >= 99))
        return occupied > 0 and lethal >= occupied

    def _stats_recorder(self, planner_id):
        """Return the callback that keeps the latest JSON statistics of one of our planners."""
        def record(message):
            try:
                self.stats[planner_id] = json.loads(message.data)
            except ValueError:
                self.stats[planner_id] = None
        return record

    def now(self):
        """Return the clock of this node in seconds (simulated when use_sim_time is set)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_until(self, condition, timeout):
        """Spin until the condition holds or the wall-clock timeout passes; return the result."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(condition())

    def spin_sim(self, duration):
        """Process callbacks for the given simulated time."""
        end = self.now() + duration
        while self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def transform_available(self, target, source):
        """Return True when the transform of source in target can be looked up now."""
        try:
            self.tf_buffer.lookup_transform(target, source, Time())
        except tf2_ros.TransformException:
            return False
        return True

    def plan(self, start, goal, planner_id):
        """Ask the planner server for a path from start to goal and return what came back."""
        request = ComputePathToPose.Goal()
        request.start = pose_stamped(start)
        request.goal = pose_stamped(goal)
        request.planner_id = planner_id
        request.use_start = True
        self.stats.pop(planner_id, None)
        answer = {'error_code': None, 'error_msg': '', 'path': [], 'planning_s': None,
                  'stats': None, 'roundtrip_ms': None}
        started = time.monotonic()
        future = self.client.send_goal_async(request)
        if not self.wait_until(future.done, REQUEST_WAIT_S):
            answer['error_msg'] = 'no response'
            return answer
        handle = future.result()
        if not handle.accepted:
            answer['error_msg'] = 'goal rejected'
            return answer
        result_future = handle.get_result_async()
        if not self.wait_until(result_future.done, REQUEST_WAIT_S):
            answer['error_msg'] = 'no result'
            return answer
        result = result_future.result().result
        if planner_id in OUR_PLANNERS:
            self.wait_until(lambda: planner_id in self.stats, STATS_WAIT_S)
        answer['error_code'] = int(result.error_code)
        answer['error_msg'] = result.error_msg
        answer['path'] = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
        answer['planning_s'] = duration_seconds(result.planning_time)
        answer['stats'] = self.stats.get(planner_id)
        answer['roundtrip_ms'] = (time.monotonic() - started) * 1000.0
        return answer

    def command(self, linear, angular, duration):
        """Send a velocity for the given simulated time, then stop."""
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        start = self.now()
        last = None
        while self.now() - start < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            if last is None or self.now() - last >= CMD_PERIOD_S:
                self.cmd.publish(message)
                last = self.now()
        self.cmd.publish(Twist())

    def drive_cycle(self):
        """Drive forward, stop, turn in place, stop: the S1 drive."""
        self.command(DRIVE_SPEED, 0.0, DRIVE_TIME)
        self.command(0.0, 0.0, STOP_TIME)
        self.command(0.0, TURN_RATE, TURN_TIME)
        self.command(0.0, 0.0, STOP_TIME)

    def truth_near(self, stamp, truth_stamps):
        """Return the ground-truth message closest in time to a stamp, or None."""
        index = bisect.bisect_left(truth_stamps, stamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(self.truth)]
        if not candidates:
            return None
        best = min(candidates, key=lambda i: abs(truth_stamps[i] - stamp))
        return self.truth[best]

    def localisation_error(self):
        """Return the stamp, position and yaw error of the latest AMCL pose, and both poses."""
        if not self.amcl or not self.truth:
            return None
        truth_stamps = [duration_seconds(m.header.stamp) for m in self.truth]
        message = self.amcl[-1]
        stamp = duration_seconds(message.header.stamp)
        truth = self.truth_near(stamp, truth_stamps)
        if truth is None:
            return None
        a = message.pose.pose
        t = truth.pose.pose
        position = math.hypot(a.position.x - t.position.x, a.position.y - t.position.y)
        yaw = math.degrees(abs(wrap(yaw_of(a.orientation) - yaw_of(t.orientation))))
        amcl_pose = (a.position.x, a.position.y, math.degrees(yaw_of(a.orientation)))
        truth_pose = (t.position.x, t.position.y, math.degrees(yaw_of(t.orientation)))
        return (stamp, position, yaw, amcl_pose, truth_pose)

    def worst_localisation_error(self, since):
        """Return the largest position (m) and yaw (deg) error of the poses after index."""
        truth_stamps = [duration_seconds(m.header.stamp) for m in self.truth]
        worst_position = 0.0
        worst_yaw = 0.0
        for message in self.amcl[since:]:
            truth = self.truth_near(duration_seconds(message.header.stamp), truth_stamps)
            if truth is None:
                continue
            a = message.pose.pose
            t = truth.pose.pose
            distance = math.hypot(a.position.x - t.position.x, a.position.y - t.position.y)
            worst_position = max(worst_position, distance)
            worst_yaw = max(worst_yaw, math.degrees(abs(wrap(
                yaw_of(a.orientation) - yaw_of(t.orientation)))))
        return worst_position, worst_yaw


def localisation_checkpoint(node, report, values, label, since):
    """Compare the latest AMCL pose with the ground truth and record the window's worst error."""
    error = node.localisation_error()
    worst_position, worst_yaw = node.worst_localisation_error(since)
    if error is None:
        report.add('amcl ' + label, 'no pose', '<= 0.10 m, <= 3 deg', 'FAIL')
        return
    stamp, position, yaw, amcl_pose, truth_pose = error
    values['amcl ' + label] = {
        'position_m': position, 'yaw_deg': yaw, 'worst_position_m': worst_position,
        'worst_yaw_deg': worst_yaw, 'amcl_xy_yaw': amcl_pose, 'truth_xy_yaw': truth_pose,
    }
    ok = position <= POSITION_TOLERANCE_M and yaw <= YAW_TOLERANCE_DEG
    report.add('amcl ' + label,
               '{} m, {} deg (window worst {} m, {} deg)'.format(
                   format_number(position, 3), format_number(yaw, 2),
                   format_number(worst_position, 3), format_number(worst_yaw, 2)),
               '<= 0.10 m, <= 3 deg', verdict_of(ok))


def run_localisation(node, report, values):
    """Check AMCL against the ground truth at rest and after each of two drive cycles."""
    if not node.wait_until(lambda: bool(node.amcl) and bool(node.truth), CLOCK_WAIT_S):
        report.add('amcl pose', 'none', 'messages on /amcl_pose', 'FAIL')
        return
    since = len(node.amcl)
    node.spin_sim(SETTLE_SIM_S)
    localisation_checkpoint(node, report, values, 'at rest', since)
    for cycle in (1, 2):
        since = len(node.amcl)
        node.drive_cycle()
        node.spin_sim(CYCLE_SETTLE_S)
        localisation_checkpoint(node, report, values, 'after drive cycle {}'.format(cycle), since)


def run_queries(node, map_name, queries, planner_ids, occupied):
    """Run every query on every planner and return one row per pair, with the path kept aside."""
    rows = []
    paths = {}
    for query in queries:
        for planner_id in planner_ids:
            answer = node.plan(query['start'], query['goal'], planner_id)
            points = np.asarray(answer['path'], dtype=np.float64).reshape(-1, 2)
            ok = answer['error_code'] == 0 and points.shape[0] > 0
            clearance = clearances(points, occupied)
            stats = answer['stats'] or {}
            code = answer['error_code']
            row = {
                'map': map_name,
                'query': query['name'],
                'expect': query['expect'],
                'planner': planner_id,
                'ok': ok,
                'error_code': code,
                'error_name': ERROR_NAMES.get(code, str(code)) if code is not None else 'none',
                'error_msg': answer['error_msg'],
                'poses': int(points.shape[0]),
                'length_m': path_length(points) if ok else None,
                'max_step_m': max_step(points) if ok else None,
                'heading_deg_per_m': heading_change_per_metre(points) if ok else None,
                'min_clearance_m': float(clearance.min()) if ok else None,
                'roundtrip_ms': answer['roundtrip_ms'],
                'planning_ms': answer['planning_s'] * 1000.0 if answer['planning_s'] is not None
                else None,
                'expansions': stats.get('expansions'),
                'stats_status': stats.get('status'),
                'stats_time_ms': stats.get('time_ms'),
            }
            rows.append(row)
            paths[(query['name'], planner_id)] = points
    return rows, paths


def median_or_none(values):
    """Return the median of the values, or None when there are none."""
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def mean_or_none(values):
    """Return the mean of the values, or None when there are none."""
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def evaluate(rows, planner_ids, profile):
    """Return the checks: per planner the feasible, infeasible and clearance rows; then pairs."""
    report = Report()
    feasible = [r for r in rows if r['expect'] == 'success']
    for planner_id in planner_ids:
        mine = [r for r in rows if r['planner'] == planner_id]
        mine_feasible = [r for r in mine if r['expect'] == 'success']
        succeeded = sum(1 for r in mine_feasible if r['ok'])
        report.add('{}: feasible queries succeed'.format(planner_id),
                   '{}/{}'.format(succeeded, len(mine_feasible)), 'all',
                   verdict_of(succeeded == len(mine_feasible) and mine_feasible))
        infeasible = [r for r in mine if r['expect'] != 'success']
        right = sum(1 for r in infeasible
                    if not r['ok'] and r['error_code'] == EXPECTED_CODES[r['expect']])
        if planner_id in OUR_PLANNERS:
            report.add('{}: infeasible queries fail with the right error'.format(planner_id),
                       '{}/{}'.format(right, len(infeasible)), 'all',
                       verdict_of(right == len(infeasible) and infeasible))
        else:
            # Nav2's own planners: their answers to impossible goals are a finding, not a failure.
            answers = ', '.join('{}: {}'.format(r['query'], 'path returned' if r['ok']
                                                else r['error_name']) for r in infeasible)
            report.add('{}: answers to infeasible queries'.format(planner_id),
                       answers or '-', 'reported only', 'INFO')
            into = [r for r in infeasible if r['ok']]
            if into:
                report.add('{}: returns a path into an obstacle'.format(planner_id),
                           '{} of {} infeasible queries'.format(len(into), len(infeasible)),
                           'reported only', 'INFO')
        clear = [r['min_clearance_m'] for r in mine_feasible if r['ok']]
        worst = min(clear) if clear else None
        report.add('{}: minimum clearance of every path'.format(planner_id),
                   format_number(worst, 3) + ' m' if worst is not None else '-',
                   '>= {:.2f} m'.format(CLEARANCE_MIN_M),
                   verdict_of(worst is not None and worst >= CLEARANCE_MIN_M))

    by_key = {(r['query'], r['planner']): r for r in feasible}
    names = sorted({r['query'] for r in feasible})
    pairs = [(by_key[(q, 'RapidAStar')], by_key[(q, 'RapidDijkstra')]) for q in names
             if (q, 'RapidAStar') in by_key and (q, 'RapidDijkstra') in by_key]
    compared = [(a, d) for a, d in pairs if a['ok'] and d['ok']]
    longer = [a['query'] for a, d in compared
              if a['length_m'] > LENGTH_RATIO * d['length_m'] + 1e-9]
    report.add('RapidAStar length <= 1.02 x RapidDijkstra',
               '{} pairs, {} longer'.format(len(compared), len(longer)),
               'no pair longer', verdict_of(compared and not longer))
    pairs = [(by_key[(q, 'RapidThetaStar')], by_key[(q, 'RapidAStar')]) for q in names
             if (q, 'RapidThetaStar') in by_key and (q, 'RapidAStar') in by_key]
    compared = [(t, a) for t, a in pairs if t['ok'] and a['ok']]
    longer = [t['query'] for t, a in compared if t['length_m'] > a['length_m'] + THETA_SLACK_M]
    report.add('RapidThetaStar length <= RapidAStar + 0.05 m',
               '{} pairs, {} longer'.format(len(compared), len(longer)),
               'no pair longer', verdict_of(compared and not longer))
    for planner_id in OUR_PLANNERS:
        times = [r['roundtrip_ms'] for r in feasible if r['planner'] == planner_id and r['ok']]
        median = median_or_none(times)
        if profile == 'docker':
            verdict = 'INFO'
        else:
            verdict = verdict_of(median is not None and median < TIME_LIMIT_MS)
        report.add('{}: median round-trip time'.format(planner_id),
                   format_number(median, 1) + ' ms' if median is not None else '-',
                   '< {:.0f} ms'.format(TIME_LIMIT_MS), verdict)
    return report


def table_rows(rows, planner_ids):
    """Return one table row per planner over the feasible queries."""
    table = []
    for planner_id in planner_ids:
        mine = [r for r in rows if r['planner'] == planner_id and r['expect'] == 'success']
        ok = [r for r in mine if r['ok']]
        expansions = [r['expansions'] for r in ok if r['expansions'] is not None]
        table.append({
            'planner': planner_id,
            'success': '{}/{}'.format(len(ok), len(mine)),
            'mean_length_m': mean_or_none([r['length_m'] for r in ok]),
            'mean_clearance_m': mean_or_none([r['min_clearance_m'] for r in ok]),
            'mean_heading_deg_per_m': mean_or_none([r['heading_deg_per_m'] for r in ok]),
            'median_ms': median_or_none([r['roundtrip_ms'] for r in ok]),
            'median_expansions': median_or_none(expansions) if planner_id in OUR_PLANNERS
            and expansions else None,
        })
    return table


def table_text(map_name, table):
    """Return the Markdown table of the planners."""
    lines = ['# planners on the {} map (feasible queries)'.format(map_name),
             'Round trip: wall-clock time of the planning action, request to result.', '',
             '| planner | success | mean length (m) | mean min clearance (m) | '
             'mean heading change (deg/m) | median round trip (ms) | median expansions |',
             '|---|---|---|---|---|---|---|']
    for row in table:
        lines.append('| {} | {} | {} | {} | {} | {} | {} |'.format(
            row['planner'], row['success'],
            format_number(row['mean_length_m'], 2), format_number(row['mean_clearance_m'], 3),
            format_number(row['mean_heading_deg_per_m'], 1), format_number(row['median_ms'], 1),
            format_number(row['median_expansions'], 0)))
    return '\n'.join(lines) + '\n'


def draw_paths(path_png, map_msg, rows, paths, planner_ids, scale=4):
    """Draw the map with every feasible path, one colour per planner, and a legend."""
    info = map_msg.info
    width, height = int(info.width), int(info.height)
    grid = np.asarray(map_msg.data, dtype=np.int16).reshape(height, width)
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    image[grid < 0] = (170, 170, 170)
    image[grid >= OCCUPIED_VALUE] = (0, 0, 0)
    size = (width * scale, height * scale)
    canvas = Image.fromarray(np.flipud(image)).resize(size, Image.NEAREST)
    draw = ImageDraw.Draw(canvas)
    resolution = float(info.resolution)
    origin_x = float(info.origin.position.x)
    origin_y = float(info.origin.position.y)

    def to_pixel(x, y):
        return ((x - origin_x) / resolution * scale,
                (height - (y - origin_y) / resolution) * scale)

    for index, planner_id in enumerate(planner_ids):
        colour = PALETTE[index % len(PALETTE)]
        for row in rows:
            if row['planner'] != planner_id or row['expect'] != 'success' or not row['ok']:
                continue
            points = paths[(row['query'], planner_id)]
            pixels = [to_pixel(x, y) for x, y in points]
            if len(pixels) > 1:
                draw.line(pixels, fill=colour, width=2)
    box_top = 8
    for index, planner_id in enumerate(planner_ids):
        y = box_top + index * 18
        draw.rectangle([8, y, 22, y + 12], fill=PALETTE[index % len(PALETTE)])
        draw.text((28, y), planner_id, fill=(0, 0, 0))
    canvas.save(path_png)


def write_csv(path, rows):
    """Write one row per query and planner."""
    columns = ['map', 'query', 'expect', 'planner', 'ok', 'error_code', 'error_name', 'error_msg',
               'poses', 'length_m', 'max_step_m', 'heading_deg_per_m', 'min_clearance_m',
               'roundtrip_ms', 'planning_ms', 'expansions', 'stats_status', 'stats_time_ms']
    with open(path, 'w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def summary_text(environment, checks, tables):
    """Return the Markdown summary: environment, the check table and the planner table."""
    lines = ['# planner_check summary', '']
    lines += ['- ' + line for line in environment]
    lines += ['', '| check | value | expected | verdict |', '|---|---|---|---|']
    for row in checks:
        lines.append('| {} | {} | {} | {} |'.format(
            row['check'], row['value'], row['expected'], row['verdict']))
    failures = sum(1 for row in checks if row['verdict'] == 'FAIL')
    lines += ['', 'Result: {} failed, {} checks.'.format(failures, len(checks)), '']
    lines += [tables, '']
    return '\n'.join(lines)


def parse_arguments(argv):
    """Parse the command line."""
    parser = argparse.ArgumentParser(description='Check the global planners on a RAPID-NAV map.')
    parser.add_argument('--map', choices=['arena', 'maze'], default='arena',
                        help='the map the planners are asked about (default arena)')
    parser.add_argument('--mode', choices=['offline', 'sim'], default='offline',
                        help='offline: the planner server only; sim: also AMCL and the drive')
    parser.add_argument('--out', default=None,
                        help='bundle folder (default: planner_check_<map>_<time> here)')
    parser.add_argument('--profile', choices=['native', 'docker'], default='native',
                        help='native: planning time is PASS/FAIL; docker: INFO only')
    parser.add_argument('--queries', default=None,
                        help='planner_queries.yaml (default: the installed copy)')
    return parser.parse_args(argv)


def run_checks(node, args, report, values):
    """Wait for the server and the map, check the simulation, then run the queries."""
    if not node.wait_until(lambda: node.client.server_is_ready(), SERVER_WAIT_S):
        report.add('compute_path_to_pose server', 'not ready', 'within 120 s', 'FAIL')
        return None, None, None
    report.add('compute_path_to_pose server', 'ready', 'within 120 s', 'PASS')
    if not node.wait_until(lambda: node.map_msg is not None, MAP_WAIT_S):
        report.add('/map', 'none', 'one message within 120 s', 'FAIL')
        return None, None, None
    info = node.map_msg.info
    resolution, origin = expected_map_info(args.map)
    same = (abs(float(info.resolution) - resolution) < 1e-9
            and abs(float(info.origin.position.x) - origin[0]) < 1e-6
            and abs(float(info.origin.position.y) - origin[1]) < 1e-6)
    report.add('/map is the {} map'.format(args.map),
               '{} x {} cells at {:.2f} m'.format(info.width, info.height, info.resolution),
               'origin ({:.2f}, {:.2f}) m'.format(origin[0], origin[1]), verdict_of(same))
    loaded = node.wait_until(node.costmap_has_map, MAP_WAIT_S)
    report.add('global costmap holds the map', 'yes' if loaded else 'no',
               'within {:.0f} s'.format(MAP_WAIT_S), verdict_of(loaded))
    if not loaded:
        return None, None, None
    if args.mode == 'sim':
        if not node.wait_until(lambda: node.now() > 0.0, CLOCK_WAIT_S):
            report.add('/clock', 'none', 'messages within 120 s', 'FAIL')
            return None, None, None
        mapped = node.wait_until(lambda: node.transform_available('map', 'odom'), MAP_WAIT_S)
        report.add('map to odom transform', 'available' if mapped else 'missing', 'available',
                   verdict_of(mapped))
        run_localisation(node, report, values)
    grid = np.asarray(node.map_msg.data, dtype=np.int16).reshape(info.height, info.width)
    occupied = occupied_cells_xy(grid, float(info.resolution),
                                 (float(info.origin.position.x), float(info.origin.position.y)))
    return grid, occupied, node.map_msg


def main(argv=None):
    """Run the checks, write the bundle and return the exit status (1 when a check failed)."""
    raw_argv = sys.argv[1:] if argv is None else list(argv)
    args = parse_arguments(raw_argv)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    out_dir = args.out or os.path.join(os.getcwd(), 'planner_check_{}_{}'.format(args.map, stamp))
    os.makedirs(out_dir, exist_ok=True)
    environment = environment_lines(raw_argv)
    queries = load_queries(args.map, args.queries)
    report = Report()
    values = {}
    rows = []
    paths = {}
    map_msg = None
    rclpy.init()
    node = None
    try:
        node = PlannerCheck(args.mode == 'sim')
        _, occupied, map_msg = run_checks(node, args, report, values)
        if occupied is not None:
            rows, paths = run_queries(node, args.map, queries, PLANNER_IDS, occupied)
            report.rows += evaluate(rows, PLANNER_IDS, args.profile).rows
    except KeyboardInterrupt:
        report.add('run', 'interrupted', 'complete', 'FAIL')
    finally:
        if node is not None:
            node.cmd.publish(Twist())
            node.destroy_node()
        rclpy.shutdown()

    table = table_rows(rows, PLANNER_IDS) if rows else []
    tables = table_text(args.map, table) if rows else 'no queries were run'
    with open(os.path.join(out_dir, 'environment.txt'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(environment) + '\n')
    with open(os.path.join(out_dir, 'table_{}.md'.format(args.map)), 'w',
              encoding='utf-8') as handle:
        handle.write(tables)
    write_csv(os.path.join(out_dir, 'results.csv'), rows)
    with open(os.path.join(out_dir, 'results.json'), 'w', encoding='utf-8') as handle:
        json.dump({'environment': environment, 'values': values, 'checks': report.rows,
                   'rows': rows}, handle, indent=2, default=float)
    if map_msg is not None and rows:
        draw_paths(os.path.join(out_dir, 'paths_{}.png'.format(args.map)), map_msg, rows, paths,
                   PLANNER_IDS)
    with open(os.path.join(out_dir, 'summary.md'), 'w', encoding='utf-8') as handle:
        handle.write(summary_text(environment, report.rows, tables))
    print(summary_text(environment, report.rows, tables))
    return 1 if report.count('FAIL') else 0


if __name__ == '__main__':
    sys.exit(main())
