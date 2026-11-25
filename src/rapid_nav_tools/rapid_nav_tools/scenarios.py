"""Moving obstacles of the dynamic scenarios: shapes, SDF, scripted paths and start triggers."""

import math
import re

import numpy as np
from rapid_nav_tools.checks import (
    dynamic_clearance,
    interpolate_track,
    longest_stall,
    moving_speed,
    path_conformance,
)


MASS_KG = 1000.0
TILT_INERTIA = 1.0e5
TILT_GAIN = 2.0
FLOOR_CLEARANCE_M = 0.02
# The texture the camera can follow; the flat colour stays as the fallback (ambient and diffuse).
TEXTURE_URI = 'model://rapid_nav_movers/materials/textures/mover.png'

SHAPES = {
    'person': {'kind': 'cylinder', 'radius': 0.20, 'height': 1.70, 'colour': (0.85, 0.35, 0.10)},
    'cart': {'kind': 'box', 'size': (0.60, 0.40, 0.90), 'colour': (0.15, 0.45, 0.80)},
}


def shape_height(shape):
    """Return the height of a named shape in metres."""
    spec = SHAPES[shape]
    if spec['kind'] == 'cylinder':
        return spec['height']
    return spec['size'][2]


def spawn_height(shape):
    """Return the z of the model origin: half the height plus a small gap above the floor."""
    return shape_height(shape) / 2.0 + FLOOR_CLEARANCE_M


def obstacle_topics(name):
    """Return the Gazebo topics (velocity command, odometry) of the obstacle called name."""
    return '/model/{}/cmd_vel'.format(name), '/model/{}/odometry'.format(name)


def inertia(shape, mass=MASS_KG):
    """Return (ixx, iyy, izz) of the solid shape, from its centre of mass."""
    spec = SHAPES[shape]
    if spec['kind'] == 'cylinder':
        r, h = spec['radius'], spec['height']
        side = mass * (3.0 * r * r + h * h) / 12.0
        return side, side, mass * r * r / 2.0
    sx, sy, sz = spec['size']
    return (mass * (sy * sy + sz * sz) / 12.0,
            mass * (sx * sx + sz * sz) / 12.0,
            mass * (sx * sx + sy * sy) / 12.0)


def body_command(world_vx, world_vy, roll, pitch, yaw):
    """
    Return (vx, vy, vz, wx, wy, wz): the body-frame command for a horizontal world velocity.

    The world velocity (world_vx, world_vy, 0) is turned into the body frame with the inverse of
    the full orientation (roll, pitch, yaw), so a tilted body is never driven into the floor,
    and the angular command pulls roll, pitch and yaw back to zero.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz(yaw) Ry(pitch) Rx(roll); the body velocity is R^T v.
    r = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
         (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
         (-sp, cp * sr, cp * cr))
    v = (world_vx, world_vy, 0.0)
    body = [sum(r[row][col] * v[row] for row in range(3)) for col in range(3)]
    return (body[0], body[1], body[2], -TILT_GAIN * roll, -TILT_GAIN * pitch,
            -TILT_GAIN * wrap_angle(yaw))


def _geometry(spec):
    if spec['kind'] == 'cylinder':
        return '<cylinder><radius>{}</radius><length>{}</length></cylinder>'.format(
            spec['radius'], spec['height'])
    return '<box><size>{} {} {}</size></box>'.format(*spec['size'])


def obstacle_sdf(name, shape, gravity=False):
    """Return the SDF of a movable obstacle: a body that follows a velocity command."""
    spec = SHAPES[shape]
    cmd_topic, odom_topic = obstacle_topics(name)
    # Heavy and hard to tip: a push from the robot must not move or tilt the obstacle.
    izz = inertia(shape)[2]
    ixx = iyy = TILT_INERTIA
    colour = '{} {} {} 1'.format(*spec['colour'])
    geometry = _geometry(spec)
    return (
        '<sdf version="1.9"><model name="{name}">'
        '<link name="body">'
        '<gravity>{gravity}</gravity>'
        '<inertial><mass>{mass}</mass><inertia>'
        '<ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>'
        '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
        '<collision name="collision"><geometry>{geometry}</geometry></collision>'
        '<visual name="visual"><geometry>{geometry}</geometry>'
        '<material><ambient>{colour}</ambient><diffuse>{colour}</diffuse>'
        '<pbr><metal><albedo_map>{texture}</albedo_map></metal></pbr></material></visual>'
        '</link>'
        '<plugin filename="gz-sim-velocity-control-system" '
        'name="gz::sim::systems::VelocityControl">'
        '<topic>{cmd}</topic></plugin>'
        '<plugin filename="gz-sim-odometry-publisher-system" '
        'name="gz::sim::systems::OdometryPublisher">'
        '<odom_frame>world</odom_frame>'
        '<robot_base_frame>{name}</robot_base_frame>'
        '<odom_publish_frequency>30</odom_publish_frequency>'
        '<odom_topic>{odom}</odom_topic>'
        '<tf_topic>/model/{name}/tf</tf_topic>'
        '<dimensions>3</dimensions></plugin>'
        '</model></sdf>'
    ).format(name=name, gravity='true' if gravity else 'false', mass=MASS_KG,
             ixx=ixx, iyy=iyy, izz=izz, geometry=geometry, colour=colour,
             cmd=cmd_topic, odom=odom_topic, texture=TEXTURE_URI)


MODES = ('once', 'back_and_forth', 'loop')
NAME_PATTERN = re.compile(r'^[a-z0-9_]+$')
MAX_SPEED_MPS = 1.0
ARRIVE_M = 0.05
STEP_S = 0.05
CROSS_TRACK_GAIN = 1.5
YAW_GAIN = 2.0


def validate_scenario(name, scenario):
    """Raise ValueError naming the scenario and the field when its obstacles are malformed."""
    def fail(field, why):
        raise ValueError('scenario {}: {}: {}'.format(name, field, why))

    obstacles = scenario.get('obstacles') or []
    goal_count = len(scenario.get('goals') or [])
    seen = set()
    for index, obstacle in enumerate(obstacles):
        label = 'obstacles[{}]'.format(index)
        obstacle_name = obstacle.get('name')
        if not isinstance(obstacle_name, str) or not NAME_PATTERN.match(obstacle_name):
            fail(label + '.name', 'must match [a-z0-9_]+, got {!r}'.format(obstacle_name))
        if obstacle_name in seen:
            fail(label + '.name', 'duplicate name ' + obstacle_name)
        seen.add(obstacle_name)
        if obstacle.get('shape') not in SHAPES:
            fail(label + '.shape', 'unknown shape {!r}'.format(obstacle.get('shape')))
        if obstacle.get('mode', 'once') not in MODES:
            fail(label + '.mode', 'unknown mode {!r}'.format(obstacle.get('mode')))
        path = obstacle.get('path') or []
        if len(path) < 2:
            fail(label + '.path', 'needs at least 2 points')
        for point in path:
            if 'x' not in point or 'y' not in point:
                fail(label + '.path', 'every point needs x and y')
        speed = obstacle.get('speed')
        if not isinstance(speed, (int, float)) or not 0.0 < speed <= MAX_SPEED_MPS:
            fail(label + '.speed', 'must be in (0, {}], got {!r}'.format(MAX_SPEED_MPS, speed))
        start = obstacle.get('start')
        if start is not None:
            if 'robot_within' in start and 'of' not in start:
                fail(label + '.start', 'robot_within needs of')
            goal = start.get('goal', 1)
            if not isinstance(goal, int) or goal < 1 or goal > goal_count:
                fail(label + '.start.goal', 'must be 1..{}, got {!r}'.format(goal_count, goal))


def wrap_angle(angle):
    """Return the angle wrapped to (-pi, pi]."""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return math.pi if wrapped <= -math.pi else wrapped


class PathFollower:
    """Turn the current pose into a body-frame velocity that follows a scripted polyline."""

    def __init__(self, path, speed, mode='once'):
        """Start on path[0], heading for path[1]."""
        self.points = [(float(p['x']), float(p['y'])) for p in path]
        self.speed = float(speed)
        self.mode = mode
        self.previous = 0
        self.target = 1
        self.direction = 1
        self.finished = False

    def _advance(self):
        last = len(self.points) - 1
        if self.mode == 'once':
            if self.target == last:
                self.finished = True
                return
            self.previous, self.target = self.target, self.target + 1
        elif self.mode == 'back_and_forth':
            if not 0 <= self.target + self.direction <= last:
                self.direction = -self.direction
            self.previous, self.target = self.target, self.target + self.direction
        else:
            self.previous, self.target = self.target, (self.target + 1) % len(self.points)

    def step(self, x, y, yaw):
        """Return (vx, vy, wz) in the body frame for the pose (x, y, yaw)."""
        if self.finished:
            return 0.0, 0.0, 0.0
        tx, ty = self.points[self.target]
        if math.hypot(tx - x, ty - y) <= ARRIVE_M:
            self._advance()
            if self.finished:
                return 0.0, 0.0, 0.0
            tx, ty = self.points[self.target]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        ux, uy = (dx / dist, dy / dist) if dist > 1e-9 else (0.0, 0.0)
        wx, wy = self.speed * ux, self.speed * uy
        # Cross-track term: the vector from the pose back to the line of the current segment.
        ax, ay = self.points[self.previous]
        sx, sy = tx - ax, ty - ay
        length = math.hypot(sx, sy)
        if length > 1e-9:
            along = ((x - ax) * sx + (y - ay) * sy) / (length * length)
            ex, ey = ax + along * sx - x, ay + along * sy - y
            wx += CROSS_TRACK_GAIN * ex
            wy += CROSS_TRACK_GAIN * ey
        norm = math.hypot(wx, wy)
        if norm > self.speed:
            wx, wy = wx * self.speed / norm, wy * self.speed / norm
        if dist < self.speed * STEP_S:
            norm = math.hypot(wx, wy)
            if norm > 1e-9:
                wx, wy = wx * (dist / STEP_S) / norm, wy * (dist / STEP_S) / norm
        yaw = wrap_angle(yaw)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        return (cos_y * wx + sin_y * wy, -sin_y * wx + cos_y * wy, -YAW_GAIN * yaw)


class StartTrigger:
    """Fire once: when its goal is current and the robot is close enough (latched)."""

    def __init__(self, start=None):
        """Read the optional start block of an obstacle."""
        start = start or {}
        self.goal = int(start.get('goal', 1))
        self.within = start.get('robot_within')
        centre = start.get('of')
        self.centre = (float(centre['x']), float(centre['y'])) if centre else None
        self.fired = False

    def update(self, goal_index, robot_xy):
        """Return whether the obstacle has started; goal_index is 1-based (0: none sent yet)."""
        if self.fired:
            return True
        if goal_index < self.goal:
            return False
        if self.within is not None:
            if robot_xy is None:
                return False
            if math.hypot(robot_xy[0] - self.centre[0],
                          robot_xy[1] - self.centre[1]) > float(self.within):
                return False
        self.fired = True
        return True


TRIGGER_MOVE_M = 0.5
CONFORMANCE_M = 0.10
CONTACT_M = 0.05
SPEED_TOLERANCE = 0.10
ENCOUNTER_M = 1.5


def track_spec(obstacle, times, xy):
    """Return the track dict of dynamic_clearance for a scenario obstacle and its samples."""
    spec = SHAPES[obstacle['shape']]
    track = {'name': obstacle['name'], 'times': list(times), 'xy': [list(p) for p in xy]}
    if spec['kind'] == 'cylinder':
        track.update(kind='circle', radius=spec['radius'])
    else:
        track.update(kind='box', half_x=spec['size'][0] / 2.0, half_y=spec['size'][1] / 2.0)
    return track


def validity_rows(scenario_name, scenario, tracks, robot_times, robot_xy, encounter_m, radius):
    """
    Return the rows that judge whether the scenario played out as scripted.

    tracks maps an obstacle's name to its track dict (times, xy, shape); encounter_m is the
    robot's closest approach to walker_1 during goal 2 (None when unknown).
    """
    rows = []

    def add(check, value, expected, ok):
        rows.append({'check': check, 'value': value, 'expected': expected,
                     'verdict': 'PASS' if ok else 'FAIL'})

    for obstacle in scenario.get('obstacles') or []:
        name = obstacle['name']
        track = tracks[name]
        times = np.asarray(track['times'], dtype=np.float64)
        xy = np.asarray(track['xy'], dtype=np.float64).reshape(-1, 2)
        moved = float(np.max(np.hypot(xy[:, 0] - xy[0, 0], xy[:, 1] - xy[0, 1]))) \
            if xy.size else 0.0
        if obstacle.get('start'):
            add('{} started'.format(name), '{:.2f} m moved'.format(moved),
                '> {} m'.format(TRIGGER_MOVE_M), moved > TRIGGER_MOVE_M)
        if xy.shape[0] and len(robot_times):
            at_robot = interpolate_track(robot_times, robot_xy, times)
            gap, _ = dynamic_clearance(at_robot, times, [track], radius)
            free = gap > CONTACT_M
            gap_to_robot = gap
        else:
            free = np.ones(xy.shape[0], dtype=bool)
            gap_to_robot = np.full(xy.shape[0], np.inf)
        deviation = path_conformance(xy[free], obstacle['path'], obstacle.get('mode', 'once')) \
            if free.any() else 0.0
        add('{} follows its path'.format(name), '{:.3f} m'.format(deviation),
            '<= {} m'.format(CONFORMANCE_M), deviation <= CONFORMANCE_M)
        stall = longest_stall(times, xy, gap_to_robot, float(obstacle['speed']),
                              obstacle['path'], obstacle.get('mode', 'once'))
        add('{} never stalls'.format(name), '{:.1f} s'.format(stall), '<= 1.0 s', stall <= 1.0)
        speed = moving_speed(times, xy, mask=free)
        wanted = float(obstacle['speed'])
        add('{} speed'.format(name), '{:.3f} m/s'.format(speed),
            '{:.2f} m/s +- 10 %'.format(wanted), abs(speed - wanted) <= SPEED_TOLERANCE * wanted)
    if scenario_name in ('crossing', 'head_on'):
        add('encounter happened (walker_1, goal 2)',
            '-' if encounter_m is None else '{:.2f} m'.format(encounter_m),
            '<= {} m'.format(ENCOUNTER_M), encounter_m is not None and encounter_m <= ENCOUNTER_M)
    return rows


def covering_radius(shape):
    """Return the radius of the circle around the outline (box: half the diagonal)."""
    spec = SHAPES[shape]
    if spec['kind'] == 'cylinder':
        return spec['radius']
    return 0.5 * math.hypot(spec['size'][0], spec['size'][1])


def world_velocity(orientation_xyzw, body_linear_xyz):
    """Return (vx, vy) in the world frame of a body-frame velocity (full orientation)."""
    x, y, z, w = orientation_xyzw
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    vx, vy, vz = body_linear_xyz
    row0 = (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w))
    row1 = (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w))
    return (row0[0] * vx + row0[1] * vy + row0[2] * vz,
            row1[0] * vx + row1[1] * vy + row1[2] * vz)


def yield_factor(obstacle_xy, velocity_xy, robot_xy, stop_m=0.9, slow_m=1.8, cone_deg=75.0):
    """
    Return the share (0..1) of its scripted speed an obstacle keeps to make way for the robot.

    A person slows down when the robot is ahead of them (within cone_deg of their walking
    direction) and closer than slow_m (centre to centre), and stops at stop_m, as people do; a
    robot beside or behind them does not slow them. With no robot position or no motion: 1.
    """
    if robot_xy is None:
        return 1.0
    vx, vy = velocity_xy
    speed = math.hypot(vx, vy)
    dx, dy = robot_xy[0] - obstacle_xy[0], robot_xy[1] - obstacle_xy[1]
    dist = math.hypot(dx, dy)
    if speed < 1e-9 or dist >= slow_m:
        return 1.0
    if dist > 1e-9 and (dx * vx + dy * vy) / (dist * speed) < math.cos(math.radians(cone_deg)):
        return 1.0
    return min(1.0, max(0.0, (dist - stop_m) / (slow_m - stop_m)))
