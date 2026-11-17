"""Tests of rapid_nav_tools.scenarios, with hand-computed values."""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
from rapid_nav_tools.checks import occupied_cells_xy
from rapid_nav_tools.scenarios import (
    body_command,
    inertia,
    obstacle_sdf,
    obstacle_topics,
    PathFollower,
    spawn_height,
    StartTrigger,
    track_spec,
    validate_scenario,
    validity_rows,
)
import yaml


def test_sdf_parses_and_has_both_plugins():
    root = ET.fromstring(obstacle_sdf('walker_1', 'person'))
    model = root.find('model')
    assert model.get('name') == 'walker_1'
    assert model.find('static') is None
    plugins = {p.get('name'): p for p in model.findall('plugin')}
    velocity = plugins['gz::sim::systems::VelocityControl']
    assert velocity.find('topic').text == '/model/walker_1/cmd_vel'
    odometry = plugins['gz::sim::systems::OdometryPublisher']
    assert odometry.find('odom_frame').text == 'world'
    assert odometry.find('odom_topic').text == '/model/walker_1/odometry'
    assert odometry.find('tf_topic').text == '/model/walker_1/tf'
    assert odometry.find('dimensions').text == '3'
    assert obstacle_topics('walker_1') == ('/model/walker_1/cmd_vel',
                                           '/model/walker_1/odometry')


def test_sdf_gravity_flag_and_shapes():
    off = ET.fromstring(obstacle_sdf('c', 'cart', gravity=False))
    assert off.find('model/link/gravity').text == 'false'
    on = ET.fromstring(obstacle_sdf('c', 'cart', gravity=True))
    assert on.find('model/link/gravity').text == 'true'
    assert on.find('model/link/collision/geometry/box/size').text == '0.6 0.4 0.9'
    person = ET.fromstring(obstacle_sdf('p', 'person'))
    assert person.find('model/link/collision/geometry/cylinder/radius').text == '0.2'


def test_inertia_cylinder_and_box():
    # 30 * (3 * 0.04 + 2.89) / 12 = 7.525 ; 30 * 0.04 / 2 = 0.6
    ixx, iyy, izz = inertia('person', mass=30.0)
    assert ixx == pytest.approx(7.525)
    assert iyy == pytest.approx(7.525)
    assert izz == pytest.approx(0.6)
    # box 0.6 x 0.4 x 0.9: 30 * (0.16 + 0.81) / 12 = 2.425 ; 30 * (0.36 + 0.81) / 12 = 2.925 ;
    # 30 * (0.36 + 0.16) / 12 = 1.3
    ixx, iyy, izz = inertia('cart', mass=30.0)
    assert (ixx, iyy, izz) == pytest.approx((2.425, 2.925, 1.3))


def test_spawn_height():
    assert spawn_height('person') == pytest.approx(0.87)
    assert spawn_height('cart') == pytest.approx(0.47)


DT = 0.05


def simulate(follower, start, seconds, yaw=0.0):
    """Integrate the follower's body velocity; return the list of (t, x, y)."""
    x, y = start
    track = [(0.0, x, y)]
    for k in range(int(seconds / DT)):
        vx, vy, _ = follower.step(x, y, yaw)
        x += (vx * math.cos(yaw) - vy * math.sin(yaw)) * DT
        y += (vx * math.sin(yaw) + vy * math.cos(yaw)) * DT
        track.append(((k + 1) * DT, x, y))
    return track


def test_once_reaches_the_end_in_time_and_stays():
    follower = PathFollower([{'x': 1.5, 'y': -2.0}, {'x': 1.5, 'y': 0.8}], 0.5, 'once')
    track = simulate(follower, (1.5, -2.0), 8.0)
    assert follower.finished
    reached = next(t for t, x, y in track if math.hypot(x - 1.5, y - 0.8) <= 0.05)
    assert reached == pytest.approx(2.8 / 0.5, abs=0.2)
    assert math.hypot(track[-1][1] - 1.5, track[-1][2] - 0.8) <= 0.05
    assert follower.step(1.5, 0.8, 0.0) == (0.0, 0.0, 0.0)


def visited(follower, start, seconds, points):
    """Return the indices of the points in the order they were first reached again."""
    order = []
    for _, x, y in simulate(follower, start, seconds):
        for index, (px, py) in enumerate(points):
            if math.hypot(x - px, y - py) <= 0.06 and (not order or order[-1] != index):
                order.append(index)
    return order


def test_back_and_forth_visits_in_order_for_two_cycles():
    path = [{'x': 0.0, 'y': 0.0}, {'x': 2.0, 'y': 0.0}, {'x': 2.0, 'y': 1.0}]
    points = [(p['x'], p['y']) for p in path]
    order = visited(PathFollower(path, 0.5, 'back_and_forth'), points[0], 26.0, points)
    assert order[:9] == [0, 1, 2, 1, 0, 1, 2, 1, 0]


def test_loop_wraps_to_the_first_point():
    path = [{'x': 0.0, 'y': 0.0}, {'x': 1.0, 'y': 0.0}, {'x': 1.0, 'y': 1.0}]
    points = [(p['x'], p['y']) for p in path]
    order = visited(PathFollower(path, 0.5, 'loop'), points[0], 14.0, points)
    assert order[:7] == [0, 1, 2, 0, 1, 2, 0]


def test_start_off_the_path_converges():
    follower = PathFollower([{'x': 0.0, 'y': 0.0}, {'x': 6.0, 'y': 0.0}], 0.5, 'once')
    track = simulate(follower, (0.0, 0.3), 12.0)
    after = [abs(y) for _, x, y in track if x >= 2.0 and x <= 5.5]
    assert after and max(after) < 0.02


def test_yaw_is_held_and_the_world_direction_is_kept():
    follower = PathFollower([{'x': 0.0, 'y': 0.0}, {'x': 4.0, 'y': 0.0}], 0.5, 'once')
    vx, vy, wz = follower.step(0.0, 0.0, 0.3)
    assert wz == pytest.approx(-0.6)
    assert math.hypot(vx, vy) == pytest.approx(0.5)
    world = math.atan2(vx * math.sin(0.3) + vy * math.cos(0.3),
                       vx * math.cos(0.3) - vy * math.sin(0.3))
    assert world == pytest.approx(0.0, abs=1e-9)


def test_trigger_waits_for_goal_and_distance_then_latches():
    trigger = StartTrigger({'goal': 2, 'robot_within': 2.0, 'of': {'x': 0.0, 'y': 0.0}})
    assert not trigger.update(1, (0.5, 0.0))
    assert not trigger.update(2, (3.0, 0.0))
    assert not trigger.update(2, None)
    assert trigger.update(2, (1.9, 0.0))
    assert trigger.update(2, (9.0, 0.0))
    assert not StartTrigger(None).update(0, None)
    assert StartTrigger(None).update(1, None)


def scenario(**changes):
    obstacle = {'name': 'w_1', 'shape': 'person', 'path': [{'x': 0, 'y': 0}, {'x': 1, 'y': 0}],
                'speed': 0.5, 'mode': 'once'}
    obstacle.update(changes)
    return {'goals': [{'x': 0, 'y': 0}, {'x': 1, 'y': 1}], 'obstacles': [obstacle]}


@pytest.mark.parametrize('changes, field', [
    ({'name': 'Bad Name'}, 'name'),
    ({'shape': 'dog'}, 'shape'),
    ({'mode': 'zigzag'}, 'mode'),
    ({'path': [{'x': 0, 'y': 0}]}, 'path'),
    ({'speed': 0.0}, 'speed'),
    ({'speed': 1.5}, 'speed'),
    ({'start': {'robot_within': 1.0}}, 'start'),
    ({'start': {'goal': 0}}, 'start.goal'),
    ({'start': {'goal': 3}}, 'start.goal'),
])
def test_validate_scenario_rejects(changes, field):
    with pytest.raises(ValueError) as error:
        validate_scenario('demo', scenario(**changes))
    assert 'demo' in str(error.value) and field in str(error.value)


def test_validate_scenario_rejects_duplicate_names_and_accepts_good_ones():
    good = scenario()
    validate_scenario('demo', good)
    good['obstacles'].append(dict(good['obstacles'][0]))
    with pytest.raises(ValueError, match='duplicate'):
        validate_scenario('demo', good)
    validate_scenario('static', {'goals': [{'x': 0, 'y': 0}]})


NAVIGATION = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', '..', 'rapid_nav_navigation'))
NEW_SCENARIOS = ('crossing', 'head_on', 'crowd')
OUTLINE_RADIUS = {'person': 0.20, 'cart': 0.36}


def load_world():
    """Return (scenarios, occupied cell centres of the arena map)."""
    with open(os.path.join(NAVIGATION, 'config', 'nav_scenarios.yaml'), encoding='utf-8') as f:
        scenarios = yaml.safe_load(f)
    with open(os.path.join(NAVIGATION, 'maps', 'arena.yaml'), encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    pixels = np.asarray(Image.open(os.path.join(NAVIGATION, 'maps', meta['image'])))
    grid = np.flipud(np.where(pixels < 89, 100, 0).astype(np.int8))
    cells = occupied_cells_xy(grid, float(meta['resolution']),
                              (float(meta['origin'][0]), float(meta['origin'][1])))
    return scenarios, np.asarray(cells)


def nearest_cell(cells, x, y):
    """Return the distance from (x, y) to the nearest occupied cell centre."""
    return float(np.min(np.hypot(cells[:, 0] - x, cells[:, 1] - y)))


def test_every_scenario_validates():
    scenarios, _ = load_world()
    for name, scenario in scenarios.items():
        validate_scenario(name, scenario)
    assert set(NEW_SCENARIOS) <= set(scenarios)


def test_obstacle_paths_keep_clear_of_the_map():
    scenarios, cells = load_world()
    for name in NEW_SCENARIOS:
        for obstacle in scenarios[name]['obstacles']:
            points = [(p['x'], p['y']) for p in obstacle['path']]
            if obstacle.get('mode') == 'loop':
                points.append(points[0])
            for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
                n = max(int(math.hypot(x1 - x0, y1 - y0) / 0.05), 1)
                for k in range(n + 1):
                    x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
                    gap = nearest_cell(cells, x, y) - OUTLINE_RADIUS[obstacle['shape']]
                    assert gap >= 0.30, (name, obstacle['name'], x, y, gap)


def test_new_goals_are_clear_of_the_map():
    scenarios, cells = load_world()
    for name in NEW_SCENARIOS:
        for goal in scenarios[name]['goals']:
            assert nearest_cell(cells, goal['x'], goal['y']) >= 0.6, (name, goal)


def moving_track(name, shape, x0, x1, y, seconds, step=0.1):
    times = [k * step for k in range(int(seconds / step) + 1)]
    xy = [[x0 + (x1 - x0) * min(t / seconds, 1.0), y] for t in times]
    return {'name': name, 'shape': shape, 'times': times, 'xy': xy,
            **track_spec({'name': name, 'shape': shape}, times, xy)}


def test_validity_rows_pass_for_a_scripted_run_and_fail_when_it_is_not():
    scenario = {
        'goals': [{'x': 0, 'y': 0}, {'x': 1, 'y': 1}],
        'obstacles': [{'name': 'walker_1', 'shape': 'person', 'speed': 0.5, 'mode': 'once',
                       'path': [{'x': 0.0, 'y': 2.0}, {'x': 4.0, 'y': 2.0}],
                       'start': {'goal': 2}}]}
    good = moving_track('walker_1', 'person', 0.0, 4.0, 2.0, 8.0)
    rows = validity_rows('crossing', scenario, {'walker_1': good}, [0.0, 8.0],
                         [[2.0, 0.0], [2.0, 0.0]], 1.2, 0.25)
    assert [r['verdict'] for r in rows] == ['PASS'] * 5
    slow = moving_track('walker_1', 'person', 0.0, 2.0, 2.3, 8.0)
    rows = validity_rows('crossing', scenario, {'walker_1': slow}, [0.0, 8.0],
                         [[2.0, 0.0], [2.0, 0.0]], 2.0, 0.25)
    assert [r['verdict'] for r in rows] == ['PASS', 'FAIL', 'PASS', 'FAIL', 'FAIL']


def test_sdf_is_heavy_and_hard_to_tip():
    root = ET.fromstring(obstacle_sdf('w', 'person'))
    inertial = root.find('model/link/inertial')
    assert float(inertial.find('mass').text) == 1000.0
    assert float(inertial.find('inertia/ixx').text) == 1.0e5
    assert float(inertial.find('inertia/iyy').text) == 1.0e5
    assert float(inertial.find('inertia/izz').text) == pytest.approx(1000.0 * 0.04 / 2.0)
    assert root.find('model/link/gravity').text == 'false'


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def test_body_command_level_yaw_and_tilted():
    assert body_command(0.5, 0.0, 0.0, 0.0, 0.0) == pytest.approx((0.5, 0.0, 0.0, 0, 0, 0))
    vx, vy, vz, wx, wy, wz = body_command(0.5, 0.0, 0.0, 0.0, 0.3)
    assert (vx, vy, vz) == pytest.approx((0.5 * math.cos(0.3), -0.5 * math.sin(0.3), 0.0))
    assert wz == pytest.approx(-0.6) and wx == 0.0 and wy == 0.0
    vx, vy, vz, wx, wy, wz = body_command(0.4, 0.3, 0.2, 0.0, 0.0)
    assert wx == pytest.approx(-0.4)
    assert abs(vz) > 0.0
    # back in the world, the command has no component into the floor
    world = [sum(rotation(0.2, 0.0, 0.0)[r][c] * (vx, vy, vz)[c] for c in range(3))
             for r in range(3)]
    assert world == pytest.approx([0.4, 0.3, 0.0])
    vx, vy, vz, wx, wy, wz = body_command(0.4, 0.3, 0.1, -0.15, 0.5)
    world = [sum(rotation(0.1, -0.15, 0.5)[r][c] * (vx, vy, vz)[c] for c in range(3))
             for r in range(3)]
    assert world == pytest.approx([0.4, 0.3, 0.0])
    assert (wx, wy) == pytest.approx((-0.2, 0.3))
