"""Tests of rapid_nav_tools.scenarios, with hand-computed values."""

import math
import xml.etree.ElementTree as ET

import pytest
from rapid_nav_tools.scenarios import (
    inertia,
    obstacle_sdf,
    obstacle_topics,
    PathFollower,
    spawn_height,
    StartTrigger,
    validate_scenario,
)


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
    ixx, iyy, izz = inertia('person')
    assert ixx == pytest.approx(7.525)
    assert iyy == pytest.approx(7.525)
    assert izz == pytest.approx(0.6)
    # box 0.6 x 0.4 x 0.9: 30 * (0.16 + 0.81) / 12 = 2.425 ; 30 * (0.36 + 0.81) / 12 = 2.925 ;
    # 30 * (0.36 + 0.16) / 12 = 1.3
    ixx, iyy, izz = inertia('cart')
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
