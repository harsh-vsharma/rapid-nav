"""Tests of rapid_nav_tools.scenarios, with hand-computed values."""

import xml.etree.ElementTree as ET

import pytest

from rapid_nav_tools.scenarios import (
    inertia,
    obstacle_sdf,
    obstacle_topics,
    spawn_height,
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
