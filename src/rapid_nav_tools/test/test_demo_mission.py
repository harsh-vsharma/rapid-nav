"""Tests of the demo's patrol bookkeeping."""

import math

import pytest
from rapid_nav_tools.demo_mission import (
    ACTIVE, CHECKPOINTS, Mission, parse_checkpoints, PENDING, REACHED, SKIPPED, yaw_quaternion)


def test_parse_checkpoints_groups_by_three_and_rejects_leftovers():
    assert parse_checkpoints([1, 2, 0.5, 3, 4, 0]) == [(1.0, 2.0, 0.5), (3.0, 4.0, 0.0)]
    assert len(parse_checkpoints(CHECKPOINTS)) == 6
    with pytest.raises(ValueError):
        parse_checkpoints([1, 2])


def test_mission_advances_retries_once_then_skips_and_loops():
    m = Mission(3)
    assert m.state == [ACTIVE, PENDING, PENDING]
    m.finished(True)
    assert (m.index, m.state[:2]) == (1, [REACHED, ACTIVE])
    m.finished(False)  # first failure: retry the same checkpoint
    assert m.index == 1 and m.state[1] == ACTIVE
    m.finished(False)  # second: skip it
    assert m.index == 2 and m.state[1] == SKIPPED
    m.finished(True)   # last checkpoint: a new lap from the first
    assert (m.index, m.lap, m.state) == (0, 2, [ACTIVE, PENDING, PENDING])


def test_yaw_quaternion_half_turn():
    z, w = yaw_quaternion(math.pi)
    assert z == pytest.approx(1.0) and w == pytest.approx(0.0, abs=1e-12)


def test_node_constructs():
    """The node builds (its attribute names must not clash with rclpy's Node)."""
    import rclpy
    from rapid_nav_tools.demo_mission import DemoMission
    rclpy.init()
    try:
        node = DemoMission()
        assert len(node.checkpoints) == 6
        node.destroy_node()
    finally:
        rclpy.shutdown()
