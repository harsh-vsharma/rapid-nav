"""Tests of rapid_nav_tools.track_metrics, with hand-computed values."""

import math

import numpy as np
import pytest
from rapid_nav_tools.track_metrics import (
    angle_between_deg,
    in_view,
    interpolate_xy,
    nearest_within,
    score,
    velocity_xy,
    yaw_rate,
)

CAMERA = (0.0, 0.0, 0.0)


def test_in_view_range_and_angle():
    assert in_view((0, 0), 0.0, CAMERA, (2.0, 0.0))
    assert not in_view((0, 0), 0.0, CAMERA, (0.2, 0.0))      # too near
    assert not in_view((0, 0), 0.0, CAMERA, (6.5, 0.0))      # too far
    assert in_view((0, 0), 0.0, CAMERA, (2.0, 1.3))          # 33 degrees left of the axis
    assert not in_view((0, 0), 0.0, CAMERA, (2.0, 1.5))      # 36.9 degrees: outside 34.5
    assert not in_view((0, 0), 0.0, CAMERA, (-2.0, 0.0))     # behind
    assert in_view((1, 1), math.pi / 2, CAMERA, (1.0, 3.0))  # robot turned to +y


def test_camera_offset():
    # a camera 0.2 m ahead of the robot: a point 0.4 m ahead of the robot is 0.2 m from the camera
    assert not in_view((0, 0), 0.0, (0.2, 0.0, 0.0), (0.4, 0.0))
    assert in_view((0, 0), 0.0, (0.2, 0.0, 0.0), (0.6, 0.0))


def test_interpolation_and_velocity():
    t = [0.0, 1.0, 2.0]
    xy = [[0, 0], [1, 0], [2, 0]]
    assert interpolate_xy(t, xy, 0.5) == pytest.approx((0.5, 0.0))
    assert interpolate_xy(t, xy, 5.0) == pytest.approx((2.0, 0.0))
    assert velocity_xy(t, xy, 1.0) == pytest.approx((1.0, 0.0))
    assert interpolate_xy([], [], 0.0) is None


def test_nearest_and_angle():
    assert nearest_within((0, 0), [(1, 0), (0.3, 0.1)]) == (1, pytest.approx(math.hypot(0.3, 0.1)))
    assert nearest_within((0, 0), [(1, 0)]) == (None, None)
    assert angle_between_deg((1, 0), (0, 1)) == pytest.approx(90.0)
    assert math.isnan(angle_between_deg((0, 0), (1, 0)))


def messages_for(offset=(0.0, 0.0), v=(0.5, 0.0), false_track=False):
    out = []
    for k in range(20):
        t = 0.2 * k
        tracks = [{'id': 'a', 'x': 3.0 + 0.5 * t + offset[0], 'y': offset[1],
                   'vx': v[0], 'vy': v[1]}]
        if false_track:
            tracks.append({'id': 'f', 'x': 1.0, 'y': 5.0, 'vx': 0.0, 'vy': 0.0})
        out.append({'t': t, 'recv': t + 0.3, 'tracks': tracks})
    return out


def run(messages):
    times = np.arange(0, 5.0, 0.1)
    truth = {'w': (times, np.stack([3.0 + 0.5 * times, np.zeros_like(times)], 1))}
    robot = (times, np.zeros((len(times), 2)), np.zeros(len(times)))
    return score(messages, truth, robot, CAMERA, 4.0)


def test_perfect_tracks():
    r = run(messages_for())
    assert r['detection_rate'] == 1.0
    assert r['position_error_median_m'] == pytest.approx(0.0, abs=1e-9)
    assert r['speed_error_median_mps'] == pytest.approx(0.0, abs=1e-6)
    assert r['heading_error_median_deg'] == pytest.approx(0.0, abs=1e-3)
    assert r['false_tracks'] == 0
    assert r['latency_sim_median_s'] == pytest.approx(0.3)


def test_offset_and_wrong_velocity_and_false_track():
    r = run(messages_for(offset=(0.2, 0.0), v=(0.4, 0.1), false_track=True))
    assert r['detection_rate'] == 1.0
    assert r['position_error_median_m'] == pytest.approx(0.2)
    assert r['speed_error_median_mps'] == pytest.approx(abs(math.hypot(0.4, 0.1) - 0.5), abs=1e-6)
    assert r['heading_error_median_deg'] == pytest.approx(
        math.degrees(math.atan2(0.1, 0.4)), abs=0.01)
    assert r['false_tracks'] == 1
    assert r['false_tracks_per_s'] == pytest.approx(0.25)


def test_missed_obstacle_and_out_of_view():
    msgs = [{'t': 0.2 * k, 'recv': 0.2 * k, 'tracks': []} for k in range(20)]
    r = run(msgs)
    assert r['detection_rate'] == 0.0 and r['position_error_median_m'] is None
    assert r['in_view_samples'] == 20


def test_yaw_rate_and_the_share_of_false_tracks_while_turning():
    times = np.arange(0, 5.0, 0.1)
    assert yaw_rate(times, 0.5 * times, 2.0) == pytest.approx(0.5)
    assert yaw_rate(times, np.zeros(len(times)), 2.0) == pytest.approx(0.0)
    truth = {'w': (times, np.stack([3.0 + 0.5 * times, np.zeros_like(times)], 1))}
    # the robot turns at 0.5 rad/s during the first half only
    yaw = np.where(times < 2.5, 0.5 * times, 1.25)
    robot = (times, np.zeros((len(times), 2)), yaw)
    r = score(messages_for(false_track=True), truth, robot, CAMERA, 4.0)
    assert r['false_per_message_mean'] == pytest.approx(1.0)
    assert 0.4 < r['false_turning_fraction'] < 0.7
