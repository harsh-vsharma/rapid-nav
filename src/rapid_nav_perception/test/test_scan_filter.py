"""Tests of rapid_nav_perception.scan_filter."""

import math

import numpy as np
from rapid_nav_perception.scan_filter import (
    beam_mask, filter_scan, predict, removal_radius, select)

N = 360
ANGLE_MIN = -math.pi
INC = 2 * math.pi / N
IDENTITY = (np.eye(2), np.zeros(2))


def ray_scan(circles, wall=3.0):
    """Ranges of a lidar at the origin in a round room of radius `wall` with circles (x, y, r)."""
    out = np.full(N, wall)
    for i in range(N):
        a = ANGLE_MIN + INC * i
        d = np.array([math.cos(a), math.sin(a)])
        for cx, cy, r in circles:
            c = np.array([cx, cy])
            b = d @ c
            disc = b * b - (c @ c - r * r)
            if disc >= 0 and b - math.sqrt(disc) > 0:
                out[i] = min(out[i], b - math.sqrt(disc))
    return out


def track(x, y, vx=0.0, vy=0.0, radius=0.2, sp=0.05, sv=0.1):
    """Return a track dict."""
    return {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'radius': radius,
            'position_sigma': sp, 'velocity_sigma': sv}


def test_static_scene_is_unchanged():
    ranges = ray_scan([])
    out, mask = filter_scan(ranges, ANGLE_MIN, INC, 0.12, 12.0, [], 0.0, IDENTITY)
    assert not mask.any()
    assert np.allclose(out, ranges)


def test_beams_on_the_person_are_removed_and_the_wall_behind_is_kept():
    ranges = ray_scan([(1.5, 0.0, 0.2)])
    hit = ranges < 2.9
    assert hit.sum() > 3
    out, mask = filter_scan(ranges, ANGLE_MIN, INC, 0.12, 12.0, [track(1.5, 0.0)], 0.0, IDENTITY)
    assert np.array_equal(mask, hit)
    assert np.all(np.isnan(out[mask]))
    assert np.allclose(out[~mask], ranges[~mask])


def test_track_is_moved_to_the_scan_stamp():
    # The person is at (1.5, 0) now; the track was at (1.5, -0.5) 1 s before, walking +y.
    ranges = ray_scan([(1.5, 0.0, 0.2)])
    hit = ranges < 2.9
    moved = track(1.5, -0.5, vy=0.5, sv=0.0)
    _, mask = filter_scan(ranges, ANGLE_MIN, INC, 0.12, 12.0, [moved], 1.0, IDENTITY)
    assert np.array_equal(mask, hit)
    _, stale = filter_scan(ranges, ANGLE_MIN, INC, 0.12, 12.0, [moved], 0.0, IDENTITY)
    assert not np.array_equal(stale, hit)


def test_sigma_widens_the_disk_up_to_the_cap():
    small = removal_radius(0.2, 0.0)
    wide = removal_radius(0.2, 0.2)
    assert math.isclose(small, 0.3) and math.isclose(wide, 0.7)
    assert math.isclose(removal_radius(0.2, 5.0), 0.8)
    _, sig = predict([track(0, 0, sp=0.1, sv=0.3)], 2.0)
    assert math.isclose(sig[0], math.sqrt(0.01 + 0.36))


def test_map_to_laser_transform_is_applied():
    ranges = ray_scan([(1.5, 0.0, 0.2)])
    hit = ranges < 2.9
    # map = laser shifted by (+1, +2): the track at (2.5, 2) in map is (1.5, 0) in the laser.
    _, mask = filter_scan(ranges, ANGLE_MIN, INC, 0.12, 12.0, [track(2.5, 2.0)], 0.0,
                          (np.eye(2), np.array([-1.0, -2.0])))
    assert np.array_equal(mask, hit)


def test_invalid_beams_are_never_marked_and_keep_their_value():
    ranges = ray_scan([(1.0, 0.0, 0.2)])
    ranges[180] = np.inf
    ranges[181] = np.nan
    ranges[182] = 0.05
    mask = beam_mask(ranges, ANGLE_MIN, INC, 0.12, 12.0, [[0.0, 0.0]], [0.5])
    assert not mask[180] and not mask[181] and not mask[182]
    out, _ = filter_scan(ranges, ANGLE_MIN, INC, 0.12, 12.0, [track(1.0, 0.0)], 0.0, IDENTITY)
    assert np.isinf(out[180]) and np.isnan(out[181])


def test_select_drops_stale_and_slow_tracks():
    tracks = [track(0, 0, vx=0.5), track(1, 1)]
    assert select(tracks, 0.1) == tracks
    assert select(tracks, 0.6) == []
    assert select(tracks, None) == []
    assert select(tracks, 0.1, min_speed=0.2) == tracks[:1]
