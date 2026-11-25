"""Tests of rapid_nav_perception.fusion."""

import math

import numpy as np
from rapid_nav_perception.fusion import Fusion, lidar_clusters, StaticGrid


def room_grid():
    """Return a 4 m x 4 m room (0.05 m cells) from (-2, -2) with a one-cell wall all around."""
    occupied = np.zeros((80, 80), dtype=bool)
    occupied[0, :] = occupied[-1, :] = occupied[:, 0] = occupied[:, -1] = True
    return StaticGrid(occupied, -2.0, -2.0, 0.05)


def wall_points():
    """Scan points on the inside faces of the room's walls."""
    s = np.linspace(-1.9, 1.9, 77)
    edge = np.full_like(s, 1.94)
    return np.vstack([np.stack([s, edge], 1), np.stack([s, -edge], 1),
                      np.stack([edge, s], 1), np.stack([-edge, s], 1)])


def arc_points(cx, cy, radius=0.25, n=8):
    """Points on the half of a circle that faces the origin (what a lidar there sees)."""
    facing = math.atan2(-cy, -cx)
    a = facing + np.linspace(-1.2, 1.2, n)
    return np.stack([cx + radius * np.cos(a), cy + radius * np.sin(a)], 1)


def lidar_only(fusion, steps, noise=0.03, seed=0, v=(0.5, 0.0), start=(-1.5, 0.5)):
    """Feed lidar clusters of one mover at 10 Hz for `steps` steps; return the end time."""
    rng = np.random.RandomState(seed)
    t = 0.0
    for k in range(steps):
        t = 0.1 * k
        x, y = start[0] + v[0] * t, start[1] + v[1] * t
        fusion.update_lidar([{'x': x + rng.normal(0, noise), 'y': y + rng.normal(0, noise),
                              'radius': 0.25}], t)
    return t


def test_static_scene_gives_no_clusters_and_no_tracks():
    grid = room_grid()
    assert lidar_clusters(wall_points(), grid) == []
    fusion = Fusion()
    for k in range(20):
        fusion.update_lidar(lidar_clusters(wall_points(), grid), 0.1 * k)
    assert fusion.tracks == {} and fusion.confirmed(2.0) == []


def test_mover_gives_one_cluster_and_small_groups_are_dropped():
    grid = room_grid()
    points = np.vstack([wall_points(), arc_points(1.0, 0.5), [[-1.0, -1.0], [-1.04, -1.0]]])
    out = lidar_clusters(points, grid)
    assert len(out) == 1
    # The mean of the facing arc lies between the centre and the sensor.
    assert math.hypot(out[0]['x'] - 1.0, out[0]['y'] - 0.5) < 0.2
    assert out[0]['radius'] >= 0.15 and out[0]['n'] == 8
    assert len(lidar_clusters(points, grid, min_points=2)) == 2


def test_lidar_only_converges_to_the_velocity():
    fusion = Fusion()
    t = lidar_only(fusion, 31)
    (track,) = fusion.confirmed(t)
    assert math.hypot(track['vx'] - 0.5, track['vy']) < 0.1
    assert abs(track['x'] - (-1.5 + 0.5 * t)) < 0.1


def test_good_flow_shrinks_the_velocity_sigma_faster():
    lidar = Fusion()
    t = lidar_only(lidar, 10)
    both = Fusion()
    rng = np.random.RandomState(0)
    for k in range(10):
        tk = 0.1 * k
        x, y = -1.5 + 0.5 * tk, 0.5
        both.update_lidar([{'x': x + rng.normal(0, 0.03), 'y': y + rng.normal(0, 0.03),
                            'radius': 0.25}], tk)
        # The camera is 0.1 s late: its track is valid at tk - 0.1.
        if k >= 1:
            both.update_flow([{'x': x - 0.05, 'y': y, 'vx': 0.5, 'vy': 0.0,
                               'position_sigma': 0.08, 'velocity_sigma': 0.1}], tk - 0.1)
    (a,) = lidar.confirmed(t)
    (b,) = both.confirmed(t)
    # The late camera's velocity is worth less (process noise over the 0.1 s lag), still clearly
    # better than the lidar alone after 1 s.
    assert b['velocity_sigma'] < a['velocity_sigma'] - 0.05
    assert b['confidence'] > a['confidence']
    assert math.hypot(b['vx'] - 0.5, b['vy']) < 0.1


def test_lidar_only_sigma_stays_larger_than_with_flow_after_three_seconds():
    lidar = Fusion()
    t = lidar_only(lidar, 31)
    both = Fusion()
    lidar_only(both, 1)
    for k in range(1, 31):
        tk = 0.1 * k
        both.update_lidar([{'x': -1.5 + 0.5 * tk, 'y': 0.5, 'radius': 0.25}], tk)
        both.update_flow([{'x': -1.5 + 0.5 * tk, 'y': 0.5, 'vx': 0.5, 'vy': 0.0,
                           'position_sigma': 0.08, 'velocity_sigma': 0.1}], tk)
    assert lidar.confirmed(t)[0]['velocity_sigma'] > both.confirmed(t)[0]['velocity_sigma']


def test_flow_outlier_with_large_sigma_barely_moves_the_estimate():
    fusion = Fusion()
    t = lidar_only(fusion, 20, noise=0.0)
    before = fusion.confirmed(t)[0]
    fusion.update_flow([{'x': before['x'], 'y': before['y'], 'vx': 3.0, 'vy': -3.0,
                         'position_sigma': 0.5, 'velocity_sigma': 5.0}], t)
    after = fusion.confirmed(t)[0]
    assert math.hypot(after['vx'] - before['vx'], after['vy'] - before['vy']) < 0.05
    # The same velocity with a small sigma does move it.
    fusion.update_flow([{'x': before['x'], 'y': before['y'], 'vx': 3.0, 'vy': -3.0,
                         'position_sigma': 0.05, 'velocity_sigma': 0.1}], t)
    moved = fusion.confirmed(t)[0]
    assert math.hypot(moved['vx'] - before['vx'], moved['vy'] - before['vy']) > 1.0


def test_flow_starts_no_track_and_unmatched_tracks_are_dropped():
    fusion = Fusion()
    fusion.update_flow([{'x': 0.0, 'y': 0.0, 'vx': 0.5, 'vy': 0.0, 'position_sigma': 0.1,
                         'velocity_sigma': 0.1}], 0.0)
    assert fusion.tracks == {}
    t = lidar_only(fusion, 5)
    assert len(fusion.confirmed(t)) == 1
    fusion.update_lidar([], t + 1.05)
    assert fusion.tracks == {}


def test_confirmation_needs_three_updates():
    fusion = Fusion()
    lidar_only(fusion, 2)
    assert fusion.confirmed(0.1) == []
    fusion.update_lidar([{'x': -1.4, 'y': 0.5, 'radius': 0.25}], 0.2)
    assert len(fusion.confirmed(0.2)) == 1


def test_two_crossing_movers_keep_their_ids():
    fusion = Fusion()
    rng = np.random.RandomState(4)
    ids = None
    for k in range(81):
        t = 0.1 * k
        a = (-2.0 + 0.5 * t, 0.0)      # along y = 0, +x
        b = (0.0, -2.4 + 0.5 * t)      # along x = 0, +y: 0.4 m behind a at the crossing
        clusters = [{'x': p[0] + rng.normal(0, 0.02), 'y': p[1] + rng.normal(0, 0.02),
                     'radius': 0.2} for p in (a, b)]
        fusion.update_lidar(clusters, t)
        tracks = fusion.confirmed(t)
        if k == 10:
            ids = {('a' if tr['vx'] > tr['vy'] else 'b'): tr['id'] for tr in tracks}
            assert len(ids) == 2
        if k > 10:
            assert len(tracks) == 2
            for tr in tracks:
                expected = a if tr['id'] == ids['a'] else b
                assert math.hypot(tr['x'] - expected[0], tr['y'] - expected[1]) < 0.15
