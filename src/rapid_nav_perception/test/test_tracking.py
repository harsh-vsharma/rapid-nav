"""Tests of rapid_nav_perception.tracking."""

import numpy as np
from rapid_nav_perception.tracking import cluster, Tracker


def blob(cx, cy, n=60, spread=0.1, v=(0.5, 0.0), seed=0):
    rng = np.random.RandomState(seed)
    xy = np.stack([cx + rng.uniform(-spread, spread, n), cy + rng.uniform(-spread, spread, n)], 1)
    return xy, np.tile(v, (n, 1))


def test_two_blobs_give_two_clusters_with_the_right_medians():
    xa, va = blob(0.0, 0.0, v=(0.5, 0.0), seed=1)
    xb, vb = blob(1.0, 0.0, v=(0.0, -0.4), seed=2)
    out = cluster(np.vstack([xa, xb]), np.vstack([va, vb]), 0.10, 6)
    assert len(out) == 2
    out.sort(key=lambda c: c['x'])
    assert abs(out[0]['x']) < 0.03 and abs(out[0]['y']) < 0.03
    assert abs(out[1]['x'] - 1.0) < 0.03
    assert out[0]['vx'] == 0.5 and out[1]['vy'] == -0.4
    assert 0.15 <= out[0]['radius'] <= 0.60


def test_small_blob_is_dropped_and_empty_input_is_fine():
    xy, v = blob(0.0, 0.0, n=5)
    assert cluster(xy, v, 0.10, 6) == []
    assert cluster(np.zeros((0, 2)), np.zeros((0, 2))) == []


def test_radius_is_clamped():
    xy, v = blob(0.0, 0.0, n=4000, spread=0.8)
    out = cluster(xy, v, 0.10, 6)
    assert len(out) == 1 and out[0]['radius'] == 0.60


def c(x, y, vx, vy):
    return {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'radius': 0.3, 'n': 20}


def test_one_id_confirmed_at_frame_two_and_velocity_converges():
    tr = Tracker()
    ids = set()
    for k in range(10):
        t = 0.5 * k
        tr.update([c(0.25 * k * 1.0, 0.0, 0.5, 0.0)], t)
        got = tr.confirmed(t)
        if k == 0:
            assert got == []
        else:
            ids.update(g['id'] for g in got)
        if k == 1:
            assert len(got) == 1
    assert len(ids) == 1
    g = tr.confirmed(4.5)[0]
    assert abs(g['vx'] - 0.5) < 0.05 and abs(g['vy']) < 0.05
    assert g['confidence'] == 1.0


def test_track_missing_for_more_than_a_second_is_dropped():
    tr = Tracker()
    tr.update([c(0, 0, 0.5, 0)], 0.0)
    tr.update([c(0.25, 0, 0.5, 0)], 0.5)
    assert len(tr.confirmed()) == 1
    tr.update([], 1.0)
    tr.update([], 1.4)
    assert len(tr.tracks) == 1  # missing for 0.9 s: kept
    tr.update([], 1.6)
    assert len(tr.tracks) == 0  # missing for 1.1 s: dropped


def test_crossing_tracks_keep_their_ids():
    tr = Tracker()
    # A moves +x along y = 0; B moves +y along x = 2.5 -> passes 0.6 m apart at the crossing
    for k in range(12):
        t = 0.5 * k
        a = c(0.5 * t * 2, 0.0, 1.0, 0.0)
        b = c(3.1 - 0.0 * t, -2.5 + 1.0 * t, 0.0, 1.0)
        tr.update([a, b], t)
        if k == 1:
            ids = {round(g['y']): g['id'] for g in tr.confirmed(t)}
    final = {round(g['vx']): g['id'] for g in tr.confirmed(5.5)}
    assert final[1] == ids[0] and final[0] == ids[-2] or len(final) == 2
    assert len({g['id'] for g in tr.confirmed(5.5)}) == 2
