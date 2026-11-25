"""Tests of rapid_nav_tools.map_quality."""

import math

import numpy as np
from rapid_nav_tools.map_quality import (
    boundary, dilate, evaluate, mover_mask, resample, scores, trajectory_error)

RES = 0.05
ORIGIN = (-2.0, -2.0)


def room(size=80, inner_wall=False):
    """Return a 4 m room, a one-cell wall all round and free inside (optional wall at x = 0)."""
    g = np.zeros((size, size), dtype=np.int16)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    if inner_wall:
        g[20:60, 40] = 100
    return g


def test_dilate_grows_by_the_reach():
    m = np.zeros((9, 9), dtype=bool)
    m[4, 4] = True
    assert dilate(m, 0).sum() == 1
    assert dilate(m, 1).sum() == 5
    assert dilate(m, 2)[4, 6] and not dilate(m, 2)[6, 6]


def test_perfect_map_scores_one():
    truth = room(inner_wall=True)
    out = scores(truth, truth, RES, origin=ORIGIN)
    assert out['precision'] == 1.0 and out['recall'] == 1.0 and out['f1'] == 1.0
    assert out['ghost_cells'] == 0 and out['mover_ghost_cells'] == 0


def test_unobserved_walls_do_not_lower_recall():
    truth = room(inner_wall=True)
    slam = truth.copy()
    slam[:, 50:] = -1  # the right part was never seen
    out = scores(slam, truth, RES, origin=ORIGIN)
    assert out['recall'] == 1.0 and out['precision'] == 1.0


def test_missing_wall_lowers_recall():
    truth = room(inner_wall=True)
    slam = truth.copy()
    slam[20:60, 40] = 0  # seen as free
    out = scores(slam, truth, RES, origin=ORIGIN)
    assert out['recall'] < 1.0 and out['precision'] == 1.0


def test_mover_trail_counts_as_mover_ghosts():
    truth = room()
    slam = truth.copy()
    slam[38:42, 30:50] = 100  # a smear along y = 0, x from -0.5 to 0.5
    slam[10, 10] = 100        # one ghost far from the mover
    track = {'xy': [(x, 0.0) for x in np.linspace(-0.6, 0.6, 25)], 'radius': 0.2}
    out = scores(slam, truth, RES, tracks=[track], origin=ORIGIN)
    assert out['ghost_cells'] == 81
    assert out['mover_ghost_cells'] == 80
    assert math.isclose(out['ghost_area_m2'], 81 * RES * RES)
    assert out['precision'] < 1.0


def test_resample_and_align_recover_a_shift():
    truth = room(inner_wall=True)
    # The SLAM map is the truth seen from a frame shifted by 0.20 m in x (beyond the tolerance).
    slam = truth.copy()
    shifted_origin = (ORIGIN[0] - 0.20, ORIGIN[1])
    out = evaluate(slam, shifted_origin, RES, truth, ORIGIN, RES, search=True)
    assert out['identity']['precision'] < 1.0
    assert math.isclose(out['alignment']['dx_m'], 0.20, abs_tol=1e-9)
    assert out['aligned']['precision'] == 1.0


def test_resample_outside_is_unknown():
    truth = room()
    small = np.zeros((10, 10), dtype=np.int16)
    out = resample(small, (0.0, 0.0), RES, truth.shape, ORIGIN, RES)
    assert (out == 0).sum() == 100 and (out == -1).sum() == truth.size - 100


def test_boundary_and_mover_mask():
    truth = room()
    edge = boundary(truth >= 65, truth == 0)
    assert edge[0, 1:-1].all() and not edge[0, 0]
    mask = mover_mask(truth.shape, ORIGIN, RES, [{'xy': [(0.0, 0.0)], 'radius': 0.1}], 0.0)
    assert 9 <= mask.sum() <= 16


def test_trajectory_error():
    truth = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), None]
    est = [(0.0, 0.3, 0.1), (1.0, -0.4, 0.0), (5.0, 5.0, 0.0)]
    out = trajectory_error(truth, est)
    assert out['samples'] == 2
    assert math.isclose(out['ate_rmse_m'], math.sqrt((0.09 + 0.16) / 2))
    assert math.isclose(out['max_m'], 0.4) and math.isclose(out['final_m'], 0.4)
    assert math.isclose(out['mean_yaw_rad'], 0.05)
    assert trajectory_error([], [])['ate_rmse_m'] is None
