"""Tests of the RViz geometry: lidar outline, bodies by velocity gradient, convex hulls."""

import math

import numpy as np
from rapid_nav_perception.scene_vis import (
    body_hull, confirm_movers, depth_edges, depth_jumps, index_grid, instance_motion,
    merge_instances, scan_segments, segment_instances, stable_ids, tile_indices)


def test_scan_segments_join_close_beams_only():
    ranges = [1.0, 1.0, float('inf'), 1.0, 3.0]
    segs = scan_segments(ranges, 0.0, 0.05, 0.1, 10.0, 0.2)
    assert segs.shape == (1, 2, 2)  # 0-1 joined; 2 invalid; 3-4 too far apart
    assert np.allclose(segs[0, 0], [1.0, 0.0])


def test_scan_segments_wrap():
    n = 100
    segs = scan_segments([2.0] * n, 0.0, 2 * math.pi / n, 0.1, 10.0, 0.2, wrap=True)
    assert segs.shape[0] == n


def make_scene():
    """Return a 10 x 10 grid: left half static, right half moving +x and spinning."""
    rows, cols = np.mgrid[0:10, 0:10]
    rc = np.stack([rows.ravel(), cols.ravel()], axis=1)
    pts = np.stack([cols.ravel() * 0.05, rows.ravel() * 0.05, np.full(100, 0.5)], axis=1)
    vel = np.zeros_like(pts)
    right = rc[:, 1] >= 5
    centre = pts[right].mean(axis=0)
    omega = 0.5
    r = pts[right] - centre
    vel[right, 0] = 1.0 - omega * r[:, 1]
    vel[right, 1] = omega * r[:, 0]
    return rc, pts, vel, right, omega


def test_instances_split_by_velocity_and_carry_mean_and_rotation():
    rc, pts, vel, right, omega = make_scene()
    grid = index_grid(rc, (10, 10))
    labels = segment_instances(grid, pts, vel, 0.25, 0.25)
    assert len(set(labels[right])) == 1 and len(set(labels[~right])) == 1
    assert labels[right][0] != labels[~right][0]
    insts = {i['label']: i for i in instance_motion(pts, vel, labels, 5)}
    mover = insts[labels[right][0]]
    assert np.allclose(mover['velocity'][:2], [1.0, 0.0], atol=1e-9)
    assert math.isclose(mover['omega'], omega, rel_tol=1e-9)
    assert np.allclose(insts[labels[~right][0]]['velocity'], 0.0)


def test_tile_indices_inverse_of_block_centres():
    block = 4
    rows, cols = np.array([0, 3]), np.array([2, 5])
    pix = np.stack([(cols + 0.5) * block - 0.5, (rows + 0.5) * block - 0.5], axis=1)
    assert np.array_equal(tile_indices(pix, block), np.stack([rows, cols], axis=1))


def test_confirm_movers_needs_a_consistent_previous_mover():
    def inst(x, vx):
        return {'centroid': np.array([x, 0.0, 0.5]), 'velocity': np.array([vx, 0.0, 0.0])}
    previous = [inst(1.0, 0.5)]
    assert confirm_movers([inst(1.05, 0.5)], previous, 0.1) == [True]
    assert confirm_movers([inst(1.05, -0.5)], previous, 0.1) == [False]  # velocity flipped
    assert confirm_movers([inst(3.0, 0.5)], previous, 0.1) == [False]    # elsewhere
    assert confirm_movers([inst(1.0, 0.5)], [], 0.1) == [False]


def test_depth_edges_marks_jumps_and_borders():
    grid = index_grid(np.array([[r, c] for r in range(3) for c in range(4)]), (3, 4))
    depth = np.array([1.0, 1.0, 3.0, 3.0] * 3)
    edge = depth_edges(grid, depth)
    assert edge[grid[1, 1]] and edge[grid[1, 2]]
    big = index_grid(np.array([[r, c] for r in range(5) for c in range(5)]), (5, 5))
    assert not depth_edges(big, np.ones(25))[big[2, 2]]


def test_merge_instances_joins_close_alike_pieces_only():
    def inst(label, x, vx, n=10):
        return {'label': label, 'n': n, 'centroid': np.array([x, 0.0, 0.5]),
                'velocity': np.array([vx, 0.0, 0.0]), 'omega': 0.0, 'size': 0.1}
    merged = merge_instances([inst(0, 0.0, 0.5), inst(1, 0.4, 0.55), inst(2, 0.8, 0.5),
                              inst(3, 0.2, -0.5), inst(4, 5.0, 0.5)])
    groups = sorted(sorted(m['labels']) for m in merged)
    assert groups == [[0, 1, 2], [3], [4]]
    big = [m for m in merged if len(m['labels']) == 3][0]
    assert big['n'] == 30 and math.isclose(big['centroid'][0], 0.4)


def test_depth_jumps_flags_occlusions_only():
    z0 = np.array([2.0, 2.0, 1.0])
    z1 = np.array([2.02, 3.0, 1.04])
    assert list(depth_jumps(z0, z1)) == [False, True, False]


def test_stable_ids_keep_a_body_colour_across_frames():
    ids, nxt = stable_ids([np.array([0.0, 0.0, 0.5]), np.array([3.0, 0.0, 0.5])], [], 0.6, 0)
    assert ids == [0, 1] and nxt == 2
    previous = [(np.array([0.0, 0.0, 0.5]), 0), (np.array([3.0, 0.0, 0.5]), 1)]
    # The labels came out in the other order and both moved a little; a third body is new.
    ids, nxt = stable_ids([np.array([3.1, 0.1, 0.5]), np.array([0.2, 0.0, 0.5]),
                           np.array([9.0, 9.0, 0.5])], previous, 0.6, nxt)
    assert ids == [1, 0, 2] and nxt == 3


def test_body_hull_wraps_a_box_and_skips_walls_and_scraps():
    corners = np.array([[x, y, z] for x in (0, 0.5) for y in (0, 0.5) for z in (0, 1.0)],
                       dtype=float)
    inside = np.array([[0.25, 0.25, 0.5], [0.1, 0.2, 0.3]])
    pts = np.vstack([corners, inside])
    tris = body_hull(pts)
    assert tris.shape[1] == 3 and len(tris) >= 12          # a box has 12 hull triangles
    assert not np.isin([8, 9], tris).any()                 # interior points are not on it
    wall = np.array([[x, 0.0, z] for x in np.linspace(0, 4, 9) for z in (0, 1)], dtype=float)
    assert body_hull(wall).size == 0                       # 4 m wide: no hull
    assert body_hull(corners[:3]).size == 0                # too few points
