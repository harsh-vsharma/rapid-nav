"""
The quality of a SLAM map against the ground-truth arena map, and the ghost walls of movers.

Both maps are occupancy grids (rows = y, columns = x, values -1 unknown, 0..100) with an origin
(the world position of the lower-left corner of cell (0, 0), no rotation) and a resolution. The
SLAM map is resampled onto the ground-truth grid, optionally after a small rigid alignment that
separates the drift of the map frame from the content of the map.

Measures, with a tolerance `tol` (m):

- precision: the share of the SLAM map's occupied cells within tol of a true occupied cell;
- recall: the share of the observable true boundary cells (occupied, with a 4-neighbour that is
  free in truth and known to SLAM, so walls SLAM never faced do not count) that have a SLAM
  occupied cell within tol;
- ghost cells: the SLAM occupied cells farther than tol from every true occupied cell;
- mover ghost cells: the ghost cells within (obstacle radius + `mover_margin`) of the true track
  of a moving obstacle: what the moving obstacles left in the map.

Pure numpy, no ROS imports.
"""

import math

import numpy as np

OCCUPIED = 65
FREE = 25
TOLERANCE_M = 0.10
MOVER_MARGIN_M = 0.15
SEARCH_XY_M = 0.20
SEARCH_XY_STEP_M = 0.05
SEARCH_YAW_DEG = 3.0
SEARCH_YAW_STEP_DEG = 0.5


def dilate(mask, reach):
    """Return the mask grown by a disk of `reach` cells (8-connected shifts, no wrap-around)."""
    mask = np.asarray(mask, dtype=bool)
    out = mask.copy()
    height, width = mask.shape
    reach = int(reach)
    for di in range(-reach, reach + 1):
        for dj in range(-reach, reach + 1):
            if (di == 0 and dj == 0) or di * di + dj * dj > reach * reach:
                continue
            src = mask[max(0, -dj):height - max(0, dj), max(0, -di):width - max(0, di)]
            out[max(0, dj):height - max(0, -dj), max(0, di):width - max(0, -di)] |= src
    return out


def cell_centres(shape, origin, resolution):
    """Return the world (x, y) of every cell centre as two arrays of the grid's shape."""
    height, width = shape
    xs = origin[0] + (np.arange(width) + 0.5) * resolution
    ys = origin[1] + (np.arange(height) + 0.5) * resolution
    return np.meshgrid(xs, ys)


def resample(grid, origin, resolution, target_shape, target_origin, target_resolution,
             dx=0.0, dy=0.0, dyaw=0.0):
    """
    Return the grid's values at the target grid's cell centres (nearest cell, -1 outside).

    (dx, dy, dyaw) moves the source map before the lookup: a target point p takes the value of
    the source at R(-dyaw) (p - (dx, dy)).
    """
    grid = np.asarray(grid)
    gx, gy = cell_centres(target_shape, target_origin, target_resolution)
    c, s = math.cos(-dyaw), math.sin(-dyaw)
    px, py = gx - dx, gy - dy
    sx, sy = c * px - s * py, s * px + c * py
    i = np.floor((sx - origin[0]) / resolution).astype(int)
    j = np.floor((sy - origin[1]) / resolution).astype(int)
    inside = (i >= 0) & (i < grid.shape[1]) & (j >= 0) & (j < grid.shape[0])
    out = np.full(target_shape, -1, dtype=np.int16)
    out[inside] = grid[j[inside], i[inside]]
    return out


def boundary(occupied, free):
    """Return the occupied cells that have a free 4-neighbour."""
    occupied = np.asarray(occupied, dtype=bool)
    free = np.asarray(free, dtype=bool)
    near_free = np.zeros_like(free)
    near_free[1:, :] |= free[:-1, :]
    near_free[:-1, :] |= free[1:, :]
    near_free[:, 1:] |= free[:, :-1]
    near_free[:, :-1] |= free[:, 1:]
    return occupied & near_free


def mover_mask(shape, origin, resolution, tracks, margin=MOVER_MARGIN_M):
    """
    Return the cells within (radius + margin) of any point of the moving obstacles' tracks.

    `tracks` is a list of dicts with 'xy' (N, 2) and 'radius' (m).
    """
    out = np.zeros(shape, dtype=bool)
    gx, gy = cell_centres(shape, origin, resolution)
    for track in tracks:
        xy = np.asarray(track['xy'], dtype=np.float64).reshape(-1, 2)
        reach = float(track['radius']) + margin
        # thin the track to about one point per cell: the disks overlap and cover it
        last = None
        for x, y in xy:
            if last is not None and math.hypot(x - last[0], y - last[1]) < 0.5 * resolution:
                continue
            last = (x, y)
            i0 = int(math.floor((x - reach - origin[0]) / resolution))
            i1 = int(math.floor((x + reach - origin[0]) / resolution)) + 1
            j0 = int(math.floor((y - reach - origin[1]) / resolution))
            j1 = int(math.floor((y + reach - origin[1]) / resolution)) + 1
            i0, j0 = max(i0, 0), max(j0, 0)
            i1, j1 = min(i1, shape[1]), min(j1, shape[0])
            if i0 >= i1 or j0 >= j1:
                continue
            sub = (gx[j0:j1, i0:i1] - x) ** 2 + (gy[j0:j1, i0:i1] - y) ** 2 <= reach * reach
            out[j0:j1, i0:i1] |= sub
    return out


def scores(slam_on_truth, truth, resolution, tracks=(), tol=TOLERANCE_M,
           mover_margin=MOVER_MARGIN_M, origin=(0.0, 0.0)):
    """
    Return the measures of a SLAM map already resampled onto the truth grid.

    Keys: precision, recall, f1 (None when undefined), slam_occupied, truth_boundary_observed,
    matched, ghost_cells, mover_ghost_cells, ghost_area_m2, mover_ghost_area_m2, known_cells.
    """
    slam_on_truth = np.asarray(slam_on_truth)
    truth = np.asarray(truth)
    reach = int(round(tol / resolution))
    slam_occ = slam_on_truth >= OCCUPIED
    slam_known = slam_on_truth >= 0
    true_occ = truth >= OCCUPIED
    true_free = (truth >= 0) & (truth < FREE)
    near_true = dilate(true_occ, reach)
    matched = slam_occ & near_true
    ghosts = slam_occ & ~near_true
    edge = boundary(true_occ, true_free & slam_known)
    found = edge & dilate(slam_occ, reach)
    n_slam = int(slam_occ.sum())
    n_edge = int(edge.sum())
    precision = float(matched.sum()) / n_slam if n_slam else None
    recall = float(found.sum()) / n_edge if n_edge else None
    f1 = None
    if precision is not None and recall is not None and precision + recall > 0:
        f1 = 2 * precision * recall / (precision + recall)
    movers = mover_mask(truth.shape, origin, resolution, tracks, mover_margin)
    mover_ghosts = ghosts & movers
    cell_area = resolution * resolution
    return {
        'precision': precision, 'recall': recall, 'f1': f1,
        'slam_occupied': n_slam, 'truth_boundary_observed': n_edge,
        'matched': int(matched.sum()), 'ghost_cells': int(ghosts.sum()),
        'mover_ghost_cells': int(mover_ghosts.sum()),
        'ghost_area_m2': float(ghosts.sum()) * cell_area,
        'mover_ghost_area_m2': float(mover_ghosts.sum()) * cell_area,
        'known_cells': int(slam_known.sum()),
    }


def align(slam, slam_origin, slam_resolution, truth, truth_origin, truth_resolution,
          xy=SEARCH_XY_M, xy_step=SEARCH_XY_STEP_M, yaw_deg=SEARCH_YAW_DEG,
          yaw_step_deg=SEARCH_YAW_STEP_DEG, tol=TOLERANCE_M):
    """
    Return (dx, dy, dyaw) that maximises the matched SLAM occupied cells (precision * count).

    A brute-force search over the small window; ties go to the smallest motion. Without any
    SLAM occupied cell the identity is returned.
    """
    true_near = dilate(np.asarray(truth) >= OCCUPIED, int(round(tol / truth_resolution)))
    steps = int(round(xy / xy_step))
    yaw_steps = int(round(yaw_deg / yaw_step_deg))
    best = (-1, 0.0, (0.0, 0.0, 0.0))
    for k in range(-yaw_steps, yaw_steps + 1):
        dyaw = math.radians(k * yaw_step_deg)
        for a in range(-steps, steps + 1):
            for b in range(-steps, steps + 1):
                dx, dy = a * xy_step, b * xy_step
                moved = resample(slam, slam_origin, slam_resolution, truth.shape, truth_origin,
                                 truth_resolution, dx, dy, dyaw)
                hits = int(np.count_nonzero((moved >= OCCUPIED) & true_near))
                size = math.hypot(dx, dy) + abs(dyaw)
                if hits > best[0] or (hits == best[0] and size < best[1]):
                    best = (hits, size, (dx, dy, dyaw))
    return best[2]


def evaluate(slam, slam_origin, slam_resolution, truth, truth_origin, truth_resolution,
             tracks=(), tol=TOLERANCE_M, search=True):
    """
    Return {'identity': scores, 'aligned': scores, 'alignment': {dx, dy, dyaw}}.

    'identity' takes the map frame as the arena frame; 'aligned' first applies the best small
    rigid motion (the identity when search is False).
    """
    truth = np.asarray(truth)
    identity = resample(slam, slam_origin, slam_resolution, truth.shape, truth_origin,
                        truth_resolution)
    out = {'identity': scores(identity, truth, truth_resolution, tracks, tol,
                              origin=truth_origin)}
    dx, dy, dyaw = (align(slam, slam_origin, slam_resolution, truth, truth_origin,
                          truth_resolution, tol=tol) if search else (0.0, 0.0, 0.0))
    moved = resample(slam, slam_origin, slam_resolution, truth.shape, truth_origin,
                     truth_resolution, dx, dy, dyaw)
    out['aligned'] = scores(moved, truth, truth_resolution, tracks, tol, origin=truth_origin)
    out['alignment'] = {'dx_m': dx, 'dy_m': dy, 'dyaw_rad': dyaw}
    return out


def trajectory_error(truth, estimate):
    """
    Return the error of an estimated planar trajectory against the truth at the same samples.

    Both are lists of (x, y, yaw) or None (dropped in pairs). Keys: samples, ate_rmse_m, max_m,
    final_m, mean_yaw_rad (None without samples).
    """
    pairs = [(t, e) for t, e in zip(truth, estimate) if t is not None and e is not None]
    if not pairs:
        return {'samples': 0, 'ate_rmse_m': None, 'max_m': None, 'final_m': None,
                'mean_yaw_rad': None}
    d = np.array([math.hypot(t[0] - e[0], t[1] - e[1]) for t, e in pairs])
    yaw = np.array([abs(math.atan2(math.sin(t[2] - e[2]), math.cos(t[2] - e[2])))
                    for t, e in pairs])
    return {'samples': len(pairs), 'ate_rmse_m': float(math.sqrt(np.mean(d ** 2))),
            'max_m': float(d.max()), 'final_m': float(d[-1]),
            'mean_yaw_rad': float(yaw.mean())}
