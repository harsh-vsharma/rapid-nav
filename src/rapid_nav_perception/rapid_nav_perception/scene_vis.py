"""
Geometry for the RViz view: the lidar as a polyline, the flow as bodies with convex hulls.

Pure numpy and scipy (connected components, convex hulls), no ROS imports.

Lidar: consecutive beams whose end points are closer than a gap are joined by a segment.

Flow: the samples come one per depth tile on a regular grid (egomotion.sample_points with block >
1), each with a world point and a world velocity. Grid neighbours are joined when both their
velocities and their positions are close (a small velocity gradient and no depth jump); the
connected groups are the instances. Each instance gets its mean velocity and its rotation rate
about the vertical axis, and is drawn as the convex hull of its points (body_hull).
"""

import math

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, QhullError


def scan_segments(ranges, angle_min, angle_increment, range_min, range_max, max_gap,
                  wrap=False):
    """
    Return the lidar segments as an (M, 2, 2) array of (x, y) end points in the scan frame.

    Beams outside [range_min, range_max] or not finite are dropped; two consecutive beams are
    joined when the distance between their end points is at most max_gap. With wrap the last beam
    may also join the first (a full turn).
    """
    r = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + angle_increment * np.arange(r.size)
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    xy = np.stack([r * np.cos(angles), r * np.sin(angles)], axis=1)
    a, b = np.arange(r.size - 1), np.arange(1, r.size)
    if wrap and r.size > 2:
        a, b = np.append(a, r.size - 1), np.append(b, 0)
    keep = valid[a] & valid[b]
    a, b = a[keep], b[keep]
    near = np.hypot(*(xy[a] - xy[b]).T) <= max_gap
    return np.stack([xy[a[near]], xy[b[near]]], axis=1)


def ego_speed(x0, t0, t1, dt):
    """
    Return the speed (m/s) a static point at x0 (camera frame, previous time) seems to have.

    t0 and t1 are the 4 x 4 transforms world <- camera at both times: a static point moves from
    x0 to inv(t1) t0 x0 in the camera frame. A small error in the poses or their stamps fakes a
    world speed in proportion to this, so it sets part of the noise floor.
    """
    rel = np.linalg.solve(np.asarray(t1, dtype=np.float64), np.asarray(t0, dtype=np.float64))
    moved = x0 @ rel[:3, :3].T + rel[:3, 3]
    return np.linalg.norm(moved - x0, axis=1) / dt


def suppress_noise(velocities, floor):
    """Return the velocities with those below their floor (per sample, m/s) set to zero."""
    out = np.array(velocities, dtype=np.float64)
    out[np.hypot(out[:, 0], out[:, 1]) < floor] = 0.0
    return out


def tile_indices(pixels, block):
    """Return the (row, col) grid index (N, 2) of block-tile samples from their pixel (u, v)."""
    p = np.asarray(pixels, dtype=np.float64)
    col = np.rint((p[:, 0] + 0.5) / block - 0.5).astype(int)
    row = np.rint((p[:, 1] + 0.5) / block - 0.5).astype(int)
    return np.stack([row, col], axis=1)


def index_grid(rc, shape):
    """Return a grid of the given shape with the sample index at each (row, col), -1 elsewhere."""
    grid = np.full(shape, -1, dtype=int)
    grid[rc[:, 0], rc[:, 1]] = np.arange(rc.shape[0])
    return grid


def _neighbour_pairs(grid):
    """Return (a, b) sample indices of the 4-connected grid neighbours that both hold a sample."""
    pairs = []
    for first, second in ((grid[:, :-1], grid[:, 1:]), (grid[:-1, :], grid[1:, :])):
        both = (first >= 0) & (second >= 0)
        pairs.append((first[both], second[both]))
    return (np.concatenate([p[0] for p in pairs]), np.concatenate([p[1] for p in pairs]))


def depth_edges(grid, depth, jump=0.10, relative=0.05):
    """
    Return a mask of the samples next to a depth discontinuity.

    A sample is on an edge when a 4-connected neighbour is missing or differs in depth by more
    than jump + relative x depth (m). Flow across an occlusion boundary mixes the foreground and
    the background, which fakes large 3-D velocities there.
    """
    n = depth.shape[0]
    edge = np.zeros(n, dtype=bool)
    rows, cols = grid.shape
    padded = np.full((rows + 2, cols + 2), -1, dtype=int)
    padded[1:-1, 1:-1] = grid
    centre = padded[1:-1, 1:-1]
    has = centre >= 0
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        other = padded[1 + dr:rows + 1 + dr, 1 + dc:cols + 1 + dc]
        here = centre[has]
        there = other[has]
        missing = there < 0
        z_here = depth[here]
        z_there = depth[np.where(missing, 0, there)]
        bad = missing | (np.abs(z_here - z_there) > jump + relative * z_here)
        edge[here[bad]] = True
    return edge


def depth_slope(grid, depth):
    """
    Return per sample the largest depth difference (m) to a 4-connected grid neighbour.

    Missing neighbours are ignored (a sample with none gives 0). On a slanted surface a small
    misalignment between the colour and the depth image changes the depth read at a pixel by
    about this much per tile of misalignment.
    """
    slope = np.zeros(depth.shape[0])
    rows, cols = grid.shape
    padded = np.full((rows + 2, cols + 2), -1, dtype=int)
    padded[1:-1, 1:-1] = grid
    centre = padded[1:-1, 1:-1]
    has = centre >= 0
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        other = padded[1 + dr:rows + 1 + dr, 1 + dc:cols + 1 + dc][has]
        here = centre[has]
        ok = other >= 0
        diff = np.abs(depth[here[ok]] - depth[other[ok]])
        np.maximum.at(slope, here[ok], diff)
    return slope


def segment_instances(grid, points, velocities, velocity_tol, distance_tol):
    """
    Return one label per sample: grid neighbours with close velocity and position share one.

    Two 4-connected neighbours are joined when the norm of their velocity difference is at most
    velocity_tol (m/s) and their distance at most distance_tol (m). The labels are 0..K-1.
    """
    n = points.shape[0]
    if n == 0:
        return np.zeros(0, dtype=int)
    a, b = _neighbour_pairs(grid)
    joined = ((np.linalg.norm(velocities[a] - velocities[b], axis=1) <= velocity_tol)
              & (np.linalg.norm(points[a] - points[b], axis=1) <= distance_tol))
    a, b = a[joined], b[joined]
    graph = coo_matrix((np.ones(a.size), (a, b)), shape=(n, n))
    _count, labels = connected_components(graph, directed=False)
    return labels


def instance_motion(points, velocities, labels, min_samples):
    """
    Return the instances with at least min_samples samples, as dicts.

    Each has label, n, centroid (3,), velocity (3,) (the mean), omega (rad/s, about z: the
    least-squares rotation rate of the in-plane velocity about the centroid) and size (the RMS
    distance of its samples from the centroid in the floor plane, m).
    """
    out = []
    if labels.size == 0:
        return out
    counts = np.bincount(labels)
    for label in np.flatnonzero(counts >= min_samples):
        members = labels == label
        x, v = points[members], velocities[members]
        centroid, mean = x.mean(axis=0), v.mean(axis=0)
        r, dv = x[:, :2] - centroid[:2], v[:, :2] - mean[:2]
        r2 = float(np.sum(r * r))
        omega = float(np.sum(r[:, 0] * dv[:, 1] - r[:, 1] * dv[:, 0]) / r2) if r2 > 1e-6 else 0.0
        out.append({'label': int(label), 'n': int(members.sum()), 'centroid': centroid,
                    'velocity': mean, 'omega': omega,
                    'size': math.sqrt(r2 / members.sum())})
    return out


def merge_instances(instances, distance=0.6, velocity_tol=0.3):
    """
    Return the instances with those that are close and move alike merged, as dicts.

    Texture holes and depth edges can cut one object into pieces. Two pieces whose centroids are
    within `distance` (m, floor plane) and whose mean velocities differ by at most velocity_tol
    (m/s) are merged (transitively); the result has the sample-weighted centroid, velocity, omega
    and size, n the total and labels the list of the merged labels.
    """
    groups = [[i] for i in range(len(instances))]
    changed = True
    while changed:
        changed = False
        for a in range(len(groups)):
            for b in range(a + 1, len(groups)):
                if any(np.hypot(*(instances[i]['centroid'][:2] - instances[j]['centroid'][:2]))
                       <= distance
                       and np.hypot(*(instances[i]['velocity'][:2]
                                      - instances[j]['velocity'][:2])) <= velocity_tol
                       for i in groups[a] for j in groups[b]):
                    groups[a] += groups.pop(b)
                    changed = True
                    break
            if changed:
                break
    out = []
    for g in groups:
        members = [instances[i] for i in g]
        w = np.array([m['n'] for m in members], dtype=np.float64)
        w /= w.sum()
        out.append({
            'labels': [m['label'] for m in members], 'label': members[0]['label'],
            'n': int(sum(m['n'] for m in members)),
            'centroid': sum(wi * m['centroid'] for wi, m in zip(w, members)),
            'velocity': sum(wi * m['velocity'] for wi, m in zip(w, members)),
            'omega': float(sum(wi * m['omega'] for wi, m in zip(w, members))),
            'size': float(max(m['size'] for m in members)),
        })
    return out


def confirm_movers(current, previous, dt, position_tol=0.4, velocity_tol=0.3,
                   velocity_rel=0.4):
    """
    Return one bool per current mover: whether the previous frame had a consistent one.

    current and previous are instance dicts (instance_motion). A previous mover, carried forward
    by its velocity over dt seconds, must lie within position_tol (m) of the current centroid in
    the floor plane, and the velocities must differ by at most max(velocity_tol, velocity_rel x
    current speed). Flow noise gives movers whose velocity jumps from frame to frame; a real
    mover keeps its velocity.
    """
    out = []
    for c in current:
        cv = c['velocity'][:2]
        tol = max(velocity_tol, velocity_rel * float(np.hypot(*cv)))
        ok = False
        for q in previous:
            ahead = q['centroid'][:2] + q['velocity'][:2] * dt
            if (np.hypot(*(ahead - c['centroid'][:2])) <= position_tol
                    and np.hypot(*(q['velocity'][:2] - cv)) <= tol):
                ok = True
                break
        out.append(ok)
    return out


def depth_jumps(z0, z1, jump=0.10, relative=0.05):
    """
    Return a mask of the samples whose depth jumps between the two frames (an occlusion).

    Where the background is uncovered (or covered) the flow is about zero, so a sample reads the
    object's depth in one frame and the background's in the other. A real surface moves by a few
    centimetres per frame; a change over jump + relative x depth (m) is such an occlusion.
    """
    z0, z1 = np.asarray(z0, dtype=np.float64), np.asarray(z1, dtype=np.float64)
    return np.abs(z1 - z0) > jump + relative * np.minimum(z0, z1)


# Solid body colours (r, g, b): distinct, muted, readable on a dark background.
BODY_PALETTE = ((0.40, 0.66, 0.96), (0.96, 0.62, 0.30), (0.45, 0.82, 0.55), (0.90, 0.42, 0.50),
                (0.72, 0.56, 0.92), (0.95, 0.85, 0.40), (0.35, 0.80, 0.82), (0.88, 0.55, 0.80),
                (0.62, 0.74, 0.42), (0.80, 0.80, 0.84))


def stable_ids(centroids, previous, max_distance, next_id):
    """
    Return (ids, next_id): one colour id per body, kept from the previous frame where it matches.

    The instance labels change from frame to frame, so a body keeps its colour by matching its
    centroid (floor plane) to the nearest body of the previous frame, closest pairs first, within
    max_distance (m). `previous` is a list of (centroid, id); unmatched bodies get new ids.
    """
    ids = [None] * len(centroids)
    pairs = sorted((float(np.hypot(*(np.asarray(c)[:2] - np.asarray(q)[:2]))), i, j)
                   for i, c in enumerate(centroids) for j, (q, _) in enumerate(previous))
    used = set()
    for dist, i, j in pairs:
        if dist > max_distance:
            break
        if ids[i] is None and j not in used:
            ids[i] = previous[j][1]
            used.add(j)
    for i in range(len(ids)):
        if ids[i] is None:
            ids[i], next_id = next_id, next_id + 1
    return ids, next_id


def body_hull(points, max_extent=2.5):
    """
    Return the convex hull triangles (T, 3) of one body's points (N, 3), as indices into them.

    Bodies wider than max_extent (m, in the floor plane) give no hull (a wall's hull is only a
    large flat box), nor do bodies with fewer than 4 points. Coplanar points are jittered by
    qhull ('QJ') so a flat body still gives a thin hull.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.shape[0] < 4 or np.max(np.ptp(points[:, :2], axis=0)) > max_extent:
        return np.zeros((0, 3), dtype=int)
    try:
        return ConvexHull(points, qhull_options='QJ').simplices
    except QhullError:
        return np.zeros((0, 3), dtype=int)
