"""
Pure functions behind sim_check: geometry, rate verdicts and image helpers.

Nothing here imports ROS, so the functions run on a plain Python with numpy (see the tests).
"""

import math

import numpy as np

PROFILES = ('native', 'docker')


def expected_fx(width, hfov):
    """Return the focal length in pixels of a pinhole camera of this width and field of view."""
    return (width / 2.0) / math.tan(hfov / 2.0)


def quaternion_from_rpy(roll, pitch, yaw):
    """Return the quaternion (x, y, z, w) of a roll, pitch, yaw rotation (ZYX convention)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def yaw_from_quaternion(x, y, z, w):
    """Return the yaw angle (rad) of a quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(a, b):
    """Return a - b wrapped to the interval (-pi, pi]."""
    difference = (a - b) % (2.0 * math.pi)
    if difference > math.pi:
        difference -= 2.0 * math.pi
    return difference


def rate_verdict(count, rate, expected, profile):
    """
    Return PASS, WARN or FAIL for a measured topic rate.

    native: PASS at 80 % of the expected rate or more, WARN when messages arrive but slower,
    FAIL when none arrive. docker: PASS when messages arrive, FAIL when none arrive (software
    rendering makes the rates low, so only presence is checked).
    """
    if profile not in PROFILES:
        raise ValueError("profile must be 'native' or 'docker', got {!r}".format(profile))
    if count <= 0 or rate <= 0.0:
        return 'FAIL'
    if profile == 'docker':
        return 'PASS'
    if rate >= 0.8 * expected:
        return 'PASS'
    return 'WARN'


def colourise_depth(depth, near, far):
    """Return a uint8 RGB image of a depth array: near is blue, far is red, non-finite is black."""
    depth = np.asarray(depth, dtype=np.float64)
    t = np.clip((depth - near) / (far - near), 0.0, 1.0)
    red = 255.0 * t
    green = 255.0 * (1.0 - np.abs(2.0 * t - 1.0))
    blue = 255.0 * (1.0 - t)
    rgb = np.stack([red, green, blue], axis=-1)
    rgb[~np.isfinite(depth)] = 0.0
    return np.clip(np.rint(rgb), 0, 255).astype(np.uint8)


def scan_image(ranges, angle_min, increment, size_px, max_range):
    """
    Return a top-down uint8 RGB view of a laser scan: robot at the centre, x right, y up.

    Each finite return inside max_range is drawn as a white pixel; the robot is a red pixel.
    """
    image = np.zeros((size_px, size_px, 3), dtype=np.uint8)
    centre = (size_px - 1) / 2.0
    scale = (size_px / 2.0) / max_range
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + increment * np.arange(ranges.size)
    valid = np.isfinite(ranges) & (ranges > 0.0) & (ranges <= max_range)
    x = ranges[valid] * np.cos(angles[valid])
    y = ranges[valid] * np.sin(angles[valid])
    columns = np.rint(centre + x * scale).astype(int)
    rows = np.rint(centre - y * scale).astype(int)
    inside = (columns >= 0) & (columns < size_px) & (rows >= 0) & (rows < size_px)
    image[rows[inside], columns[inside]] = (255, 255, 255)
    image[int(round(centre)), int(round(centre))] = (255, 0, 0)
    return image


def planar_delta(start_xyyaw, end_xyyaw):
    """Return (distance, yaw change) between two (x, y, yaw) poses; the yaw change is wrapped."""
    dx = end_xyyaw[0] - start_xyyaw[0]
    dy = end_xyyaw[1] - start_xyyaw[1]
    distance = math.hypot(dx, dy)
    return (distance, angle_diff(end_xyyaw[2], start_xyyaw[2]))


def _polyline(points):
    """Return the points as an (N, 2) float array."""
    return np.asarray(points, dtype=np.float64).reshape(-1, 2)


def path_length(points):
    """Return the length in metres of a polyline of (x, y) points; zero with fewer than two."""
    points = _polyline(points)
    if points.shape[0] < 2:
        return 0.0
    steps = np.diff(points, axis=0)
    return float(np.hypot(steps[:, 0], steps[:, 1]).sum())


def max_step(points):
    """Return the longest distance in metres between consecutive points; zero if fewer than two."""
    points = _polyline(points)
    if points.shape[0] < 2:
        return 0.0
    steps = np.diff(points, axis=0)
    return float(np.hypot(steps[:, 0], steps[:, 1]).max())


def heading_change_per_metre(points):
    """
    Return the total turning of a polyline in degrees per metre of its length.

    Segments of zero length are ignored; the turn at each joint is wrapped to (-pi, pi]. A
    straight line, or a path with fewer than two moving segments, turns by zero.
    """
    points = _polyline(points)
    steps = np.diff(points, axis=0)
    lengths = np.hypot(steps[:, 0], steps[:, 1])
    moving = lengths > 1e-9
    steps = steps[moving]
    lengths = lengths[moving]
    total = float(lengths.sum())
    if steps.shape[0] < 2 or total <= 0.0:
        return 0.0
    headings = np.arctan2(steps[:, 1], steps[:, 0])
    turns = np.diff(headings)
    turns = np.abs(np.arctan2(np.sin(turns), np.cos(turns)))
    return math.degrees(float(turns.sum())) / total


def clearances(points, occupied_xy, chunk=128):
    """
    Return, for each point, the distance in metres to the nearest occupied cell centre.

    points and occupied_xy are (N, 2) arrays in metres. The pairwise distances are computed in
    blocks of `chunk` points to bound the memory. With no occupied cell the clearance is infinite.
    """
    points = _polyline(points)
    occupied = _polyline(occupied_xy)
    result = np.full(points.shape[0], np.inf)
    if occupied.shape[0] == 0:
        return result
    for start in range(0, points.shape[0], chunk):
        block = points[start:start + chunk]
        offsets = block[:, None, :] - occupied[None, :, :]
        result[start:start + chunk] = np.sqrt((offsets ** 2).sum(axis=2)).min(axis=1)
    return result


def occupied_cells_xy(grid, resolution, origin):
    """
    Return the centres in metres, as an (N, 2) array, of the occupied cells of a grid.

    grid is indexed [row, column] with row 0 at the bottom of the map, as OccupancyGrid data is
    once reshaped. A cell is occupied when its value is 65 or more; unknown cells (-1) are not.
    origin is the (x, y) of the lower-left corner of the lower-left cell.
    """
    values = np.asarray(grid)
    rows, columns = np.nonzero(values >= 65)
    x = origin[0] + (columns + 0.5) * resolution
    y = origin[1] + (rows + 0.5) * resolution
    if rows.size == 0:
        return np.zeros((0, 2))
    return np.column_stack([x, y])


def body_clearance(xy, occupied_xy, extra_boxes, radius, cell=0.05, chunk=128):
    """
    Return, per robot centre, the distance in metres from the robot's circle to an obstacle.

    A negative value is an overlap. Each occupied map cell is a square of side `cell` centred on
    its centre (occupied_xy). Each extra box is (x, y, half_side) for a square of side
    2 * half_side. The distance from a centre to a square is zero inside it. With no obstacle at
    all the clearance is infinite.
    """
    centres = _polyline(xy)
    cells = _polyline(occupied_xy)
    boxes = np.asarray(extra_boxes, dtype=np.float64).reshape(-1, 3)
    squares = np.vstack([
        np.column_stack([cells, np.full(cells.shape[0], cell / 2.0)]),
        boxes,
    ])
    result = np.full(centres.shape[0], np.inf)
    if squares.shape[0] == 0:
        return result
    for start in range(0, centres.shape[0], chunk):
        block = centres[start:start + chunk]
        half = squares[None, :, 2]
        dx = np.maximum(np.abs(block[:, None, 0] - squares[None, :, 0]) - half, 0.0)
        dy = np.maximum(np.abs(block[:, None, 1] - squares[None, :, 1]) - half, 0.0)
        nearest = np.hypot(dx, dy).min(axis=1)
        result[start:start + chunk] = nearest - radius
    return result


def interpolate_track(times, xy, query_times):
    """Return the track's (x, y) at the query times: linear per axis, held outside the samples."""
    times = np.asarray(times, dtype=np.float64).ravel()
    points = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    query = np.asarray(query_times, dtype=np.float64).ravel()
    return np.column_stack([np.interp(query, times, points[:, 0]),
                            np.interp(query, times, points[:, 1])])


def dynamic_clearance(robot_xy, robot_times, tracks, radius):
    """
    Return (per_sample, per_obstacle_min): the robot circle's clearance to moving obstacles.

    Each track is a dict with name, times, xy and kind: 'circle' (radius) or 'box' (half_x,
    half_y, axis-aligned). The clearance of a sample is the distance from the robot's circle to
    the obstacle's outline at the same time (negative: overlap). per_sample is the minimum over
    the obstacles (infinite with none); per_obstacle_min maps each name to its own minimum.
    """
    centres = _polyline(robot_xy)
    per_sample = np.full(centres.shape[0], np.inf)
    per_obstacle = {}
    for track in tracks:
        at = interpolate_track(track['times'], track['xy'], robot_times)
        if track['kind'] == 'circle':
            gap = np.hypot(centres[:, 0] - at[:, 0], centres[:, 1] - at[:, 1]) \
                - track['radius'] - radius
        else:
            dx = np.maximum(np.abs(centres[:, 0] - at[:, 0]) - track['half_x'], 0.0)
            dy = np.maximum(np.abs(centres[:, 1] - at[:, 1]) - track['half_y'], 0.0)
            gap = np.hypot(dx, dy) - radius
        per_obstacle[track['name']] = float(gap.min()) if gap.size else float('inf')
        per_sample = np.minimum(per_sample, gap)
    return per_sample, per_obstacle


def path_conformance(track_xy, path, mode):
    """Return the largest distance in metres from a track point to the scripted polyline."""
    points = np.asarray(track_xy, dtype=np.float64).reshape(-1, 2)
    vertices = [(float(p['x']), float(p['y'])) if isinstance(p, dict)
                else (float(p[0]), float(p[1])) for p in path]
    if mode == 'loop':
        vertices.append(vertices[0])
    if points.shape[0] == 0:
        return 0.0
    nearest = np.full(points.shape[0], np.inf)
    for (ax, ay), (bx, by) in zip(vertices[:-1], vertices[1:]):
        sx, sy = bx - ax, by - ay
        length2 = sx * sx + sy * sy
        if length2 == 0.0:
            along = np.zeros(points.shape[0])
        else:
            along = np.clip(((points[:, 0] - ax) * sx + (points[:, 1] - ay) * sy) / length2,
                            0.0, 1.0)
        nearest = np.minimum(nearest, np.hypot(points[:, 0] - (ax + along * sx),
                                               points[:, 1] - (ay + along * sy)))
    return float(nearest.max())


def moving_speed(times, xy, min_speed=0.05, mask=None):
    """
    Return the mean speed over the steps faster than min_speed (zero when none is).

    With a mask (one bool per sample), only the steps whose two ends are both True count, so
    the samples where something else pushed the track can be left out.
    """
    t = np.asarray(times, dtype=np.float64).ravel()
    points = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if t.size < 2:
        return 0.0
    dt = np.diff(t)
    valid = dt > 0
    if mask is not None:
        keep = np.asarray(mask, dtype=bool).ravel()
        valid &= keep[:-1] & keep[1:]
    speed = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))[valid] / dt[valid]
    moving = speed[speed > min_speed]
    return float(moving.mean()) if moving.size else 0.0


def longest_stall(times, xy, gap, speed, path, mode, window=1.0, near_robot=0.30,
                  started=0.30, finished=0.10, near_vertex=0.30):
    """
    Return the longest time in seconds a track stood still although nothing explains it.

    A window is the span of `window` seconds starting at a sample. Only windows where the
    obstacle is free (its clearance `gap` to the robot is above near_robot throughout), has
    started (moved more than `started` from its first point), is not finished (for mode 'once',
    more than `finished` from the last path point) and stays more than `near_vertex` from every
    path vertex (corners and reversals slow down legitimately) are judged. A window is stalled
    when the net displacement is below half of speed * window. A run of consecutive stalled
    windows covers (last start - first start + window) seconds; the longest one is returned
    (zero when none is stalled).
    """
    t = np.asarray(times, dtype=np.float64).ravel()
    points = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    free = np.asarray(gap, dtype=np.float64).ravel() > near_robot
    vertices = np.asarray([(float(p['x']), float(p['y'])) if isinstance(p, dict)
                           else (float(p[0]), float(p[1])) for p in path], dtype=np.float64)
    if t.size < 2:
        return 0.0
    to_vertex = np.min(np.hypot(points[:, None, 0] - vertices[None, :, 0],
                                points[:, None, 1] - vertices[None, :, 1]), axis=1)
    far_from_vertices = to_vertex > near_vertex
    moved = np.hypot(points[:, 0] - points[0, 0], points[:, 1] - points[0, 1]) > started
    if mode == 'once':
        active = np.hypot(points[:, 0] - vertices[-1, 0],
                          points[:, 1] - vertices[-1, 1]) > finished
    else:
        active = np.ones(t.size, dtype=bool)
    eligible_sample = free & far_from_vertices & active
    longest = 0.0
    run_first = None
    for i in range(t.size):
        j = int(np.searchsorted(t, t[i] + window - 1e-9))
        if j >= t.size:
            break
        ok = bool(moved[i]) and bool(eligible_sample[i:j + 1].all())
        if ok:
            displacement = float(np.hypot(*(points[j] - points[i])))
            stalled = displacement < 0.5 * speed * (t[j] - t[i])
        else:
            stalled = False
        if stalled:
            if run_first is None:
                run_first = t[i]
            longest = max(longest, t[i] - run_first + window)
        else:
            run_first = None
    return float(longest)


def collision_episodes(clearances, threshold=0.0):
    """Return how many times the clearance falls below the threshold: entries, not samples."""
    values = np.asarray(clearances, dtype=np.float64).ravel()
    below = values < threshold
    entered = below & ~np.concatenate(([False], below[:-1]))
    return int(entered.sum())


def tracking_error(xy, path_xy):
    """Return the mean distance in metres from each point to the nearest point of the path."""
    points = _polyline(xy)
    path = _polyline(path_xy)
    if points.shape[0] == 0 or path.shape[0] == 0:
        return float('nan')
    return float(np.mean(clearances(points, path)))


def tracking_error_in_force(xy, times, plan_times, plans):
    """
    Return the mean distance, in metres, from each point to the plan in force at its time.

    A plan is in force from the moment it is published until the next one (plan_times, sorted).
    A point before the first plan is skipped, and so is a plan with no point; with nothing left to
    compare the error is not a number.
    """
    points = _polyline(xy)
    when = np.asarray(times, dtype=np.float64).ravel()
    starts = np.asarray(plan_times, dtype=np.float64).ravel()
    distances = []
    for point, moment in zip(points, when):
        index = int(np.searchsorted(starts, moment, side='right')) - 1
        if index < 0:
            continue
        path = _polyline(plans[index])
        if path.shape[0] == 0:
            continue
        distances.append(float(clearances(point[None, :], path)[0]))
    if not distances:
        return float('nan')
    return float(np.mean(distances))


def travelled_length(xy):
    """Return the length in metres of the path the robot drove through its positions."""
    return path_length(xy)
