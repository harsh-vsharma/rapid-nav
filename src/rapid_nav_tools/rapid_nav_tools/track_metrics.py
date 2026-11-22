"""Scoring of camera tracks against the truth: pure functions (no ROS), used by track_check."""

import math

import numpy as np

HALF_FOV_RAD = math.radians(69.0) / 2.0
MIN_RANGE_M = 0.3
MAX_RANGE_M = 6.0
MATCH_M = 0.5
TURN_RATE_RAD_S = 0.3  # the robot counts as turning above this yaw rate


def interpolate_xy(times, xy, t):
    """Return the position of a recorded path at time t (clamped at the ends), or None if empty."""
    if len(times) == 0:
        return None
    times = np.asarray(times, dtype=np.float64)
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    return (float(np.interp(t, times, xy[:, 0])), float(np.interp(t, times, xy[:, 1])))


def yaw_rate(times, yaw, t, half_window=0.25):
    """Return the robot's yaw rate (rad/s) around time t from the recorded yaw (0 if unknown)."""
    if len(times) < 2:
        return 0.0
    a = float(np.interp(t - half_window, times, np.unwrap(yaw)))
    b = float(np.interp(t + half_window, times, np.unwrap(yaw)))
    return (b - a) / (2.0 * half_window)


def velocity_xy(times, xy, t, half_window=0.15):
    """Return the finite-difference velocity of a recorded path around time t, or None."""
    if len(times) < 2:
        return None
    a = interpolate_xy(times, xy, t - half_window)
    b = interpolate_xy(times, xy, t + half_window)
    span = 2.0 * half_window
    return ((b[0] - a[0]) / span, (b[1] - a[1]) / span)


def in_view(robot_xy, robot_yaw, camera, point_xy):
    """
    Return True when a floor point is inside the camera's field of view and range.

    camera = (ox, oy, forward_yaw): the camera's position in the robot frame and the direction of
    its optical axis relative to the robot's heading. The view is 69 degrees wide, 0.3 to 6 m deep.
    """
    ox, oy, fwd = camera
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    cx = robot_xy[0] + c * ox - s * oy
    cy = robot_xy[1] + s * ox + c * oy
    dx, dy = point_xy[0] - cx, point_xy[1] - cy
    # components along the optical axis (depth) and to its right
    axis = robot_yaw + fwd
    depth = dx * math.cos(axis) + dy * math.sin(axis)
    side = dx * math.sin(axis) - dy * math.cos(axis)
    if depth < MIN_RANGE_M or depth > MAX_RANGE_M:
        return False
    return abs(math.atan2(side, depth)) <= HALF_FOV_RAD


def nearest_within(point_xy, candidates_xy, radius=MATCH_M):
    """Return (index, distance) of the nearest candidate within radius, or (None, None)."""
    best, best_d = None, None
    for index, c in enumerate(candidates_xy):
        d = math.hypot(c[0] - point_xy[0], c[1] - point_xy[1])
        if d <= radius and (best_d is None or d < best_d):
            best, best_d = index, d
    return best, best_d


def angle_between_deg(a, b):
    """Return the angle in degrees between two 2-D vectors."""
    na, nb = math.hypot(*a), math.hypot(*b)
    if na == 0.0 or nb == 0.0:
        return float('nan')
    cos = max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (na * nb)))
    return math.degrees(math.acos(cos))


def score(messages, truth, robot, camera, duration_s):
    """
    Score the recorded tracks against the truth.

    messages: list of dicts with 't' (stamp, s), 'recv' (sim time of arrival, s) and 'tracks'
    (a list of dicts id, x, y, vx, vy). truth: {name: (times, xy)} of the obstacles. robot:
    (times, xy, yaw) arrays of the robot's true pose. duration_s: simulated seconds recorded.
    Returns a dict of metrics (None where nothing was measured).
    """
    in_view_n = matched_n = 0
    pos_err, speed_err, head_err, latency = [], [], [], []
    track_msgs = {}  # id -> [messages, false messages]
    false_total = false_turning = 0  # false (track, message) pairs, and those while turning
    r_times, r_xy, r_yaw = robot
    for m in messages:
        t = m['t']
        latency.append(m['recv'] - t)
        if len(r_times) == 0:
            continue
        rxy = interpolate_xy(r_times, r_xy, t)
        ryaw = float(np.interp(t, r_times, r_yaw))
        truth_now = {n: interpolate_xy(ts, xy, t) for n, (ts, xy) in truth.items()}
        track_xy = [(k['x'], k['y']) for k in m['tracks']]
        for name, pos in truth_now.items():
            if pos is None or not in_view(rxy, ryaw, camera, pos):
                continue
            in_view_n += 1
            index, dist = nearest_within(pos, track_xy)
            if index is None:
                continue
            matched_n += 1
            pos_err.append(dist)
            k = m['tracks'][index]
            v = velocity_xy(*truth[name], t)
            if v is not None:
                speed_err.append(abs(math.hypot(k['vx'], k['vy']) - math.hypot(*v)))
                if math.hypot(*v) >= 0.1 and math.hypot(k['vx'], k['vy']) >= 0.05:
                    head_err.append(angle_between_deg((k['vx'], k['vy']), v))
        turning = abs(yaw_rate(r_times, r_yaw, t)) > TURN_RATE_RAD_S
        for k in m['tracks']:
            index, _ = nearest_within((k['x'], k['y']), [p for p in truth_now.values() if p])
            entry = track_msgs.setdefault(k['id'], [0, 0])
            entry[0] += 1
            entry[1] += 1 if index is None else 0
            if index is None:
                false_total += 1
                false_turning += 1 if turning else 0
    false_tracks = [tid for tid, (n, f) in track_msgs.items() if f * 2 > n]

    def pct(values, q):
        return float(np.percentile(values, q)) if values else None

    return {
        'in_view_samples': in_view_n,
        'matched_samples': matched_n,
        'detection_rate': (matched_n / in_view_n) if in_view_n else None,
        'position_error_median_m': pct(pos_err, 50),
        'position_error_p95_m': pct(pos_err, 95),
        'speed_error_median_mps': pct(speed_err, 50),
        'heading_error_median_deg': pct([h for h in head_err if not math.isnan(h)], 50),
        'tracks_confirmed': len(track_msgs),
        'false_tracks': len(false_tracks),
        'false_track_messages': false_total,
        'false_turning_fraction': (false_turning / false_total) if false_total else None,
        'false_per_message_mean': (false_total / len(messages)) if messages else None,
        'false_tracks_per_s': (len(false_tracks) / duration_s) if duration_s > 0 else None,
        'latency_sim_median_s': pct(latency, 50),
        'messages': len(messages),
        'duration_s': duration_s,
    }
