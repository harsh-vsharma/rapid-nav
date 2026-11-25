"""
Remove the lidar beams that hit tracked moving obstacles, so that SLAM maps only the static world.

Each track (position, velocity, radius and sigmas in the map frame, valid at its own stamp) is
moved to the scan stamp with its velocity. A beam whose end point lies inside the disk of radius
`radius + margin + k * sigma` around the predicted centre is removed (capped at `max_radius`, so
an uncertain track cannot erase a whole wall). sigma combines the position sigma and the velocity
sigma over the prediction time: sqrt(sigma_p^2 + (sigma_v * dt)^2).

A removed beam becomes NaN. slam_toolbox's Karto skips NaN readings for both the hit and the ray
clearing; a reading beyond the range would instead clear the cells along the beam, the wall behind
the person included.

Pure numpy and the standard library, no ROS imports.
"""

import math

import numpy as np

MARGIN = 0.10      # m added to the track's radius
K_SIGMA = 2.0      # sigmas added to the radius
MAX_RADIUS = 0.8   # m: largest removal radius
MIN_SPEED = 0.0    # m/s: slower tracks are not removed (0: every confirmed track)
MAX_AGE = 0.5      # s: tracks older than this (against the scan) are ignored


def predict(tracks, dt):
    """
    Return (centres (N, 2), sigmas (N,)) of tracks moved forward by dt seconds.

    `tracks` is a list of dicts x, y, vx, vy, position_sigma, velocity_sigma (sigmas 0 = unknown).
    """
    if not tracks:
        return np.zeros((0, 2)), np.zeros(0)
    p = np.array([[t['x'], t['y']] for t in tracks], dtype=np.float64)
    v = np.array([[t['vx'], t['vy']] for t in tracks], dtype=np.float64)
    sp = np.array([float(t.get('position_sigma', 0.0)) for t in tracks])
    sv = np.array([float(t.get('velocity_sigma', 0.0)) for t in tracks])
    return p + v * float(dt), np.sqrt(sp ** 2 + (sv * float(dt)) ** 2)


def removal_radius(radius, sigma, margin=MARGIN, k=K_SIGMA, max_radius=MAX_RADIUS):
    """Return min(radius + margin + k sigma, max_radius), element-wise."""
    r = np.asarray(radius, dtype=np.float64) + margin + k * np.asarray(sigma, dtype=np.float64)
    return np.minimum(r, max_radius)


def beam_mask(ranges, angle_min, angle_increment, range_min, range_max, centres, radii):
    """
    Return a boolean mask of the beams whose end point lies inside one of the disks.

    `centres` (N, 2) and `radii` (N,) are in the laser frame. Invalid beams (not finite, outside
    [range_min, range_max]) are never marked.
    """
    r = np.asarray(ranges, dtype=np.float64)
    mask = np.zeros(r.shape[0], dtype=bool)
    centres = np.asarray(centres, dtype=np.float64).reshape(-1, 2)
    radii = np.asarray(radii, dtype=np.float64).reshape(-1)
    if r.shape[0] == 0 or centres.shape[0] == 0:
        return mask
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    a = angle_min + angle_increment * np.arange(r.shape[0])
    rv = np.where(valid, r, 0.0)
    ends = np.stack([rv * np.cos(a), rv * np.sin(a)], axis=1)
    d2 = np.sum((ends[:, None, :] - centres[None, :, :]) ** 2, axis=2)
    inside = np.any(d2 <= (radii[None, :] ** 2), axis=1)
    return inside & valid


def filtered_ranges(ranges, mask):
    """Return a float32 copy of the ranges with NaN where the mask is True."""
    out = np.array(ranges, dtype=np.float32, copy=True)
    out[np.asarray(mask, dtype=bool)] = np.nan
    return out


def to_frame(centres, rotation_2x2, translation_2):
    """Return the centres (N, 2) moved by a planar transform: R c + t."""
    c = np.asarray(centres, dtype=np.float64).reshape(-1, 2)
    return c @ np.asarray(rotation_2x2, dtype=np.float64).T + np.asarray(translation_2)


def select(tracks, track_age, min_speed=MIN_SPEED, max_age=MAX_AGE):
    """Return the tracks to remove: fresh enough (age <= max_age) and at least min_speed fast."""
    if track_age is None or track_age > max_age:
        return []
    return [t for t in tracks if math.hypot(t['vx'], t['vy']) >= min_speed]


def filter_scan(ranges, angle_min, angle_increment, range_min, range_max, tracks, dt,
                map_to_laser, margin=MARGIN, k=K_SIGMA, max_radius=MAX_RADIUS):
    """
    Return (filtered ranges, mask) for one scan.

    `tracks` are in the map frame, `dt` = scan stamp - track stamp, and `map_to_laser` = (R, t),
    the planar transform from map to laser coordinates.
    """
    centres, sigmas = predict(tracks, dt)
    if centres.shape[0] == 0:
        mask = np.zeros(len(ranges), dtype=bool)
        return filtered_ranges(ranges, mask), mask
    radii = removal_radius([t['radius'] for t in tracks], sigmas, margin, k, max_radius)
    local = to_frame(centres, *map_to_laser)
    mask = beam_mask(ranges, angle_min, angle_increment, range_min, range_max, local, radii)
    return filtered_ranges(ranges, mask), mask
