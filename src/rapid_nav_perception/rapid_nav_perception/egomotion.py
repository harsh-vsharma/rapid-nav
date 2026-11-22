"""
Ego-motion removal: the world velocity of each flow sample, from flow, two depths and two poses.

Pure numpy, no ROS imports. A flow sample is a pixel p of the previous frame that moved to
q = p + flow(p) in the current frame. With depth at both times it is a 3-D point at both times;
with the camera pose at both times (map <- camera) it is a world point at both times, so a static
point gives a velocity of zero whatever the robot does.
"""

import numpy as np


def sample_points(flow, depth0, depth1, intrinsics, stride=4, min_depth=0.3, max_depth=6.0):
    """
    Return (p, X0, X1): the pixels and the camera-frame points of the usable flow samples.

    The previous image is sampled on a grid of `stride` pixels (pixel (u, v), x right, y down). A
    sample is skipped when its target q is outside the image, when D0 at p or D1 at round(q) is not
    finite, or when either depth lies outside [min_depth, max_depth]. p is (N, 2) float64 (u, v),
    X0 and X1 are (N, 3) float64 in the optical frame (z forward).
    """
    fx, fy, cx, cy = intrinsics
    flow = np.asarray(flow, dtype=np.float64)
    depth0 = np.asarray(depth0, dtype=np.float64)
    depth1 = np.asarray(depth1, dtype=np.float64)
    height, width = depth0.shape
    rows, cols = np.mgrid[0:height:stride, 0:width:stride]
    u = cols.ravel().astype(np.float64)
    v = rows.ravel().astype(np.float64)
    f = flow[rows.ravel(), cols.ravel()]
    qu = u + f[:, 0]
    qv = v + f[:, 1]
    iu = np.rint(qu)
    iv = np.rint(qv)
    inside = (np.isfinite(qu) & np.isfinite(qv) & (iu >= 0) & (iu <= width - 1)
              & (iv >= 0) & (iv <= height - 1))
    iu = np.where(inside, iu, 0).astype(int)
    iv = np.where(inside, iv, 0).astype(int)
    d0 = depth0[rows.ravel(), cols.ravel()]
    d1 = depth1[iv, iu]
    ok = (inside & np.isfinite(d0) & np.isfinite(d1) & (d0 >= min_depth) & (d0 <= max_depth)
          & (d1 >= min_depth) & (d1 <= max_depth))
    u, v, qu, qv, d0, d1 = u[ok], v[ok], qu[ok], qv[ok], d0[ok], d1[ok]
    x0 = d0 * np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=1).T
    x1 = d1 * np.stack([(qu - cx) / fx, (qv - cy) / fy, np.ones_like(qu)], axis=1).T
    return np.stack([u, v], axis=1), x0.T, x1.T


def transform_points(transform, points):
    """Return the points (N, 3) mapped by a 4 x 4 homogeneous transform."""
    t = np.asarray(transform, dtype=np.float64)
    return np.asarray(points, dtype=np.float64) @ t[:3, :3].T + t[:3, 3]


def world_velocities(x0, x1, t0, t1, dt):
    """
    Return (W1, w): the world points at the current time and their velocities in m/s.

    x0 and x1 are the camera-frame points at the previous and the current time, t0 and t1 the
    4 x 4 transforms map <- camera at those times, dt the time between them in seconds.
    """
    w0 = transform_points(t0, x0)
    w1 = transform_points(t1, x1)
    return w1, (w1 - w0) / dt


def moving_mask(w1, w, min_speed=0.20, max_speed=3.0, min_z=0.05, max_z=2.0):
    """Return a mask of the samples that move on the floor plane (speed and height cuts)."""
    speed = np.hypot(w[:, 0], w[:, 1])
    return ((speed >= min_speed) & (speed <= max_speed) & (w1[:, 2] >= min_z)
            & (w1[:, 2] <= max_z))
