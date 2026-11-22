"""
Ego-motion removal: the world velocity of each flow sample, from flow, two depths and two poses.

Pure numpy, no ROS imports. A flow sample is a pixel p of the previous frame that moved to
q = p + flow(p) in the current frame. With depth at both times it is a 3-D point at both times;
with the camera pose at both times (map <- camera) it is a world point at both times, so a static
point gives a velocity of zero whatever the robot does.
"""

import warnings

import numpy as np


def block_median(values, block, min_valid=8):
    """
    Return the median of each block x block tile of a 2-D array, over its finite values.

    A tile with fewer than `min_valid` finite values gives NaN. The result has shape
    (height // block, width // block); a partial tile at the right or bottom edge is dropped.
    """
    values = np.asarray(values, dtype=np.float64)
    rows, cols = values.shape[0] // block, values.shape[1] // block
    tiles = values[:rows * block, :cols * block].reshape(rows, block, cols, block)
    tiles = tiles.transpose(0, 2, 1, 3).reshape(rows, cols, block * block)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        out = np.nanmedian(tiles, axis=-1)
    out[np.sum(np.isfinite(tiles), axis=-1) < min_valid] = np.nan
    return out


def block_texture(gray, block):
    """
    Return the smaller eigenvalue of the 2 x 2 structure tensor of a grey image, per tile.

    The gradients (central differences, grey levels per pixel) are squared and summed over each
    block x block tile; the smaller eigenvalue is large only where the image varies in two
    directions, i.e. where optical flow is well defined (a flat patch gives 0, a straight edge
    gives 0 as well: the aperture problem). The result has shape (height // block, width // block).
    """
    gray = np.asarray(gray, dtype=np.float64)
    gy, gx = np.gradient(gray)
    rows, cols = gray.shape[0] // block, gray.shape[1] // block

    def tile_sum(values):
        tiles = values[:rows * block, :cols * block].reshape(rows, block, cols, block)
        return tiles.sum(axis=(1, 3))

    a, b, c = tile_sum(gx * gx), tile_sum(gx * gy), tile_sum(gy * gy)
    return 0.5 * (a + c) - np.sqrt((0.5 * (a - c)) ** 2 + b ** 2)


def texture_mask(p, texture, block, min_texture):
    """Return a mask of the samples p (N, 2) of (u, v) whose tile has texture >= min_texture."""
    iu = np.clip((p[:, 0] // block).astype(int), 0, texture.shape[1] - 1)
    iv = np.clip((p[:, 1] // block).astype(int), 0, texture.shape[0] - 1)
    return texture[iv, iu] >= min_texture


def sample_points(flow, depth0, depth1, intrinsics, stride=4, min_depth=0.3, max_depth=6.0,
                  block=1):
    """
    Return (p, X0, X1): the pixels and the camera-frame points of the usable flow samples.

    The previous image is sampled on a grid of `stride` pixels (pixel (u, v), x right, y down). A
    sample is skipped when its target q is outside the image, when D0 at p or D1 at round(q) is not
    finite, or when either depth lies outside [min_depth, max_depth]. p is (N, 2) float64 (u, v),
    X0 and X1 are (N, 3) float64 in the optical frame (z forward).

    With `block` > 1 both depth images and the flow are first reduced to the median of each
    block x block tile (the depth noise of the sensor would otherwise look like motion); one sample
    is then taken per tile, at the tile centre, and `stride` is not used.
    """
    fx, fy, cx, cy = intrinsics
    flow = np.asarray(flow, dtype=np.float64)
    depth0 = np.asarray(depth0, dtype=np.float64)
    depth1 = np.asarray(depth1, dtype=np.float64)
    if block > 1:
        return _sample_blocks(flow, depth0, depth1, intrinsics, block, min_depth, max_depth)
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


def _sample_blocks(flow, depth0, depth1, intrinsics, block, min_depth, max_depth):
    """Return (p, X0, X1) with one sample per block x block tile (see sample_points)."""
    fx, fy, cx, cy = intrinsics
    d0 = block_median(depth0, block)
    d1 = block_median(depth1, block)
    fu = block_median(flow[:, :, 0], block, 1)
    fv = block_median(flow[:, :, 1], block, 1)
    rows, cols = d0.shape
    jj, ii = np.meshgrid(np.arange(cols), np.arange(rows))
    u = ((jj + 0.5) * block - 0.5).ravel()
    v = ((ii + 0.5) * block - 0.5).ravel()
    qu = u + fu.ravel()
    qv = v + fv.ravel()
    tu = np.floor((qu + 0.5) / block)
    tv = np.floor((qv + 0.5) / block)
    inside = (np.isfinite(qu) & np.isfinite(qv) & (tu >= 0) & (tu <= cols - 1)
              & (tv >= 0) & (tv <= rows - 1))
    tu = np.where(inside, tu, 0).astype(int)
    tv = np.where(inside, tv, 0).astype(int)
    a = d0.ravel()
    b = d1[tv, tu]
    ok = (inside & np.isfinite(a) & np.isfinite(b) & (a >= min_depth) & (a <= max_depth)
          & (b >= min_depth) & (b <= max_depth))
    u, v, qu, qv, a, b = u[ok], v[ok], qu[ok], qv[ok], a[ok], b[ok]
    x0 = a * np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=1).T
    x1 = b * np.stack([(qu - cx) / fx, (qv - cy) / fy, np.ones_like(qu)], axis=1).T
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


def depth_speed_floor(z, fx, dt, k=2.0, sigma_px=0.5):
    """
    Return the speed (m/s) that flow error alone can fake at depth z: k * z * sigma_px / (fx * dt).

    A flow error of sigma_px pixels moves a point at depth z sideways by z * sigma_px / fx metres,
    which over dt seconds looks like a speed; k is the number of sigmas that must be exceeded.
    """
    return k * np.asarray(z, dtype=np.float64) * sigma_px / (fx * dt)


def moving_mask(w1, w, min_speed=0.20, max_speed=3.0, min_z=0.05, max_z=2.0):
    """
    Return a mask of the samples that move on the floor plane (speed and height cuts).

    min_speed may be one number or one value per sample (see depth_speed_floor).
    """
    speed = np.hypot(w[:, 0], w[:, 1])
    return ((speed >= min_speed) & (speed <= max_speed) & (w1[:, 2] >= min_z)
            & (w1[:, 2] <= max_z))
