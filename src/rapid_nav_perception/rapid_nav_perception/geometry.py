"""
Camera geometry: intrinsics under resizing, depth resizing and back-projection.

Pure numpy, no ROS imports. Pixel centres follow the convention of OpenCV: pixel (0, 0) has its
centre at (0, 0) and the image spans [-0.5, W - 0.5].
"""

import numpy as np


def scale_intrinsics(fx, fy, cx, cy, sx, sy):
    """Return (fx, fy, cx, cy) for an image scaled by sx horizontally and sy vertically."""
    return (fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5)


def resize_depth_nearest(depth, width, height):
    """Return the depth resized to (height, width) by nearest-neighbour sampling; values kept."""
    depth = np.asarray(depth)
    in_height, in_width = depth.shape[:2]
    rows = np.floor((np.arange(height) + 0.5) * in_height / height).astype(int)
    cols = np.floor((np.arange(width) + 0.5) * in_width / width).astype(int)
    rows = np.clip(rows, 0, in_height - 1)
    cols = np.clip(cols, 0, in_width - 1)
    return depth[np.ix_(rows, cols)]


def backproject(depth, fx, fy, cx, cy, stride=1):
    """
    Return the points (N, 3) float32 of a depth image in the optical frame.

    Only finite, positive depths are used. The points are sampled every `stride` pixels in both
    directions. x = (u - cx) z / fx, y = (v - cy) z / fy, z = depth, with x right, y down and z
    forward.
    """
    depth = np.asarray(depth, dtype=np.float64)[::stride, ::stride]
    rows, cols = np.indices(depth.shape)
    u = cols * stride
    v = rows * stride
    valid = np.isfinite(depth) & (depth > 0.0)
    z = depth[valid]
    x = (u[valid] - cx) * z / fx
    y = (v[valid] - cy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def backproject_row(depth, row, fx, cx):
    """Return (u, x, z) of the finite, positive depths of one row: column, x and depth."""
    depth = np.asarray(depth, dtype=np.float64)
    z_row = depth[row, :]
    u = np.arange(z_row.size, dtype=np.float64)
    valid = np.isfinite(z_row) & (z_row > 0.0)
    z = z_row[valid]
    x = (u[valid] - cx) * z / fx
    return u[valid], x, z
