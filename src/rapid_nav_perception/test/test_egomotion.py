"""Tests of rapid_nav_perception.egomotion on analytically rendered scenes."""

import math

import numpy as np
from rapid_nav_perception.egomotion import (
    moving_mask,
    sample_points,
    transform_points,
    world_velocities,
)

W, H = 320, 240
FX = FY = 160.0 / math.tan(1.211 / 2.0)
CX, CY = 159.5, 119.5
K = (FX, FY, CX, CY)
PLANE_Z = 2.0
DT = 0.1


def pose(yaw_deg=0.0, translation=(0.0, 0.0, 0.0)):
    """Return a 4 x 4 camera-to-world transform: yaw about the optical y axis, then a shift."""
    a = math.radians(yaw_deg)
    t = np.eye(4)
    t[:3, :3] = [[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0],
                 [-math.sin(a), 0.0, math.cos(a)]]
    t[:3, 3] = translation
    return t


def rays():
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    return u, v, np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u)], axis=-1)


def project(points_cam):
    z = points_cam[..., 2]
    return FX * points_cam[..., 0] / z + CX, FY * points_cam[..., 1] / z + CY


def render(t1, patch=None, shift=(0.0, 0.0)):
    """Return (flow, depth0, depth1) of the plane z = 2 (world = camera at t0) seen from t1."""
    u, v, d = rays()
    depth0 = np.full((H, W), PLANE_Z)
    world0 = d * PLANE_Z
    world1 = world0.copy()
    if patch is not None:
        x0, x1, y0, y1 = patch
        inside = (world0[..., 0] >= x0) & (world0[..., 0] <= x1) & (world0[..., 1] >= y0) \
            & (world0[..., 1] <= y1)
        world1[inside, 0] += shift[0]
        world1[inside, 1] += shift[1]
    r, t = t1[:3, :3], t1[:3, 3]
    cam1 = (world1 - t) @ r  # R^T (W - t)
    qu, qv = project(cam1)
    flow = np.stack([qu - u, qv - v], axis=-1)
    # depth of every pixel of the second image: ray from camera 1 to the plane
    wd = d @ r.T
    s = (PLANE_Z - t[2]) / wd[..., 2]
    return flow, depth0, s


def velocities(flow, d0, d1, t0, t1, **kw):
    p, x0, x1 = sample_points(flow, d0, d1, K, **kw)
    w1, w = world_velocities(x0, x1, t0, t1, DT)
    return p, w1, w


def test_static_plane_while_the_camera_moves_and_turns():
    t0, t1 = pose(), pose(3.0, (0.0, 0.0, 0.1))
    flow, d0, d1 = render(t1)
    p, w1, w = velocities(flow, d0, d1, t0, t1)
    assert len(p) > 1000
    assert np.max(np.linalg.norm(w, axis=1)) < 0.02


def test_moving_patch_is_found_and_the_rest_is_still():
    t0, t1 = pose(), pose(3.0, (0.0, 0.0, 0.1))
    flow, d0, d1 = render(t1, patch=(-0.2, 0.2, -0.5, 0.5), shift=(0.05, 0.0))
    p, w1, w = velocities(flow, d0, d1, t0, t1)
    # samples whose previous world point lies in the patch
    wx = (p[:, 0] - CX) / FX * PLANE_Z
    wy = (p[:, 1] - CY) / FY * PLANE_Z
    inside = (wx >= -0.18) & (wx <= 0.18) & (wy >= -0.48) & (wy <= 0.48)
    outside = (np.abs(wx) > 0.3) | (np.abs(wy) > 0.6)
    assert inside.sum() > 50
    assert np.all(np.abs(w[inside, 0] - 0.5) < 0.05)
    assert np.all(np.abs(w[inside, 1]) < 0.05)
    assert np.max(np.linalg.norm(w[outside], axis=1)) < 0.02


def test_radial_motion_with_zero_image_flow_is_found():
    # a point 3 m ahead moves to 2.9 m: the flow is zero, the depth changes
    flow = np.zeros((H, W, 2))
    d0 = np.full((H, W), 3.0)
    d1 = np.full((H, W), 2.9)
    t = pose()
    p, w1, w = velocities(flow, d0, d1, t, t)
    # every sample moves along its own ray by 0.1 m in 0.1 s: speed = ||ray|| * 1 m/s
    ray = np.stack([(p[:, 0] - CX) / FX, (p[:, 1] - CY) / FY, np.ones(len(p))], axis=1)
    assert np.allclose(np.linalg.norm(w, axis=1), np.linalg.norm(ray, axis=1), atol=1e-6)
    centre = np.argmin(np.hypot(p[:, 0] - CX, p[:, 1] - CY))
    assert np.allclose(w[centre], [0.0, 0.0, -1.0], atol=0.02)


def test_depth_range_and_image_edge_skips():
    flow = np.zeros((H, W, 2))
    d0 = np.full((H, W), 2.0)
    d1 = np.full((H, W), 2.0)
    n_all = sample_points(flow, d0, d1, K)[0].shape[0]
    assert n_all == 80 * 60
    # too near and too far in the previous depth, a NaN and an out-of-range current depth
    d0b = d0.copy()
    d0b[0:40, :] = 0.2
    d0b[40:80, :] = 7.0
    assert sample_points(flow, d0b, d1, K)[0].shape[0] == n_all - 20 * 80
    d1b = d1.copy()
    d1b[100:120, :] = np.nan
    d1b[120:140, :] = 0.1
    assert sample_points(flow, d0, d1b, K)[0].shape[0] == n_all - 10 * 80
    # flow that leaves the image
    flow2 = flow.copy()
    flow2[:, :, 0] = 400.0
    assert sample_points(flow2, d0, d1, K)[0].shape[0] == 0
    flow3 = flow.copy()
    flow3[0:4, :, 1] = -3.0  # the first sample row goes above the top edge
    assert sample_points(flow3, d0, d1, K)[0].shape[0] == n_all - 80


def test_stride_changes_the_grid():
    flow = np.zeros((H, W, 2))
    d = np.full((H, W), 2.0)
    assert sample_points(flow, d, d, K, stride=3)[0].shape[0] == 107 * 80
    assert sample_points(flow, d, d, K, stride=8)[0].shape[0] == 40 * 30


def test_transform_points_known_answer():
    t = pose(90.0, (1.0, 2.0, 3.0))
    out = transform_points(t, np.array([[0.0, 0.0, 1.0]]))
    assert np.allclose(out, [[2.0, 2.0, 3.0]], atol=1e-9)


def test_moving_mask_speed_and_height_cuts():
    w1 = np.array([[0, 0, 0.5], [0, 0, 0.5], [0, 0, 0.5], [0, 0, 0.02], [0, 0, 2.5], [0, 0, 0.5]])
    w = np.array([[0.5, 0, 0], [0.1, 0, 0], [5.0, 0, 0], [0.5, 0, 0], [0.5, 0, 0], [0, 0, 1.0]])
    assert list(moving_mask(w1, w)) == [True, False, False, False, False, False]
