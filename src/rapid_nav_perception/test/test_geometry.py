"""Tests of the camera geometry, with hand-checkable values."""

import numpy as np
import pytest

from rapid_nav_perception.geometry import (
    backproject,
    backproject_row,
    resize_depth_nearest,
    scale_intrinsics,
)


def test_constant_depth_back_projects_to_its_plane():
    """A constant depth of 2 m gives z = 2 everywhere and x = 2 (u - cx) / fx."""
    depth = np.full((4, 6), 2.0, dtype=np.float32)
    points = backproject(depth, fx=100.0, fy=100.0, cx=2.5, cy=1.5)
    assert points.shape == (24, 3)
    assert np.allclose(points[:, 2], 2.0)
    assert points.dtype == np.float32
    # the pixel at column 5 of row 0: x = 2 (5 - 2.5) / 100, y = 2 (0 - 1.5) / 100
    index = 5
    assert points[index, 0] == pytest.approx(2.0 * (5 - 2.5) / 100.0)
    assert points[index, 1] == pytest.approx(2.0 * (0 - 1.5) / 100.0)


def test_stride_samples_every_nth_pixel():
    """Keep every second pixel in both directions with stride 2."""
    depth = np.full((4, 6), 1.0, dtype=np.float32)
    points = backproject(depth, fx=10.0, fy=10.0, cx=0.0, cy=0.0, stride=2)
    assert points.shape == (6, 3)


def test_nan_zero_and_negative_depths_are_dropped():
    """Only finite, positive depths are back-projected."""
    depth = np.array([[1.0, np.nan], [0.0, -3.0]], dtype=np.float32)
    points = backproject(depth, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    assert points.shape == (1, 3)
    assert points[0, 2] == pytest.approx(1.0)


def test_row_back_projection_gives_bearings_and_depths():
    """backproject_row returns the column, x and depth of the valid pixels of one row."""
    depth = np.array([[1.0, np.nan, 2.0], [3.0, 3.0, 3.0]], dtype=np.float64)
    u, x, z = backproject_row(depth, row=0, fx=2.0, cx=1.0)
    assert list(u) == [0.0, 2.0]
    assert list(z) == [1.0, 2.0]
    assert x == pytest.approx([(0 - 1.0) * 1.0 / 2.0, (2 - 1.0) * 2.0 / 2.0])


def test_scale_intrinsics_halves_at_half_size():
    """At half size fx and fy halve, and cx 319.5 maps to 159.5 (pixel-centre convention)."""
    fx, fy, cx, cy = scale_intrinsics(462.0, 462.0, 319.5, 239.5, 0.5, 0.5)
    assert fx == pytest.approx(231.0)
    assert fy == pytest.approx(231.0)
    assert cx == pytest.approx(159.5)
    assert cy == pytest.approx(119.5)


def test_nearest_resize_keeps_values():
    """Every value of the resized depth image is a value of the original."""
    depth = np.arange(16, dtype=np.float32).reshape(4, 4)
    small = resize_depth_nearest(depth, width=2, height=2)
    assert small.shape == (2, 2)
    assert set(small.ravel().tolist()) <= set(depth.ravel().tolist())
    assert small.tolist() == [[5.0, 7.0], [13.0, 15.0]]
