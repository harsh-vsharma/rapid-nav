"""Tests of the flow colour picture, with hand-checkable colours."""

import numpy as np

from rapid_nav_perception.flowvis import flow_to_rgb


def test_zero_flow_is_white():
    """No motion is white."""
    picture = flow_to_rgb(np.zeros((3, 4, 2), dtype=np.float32), max_magnitude=1.0)
    assert picture.shape == (3, 4, 3)
    assert picture.dtype == np.uint8
    assert np.all(picture == 255)


def test_opposite_flows_have_opposite_hues():
    """Right is red and left is cyan at full saturation."""
    right = np.zeros((1, 1, 2), dtype=np.float32)
    right[0, 0, 0] = 1.0
    left = np.zeros((1, 1, 2), dtype=np.float32)
    left[0, 0, 0] = -1.0
    assert tuple(flow_to_rgb(right, max_magnitude=1.0)[0, 0]) == (255, 0, 0)
    assert tuple(flow_to_rgb(left, max_magnitude=1.0)[0, 0]) == (0, 255, 255)


def test_default_magnitude_has_at_least_one_pixel():
    """Without max_magnitude, a tiny flow is still drawn at most at full saturation."""
    tiny = np.zeros((2, 2, 2), dtype=np.float32)
    tiny[..., 0] = 0.01
    picture = flow_to_rgb(tiny)
    assert picture.shape == (2, 2, 3)
    assert picture[0, 0, 0] > picture[0, 0, 1]
