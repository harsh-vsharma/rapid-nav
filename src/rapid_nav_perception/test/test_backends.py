"""Tests of the flow backends on a shifted textured image, and of the backend choice."""

import cv2
import numpy as np
import pytest

from rapid_nav_perception import backends
from rapid_nav_perception.backends import FarnebackFlow, make_backend

SHIFT_X, SHIFT_Y = 3, -2


def textured_pair():
    """Return (previous, current) RGB images, the current one shifted by (+3, -2) pixels."""
    rng = np.random.default_rng(7)
    noise = rng.random((240, 320)).astype(np.float32)
    smooth = cv2.GaussianBlur(noise, (0, 0), 1.5)
    smooth = (smooth - smooth.min()) / (smooth.max() - smooth.min())
    grey = (smooth * 255.0).astype(np.uint8)
    previous = np.stack([grey, grey, grey], axis=-1)
    current = np.roll(previous, shift=(SHIFT_Y, SHIFT_X), axis=(0, 1))
    return previous, current


def interior_median(flow, margin=10):
    """Return the median flow (x, y) in the interior of the image."""
    inner = flow[margin:-margin, margin:-margin]
    return np.median(inner.reshape(-1, 2), axis=0)


def test_farneback_recovers_the_shift():
    """Farnebäck's median flow in the interior is within 0.3 px of the true shift."""
    previous, current = textured_pair()
    flow = FarnebackFlow().compute(previous, current)
    assert flow.shape == (240, 320, 2)
    assert flow.dtype == np.float32
    median = interior_median(flow)
    assert abs(median[0] - SHIFT_X) < 0.3
    assert abs(median[1] - SHIFT_Y) < 0.3


def test_raft_small_recovers_the_shift():
    """Check that RAFT-Small's interior median is within 0.3 px of the shift (needs PyTorch)."""
    pytest.importorskip('torch')
    previous, current = textured_pair()
    backend, warning = make_backend('raft_small', device='cpu')
    assert warning is None
    flow = backend.compute(previous, current)
    assert flow.shape == (240, 320, 2)
    median = interior_median(flow)
    assert abs(median[0] - SHIFT_X) < 0.3
    assert abs(median[1] - SHIFT_Y) < 0.3


def test_raft_without_torch_falls_back_to_farneback(monkeypatch):
    """Without PyTorch, a RAFT name gives Farnebäck and the install warning."""
    monkeypatch.setattr(backends, 'torch_available', lambda: False)
    backend, warning = make_backend('raft_small')
    assert isinstance(backend, FarnebackFlow)
    assert warning == backends.NO_TORCH_WARNING
    assert 'docs/perception.md' in warning


def test_unknown_backend_raises():
    """A backend name that does not exist is an error."""
    with pytest.raises(ValueError):
        make_backend('tvl1')


def test_farneback_backend_has_no_warning():
    """Asking for farneback gives no warning."""
    backend, warning = make_backend('farneback')
    assert backend.name == 'farneback'
    assert warning is None
