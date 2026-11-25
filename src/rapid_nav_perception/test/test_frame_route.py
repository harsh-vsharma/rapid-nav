"""Tests of frame routing: the recheck gate with stub flows, and the real networks if present."""

import numpy as np
import pytest

from rapid_nav_perception import backends
from rapid_nav_perception.backends import FarnebackFlow, make_backend
from rapid_nav_perception.frame_route import DEFAULT_FLOWNETS, fast_share, FrameRouter

H, W = 48, 64


def uniform_flow(dx):
    """Return an (H, W, 2) flow that moves every pixel dx px to the right."""
    flow = np.zeros((H, W, 2), np.float32)
    flow[..., 0] = dx
    return flow


class Stub:
    """A flow callable that returns a fixed flow per call and counts its calls."""

    def __init__(self, speeds):
        """Return uniform_flow(speeds[i]) on the i-th call."""
        self.speeds = list(speeds)
        self.calls = 0

    def __call__(self, prev_rgb, curr_rgb):
        """Return the next flow."""
        flow = uniform_flow(self.speeds[self.calls])
        self.calls += 1
        return flow


def frames():
    """Return two blank images; the stubs ignore them."""
    return np.zeros((H, W, 3), np.uint8), np.zeros((H, W, 3), np.uint8)


def test_fast_share_counts_sampled_pixels_above_the_threshold():
    """Half the pixels moving 8 px, half 1 px: half of them are fast at 5 px."""
    flow = uniform_flow(1.0)
    flow[:, W // 2:, 0] = 8.0
    assert fast_share(flow, 5.0) == pytest.approx(0.5)
    assert fast_share(flow, 10.0) == 0.0


def test_first_pair_goes_to_searaft():
    """With no history the router is safe: SEA-RAFT runs, FlowNetS does not."""
    flownet, stream = Stub([]), Stub([1.0])
    router = FrameRouter(flownet, stream, disp_px=5.0, frac=0.3)
    router.compute(*frames())
    assert router.last_ran == 'searaft'
    assert (flownet.calls, stream.calls) == (0, 1)


def test_calm_frames_cost_one_flownets_pass():
    """After a calm SEA-RAFT frame, calm pairs run FlowNetS only."""
    flownet, stream = Stub([1.0, 2.0, 1.0]), Stub([1.0])
    router = FrameRouter(flownet, stream, disp_px=5.0, frac=0.3)
    for _ in range(4):
        flow = router.compute(*frames())
    assert router.last_ran == 'flownets'
    assert (flownet.calls, stream.calls) == (3, 1)
    assert flow[0, 0, 0] == 1.0


def test_motion_onset_reruns_the_same_pair_on_searaft():
    """A calm-predicted pair whose FlowNetS flow is fast is re-run: SEA-RAFT's flow is kept."""
    flownet, stream = Stub([1.0, 9.0]), Stub([1.0, 12.0])
    router = FrameRouter(flownet, stream, disp_px=5.0, frac=0.3)
    router.compute(*frames())                  # first pair: SEA-RAFT, calm
    router.compute(*frames())                  # FlowNetS, calm
    flow = router.compute(*frames())           # FlowNetS says fast: re-run
    assert router.rechecked
    assert router.last_ran == 'searaft'
    assert flow[0, 0, 0] == 12.0


def test_fast_frames_skip_flownets_until_motion_stops():
    """While SEA-RAFT's flow is fast, FlowNetS does not run; a calm SEA-RAFT flow hands back."""
    flownet, stream = Stub([3.0]), Stub([12.0, 12.0, 2.0])
    router = FrameRouter(flownet, stream, disp_px=5.0, frac=0.3)
    for _ in range(3):
        router.compute(*frames())
    assert (flownet.calls, stream.calls) == (0, 3)
    router.compute(*frames())
    assert router.last_ran == 'flownets'
    assert not router.rechecked


def test_frame_route_without_torch_falls_back_to_farneback(monkeypatch):
    """Without PyTorch, frame_route gives Farnebäck and the install warning."""
    monkeypatch.setattr(backends, 'torch_available', lambda: False)
    backend, warning = make_backend('frame_route')
    assert isinstance(backend, FarnebackFlow)
    assert 'frame_route' in warning


def test_frame_route_recovers_the_shift():
    """With PyTorch and both checkpoints, the routed flow recovers a (+3, -2) px shift."""
    pytest.importorskip('torch')
    if not DEFAULT_FLOWNETS.is_file():
        pytest.skip('FlowNetS checkpoint not downloaded')
    from test_backends import interior_median, SHIFT_X, SHIFT_Y, textured_pair

    try:
        backend, warning = make_backend('frame_route', device='cpu')
    except OSError as error:       # SEA-RAFT checkpoint not cached and no network
        pytest.skip('SEA-RAFT checkpoint unavailable: {}'.format(error))
    assert warning is None
    previous, current = textured_pair()
    for expected in ('frame_route:searaft', 'frame_route:flownets'):
        flow = backend.compute(previous, current)
        assert backend.last_ran == expected
        assert flow.shape == (240, 320, 2)
        median = interior_median(flow)
        assert abs(median[0] - SHIFT_X) < 0.5
        assert abs(median[1] - SHIFT_Y) < 0.5
