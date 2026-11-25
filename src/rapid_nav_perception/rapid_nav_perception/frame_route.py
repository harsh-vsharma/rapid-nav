"""
Frame routing: FlowNetS on calm frames, streaming SEA-RAFT-S on fast ones (gate 'recheck').

FlowNetS is fast and good on small motion (sub-pixel to about 1.5 px error on calm and
mid-motion scenes) but 10 px or more off on fast motion. SEA-RAFT is accurate everywhere but
several times slower. The router sends whole frames, not pixels or tiles, to one of the two:

* A frame counts as fast when more than route_frac of its sampled pixels (every step px)
  move more than route_disp_px.
* While the scene is calm each pair costs one FlowNetS pass, and its own flow decides.
  When that flow shows fast motion (motion onset), the same pair is re-run on SEA-RAFT, so
  no fast frame keeps FlowNetS's flow.
* While the scene is fast each pair costs one SEA-RAFT pass (warm start, 2 iterations), not
  both: SEA-RAFT's own flow decides whether the next pair goes back to FlowNetS.

Benchmark (54 MPI-Sintel clean pairs at 640x480, CPU; route_disp_px 10 there): EPE 2.03 px at
352 ms per pair, against 1.61 px at 620 ms for SEA-RAFT-S on every frame and 6.24 px at
156 ms for FlowNetS alone. The router itself is plain numpy; the networks need PyTorch.
"""

import os
from pathlib import Path

import numpy as np
from rapid_nav_perception.backends import FlowBackend, resolve_device

DEFAULT_SEARAFT = 'Tartan-C-T-TSKH432x960-S'
WEIGHTS_DIR = Path(os.environ.get('RAPID_NAV_WEIGHTS', '~/.cache/rapid_nav')).expanduser()
DEFAULT_FLOWNETS = WEIGHTS_DIR / 'flownets' / 'flownets_EPE1.951.pth.tar'


def fast_share(flow, disp_px, step=4):
    """Return the share of sampled pixels (every step px) whose flow is above disp_px."""
    sampled = flow[::step, ::step]
    return float((np.hypot(sampled[..., 0], sampled[..., 1]) > disp_px).mean())


class FrameRouter:
    """
    Route whole frames between a fast and an accurate flow, with the 'recheck' gate.

    flownet and stream are callables (prev_rgb, curr_rgb) -> flow (H, W, 2). After each
    compute, last_ran is 'flownets' or 'searaft' and rechecked says whether FlowNetS ran first
    and its flow was replaced (motion onset).
    """

    def __init__(self, flownet, stream, disp_px=5.0, frac=0.3, step=4):
        """Keep the two flows and the gate settings; the first pair goes to SEA-RAFT."""
        self.flownet = flownet
        self.stream = stream
        self.disp_px = float(disp_px)
        self.frac = float(frac)
        self.step = int(step)
        self.hard = True              # no history yet: be safe
        self.last_ran = None
        self.rechecked = False

    def compute(self, prev_rgb, curr_rgb):
        """Return the flow from prev_rgb to curr_rgb and update the routing state."""
        self.rechecked = False
        if not self.hard:
            flow = self.flownet(prev_rgb, curr_rgb)
            if fast_share(flow, self.disp_px, self.step) <= self.frac:
                self.last_ran = 'flownets'
                return flow
            self.rechecked = True     # motion onset: re-run this same pair on SEA-RAFT
        flow = self.stream(prev_rgb, curr_rgb)
        self.last_ran = 'searaft'
        self.hard = fast_share(flow, self.disp_px, self.step) > self.frac
        return flow


class FlowNetSRunner:
    """FlowNetS on HxWx3 uint8 pairs; the image size is padded to a multiple of 64."""

    MEAN = (0.45, 0.432, 0.411)
    DIV_FLOW = 20.0

    def __init__(self, model, device):
        """Wrap a loaded FlowNetS model."""
        import torch

        self.model = model.to(device).eval()
        self.device = device
        self.mean = torch.tensor(self.MEAN, device=device).view(1, 3, 1, 1)

    def __call__(self, prev_rgb, curr_rgb):
        """Return the flow (H, W, 2) float32 from prev_rgb to curr_rgb."""
        import torch
        import torch.nn.functional as F

        def tensor(rgb):
            x = torch.from_numpy(np.ascontiguousarray(rgb)).to(self.device)
            return x.permute(2, 0, 1)[None].float().div_(255.0) - self.mean

        height, width = prev_rgb.shape[:2]
        x = torch.cat([tensor(prev_rgb), tensor(curr_rgb)], 1)
        ph, pw = (-height) % 64, (-width) % 64
        if ph or pw:
            x = F.pad(x, (0, pw, 0, ph), mode='replicate')
        with torch.inference_mode():
            out = self.model(x)
        # 1/4-resolution output, already in full-resolution pixels after DIV_FLOW
        out = F.interpolate(out, size=x.shape[-2:], mode='bilinear',
                            align_corners=False) * self.DIV_FLOW
        return out[0, :, :height, :width].permute(1, 2, 0).cpu().numpy().astype(np.float32)


class FrameRouteFlow(FlowBackend):
    """The frame_route flow backend: FrameRouter over FlowNetS and streaming SEA-RAFT."""

    name = 'frame_route'

    def __init__(self, device, flownet_weights='', searaft_weights=DEFAULT_SEARAFT,
                 disp_px=5.0, frac=0.3, stream_iters=2, key_iters=4, num_threads=0):
        """Load both networks; flownet_weights '' means DEFAULT_FLOWNETS."""
        from rapid_nav_perception.nets.flownets import load_flownets
        from rapid_nav_perception.searaft import load_searaft, StreamingSeaRaft
        import torch

        if num_threads > 0:
            torch.set_num_threads(int(num_threads))
        self.device = resolve_device(device)
        path = Path(flownet_weights).expanduser() if flownet_weights else DEFAULT_FLOWNETS
        if not path.is_file():
            raise FileNotFoundError(
                'FlowNetS checkpoint not found at {}: download flownets_EPE1.951.pth.tar as in '
                'docs/perception.md, or set flownet_weights'.format(path))
        flownet = FlowNetSRunner(load_flownets(path), self.device)
        stream = StreamingSeaRaft(load_searaft(searaft_weights, self.device), self.device,
                                  iters=stream_iters, key_iters=key_iters)
        self.router = FrameRouter(flownet, stream.compute, disp_px=disp_px, frac=frac)

    @property
    def last_ran(self):
        """Return 'frame_route:flownets' or 'frame_route:searaft' for the last pair."""
        return '{}:{}'.format(self.name, self.router.last_ran or 'none')

    def compute(self, prev_rgb, curr_rgb):
        """Return the routed flow (H, W, 2) float32, in pixels, x right and y down."""
        return self.router.compute(prev_rgb, curr_rgb)
