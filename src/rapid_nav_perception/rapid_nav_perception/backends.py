"""
Optical flow backends behind one interface.

FarnebackFlow is OpenCV's dense flow and always available. RaftFlow is torchvision's RAFT and needs
PyTorch; PyTorch is imported only when a RAFT backend is built, so the module imports without it.
"""

import numpy as np

RAFT_VARIANTS = ('raft_small', 'raft_large')
NO_TORCH_WARNING = (
    'PyTorch not found: using farneback. '
    'Install it as in docs/perception.md to use raft_small or raft_large.'
)


class FlowBackend:
    """Base class: compute(prev_rgb, curr_rgb) gives the flow from the first to the second."""

    name = 'base'

    def compute(self, prev_rgb, curr_rgb):
        """Return the flow (H, W, 2) float32, in pixels, x right and y down."""
        raise NotImplementedError


class FarnebackFlow(FlowBackend):
    """Dense optical flow by Farnebäck's method (OpenCV), on grey images."""

    name = 'farneback'

    def compute(self, prev_rgb, curr_rgb):
        """Return the Farnebäck flow of two RGB images."""
        import cv2

        prev_grey = cv2.cvtColor(np.ascontiguousarray(prev_rgb), cv2.COLOR_RGB2GRAY)
        curr_grey = cv2.cvtColor(np.ascontiguousarray(curr_rgb), cv2.COLOR_RGB2GRAY)
        flow = cv2.calcOpticalFlowFarneback(
            prev_grey, curr_grey, None, 0.5, 3, 15, 3, 5, 1.2, 0)
        return flow.astype(np.float32)


def torch_available():
    """Return True when PyTorch can be imported."""
    try:
        import torch  # noqa: F401
    except ImportError:
        return False
    return True


def resolve_device(device):
    """Return 'cuda' or 'cpu'. 'auto' picks cuda when PyTorch sees a GPU."""
    if device == 'auto':
        if not torch_available():
            return 'cpu'
        import torch

        return 'cuda' if torch.cuda.is_available() else 'cpu'
    if device not in ('cpu', 'cuda'):
        raise ValueError("device must be 'auto', 'cpu' or 'cuda', got {!r}".format(device))
    return device


class RaftFlow(FlowBackend):
    """Dense optical flow by RAFT (torchvision): raft_small or raft_large."""

    def __init__(self, variant, device, weights_path=None, iterations=12, num_threads=0):
        """Build the model; default torchvision weights, or the state dict at weights_path."""
        import torch
        from torchvision.models.optical_flow import (
            Raft_Large_Weights,
            Raft_Small_Weights,
            raft_large,
            raft_small,
        )

        if variant not in RAFT_VARIANTS:
            raise ValueError('unknown RAFT variant {!r}'.format(variant))
        self.name = variant
        self.iterations = int(iterations)
        if num_threads > 0:
            torch.set_num_threads(int(num_threads))
        self.device = resolve_device(device)
        if variant == 'raft_small':
            builder, weights = raft_small, Raft_Small_Weights.DEFAULT
        else:
            builder, weights = raft_large, Raft_Large_Weights.DEFAULT
        if weights_path:
            model = builder(weights=None)
            model.load_state_dict(torch.load(weights_path, map_location='cpu'))
        else:
            model = builder(weights=weights)
        self.transforms = weights.transforms()
        self.model = model.eval().to(self.device)

    def compute(self, prev_rgb, curr_rgb):
        """Return the RAFT flow of two RGB images whose height and width are multiples of 8."""
        import torch

        height, width = prev_rgb.shape[:2]
        if height % 8 or width % 8:
            raise ValueError('RAFT needs a height and width divisible by 8, got {}x{}'.format(
                width, height))
        first = torch.from_numpy(np.ascontiguousarray(prev_rgb)).permute(2, 0, 1).unsqueeze(0)
        second = torch.from_numpy(np.ascontiguousarray(curr_rgb)).permute(2, 0, 1).unsqueeze(0)
        first, second = self.transforms(first, second)
        with torch.inference_mode():
            predictions = self.model(first.to(self.device), second.to(self.device),
                                     num_flow_updates=self.iterations)
        flow = predictions[-1][0].permute(1, 2, 0).cpu().numpy()
        return flow.astype(np.float32)


def make_backend(name, device='auto', weights_path=None, iterations=12, num_threads=0,
                 logger=None):
    """
    Return (backend, warning) for a backend name.

    A RAFT name without PyTorch gives FarnebackFlow and the warning text; the warning is also
    logged when a logger is given. Unknown names raise ValueError.
    """
    if name == 'farneback':
        return FarnebackFlow(), None
    if name in RAFT_VARIANTS:
        if not torch_available():
            if logger is not None:
                logger.warning(NO_TORCH_WARNING)
            return FarnebackFlow(), NO_TORCH_WARNING
        return RaftFlow(name, device, weights_path=weights_path, iterations=iterations,
                        num_threads=num_threads), None
    raise ValueError('unknown flow backend {!r}; use farneback, raft_small or raft_large'.format(
        name))
