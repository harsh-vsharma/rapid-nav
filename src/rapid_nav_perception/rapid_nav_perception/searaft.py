"""
SEA-RAFT inference for the frame_route backend: checkpoints, the staged loop, streaming.

SEA-RAFT on a pair (Wang, Lipson and Deng, ECCV 2024): the context encoder sees both frames
and gives the hidden state and a regressed initial flow; the feature encoder sees each frame;
an all-pairs correlation pyramid at 1/8 resolution is looked up around the current flow for a
few update steps; the flow is upsampled to full resolution at the end.

StreamingSeaRaft runs it on a video: each frame goes through the feature encoder once (its
features are reused when it becomes the first frame of the next pair), and each pair starts
from the previous flow pushed one frame forward (warm start). Same weights and the same
results as the upstream forward within float rounding when warm start is off.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import urllib.request

import numpy as np
from rapid_nav_perception.nets.searaft import searaft_args, SeaRaftNet
import torch
import torch.nn.functional as F

HF_URL = 'https://huggingface.co/MemorySlices/{name}/resolve/main/model.safetensors'
CACHE_DIR = Path(os.environ.get('RAPID_NAV_WEIGHTS', '~/.cache/rapid_nav')).expanduser()

# Checkpoint name (its HuggingFace repo) -> SHA-256 of model.safetensors.
# C-T: TartanAir, FlyingChairs, FlyingThings. C-T-TSKH: also fine-tuned on Things, Sintel,
# KITTI and HD1K, the better choice on a robot.
ZOO = {
    'Tartan-C-T432x960-S': '494a520b1f79f2f3bb14356255095f22a00045611b8d611601aa2ba9f4da3951',
    'Tartan-C-T432x960-M': '41f9a087377a4d3578e57e461cc07642a41dfe5fde992215d4d26288ddba27c7',
    'Tartan-C-T-TSKH432x960-S': '9752a49fca0a3f33551d51fec7bc5bd87f93a155b1e9fadd803e8176f4edb423',
    'Tartan-C-T-TSKH432x960-M': '83e4319d1029f414a342227db17f64089f21b7324da085122beb008aa9c4d0e5',
}

_ST_DTYPES = {'F32': torch.float32, 'F16': torch.float16, 'BF16': torch.bfloat16,
              'I64': torch.int64}


def read_safetensors(path):
    """
    Read a .safetensors file into a state dict, without the safetensors package.

    Layout: an 8-byte little-endian header length, a JSON header, then the raw tensor data.
    Tensors shared in the saved model are stored once; __metadata__ maps the dropped names to
    the stored ones and both names are restored.
    """
    buf = bytearray(Path(path).read_bytes())
    n = int.from_bytes(buf[:8], 'little')
    header = json.loads(buf[8:8 + n].decode('utf-8'))
    meta = header.pop('__metadata__', None) or {}
    out = {}
    for name, info in header.items():
        dtype = _ST_DTYPES[info['dtype']]
        start, end = info['data_offsets']
        count = (end - start) // torch.empty((), dtype=dtype).element_size()
        tensor = (torch.frombuffer(buf, dtype=dtype, count=count, offset=8 + n + start)
                  if count else torch.empty(0, dtype=dtype))
        out[name] = tensor.reshape(info['shape'])
    for alias, target in meta.items():
        if alias not in out and target in out:
            out[alias] = out[target]
    return out


def sha256(path):
    """Return the SHA-256 hex digest of a file."""
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_checkpoint(spec, cache_dir=CACHE_DIR):
    """
    Return the path of a SEA-RAFT checkpoint given as a file path or a ZOO name.

    A ZOO checkpoint missing from cache_dir is downloaded from HuggingFace once and checked
    against its SHA-256.
    """
    path = Path(spec).expanduser()
    if path.is_file():
        return path
    if spec not in ZOO:
        raise FileNotFoundError(
            '{!r} is neither a file nor a SEA-RAFT checkpoint name ({})'.format(
                spec, ', '.join(ZOO)))
    path = Path(cache_dir) / 'searaft' / '{}.safetensors'.format(spec)
    if path.is_file():
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_suffix('.part')
    urllib.request.urlretrieve(HF_URL.format(name=spec), part)
    got = sha256(part)
    if got != ZOO[spec]:
        part.unlink()
        raise RuntimeError('checksum mismatch for {}: got {}, expected {}'.format(
            spec, got, ZOO[spec]))
    os.replace(part, path)
    return path


def load_searaft(spec, device='cpu'):
    """Return SeaRaftNet in eval mode with a checkpoint; the -S or -M name suffix picks it."""
    path = resolve_checkpoint(spec)
    variant = path.stem.rsplit('-', 1)[-1]
    if variant not in ('S', 'M'):
        raise ValueError('the SEA-RAFT checkpoint name must end in -S or -M, got {!r}'.format(
            path.name))
    net = SeaRaftNet(searaft_args(variant))
    if path.suffix == '.safetensors':
        state = read_safetensors(path)
    else:
        state = torch.load(path, map_location='cpu', weights_only=True)
        state = {k.removeprefix('module.'): v for k, v in state.items()}
    net.load_state_dict(state, strict=True)
    return net.to(device).eval()


def forward_splat(flow):
    """Push flow (B, 2, h, w) one frame forward: each value is splatted to where it lands."""
    batch, _, h, w = flow.shape
    ys, xs = torch.meshgrid(torch.arange(h, device=flow.device, dtype=flow.dtype),
                            torch.arange(w, device=flow.device, dtype=flow.dtype),
                            indexing='ij')
    x = xs + flow[:, 0]
    y = ys + flow[:, 1]
    x0, y0 = x.floor(), y.floor()
    out = torch.zeros(batch, 2, h * w, device=flow.device, dtype=flow.dtype)
    wsum = torch.zeros(batch, 1, h * w, device=flow.device, dtype=flow.dtype)
    src = flow.reshape(batch, 2, -1)
    for dx in (0, 1):
        for dy in (0, 1):
            xi, yi = x0 + dx, y0 + dy
            weight = (1 - (x - xi).abs()) * (1 - (y - yi).abs())
            inside = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h) & (weight > 0)
            weight = (weight * inside).reshape(batch, 1, -1)
            index = (yi.clamp(0, h - 1) * w + xi.clamp(0, w - 1)).long().reshape(batch, 1, -1)
            out.scatter_add_(2, index.expand(-1, 2, -1), src * weight)
            wsum.scatter_add_(2, index, weight)
    out = torch.where(wsum > 1e-3, out / wsum.clamp_min(1e-6), src)   # holes keep their flow
    return out.reshape(batch, 2, h, w)


class StreamingSeaRaft:
    """SEA-RAFT on a stream of frames: one feature pass per frame, warm-started iterations."""

    def __init__(self, net, device, iters=2, key_iters=4):
        """Wrap a loaded SeaRaftNet; iters per pair, key_iters on the first pair of a stream."""
        self.net = net
        self.device = torch.device(device)
        self.iters = int(iters)
        self.key_iters = int(key_iters)
        self.amp = self.device.type == 'cuda'
        self.dim = net.args.dim
        self.levels = net.args.corr_levels
        r = net.args.corr_radius
        d = torch.linspace(-r, r, 2 * r + 1, device=self.device)
        # (dy, dx) order, as upstream: the motion encoder was trained with it
        self.delta = torch.stack(torch.meshgrid(d, d, indexing='ij'), -1).view(
            1, 2 * r + 1, 2 * r + 1, 2)
        if self.amp:
            torch.backends.cudnn.benchmark = True
        self.reset()

    def reset(self):
        """Forget the cached frame, features and flow (the next pair is a keyframe)."""
        self.last_rgb = None
        self.last_x = None
        self.last_features = None
        self.last_flow8 = None

    def autocast(self):
        """Return the fp16 autocast context on CUDA (a no-op on the CPU)."""
        return torch.autocast(device_type=self.device.type, dtype=torch.float16,
                              enabled=self.amp)

    @staticmethod
    def pads(height, width):
        """Return (left, right, top, bottom) replicate padding to a multiple of 8, centred."""
        ph, pw = (-height) % 8, (-width) % 8
        return pw // 2, pw - pw // 2, ph // 2, ph - ph // 2

    def prepare(self, rgb):
        """Return an HxWx3 uint8 image as a padded (1, 3, H, W) tensor in [-1, 1]."""
        x = torch.from_numpy(np.ascontiguousarray(rgb)).to(self.device)
        x = x.permute(2, 0, 1)[None].float().div_(127.5).sub_(1.0)
        pad = self.pads(*x.shape[-2:])
        return F.pad(x, pad, mode='replicate') if any(pad) else x

    def features(self, x):
        """Return the feature encoder output (fp32)."""
        with self.autocast():
            return self.net.fnet(x).float()

    def pyramid(self, f1, f2):
        """Return the all-pairs correlation of f1 with f2 at 4 scales of f2 (fp32)."""
        batch, channels, h, w = f1.shape
        a = f1.reshape(batch, channels, h * w).transpose(1, 2)
        scale = 1.0 / math.sqrt(channels)
        levels = []
        for i in range(self.levels):
            h2, w2 = f2.shape[-2:]
            corr = torch.bmm(a, f2.reshape(batch, channels, h2 * w2)) * scale
            levels.append(corr.reshape(batch * h * w, 1, h2, w2))
            if i + 1 < self.levels:
                f2 = F.interpolate(f2, scale_factor=0.5, mode='bilinear', align_corners=False)
        return levels

    def lookup(self, levels, coords):
        """Sample a (2r+1)^2 window around coords (B, 2, h, w) at every pyramid level."""
        batch, _, h, w = coords.shape
        c = coords.permute(0, 2, 3, 1).reshape(batch * h * w, 1, 1, 2)
        out = []
        for i, corr in enumerate(levels):
            h2, w2 = corr.shape[-2:]
            norm = torch.tensor([2.0 / max(w2 - 1, 1), 2.0 / max(h2 - 1, 1)], device=self.device)
            grid = (c / 2 ** i + self.delta) * norm - 1
            sample = F.grid_sample(corr, grid, mode='bilinear', padding_mode='zeros',
                                   align_corners=True)
            out.append(sample.view(batch, h, w, -1))
        return torch.cat(out, -1).permute(0, 3, 1, 2).contiguous()

    def upsample(self, hidden, flow8):
        """Return the convex 8x upsampling of flow8 with the mask from the hidden state."""
        with self.autocast():
            mask = self.net.upsample_weight(hidden)
        batch, _, h, w = flow8.shape
        mask = (0.25 * mask.float()).view(batch, 1, 9, 8, 8, h, w).softmax(dim=2)
        up = F.unfold(8 * flow8, [3, 3], padding=1).view(batch, 2, 9, 1, 1, h, w)
        up = (mask * up).sum(dim=2)
        return up.permute(0, 1, 4, 2, 5, 3).reshape(batch, 2, 8 * h, 8 * w)

    @torch.inference_mode()
    def compute(self, prev_rgb, curr_rgb):
        """Return the flow (H, W, 2) float32 from prev_rgb to curr_rgb (HxWx3 uint8)."""
        height, width = curr_rgb.shape[:2]
        cont = (self.last_rgb is not None and self.last_rgb.shape == prev_rgb.shape
                and np.array_equal(self.last_rgb, prev_rgb))
        x2 = self.prepare(curr_rgb)
        if cont:
            x1, f1 = self.last_x, self.last_features
            f2 = self.features(x2)
        else:                                       # new stream: encode both frames
            self.reset()
            x1 = self.prepare(prev_rgb)
            f1, f2 = self.features(x1), self.features(x2)
        with self.autocast():
            context = self.net.init_conv(self.net.cnet(torch.cat([x1, x2], 1)))
        hidden, context = context.split([self.dim, self.dim], 1)
        if cont and self.last_flow8 is not None:
            flow8 = forward_splat(self.last_flow8)
        else:
            with self.autocast():
                flow8 = self.net.flow_head(hidden)[:, :2].float()
        levels = self.pyramid(f1, f2)
        h8, w8 = flow8.shape[-2:]
        ys, xs = torch.meshgrid(torch.arange(h8, device=self.device, dtype=torch.float32),
                                torch.arange(w8, device=self.device, dtype=torch.float32),
                                indexing='ij')
        grid = torch.stack([xs, ys])[None]
        for _ in range(self.iters if cont else self.key_iters):
            corr = self.lookup(levels, grid + flow8)
            with self.autocast():
                hidden = self.net.update_block(hidden, context, corr, flow8)
                flow8 = flow8 + self.net.flow_head(hidden)[:, :2].float()
        flow = self.upsample(hidden, flow8)
        left, _, top, _ = self.pads(height, width)
        flow = flow[..., top:top + height, left:left + width]
        self.last_rgb, self.last_x, self.last_features, self.last_flow8 = (
            curr_rgb, x2, f2, flow8)
        return flow[0].permute(1, 2, 0).float().cpu().numpy()
