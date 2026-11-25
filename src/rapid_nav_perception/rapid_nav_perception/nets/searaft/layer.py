"""SEA-RAFT layers, from core/layer.py, trimmed to the ones inference uses."""

import torch
import torch.nn as nn
import torch.nn.functional as F


class ConvNextBlock(nn.Module):
    """ConvNeXt block (DwConv, LayerNorm, Linear, GELU, Linear), then a 1x1 conv to output_dim."""

    def __init__(self, dim, output_dim, layer_scale_init_value=1e-6):
        """Build the block."""
        super().__init__()
        self.dwconv = nn.Conv2d(dim, dim, kernel_size=7, padding=3, groups=dim)
        self.norm = LayerNorm(dim, eps=1e-6)
        self.pwconv1 = nn.Linear(dim, 4 * output_dim)
        self.act = nn.GELU()
        self.pwconv2 = nn.Linear(4 * output_dim, dim)
        self.gamma = nn.Parameter(layer_scale_init_value * torch.ones((dim)),
                                  requires_grad=True) if layer_scale_init_value > 0 else None
        self.final = nn.Conv2d(dim, output_dim, kernel_size=1, padding=0)

    def forward(self, x):
        """Apply the block to x (N, C, H, W)."""
        skip = x
        x = self.dwconv(x)
        x = x.permute(0, 2, 3, 1)
        x = self.pwconv2(self.act(self.pwconv1(self.norm(x))))
        if self.gamma is not None:
            x = self.gamma * x
        x = x.permute(0, 3, 1, 2)
        return self.final(skip + x)


class LayerNorm(nn.Module):
    """LayerNorm over the last dimension (channels last)."""

    def __init__(self, normalized_shape, eps=1e-6):
        """Build the weight and bias."""
        super().__init__()
        self.weight = nn.Parameter(torch.ones(normalized_shape))
        self.bias = nn.Parameter(torch.zeros(normalized_shape))
        self.eps = eps
        self.normalized_shape = (normalized_shape, )

    def forward(self, x):
        """Normalise x over its last dimension."""
        return F.layer_norm(x, self.normalized_shape, self.weight, self.bias, self.eps)


def conv1x1(in_planes, out_planes, stride=1):
    """Return a 1x1 convolution without padding."""
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, padding=0)


def conv3x3(in_planes, out_planes, stride=1):
    """Return a 3x3 convolution with padding."""
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride, padding=1)


class BasicBlock(nn.Module):
    """Residual block of two 3x3 convolutions, with a strided 1x1 shortcut when needed."""

    def __init__(self, in_planes, planes, stride=1, norm_layer=nn.BatchNorm2d):
        """Build the block."""
        super().__init__()
        self.conv1 = conv3x3(in_planes, planes, stride)
        self.conv2 = conv3x3(planes, planes)
        self.bn1 = norm_layer(planes)
        self.bn2 = norm_layer(planes)
        self.relu = nn.ReLU(inplace=True)
        if stride == 1 and in_planes == planes:
            self.downsample = None
        else:
            # bn3 is registered twice (as bn3 and as downsample.1); checkpoints store it once
            self.bn3 = norm_layer(planes)
            self.downsample = nn.Sequential(conv1x1(in_planes, planes, stride=stride), self.bn3)

    def forward(self, x):
        """Apply the block to x."""
        y = self.relu(self.bn1(self.conv1(x)))
        y = self.relu(self.bn2(self.conv2(y)))
        if self.downsample is not None:
            x = self.downsample(x)
        return self.relu(x + y)
