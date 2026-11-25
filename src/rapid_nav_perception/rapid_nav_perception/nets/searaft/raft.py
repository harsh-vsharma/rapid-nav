"""
SEA-RAFT network, from core/raft.py.

Changes from upstream: class RAFT renamed SeaRaftNet, no HuggingFace mixin, no forward (the
staged inference loop is in rapid_nav_perception/searaft.py), no ImageNet download.
"""

from types import SimpleNamespace

from rapid_nav_perception.nets.searaft.extractor import ResNetFPN
from rapid_nav_perception.nets.searaft.layer import conv3x3
from rapid_nav_perception.nets.searaft.update import BasicUpdateBlock
import torch.nn as nn


def searaft_args(variant='S'):
    """
    Return the hyper-parameters of the released S or M network.

    They differ only in the encoder trunk: ResNet18 (S) or ResNet34 (M).
    """
    backbone = {'S': 'resnet18', 'M': 'resnet34'}
    if variant not in backbone:
        raise ValueError('unknown SEA-RAFT variant {!r}, expected S or M'.format(variant))
    return SimpleNamespace(
        variant=variant, pretrain=backbone[variant],
        dim=128, radius=4, initial_dim=64, block_dims=[64, 128, 256], num_blocks=2,
        iters=4, use_var=True, var_min=0, var_max=10)


class SeaRaftNet(nn.Module):
    """The SEA-RAFT modules: context and feature encoders, update block and heads."""

    def __init__(self, args):
        """Build the modules for the given hyper-parameters."""
        super().__init__()
        self.args = args
        self.output_dim = args.dim * 2
        self.args.corr_levels = 4
        self.args.corr_radius = args.radius
        self.args.corr_channel = args.corr_levels * (args.radius * 2 + 1) ** 2
        self.cnet = ResNetFPN(args, input_dim=6, output_dim=2 * self.args.dim,
                              norm_layer=nn.BatchNorm2d)
        self.init_conv = conv3x3(2 * args.dim, 2 * args.dim)
        self.upsample_weight = nn.Sequential(
            nn.Conv2d(args.dim, args.dim * 2, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(args.dim * 2, 64 * 9, 1, padding=0),
        )
        self.flow_head = nn.Sequential(
            # flow (2) + mixture weights (2) + log scales (2)
            nn.Conv2d(args.dim, 2 * args.dim, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(2 * args.dim, 6, 3, padding=1),
        )
        self.fnet = ResNetFPN(args, input_dim=3, output_dim=self.output_dim,
                              norm_layer=nn.BatchNorm2d)
        self.update_block = BasicUpdateBlock(args, hdim=args.dim, cdim=args.dim)
