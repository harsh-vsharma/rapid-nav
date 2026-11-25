"""
SEA-RAFT encoder, from core/extractor.py.

Change from upstream: no weight initialisation (upstream loads ImageNet ResNet weights for
training; the checkpoint overwrites every weight at inference).
"""

from rapid_nav_perception.nets.searaft.layer import BasicBlock, conv1x1
import torch.nn as nn


class ResNetFPN(nn.Module):
    """ResNet18/34 trunk without max-pool, output at 1/8 resolution."""

    def __init__(self, args, input_dim=3, output_dim=256, norm_layer=nn.BatchNorm2d):
        """Build the trunk for args.pretrain ('resnet18' or 'resnet34')."""
        super().__init__()
        block_dims = list(args.block_dims)
        initial_dim = args.initial_dim
        self.input_dim = input_dim
        self.in_planes = initial_dim
        self.conv1 = nn.Conv2d(input_dim, initial_dim, kernel_size=7, stride=2, padding=3)
        self.bn1 = norm_layer(initial_dim)
        self.relu = nn.ReLU(inplace=True)
        if args.pretrain == 'resnet34':
            n_block = [3, 4, 6]
        elif args.pretrain == 'resnet18':
            n_block = [2, 2, 2]
        else:
            raise NotImplementedError(args.pretrain)
        self.layer1 = self._make_layer(block_dims[0], 1, norm_layer, n_block[0])  # 1/2
        self.layer2 = self._make_layer(block_dims[1], 2, norm_layer, n_block[1])  # 1/4
        self.layer3 = self._make_layer(block_dims[2], 2, norm_layer, n_block[2])  # 1/8
        self.final_conv = conv1x1(block_dims[2], output_dim)

    def _make_layer(self, dim, stride, norm_layer, num):
        layers = [BasicBlock(self.in_planes, dim, stride=stride, norm_layer=norm_layer)]
        for _ in range(num - 1):
            layers.append(BasicBlock(dim, dim, stride=1, norm_layer=norm_layer))
        self.in_planes = dim
        return nn.Sequential(*layers)

    def forward(self, x):
        """Return the 1/8-resolution features of x."""
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        return self.final_conv(x)
