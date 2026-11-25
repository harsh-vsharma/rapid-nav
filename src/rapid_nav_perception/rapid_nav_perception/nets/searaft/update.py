"""SEA-RAFT update block, from core/update.py (the unused FlowHead class dropped)."""

from rapid_nav_perception.nets.searaft.layer import ConvNextBlock
import torch
import torch.nn as nn
import torch.nn.functional as F


class BasicMotionEncoder(nn.Module):
    """Encode the correlation lookup and the current flow into motion features."""

    def __init__(self, args, dim=128):
        """Build the convolutions."""
        super().__init__()
        cor_planes = args.corr_channel
        self.convc1 = nn.Conv2d(cor_planes, dim * 2, 1, padding=0)
        self.convc2 = nn.Conv2d(dim * 2, dim + dim // 2, 3, padding=1)
        self.convf1 = nn.Conv2d(2, dim, 7, padding=3)
        self.convf2 = nn.Conv2d(dim, dim // 2, 3, padding=1)
        self.conv = nn.Conv2d(dim * 2, dim - 2, 3, padding=1)

    def forward(self, flow, corr):
        """Return the motion features, with the flow appended."""
        cor = F.relu(self.convc2(F.relu(self.convc1(corr))))
        flo = F.relu(self.convf2(F.relu(self.convf1(flow))))
        out = F.relu(self.conv(torch.cat([cor, flo], dim=1)))
        return torch.cat([out, flow], dim=1)


class BasicUpdateBlock(nn.Module):
    """Update the hidden state from the context, the correlation and the flow."""

    def __init__(self, args, hdim=128, cdim=128):
        """Build the motion encoder and the ConvNeXt refinement blocks."""
        super().__init__()
        self.args = args
        self.encoder = BasicMotionEncoder(args, dim=cdim)
        self.refine = nn.ModuleList(
            [ConvNextBlock(2 * cdim + hdim, hdim) for _ in range(args.num_blocks)])

    def forward(self, net, inp, corr, flow):
        """Return the new hidden state."""
        motion_features = self.encoder(flow, corr)
        inp = torch.cat([inp, motion_features], dim=1)
        for blk in self.refine:
            net = blk(torch.cat([net, inp], dim=1))
        return net
