"""
FlowNetS (Dosovitskiy et al., ICCV 2015), the architecture only, for inference.

From ClementPinard/FlowNetPytorch (MIT licence), models/FlowNetS.py and models/util.py, without
the batch-norm variant, the FlowNetC correlation and the training outputs. The module names are
unchanged, so the published checkpoint flownets_EPE1.951.pth.tar loads strictly.
"""

import torch
import torch.nn as nn


def conv(in_planes, out_planes, kernel_size=3, stride=1):
    """Return a convolution with 'same' padding and a leaky ReLU."""
    return nn.Sequential(
        nn.Conv2d(in_planes, out_planes, kernel_size=kernel_size, stride=stride,
                  padding=(kernel_size - 1) // 2, bias=True),
        nn.LeakyReLU(0.1, inplace=True),
    )


def predict_flow(in_planes):
    """Return the 3x3 convolution that predicts a two-channel flow."""
    return nn.Conv2d(in_planes, 2, kernel_size=3, stride=1, padding=1, bias=False)


def deconv(in_planes, out_planes):
    """Return a stride-2 transposed convolution with a leaky ReLU."""
    return nn.Sequential(
        nn.ConvTranspose2d(in_planes, out_planes, kernel_size=4, stride=2, padding=1, bias=False),
        nn.LeakyReLU(0.1, inplace=True),
    )


def crop_like(tensor, target):
    """Crop tensor to the height and width of target."""
    return tensor[:, :, :target.size(2), :target.size(3)]


class FlowNetS(nn.Module):
    """FlowNetS without batch norm; forward gives the flow at 1/4 resolution, divided by 20."""

    def __init__(self):
        """Build the layers (weights come from the checkpoint)."""
        super().__init__()
        self.conv1 = conv(6, 64, kernel_size=7, stride=2)
        self.conv2 = conv(64, 128, kernel_size=5, stride=2)
        self.conv3 = conv(128, 256, kernel_size=5, stride=2)
        self.conv3_1 = conv(256, 256)
        self.conv4 = conv(256, 512, stride=2)
        self.conv4_1 = conv(512, 512)
        self.conv5 = conv(512, 512, stride=2)
        self.conv5_1 = conv(512, 512)
        self.conv6 = conv(512, 1024, stride=2)
        self.conv6_1 = conv(1024, 1024)
        self.deconv5 = deconv(1024, 512)
        self.deconv4 = deconv(1026, 256)
        self.deconv3 = deconv(770, 128)
        self.deconv2 = deconv(386, 64)
        self.predict_flow6 = predict_flow(1024)
        self.predict_flow5 = predict_flow(1026)
        self.predict_flow4 = predict_flow(770)
        self.predict_flow3 = predict_flow(386)
        self.predict_flow2 = predict_flow(194)
        self.upsampled_flow6_to_5 = nn.ConvTranspose2d(2, 2, 4, 2, 1, bias=False)
        self.upsampled_flow5_to_4 = nn.ConvTranspose2d(2, 2, 4, 2, 1, bias=False)
        self.upsampled_flow4_to_3 = nn.ConvTranspose2d(2, 2, 4, 2, 1, bias=False)
        self.upsampled_flow3_to_2 = nn.ConvTranspose2d(2, 2, 4, 2, 1, bias=False)

    def forward(self, x):
        """Return the finest flow prediction for the stacked image pair x (N, 6, H, W)."""
        out_conv2 = self.conv2(self.conv1(x))
        out_conv3 = self.conv3_1(self.conv3(out_conv2))
        out_conv4 = self.conv4_1(self.conv4(out_conv3))
        out_conv5 = self.conv5_1(self.conv5(out_conv4))
        out_conv6 = self.conv6_1(self.conv6(out_conv5))

        flow6 = self.predict_flow6(out_conv6)
        flow6_up = crop_like(self.upsampled_flow6_to_5(flow6), out_conv5)
        out_deconv5 = crop_like(self.deconv5(out_conv6), out_conv5)
        concat5 = torch.cat((out_conv5, out_deconv5, flow6_up), 1)
        flow5 = self.predict_flow5(concat5)
        flow5_up = crop_like(self.upsampled_flow5_to_4(flow5), out_conv4)
        out_deconv4 = crop_like(self.deconv4(concat5), out_conv4)
        concat4 = torch.cat((out_conv4, out_deconv4, flow5_up), 1)
        flow4 = self.predict_flow4(concat4)
        flow4_up = crop_like(self.upsampled_flow4_to_3(flow4), out_conv3)
        out_deconv3 = crop_like(self.deconv3(concat4), out_conv3)
        concat3 = torch.cat((out_conv3, out_deconv3, flow4_up), 1)
        flow3 = self.predict_flow3(concat3)
        flow3_up = crop_like(self.upsampled_flow3_to_2(flow3), out_conv2)
        out_deconv2 = crop_like(self.deconv2(concat3), out_conv2)
        concat2 = torch.cat((out_conv2, out_deconv2, flow3_up), 1)
        return self.predict_flow2(concat2)


def load_flownets(path):
    """Return FlowNetS in eval mode with the checkpoint at path (a dict with 'state_dict')."""
    data = torch.load(path, map_location='cpu', weights_only=True)
    model = FlowNetS()
    model.load_state_dict(data['state_dict'] if 'state_dict' in data else data, strict=True)
    return model.eval()
