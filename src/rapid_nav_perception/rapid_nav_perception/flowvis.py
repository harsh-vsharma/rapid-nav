"""
Colour picture of an optical flow field: hue gives the direction, saturation the magnitude.

Pure numpy, no ROS imports.
"""

import numpy as np


def flow_to_rgb(flow, max_magnitude=None):
    """
    Return a uint8 RGB picture of a flow field (H, W, 2).

    Hue is the direction of the flow (the usual colour wheel: right is red, left is cyan),
    saturation is the magnitude divided by max_magnitude (clipped to 1), and value is 1. Zero flow
    is white. max_magnitude defaults to the 99th percentile of the magnitude, and to at least 1
    pixel.
    """
    flow = np.asarray(flow, dtype=np.float64)
    magnitude = np.hypot(flow[..., 0], flow[..., 1])
    if max_magnitude is None:
        max_magnitude = max(float(np.percentile(magnitude, 99)), 1.0)
    saturation = np.clip(magnitude / max_magnitude, 0.0, 1.0)
    hue = (np.arctan2(flow[..., 1], flow[..., 0]) / (2.0 * np.pi)) % 1.0

    # HSV to RGB with value 1.
    scaled = hue * 6.0
    sector = np.floor(scaled).astype(int) % 6
    fraction = scaled - np.floor(scaled)
    p = 1.0 - saturation
    q = 1.0 - saturation * fraction
    t = 1.0 - saturation * (1.0 - fraction)
    red = np.choose(sector, [np.ones_like(p), q, p, p, t, np.ones_like(p)])
    green = np.choose(sector, [t, np.ones_like(p), np.ones_like(p), q, p, p])
    blue = np.choose(sector, [p, p, t, np.ones_like(p), np.ones_like(p), q])
    rgb = np.stack([red, green, blue], axis=-1) * 255.0
    return np.clip(np.rint(rgb), 0, 255).astype(np.uint8)
