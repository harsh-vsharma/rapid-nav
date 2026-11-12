"""
Pure functions behind sim_check: geometry, rate verdicts and image helpers.

Nothing here imports ROS, so the functions run on a plain Python with numpy (see the tests).
"""

import math

import numpy as np

PROFILES = ('native', 'docker')


def expected_fx(width, hfov):
    """Return the focal length in pixels of a pinhole camera of this width and field of view."""
    return (width / 2.0) / math.tan(hfov / 2.0)


def quaternion_from_rpy(roll, pitch, yaw):
    """Return the quaternion (x, y, z, w) of a roll, pitch, yaw rotation (ZYX convention)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def yaw_from_quaternion(x, y, z, w):
    """Return the yaw angle (rad) of a quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def angle_diff(a, b):
    """Return a - b wrapped to the interval (-pi, pi]."""
    difference = (a - b) % (2.0 * math.pi)
    if difference > math.pi:
        difference -= 2.0 * math.pi
    return difference


def rate_verdict(count, rate, expected, profile):
    """
    Return PASS, WARN or FAIL for a measured topic rate.

    native: PASS at 80 % of the expected rate or more, WARN when messages arrive but slower, FAIL
    when none arrive. docker: PASS when messages arrive, FAIL when none arrive (software rendering
    makes the rates low, so only presence is checked).
    """
    if profile not in PROFILES:
        raise ValueError("profile must be 'native' or 'docker', got {!r}".format(profile))
    if count <= 0 or rate <= 0.0:
        return 'FAIL'
    if profile == 'docker':
        return 'PASS'
    if rate >= 0.8 * expected:
        return 'PASS'
    return 'WARN'


def colourise_depth(depth, near, far):
    """Return a uint8 RGB image of a depth array: near is blue, far is red, non-finite is black."""
    depth = np.asarray(depth, dtype=np.float64)
    t = np.clip((depth - near) / (far - near), 0.0, 1.0)
    red = 255.0 * t
    green = 255.0 * (1.0 - np.abs(2.0 * t - 1.0))
    blue = 255.0 * (1.0 - t)
    rgb = np.stack([red, green, blue], axis=-1)
    rgb[~np.isfinite(depth)] = 0.0
    return np.clip(np.rint(rgb), 0, 255).astype(np.uint8)


def scan_image(ranges, angle_min, increment, size_px, max_range):
    """
    Return a top-down uint8 RGB view of a laser scan: robot at the centre, x right, y up.

    Each finite return inside max_range is drawn as a white pixel; the robot is a red pixel.
    """
    image = np.zeros((size_px, size_px, 3), dtype=np.uint8)
    centre = (size_px - 1) / 2.0
    scale = (size_px / 2.0) / max_range
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + increment * np.arange(ranges.size)
    valid = np.isfinite(ranges) & (ranges > 0.0) & (ranges <= max_range)
    x = ranges[valid] * np.cos(angles[valid])
    y = ranges[valid] * np.sin(angles[valid])
    columns = np.rint(centre + x * scale).astype(int)
    rows = np.rint(centre - y * scale).astype(int)
    inside = (columns >= 0) & (columns < size_px) & (rows >= 0) & (rows < size_px)
    image[rows[inside], columns[inside]] = (255, 255, 255)
    image[int(round(centre)), int(round(centre))] = (255, 0, 0)
    return image


def planar_delta(start_xyyaw, end_xyyaw):
    """Return (distance, yaw change) between two (x, y, yaw) poses; the yaw change is wrapped."""
    dx = end_xyyaw[0] - start_xyyaw[0]
    dy = end_xyyaw[1] - start_xyyaw[1]
    distance = math.hypot(dx, dy)
    return (distance, angle_diff(end_xyyaw[2], start_xyyaw[2]))


def predicted_rotation_flow(fx, omega, dt):
    """Return the image shift in pixels of a turn at omega rad/s for dt s: fx tan(omega dt)."""
    return fx * math.tan(omega * dt)


def predicted_turn_flow_u(u_offset, depth, fx, theta, lever):
    """
    Return the horizontal flow in pixels of a turn in place by theta rad (positive = left).

    The camera sits `lever` metres ahead of the turning axis, so it also moves sideways; that adds
    flow that depends on the depth. u_offset is u - cx per pixel, depth the optical z in metres
    (arrays of the same shape). Exact for a level camera turning about a vertical axis.
    """
    u_offset = np.asarray(u_offset, dtype=np.float64)
    depth = np.asarray(depth, dtype=np.float64)
    forward = depth                       # along the robot's x before the turn
    left = -u_offset * depth / fx         # along the robot's y (optical -x)
    forward_rel = forward - lever * (math.cos(theta) - 1.0)
    left_rel = left - lever * math.sin(theta)
    z_new = forward_rel * math.cos(theta) + left_rel * math.sin(theta)
    x_new = forward_rel * math.sin(theta) - left_rel * math.cos(theta)
    return fx * x_new / z_new - u_offset


def central_window(array, fraction):
    """Return the central window that covers the given fraction of the image height and width."""
    array = np.asarray(array)
    height, width = array.shape[0], array.shape[1]
    top = int(round(height * (1.0 - fraction) / 2.0))
    left = int(round(width * (1.0 - fraction) / 2.0))
    return array[top:height - top, left:width - left]


def band(array, top_fraction, bottom_fraction, left_fraction, right_fraction):
    """Return the part of an image between the given fractions of its height and of its width."""
    array = np.asarray(array)
    height, width = array.shape[0], array.shape[1]
    return array[int(round(height * top_fraction)):int(round(height * bottom_fraction)),
                 int(round(width * left_fraction)):int(round(width * right_fraction))]


def transform_points(points, translation, quaternion):
    """Return points (N, 3) rotated by the quaternion (x, y, z, w), then translated."""
    x, y, z, w = quaternion
    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return points @ rotation.T + np.asarray(translation, dtype=np.float64)


def scan_range_at(bearing, angle_min, increment, ranges):
    """Return the range of the beam nearest to a bearing (rad); NaN when outside the scan."""
    ranges = np.asarray(ranges, dtype=np.float64)
    if ranges.size == 0 or increment == 0.0:
        return float('nan')
    index = int(round((bearing - angle_min) / increment))
    if index < 0 or index >= ranges.size:
        return float('nan')
    return float(ranges[index])
