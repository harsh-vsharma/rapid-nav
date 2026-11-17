"""Tests of the pure functions in rapid_nav_tools.checks, with hand-computed values."""

import math

import numpy as np
import pytest

from rapid_nav_tools.checks import (
    angle_diff,
    body_clearance,
    clearances,
    collision_episodes,
    colourise_depth,
    dynamic_clearance,
    expected_fx,
    heading_change_per_metre,
    interpolate_track,
    max_step,
    moving_speed,
    occupied_cells_xy,
    path_conformance,
    path_length,
    planar_delta,
    quaternion_from_rpy,
    rate_verdict,
    scan_image,
    tracking_error,
    tracking_error_in_force,
    travelled_length,
    yaw_from_quaternion,
)


def test_expected_fx_matches_the_simulated_camera():
    """Return 462.27 pixels for a 640 px camera with 1.211 rad, as the simulated camera_info."""
    assert expected_fx(640, 1.211) == pytest.approx(462.27, abs=0.02)


def test_expected_fx_halves_with_half_the_width_at_the_same_field_of_view():
    """Make focal length in pixels proportional to the width at one field of view."""
    assert expected_fx(320, 1.211) == pytest.approx(expected_fx(640, 1.211) / 2.0)


def test_quaternion_of_the_optical_frame_rotation():
    """Return (-0.5, 0.5, -0.5, 0.5) for rpy (-pi/2, 0, -pi/2)."""
    quaternion = quaternion_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)
    assert quaternion == pytest.approx((-0.5, 0.5, -0.5, 0.5), abs=1e-12)


def test_quaternion_identity_and_pure_yaw():
    """A zero rotation is the identity, and a yaw of 90 degrees is about the z axis."""
    assert quaternion_from_rpy(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0), abs=1e-12)
    assert z == pytest.approx(math.sqrt(0.5))
    assert w == pytest.approx(math.sqrt(0.5))


def test_yaw_round_trip():
    """yaw_from_quaternion inverts quaternion_from_rpy for a pure yaw."""
    for yaw in (-2.5, -0.3, 0.0, 0.7, 3.0):
        quaternion = quaternion_from_rpy(0.0, 0.0, yaw)
        assert yaw_from_quaternion(*quaternion) == pytest.approx(yaw, abs=1e-9)


def test_angle_diff_wraps_across_pi():
    """Differences across the +/- pi boundary take the short way round."""
    assert angle_diff(3.1, -3.1) == pytest.approx(-0.0831853, abs=1e-6)
    assert angle_diff(-3.1, 3.1) == pytest.approx(0.0831853, abs=1e-6)


def test_angle_diff_interval_is_half_open():
    """The result lies in (-pi, pi]: pi stays pi, and a plain difference is unchanged."""
    assert angle_diff(math.pi, 0.0) == pytest.approx(math.pi)
    assert angle_diff(0.1, 0.0) == pytest.approx(0.1)
    assert angle_diff(0.0, 0.1) == pytest.approx(-0.1)


def test_rate_verdict_native():
    """Native: PASS at 80 % of the expected rate, WARN below it, FAIL with no messages."""
    assert rate_verdict(300, 29.9, 30, 'native') == 'PASS'
    assert rate_verdict(240, 24.0, 30, 'native') == 'PASS'
    assert rate_verdict(200, 20.0, 30, 'native') == 'WARN'
    assert rate_verdict(0, 0.0, 30, 'native') == 'FAIL'


def test_rate_verdict_docker_only_checks_presence():
    """Docker: any message passes, no message fails, whatever the rate."""
    assert rate_verdict(5, 0.5, 30, 'docker') == 'PASS'
    assert rate_verdict(0, 0.0, 30, 'docker') == 'FAIL'


def test_rate_verdict_rejects_an_unknown_profile():
    """An unknown profile name is an error, not a silent PASS."""
    with pytest.raises(ValueError):
        rate_verdict(10, 10.0, 10, 'gpu')


def test_colourise_depth_colours_and_non_finite():
    """Near is blue, far is red, the middle is green-ish, and non-finite values are black."""
    depth = np.array([[0.1, 10.0], [np.nan, 5.05]])
    rgb = colourise_depth(depth, 0.1, 10.0)
    assert rgb.shape == (2, 2, 3)
    assert rgb.dtype == np.uint8
    assert tuple(rgb[0, 0]) == (0, 0, 255)
    assert tuple(rgb[0, 1]) == (255, 0, 0)
    assert tuple(rgb[1, 0]) == (0, 0, 0)
    assert rgb[1, 1, 1] > 200


def test_scan_image_places_a_return_ahead_of_the_robot():
    """
    A return 1 m ahead at max range 2 m lands 25 px right of the centre on a 101 px view.

    Infinite and NaN returns are ignored, so the only white pixel is the return and the only
    red pixel is the robot.
    """
    ranges = np.array([1.0, np.inf, np.nan])
    image = scan_image(ranges, 0.0, 0.0, 101, 2.0)
    assert image.shape == (101, 101, 3)
    assert tuple(image[50, 75]) == (255, 255, 255)
    assert tuple(image[50, 50]) == (255, 0, 0)
    white = np.all(image == 255, axis=-1)
    assert int(white.sum()) == 1


def test_scan_image_maps_left_to_up():
    """A return at +90 degrees (left of the robot, y positive) appears above the centre."""
    image = scan_image(np.array([1.0]), math.pi / 2, 0.0, 101, 2.0)
    assert tuple(image[25, 50]) == (255, 255, 255)


def test_planar_delta_distance_and_wrapped_yaw():
    """The distance is the straight-line length; the yaw change is wrapped."""
    distance, yaw_change = planar_delta((0.0, 0.0, 0.0), (1.0, 0.0, 0.5))
    assert distance == pytest.approx(1.0)
    assert yaw_change == pytest.approx(0.5)
    distance, yaw_change = planar_delta((0.0, 0.0, 3.1), (0.0, 0.0, -3.1))
    assert distance == pytest.approx(0.0)
    assert yaw_change == pytest.approx(0.0831853, abs=1e-6)
    distance, _ = planar_delta((1.0, 1.0, 0.0), (4.0, 5.0, 0.0))
    assert distance == pytest.approx(5.0)


def test_path_length_of_an_l_shape():
    """A path of 3 m east then 4 m north is 7 m long; fewer than two points have no length."""
    assert path_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) == pytest.approx(7.0)
    assert path_length([(1.0, 1.0)]) == 0.0
    assert path_length([]) == 0.0


def test_max_step_is_the_longest_gap():
    """The longest gap of a path with a diagonal 5 m step is 5 m."""
    assert max_step([(0.0, 0.0), (1.0, 0.0), (4.0, 4.0)]) == pytest.approx(5.0)
    assert max_step([(0.0, 0.0)]) == 0.0


def test_heading_change_of_two_right_turns_is_sixty_degrees_per_metre():
    """Two 90 degree turns over 3 m of path are 180 degrees in total: 60 degrees per metre."""
    corner = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    assert heading_change_per_metre(corner) == pytest.approx(60.0)


def test_heading_change_of_a_straight_line_is_zero():
    """Collinear points turn by nothing, and repeated points are ignored."""
    assert heading_change_per_metre([(0.0, 0.0), (1.0, 1.0), (1.0, 1.0), (2.0, 2.0)]) == 0.0
    assert heading_change_per_metre([(0.0, 0.0)]) == 0.0


def test_heading_change_wraps_a_turn_past_pi():
    """A turn from heading 170 to heading -170 degrees is 20 degrees, not 340."""
    a = math.radians(170.0)
    b = math.radians(-170.0)
    points = [(0.0, 0.0), (math.cos(a), math.sin(a)), (math.cos(a) + math.cos(b),
                                                       math.sin(a) + math.sin(b))]
    assert heading_change_per_metre(points) == pytest.approx(20.0 / 2.0, abs=1e-6)


def test_clearances_match_a_brute_force_search():
    """Chunked clearances equal the brute-force distance to the nearest occupied centre."""
    rng = np.random.default_rng(3)
    points = rng.uniform(-2.0, 2.0, size=(300, 2))
    occupied = rng.uniform(-2.0, 2.0, size=(40, 2))
    expected = np.array([np.min(np.hypot(*(occupied - p).T)) for p in points])
    assert clearances(points, occupied, chunk=64) == pytest.approx(expected)


def test_clearances_are_infinite_without_obstacles():
    """With no occupied cell the clearance of every point is infinite."""
    result = clearances([(0.0, 0.0), (1.0, 1.0)], np.zeros((0, 2)))
    assert np.all(np.isinf(result))


def test_occupied_cells_xy_places_cell_centres_in_metres():
    """Row 0 is the map's bottom row: row 1, column 2 has its centre at (0.125, 0.075)."""
    grid = np.zeros((3, 4), dtype=np.int8)
    grid[1, 2] = 100
    grid[0, 0] = -1
    centres = occupied_cells_xy(grid, 0.05, (0.0, 0.0))
    assert centres.shape == (1, 2)
    assert centres[0] == pytest.approx([0.125, 0.075])
    assert occupied_cells_xy(np.zeros((2, 2)), 0.05, (0.0, 0.0)).shape == (0, 2)


def test_body_clearance_of_a_cell_and_an_overlap():
    """
    Check the clearance to a cell, and an overlap, of a robot of radius 0.25 by hand.

    A cell of side 0.1 at (1, 0): the centre at the origin is 0.95 m from the cell's near edge, a
    clearance of 0.70 m; at x = 0.8 the centre is 0.15 m from the edge, an overlap of -0.10 m.
    """
    cells = np.array([[1.0, 0.0]])
    result = body_clearance([(0.0, 0.0), (0.8, 0.0)], cells, np.zeros((0, 3)), 0.25, cell=0.1)
    assert result == pytest.approx([0.70, -0.10])


def test_body_clearance_counts_extra_boxes_and_is_infinite_without_obstacles():
    """A box of half side 0.25 at (-2, 1) is 0.75 m from (-2, 0): a clearance of 0.5 m."""
    result = body_clearance([(-2.0, 0.0)], np.zeros((0, 2)), [(-2.0, 1.0, 0.25)], 0.25)
    assert result == pytest.approx([0.5])
    empty = body_clearance([(0.0, 0.0)], np.zeros((0, 2)), np.zeros((0, 3)), 0.25)
    assert np.all(np.isinf(empty))


def test_collision_episodes_count_entries_not_samples():
    """Two entries below zero in the first list; a start below zero is one entry."""
    assert collision_episodes([1.0, -0.1, -0.2, 0.5, -0.3]) == 2
    assert collision_episodes([-1.0, -2.0]) == 1
    assert collision_episodes([0.2, 0.1]) == 0


def test_tracking_error_is_the_mean_distance_to_the_nearest_path_point():
    """
    Check the tracking error to the nearest path point by hand.

    The path has points at x = 0, 0.5 and 1. The point (0.5, 0.2) is 0.2 m from (0.5, 0); the point
    (1.0, 0.5) is 0.5 m from (1, 0): a mean of 0.35 m. With no path the error is not a number.
    """
    path = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
    assert tracking_error([(0.5, 0.2), (1.0, 0.5)], path) == pytest.approx(0.35)
    assert math.isnan(tracking_error([(0.5, 0.2)], []))


def test_tracking_error_in_force_uses_the_plan_published_last_before_each_point():
    """
    Check the tracking error to the plan in force, with two plans.

    One plan lies along y = 0 from t = 0, the other along y = 1 from t = 1. The point (0.5, 0.2) at
    t = 0.5 is 0.2 m from the first; the point (0.5, 1.3) at t = 1.5 is 0.3 m from the second: a
    mean of 0.25 m. The point at t = -1, before any plan, is skipped.
    """
    plans = [[(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)], [(0.0, 1.0), (0.5, 1.0), (1.0, 1.0)]]
    points = [(0.5, 0.2), (0.5, 1.3), (0.5, 5.0)]
    times = [0.5, 1.5, -1.0]
    result = tracking_error_in_force(points, times, [0.0, 1.0], plans)
    assert result == pytest.approx(0.25)
    assert math.isnan(tracking_error_in_force(points, times, [], []))


def test_travelled_length_follows_the_positions():
    """Three metres east then four north is seven metres of travel."""
    assert travelled_length([(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]) == pytest.approx(7.0)


def test_interpolate_track_is_linear_and_held_outside():
    at = interpolate_track([0.0, 2.0], [[0.0, 0.0], [2.0, 4.0]], [-1.0, 0.5, 1.0, 3.0])
    assert at.tolist() == [[0.0, 0.0], [0.5, 1.0], [1.0, 2.0], [2.0, 4.0]]


def test_dynamic_clearance_circle_and_box():
    robot = [[0.0, 0.0], [0.0, 0.0]]
    tracks = [
        {'name': 'p', 'times': [0.0, 1.0], 'xy': [[2.0, 0.0], [1.0, 0.0]], 'kind': 'circle',
         'radius': 0.2},
        {'name': 'c', 'times': [0.0, 1.0], 'xy': [[0.0, 3.0], [0.0, 3.0]], 'kind': 'box',
         'half_x': 0.3, 'half_y': 0.2},
    ]
    per_sample, per_obstacle = dynamic_clearance(robot, [0.0, 1.0], tracks, 0.25)
    # circle: 2.0 - 0.2 - 0.25 = 1.55 then 1.0 - 0.45 = 0.55; box: 3.0 - 0.2 - 0.25 = 2.55
    assert per_sample.tolist() == pytest.approx([1.55, 0.55])
    assert per_obstacle == pytest.approx({'p': 0.55, 'c': 2.55})
    none, empty = dynamic_clearance(robot, [0.0, 1.0], [], 0.25)
    assert np.isinf(none).all() and empty == {}


def test_dynamic_clearance_box_corner_and_overlap():
    tracks = [{'name': 'c', 'times': [0.0], 'xy': [[1.0, 1.0]], 'kind': 'box',
               'half_x': 0.5, 'half_y': 0.5}]
    corner, _ = dynamic_clearance([[0.0, 0.0]], [0.0], tracks, 0.25)
    assert corner[0] == pytest.approx(math.hypot(0.5, 0.5) - 0.25)
    inside, _ = dynamic_clearance([[1.0, 1.0]], [0.0], tracks, 0.25)
    assert inside[0] == pytest.approx(-0.25)


def test_path_conformance_offset_and_loop():
    path = [{'x': 0.0, 'y': 0.0}, {'x': 4.0, 'y': 0.0}]
    assert path_conformance([[1.0, 0.03], [2.0, -0.01]], path, 'once') == pytest.approx(0.03)
    assert path_conformance([[5.0, 0.0]], path, 'once') == pytest.approx(1.0)
    square = [{'x': 0.0, 'y': 0.0}, {'x': 2.0, 'y': 0.0}, {'x': 2.0, 'y': 2.0}]
    # the closing segment runs from (2, 2) back to (0, 0): (1, 1) is on it only for a loop
    assert path_conformance([[1.0, 1.0]], square, 'loop') == pytest.approx(0.0)
    assert path_conformance([[1.0, 1.0]], square, 'once') == pytest.approx(1.0)


def test_moving_speed_ignores_standing_steps():
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    xy = [[0.0, 0.0], [0.0, 0.0], [0.5, 0.0], [1.0, 0.0], [1.0, 0.0]]
    assert moving_speed(times, xy) == pytest.approx(0.5)
    assert moving_speed([0.0, 1.0], [[0, 0], [0, 0]]) == 0.0
