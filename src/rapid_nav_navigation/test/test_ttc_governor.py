"""Checks of the time-to-collision speed governor's helpers (no ROS needed)."""

import importlib.util
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.normpath(os.path.join(HERE, '..', 'scripts', 'ttc_governor.py'))


def load():
    """Import scripts/ttc_governor.py from the source tree."""
    spec = importlib.util.spec_from_file_location('ttc_governor', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


G = load()


def mover(x, y, vx, vy, radius=0.25, sp=0.0, sv=0.0):
    """Return an obstacle dict."""
    return {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'radius': radius, 'position_sigma': sp,
            'velocity_sigma': sv}


def test_closest_approach():
    t, d = G.closest_approach((4.0, 1.0), (-1.0, 0.0))
    assert abs(t - 4.0) < 1e-12 and abs(d - 1.0) < 1e-12
    t, d = G.closest_approach((2.0, 0.0), (0.0, 0.0))
    assert t == 0.0 and d == 2.0


def test_head_on_gives_a_drop():
    # Robot at 0.5 m/s, obstacle 2.4 m ahead at 0.5 m/s towards it: t* = 2.4 s.
    limit = G.speed_limit((0.0, 0.0), (0.5, 0.0), [mover(2.4, 0.0, -0.5, 0.0)])
    assert abs(limit - 2.4 / 3.0) < 1e-9
    # Closer: the floor of 20 %.
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), [mover(0.3, 0.0, -0.5, 0.0)]) == 0.2


def test_diverging_or_passing_track_gives_none():
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), [mover(-1.0, 0.0, -0.5, 0.0)]) == 1.0
    # Parallel lane 1.5 m away: d* = 1.5 > 0.5 m of radii.
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), [mover(2.0, 1.5, -0.5, 0.0)]) == 1.0
    # Beyond the horizon: t* = 5 s.
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), [mover(5.0, 0.0, -0.5, 0.0)]) == 1.0
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), []) == 1.0


def test_large_sigma_drops_earlier():
    # A crossing that misses by 0.7 m at t* = 2.3 s: no limit when certain (0.5 m of radii), a
    # limit with a large sigma.
    crossing = mover(1.78, -2.0, 0.0, 1.0)
    t, d = G.closest_approach((1.78, -2.0), (-0.5, 1.0))
    assert abs(d - 0.7) < 0.01 and abs(t - 2.31) < 0.01
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), [crossing]) == 1.0
    unsure = dict(crossing, position_sigma=0.1, velocity_sigma=0.3)
    limit, (t, inflated) = G.obstacle_limit((0.0, 0.0), (0.5, 0.0), unsure)
    assert inflated < 0.0 and 0.0 < t < 3.0 and limit < 1.0


def test_smallest_limit_wins():
    near = mover(1.0, 0.0, -0.5, 0.0)
    far = mover(2.4, 0.0, -0.5, 0.0)
    assert G.speed_limit((0.0, 0.0), (0.5, 0.0), [far, near]) == \
        G.speed_limit((0.0, 0.0), (0.5, 0.0), [near])


def test_rate_limiter_drops_at_once_and_rises_slowly():
    r = G.RateLimiter(0.2)
    assert r.step(0.3, 0.1) == 0.3
    assert abs(r.step(1.0, 0.1) - 0.32) < 1e-12
    for _ in range(40):
        r.step(1.0, 0.1)
    assert r.value == 1.0
    assert r.step(0.5, 0.0) == 0.5


def test_yaw_of():
    half = math.sqrt(0.5)
    assert abs(G.yaw_of((0.0, 0.0, half, half)) - math.pi / 2) < 1e-12
