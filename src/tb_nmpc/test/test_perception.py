import numpy as np

from tb_nmpc.perception import PerceptionParams, detect


def ray_scan(circles, walls_y=None, n=360, rmax=3.5):
    """Ranges of an ideal lidar at the origin, beams from +pi downwards (as in Webots)."""
    a = np.pi - 2 * np.pi * np.arange(n) / n
    r = np.full(n, np.inf)
    d = np.stack([np.cos(a), np.sin(a)], 1)
    for cx, cy, rad in circles:
        b = d @ np.array([cx, cy])
        disc = b ** 2 - (cx ** 2 + cy ** 2 - rad ** 2)
        hit = (disc >= 0) & (b > 0)
        r[hit] = np.minimum(r[hit], b[hit] - np.sqrt(disc[hit]))
    if walls_y is not None:
        t = walls_y / np.where(np.abs(d[:, 1]) > 1e-9, d[:, 1], np.nan)
        ok = np.isfinite(t) & (t > 0)
        r[ok] = np.minimum(r[ok], t[ok])
    r[r > rmax] = np.inf
    return r, np.pi, -2 * np.pi / n


def test_cylinder_centre_and_radius():
    r, amin, inc = ray_scan([(1.2, 0.5, 0.15)])
    small, static = detect(r, amin, inc, (0.0, 0.0, 0.0), PerceptionParams())
    assert len(small) == 1 and len(static) == 0
    assert np.hypot(small[0, 0] - 1.2, small[0, 1] - 0.5) < 0.06
    assert 0.08 <= small[0, 2] <= 0.25


def test_wall_becomes_static_chunks():
    r, amin, inc = ray_scan([], walls_y=1.0)
    small, static = detect(r, amin, inc, (0.0, 0.0, 0.0), PerceptionParams())
    assert len(static) >= 5
    assert np.all(np.abs(static[:, 1] - 1.0) < 0.1)


def test_sensor_pose_is_applied():
    r, amin, inc = ray_scan([(1.0, 0.0, 0.15)])
    small, _ = detect(r, amin, inc, (2.0, 1.0, np.pi / 2), PerceptionParams())
    assert np.hypot(small[0, 0] - 2.0, small[0, 1] - 2.0) < 0.06


def test_chair_column_gets_the_base_radius():
    r, amin, inc = ray_scan([(1.0, 0.0, 0.025)])          # a 5 cm chair column
    p = PerceptionParams(pole_radius=0.32, merge_dist=0.35)
    small, _ = detect(r, amin, inc, (0.0, 0.0, 0.0), p)
    assert len(small) == 1 and abs(small[0, 2] - 0.32) < 1e-9
    r, amin, inc = ray_scan([(1.2, 0.5, 0.15)])           # a 30 cm object keeps its size
    small, _ = detect(r, amin, inc, (0.0, 0.0, 0.0), p)
    assert small[0, 2] < 0.25
