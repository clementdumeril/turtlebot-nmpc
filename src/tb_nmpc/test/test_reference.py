import numpy as np

from tb_nmpc.reference import Path, anchored, lemniscate


def test_starts_at_origin_heading_x():
    p = Path(lemniscate(1.5))
    assert np.allclose(p.xy[0], 0.0)
    assert abs(p.theta[0]) < 5e-3
    assert 9.0 < p.length < 9.3


def test_projection_stays_on_branch_at_the_crossing():
    p = Path(lemniscate(1.5))
    s = 0.0
    for s_true in np.arange(0.0, p.length, 0.05):
        x, y, _, _ = p.sample(s_true)
        s, d = p.project((float(x), float(y)), s)
        assert abs(s - s_true) < 0.02 and d < 1e-3


def test_anchored_rotates_and_shifts():
    p = anchored(1.0, 1.0, (1.0, 2.0, np.pi / 2))
    assert np.allclose(p.xy[0], [1.0, 2.0])
    assert abs(p.theta[0] - np.pi / 2) < 5e-3


def test_horizon_slows_down_at_the_end():
    p = Path(lemniscate(1.0))
    ref = p.horizon(p.length - 0.1, 0.2, 0.1, 10)
    assert ref[-1, 3] < 0.05


def test_skip_blocked_moves_reference_past_an_obstacle():
    p = Path(lemniscate(1.5))
    s, v = p.horizon_s(1.0, 0.18, 0.1, 30)
    x, y, _, _ = p.sample(1.3)
    s2 = p.skip_blocked(s, np.array([[float(x), float(y), 0.3]]))
    pts = np.stack(p.sample(s2)[:2], 1)[1:]
    assert np.all(np.hypot(pts[:, 0] - x, pts[:, 1] - y) >= 0.3 - 1e-9)
    assert np.all(np.diff(s2) >= 0) and s2[0] == s[0]
