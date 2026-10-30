import numpy as np

from tb_nmpc.tracker import Tracker


def test_constant_velocity_target():
    tr = Tracker()
    rng = np.random.default_rng(0)
    v = np.array([0.5, -0.2])
    for k in range(15):  # 3 s of a 5 Hz lidar
        t = 0.2 * k
        p = np.array([1.0, 2.0]) + v * t + rng.normal(0, 0.02, 2)
        tr.update(t, [[p[0], p[1], 0.15]])
    assert len(tr.tracks) == 1
    est = tr.velocity(tr.tracks[0])
    assert np.linalg.norm(est - v) < 0.1
    pred = tr.predict(tr.tracks[0], np.array([0.0, 1.0]))
    assert np.linalg.norm(pred[1, :2] - (np.array([1.0, 2.0]) + v * 3.8)) < 0.15
    assert pred[1, 2] > pred[0, 2]  # radius grows with the horizon


def test_static_target_has_no_velocity():
    tr = Tracker()
    for k in range(10):
        tr.update(0.2 * k, [[1.0 + 0.01 * (-1) ** k, 0.0, 0.2]])
    assert np.allclose(tr.velocity(tr.tracks[0]), 0.0)


def test_track_is_dropped_after_max_age():
    tr = Tracker()
    tr.update(0.0, [[0.0, 0.0, 0.1]])
    tr.update(2.0, np.zeros((0, 3)))
    assert tr.tracks == []
