import numpy as np

from tb_explore.grid import Grid, costmap, densify, frontiers, plan_to_frontier, plan_to_point
from tb_explore.mapping import LogOddsMap


def room(n=60, res=0.05):
    """Free square room with walls, the right half unknown."""
    d = np.zeros((n, n), np.int16)
    d[0, :] = d[-1, :] = d[:, 0] = d[:, -1] = 100
    d[:, n // 2:] = -1
    return Grid(d, res, 0.0, 0.0)


def test_frontier_is_the_boundary_with_unknown():
    g = room()
    cl = frontiers(g, 5)
    assert len(cl) == 1
    assert np.all(cl[0][:, 1] == 29)


def test_costmap_blocks_near_walls_and_unknown():
    g = room()
    c = costmap(g, 0.15, 0.4)
    assert np.isinf(c[1, 10]) and np.isinf(c[10, 40]) and np.isfinite(c[20, 15])
    assert c[20, 15] < c[5, 15]


def test_plan_reaches_the_frontier_without_touching_walls():
    g = room()
    plan, _ = plan_to_frontier(g, (0.5, 1.5), 0.15, 0.4, 5, 0.2, 0.5, [], 0.3)
    assert plan is not None
    assert plan.goal_xy[0] > 1.0                      # near the frontier at x = 1.45
    xy = densify(plan.path_xy, 0.02)
    i = np.floor(xy[:, 1] / g.res).astype(int)
    j = np.floor(xy[:, 0] / g.res).astype(int)
    assert np.all(g.data[i, j] < 65)


def test_blacklisted_goal_is_skipped():
    g = room()
    plan, _ = plan_to_frontier(g, (0.5, 1.5), 0.15, 0.4, 5, 0.2, 0.5, [], 0.3)
    again, _ = plan_to_frontier(g, (0.5, 1.5), 0.15, 0.4, 5, 0.2, 0.5, [plan.goal_xy], 3.0)
    assert again is None


def test_plan_home():
    g = room()
    assert plan_to_point(g, (0.5, 1.5), (1.2, 0.5), 0.15, 0.4) is not None
    assert plan_to_point(g, (0.5, 1.5), (2.5, 0.5), 0.15, 0.4) is None   # unknown side


def test_logodds_map_sees_a_wall():
    m = LogOddsMap(4.0, 0.05, (0.0, 0.0))
    n = 360
    a = -np.pi + 2 * np.pi * np.arange(n) / n
    r = np.where(np.abs(a) < 0.3, 1.0 / np.cos(a), np.inf)   # wall at x = 1 in front
    for _ in range(5):
        m.update((0.0, 0.0, 0.0), r, -np.pi, 2 * np.pi / n, 0.1)
    g = m.grid()
    i, j = g.cell(1.0, 0.0)
    assert g.data[i, j] >= 65
    i, j = g.cell(0.5, 0.0)
    assert 0 <= g.data[i, j] < 25
    i, j = g.cell(-1.9, 1.9)
    assert g.data[i, j] == -1
