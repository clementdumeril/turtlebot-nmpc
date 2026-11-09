"""Occupancy-grid tools for exploration: frontiers, a costmap, Dijkstra, path smoothing.

Pure numpy, no ROS, so it is unit-tested and reused by the offline check. Grid convention is
nav_msgs/OccupancyGrid: data[i, j] with row i along y and column j along x, -1 unknown,
0..100 occupancy probability, origin at the corner of cell (0, 0), no rotation.
"""
from __future__ import annotations

import heapq
from dataclasses import dataclass

import numpy as np

FREE_MAX = 25      # cells below this are free (cartographer: ~0-20 free, 65-100 occupied)
OCC_MIN = 65       # cells at or above this are obstacles


@dataclass
class Grid:
    """Occupancy grid in the OccupancyGrid layout, with cell <-> world conversions and masks."""
    data: np.ndarray   # (H, W) int
    res: float         # [m] cell size
    ox: float          # [m] world x of the grid corner
    oy: float

    @property
    def shape(self):
        return self.data.shape

    def cell(self, x: float, y: float) -> tuple[int, int]:
        """(i, j) of the cell containing world point (x, y) [m]; may be outside the grid."""
        return int(np.floor((y - self.oy) / self.res)), int(np.floor((x - self.ox) / self.res))

    def world(self, i, j):
        """World (x, y) [m] of cell centres; i and j may be scalars or arrays."""
        return self.ox + (np.asarray(j) + 0.5) * self.res, self.oy + (np.asarray(i) + 0.5) * self.res

    def inside(self, i: int, j: int) -> bool:
        return 0 <= i < self.shape[0] and 0 <= j < self.shape[1]

    @property
    def free(self):
        """Boolean mask of known cells below FREE_MAX."""
        return (self.data >= 0) & (self.data < FREE_MAX)

    @property
    def occupied(self):
        """Boolean mask of cells at or above OCC_MIN."""
        return self.data >= OCC_MIN

    @property
    def unknown(self):
        """Boolean mask of never-observed cells (-1)."""
        return self.data < 0

    @property
    def unexplored(self):
        """Unknown, or seen too little to be called free or occupied. Cartographer leaves a band of
        such cells between free space and unknown space: counting only -1 there found no frontier
        at all in the lab (exploration stopped after 2 goals)."""
        return (self.data < 0) | ((self.data >= FREE_MAX) & (self.data < OCC_MIN))


def disk(r: int) -> list[tuple[int, int]]:
    """(di, dj) offsets of all cells within r cells of the origin, origin included."""
    return [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1) if di * di + dj * dj <= r * r]


def shift_or(mask: np.ndarray, offsets) -> np.ndarray:
    """Binary dilation of mask by a set of (di, dj) offsets."""
    out = mask.copy()
    H, W = mask.shape
    for di, dj in offsets:
        if di == 0 and dj == 0:
            continue
        # OR the mask shifted by (di, dj) into out, cropping what falls off the edges
        src = mask[max(0, -di):H - max(0, di), max(0, -dj):W - max(0, dj)]
        out[max(0, di):H - max(0, -di), max(0, dj):W - max(0, -dj)] |= src
    return out


def obstacle_distance(occ: np.ndarray, max_cells: int) -> np.ndarray:
    """Distance in cells to the nearest obstacle, up to max_cells (beyond: max_cells + 1)."""
    d = np.full(occ.shape, max_cells + 1, dtype=float)
    d[occ] = 0.0
    grown = occ.copy()
    for k in range(1, max_cells + 1):
        # cells reached by growing obstacles to radius k but not to k - 1 are at distance k
        ring = shift_or(occ, disk(k)) & ~grown
        d[ring] = k
        grown |= ring
    return d


def small_blobs(occ: np.ndarray, max_cells: int) -> np.ndarray:
    """Occupied connected components of at most max_cells cells (8-connected)."""
    eight = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]
    seen = np.zeros_like(occ)
    out = np.zeros_like(occ)
    H, W = occ.shape
    for i0, j0 in zip(*np.nonzero(occ)):
        if seen[i0, j0]:
            continue
        # depth-first flood fill of the component that contains (i0, j0)
        stack, cells = [(i0, j0)], []
        seen[i0, j0] = True
        while stack:
            i, j = stack.pop()
            cells.append((i, j))
            for di, dj in eight:
                a, b = i + di, j + dj
                if 0 <= a < H and 0 <= b < W and occ[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    stack.append((a, b))
        if len(cells) <= max_cells:
            ii, jj = zip(*cells)
            out[list(ii), list(jj)] = True
    return out


def costmap(g: Grid, hard_radius: float, soft_radius: float, soft_weight: float = 4.0,
            chair_radius: float = 0.0, chair_cells: int = 6) -> np.ndarray:
    """Per-cell traversal cost: inf on obstacles, within hard_radius of one, and on unknown cells;
    1 + soft_weight * (closeness within soft_radius) elsewhere, so paths keep to the middle.

    chair_radius > 0: small isolated obstacles (<= chair_cells cells, a chair column or a table
    leg as the lidar sees it) are grown to that radius first, for the chair base under the scan."""
    hard = int(np.ceil(hard_radius / g.res))
    soft = max(hard + 1, int(np.ceil(soft_radius / g.res)))
    occ = g.occupied
    if chair_radius > 0:
        occ = occ | shift_or(small_blobs(occ, chair_cells), disk(int(round(chair_radius / g.res))))
    d = obstacle_distance(occ, soft)
    # linear ramp: soft_weight extra at the hard radius, 0 at the soft radius and beyond
    cost = 1.0 + soft_weight * np.clip((soft - d) / max(soft - hard, 1), 0.0, 1.0)
    cost[d <= hard] = np.inf
    cost[~g.free] = np.inf
    return cost


def frontiers(g: Grid, min_cells: int) -> list[np.ndarray]:
    """Clusters of free cells next to unknown ones, each an (n, 2) array of (i, j), largest first."""
    eight = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1)]
    # frontier cell: free, with at least one unexplored 8-neighbour (dilating the unexplored mask)
    mask = g.free & shift_or(g.unexplored, eight)
    seen = np.zeros_like(mask)
    clusters = []
    for i0, j0 in zip(*np.nonzero(mask)):
        if seen[i0, j0]:
            continue
        # group touching frontier cells into one cluster (8-connected flood fill)
        stack, cells = [(i0, j0)], []
        seen[i0, j0] = True
        while stack:
            i, j = stack.pop()
            cells.append((i, j))
            for di, dj in eight:
                a, b = i + di, j + dj
                if 0 <= a < mask.shape[0] and 0 <= b < mask.shape[1] and mask[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    stack.append((a, b))
        if len(cells) >= min_cells:  # drop specks of noise
            clusters.append(np.array(cells))
    clusters.sort(key=len, reverse=True)
    return clusters


def dijkstra(cost: np.ndarray, start: tuple[int, int]):
    """Shortest paths from start over finite-cost cells (8-connected). Returns (dist [cells], parent flat index)."""
    H, W = cost.shape
    # cells are addressed by flat index u = i * W + j; parent -1 marks the start / unreached cells
    dist = np.full(H * W, np.inf)
    parent = np.full(H * W, -1, dtype=np.int64)
    s = start[0] * W + start[1]
    dist[s] = 0.0
    flat = cost.ravel()
    # (di, dj, step length in cells): 4 straight moves, 4 diagonal ones of length sqrt(2)
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    heap = [(0.0, s)]
    while heap:
        d0, u = heapq.heappop(heap)
        if d0 > dist[u]:  # stale heap entry: u was already settled with a shorter distance
            continue
        i, j = divmod(u, W)
        cu = flat[u] if np.isfinite(flat[u]) else 1.0  # the start may sit in the inflated band
        for di, dj, step in steps:
            a, b = i + di, j + dj
            if not (0 <= a < H and 0 <= b < W):
                continue
            v = a * W + b
            cv = flat[v]
            if not np.isfinite(cv):  # obstacle, inflated band or unknown: never entered
                continue
            # edge weight: step length times the mean cost of the two cells
            nd = d0 + step * 0.5 * (cu + cv)
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd, v))
    return dist.reshape(H, W), parent


def extract(parent: np.ndarray, W: int, goal: tuple[int, int]) -> list[tuple[int, int]]:
    """Cells (i, j) from the Dijkstra start to goal, following the parent links back."""
    out, v = [], goal[0] * W + goal[1]
    while v >= 0:
        out.append(divmod(int(v), W))
        v = parent[v]
    return out[::-1]


def line_free(cost: np.ndarray, a, b) -> bool:
    """True if the straight segment a -> b (cells) stays on cells of cost < 2.5, i.e. away from obstacles."""
    n =int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    ii = np.rint(np.linspace(a[0], b[0], n)).astype(int)
    jj = np.rint(np.linspace(a[1], b[1], n)).astype(int)
    return bool(np.all(np.isfinite(cost[ii, jj])) and np.all(cost[ii, jj] < 2.5))


def smooth(cells: list[tuple[int, int]], cost: np.ndarray) -> list[tuple[int, int]]:
    """Greedy line-of-sight shortcuts through cells that are free and away from obstacles."""
    if len(cells) < 3:
        return cells
    out, k = [cells[0]], 0
    while k < len(cells) - 1:
        nxt = k + 1
        # jump to the farthest later cell visible in a straight line from cell k
        for m in range(len(cells) - 1, k + 1, -1):
            if line_free(cost, cells[k], cells[m]):
                nxt = m
                break
        out.append(cells[nxt])
        k = nxt
    return out


@dataclass
class Plan:
    """Result of plan_to_frontier: goal, smoothed path and the numbers used to pick it."""
    goal_xy: tuple[float, float]
    path_xy: np.ndarray        # (n, 2) world points from the robot to the goal
    length: float              # [m]
    cluster_size: int          # frontier cells


def plan_to_frontier(g: Grid, robot_xy, hard_radius: float, soft_radius: float, min_cells: int,
                     min_goal_dist: float, size_weight: float, blacklist: list, blacklist_radius: float,
                     approach: float = 0.35, chair_radius: float = 0.0, chair_cells: int = 6):
    """Best reachable frontier: score = path length - size_weight * frontier length [m].

    The goal is the reachable cell closest (by path) among those within `approach` of the
    cluster, since frontier cells themselves are often inside the inflated band.
    Returns (Plan or None, clusters).
    """
    cost = costmap(g, hard_radius, soft_radius, chair_radius=chair_radius, chair_cells=chair_cells)
    start = g.cell(*robot_xy)
    if not g.inside(*start):
        return None, []
    dist, parent = dijkstra(cost, start)
    clusters = frontiers(g, min_cells)
    best, best_score = None, np.inf
    r = int(np.ceil(approach / g.res))
    H, W = g.shape
    for cl in clusters:
        # work in a window around the cluster's bounding box, padded by r, to keep it cheap
        i0, j0 = cl.min(0) - r
        i1, j1 = cl.max(0) + r + 1
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, H), min(j1, W)
        win = np.zeros((i1 - i0, j1 - j0), bool)
        win[cl[:, 0] - i0, cl[:, 1] - j0] = True
        # candidate goals: reachable cells within r of a frontier cell
        near = shift_or(win, disk(r)) & np.isfinite(dist[i0:i1, j0:j1])
        if not near.any():
            continue
        # goal = the candidate with the shortest path from the robot
        sub = np.where(near, dist[i0:i1, j0:j1], np.inf)
        k = np.unravel_index(np.argmin(sub), sub.shape)
        gi, gj = k[0] + i0, k[1] + j0
        gx, gy = g.world(gi, gj)
        length = dist[gi, gj] * g.res
        if np.hypot(gx - robot_xy[0], gy - robot_xy[1]) < min_goal_dist:
            continue
        if any(np.hypot(gx - bx, gy - by) < blacklist_radius for bx, by in blacklist):
            continue
        # lower is better: short paths and large frontiers win
        score = length - size_weight * len(cl) * g.res
        if score < best_score:
            best_score, best = score, (gi, gj, length, len(cl))
    if best is None:
        return None, clusters
    gi, gj, length, size = best
    cells = smooth(extract(parent, W, (gi, gj)), cost)
    xs, ys = g.world(np.array([c[0] for c in cells]), np.array([c[1] for c in cells]))
    xy = np.stack([xs, ys], 1)
    xy[0] = robot_xy  # start exactly at the robot, not at its cell centre
    return Plan((float(xy[-1, 0]), float(xy[-1, 1])), xy, float(length), int(size)), clusters


def plan_to_point(g: Grid, robot_xy, goal_xy, hard_radius: float, soft_radius: float,
                  chair_radius: float = 0.0, chair_cells: int = 6):
    """Path to a given point (going home), or None if it is not reachable."""
    cost = costmap(g, hard_radius, soft_radius, chair_radius=chair_radius, chair_cells=chair_cells)
    start, goal = g.cell(*robot_xy), g.cell(*goal_xy)
    if not (g.inside(*start) and g.inside(*goal)):
        return None
    dist, parent = dijkstra(cost, start)
    if not np.isfinite(dist[goal]):
        return None
    cells = smooth(extract(parent, g.shape[1], goal), cost)
    xs, ys = g.world(np.array([c[0] for c in cells]), np.array([c[1] for c in cells]))
    xy = np.stack([xs, ys], 1)
    xy[0], xy[-1] = robot_xy, goal_xy
    return xy


def densify(xy: np.ndarray, step: float = 0.02) -> np.ndarray:
    """Resample a polyline every `step` metres (the NMPC reference needs a dense path)."""
    xy = np.asarray(xy, float)
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.r_[0.0, np.cumsum(seg)]
    if s[-1] < 1e-6:
        return xy[:1].repeat(2, 0)
    q = np.r_[np.arange(0.0, s[-1], step), s[-1]]
    return np.stack([np.interp(q, s, xy[:, 0]), np.interp(q, s, xy[:, 1])], 1)
