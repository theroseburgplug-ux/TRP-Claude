"""Frontier detection and grid path planning."""

from __future__ import annotations

from collections import deque
from typing import Iterable, List, Optional, Tuple

import numpy as np

from .grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, inflate

Cell = Tuple[int, int]  # (col, row)

_N4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
_N8 = _N4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def passable_mask(grid: OccupancyGrid, robot_radius: float) -> np.ndarray:
    """Cells the robot centre may occupy: known-free and clear of obstacles."""
    cls = grid.classify()
    radius_cells = int(np.ceil(robot_radius / grid.resolution))
    blocked = inflate(cls == OCCUPIED, radius_cells)
    return (cls == FREE) & ~blocked


def frontier_mask(grid: OccupancyGrid) -> np.ndarray:
    """Free cells that touch never-observed space (4-neighbourhood).

    Cells with conflicting evidence (e.g. a wall edge hit by one ray and
    crossed by another) are observed, so they do not count as frontier.
    """
    free = grid.classify() == FREE
    unknown = ~grid.observed
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    return free & near_unknown


def cluster_sizes(mask: np.ndarray) -> np.ndarray:
    """Per-cell size of the 8-connected cluster the cell belongs to (0 outside)."""
    h, w = mask.shape
    sizes = np.zeros(mask.shape, dtype=np.int32)
    seen = np.zeros(mask.shape, dtype=bool)
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        members = []
        q = deque([(r0, c0)])
        seen[r0, c0] = True
        while q:
            r, c = q.popleft()
            members.append((r, c))
            for dc, dr in _N8:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    q.append((rr, cc))
        for r, c in members:
            sizes[r, c] = len(members)
    return sizes


def plan_to_nearest_frontier(
    grid: OccupancyGrid,
    start: Cell,
    robot_radius: float,
    min_cluster: int = 3,
    ignore: Iterable[Cell] = (),
    ignore_radius_cells: int = 6,
) -> Optional[List[Cell]]:
    """Shortest path (BFS, 8-connected) from `start` to the nearest usable frontier.

    `ignore` lists cells whose surroundings were already visited without
    resolving the frontier; those frontiers are skipped so the robot does not
    loop on gaps it can never see into (e.g. a thin crack beside a wall).
    Returns the path including start and goal, or None if nothing is reachable.
    """
    passable = passable_mask(grid, robot_radius)
    # The robot is physically standing at `start`, so let it leave even if
    # sensor noise put the cells around it inside an inflated obstacle zone.
    free = grid.classify() == FREE
    sc, sr = start
    near = np.zeros_like(passable)
    near[max(0, sr - 3) : sr + 4, max(0, sc - 3) : sc + 4] = True
    passable |= near & free
    frontier = frontier_mask(grid) & passable
    frontier &= cluster_sizes(frontier_mask(grid)) >= min_cluster
    for c, r in ignore:
        r0, r1 = max(0, r - ignore_radius_cells), r + ignore_radius_cells + 1
        c0, c1 = max(0, c - ignore_radius_cells), c + ignore_radius_cells + 1
        frontier[r0:r1, c0:c1] = False
    if not frontier.any():
        return None

    h, w = passable.shape
    parent = {start: None}
    q = deque([start])
    while q:
        cur = q.popleft()
        c, r = cur
        if cur != start and frontier[r, c]:
            path = []
            node: Optional[Cell] = cur
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1]
        for dc, dr in _N8:
            nc, nr = c + dc, r + dr
            nxt = (nc, nr)
            if not (0 <= nc < w and 0 <= nr < h) or nxt in parent:
                continue
            if not passable[nr, nc]:
                continue
            # No cutting corners between two blocked cells.
            if dc and dr and not (passable[r, nc] and passable[nr, c]):
                continue
            parent[nxt] = cur
            q.append(nxt)
    return None
