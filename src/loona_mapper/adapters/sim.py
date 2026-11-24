"""A 2D simulated room and robot for developing and testing without hardware."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from ..grid import OccupancyGrid
from ..robot import Pose, RobotAdapter, normalize_angle

Rect = Tuple[float, float, float, float]  # xmin, ymin, xmax, ymax
Segment = Tuple[float, float, float, float]


def _rect_segments(r: Rect) -> List[Segment]:
    x0, y0, x1, y1 = r
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


@dataclass
class SimWorld:
    """Rectangular room with rectangular obstacles (furniture, partitions)."""

    room: Rect
    obstacles: List[Rect] = field(default_factory=list)

    @classmethod
    def demo(cls) -> "SimWorld":
        """5 x 4 m room with a couch, a table and a partial divider wall."""
        return cls(
            room=(-2.5, -2.0, 2.5, 2.0),
            obstacles=[
                (1.0, 0.4, 1.8, 1.0),  # couch
                (-1.8, -1.5, -1.0, -1.1),  # table
                (-0.55, -2.0, -0.45, 0.4),  # divider wall with a gap at the top
            ],
        )

    def segments(self) -> List[Segment]:
        segs = _rect_segments(self.room)
        for o in self.obstacles:
            segs += _rect_segments(o)
        return segs

    def is_free(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self.room
        if not (x0 < x < x1 and y0 < y < y1):
            return False
        return not any(a < x < c and b < y < d for a, b, c, d in self.obstacles)

    def raycast(self, x: float, y: float, theta: float, max_range: float) -> Optional[float]:
        """Distance to the nearest wall along a ray, or None beyond max_range."""
        dx, dy = math.cos(theta), math.sin(theta)
        best = None
        for x1, y1, x2, y2 in self.segments():
            ex, ey = x2 - x1, y2 - y1
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / denom
            u = ((x1 - x) * dy - (y1 - y) * dx) / denom
            if t >= 0 and 0 <= u <= 1 and (best is None or t < best):
                best = t
        if best is None or best > max_range:
            return None
        return best

    def truth_masks(self, grid: OccupancyGrid) -> Tuple[np.ndarray, np.ndarray]:
        """(free, wall) boolean masks on `grid`'s cells for scoring a result.

        `wall` is the one-cell-thick rasterisation of every wall segment.
        """
        free = np.zeros((grid.height, grid.width), dtype=bool)
        wall = np.zeros_like(free)
        for r in range(grid.height):
            for c in range(grid.width):
                x, y = grid.cell_to_world(c, r)
                free[r, c] = self.is_free(x, y)
        for x1, y1, x2, y2 in self.segments():
            n = int(math.hypot(x2 - x1, y2 - y1) / (grid.resolution / 2)) + 1
            for i in range(n + 1):
                t = i / n
                c, r = grid.world_to_cell(x1 + t * (x2 - x1), y1 + t * (y2 - y1))
                if grid.in_bounds(c, r):
                    wall[r, c] = True
        return free, wall


class SimulatedRobot(RobotAdapter):
    """Differential-drive-ish robot with a forward ToF sensor in a SimWorld.

    `drift` adds proportional error to the true motion while the reported
    (dead-reckoned) pose stays ideal, which is how real odometry fails.
    """

    def __init__(
        self,
        world: SimWorld,
        start: Pose = Pose(0.0, 0.0, 0.0),
        max_range: float = 2.0,
        body_radius: float = 0.12,
        range_noise: float = 0.0,
        drift: float = 0.0,
        seed: int = 0,
    ) -> None:
        self.world = world
        self.max_range = max_range
        self.body_radius = body_radius
        self.range_noise = range_noise
        self.drift = drift
        self._rng = random.Random(seed)
        self.true = start
        self._odom = start

    def pose(self) -> Pose:
        return self._odom

    def rotate(self, radians: float) -> None:
        err = 1.0 + self._rng.gauss(0, self.drift) if self.drift else 1.0
        self.true = Pose(
            self.true.x, self.true.y, normalize_angle(self.true.theta + radians * err)
        )
        self._odom = Pose(
            self._odom.x, self._odom.y, normalize_angle(self._odom.theta + radians)
        )

    def forward(self, meters: float) -> float:
        t = self.true
        wall = self.world.raycast(t.x, t.y, t.theta, meters + self.body_radius + 1.0)
        free_run = meters if wall is None else max(0.0, wall - self.body_radius)
        moved = min(meters, free_run)
        err = 1.0 + self._rng.gauss(0, self.drift) if self.drift else 1.0
        self.true = Pose(
            t.x + moved * err * math.cos(t.theta),
            t.y + moved * err * math.sin(t.theta),
            t.theta,
        )
        o = self._odom
        self._odom = Pose(
            o.x + moved * math.cos(o.theta), o.y + moved * math.sin(o.theta), o.theta
        )
        return moved

    def read_range(self) -> Optional[float]:
        t = self.true
        d = self.world.raycast(t.x, t.y, t.theta, self.max_range)
        if d is not None and self.range_noise:
            d = max(0.0, d + self._rng.gauss(0, self.range_noise))
        return d
