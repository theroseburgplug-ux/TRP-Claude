"""RoomMapper: drives a robot around and builds an occupancy grid."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from .grid import OccupancyGrid
from .planner import Cell, plan_to_nearest_frontier
from .robot import Pose, RobotAdapter, normalize_angle


@dataclass
class MapperConfig:
    scan_step_deg: float = 6.0  # rotation between readings during a sweep
    beam_width_deg: float = 6.0  # angular slice covered by each reading
    step_m: float = 0.25  # longest single drive command
    clearance_m: float = 0.05  # planner keeps this much beyond the body radius
    max_iterations: int = 200  # safety cap on explore loop
    min_cluster: int = 3  # ignore frontiers smaller than this many cells
    arrive_tol_m: float = 0.15


@dataclass
class MappingResult:
    grid: OccupancyGrid
    trail: List[Tuple[float, float]] = field(default_factory=list)
    iterations: int = 0
    finished: bool = False  # True if exploration ran out of frontiers
    reason: str = ""


class RoomMapper:
    def __init__(
        self,
        robot: RobotAdapter,
        grid: Optional[OccupancyGrid] = None,
        config: Optional[MapperConfig] = None,
        on_progress: Optional[Callable[[MappingResult], None]] = None,
    ) -> None:
        self.robot = robot
        self.grid = grid or OccupancyGrid()
        self.cfg = config or MapperConfig()
        self.on_progress = on_progress
        self.result = MappingResult(grid=self.grid)
        self._visited: List[Cell] = []
        self._record_pose()

    # -- sensing -----------------------------------------------------------

    def _record_pose(self) -> None:
        p = self.robot.pose()
        self.result.trail.append((p.x, p.y))

    def _integrate(self, beam_width: float = 0.0) -> Optional[float]:
        """Take one reading at the current pose and add it to the grid."""
        p = self.robot.pose()
        d = self.robot.read_range()
        self.grid.integrate_reading(
            p.x, p.y, p.theta, d, self.robot.max_range, beam_width
        )
        return d

    def sweep(self) -> None:
        """Rotate a full turn in place, integrating a reading at every step."""
        step = math.radians(self.cfg.scan_step_deg)
        beam = math.radians(self.cfg.beam_width_deg)
        n = int(round(2 * math.pi / step))
        for _ in range(n):
            self._integrate(beam)
            self.robot.rotate(step)
        self._integrate(beam)

    # -- motion ------------------------------------------------------------

    def _face(self, target_theta: float) -> None:
        delta = normalize_angle(target_theta - self.robot.pose().theta)
        if abs(delta) > 1e-3:
            self.robot.rotate(delta)

    def _drive_to(self, x: float, y: float) -> bool:
        """Drive to a world point in short, sensed hops. False if blocked."""
        while True:
            p = self.robot.pose()
            dx, dy = x - p.x, y - p.y
            dist = math.hypot(dx, dy)
            if dist < self.cfg.arrive_tol_m / 2:
                return True
            self._face(math.atan2(dy, dx))
            hop = min(dist, self.cfg.step_m)
            ahead = self._integrate(beam_width=math.radians(self.cfg.beam_width_deg))
            if ahead is not None and ahead < hop + self.robot.body_radius:
                return False
            moved = self.robot.forward(hop)
            self._record_pose()
            if moved < hop - 0.02:
                return False  # bumped

    # -- exploration -------------------------------------------------------

    def explore(self) -> MappingResult:
        """Frontier-based exploration until nothing reachable is left to see."""
        cfg = self.cfg
        self.sweep()
        for i in range(cfg.max_iterations):
            self.result.iterations = i + 1
            p = self.robot.pose()
            start = self.grid.world_to_cell(p.x, p.y)
            path = plan_to_nearest_frontier(
                self.grid,
                start,
                self.robot.body_radius + cfg.clearance_m,
                min_cluster=cfg.min_cluster,
                ignore=self._visited,
            )
            if path is None:
                self.result.finished = True
                self.result.reason = "no reachable frontiers left"
                break

            goal = path[-1]
            reached = True
            # Follow the path, skipping cells to take longer straight hops.
            for cell in path[1 :: max(1, int(cfg.step_m / self.grid.resolution))] + [goal]:
                wx, wy = self.grid.cell_to_world(*cell)
                if not self._drive_to(wx, wy):
                    reached = False
                    break
            self.sweep()
            if reached:
                # Whatever is still frontier here cannot be resolved from this
                # spot; do not come back for it.
                self._visited.append(goal)
            if self.on_progress:
                self.on_progress(self.result)
        else:
            self.result.reason = "iteration limit reached"
        return self.result
