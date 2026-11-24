"""The only interface the mapper needs from a robot.

Loona has a small sensor set, so the interface is deliberately minimal: one
forward-facing distance reading, in-place rotation, straight-line driving and
a dead-reckoned pose. The mapper builds a 360 degree scan by rotating in
place and taking a reading at every step.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Pose:
    """Robot pose in the map frame. x/y in metres, theta in radians (CCW)."""

    x: float
    y: float
    theta: float


def normalize_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class RobotAdapter(ABC):
    """Implement this for a real robot (see adapters/loona.py) or a simulator."""

    #: Furthest distance the range sensor can report, in metres.
    max_range: float = 2.0
    #: Radius used to keep the robot away from walls, in metres.
    body_radius: float = 0.12

    @abstractmethod
    def pose(self) -> Pose:
        """Current dead-reckoned pose. The start pose defines the map origin."""

    @abstractmethod
    def rotate(self, radians: float) -> None:
        """Turn in place by `radians` (positive = counter-clockwise). Blocks."""

    @abstractmethod
    def forward(self, meters: float) -> float:
        """Drive straight. Returns the distance actually travelled (may be
        shorter than requested if the robot bumped into something). Blocks."""

    @abstractmethod
    def read_range(self) -> Optional[float]:
        """Distance to the nearest obstacle straight ahead, in metres.
        Returns None if nothing was detected within `max_range`."""

    def close(self) -> None:
        """Release connections. Optional."""
