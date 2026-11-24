"""Adapter for the real Loona robot.

This is the only file that has to change to run the mapper on hardware.

I do not know of a documented public control API for Loona, so every method
below is a stub that says what it needs. Fill them in once you know how to
reach the robot (official SDK, local network protocol, BLE, ADB, ...).

What the mapper requires, and what to look for on the Loona side:

* pose()        Dead-reckoned (x, y, theta) in metres/radians, starting at
                (0, 0, 0). If Loona exposes wheel odometry or an IMU heading,
                integrate it here. If not, integrate your own commanded moves
                (what SimulatedRobot does).
* rotate()      Turn in place and block until done.
* forward()     Drive straight and block; return how far it really went.
* read_range()  Loona's front-facing distance (ToF / proximity) sensor, in
                metres, or None when nothing is in range. Also set
                `max_range` to the sensor's real limit.

Calibrate before trusting a map: command a 1 m drive and a 360 degree turn,
measure what really happened, and scale the numbers in here accordingly.
"""

from __future__ import annotations

from typing import Optional

from ..robot import Pose, RobotAdapter

_MSG = (
    "LoonaAdapter.{} is not implemented: wire it to Loona's control interface "
    "(see the module docstring in adapters/loona.py)."
)


class LoonaAdapter(RobotAdapter):
    max_range = 1.0  # TODO: set to the real sensor limit
    body_radius = 0.12  # TODO: measure

    def __init__(self, host: Optional[str] = None) -> None:
        self.host = host

    def pose(self) -> Pose:
        raise NotImplementedError(_MSG.format("pose"))

    def rotate(self, radians: float) -> None:
        raise NotImplementedError(_MSG.format("rotate"))

    def forward(self, meters: float) -> float:
        raise NotImplementedError(_MSG.format("forward"))

    def read_range(self) -> Optional[float]:
        raise NotImplementedError(_MSG.format("read_range"))
