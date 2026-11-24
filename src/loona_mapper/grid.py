"""Log-odds occupancy grid with ray-cast sensor updates."""

from __future__ import annotations

import json
import math
from typing import Optional, Tuple

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 1


class OccupancyGrid:
    """2D grid in the map frame.

    Cell (col, row) covers x in [origin_x + col*res, origin_x + (col+1)*res),
    and likewise for y/row. Row 0 is the *lowest* y.
    """

    L_OCC = 0.85
    L_FREE = -0.4
    L_MIN, L_MAX = -5.0, 5.0
    FREE_THRESH = -0.35
    OCC_THRESH = 0.8

    def __init__(
        self,
        width_m: float = 8.0,
        height_m: float = 8.0,
        resolution: float = 0.05,
        origin: Optional[Tuple[float, float]] = None,
    ) -> None:
        self.resolution = resolution
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        # Default: robot start (0, 0) is at the centre of the grid.
        self.origin = origin if origin is not None else (-width_m / 2, -height_m / 2)
        self.logodds = np.zeros((self.height, self.width), dtype=np.float32)
        # True once any reading has touched the cell. Distinguishes "never
        # looked" from "looked and the evidence conflicts".
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    # -- coordinates -------------------------------------------------------

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = int(math.floor((x - self.origin[0]) / self.resolution))
        row = int(math.floor((y - self.origin[1]) / self.resolution))
        return col, row

    def cell_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Centre of the cell."""
        return (
            self.origin[0] + (col + 0.5) * self.resolution,
            self.origin[1] + (row + 0.5) * self.resolution,
        )

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    # -- sensor updates ----------------------------------------------------

    def integrate_reading(
        self,
        x: float,
        y: float,
        heading: float,
        distance: Optional[float],
        max_range: float,
        beam_width: float = 0.0,
    ) -> None:
        """Fold one range reading into the grid.

        `distance=None` means "nothing within max_range": everything along the
        beam is marked free and no obstacle is recorded.

        A single-beam sensor reading represents a slice of space, not an
        infinitely thin ray. `beam_width` (radians) spreads the update over
        several sub-rays so adjacent readings leave no unknown gaps. Each cell
        is updated at most once per reading, however many sub-rays cross it.
        """
        hit = distance is not None
        reach = min(distance, max_range) if hit else max_range
        res = self.resolution
        step = res / 2  # sample twice per cell so no cell on the ray is skipped
        if beam_width > 0 and reach > 0:
            n = max(1, int(math.ceil(2 * reach * beam_width / res)))
            angles = heading + beam_width * ((np.arange(n) + 0.5) / n - 0.5)
        else:
            angles = np.array([heading])
        cos, sin = np.cos(angles)[:, None], np.sin(angles)[:, None]

        t = np.arange(0.0, reach if hit else reach + 1e-9, step)[None, :]
        free = self._cell_indices(x + t * cos, y + t * sin)
        if hit:
            end = self._cell_indices(x + reach * cos, y + reach * sin)
            free = np.setdiff1d(free, end, assume_unique=True)
            self._add(end, self.L_OCC)
        self._add(free, self.L_FREE)

    def _cell_indices(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Unique flat indices of in-bounds cells containing the points."""
        cols = np.floor((xs - self.origin[0]) / self.resolution).astype(np.int64)
        rows = np.floor((ys - self.origin[1]) / self.resolution).astype(np.int64)
        ok = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        return np.unique(rows[ok] * self.width + cols[ok])

    def _add(self, flat_idx: np.ndarray, delta: float) -> None:
        lo = self.logodds.reshape(-1)
        lo[flat_idx] = np.clip(lo[flat_idx] + delta, self.L_MIN, self.L_MAX)
        self.observed.reshape(-1)[flat_idx] = True

    # -- queries -----------------------------------------------------------

    def classify(self) -> np.ndarray:
        """int8 array of UNKNOWN / FREE / OCCUPIED, shape (height, width)."""
        out = np.full(self.logodds.shape, UNKNOWN, dtype=np.int8)
        out[self.logodds <= self.FREE_THRESH] = FREE
        out[self.logodds >= self.OCC_THRESH] = OCCUPIED
        return out

    def probability(self) -> np.ndarray:
        """Occupancy probability per cell; 0.5 where unknown."""
        return 1.0 - 1.0 / (1.0 + np.exp(self.logodds))

    def explored_area(self) -> float:
        """Known (free + occupied) area in square metres."""
        return float((self.classify() != UNKNOWN).sum()) * self.resolution**2

    def bounds(self) -> Optional[Tuple[int, int, int, int]]:
        """Bounding box of known cells as (col_min, row_min, col_max, row_max)."""
        rows, cols = np.nonzero(self.classify() != UNKNOWN)
        if len(rows) == 0:
            return None
        return int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())

    # -- persistence -------------------------------------------------------

    def to_dict(self) -> dict:
        return {
            "format": "loona-mapper/1",
            "resolution": self.resolution,
            "width": self.width,
            "height": self.height,
            "origin": list(self.origin),
            # Only store cells that carry information, as [row, col, value].
            "cells": [
                [int(r), int(c), round(float(self.logodds[r, c]), 3)]
                for r, c in zip(*np.nonzero(self.logodds))
            ],
        }

    @classmethod
    def from_dict(cls, d: dict) -> "OccupancyGrid":
        if d.get("format") != "loona-mapper/1":
            raise ValueError("not a loona-mapper map file")
        g = cls(
            width_m=d["width"] * d["resolution"],
            height_m=d["height"] * d["resolution"],
            resolution=d["resolution"],
            origin=tuple(d["origin"]),
        )
        for r, c, v in d["cells"]:
            g.logodds[r, c] = v
        g.observed = g.logodds != 0
        return g

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f)

    @classmethod
    def load(cls, path: str) -> "OccupancyGrid":
        with open(path) as f:
            return cls.from_dict(json.load(f))


def inflate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate a boolean mask by a disc of `radius_cells` (pure numpy)."""
    if radius_cells <= 0:
        return mask.copy()
    out = np.zeros_like(mask)
    h, w = mask.shape
    r = radius_cells
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r:
                continue
            src = mask[max(0, -dy) : h - max(0, dy), max(0, -dx) : w - max(0, dx)]
            out[max(0, dy) : h - max(0, -dy), max(0, dx) : w - max(0, -dx)] |= src
    return out
