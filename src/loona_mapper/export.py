"""Write maps as PNG (for people) and PGM+YAML (ROS map_server compatible)."""

from __future__ import annotations

import struct
import zlib
from typing import Iterable, Optional, Tuple

import numpy as np

from .grid import FREE, OCCUPIED, OccupancyGrid

_COLOURS = {
    "unknown": (150, 150, 150),
    "free": (255, 255, 255),
    "occupied": (20, 20, 20),
    "trail": (220, 60, 60),
}


def render_rgb(
    grid: OccupancyGrid,
    trail: Optional[Iterable[Tuple[float, float]]] = None,
    scale: int = 4,
    crop: bool = True,
) -> np.ndarray:
    """RGB uint8 image with +y up, optionally cropped to the explored area."""
    cls = grid.classify()
    img = np.empty(cls.shape + (3,), dtype=np.uint8)
    img[:] = _COLOURS["unknown"]
    img[cls == FREE] = _COLOURS["free"]
    img[cls == OCCUPIED] = _COLOURS["occupied"]
    if trail:
        for x, y in trail:
            c, r = grid.world_to_cell(x, y)
            if grid.in_bounds(c, r):
                img[r, c] = _COLOURS["trail"]
    b = grid.bounds() if crop else None
    if b:
        pad = 4
        c0, r0, c1, r1 = b
        img = img[max(0, r0 - pad) : r1 + pad + 1, max(0, c0 - pad) : c1 + pad + 1]
    img = img[::-1]  # row 0 is lowest y; images put the top first
    return np.repeat(np.repeat(img, scale, axis=0), scale, axis=1)


def write_png(path: str, rgb: np.ndarray) -> None:
    h, w, _ = rgb.shape

    def chunk(tag: bytes, data: bytes) -> bytes:
        body = tag + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )
    with open(path, "wb") as f:
        f.write(png)


def write_ros_map(grid: OccupancyGrid, stem: str) -> None:
    """`<stem>.pgm` + `<stem>.yaml`, loadable by ROS nav2/map_server."""
    cls = grid.classify()
    pixels = np.full(cls.shape, 205, dtype=np.uint8)  # unknown
    pixels[cls == FREE] = 254
    pixels[cls == OCCUPIED] = 0
    pixels = pixels[::-1]
    with open(stem + ".pgm", "wb") as f:
        f.write(f"P5\n{grid.width} {grid.height}\n255\n".encode())
        f.write(pixels.tobytes())
    name = stem.replace("\\", "/").rsplit("/", 1)[-1]
    with open(stem + ".yaml", "w") as f:
        f.write(
            f"image: {name}.pgm\n"
            f"resolution: {grid.resolution}\n"
            f"origin: [{grid.origin[0]}, {grid.origin[1]}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
        )
