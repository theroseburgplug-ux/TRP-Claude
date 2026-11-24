import math

from loona_mapper.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, inflate

import numpy as np


def test_world_cell_roundtrip():
    g = OccupancyGrid(4, 4, 0.05)
    c, r = g.world_to_cell(0.0, 0.0)
    assert (c, r) == (40, 40)
    x, y = g.cell_to_world(c, r)
    assert abs(x - 0.025) < 1e-9 and abs(y - 0.025) < 1e-9


def test_hit_marks_free_then_occupied():
    g = OccupancyGrid(4, 4, 0.05)
    g.integrate_reading(0, 0, 0.0, 1.0, max_range=2.0)  # wall 1 m ahead (+x)
    cls = g.classify()
    assert cls[g.world_to_cell(0.5, 0.0)[::-1]] == FREE
    assert cls[g.world_to_cell(1.0, 0.0)[::-1]] == OCCUPIED
    assert cls[g.world_to_cell(1.5, 0.0)[::-1]] == UNKNOWN  # behind the wall


def test_no_return_marks_free_only():
    g = OccupancyGrid(4, 4, 0.05)
    g.integrate_reading(0, 0, math.pi / 2, None, max_range=1.0)
    assert (g.classify() == OCCUPIED).sum() == 0
    assert g.classify()[g.world_to_cell(0.0, 0.8)[::-1]] == FREE


def test_beam_width_leaves_no_gaps_between_adjacent_readings():
    g = OccupancyGrid(6, 6, 0.05)
    step = math.radians(6)
    for i in range(60):
        g.integrate_reading(0, 0, i * step, 2.0, 2.5, beam_width=step)
    # Every cell strictly inside the ring (r < 1.9 m) must be known.
    for r in range(g.height):
        for c in range(g.width):
            x, y = g.cell_to_world(c, r)
            if 0.1 < math.hypot(x, y) < 1.9:
                assert g.classify()[r, c] == FREE, (x, y)


def test_logodds_clamped_and_save_load(tmp_path):
    g = OccupancyGrid(2, 2, 0.05)
    for _ in range(100):
        g.integrate_reading(0, 0, 0.0, 0.5, 1.0)
    assert g.logodds.max() <= g.L_MAX
    p = tmp_path / "m.json"
    g.save(str(p))
    g2 = OccupancyGrid.load(str(p))
    assert np.array_equal(g.classify(), g2.classify())


def test_inflate_radius():
    m = np.zeros((11, 11), dtype=bool)
    m[5, 5] = True
    out = inflate(m, 2)
    assert out[5, 7] and out[7, 5] and not out[7, 7] and not out[5, 8]
