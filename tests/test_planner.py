import numpy as np

from loona_mapper.grid import OccupancyGrid
from loona_mapper.planner import frontier_mask, plan_to_nearest_frontier


def _open_disc(radius_m):
    g = OccupancyGrid(6, 6, 0.05)
    for i in range(120):
        g.integrate_reading(0, 0, i * 3.14159 / 60, radius_m, 3.0, beam_width=0.06)
    return g


def test_frontier_found_on_unexplored_edge():
    g = OccupancyGrid(6, 6, 0.05)
    g.integrate_reading(0, 0, 0.0, None, 1.0, beam_width=0.2)  # open ahead, no wall
    assert frontier_mask(g).any()
    path = plan_to_nearest_frontier(g, g.world_to_cell(0, 0), 0.15)
    assert path is not None and path[0] == g.world_to_cell(0, 0)


def test_fully_walled_room_has_no_frontier():
    g = _open_disc(1.0)
    assert plan_to_nearest_frontier(g, g.world_to_cell(0, 0), 0.15) is None


def test_ignore_suppresses_visited_frontier():
    g = OccupancyGrid(6, 6, 0.05)
    g.integrate_reading(0, 0, 0.0, None, 1.0, beam_width=0.2)
    start = g.world_to_cell(0, 0)
    path = plan_to_nearest_frontier(g, start, 0.15)
    assert plan_to_nearest_frontier(g, start, 0.15, ignore=[path[-1]], ignore_radius_cells=60) is None
