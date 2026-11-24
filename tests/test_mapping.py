"""End-to-end: explore a simulated room and score the map against the truth."""

import numpy as np
import pytest

from loona_mapper.adapters.sim import SimulatedRobot, SimWorld
from loona_mapper.adapters.loona import LoonaAdapter
from loona_mapper.cli import main
from loona_mapper.grid import FREE, OCCUPIED, inflate
from loona_mapper.mapper import RoomMapper


def _score(world, res):
    """Compare against ground truth, tolerating one cell (5 cm) of blur."""
    free_t, wall_t = world.truth_masks(res.grid)
    cls = res.grid.classify()
    occ = cls == OCCUPIED
    near_wall = inflate(wall_t, 1)
    near_occ = inflate(occ, 1)
    interior = free_t & ~near_wall
    free_recall = ((cls == FREE) & interior).sum() / interior.sum()
    wall_precision = (occ & near_wall).sum() / max(1, occ.sum())
    wall_recall = (wall_t & near_occ).sum() / wall_t.sum()
    return free_recall, wall_precision, wall_recall


def test_explores_demo_room_accurately():
    world = SimWorld.demo()
    res = RoomMapper(SimulatedRobot(world)).explore()
    assert res.finished
    free_recall, wall_p, wall_r = _score(world, res)
    assert free_recall > 0.95
    assert wall_p > 0.95
    assert wall_r > 0.8  # some wall faces are never visible to the robot


def test_robust_to_sensor_noise():
    world = SimWorld.demo()
    res = RoomMapper(SimulatedRobot(world, range_noise=0.01, seed=3)).explore()
    free_recall, wall_p, _ = _score(world, res)
    assert res.finished and free_recall > 0.9 and wall_p > 0.85


def test_drifting_odometry_terminates_but_map_quality_is_not_guaranteed():
    # Known limitation: with no scan matching, odometry drift smears the map.
    # This only guards against hangs/crashes under bad odometry.
    world = SimWorld.demo()
    res = RoomMapper(SimulatedRobot(world, drift=0.05, seed=5)).explore()
    assert res.iterations <= 200
    assert res.grid.explored_area() > 1.0


def test_robot_never_leaves_the_room():
    world = SimWorld.demo()
    robot = SimulatedRobot(world)
    res = RoomMapper(robot).explore()
    assert all(world.is_free(x, y) for x, y in res.trail)


def test_cli_sim_writes_all_outputs(tmp_path):
    assert main(["sim", "--out", str(tmp_path)]) == 0
    for name in ("map.json", "map.png", "map.pgm", "map.yaml"):
        assert (tmp_path / name).stat().st_size > 0
    assert (tmp_path / "map.png").read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"
    assert main(["render", str(tmp_path / "map.json"), str(tmp_path / "again.png")]) == 0


def test_loona_adapter_is_an_explicit_stub(capsys):
    with pytest.raises(NotImplementedError):
        LoonaAdapter().read_range()
    assert main(["run", "--out", "unused"]) == 2
    assert "not implemented" in capsys.readouterr().err


def test_discover_finds_open_port_and_banner():
    import socket
    import threading

    from loona_mapper.discover import expand, scan

    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        conn, _ = srv.accept()
        conn.recv(1024)
        conn.sendall(b"HTTP/1.0 200 OK\r\nServer: fake-loona\r\n\r\n")
        conn.close()

    threading.Thread(target=serve, daemon=True).start()
    found = scan(["127.0.0.1"], ports=[port, port + 1 if port < 65535 else port - 1])
    srv.close()
    assert "fake-loona" in found["127.0.0.1"][port]
    assert len(found["127.0.0.1"]) == 1
    assert expand("10.0.0.0/30") == ["10.0.0.1", "10.0.0.2"]
    with pytest.raises(ValueError):
        expand("10.0.0.0/16")
