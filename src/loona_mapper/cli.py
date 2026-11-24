"""Command line entry point: `loona-map`."""

from __future__ import annotations

import argparse
import os
import sys

from .adapters.sim import SimulatedRobot, SimWorld
from .export import render_rgb, write_png, write_ros_map
from .grid import OccupancyGrid
from .mapper import RoomMapper


def _save_all(grid, trail, out_dir: str, name: str = "map") -> None:
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, name)
    grid.save(stem + ".json")
    write_png(stem + ".png", render_rgb(grid, trail))
    write_ros_map(grid, stem)
    print(f"wrote {stem}.json, .png, .pgm, .yaml")


def _cmd_sim(a: argparse.Namespace) -> int:
    robot = SimulatedRobot(
        SimWorld.demo(), range_noise=a.range_noise, drift=a.drift, seed=a.seed
    )
    mapper = RoomMapper(robot)
    res = mapper.explore()
    print(
        f"{res.reason}; {res.iterations} iterations, "
        f"explored {res.grid.explored_area():.1f} m^2"
    )
    _save_all(res.grid, res.trail, a.out)
    return 0


def _cmd_run(a: argparse.Namespace) -> int:
    from .adapters.loona import LoonaAdapter

    robot = LoonaAdapter(host=a.host)
    try:
        res = RoomMapper(robot).explore()
    except NotImplementedError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    finally:
        robot.close()
    _save_all(res.grid, res.trail, a.out)
    return 0


def _cmd_render(a: argparse.Namespace) -> int:
    grid = OccupancyGrid.load(a.map)
    write_png(a.png, render_rgb(grid, scale=a.scale))
    print(f"wrote {a.png}")
    return 0


def _cmd_discover(a: argparse.Namespace) -> int:
    from .discover import DEFAULT_PORTS, expand, scan

    ports = [int(p) for p in a.ports.split(",")] if a.ports else DEFAULT_PORTS
    try:
        hosts = expand(a.target)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(f"probing {len(hosts)} host(s) x {len(ports)} port(s)...")
    found = scan(hosts, ports, timeout=a.timeout)
    if not found:
        print("no open TCP ports found. The robot may use UDP, BLE, or only talk to a cloud service.")
        return 1
    for host, open_ports in sorted(found.items()):
        print(host)
        for port, banner in sorted(open_ports.items()):
            print(f"  {port}/tcp open  {banner}")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="loona-map", description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("sim", help="explore a simulated room (no robot needed)")
    s.add_argument("--out", default="out")
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--range-noise", type=float, default=0.01, help="sensor sigma, m")
    s.add_argument("--drift", type=float, default=0.0, help="motion error sigma, fraction")
    s.set_defaults(fn=_cmd_sim)

    r = sub.add_parser("run", help="explore with a real Loona (needs adapter wired up)")
    r.add_argument("--out", default="out")
    r.add_argument("--host", default=None)
    r.set_defaults(fn=_cmd_run)

    g = sub.add_parser("render", help="re-render a saved map.json to PNG")
    g.add_argument("map")
    g.add_argument("png")
    g.add_argument("--scale", type=int, default=4)
    g.set_defaults(fn=_cmd_render)

    d = sub.add_parser("discover", help="probe your LAN for what Loona exposes")
    d.add_argument("target", help="IP address, hostname, or CIDR block (e.g. 192.168.1.0/24)")
    d.add_argument("--ports", help="comma-separated TCP ports (default: common IoT ports)")
    d.add_argument("--timeout", type=float, default=0.5)
    d.set_defaults(fn=_cmd_discover)

    a = p.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
