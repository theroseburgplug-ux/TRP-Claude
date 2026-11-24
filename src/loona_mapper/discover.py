"""Find what a device on your LAN exposes. Use it to learn how Loona is controlled.

    loona-map discover 192.168.1.50          # probe one host (Loona's IP)
    loona-map discover 192.168.1.0/24        # sweep a subnet for open ports

Only run this against your own network. It makes plain TCP connections and, on
open ports, sends one harmless HTTP request to read a banner. Nothing is
modified on the target.
"""

from __future__ import annotations

import ipaddress
import socket
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterable, List, Optional

# Ports commonly used by hobby robots / IoT devices for control, video, MQTT, debug.
DEFAULT_PORTS = [
    21, 22, 23, 53, 80, 81, 443, 554, 1883, 5000, 5555, 6668, 7001, 7070,
    8000, 8001, 8080, 8081, 8443, 8554, 8883, 8888, 9000, 9090, 10000, 12345,
]


def _probe(host: str, port: int, timeout: float) -> Optional[str]:
    """Return a banner string if the TCP port is open, else None."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            try:
                s.sendall(f"GET / HTTP/1.0\r\nHost: {host}\r\n\r\n".encode())
                data = s.recv(256)
            except OSError:
                data = b""
            return data.decode("latin-1", "replace").strip().replace("\r\n", " | ")[:120]
    except OSError:
        return None


def scan(
    targets: Iterable[str],
    ports: Iterable[int] = DEFAULT_PORTS,
    timeout: float = 0.5,
    workers: int = 128,
) -> Dict[str, Dict[int, str]]:
    """Map host -> {open port: banner}. Hosts with nothing open are omitted."""
    jobs = [(h, p) for h in targets for p in ports]
    found: Dict[str, Dict[int, str]] = {}
    with ThreadPoolExecutor(max_workers=workers) as ex:
        for (h, p), banner in zip(jobs, ex.map(lambda j: _probe(*j, timeout), jobs)):
            if banner is not None:
                found.setdefault(h, {})[p] = banner
    return found


def expand(target: str) -> List[str]:
    """A single IP/hostname, or every usable host in a CIDR block (max /22)."""
    if "/" not in target:
        return [target]
    net = ipaddress.ip_network(target, strict=False)
    if net.num_addresses > 1024:
        raise ValueError("refusing to scan more than 1024 addresses; use a smaller block")
    return [str(h) for h in net.hosts()]
