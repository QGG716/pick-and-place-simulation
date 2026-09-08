from __future__ import annotations

import os
import sys


def require_humble_python310() -> None:
    distro = os.environ.get("ROS_DISTRO")
    if distro != "humble":
        raise RuntimeError(f"ROS_DISTRO must be 'humble', got {distro!r}")
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError(f"ROS nodes require Python 3.10 on Ubuntu 22.04, got {sys.version.split()[0]}")


def float_to_time(value: float):
    from builtin_interfaces.msg import Time

    seconds = int(value)
    return Time(sec=seconds, nanosec=int(round((value - seconds) * 1_000_000_000)))
