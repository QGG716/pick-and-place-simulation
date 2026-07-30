"""Standalone PyBullet viewer entry point for KUKA KR 50 R2500."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.pybullet_sim import DEFAULT_KUKA_KR50_PACKAGE_ROOT, DEFAULT_KUKA_KR50_URDF, parse_args, run_viewer
else:
    from .pybullet_sim import DEFAULT_KUKA_KR50_PACKAGE_ROOT, DEFAULT_KUKA_KR50_URDF, parse_args, run_viewer


def main(argv: Sequence[str] | None = None) -> None:
    user_args = list(sys.argv[1:] if argv is None else argv)
    defaults = [
        "--config", "config/kuka_kr50.yaml",
        "--trajectory", "outputs/kuka_kr50/trajectory.csv",
        "--robot-urdf", str(DEFAULT_KUKA_KR50_URDF),
        "--package-root", str(DEFAULT_KUKA_KR50_PACKAGE_ROOT),
        "--end-effector-link", "flange",
        "--base-position", "-1.15", "0.0", "0.0",
        "--base-rpy", "0.0", "0.0", "0.0",
        "--gripper-xyz", "0.0", "0.0", "0.055",
        "--suction-radius", "0.065",
    ]
    run_viewer(parse_args([*defaults, *user_args]))


if __name__ == "__main__":
    main()