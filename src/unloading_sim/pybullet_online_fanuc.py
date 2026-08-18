"""FANUC M-20iD/35 entry point for PyBullet online-plan replay."""

from __future__ import annotations

import sys

from .pybullet_online_kuka import main


if __name__ == "__main__":
    main(["--plan", "outputs/fanuc_m20id35/online_plan.json", *sys.argv[1:]])
