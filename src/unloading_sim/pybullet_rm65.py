"""Standalone PyBullet viewer entry point for the RealMan RM65 setup."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from unloading_sim.pybullet_sim import parse_args, run_viewer
else:
    from .pybullet_sim import parse_args, run_viewer


def main(argv: Sequence[str] | None = None) -> None:
    run_viewer(parse_args(argv))


if __name__ == "__main__":
    main()