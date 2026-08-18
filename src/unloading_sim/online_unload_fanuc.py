"""FANUC M-20iD/35 entry point for the shared unloading planner."""

from __future__ import annotations

import sys

from .online_unload import main


if __name__ == "__main__":
    main(["--config", "config/fanuc_m20id35.yaml", *sys.argv[1:]])
