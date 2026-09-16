"""Compatibility CLI: historical hints enter the ordinary bounded planner."""
from __future__ import annotations
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.history_adaptation import recheck_current_release


def main(argv=None):
    # Imported lazily so the shared geometry helper remains importable by callers.
    sys.path.insert(0, str(ROOT))
    from tools.run_m710_contact_unloading import main as plan_main
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--actual-state", type=Path)
    parser.add_argument("--simplify-unloaded", action="store_true",
                        help="deprecated: current planner validates the entire free prefix")
    parser.add_argument("--config", default="configs/validation/m710id70_layout_v1_single_carton.yaml")
    parser.add_argument("--execution-config", default="configs/simulation/m710id70_first_row_recording_v1.yaml")
    parser.add_argument("--planning-wall-time-s", type=float)
    args = parser.parse_args(argv)
    command = ["--reuse-motion", str(args.motion), "--output", str(args.output),
               "--config", args.config, "--execution-config", args.execution_config]
    if args.actual_state:
        command += ["--actual-state", str(args.actual_state)]
    if args.planning_wall_time_s is not None:
        command += ["--planning-wall-time-s", str(args.planning_wall_time_s)]
    return plan_main(command)


if __name__ == "__main__":
    raise SystemExit(main())
