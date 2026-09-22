"""Run the frozen-layout M-710iD/70 single-carton motion audit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (  # noqa: E402
    run_layout_single_carton_audit,
    write_layout_single_carton_audit,
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs/validation/m710id70_proof_of_concept.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "outputs/m710id70_layout_v1/single_carton_motion_audit.json",
    )
    parser.add_argument("--backend", choices=("core", "moveit2"), default="core")
    parser.add_argument("--moveit-command", help="JSONL resident worker launcher; alternatively M710_MOVEIT_COMMAND")
    parser.add_argument("--target", help="One legal carton from the unchanged frozen scene")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = run_layout_single_carton_audit(args.config, project_root=ROOT, backend=args.backend, moveit_command=args.moveit_command, target_id=args.target)
        output = write_layout_single_carton_audit(result, args.output)
    except Exception as exc:
        failure={"run_status": "BLOCKED", "backend": args.backend,
            "complete_trajectory_status": "NOT_RUN", "reason": str(exc)}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(failure,ensure_ascii=False,indent=2),encoding="utf-8")
        print(json.dumps(failure,ensure_ascii=False))
        return 1
    print(
        json.dumps(
            {
                "run_status": result["run_status"],
                "complete_trajectory_status": result["complete_trajectory_status"],
                "complete_trajectory_failure_reason": result["complete_trajectory_failure_reason"],
                "task_population": result["task_population"]["carton_ids"],
                "statistics": result["statistics"],
                "scene_fingerprint": result["scene_fingerprint"],
                "evidence_fingerprint": result["evidence_fingerprint"],
                "output": str(output.resolve()),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 1 if result["run_status"] == "BLOCKED" else 0


if __name__ == "__main__":
    raise SystemExit(main())
