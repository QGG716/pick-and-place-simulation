"""Prepare a fail-closed M-710 dynamic-execution bundle without launching Isaac."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.m710_execution import (  # noqa: E402
    DEFAULT_CONFIG_PATH,
    build_m710_execution_preflight,
    verify_m710_execution_preflight,
    write_m710_execution_preflight,
)


DEFAULT_MOTION_RESULT = ROOT / "outputs/m710id70_layout_v1/single_carton_motion_audit.json"
DEFAULT_OUTPUT = ROOT / "outputs/m710id70_layout_v1/dynamic_execution_preflight.json"


def _read_mapping(path: Path, label: str) -> Mapping[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"{label} is missing: {path}")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain one JSON object")
    return value


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument(
        "--motion-result",
        type=Path,
        default=DEFAULT_MOTION_RESULT,
        help="content-addressed output from run_m710id70_layout_single_carton.py",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        motion = _read_mapping(args.motion_result.resolve(), "motion result")
        result = build_m710_execution_preflight(
            args.config.resolve(),
            motion_result=motion,
            backend_execution_status="NOT_RUN",
        )
        verification = verify_m710_execution_preflight(result)
        destination = write_m710_execution_preflight(result, args.output.resolve())
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    print(
        json.dumps(
            {
                "status": result["status"],
                "simulation_execution_ready": result["simulation_execution_ready"],
                "execution_qualified": result["execution_qualified"],
                "simulation_readiness_blockers": result["simulation_readiness_blockers"],
                "machine_qualified": result["machine_qualified"],
                "machine_qualification_warnings": result["machine_qualification_warnings"],
                "backend_execution_status": result["backend_execution_status"],
                "isaac_validation_performed": result["isaac_validation_performed"],
                "video": result["claims"]["video"],
                "dynamic_carton_count": result["scene"]["dynamic_carton_count"],
                "preflight_fingerprint": verification["preflight_fingerprint"],
                "output": str(destination),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
