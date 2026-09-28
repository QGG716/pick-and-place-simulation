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
    parser.add_argument("--fixed-history-fixture", type=Path, help="Validate only this fixed candidate; requires --target, no candidate sweep")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.fixed_history_fixture is not None:
            # The repository fixture is an extracted hint, not a complete prior
            # audit document. Seal a separate derivative; never overwrite it or
            # inherit its historical validation. HistorySource still verifies the
            # derivative content fingerprint and compatible_hint checks assets/FK.
            import hashlib
            from unloading_sim.workcell_layout import canonical_digest
            raw = args.fixed_history_fixture.read_bytes()
            hint = json.loads(raw)
            hint["fixed_fixture_derivation"] = {
                "source_path": str(args.fixed_history_fixture.resolve()),
                "source_sha256": hashlib.sha256(raw).hexdigest(),
                "source_evidence_fingerprint": hint.get("evidence_fingerprint"),
                "validation_inherited": False,
                "scope": "fixed candidate hint; all current complete-task checks required"}
            # Extracted fixtures omit old report metadata. Keep it explicitly
            # unknown; never substitute the current policy/implementation as old.
            hint.setdefault("policy_fingerprint", None)
            hint.setdefault("implementation_identity", None)
            hint["run_status"] = "HINT_ONLY"
            hint["complete_trajectory_status"] = "NOT_VALIDATED"
            hint["evidence_fingerprint"] = canonical_digest({k:v for k,v in hint.items() if k!="evidence_fingerprint"})
            derived = args.output.with_suffix(".hint.json")
            derived.parent.mkdir(parents=True,exist_ok=True)
            derived.write_text(json.dumps(hint,indent=2,allow_nan=False),encoding="utf-8")
            args.fixed_history_fixture = derived
        result = run_layout_single_carton_audit(args.config, project_root=ROOT, backend=args.backend, moveit_command=args.moveit_command, target_id=args.target, fixed_history_fixture=args.fixed_history_fixture)
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
