"""Check an Isaac world handoff against a detached feasibility checkout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "packages/unloading_contracts/src"))
sys.path.insert(0, str(ROOT / "src"))

from unloading_perception.isaac_validation import (  # noqa: E402
    IsaacSceneManifest,
    check_feasibility_handoff,
    write_json,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--handoff", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--feasibility-worktree", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-9)
    args = parser.parse_args()

    worktree = args.feasibility_worktree.resolve()
    actual_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=worktree, check=True,
        text=True, capture_output=True, timeout=10,
    ).stdout.strip()
    manifest = IsaacSceneManifest.from_dict(json.loads(args.manifest.read_text(encoding="utf-8")))
    handoff = json.loads(args.handoff.read_text(encoding="utf-8"))
    contract_path = worktree / "docs/validation/evidence/m710id70_layout_v1/isaac_layout_contract.json"
    if not contract_path.is_file():
        raise FileNotFoundError(f"feasibility checkout has no stable layout contract: {contract_path}")
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    report = check_feasibility_handoff(handoff, manifest, contract, tolerance=args.tolerance)
    report["actual_feasibility_reference_commit"] = actual_commit
    report["recorded_feasibility_reference_commit"] = manifest.source["feasibility_reference_commit"]
    if actual_commit != manifest.source["feasibility_reference_commit"]:
        report["status"] = "FAIL"
        report["differences"].append({
            "field": "feasibility_reference_commit",
            "actual": actual_commit,
            "expected": manifest.source["feasibility_reference_commit"],
        })
    output = args.output or args.handoff.with_name("feasibility_compatibility_report.json")
    write_json(output, report)
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())

