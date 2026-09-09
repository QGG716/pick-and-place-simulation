"""Export one content-addressed Isaac initialization contract from a snapshot."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.isaac_layout_replay import write_isaac_layout_contract


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--snapshot",
        type=Path,
        default=ROOT / "outputs/m710id70_layout_v1/scene_snapshot.json",
    )
    parser.add_argument("--project-root", type=Path, default=ROOT)
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "outputs/m710id70_layout_v1/isaac/contract.json",
    )
    parser.add_argument("--target", default="carton_l07_c02")
    args = parser.parse_args()
    contract = write_isaac_layout_contract(
        args.snapshot,
        args.project_root,
        args.output,
        target=args.target,
    )
    print(
        json.dumps(
            {
                "status": "PASS",
                "scope": contract["scope"],
                "target": contract["target"],
                "carton_count": contract["claims"]["carton_count"],
                "contract_fingerprint": contract["contract_fingerprint"],
                "output": str(args.output.resolve()),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
