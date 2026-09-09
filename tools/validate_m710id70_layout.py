"""Validate and freeze the confirmed M-710iD/70 unloading layout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.workcell_layout import (
    audit_initial_state,
    audit_layout_constraints,
    audit_snapshot_consistency,
    find_initial_state_witness,
    load_layout_validation_config,
    verify_scene_snapshot,
    write_scene_snapshot,
)


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def run_layout_validation(config_path: Path, output_dir: Path) -> dict:
    config = load_layout_validation_config(config_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    numeric = audit_layout_constraints(config.layout, tolerance=config.contact_tolerance_m)
    previous = audit_initial_state(config, np.asarray(config.data["initial_state"]["previous_v3_q_rad"], dtype=float))
    witness = find_initial_state_witness(
        config, int(config.data["initial_state"]["search_maximum_random_draws"])
    )
    initial = audit_initial_state(config)
    _write(output_dir / "numeric_audit.json", numeric)
    _write(output_dir / "previous_v3_initial_state_audit.json", previous)
    _write(output_dir / "initial_state_search.json", witness)
    _write(output_dir / "initial_state_audit.json", initial)
    if numeric["overall_status"] != "PASS" or initial["status"] != "PASS":
        raise RuntimeError("layout numeric or initial-state audit failed")
    if witness["status"] != "PASS" or not np.allclose(
        witness["q_rad"], config.initial_q, atol=1e-12, rtol=0
    ):
        raise RuntimeError("configured initial state does not match deterministic witness search")
    snapshot = write_scene_snapshot(config, output_dir / "scene_snapshot.json")
    verification = verify_scene_snapshot(snapshot, ROOT)
    consistency = audit_snapshot_consistency(config, snapshot)
    _write(output_dir / "snapshot_asset_verification.json", verification)
    _write(output_dir / "snapshot_replay_consistency.json", consistency)
    result = {
        "status": "PASS",
        "layout_id": config.layout.data["layout_id"],
        "layout_fingerprint": config.layout.layout_fingerprint,
        "scene_fingerprint": snapshot["scene_fingerprint"],
        "carton_count": len(snapshot["cartons"]),
        "initial_state_status": initial["status"],
        "complete_workcell_clearance": initial["complete_workcell_clearance"],
        "receiver_transport": snapshot["receiver"]["transport_capability"],
        "output_dir": str(output_dir.resolve()),
    }
    _write(output_dir / "validation_summary.json", result)
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs/validation/m710id70_layout_v1.yaml",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "outputs/m710id70_layout_v1",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = run_layout_validation(args.config, args.output_dir)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
