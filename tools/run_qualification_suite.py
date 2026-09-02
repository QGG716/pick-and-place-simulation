"""Run load, reachability-screen and cycle studies into one review bundle."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="fanuc_m20id_35")
    parser.add_argument("--tool", default="configs/tools/unloading_gripper.yaml")
    parser.add_argument("--truck", default="configs/trucks/2p3x2p7.yaml")
    parser.add_argument("--cycle", default="configs/cycle/unloading_900pph.yaml")
    parser.add_argument("--payloads", nargs="+", type=float, default=(5.0, 15.0, 25.0))
    parser.add_argument("--output-dir")
    args = parser.parse_args(argv)
    output = Path(args.output_dir).resolve() if args.output_dir else ROOT / "results" / datetime.now().strftime("%Y%m%dT%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    parts = output / "components"
    load_dir, reach_dir, cycle_dir = parts / "load", parts / "reachability", parts / "cycle"
    commands = [
        [sys.executable, str(ROOT / "tools/analyze_payload_envelope.py"), "--robot", args.robot, "--tool", args.tool, "--output-dir", str(load_dir)],
        [sys.executable, str(ROOT / "tools/run_reachability_study.py"), "--robot", args.robot, "--tool", args.tool, "--truck", args.truck, "--payloads", *(str(value) for value in args.payloads), "--output-dir", str(reach_dir)],
        [sys.executable, str(ROOT / "tools/run_cycle_simulation.py"), "--config", args.cycle, "--output-dir", str(cycle_dir)],
    ]
    command_records = []
    for command in commands:
        completed = subprocess.run(command, cwd=ROOT, check=True, text=True, capture_output=True)
        command_records.append({"argv": command, "stdout": completed.stdout.strip(), "stderr": completed.stderr.strip()})

    copies = {
        load_dir / "robot_load_results.csv": output / "robot_load_results.csv",
        load_dir / "load_envelope.png": output / "load_envelope.png",
        load_dir / "moment_envelope.png": output / "moment_envelope.png",
        load_dir / "inertia_envelope.png": output / "inertia_envelope.png",
        load_dir / "tcp_payload_envelope.png": output / "tcp_payload_envelope.png",
        load_dir / "tool_mass_envelope.png": output / "tool_mass_envelope.png",
        reach_dir / "reachability_geometry.png": output / "reachability_geometry.png",
        cycle_dir / "cycle_time.csv": output / "cycle_time.csv",
        cycle_dir / "cycle_time_distribution.png": output / "cycle_time_distribution.png",
        cycle_dir / "throughput_vs_normal_cycle.png": output / "throughput_vs_normal_cycle.png",
    }
    for payload in args.payloads:
        label = f"{int(payload) if payload.is_integer() else payload:g}kg"
        copies[reach_dir / f"reachability_{label}.csv"] = output / f"reachability_{label}.csv"
        copies[reach_dir / f"reachability_{label}.png"] = output / f"reachability_{label}.png"
    for source, destination in copies.items():
        shutil.copy2(source, destination)

    load_summary = json.loads((load_dir / "summary.json").read_text(encoding="utf-8"))
    reach_summary = json.loads((reach_dir / "summary.json").read_text(encoding="utf-8"))
    cycle_summary = json.loads((cycle_dir / "summary.json").read_text(encoding="utf-8"))
    aggregate = {
        "schema_version": "unloading_qualification_suite_v1",
        "load": load_summary,
        "reachability": reach_summary,
        "cycle": cycle_summary,
        "decision": "NOT_QUALIFIED",
        "decision_reason": "25 kg required load cases have known wrist failures and full 15-condition task reachability remains incomplete",
    }
    (output / "summary.json").write_text(json.dumps(aggregate, indent=2, ensure_ascii=False), encoding="utf-8")
    config = {
        "schema_version": "unloading_qualification_suite_v1",
        "arguments": vars(args),
        "python": sys.version,
        "component_commands": command_records,
        "input_sha256": {
            value: _sha256((ROOT / value).resolve())
            for value in (args.tool, args.truck, args.cycle, "configs/robots/fanuc_m20id_35.yaml")
        },
    }
    (output / "config.json").write_text(json.dumps(config, indent=2, ensure_ascii=False), encoding="utf-8")
    report = f"""# Technical qualification report

## 1. 25 kg box plus current tool

**NOT QUALIFIED.** The configured tool is {load_summary['payload_plus_tool_kg'] - 25.0:.3f} kg. All {load_summary['required_25kg_cases']} required 25 kg size/grasp cases fail a known published payload, moment, or inertia check in the Phase 1 envelope. The official payload/CoM curve is also `{load_summary['official_com_curve_status']}`.

## 2. Limiting factors

- Payload/moment/inertia: row-level results and source metadata are in `robot_load_results.csv`; known failures are {load_summary['qualification_counts']}.
- Reach/collision/extraction: the lightweight 50 mm screen is fail-closed and reports zero task-qualified cells. Pose-specific IK and full swept extraction remain required before any reachable claim.
- Cycle time: under the configured engineering event assumptions, {cycle_summary['boxes_per_hour']:.1f} boxes/h at a {cycle_summary['normal_cycle_seconds']:.3f} s normal cycle.

## 3. Maximum recommended work envelope

No vendor-qualified maximum envelope is issued. The published-limit engineering regions are plotted, but the FANUC payload/CoM curve and pose-specific extraction evidence are incomplete.

## 4–7. Z axis, mount height/offset and conveyor release position

`NOT_EVALUATED` by this suite. The repository contains a heavier PyBullet cross-section study, but its result must be rerun and integrated with this load source before making a production recommendation.

## 8. NORMAL cycle required for 900 boxes/h

The configured Monte Carlo model requires at most **{cycle_summary['required_normal_cycle_seconds_for_target']:.3f} s**. Its probabilities and durations are `ENGINEERING_ASSUMPTION` values pending production-log calibration.

## Evidence status

The bundle is deterministic for the recorded inputs and seeds. `config.json` records commands and SHA-256 input hashes. Unknown evidence remains `NOT_EVALUATED`; no collision tolerance, mass, inertia, gravity, or manufacturer limit was relaxed to create a pass.
"""
    (output / "technical_qualification_report.md").write_text(report, encoding="utf-8")
    manifest = {path.name: _sha256(path) for path in output.iterdir() if path.is_file()}
    (output / "manifest_sha256.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(output)
    print(json.dumps({"decision": aggregate["decision"], "required_normal_cycle_seconds": cycle_summary["required_normal_cycle_seconds_for_target"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
