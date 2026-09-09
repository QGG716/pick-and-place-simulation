"""Create a non-overwriting lightweight evidence archive for layout v1."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import platform
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))

from tools.render_m710id70_layout import render
from tools.validate_m710id70_layout import run_layout_validation
from unloading_sim.isaac_layout_replay import write_isaac_layout_contract
from unloading_sim.workcell_layout import sha256_file


IMPLEMENTATION_PATHS = (
    "configs/workcells/m710id70_unloading_layout_v1.yaml",
    "configs/validation/m710id70_layout_v1.yaml",
    "src/unloading_sim/workcell_layout.py",
    "src/unloading_sim/isaac_layout_replay.py",
    "tools/validate_m710id70_layout.py",
    "tools/render_m710id70_layout.py",
    "tools/export_m710id70_isaac_layout.py",
    "scripts/isaacsim_m710_layout_replay.py",
    "tests/test_workcell_layout.py",
    "tests/test_isaac_layout_replay.py",
)


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, capture_output=True, text=True
    ).stdout.strip()


def _manifest_files(directory: Path) -> list[dict[str, object]]:
    return [
        {
            "path": path.relative_to(directory).as_posix(),
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        for path in sorted(directory.rglob("*"))
        if path.is_file() and path.name != "evidence_manifest.json"
    ]


def archive(destination: Path) -> dict[str, object]:
    destination = destination.resolve()
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite evidence archive: {destination}")
    commit = _git("rev-parse", "HEAD")
    branch = _git("rev-parse", "--abbrev-ref", "HEAD")
    dirty = _git("status", "--short").splitlines()
    destination.mkdir(parents=True)
    config = ROOT / "configs/validation/m710id70_layout_v1.yaml"
    layout = ROOT / "configs/workcells/m710id70_unloading_layout_v1.yaml"
    validation = run_layout_validation(config, destination / "validation")
    rendering = render(
        destination / "validation/scene_snapshot.json",
        config,
        destination / "figures",
    )
    contract = write_isaac_layout_contract(
        destination / "validation/scene_snapshot.json",
        ROOT,
        destination / "isaac_layout_contract.json",
    )
    shutil.copy2(layout, destination / "source_layout_config.yaml")
    shutil.copy2(config, destination / "source_validation_config.yaml")
    manifest: dict[str, object] = {
        "schema": "m710id70_layout_v1_evidence_manifest",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "branch": branch,
        "code_identity": {
            "commit": commit,
            "dirty": bool(dirty),
            "dirty_paths": dirty,
            "starting_commit_before_layout_work": "1bb80f5c8400fbcb5f627882c79eb280fe0bd6c2",
        },
        "scope": "confirmed_layout_static_initialization_and_snapshot_replay",
        "population": {
            "cartons": 40,
            "selected_isaac_focus_carton": contract["target"],
            "legacy_v3_104_tasks": "NOT_RERUN_NOT_REINTERPRETED",
            "legacy_continuous_129": "NOT_RERUN_NOT_REINTERPRETED",
        },
        "identity": {
            "layout_id": validation["layout_id"],
            "layout_fingerprint": validation["layout_fingerprint"],
            "scene_fingerprint": validation["scene_fingerprint"],
            "isaac_contract_fingerprint": contract["contract_fingerprint"],
        },
        "claims": {
            "numeric_layout_audit": validation["status"],
            "initial_state_audit": validation["initial_state_status"],
            "physical_grasp": "NOT_EVALUATED",
            "payload_dynamics": "NOT_EVALUATED",
            "complete_workcell_clearance": validation["complete_workcell_clearance"],
            "receiver_transport": validation["receiver_transport"],
        },
        "commands": [
            "python tools/validate_m710id70_layout.py --config configs/validation/m710id70_layout_v1.yaml --output-dir <archive>/validation",
            "python tools/render_m710id70_layout.py --snapshot <archive>/validation/scene_snapshot.json --config configs/validation/m710id70_layout_v1.yaml --output-dir <archive>/figures",
            "python tools/export_m710id70_isaac_layout.py --snapshot <archive>/validation/scene_snapshot.json --project-root . --output <archive>/isaac_layout_contract.json",
            "python -m pytest -q",
        ],
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "numpy": importlib.metadata.version("numpy"),
            "pyyaml": importlib.metadata.version("PyYAML"),
            "matplotlib": importlib.metadata.version("matplotlib"),
        },
        "rendering": rendering,
        "implementation_files": [
            {"path": name, "sha256": sha256_file(ROOT / name)}
            for name in IMPLEMENTATION_PATHS
        ],
    }
    manifest["archive_files"] = _manifest_files(destination)
    (destination / "evidence_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--destination",
        type=Path,
        default=ROOT / "docs/validation/evidence/m710id70_layout_v1",
    )
    args = parser.parse_args()
    result = archive(args.destination)
    print(
        json.dumps(
            {
                "status": "PASS",
                "destination": str(args.destination.resolve()),
                "layout_fingerprint": result["identity"]["layout_fingerprint"],
                "archive_file_count": len(result["archive_files"]),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
