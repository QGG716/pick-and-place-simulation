"""Freshly check a saved complete task, then invoke the existing Isaac executor once.

Run with the CPU validation Python; Isaac stays in its separate optional process.
An existing output directory is never reused, including after a physical failure.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require_same_reference(saved, current):
    """Source bindings may change; the executed time/space reference may not."""
    for key in ("timestamps_seconds", "positions_rad"):
        if saved[key] != current[key]:
            raise ValueError(f"regeneration changed controller {key}")
    for key in ("joint_names", "joint_reference", "target", "initial_actual_state_context"):
        if saved["metadata"][key] != current["metadata"][key]:
            raise ValueError(f"regeneration changed bound {key}")


def absolute_input_path(name, value):
    # A venv interpreter symlink must retain the venv path.
    return value.absolute() if name == "isaac_python" else value.resolve()


def isaac_command(args, root, bundle):
    return [str(args.isaac_python), str(root / "scripts/isaacsim_fanuc_replay.py"),
            "--bundle", str(bundle), "--project-root", str(root),
            "--usd-directory", str(args.output / "usd"),
            "--reuse-usd-entrypoint", str(args.reuse_usd_entrypoint),
            "--reuse-usd-run-evidence", str(args.reuse_usd_run_evidence),
            "--reuse-usd-source-contract", str(args.reuse_usd_source_contract),
            "--output", str(args.output / "isaac"),
            "--record-video", "--video-preview-speed", "1", "--maximum-segments", "1"]


def observed_world_id(output):
    # A Python startup timestamp is not evidence that Isaac created a world.
    for name in ("initialized_actual_state.json", "actual_remaining_state.json"):
        path = output / name
        if path.is_file():
            return json.loads(path.read_text())["world_session_id"]
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("motion", "state", "saved-bundle", "execution-config", "output", "isaac-python",
                 "reuse-usd-entrypoint", "reuse-usd-run-evidence", "reuse-usd-source-contract"):
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--config", type=Path,
                        default=Path("configs/validation/m710id70_handoff_continuation.yaml"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    for name, value in vars(args).items():
        if isinstance(value, Path):
            setattr(args, name, absolute_input_path(name, value))
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = {"schema": "saved_curobo_single_isaac_trial_v1", "trial_id": str(uuid.uuid4()),
                "created_unix_s": time.time(), "planning_invoked": False,
                "isaac_process_launches": 0, "status": "CHECKING",
                "input_files": {}, "commands": []}
    def save():
        (args.output / "trial.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    def run(command, log):
        manifest["commands"].append(command)
        save()
        with (args.output / log).open("wb") as stream:
            return subprocess.run(command, cwd=root, env=env, stdout=stream,
                                  stderr=subprocess.STDOUT, check=False).returncode
    env = dict(os.environ, PYTHONPATH=str(root / "src"))
    try:
        for name in ("motion", "state", "saved_bundle", "config", "execution_config"):
            path = getattr(args, name)
            manifest["input_files"][name] = {"path": str(path), "sha256": file_sha(path)}
        manifest["source_sha256"] = {str(p.relative_to(root)): file_sha(p)
            for directory in ("src/unloading_sim", "scripts", "tools")
            for p in sorted((root / directory).rglob("*.py"))}
        checked = args.output / "execution"
        code = run([sys.executable, str(root / "tools/validate_saved_curobo_execution.py"),
                    "--motion", str(args.motion), "--state", str(args.state),
                    "--config", str(args.config), "--execution-config", str(args.execution_config),
                    "--output", str(checked)], "validation.log")
        manifest["validation_exit_code"] = code
        if code:
            manifest["status"] = "PRECONDITION_REJECTED"
            return code
        current = json.loads((checked / "replay_bundle.json").read_text())
        require_same_reference(json.loads(args.saved_bundle.read_text()), current)
        manifest.update(reference_unchanged=True,
                        reference_duration_s=current["timestamps_seconds"][-1],
                        actual_bundle_sha256=file_sha(checked / "replay_bundle.json"),
                        actual_bundle_payload_sha256=current["bundle_payload_sha256"],
                        status="ISAAC_PROCESS_STARTING")
        manifest["isaac_process_launches"] = 1
        code = run(isaac_command(args, root, checked / "replay_bundle.json"), "isaac.log")
        manifest.update(isaac_exit_code=code, status="ISAAC_PROCESS_EXITED_NOT_A_SUCCESS_VERDICT")
        status_path = args.output / "isaac/run_status.json"
        if status_path.is_file():
            status = json.loads(status_path.read_text())
            manifest["runtime_status"] = status
            manifest["runtime_start_id"] = str(status["started_unix_s"])
            manifest["world_id"] = observed_world_id(args.output / "isaac")
        return code
    except Exception as exc:
        manifest.update(status="LAUNCH_FLOW_FAILED", failure=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        manifest["finished_unix_s"] = time.time()
        save()


if __name__ == "__main__":
    raise SystemExit(main())
