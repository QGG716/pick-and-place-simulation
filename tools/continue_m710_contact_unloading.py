"""Plan one next action from a retained Isaac world and publish its request.

Run with the isolated server CPU interpreter.  The existing planner explores
the actual remaining highest row; this entry never edits or reconstructs the
physical world and never replaces an existing continuation request.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def publish_request(path: Path, request: dict) -> None:
    """Publish complete JSON atomically, with no-overwrite semantics."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".next-plan-", suffix=".json", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(request, stream, indent=2)
            stream.flush()
            os.fsync(stream.fileno())
        # A link appears atomically only after the JSON is complete.  Unlike
        # replace(), link() refuses a concurrent or previously submitted plan.
        os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ready-file", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--config", default="configs/validation/m710id70_layout_v1_single_carton.yaml")
    parser.add_argument("--execution-config", type=Path)
    args = parser.parse_args(argv)
    ready_bytes = args.ready_file.read_bytes()
    ready = json.loads(ready_bytes)
    if ready.get("status") != "WORLD_RETAINED_AWAITING_OFFLINE_NEXT_PLAN":
        raise ValueError("Isaac is not retaining a world for this next action")
    request_path = Path(ready["request_path"]).resolve()
    next_index = int(ready["completed_segments"]) + 1
    if (request_path.parent != args.ready_file.resolve().parent
            or request_path.name != f"segment_{next_index:03d}_request.json"):
        raise ValueError("ready record has an inconsistent request destination")
    if request_path.exists():
        raise FileExistsError("a continuation request was already submitted")
    actual_path = Path(ready["actual_state_path"]).resolve()
    actual_bytes = actual_path.read_bytes()
    args.output.mkdir(parents=True, exist_ok=False)
    bundle = (args.output / "replay_bundle.json").resolve()
    command = [sys.executable, str(ROOT / "tools/run_m710_contact_unloading.py"),
               "--actual-state", str(actual_path), "--config", args.config,
               "--output", str(args.output.resolve()), "--execution-bundle", str(bundle)]
    if args.execution_config is not None:
        command += ["--execution-config", str(args.execution_config)]
    process = subprocess.run(command, cwd=ROOT, check=False)
    if args.ready_file.read_bytes() != ready_bytes or actual_path.read_bytes() != actual_bytes:
        raise RuntimeError("retained world state changed or its wait budget ended during planning")
    request = {"world_session_id": ready["world_session_id"],
               "actual_state_sha256": hashlib.sha256(actual_bytes).hexdigest(),
               "planning_output": str(args.output.resolve())}
    if process.returncode:
        request.update(stop=True, reason={2: "ROW_BLOCKED", 3: "PREFLIGHT_BLOCKED"}.get(
            process.returncode, "OFFLINE_PLANNING_FAILED"), planner_returncode=process.returncode)
    else:
        if not bundle.is_file():
            raise FileNotFoundError("successful planner did not produce an execution bundle")
        request["bundle_path"] = str(bundle)
    publish_request(request_path, request)
    print(json.dumps({"request_path": str(request_path), **request}), flush=True)
    return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
