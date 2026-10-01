"""One new retained world, one cold request, and at most one gated execution.

The ordinary planner, exporter and existing supervised Isaac loop retain all
their gates. This driver never retries a failed world or planning request.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.native_cold_entry import native_budget_config, read_native_initial_state


def write(path, value):
    temporary = path.with_suffix(path.suffix + ".pending")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")
    os.replace(temporary, path)


def source_hashes():
    result = {}
    for directory in ("src", "tools", "scripts", "configs", "ros2"):
        for path in sorted((ROOT / directory).rglob("*")):
            if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
                result[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def validate_world_ready_paths(output, ready):
    expected = {"actual_state_path": output / "physics/initialized_actual_state.json",
                "request_path": output / "handshake/initial_request.json"}
    for name, path in expected.items():
        if not isinstance(ready.get(name), str) or Path(ready[name]).resolve() != path.resolve():
            raise ValueError("NATIVE_COLD_READY_PATH_NOT_THIS_RUN:" + name)


def successful_isaac_result(result):
    if not isinstance(result, dict):
        return False
    counts = result.get("execution_counts") or {}
    initial = result.get("native_cold_initialization") or {}
    return (result.get("workflow_cycle_completed") is True
        and result.get("runtime_stop_reason") is None
        and type(counts.get("actual_grasp")) is int and counts["actual_grasp"] == 1
        and type(counts.get("actual_release")) is int and counts["actual_release"] == 1
        and initial.get("same_world_retained_for_planning_and_execution") is True
        and initial.get("maximum_physical_segments") == 1
        and initial.get("initial_completed_segments") == 0
        and initial.get("historical_motion_inputs_read") == 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--isaac-python", type=Path, required=True)
    parser.add_argument("--worker-command", required=True)
    parser.add_argument("--worker-binary", type=Path, required=True)
    parser.add_argument("--native-asset-root", required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--task-budget-s", type=float, default=3600.)
    parser.add_argument("--stage-budget-s", type=float, default=300.)
    parser.add_argument("--ipc-timeout-s", type=float, default=360.)
    args = parser.parse_args()
    if re.fullmatch(r"[0-9a-f]{40}", args.source_commit) is None:
        parser.error("source-commit must be the full committed source SHA")
    native_budget_config(dict(task_wall_time_s=args.task_budget_s,
        stage_wall_time_s=args.stage_budget_s, ipc_timeout_s=args.ipc_timeout_s))
    output = args.output.resolve()
    if not output.is_relative_to(ROOT):
        parser.error("output must be in this isolated repository")
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    sources = source_hashes()
    binary_hash = hashlib.sha256(args.worker_binary.read_bytes()).hexdigest()
    run_id = "native-cold-" + uuid.uuid4().hex
    status = dict(schema="m710_native_cold_single_run_v1", run_id=run_id,
        source_commit=args.source_commit, worker_sha256=binary_hash,
        random_seed=71070, target="carton_l07_c02", carton_count=40,
        planning_requests=0, world_launches=0, execution_requests=0,
        status="PREPARING", real_machine_commands=False, timings={})
    write(output / "source-manifest.json", sources)
    write(output / "run.json", status)
    environment = os.environ.copy()
    environment.update(PYTHONPATH=str(ROOT / "src"), M710_MOVEIT_COMMAND=args.worker_command,
        M710_MOVEIT_ASSET_ROOT=args.native_asset_root, M710_MOVEIT_SEED="71070",
        M710_MOVEIT_LOG=str(output / "native-worker.log"),
        M710_MOVEIT_REQUEST_LOG=str(output / "native-requests.jsonl"),
        M710_AUTHORITY_TRACE=str(output / "authority.jsonl"))
    environment["M710_EXECUTION_SOURCE_COMMIT"] = args.source_commit
    world = None
    ready = None
    world_log = None
    def publish(phase, **fields):
        status.update(status=phase, wall_seconds=time.monotonic()-started, **fields)
        write(output / "run.json", status)
        print(json.dumps({"run_id": run_id, "status": phase, **fields}), flush=True)
    def sources_unchanged():
        if source_hashes() != sources or hashlib.sha256(args.worker_binary.read_bytes()).hexdigest() != binary_hash:
            raise RuntimeError("NATIVE_COLD_SOURCE_OR_BINARY_CHANGED_DURING_RUN")
    def signal_world(request):
        destination = Path(ready["request_path"])
        if destination.exists():
            raise RuntimeError("SINGLE_WORLD_REQUEST_ALREADY_SENT")
        write(destination, {"world_session_id": ready["world_session_id"],
            "actual_state_sha256": ready["actual_state_sha256"], **request})
    try:
        subprocess.run([sys.executable, str(ROOT / "tools/prepare_m710_native_bootstrap.py"),
            "--output", str(output / "bootstrap.json")], cwd=ROOT, env=environment, check=True)
        sources_unchanged()
        command = [str(args.isaac_python), str(ROOT / "scripts/isaacsim_fanuc_replay.py"),
            "--bootstrap-contract", str(output / "bootstrap.json"), "--project-root", str(ROOT),
            "--usd-directory", str(output / "usd"), "--output", str(output / "physics"),
            "--continuation-dir", str(output / "handshake"), "--maximum-segments", "1",
            "--continuation-wait-seconds", str(args.task_budget_s + 900.),
            "--record-video", "--video-preview-speed", "1", "--width", "640", "--height", "360", "--output-fps", "5"]
        write(output / "commands.json", {"bootstrap_world": command})
        world_log = (output / "isaac-driver.log").open("x", encoding="utf-8")
        world = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=world_log, stderr=subprocess.STDOUT)
        status["world_launches"] = 1
        publish("INITIALIZING_ONE_WORLD", world_pid=world.pid)
        ready_path = output / "handshake/initial_ready.json"
        initialization_started = time.monotonic()
        while not ready_path.exists():
            if world.poll() is not None:
                raise RuntimeError("INITIAL_WORLD_EXITED_BEFORE_READY:" + str(world.returncode))
            if time.monotonic()-initialization_started > 1200.:
                raise RuntimeError("INITIALIZATION_WALL_BUDGET_EXHAUSTED")
            time.sleep(1.)
        status["timings"]["initialization_wall_s"] = time.monotonic()-initialization_started
        observed_ready = json.loads(ready_path.read_text(encoding="utf-8"))
        validate_world_ready_paths(output, observed_ready)
        ready = observed_ready
        actual_path = Path(ready["actual_state_path"])
        state, raw = read_native_initial_state(actual_path, ready_path)
        sources_unchanged()
        write(output / "input-manifest.json", dict(run_id=run_id, source_commit=args.source_commit,
            worker_sha256=binary_hash, scene="m710id70_unloading_layout_v1", target=status["target"],
            random_seed=71070, carton_count=40, tool_mass_kg=20., target_mass_kg=42.5,
            actual_state_path=str(actual_path), actual_state_sha256=hashlib.sha256(raw).hexdigest(),
            world_session_id=state["world_session_id"], history_enabled=False,
            allowed_inputs=["official model", "frozen current scene", "this measured initial state",
                "current tool/cup geometry", "current process and receiver/direction policy", "fixed seed"],
            bootstrap_contract_sha256=ready["bootstrap_contract_sha256"]))
        planning_command = [sys.executable, str(ROOT / "tools/run_m710_contact_unloading.py"),
            "--backend", "moveit2", "--native-cold", "--actual-state", str(actual_path),
            "--initial-ready", str(ready_path), "--target-id", status["target"],
            "--native-task-budget-s", str(args.task_budget_s), "--native-stage-budget-s", str(args.stage_budget_s),
            "--native-ipc-timeout-s", str(args.ipc_timeout_s), "--output", str(output / "plan"), "--diagnostics"]
        write(output / "commands.json", {"bootstrap_world": command, "ordinary_native_cold_request": planning_command})
        status["planning_requests"] = 1
        publish("PLANNING_NATIVE_COLD", world_session_id=state["world_session_id"])
        planning_started = time.monotonic()
        with (output / "planning.log").open("x", encoding="utf-8") as stream:
            planning = subprocess.run(planning_command, cwd=ROOT, env=environment, stdout=stream,
                stderr=subprocess.STDOUT, timeout=args.task_budget_s+600.)
        status["timings"]["planning_and_delivery_wall_s"] = time.monotonic()-planning_started
        sources_unchanged()
        if planning.returncode != 0:
            raise RuntimeError("NATIVE_COLD_PLANNING_OR_DELIVERY_GATE_FAILED:" + str(planning.returncode))
        delivery = json.loads((output / "plan/delivery.json").read_text(encoding="utf-8"))
        motion = json.loads((output / "plan/motion.json").read_text(encoding="utf-8"))
        if (motion.get("complete_trajectory_status") != "PASS" or motion.get("native_cold") is not True
                or delivery.get("simulation_execution_ready") is not True
                or delivery.get("bundle_readback", {}).get("status") != "PASS"):
            raise RuntimeError("NATIVE_COLD_REQUIRED_DELIVERY_GATE_NOT_PASSED")
        if actual_path.read_bytes() != raw or world.poll() is not None:
            raise RuntimeError("RETAINED_WORLD_OR_INITIAL_STATE_CHANGED")
        signal_world({"bundle_path": delivery["bundle_path"]})
        status["execution_requests"] = 1
        publish("EXECUTING_ONE_GATED_SEGMENT")
        execution_started = time.monotonic()
        world.wait(timeout=3600.)
        status["timings"]["isaac_execution_wall_s"] = time.monotonic()-execution_started
        result_path = output / "physics/result.json"
        status["isaac_result"] = json.loads(result_path.read_text(encoding="utf-8")) if result_path.exists() else None
        completed = world.returncode == 0 and successful_isaac_result(status["isaac_result"])
        publish("ISAAC_WORKFLOW_COMPLETED_UNDER_DECLARED_ASSUMPTIONS" if completed else "ISAAC_FAILED",
                isaac_exit_code=world.returncode,
                physical_qualification_passed=(status["isaac_result"] or {}).get("qualification_passed"))
        return 0 if completed else 1
    except BaseException as exc:
        publish("BLOCKED", reason=str(exc), exception_type=type(exc).__name__)
        if world is not None and world.poll() is None:
            try:
                if ready is not None and status["execution_requests"] == 0:
                    signal_world({"stop": True, "reason": str(exc)})
                    world.wait(timeout=60.)
            except Exception as cleanup:
                publish("BLOCKED", cleanup_error=str(cleanup))
            finally:
                if world.poll() is None:
                    world.terminate()
                    try:
                        world.wait(timeout=30.)
                    except subprocess.TimeoutExpired:
                        world.kill()
                        world.wait(timeout=30.)
        return 1
    finally:
        if world_log is not None:
            world_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
