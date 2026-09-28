"""Lightweight environment/configure check. No plan, state audit, IK or Isaac."""
from pathlib import Path
import hashlib
import os
import sys
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from tools.run_tesseract_ompl_task import parse_args, planner_configuration, RecordingWorker, write_json
from tools.run_tesseract_ompl_comparison import fixture_context
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.layout_single_carton import load_layout_motion_policy
from unloading_sim.m710_execution import load_m710_execution_config
from unloading_sim.m710_dynamics import load_m710id70_dynamics
from unloading_sim.planning_profile import profile_evidence
import json


def main(argv=None):
    args = parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=False)
    result = dict(status="FAIL", operation="configure", task_plan_calls=0, engineering_search_calls=0,
                  path_success=False, isaac_calls=0)
    began = perf_counter()
    worker = None
    try:
        import pinocchio
        import coal
        result["environment"] = dict(python=sys.executable, python_version=sys.version,
            pinocchio_version=pinocchio.__version__, pinocchio_module=pinocchio.__file__,
            coal_module=coal.__file__, LD_LIBRARY_PATH=os.environ.get("LD_LIBRARY_PATH"),
            worker=str(Path(args.worker).resolve()), worker_environment="INHERITED_CPU_ENVIRONMENT_WITH_EXISTING_WORKER_RPATH",
            worker_sha256=hashlib.sha256(Path(args.worker).read_bytes()).hexdigest())
        config = planner_configuration(args)
        scene, _, connector, _ = fixture_context(args.state, args.segment)
        if args.execution_config is None:
            raise ValueError("startup requires an explicit matching execution configuration")
        execution = load_m710_execution_config(args.execution_config)
        policy = load_layout_motion_policy(execution.motion_policy_path)
        if policy.policy_fingerprint != scene.policy.policy_fingerprint:
            raise ValueError("execution motion policy differs from actual frozen scene policy")
        if execution.layout_validation_path != scene.policy.layout_validation.config_path:
            raise ValueError("execution layout/model source differs from actual frozen scene")
        dynamics = load_m710id70_dynamics(execution.dynamics_path)
        result["execution_configuration"] = dict(path=str(execution.config_path), fingerprint=execution.fingerprint,
            policy_fingerprint=policy.policy_fingerprint, simulation_profile=profile_evidence(policy.data),
            tool_mass_kg=dynamics.tool.mass_kg, carton_mass_kg=dynamics.cartons.mass_kg_each,
            dynamics_fingerprint=dynamics.fingerprint, robot_manifest=str(execution.robot_manifest_path),
            tool_manifest=str(execution.tool_manifest_path), scope="INPUT_COMPATIBILITY_ONLY_NOT_EXECUTION_PREFLIGHT")
        expected = json.loads(args.expected_first_request.read_text())
        message = dict(operation="configure", scene=export_scene(connector, scene.all_obstacles, stage="pregrasp"),
            q_start=tuple(expected["q_start"]), q_goal=tuple(expected["q_goal"]), seed=expected["seed"],
            planner_config=config.to_mapping(), max_state_checks=100000, refinement=0,
            l1_resolution_rad=expected["l1_resolution_rad"], wall_time_s=0., profile=args.profile)
        worker = RecordingWorker(args.worker, args.output, args.expected_first_request)
        native = worker.call(message)
        result["native_activity"] = worker.activity()
        if (native.get("status") != "CONFIGURED" or native.get("search_started") is not False
                or native.get("planner_config") != config.to_mapping()
                or native.get("effective_sampler", {}).get("configuration") != config.to_mapping().get("sampling", {"type": "uniform"})
                or native.get("effective_planner", {}).get("type") != "ompl::geometric::LazyPRM"
                or native["effective_planner"].get("star") is not False
                or native["effective_planner"].get("max_nearest_neighbors") != 5):
            raise ValueError("worker configure did not confirm the required LazyPRM/sampler contract")
        result.update(status="PASS", handshake="NativeWorker protocol=1 verified before configure",
                      effective_planner=native["effective_planner"], effective_sampler=native["effective_sampler"])
    except Exception as exc:
        result["failure"] = dict(type=type(exc).__name__, reason=str(exc))
    finally:
        if worker is not None:
            worker.close()
        result["elapsed_s"] = perf_counter()-began
        write_json(args.output / "startup.json", result)
    print(json.dumps(result), flush=True)
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
