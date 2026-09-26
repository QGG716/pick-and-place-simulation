"""Optional subprocess adapter. No native imports enter the core interpreter."""
from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
from time import perf_counter

import numpy as np

from .planning_contract import (FreeMotionRequest, FreeMotionResult, PlanningStatus,
                                fingerprint, validate_request, subdivision_rule)


class NativeWorker:
    """One serial worker owns reusable geometry and creates a new tree per solve."""
    def __init__(self, executable=None):
        self.executable = executable or os.environ.get("UNLOADING_TESSERACT_WORKER")
        self.process = None
        self.lock = threading.Lock()
        self.messages = queue.Queue()
        self.directory = tempfile.TemporaryDirectory(prefix="unloading-tesseract-")
        self.log = open(Path(self.directory.name) / "native.log", "w+", encoding="utf-8")
        self.cold_start_s = None

    def _read(self):
        try:
            for line in self.process.stdout:
                self.messages.put(json.loads(line))
        except Exception as exc:
            self.messages.put(exc)
        finally:
            self.messages.put(RuntimeError("native worker closed its response stream"))

    def _start(self):
        if not self.executable or not Path(self.executable).is_file():
            raise FileNotFoundError("UNLOADING_TESSERACT_WORKER must name the built native executable")
        start = perf_counter()
        self.process = subprocess.Popen([str(self.executable)], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=self.log, text=True, encoding="utf-8", bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()
        try:
            response = self.messages.get(timeout=60)
        except queue.Empty:
            self.close()
            raise RuntimeError("native worker did not complete dependency startup in 60 seconds")
        if isinstance(response, Exception):
            raise response
        if response != {"ready": True, "protocol": 1}:
            raise RuntimeError(f"unexpected worker handshake: {response}")
        self.cold_start_s = perf_counter() - start

    def call(self, message, cancelled=lambda: False):
        with self.lock:
            cold = self.process is None
            if cold:
                self._start()
            marker = Path(self.directory.name) / "cancel"
            marker.unlink(missing_ok=True)
            if cancelled():
                marker.touch()
            start = perf_counter()
            self.process.stdin.write(json.dumps({**message, "cancel_file": str(marker)}, allow_nan=False) + "\n")
            self.process.stdin.flush()
            while True:
                if cancelled():
                    marker.touch(exist_ok=True)
                try:
                    response = self.messages.get(timeout=.05)
                    break
                except queue.Empty:
                    if self.process.poll() is not None:
                        raise RuntimeError(f"native worker exited {self.process.returncode}")
            if isinstance(response, Exception):
                raise response
            response.setdefault("timings", {})["worker_roundtrip_s_inclusive"] = perf_counter() - start
            response["timings"]["worker_cold_start_s"] = self.cold_start_s if cold else 0.
            return response

    def close(self):
        if self.process is not None:
            if self.process.poll() is None:
                self.process.stdin.close()
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.process.terminate()
                    self.process.wait(timeout=3)
            self.process = None
        self.log.close()
        self.directory.cleanup()


class TesseractOMPLBackend:
    name = "tesseract_ompl"

    def __init__(self, executable=None, *, worker=None):
        self.worker = worker or NativeWorker(executable)

    def plan(self, request: FreeMotionRequest, scene, authority):
        started = perf_counter()
        result = FreeMotionResult(PlanningStatus.INTERNAL_ERROR, self.name)
        attempts = []
        remaining = request.budget.max_state_checks
        refinement = 0
        seen = set()
        result.diagnostics.update(request_id=request.request_id, seed=request.seed,
            stage=request.stage,
            scene_revision=request.scene_revision, scene_fingerprint=request.scene_fingerprint,
            model_fingerprint=request.model_fingerprint, tool_fingerprint=request.tool_fingerprint,
            policy_fingerprint=request.policy_fingerprint, attempts=attempts)
        try:
            # Own a JSON snapshot rather than lending a mutable caller dictionary
            # to a worker while another thread updates it.
            invalid = validate_request(request, scene["joint_limits"])
            if invalid is not None:
                result.status = invalid
                return result
            scene = json.loads(json.dumps(scene, allow_nan=False))
            unsigned = {k: v for k, v in scene.items() if k != "fingerprint"}
            if fingerprint(unsigned) != scene["fingerprint"] or request.scene_fingerprint != scene["fingerprint"]:
                result.status = PlanningStatus.STALE_SCENE
                return result
            if (tuple(scene["joint_names"]) != request.joint_names or request.constraints != scene["constraints"]
                    or request.frames != scene["frames"] or request.attachment != scene["attachment"]
                    or any(getattr(request, key) != scene[key] for key in
                           ("model_fingerprint", "tool_fingerprint", "policy_fingerprint", "stage"))):
                result.status = PlanningStatus.UNSUPPORTED_CONSTRAINT
                return result
            for path, digest in scene["mesh_files"].items():
                import hashlib
                if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
                    result.status = PlanningStatus.STALE_SCENE
                    result.diagnostics["changed_asset"] = path
                    return result
            result.timings["input_validation_s"] = perf_counter() - started
            for index in range(request.budget.max_attempts):
                if request.cancelled():
                    result.status = PlanningStatus.CANCELLED
                    break
                if request.current_revision and request.current_revision() != request.scene_revision:
                    result.status = PlanningStatus.STALE_SCENE
                    break
                wall = None if request.budget.wall_time_s is None else request.budget.wall_time_s - (perf_counter()-started)
                if remaining <= 0 or (wall is not None and wall <= 0):
                    result.status = PlanningStatus.BUDGET_EXHAUSTED
                    break
                message = dict(scene=scene, q_start=request.q_start, q_goal=request.q_goal,
                    seed=request.seed, max_state_checks=remaining, wall_time_s=wall or 0.,
                    refinement=refinement, range_rad=.18,
                    l1_resolution_rad=subdivision_rule(scene["constraints"], refinement)["l1_resolution_rad"],
                    profile=getattr(self, "profile", False))
                raw = self.worker.call(message, request.cancelled)
                attempts.append(raw)
                consumed = int(raw.get("counters", {}).get("state_checks", 0))
                remaining -= consumed
                result.counters = {"state_checks": request.budget.max_state_checks-remaining,
                    "edge_checks": sum(a.get("counters", {}).get("edge_checks", 0) for a in attempts),
                    "subdivision_samples": sum(a.get("counters", {}).get("subdivision_samples", 0) for a in attempts)}
                for key, value in raw.get("timings", {}).items():
                    result.timings[key] = result.timings.get(key, 0.) + value
                result.candidate_found = bool(raw.get("candidate_found"))
                result.exact_solution = bool(raw.get("exact_solution"))
                result.native_validated = bool(raw.get("native_validated"))
                result.diagnostics["backend_versions"] = raw.get("versions")
                if raw.get("status") != "CANDIDATE":
                    result.status = PlanningStatus(raw.get("status", "INTERNAL_ERROR"))
                    break
                # The worker may finish after cancellation, revision change or
                # deadline expiry. Do not start the expensive authority pass.
                if request.cancelled():
                    result.status = PlanningStatus.CANCELLED
                    break
                if request.current_revision and request.current_revision() != request.scene_revision:
                    result.status = PlanningStatus.STALE_SCENE
                    break
                if request.budget.wall_time_s is not None and perf_counter()-started >= request.budget.wall_time_s:
                    result.status = PlanningStatus.BUDGET_EXHAUSTED
                    break
                converted = perf_counter()
                path = np.asarray(raw.get("path"), float)
                if (not result.exact_solution or not result.native_validated or path.ndim != 2
                        or path.shape[1] != len(request.joint_names) or len(path) < 2 or not np.isfinite(path).all()
                        or not np.allclose(path[0], request.q_start, atol=1e-9, rtol=0)
                        or not np.allclose(path[-1], request.q_goal, atol=1e-9, rtol=0)
                        or raw.get("scene_fingerprint") != request.scene_fingerprint):
                    raise ValueError("invalid native candidate contract")
                signature = fingerprint(path.tolist())
                result.timings["result_conversion_s"] = result.timings.get("result_conversion_s", 0.) + perf_counter()-converted
                raw["repeated_candidate"] = signature in seen
                seen.add(signature)
                check = perf_counter()
                rejection = authority(path)
                result.timings["authority_s"] = result.timings.get("authority_s", 0.) + perf_counter()-check
                if request.cancelled():
                    result.status = PlanningStatus.CANCELLED
                    break
                if request.current_revision and request.current_revision() != request.scene_revision:
                    result.status = PlanningStatus.STALE_SCENE
                    break
                if request.budget.wall_time_s is not None and perf_counter()-started >= request.budget.wall_time_s:
                    result.status = PlanningStatus.BUDGET_EXHAUSTED
                    break
                if rejection is None:
                    result.status = PlanningStatus.VERIFIED
                    result.authority_validated = True
                    result.path = path.tolist()
                    break
                result.status = PlanningStatus.AUTHORITY_REJECTED
                raw["authority_rejection"] = dict(rejection)
                q = rejection.get("q_rad")
                if q is None or remaining < 2:
                    raw["repair"] = "MODEL_OR_POLICY_MISMATCH_NO_NATIVE_POINT_WITNESS"
                    break
                probe = self.worker.call({**message, "q_start": q, "q_goal": q,
                                          "max_state_checks": remaining}, request.cancelled)
                remaining -= int(probe.get("counters", {}).get("state_checks", 0))
                raw["rejected_state_native_probe"] = probe
                if probe.get("status") in {"CANCELLED", "BUDGET_EXHAUSTED"}:
                    result.status = PlanningStatus(probe["status"])
                    break
                # Only a point rejected by both validators can demonstrate an
                # interpolation-grid miss. A native-valid rejected point is a
                # model/policy mismatch: stop, never gamble with a different seed.
                if probe.get("status") != "INVALID_START":
                    raw["repair"] = "MODEL_OR_POLICY_MISMATCH_STOP"
                    break
                raw["repair"] = "REFINE_GRID_REBUILD_PLANNER"
                refinement += 1
            result.diagnostics["authority_rejections"] = sum("authority_rejection" in a for a in attempts)
            result.diagnostics["termination"] = result.status.value
            return result
        except (OSError, RuntimeError, ValueError) as exc:
            result.status = PlanningStatus.BACKEND_UNAVAILABLE if not attempts else PlanningStatus.INTERNAL_ERROR
            result.diagnostics["error"] = str(exc)
            return result
        finally:
            if attempts:
                calls = [call for attempt in attempts for call in (
                    [attempt, attempt["rejected_state_native_probe"]]
                    if "rejected_state_native_probe" in attempt else [attempt])]
                keys = {key for call in calls for key in call.get("counters", {})}
                result.counters = {key: sum(call["counters"][key] for call in calls)
                                   if all(call.get("counters", {}).get(key) is not None for call in calls) else None
                                   for key in keys}
                for key in {key for call in calls for key in call.get("timings", {})}:
                    result.timings[key] = sum(call.get("timings", {}).get(key, 0.) for call in calls)
                result.diagnostics["termination"] = result.status.value
            result.timings["request_total_s"] = perf_counter() - started


class LegacyBackend:
    name = "legacy"

    def plan(self, request, scene, authority):
        from .planner import RRTConnectPlanner
        start = perf_counter()
        result = FreeMotionResult(PlanningStatus.INTERNAL_ERROR, self.name)
        invalid = validate_request(request, scene["joint_limits"])
        if invalid:
            result.status = invalid
            return result
        if (fingerprint({key: value for key, value in scene.items() if key != "fingerprint"}) != scene["fingerprint"]
                or request.scene_fingerprint != scene["fingerprint"]):
            result.status = PlanningStatus.STALE_SCENE
            return result
        if (tuple(scene["joint_names"]) != request.joint_names or request.constraints != scene["constraints"]
                or request.frames != scene["frames"] or request.attachment != scene["attachment"]
                or any(getattr(request, key) != scene[key] for key in
                       ("model_fingerprint", "tool_fingerprint", "policy_fingerprint", "stage"))):
            result.status = PlanningStatus.UNSUPPORTED_CONSTRAINT
            return result
        limits = np.asarray(scene["joint_limits"])
        planner = RRTConnectPlanner(limits[:, 0], limits[:, 1],
            lambda q: not request.cancelled() and authority([q]) is None,
            max_iterations=request.budget.legacy_iterations,
            edge_resolution=.5*scene["constraints"]["edge_resolution_rad"], rng=np.random.default_rng(request.seed))
        searched = perf_counter()
        candidate = planner.plan(np.array(request.q_start), np.array(request.q_goal), request.budget.wall_time_s)
        result.timings["legacy_search_s_inclusive"] = perf_counter()-searched
        result.candidate_found = candidate.success
        result.exact_solution = candidate.success
        result.diagnostics = dict(candidate.search_evidence)
        result.counters = {key: candidate.search_evidence.get(key) for key in
            ("state_validations", "edge_validation_calls", "edge_state_samples", "extension_attempts")}
        result.counters["legacy_iterations"] = candidate.iterations
        result.diagnostics["scene_fingerprint"] = request.scene_fingerprint
        result.diagnostics["seed"] = request.seed
        if request.cancelled():
            result.status = PlanningStatus.CANCELLED
        elif request.current_revision and request.current_revision() != request.scene_revision:
            result.status = PlanningStatus.STALE_SCENE
        elif candidate.success:
            checked = perf_counter(); rejection = authority(candidate.path)
            result.timings["authority_s"] = perf_counter()-checked
            result.authority_validated = rejection is None
            result.status = PlanningStatus.VERIFIED if rejection is None else PlanningStatus.AUTHORITY_REJECTED
            if request.cancelled():
                result.status = PlanningStatus.CANCELLED
                result.authority_validated = False
            elif request.current_revision and request.current_revision() != request.scene_revision:
                result.status = PlanningStatus.STALE_SCENE
                result.authority_validated = False
            elif request.budget.wall_time_s is not None and perf_counter()-start >= request.budget.wall_time_s:
                result.status = PlanningStatus.BUDGET_EXHAUSTED
                result.authority_validated = False
            elif rejection is None:
                result.path = [q.tolist() for q in candidate.path]
            else:
                result.diagnostics["authority_rejection"] = rejection
        else:
            result.status = {"start state is invalid": PlanningStatus.INVALID_START,
                             "goal state is invalid": PlanningStatus.INVALID_GOAL}.get(candidate.message, PlanningStatus.BUDGET_EXHAUSTED)
        result.timings["request_total_s"] = perf_counter()-start
        return result


def create_free_motion_backend(name, **kwargs):
    if name == "legacy":
        return LegacyBackend()
    if name == "tesseract_ompl":
        return TesseractOMPLBackend(**kwargs)
    raise ValueError(f"unknown free motion backend: {name}")


def connect_free_motion(connector, backend, start, goal, obstacles, *, seed, iteration_budget,
                        attachment=None, support_names=(), target_contact=None, stage):
    from .planning_contract import PlanningBudget
    from .tesseract_scene import export_scene
    began = perf_counter()
    try:
        scene = export_scene(connector, obstacles, stage=stage, attachment=attachment,
                             support_names=support_names, target_contact=target_contact)
    except (ValueError, OSError) as exc:
        reason = "UNSUPPORTED_CONSTRAINT" if isinstance(exc, ValueError) else "BACKEND_UNAVAILABLE"
        return [], dict(reason=reason, stage=stage, detail=str(exc)), dict(
            backend=backend.name, success=False, search_success=False, planning_iterations_consumed=0)
    conversion = perf_counter()-began
    def context():
        identity = connector._context_identity(obstacles, attachment=attachment,
            support_names=support_names, target_contact=target_contact, stage=stage)
        return fingerprint([identity, None if attachment is None else [attachment.rigid.name,
                            np.asarray(attachment.rigid.half_extents).tolist()]])
    revision = context()
    request = FreeMotionRequest(request_id=f"{stage}:{seed}:{revision[:12]}", scene_revision=revision,
        scene_fingerprint=scene["fingerprint"], model_fingerprint=scene["model_fingerprint"],
        tool_fingerprint=scene["tool_fingerprint"], policy_fingerprint=scene["policy_fingerprint"],
        stage=stage, joint_names=tuple(scene["joint_names"]), q_start=tuple(start), q_goal=tuple(goal),
        constraints=scene["constraints"], frames=scene["frames"], attachment=scene["attachment"], seed=int(seed),
        budget=PlanningBudget(legacy_iterations=int(iteration_budget),
            max_state_checks=getattr(backend, "max_state_checks", 100000),
            wall_time_s=connector._limit(connector._remaining_wall_time(), connector.budget.stage_wall_time_s)),
        cancelled=getattr(connector, "planning_cancelled", lambda: False), current_revision=context)
    authority = lambda path: connector._path_failure(path, obstacles, attachment=attachment,
        support_names=support_names, target_contact=target_contact, stage=stage,
        diagnostic_origin="tesseract_candidate_authority")
    progress = getattr(connector, "progress_callback", None)
    if progress is not None:
        progress(dict(event="connection_started", stage=stage, backend=backend.name,
                      seed=int(seed), max_state_checks=request.budget.max_state_checks))
    result = backend.plan(request, scene, authority)
    if progress is not None:
        progress(dict(event="connection_finished", stage=stage, backend=backend.name,
                      success=result.deliverable, termination=result.status.value,
                      counters=result.counters, timings=result.timings))
    result.timings["scene_export_s"] = conversion
    evidence = result.to_mapping()
    evidence.update(stage=stage, seed=int(seed), success=result.deliverable,
        search_success=result.candidate_found, validation_level="B_STRICT_LOCAL_CONNECTION" if result.deliverable else "A_UNVERIFIED_GEOMETRY",
        # Debit the caller's allocated attempt slice. This is scheduling accounting,
        # explicitly NOT a measured OMPL iteration count or work-equivalence claim.
        planning_iterations_consumed=int(iteration_budget),
        outer_budget_accounting="ALLOCATED_LEGACY_SLICE_NOT_MEASURED_OMPL_ITERATIONS")
    connector._statistics["connection_attempts"] += 1
    connector._statistics["path_connection_wall_seconds_inclusive"] += perf_counter()-began
    records = getattr(connector, "free_motion_records", None)
    if records is None:
        connector.free_motion_records = records = []
    records.append(evidence)
    if not result.deliverable:
        return [], dict(reason=result.status.value, stage=stage, detail=result.diagnostics), evidence
    connector._remember_path(result.path, revision, stage, "B_STRICT_LOCAL_CONNECTION")
    return [np.asarray(q) for q in result.path], None, evidence
