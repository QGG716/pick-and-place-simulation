"""Bounded, inert JSON hints. Historical validation never grants permission."""
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter
import hashlib
import json
import os

import numpy as np

from .planning_profile import optional_seconds, deadline_after
from .geometry import rotation_vector_from_matrix
from .workcell_layout import canonical_digest


@dataclass(frozen=True)
class HistoryPolicy:
    source: str | None = None
    register_directory: str | None = None
    maximum_files: int = 16
    maximum_file_bytes: int = 4_000_000
    maximum_candidates: int = 2
    maximum_variants: int = 4
    screening_wall_time_s: float = 2.
    wall_time_s: float = 45.
    request_fraction: float = .5
    candidate_wall_time_s: float = 20.
    normal_step_m: float = .0005
    maximum_normal_offset_m: float = .0015
    maximum_target_translation_m: float = .02
    maximum_target_rotation_rad: float = .03
    maximum_joint_adaptation_rad: float = .15

    def __post_init__(self):
        for name in ("source", "register_directory"):
            if getattr(self, name) is not None and not isinstance(getattr(self, name), str):
                raise ValueError(f"history.{name} must be a path string or null")
        for name, maximum in (("maximum_files", 64), ("maximum_file_bytes", 8_000_000),
                              ("maximum_candidates", 4), ("maximum_variants", 8)):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError(f"history.{name} must be an integer in [1, {maximum}]")
        for name in ("screening_wall_time_s", "wall_time_s", "candidate_wall_time_s"):
            optional_seconds(getattr(self, name), name)
        for name in ("normal_step_m", "maximum_normal_offset_m", "maximum_target_translation_m",
                     "maximum_target_rotation_rad", "maximum_joint_adaptation_rad"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"history.{name} must be finite and positive")
        if not 0 < self.request_fraction <= .5:
            raise ValueError("history must reserve at least half the remaining time for ordinary search")
        if self.maximum_normal_offset_m > .002 or self.normal_step_m > self.maximum_normal_offset_m:
            raise ValueError("history normal offsets must stay within the existing 2 mm geometric contact bound")


def history_policy(value=None):
    return HistoryPolicy(**dict(value or {}))


def _json(data):
    def invalid(value):
        raise ValueError(f"nonfinite JSON number: {value}")
    def unique(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    result = json.loads(data, parse_constant=invalid, object_pairs_hook=unique)
    # Also catches exponent overflow, e.g. 1e999, without accepting JSON Infinity.
    pending = [result]
    while pending:
        item = pending.pop()
        if isinstance(item, dict):
            pending.extend(item.values())
        elif isinstance(item, list):
            pending.extend(item)
        elif isinstance(item, float) and not np.isfinite(item):
            raise ValueError("nonfinite JSON number")
    return result


class HistorySource:
    def __init__(self, policy, *, deadline=None):
        self.policy = policy
        self.documents = []
        self.records = []
        self.targets_screened = {}
        self.deadline = deadline
        started = perf_counter()
        limits = [v for v in (deadline_after(started, policy.screening_wall_time_s), deadline) if v is not None]
        stop = min(limits) if limits else None
        if policy.source is None:
            self.records.append({"status": "NOT_CONFIGURED"})
        else:
            source = Path(policy.source).expanduser()
            try:
                # Nonrecursive; no index, script, pickle, symlink or archive execution.
                if source.is_symlink():
                    raise ValueError("history source must not be a symlink")
                if source.is_dir():
                    names = []
                    with os.scandir(source) as entries:
                        for index, entry in enumerate(entries):
                            if index >= policy.maximum_files * 8:
                                self.records.append({"status": "DIRECTORY_ENTRY_LIMIT"})
                                break
                            if stop is not None and perf_counter() >= stop:
                                break
                            if entry.name.endswith(".json") and entry.is_file(follow_symlinks=False):
                                names.append(Path(entry.path))
                    paths = sorted(names)[:policy.maximum_files]
                    if len(names) > len(paths):
                        self.records.append({"status": "INPUT_FILE_LIMIT", "omitted_files": len(names)-len(paths)})
                else:
                    paths = [source]
                for path in paths:
                    if stop is not None and perf_counter() >= stop:
                        self.records.append({"status": "SCREENING_DEADLINE"})
                        break
                    record = {"path": str(path.resolve())}
                    self.records.append(record)
                    try:
                        if path.suffix != ".json" or path.is_symlink():
                            raise ValueError("only regular JSON files are supported")
                        with path.open("rb") as stream:
                            data = stream.read(policy.maximum_file_bytes + 1)
                        if len(data) > policy.maximum_file_bytes:
                            raise ValueError("history file size limit")
                        record["sha256"] = hashlib.sha256(data).hexdigest()
                        document = _json(data)
                        if not isinstance(document, dict):
                            raise ValueError("historical motion must be a JSON object")
                        if document.get("schema") != "m710id70_layout_single_carton_motion_audit_v1":
                            raise ValueError("unsupported historical motion schema")
                        fingerprint = document.get("evidence_fingerprint")
                        if fingerprint != canonical_digest({k: v for k, v in document.items()
                                                            if k != "evidence_fingerprint"}):
                            raise ValueError("history content fingerprint mismatch")
                        if stop is not None and perf_counter() >= stop:
                            raise ValueError("SCREENING_DEADLINE")
                        if any(item[1]["sha256"] == record["sha256"] for item in self.documents):
                            record["status"] = "DUPLICATE_SOURCE"
                            continue
                        record["status"] = "READ_HINT_ONLY"
                        self.documents.append((document, record))
                    except (OSError, ValueError, KeyError, TypeError, RecursionError, OverflowError) as exc:
                        record.update(status="REJECTED", reason=str(exc))
                if not paths:
                    self.records.append({"status": "EMPTY_SOURCE"})
            except (OSError, ValueError) as exc:
                self.records.append({"status": "SOURCE_UNAVAILABLE", "reason": str(exc)})
        self.screening_seconds = perf_counter() - started

    def candidates(self, scene, connector, target, backend):
        records = self.targets_screened.setdefault(target.name, [])
        found = []
        for document, source in self.documents:
            if len(found) >= self.policy.maximum_candidates or connector._deadline_reached():
                break
            record = {"source_sha256": source["sha256"], "target": target.name}
            records.append(record)
            try:
                hint = compatible_hint(document, scene, connector, target, backend, self.policy)
                hint["provenance"] = {**source, "source_evidence_fingerprint": document["evidence_fingerprint"],
                    "old_scene_fingerprint": document["scene_fingerprint"],
                    "old_policy_fingerprint": document["policy_fingerprint"],
                    "old_implementation_identity": document["implementation_identity"],
                    "validation_inherited": False}
                found.append(hint)
                record["status"] = "COMPATIBLE_HINT_REQUIRES_FULL_REVALIDATION"
            except (ValueError, KeyError, TypeError, IndexError, OverflowError, np.linalg.LinAlgError) as exc:
                record.update(status="REJECTED", reason=str(exc))
        return found

    def evidence(self):
        return {"source": self.policy.source, "screening_seconds": self.screening_seconds,
                "files": self.records, "targets": self.targets_screened,
                "old_validation_inherited": False}


def compatible_hint(old, scene, c, target, backend, policy):
    from .layout_trajectory import validate_layout_trajectory_stage_contract
    from .release_motion import MOTION_SEMANTICS
    from .conveyor_placement import PLACEMENT_SEMANTICS
    segment = old["selected_trajectory_segment"]
    if segment["target"] != target.name:
        raise ValueError("target identity mismatch")
    if old["layout_fingerprint"] != scene.snapshot["layout_fingerprint"]:
        raise ValueError("fixed layout mismatch")
    original = old["trajectory_backend"]["source_identity"]
    current = backend["source_identity"]
    # Deliberately exclude mutable policy, scene and implementation identity.
    identity_keys = ("urdf_sha256", "official_manifest_sha256", "official_upstream_commit",
                     "collision_link_names", "rigid_tool_solid_count", "rigid_tool_compound_q0_sha256",
                     "base_transform", "flange_from_virtual_task_tcp", "flange_from_physical_contact")
    for key in identity_keys:
        if original[key] != current[key]:
            raise ValueError(f"robot/tool compatibility mismatch: {key}")
    # Compare the audited SRDF semantics, not platform-dependent line endings.
    for key in ("group", "disabled_self_collision_pairs", "non_adjacent_exclusions_allowed"):
        if old["trajectory_backend"]["official_srdf_policy"][key] != backend["official_srdf_policy"][key]:
            raise ValueError(f"SRDF semantics mismatch: {key}")
    if old["strict_contract"]["tool_frames"] != scene.policy.tool_frames.evidence():
        raise ValueError("physical/virtual tool frame convention mismatch")
    if segment.get("motion_semantics") != MOTION_SEMANTICS or segment.get("placement_semantics") != PLACEMENT_SEMANTICS:
        raise ValueError("unsupported historical stage semantics")
    validate_layout_trajectory_stage_contract(segment)  # Structure only; no old pass is accepted.
    path = np.asarray(segment["path"], float)
    if len(path) > 3000 or path.shape[1] != len(c.robot.joint_limits):
        raise ValueError("history path size or joint dimension mismatch")
    declared_order = old.get("history_compatibility", {}).get("joint_names")
    order = list(scene.snapshot["robot"]["joint_names"])
    if declared_order is not None and declared_order != order:
        raise ValueError("joint order mismatch")
    # The supported legacy schema uses the verified URDF active-joint order.
    # Confirm its interpretation against recorded FK; never infer an arbitrary order.
    for q, recorded in ((path[segment["grasp_index"]], segment["contact"]["actual_virtual_task_tcp_pose_world"]),):
        if not np.allclose(c.robot.fk(q), recorded, atol=1e-9, rtol=0):
            raise ValueError("historical joint/FK interpretation mismatch")
    dimensions = np.asarray(segment["place"]["selection"]["half_extents_m"], float)
    if not np.array_equal(dimensions, target.half_extents):
        raise ValueError("target dimensions mismatch")
    matches = [task for task in old["tasks"] if task["task_id"] == target.name]
    if len(matches) != 1:
        raise ValueError("ambiguous historical target pose")
    old_box = c._se3(np.asarray(matches[0]["target_pose_world"]), "historical target")
    if (np.linalg.norm(old_box[:3, 3] - target.center) > policy.maximum_target_translation_m
            or np.linalg.norm(rotation_vector_from_matrix(target.rotation @ old_box[:3, :3].T)) > policy.maximum_target_rotation_rad):
        raise ValueError("target pose change exceeds local adaptation bounds")
    old_contact = c._se3(np.asarray(segment["contact"]["actual_physical_contact_pose_world"]), "historical physical contact")
    local_contact = np.linalg.inv(old_box) @ old_contact
    approach = segment["approach"]
    gate = approach.get("free_connection_end_index")
    if gate is None:
        # Explicit legacy direct-approach contract: one appended node per terminal sample.
        accepted = [a for a in approach["attempts"] if a["failure"] is None]
        if approach.get("selected_mode") != "direct" or len(accepted) != 1:
            raise ValueError("legacy free/contact boundary is ambiguous")
        gate = segment["grasp_index"] - len(accepted[0]["search"]["terminal"]["samples"])
    if type(gate) is not int or not 0 <= gate < segment["grasp_index"]:
        raise ValueError("invalid history free/contact boundary")
    return {"source_profile": old.get("simulation_profile", {"name": "physical_reception"}),
            "validation_inherited": False, "segment": segment, "local_contact": local_contact,
            "old_target_pose": old_box, "gate": gate, "policy": policy}


def contact_variants(hint, target):
    policy = hint["policy"]
    contact = target.world_from_local @ hint["local_contact"]
    count = min(policy.maximum_variants, 1 + int(np.floor(policy.maximum_normal_offset_m / policy.normal_step_m + 1e-9)))
    for index in range(count):
        displacement = index * policy.normal_step_m
        pose = contact.copy()
        pose[:3, 3] -= pose[:3, 2] * displacement
        yield pose, {"normal_outward_m": displacement, "frame": "CURRENT_PHYSICAL_CONTACT",
                     "offset_local_m": [0., 0., -displacement]}


def evaluate_history(source, scene, c, target, backend, *, deadline, attempt_limit):
    """Same connector/request/target, with a bounded share before normal search."""
    attempts, selected = [], None
    if c is None or (deadline is not None and perf_counter() >= deadline):
        return attempts, selected
    seen = set()
    with c._budget_scope(deadline):
        hints = source.candidates(scene, c, target, backend)
        for hint in hints:
            old = hint["segment"]
            for physical, offset in contact_variants(hint, target):
                if len(attempts) >= attempt_limit or c._deadline_reached():
                    return attempts, selected
                geometry_id = canonical_digest([target.name, target.world_from_local.tolist(),
                    old["face"], physical.tolist(), scene.snapshot["scene_fingerprint"]])
                identity = canonical_digest([hint["provenance"]["sha256"], geometry_id,
                                             scene.policy.policy_fingerprint])
                if identity in seen:
                    continue
                seen.add(identity)
                seed = int(identity[:8], 16)
                provenance = {"source": hint["provenance"], "candidate_id": identity,
                    "geometry_id": geometry_id, "offset": offset,
                    "old_contact_pose_world": old["contact"]["actual_physical_contact_pose_world"],
                    "old_target_pose_world": hint["old_target_pose"].tolist(),
                    "current_target_pose_world": target.world_from_local.tolist(),
                    "requested_contact_pose_world": physical.tolist(),
                    "old_commanded_mask": old["contact"]["cup_selection"]["commanded_active_mask"],
                    "old_physical_contact_from_box": old["contact"]["physical_contact_from_box"],
                    "old_release_q_rad": old["path"][old["release_index"]],
                    "old_release_box_pose_world": old["place"]["actual_box_pose_world"],
                    "reconstructed": ["actual_start_join", "terminal_contact", "attachment", "receiver_endpoints", "release_prediction", "events"],
                    "retained_as_hints_full_recheck": ["free_prefix", "support-release", "extraction", "transit", "place", "withdrawal"]}
                hint = {**hint, "attempt_provenance": provenance}
                started = perf_counter()
                if getattr(c, "progress_callback", None):
                    c.progress_callback({"event": "HISTORY_CANDIDATE_STARTED", "target": target.name,
                        "candidate_id": identity, "offset": offset, "seed": seed, "round": 0, "stage": "history_adaptation"})
                previous_slice = getattr(c, "candidate_slice_s", None)
                c.candidate_slice_s = source.policy.candidate_wall_time_s
                try:
                    with c._contact_context():
                        outcome = c.plan(target=target, face=old["face"],
                            requested_virtual_contact=c.virtual_from_physical(physical),
                            grasp_candidates=[{"q_rad": old["path"][old["grasp_index"]], "candidate_id": identity}],
                            home_q=scene.policy.layout_validation.initial_q, all_obstacles=scene.all_obstacles,
                            receiver=scene.receiver, support_names=tuple(scene.support_graph.supported_by[target.name]),
                            suction=scene.policy.data["suction"], seed=seed, history_hint=hint)
                finally:
                    if previous_slice is None:
                        del c.candidate_slice_s
                    else:
                        c.candidate_slice_s = previous_slice
                failure = outcome.failure or {}
                history = (outcome.attempts[-1].get("trace", {}).get("history", provenance)
                           if outcome.attempts else provenance)
                q = history.get("new_contact_q_rad")
                attempt = {"source": "HISTORY_ADAPTATION", "face": old["face"], "roll_deg": 0,
                    "candidate_id": identity, "attempt_id": identity + ":history_0", "geometry_id": geometry_id,
                    "family_id": canonical_digest([target.name, old["face"], "history"]),
                    "requested_physical_contact_pose_world": physical.tolist(),
                    "requested_virtual_task_tcp_pose_world": c.virtual_from_physical(physical).tolist(),
                    "strict_grasp_candidates": [] if q is None else [{"q_rad": q, "candidate_id": identity}],
                    "ik_stream": None, "complete_trajectory": outcome.success,
                    "failure_stage": failure.get("stage"), "failure_reason": failure.get("reason"),
                    "search_status": outcome.statistics["termination"], "path_search": "PASS" if outcome.success else "FAIL",
                    "path_connection_attempts": int(outcome.statistics.get("connection_attempts", 0)),
                    "trajectory_search": {"statistics": outcome.statistics, "attempts": list(outcome.attempts), "failure": outcome.failure},
                    "history": history, "elapsed_s": perf_counter()-started,
                    "actual_complete_connection_attempted": True}
                attempts.append(attempt)
                if getattr(c, "progress_callback", None):
                    c.progress_callback({"event": "HISTORY_CANDIDATE_FINISHED", "target": target.name,
                        "candidate_id": identity, "success": outcome.success, "failure": outcome.failure,
                        "elapsed_s": attempt["elapsed_s"]})
                if outcome.success:
                    return attempts, outcome.segment
    return attempts, selected


def register_planned_motion(directory, result, preflight):
    """Append a content-addressed hint; never append physical completion events."""
    if result.get("complete_trajectory_status") != "PASS" or not preflight.get("simulation_execution_ready"):
        raise ValueError("only a current complete plan with ready preflight can be registered")
    fingerprint = result.get("evidence_fingerprint")
    if (fingerprint != canonical_digest({k: v for k, v in result.items() if k != "evidence_fingerprint"})
            or preflight.get("input_identity", {}).get("motion_evidence_fingerprint") != fingerprint):
        raise ValueError("history registration requires the same bound motion and preflight")
    data = json.dumps(result, indent=2, allow_nan=False).encode("utf-8")
    digest = hashlib.sha256(data).hexdigest()
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    destination = root / (digest + ".json")
    try:
        with destination.open("xb") as stream:
            stream.write(data)
    except FileExistsError:
        if destination.read_bytes() != data:
            raise ValueError("registered hash path contains different data")
    return {"path": str(destination), "sha256": digest, "planning_passed": True,
            "preflight_passed": True, "physical_execution_completed": False,
            "completed_carton_ids_modified": False}
