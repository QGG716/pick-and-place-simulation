"""Optional resident MTC/Pilz/OMPL adapter; importing this module requires no ROS.

Native collision checks generate candidates. Existing exact full-edge checks
remain authoritative, including pair gaps and bounded contact semantics.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import atexit
import itertools
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import threading
from time import perf_counter
import uuid
import xml.etree.ElementTree as ET

import numpy as np

from .layout_trajectory import LayoutTrajectoryConnector, OFFICIAL_MODEL_URDF, OFFICIAL_MODEL_SRDF
from .stage_motion_policy import MotionPurpose, GenerationMethod, require_purpose

SCHEMA = "m710_native_stage_v1"
TASK_TCP_LINK = "m710_virtual_task_tcp"
JOINT_NAMES = [f"J{i}" for i in range(1, 7)]


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def box_message(box, pose=None):
    return dict(id=box.name, size=(2*np.asarray(box.half_extents)).tolist(),
                pose=np.asarray(box.world_from_local if pose is None else pose).tolist(),
                category=box.category, geometry_source="frozen_layout_or_audited_tool_OBB")


class MoveItUnavailable(RuntimeError):
    pass


def native_rest_start_contract(snapshot):
    """Validate an observed rest gate before requesting stopped native motion.

    Raw velocities are retained.  A small PhysX velocity alone is insufficient:
    the existing position-derived-speed and stable-duration gate must replay.
    """
    robot=snapshot.get("robot",{})
    observed=robot.get("qd_rad_s")
    context=snapshot.get("actual_state_context") or {}
    if observed is None:
        if context:
            raise MoveItUnavailable("UNSUPPORTED_MISSING_START_VELOCITY")
        return dict(schema="m710_native_rest_start_contract_v1",source="CONFIGURED_DEVELOPMENT_REST_STATE",
            measured_qd_rad_s=None,solver_start_velocity_rad_s=[0.]*6,measured_rest_gate_used=False,
            physical_initial_state_claim=False)
    velocity=np.asarray(observed,dtype=float)
    if velocity.shape!=(6,) or not np.isfinite(velocity).all():
        raise MoveItUnavailable("UNSUPPORTED_NONZERO_OR_INVALID_START_VELOCITY")
    evidence=robot.get("native_rest_start_evidence",context.get("native_rest_start_evidence"))
    if evidence is None and np.all(velocity==0):
        return dict(schema="m710_native_rest_start_contract_v1",source="MEASURED_EXACT_ZERO_VELOCITY",
            measured_qd_rad_s=velocity.tolist(),solver_start_velocity_rad_s=[0.]*6,measured_rest_gate_used=False,
            world_session_id=context.get("world_session_id"))
    if not isinstance(evidence,dict):
        raise MoveItUnavailable("UNSUPPORTED_NONZERO_OR_INVALID_START_VELOCITY: REST_GATE_EVIDENCE_REQUIRED")
    from .m710_replay_physics import RestStartGate
    gate=RestStartGate()
    keys=("maximum_wait_s","position_tolerance_rad","velocity_tolerance_rad_s",
          "position_derived_speed_tolerance_rad_s","required_stable_s")
    expected={key:getattr(gate,key) for key in keys}
    source=Path(__file__).with_name("m710_replay_physics.py")
    source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
    if (evidence.get("schema")!="m710_native_rest_start_evidence_v1" or evidence.get("gate")!="RestStartGate" or
            evidence.get("passed") is not True or evidence.get("parameters")!=expected or
            evidence.get("implementation_sha256")!=source_sha):
        raise MoveItUnavailable("NATIVE_REST_GATE_CONTRACT_MISMATCH")
    session=context.get("world_session_id")
    if not session or evidence.get("world_session_id")!=session:
        raise MoveItUnavailable("NATIVE_REST_GATE_WORLD_MISMATCH")
    samples=evidence.get("samples")
    if not isinstance(samples,list) or len(samples)<2:
        raise MoveItUnavailable("NATIVE_REST_GATE_OBSERVATIONS_MISSING")
    previous=None
    for row in samples:
        time=float(row["time_s"])
        values=[np.asarray(row[key],dtype=float) for key in ("q_rad","qd_rad_s","reference_q_rad")]
        if (not np.isfinite(time) or time<0 or (previous is not None and time<=previous) or
                any(q.shape!=(6,) or not np.isfinite(q).all() for q in values)):
            raise MoveItUnavailable("NATIVE_REST_GATE_INVALID_OBSERVATION")
        if gate.passed:
            raise MoveItUnavailable("NATIVE_REST_GATE_MEASUREMENT_AFTER_PASS")
        result=gate.evaluate(time,*values)
        if result!=row.get("result") or result.get("reason") is not None:
            raise MoveItUnavailable("NATIVE_REST_GATE_REPLAY_MISMATCH")
        previous=time
    if not gate.passed or result["hold"]:
        raise MoveItUnavailable("NATIVE_REST_GATE_NOT_PASSED")
    if not np.array_equal(values[0],np.asarray(robot.get("q_rad"))) or not np.array_equal(values[1],velocity):
        raise MoveItUnavailable("NATIVE_REST_GATE_LAST_MEASUREMENT_MISMATCH")
    return dict(schema="m710_native_rest_start_contract_v1",source="EXISTING_MEASURED_REST_START_GATE",
        world_session_id=session,measured_q_rad=values[0].tolist(),measured_qd_rad_s=velocity.tolist(),
        solver_start_velocity_rad_s=[0.]*6,measured_rest_gate_used=True,
        gate_evidence_sha256=digest(evidence),gate_implementation_sha256=source_sha,gate_parameters=expected,
        gate_final_observation=deepcopy(samples[-1]),
        transformation="stopped native boundary conditioned on measured stable-position equilibrium; raw velocity retained",
        execution_requires_same_world_rest_start_gate=True)


class NativeIKCandidateStream:
    """Lazy deterministic native IK; FK/collision authority remains independent."""
    def __init__(self, connector, pose, seeds, obstacles, *, seed, attachment=None,
                 support_names=(), target_contact=None, stage, contact_candidate=None,
                 endpoint_checks=None):
        self.connector, self.pose, self.obstacles = connector, np.asarray(pose), obstacles
        self.seed, self.stage = int(seed), stage
        self.options = dict(attachment=attachment, support_names=support_names,
                            target_contact=target_contact)
        self.contact_candidate, self.endpoint_checks = contact_candidate, endpoint_checks
        self.seeds = list({tuple(np.asarray(q, dtype=float)): np.asarray(q, dtype=float)
                           for q in seeds}.values())
        self.explicit_seed_count = len(self.seeds)
        self.seeds.extend(np.random.default_rng(seed).uniform(
            connector.robot.joint_limits[:, 0], connector.robot.joint_limits[:, 1],
            size=(int(connector.ik["random_restarts"]), 6)))
        self.seed_index = 0
        self.solutions = []
        self.best_failure = None
        self.termination = "NOT_STARTED"
        self.native_calls = 0
        self.converged_pose_results = 0
        self.duplicate_candidates = 0

    def __iter__(self):
        return self

    def __next__(self):
        from contextlib import nullcontext
        from .ik import IKResult, pose_error
        c = self.connector
        if len(self.solutions) >= c.budget.stage_ik_candidates:
            self.termination = "IK_CANDIDATE_LIMIT_REACHED"
            raise StopIteration
        while self.seed_index < len(self.seeds):
            if c._native_cancelled():
                self.termination = "VALIDATION_CANCELLED" if getattr(c, "cancel_requested", False) else "PLANNING_WALL_CLOCK_DEADLINE"
                raise StopIteration
            index = self.seed_index
            q_seed = self.seeds[index]
            self.seed_index += 1
            request = c._build_native_request(q_seed, None, self.obstacles, seed=self.seed+index,
                stage=self.stage, goal_pose=self.pose, **self.options)
            request.update(op="ik", position_tolerance_m=float(c.ik["position_tolerance_m"]),
                orientation_tolerance_rad=float(c.ik["orientation_tolerance_rad"]),
                ik_timeout_s=c.native_ik_seconds, collision_check_in_native_ik=False)
            started = perf_counter()
            stage_seconds, timeout = c._native_limits()
            request["allowed_planning_time_s"] = stage_seconds
            request["ik_timeout_s"] = min(c.native_ik_seconds, stage_seconds)
            raw = c.native.request(request, timeout=timeout, cancelled=c._native_cancelled)
            self.native_calls += int(raw.get("native_ik_calls", 0))
            record = {**raw, "stage": self.stage, "seed": self.seed+index,
                "request_fingerprint": digest(request), "elapsed_s": perf_counter()-started,
                "input_state_sha256": digest(q_seed.tolist())}
            c.native_ik_evidence.append(record)
            if raw.get("native_ik_calls") != 1:
                raise MoveItUnavailable("NATIVE_IK_CALL_EVIDENCE_MISSING")
            if raw.get("status") != "SUCCESS":
                self.termination = raw.get("status", "NATIVE_IK_FAILED")
                continue
            q = np.asarray(raw.get("q"), dtype=float)
            if q.shape != (6,) or not np.isfinite(q).all() or not c.robot.within_limits(q):
                raise MoveItUnavailable("NATIVE_IK_INVALID_JOINT_STATE")
            _, position, angle = pose_error(c.robot.fk(q), self.pose)
            if position > c.ik["position_tolerance_m"] or angle > c.ik["orientation_tolerance_rad"]:
                record["authority_failure"] = {"reason": "NATIVE_IK_FK_RESIDUAL", "position_m": position, "orientation_rad": angle}
                continue
            self.converged_pose_results += 1
            if any(np.max(np.abs(q-old)) <= c.ik["candidate_dedup_tolerance_rad"] for old in self.solutions):
                self.duplicate_candidates += 1
                record["authority_failure"] = {"reason": "DUPLICATE_NATIVE_IK"}
                continue
            contact = self.contact_candidate is not None and self.options["target_contact"] is not None
            endpoint_stage = "contact_endpoint" if contact else self.stage+"_ik_endpoint"
            failure, cups = None, None
            with c._contact_context() if contact else nullcontext():
                if contact:
                    try:
                        cups = c._contact_selection(q, self.options["target_contact"],
                            self.contact_candidate["face"], self.contact_candidate["suction"])
                    except ValueError as exc:
                        failure = {"reason": "NATIVE_IK_CONTACT_COVERAGE_FAILED", "detail": str(exc)}
                if failure is None:
                    failure = c._state_failure(q, self.obstacles, stage=endpoint_stage, **self.options)
            record["authority_failure"] = failure
            if self.endpoint_checks is not None:
                self.endpoint_checks.append(dict(requested_stage=self.stage, endpoint_stage=endpoint_stage,
                    q_rad=q.tolist(), failure=failure, native_request_id=raw.get("request_id"),
                    commanded_active_mask=None if cups is None else cups["commanded_active_mask"]))
            result = IKResult(failure is None, q, 0, float(position), float(angle),
                "native MoveIt IK with independent endpoint validation", {**self.evidence(),
                    "candidate_id": "native-ik-"+digest([request, raw.get("request_id"), q.tolist()]),
                    "native_request_id": raw.get("request_id"), "native_solver": raw.get("solver")})
            if failure is not None:
                self.best_failure = result
                continue
            record.update(candidate_id=result.search_evidence["candidate_id"],
                target_id=None if self.options["target_contact"] is None else self.options["target_contact"].name,
                task_id=c.native_task_id)
            self.solutions.append(q.copy())
            self.termination = "YIELDED_NATIVE_IK"
            return result
        self.termination = "NATIVE_IK_SEEDS_EXHAUSTED"
        raise StopIteration

    def evidence(self):
        return dict(generator="moveit_native_ik", seeds_available=len(self.seeds),
            seed_pool_available=len(self.seeds),
            seeds_attempted=self.seed_index, unsearched_seed_count=len(self.seeds)-self.seed_index,
            explicit_seed_count=self.explicit_seed_count, native_ik_calls=self.native_calls,
            iterations_consumed=0, iterations_observable=False, valid_solutions=len(self.solutions),
            converged_pose_results=self.converged_pose_results,deduplicated_candidates=len(self.solutions),
            duplicate_candidates=self.duplicate_candidates,
            termination=self.termination, candidate_limit=self.connector.budget.stage_ik_candidates)


class ResidentMoveItClient:
    """One serialized JSONL stream. Timeout poisons the process; no late replies."""
    def __init__(self, command, *, log_path=None):
        if not command:
            raise MoveItUnavailable("MOVEIT2_UNAVAILABLE: set M710_MOVEIT_COMMAND to the resident worker launcher")
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._replies = queue.Queue()
        self._log = open(log_path, "a", encoding="utf-8") if log_path else None
        try:
            self.process = subprocess.Popen(shlex.split(command) if isinstance(command, str) else command,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self._log,
                text=True, encoding="utf-8", bufsize=1)
        except OSError as exc:
            if self._log: self._log.close()
            raise MoveItUnavailable(f"MOVEIT2_UNAVAILABLE: {exc}") from exc
        def read():
            try:
                for line in self.process.stdout:
                    self._replies.put(json.loads(line))
            except Exception as exc:
                self._replies.put(exc)
            finally:
                self._replies.put(MoveItUnavailable("MOVEIT2_WORKER_EXITED"))
        threading.Thread(target=read, daemon=True).start()
        atexit.register(self.close)

    def request(self, payload, *, timeout=120., cancelled=None):
        with self._lock:
            request = {**payload, "request_id": str(next(self._ids))}
            started = perf_counter()
            try:
                self.process.stdin.write(json.dumps(request, allow_nan=False)+"\n")
                self.process.stdin.flush()
                while True:
                    if cancelled is not None and cancelled():
                        raise MoveItUnavailable("CANCELLED")
                    if perf_counter()-started >= timeout:
                        raise MoveItUnavailable("MOVEIT2_REQUEST_TIMEOUT")
                    try:
                        result = self._replies.get(timeout=min(.1, timeout))
                        break
                    except queue.Empty:
                        continue
                if isinstance(result, Exception): raise result
                if result.get("request_id") != request["request_id"]:
                    raise MoveItUnavailable("MOVEIT2_REQUEST_ID_MISMATCH")
                result["transport_and_worker_s"] = perf_counter()-started
                if result.get("status") == "ERROR":
                    raise MoveItUnavailable(result.get("detail", "MOVEIT2_ERROR"))
                return result
            except Exception:
                self.close()
                raise

    def close(self):
        atexit.unregister(self.close)
        if self.process.poll() is None:
            self.process.terminate()
            try: self.process.wait(timeout=5)
            except subprocess.TimeoutExpired: self.process.kill(); self.process.wait()
        if self._log and not self._log.closed: self._log.close()


def _rpy(rotation):
    # These fixed assembly/tool transforms use the ordinary URDF Rz Ry Rx convention.
    r = np.asarray(rotation)
    pitch = np.arctan2(-r[2, 0], np.hypot(r[0, 0], r[1, 0]))
    if abs(np.cos(pitch)) < 1e-10:
        return [0., float(pitch), float(np.arctan2(-r[0, 1], r[1, 1]))]
    return [float(np.arctan2(r[2, 1], r[2, 2])), float(pitch), float(np.arctan2(r[1, 0], r[0, 0]))]


def native_tool_collision_boxes(connector,q):
    """Use the authority's nominally compressed lip solids, by exact ownership."""
    original=list(connector.tool_collision_obbs_provider(q))
    compressed=list(connector.robot_state_validator._compliant_boxes(q))
    by_name={box.name:box for box in compressed}
    names=[box.name for box in original]
    if len(by_name)!=len(compressed) or len(set(names))!=len(names) or not set(by_name)<=set(names):
        raise ValueError("NATIVE_TOOL_COMPLIANT_OWNERSHIP_MISMATCH")
    boxes=[by_name.get(box.name,box) for box in original]
    return boxes,dict(source="ExactM710LayoutStateValidator._compliant_boxes",
        physical_geometry_source="tool_collision_obbs_provider",
        nominal_compression_m=float(connector.robot_state_validator.nominal_cup_compression_m),
        compliant_replacements=sorted(by_name),rigid_links_unchanged=sorted(set(names)-set(by_name)),
        collision_semantics="same_nominal_compressed_lip_OBBs_as_CPU_authority; additional_compression_limit_unchanged")


def model_request(connector, scene, *, asset_root=None, seed=71070):
    from .independent_cups import m710_cup_array_from_mapping
    root = scene.policy.project_root
    urdf = ET.parse(root / OFFICIAL_MODEL_URDF).getroot()
    names = [j.attrib["name"] for j in urdf.findall("joint") if j.attrib["type"] == "revolute"]
    srdf = ET.parse(root / OFFICIAL_MODEL_SRDF).getroot()
    mount = np.asarray(connector.robot.base_transform)
    for joint in urdf.findall("joint"):
        if joint.find("parent").attrib["link"] == "world" and joint.find("child").attrib["link"] == "base_link":
            origin = joint.find("origin")
            if origin is None: origin = ET.SubElement(joint, "origin")
            origin.set("xyz", " ".join(map(str, mount[:3, 3])))
            origin.set("rpy", " ".join(map(str, _rpy(mount[:3, :3]))))
    native_root = Path(asset_root) if asset_root else root
    package = native_root / "assets/robots/fanuc_m710id_70/official"
    for mesh in urdf.iter("mesh"):
        mesh.set("filename", "file://"+str(package / mesh.attrib["filename"].removeprefix("package://")).replace("\\", "/"))
    q0 = np.zeros(6)
    flange = connector.robot.named_link_frames(q0)["flange"]
    tool, tool_geometry = native_tool_collision_boxes(connector,q0)
    tool_names = [b.name for b in tool]
    compliant = {b.name for b in connector.robot_state_validator._compliant_boxes(q0)}
    for box in tool:
        local = np.linalg.inv(flange) @ box.world_from_local
        link = ET.SubElement(urdf, "link", name=box.name)
        collision = ET.SubElement(link, "collision")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", size=" ".join(map(str, 2*box.half_extents)))
        joint = ET.SubElement(urdf, "joint", name="mount_"+box.name, type="fixed")
        ET.SubElement(joint, "parent", link="flange"); ET.SubElement(joint, "child", link=box.name)
        ET.SubElement(joint, "origin", xyz=" ".join(map(str, local[:3, 3])), rpy=" ".join(map(str, _rpy(local[:3, :3]))))
    # Collisionless fixed frame: Pilz interpolates this link itself, including
    # rotation. Official physical links, tool solids and attachment frame stay intact.
    tcp = np.asarray(connector.flange_from_virtual_task_tcp)
    ET.SubElement(urdf, "link", name=TASK_TCP_LINK)
    joint = ET.SubElement(urdf, "joint", name="mount_"+TASK_TCP_LINK, type="fixed")
    ET.SubElement(joint, "parent", link="flange"); ET.SubElement(joint, "child", link=TASK_TCP_LINK)
    ET.SubElement(joint, "origin", xyz=" ".join(map(str,tcp[:3,3])), rpy=" ".join(map(str,_rpy(tcp[:3,:3]))))
    srdf.find("group[@name='manipulator']/chain").set("tip_link", TASK_TCP_LINK)
    # Owned tool solids form one assembly. Wrist exceptions retain exact ownership.
    for first, second in itertools.combinations(tool_names, 2):
        ET.SubElement(srdf, "disable_collisions", link1=first, link2=second, reason="Owned_tool_assembly")
    for first, second in sorted(connector.collision_policy.wrist_tool_pairs(tool_names)):
        ET.SubElement(srdf, "disable_collisions", link1=first, link2=second, reason="Approved_owned_wrist_tool")
    params = {"robot_description": ET.tostring(urdf, encoding="unicode"),
        "robot_description_semantic": ET.tostring(srdf, encoding="unicode"),
        "robot_description_kinematics.manipulator.kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
        "robot_description_kinematics.manipulator.kinematics_solver_timeout": .1,
        "pilz_industrial_motion_planner.planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        "pilz_industrial_motion_planner.request_adapters": "",
        "ompl.planning_plugin": "ompl_interface/OMPLPlanner",
        "ompl.request_adapters": "",
        "ompl.planner_configs.RRTConnectkConfigDefault.type": "geometric::RRTConnect",
        "ompl.planner_configs.RRTConnectkConfigDefault.range": .2,
        "ompl.manipulator.planner_configs": ["RRTConnectkConfigDefault"],
        "ompl.manipulator.longest_valid_segment_fraction": .001,
        "robot_description_planning.cartesian_limits.max_trans_vel": .2,
        "robot_description_planning.cartesian_limits.max_trans_acc": .4,
        "robot_description_planning.cartesian_limits.max_trans_dec": -.4,
        "robot_description_planning.cartesian_limits.max_rot_vel": .4}
    for joint in names:
        # Conservative experiment acceleration, not an invented vendor limit.
        params[f"robot_description_planning.joint_limits.{joint}.has_acceleration_limits"] = True
        params[f"robot_description_planning.joint_limits.{joint}.max_acceleration"] = .5
        # Humble Pilz derives deceleration=-acceleration. Do not predeclare
        # its extension parameters: 2.5.10 redeclares them internally.
    identity = dict(schema=SCHEMA,
        reviewed_sources={"feasibility_core":"4ceb9487e8e650a82243fa10b2c4e91ff3ec48cd",
                          "moveit_backend":"c7e4b486b3bf50fc607cf1c826c441fc688dbdfe"},
        scene_version="m710id70_unloading_layout_v1", scene_fingerprint=scene.snapshot.get("scene_fingerprint"),
        policy_fingerprint=scene.policy.policy_fingerprint, validator_identity=connector.validator_identity,
        model_tool_fingerprint=digest(params), task_tcp_link=TASK_TCP_LINK, flange_from_task_tcp=tcp.tolist(),
        task_tcp_fingerprint=digest(tcp.tolist()),
        policy_scope=("native_POC_stage_scoped_process_with_ordered_checks_plus_independent_authority"
                      if getattr(connector,"native_cold",False) else
                      "native_POC_free_pair_clearance_search_and_edges_plus_existing_authority"))
    return dict(op="init", parameters=params, identity=identity, joint_names=names, seed=seed,
        collision_policy=connector.collision_policy.to_mapping(),tool_links=sorted(tool_names),
        compliant_tool_links=sorted(compliant),
        tool_geometry_binding=tool_geometry,
        process_geometry=dict(cups=[cup.to_dict() for cup in m710_cup_array_from_mapping(scene.policy.data["suction"]).cups],
            flange_from_physical_contact=connector.flange_from_physical_contact.tolist()),
        expected_collision_shapes={link.attrib["name"]:len(link.findall("collision")) for link in urdf.findall("link") if link.findall("collision")}), tool_names, compliant


def validate_native_result(result, start, goal, names):
    if result.get("joint_names") != names: raise ValueError("OUTPUT_JOINT_ORDER_MISMATCH")
    points = result["points"]
    path = np.asarray([p["q"] for p in points], dtype=float)
    times = np.asarray([p["t"] for p in points], dtype=float)
    if path.ndim != 2 or path.shape[1] != 6 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("INVALID_NATIVE_PATH")
    if not np.isfinite(times).all() or abs(times[0])>1e-9 or np.any(np.diff(times)<=0):
        raise ValueError("INVALID_NATIVE_TIMES")
    if not np.allclose(path[0], start, atol=1e-9, rtol=0): raise ValueError("START_CHANGED")
    if goal is not None and not np.allclose(path[-1], goal, atol=1e-9, rtol=0): raise ValueError("GOAL_CHANGED")
    for point in points:
        for name in ("v", "a"):
            vector=np.asarray(point[name],dtype=float)
            if vector.shape!=(6,) or not np.isfinite(vector).all(): raise ValueError("INVALID_NATIVE_DERIVATIVES")
    for point in (points[0], points[-1]):
        if len(point["v"])!=6 or np.max(np.abs(point["v"]))>1e-8: raise ValueError("NONZERO_BOUNDARY_VELOCITY")
    return [q.copy() for q in path]


def linear_capability(origin, destination, flange_from_task_tcp, *, stage, location, bound_tcp=None):
    """Structural support only; no IK, reachability or collision queries."""
    values=[np.asarray(x,dtype=float) for x in (origin,destination,flange_from_task_tcp)]
    valid=all(x.shape==(4,4) and np.isfinite(x).all() and
        np.allclose(x[3],[0,0,0,1],atol=1e-12,rtol=0) and
        np.allclose(x[:3,:3].T@x[:3,:3],np.eye(3),atol=1e-8,rtol=0) and
        abs(np.linalg.det(x[:3,:3])-1)<1e-8 for x in values)
    reason=None
    if not valid: reason="UNSUPPORTED_INVALID_TCP_TRANSFORM"
    elif stage not in {"pregrasp","transit","residence","contact","support-release","extraction","place","withdrawal"}:
        reason="UNSUPPORTED_POLICY_STAGE"
    elif bound_tcp is not None and not np.array_equal(values[2],np.asarray(bound_tcp)):
        reason="UNSUPPORTED_TASK_TCP_CONTEXT_MISMATCH"
    return dict(schema="m710_native_capability_v2",supported=reason is None,reason=reason,
        stage=stage,location=location,reference_frame="world",
        implementation_scope="fixed_task_tcp_link_Pilz_LIN_shortest_rotation_stopped_motion",
        task_tcp_link=TASK_TCP_LINK,task_tcp_fingerprint=digest(values[2].tolist()) if valid else None,
        flange_from_task_tcp=values[2].tolist(),start_pose_world=values[0].tolist(),goal_pose_world=values[1].tolist())


class MoveItLayoutConnector(LayoutTrajectoryConnector):
    @classmethod
    def from_existing(cls, connector, scene, command=None, *, native_client=None,
                      native_cold=False, require_native_motion=None, native_budget=None):
        self = cls.__new__(cls); self.__dict__.update(connector.__dict__)
        self.native_cold = bool(native_cold)
        self.require_native_motion = self.native_cold if require_native_motion is None else bool(require_native_motion)
        if self.native_cold and not self.require_native_motion:
            raise ValueError("native-cold requires native motion")
        self.native_task_id = "native-cold-"+uuid.uuid4().hex
        self._native_stage_ids = itertools.count(1)
        self._cold_counters = dict(history_enabled=not self.native_cold, history_inputs_read=0,
            legacy_motion_generator_calls=0, forbidden_entry_attempts=0)
        self.native_ik_evidence = []
        self.native_semantic_events = []
        self._native_contact_context = None
        self.native_rest_start=native_rest_start_contract(scene.snapshot)
        self.native = native_client or ResidentMoveItClient(command or os.environ.get("M710_MOVEIT_COMMAND"),
            log_path=os.environ.get("M710_MOVEIT_LOG"))
        init, self.native_tools, self.native_compliant = model_request(self, scene,
            asset_root=os.environ.get("M710_MOVEIT_ASSET_ROOT"), seed=int(os.environ.get("M710_MOVEIT_SEED", "71070")))
        self.native_identity=init["identity"]; self.native_joint_names=init["joint_names"]
        limits = native_budget or {}
        self.native_stage_seconds=float(limits.get("stage_wall_time_s", os.environ.get("M710_MOVEIT_STAGE_SECONDS", "60")))
        self.native_request_timeout=float(limits.get("ipc_timeout_s", os.environ.get("M710_MOVEIT_REQUEST_TIMEOUT", "900")))
        self.native_task_seconds = limits.get("task_wall_time_s")
        self._native_task_started = None
        self.native_ik_seconds=float(os.environ.get("M710_MOVEIT_IK_SECONDS", "0.25"))
        if (not np.isfinite(self.native_stage_seconds) or self.native_stage_seconds<=0 or
                not np.isfinite(self.native_request_timeout) or self.native_request_timeout<=self.native_stage_seconds or
                not np.isfinite(self.native_ik_seconds) or self.native_ik_seconds<=0):
            self.native.close(); raise ValueError("invalid native stage time budget")
        self.native_scene=scene; self.native_evidence=[]; self.native_verified=[]; self.capability_evidence=[]; self.authority_path_evidence=[]
        try:
            started = perf_counter()
            self.native_startup=self.native.request(init)
            self.check_fk()
            self.native_model_warmup_s = perf_counter()-started
        except Exception:
            self.native.close(); raise
        return self

    def _native_cancelled(self):
        return bool(getattr(self, "cancel_requested", False) or self._deadline_reached() or
            (self._native_task_started is not None and self.native_task_seconds is not None and
             perf_counter()-self._native_task_started >= float(self.native_task_seconds)))

    def start_planning_request(self, start_monotonic=None):
        super().start_planning_request(start_monotonic)
        if hasattr(self,"native"):
            self.native_task_id="native-cold-"+uuid.uuid4().hex
            self._native_stage_ids=itertools.count(1)
            self._native_task_started=None
            self.native_evidence.clear();self.native_verified.clear();self.native_ik_evidence.clear()
            self.native_semantic_events.clear();self._native_contact_context=None
            self._cold_counters=dict(history_enabled=not self.native_cold,history_inputs_read=0,
                legacy_motion_generator_calls=0,forbidden_entry_attempts=0)

    def _native_limits(self):
        if self._native_task_started is None:
            self._native_task_started = perf_counter()
        deadlines = [float(value)-perf_counter() for value in (
            getattr(self, "_request_deadline_monotonic", None), getattr(self, "_deadline_monotonic", None))
            if value is not None]
        if self.native_task_seconds is not None:
            deadlines.append(float(self.native_task_seconds)-(perf_counter()-self._native_task_started))
        remaining = min(deadlines) if deadlines else float("inf")
        if remaining <= 0:
            raise MoveItUnavailable("NATIVE_REQUEST_SHARED_BUDGET_EXHAUSTED")
        return min(self.native_stage_seconds, remaining), min(self.native_request_timeout, remaining)

    def _forbid_legacy(self, entry):
        if getattr(self, "require_native_motion", False):
            self._cold_counters["forbidden_entry_attempts"] += 1
            raise MoveItUnavailable("NATIVE_COLD_FORBIDDEN_ENTRY:"+entry)

    def native_ik_stream(self, pose, seeds, obstacles, *, seed, attachment=None,
                         support_names=(), target_contact=None, stage,
                         contact_candidate=None, endpoint_checks=None):
        if not self.native_startup.get("capabilities", {}).get("native_ik", False):
            raise MoveItUnavailable("NATIVE_IK_UNAVAILABLE")
        return NativeIKCandidateStream(self, pose, seeds, obstacles, seed=seed,
            attachment=attachment, support_names=support_names, target_contact=target_contact,
            stage=stage, contact_candidate=contact_candidate, endpoint_checks=endpoint_checks)

    def _ik_stream(self, *args, **kwargs):
        if getattr(self, "require_native_motion", False):
            return self.native_ik_stream(*args, **kwargs)
        self._cold_counters["legacy_motion_generator_calls"] += 1
        return super()._ik_stream(*args, **kwargs)

    def _connect_pose(self, *args, **kwargs):
        selected,path,failure,evidence=super()._connect_pose(*args,**kwargs)
        if not getattr(self,"require_native_motion",False) or failure is not None or selected is None:
            return selected,path,failure,evidence
        if not path:
            raise MoveItUnavailable("NATIVE_CONNECTION_SUCCESS_WITHOUT_PATH")
        # The core returns its IK goal as `selected`. A native planner can emit
        # a numerically distinct, already checked endpoint. Carry that actual
        # sample into the next stage, without modifying either native path or
        # relaxing the exact source/parent match.
        endpoint=np.asarray(path[-1],dtype=float).copy()
        evidence={**evidence,"returned_endpoint":dict(source="NATIVE_PATH_FINAL_SAMPLE",
            ik_goal_q_rad=np.asarray(selected).tolist(),q_rad=endpoint.tolist(),
            differs_from_ik_goal=not np.array_equal(endpoint,np.asarray(selected)))}
        return endpoint,path,failure,evidence

    def _native_parent_stage_id(self,start):
        return next((record["stage_id"] for record in reversed(self.native_verified)
            if np.array_equal(np.asarray(record["points"][-1]["q"]),start)), "")

    def _rrt_transit(self, *args, **kwargs):
        self._forbid_legacy("core_rrt")
        self._cold_counters["legacy_motion_generator_calls"] += 1
        return super()._rrt_transit(*args, **kwargs)

    def _cartesian_process(self, *args, **kwargs):
        self._forbid_legacy("core_cartesian")
        self._cold_counters["legacy_motion_generator_calls"] += 1
        return super()._cartesian_process(*args, **kwargs)

    def plan(self, **kwargs):
        if getattr(self, "native_cold", False) and kwargs.get("history_hint") is not None:
            self._forbid_legacy("history_hint")
        if getattr(self,"require_native_motion",False):
            for candidate in kwargs.get("grasp_candidates",()):
                if not any(record.get("candidate_id")==candidate.get("candidate_id") and
                    record.get("task_id")==self.native_task_id and record.get("authority_failure") is None and
                    record.get("target_id")==kwargs["target"].name and
                    np.array_equal(np.asarray(record.get("q")),np.asarray(candidate.get("q_rad")))
                    for record in self.native_ik_evidence):
                    self._forbid_legacy("grasp_candidate_without_current_native_ik")
        return super().plan(**kwargs)

    def _contact_selection(self, q, target, face, suction):
        selection = super()._contact_selection(q, target, face, suction)
        self._native_contact_context = dict(target=target, face=face, suction=deepcopy(suction),
            selection=deepcopy(selection))
        return selection

    def _support_release(self,*args,**kwargs):
        path,failure,evidence=super()._support_release(*args,**kwargs)
        if getattr(self,"require_native_motion",False) and failure is None and len(path)==1:
            self.native_semantic_events.append(dict(stage="support-release",event="CONDITIONAL_ZERO_MOTION_SUPPORT_RELEASE",
                q_rad=np.asarray(path[0]).tolist(),evidence=deepcopy(evidence)))
        return path,failure,evidence

    def native_cold_evidence(self):
        stages = {}
        for record in self.native_evidence:
            stage = stages.setdefault(record.get("stage", "unknown"), {"PTP": 0, "LIN": 0, "OMPL": 0, "native_ik": 0})
            counts = record.get("native_solver_calls", {})
            for name in ("PTP", "LIN", "OMPL"):
                stage[name] += int(counts.get(name, 0))
        for record in self.native_ik_evidence:
            stage = stages.setdefault(record.get("stage", "unknown"), {"PTP": 0, "LIN": 0, "OMPL": 0, "native_ik": 0})
            stage["native_ik"] += int(record.get("native_ik_calls", 0))
        return {**self._cold_counters, "task_id": self.native_task_id, "per_stage_calls": stages,
            "ompl_invoked": any(stage["OMPL"] for stage in stages.values()),
            "native_stage_seconds": self.native_stage_seconds, "native_ik_seconds": self.native_ik_seconds,
            "native_task_seconds": self.native_task_seconds,
            "ipc_watchdog_seconds": self.native_request_timeout,
            "model_warmup_s": self.native_model_warmup_s,
            "rest_start_contract":deepcopy(self.native_rest_start),
            "zero_length_semantic_events": deepcopy(self.native_semantic_events)}

    def check_fk(self):
        checks=[]
        for q in (np.zeros(6), np.array([.2,-.3,.4,.5,-.6,.7]), np.array([-.4,.2,-.35,-.6,.4,-.8])):
            links=["base_link", "J3_link", "J6_link", "flange", "fanuc_flange", "tool0"]
            reference=self.robot.named_link_frames(q)
            links=[l for l in links if l in reference]
            result=self.native.request(dict(op="fk",identity=self.native_identity,q_start=q.tolist(),links=links+[TASK_TCP_LINK]))
            error=max(float(np.max(np.abs(np.asarray(result["frames"][l])-reference[l]))) for l in links)
            tcp=np.asarray(result["frames"][TASK_TCP_LINK])
            error=max(error,float(np.max(np.abs(tcp-self.robot.fk(q)))))
            checks.append(dict(q=q.tolist(),max_matrix_error=error,links=links))
            if error>1e-8: raise MoveItUnavailable(f"MODEL_FK_MISMATCH: {error}")
        self.native_fk=checks

    def _build_native_request(self,start,goal,obstacles,*,seed,attachment=None,support_names=(),target_contact=None,
                              initial_proximity=None,stage,goal_pose=None):
        world=[box_message(b) for b in obstacles]
        if len({b["id"] for b in world})!=len(world): raise ValueError("DUPLICATE_OBJECT")
        pairs=[["base_link", self.robot_state_validator.base_support_obstacle_name]]
        target_name=attachment.rigid.name if attachment is not None else (target_contact.name if target_contact is not None else self.robot_state_validator.contact_target_name)
        if self.collision_policy.compliant_cup_neighbor_contact_mode=="ignore" and target_name:
            for b in obstacles:
                if b.name!=target_name and b.name in (self.stack_carton_names or []):
                    pairs.extend([[cup,b.name] for cup in sorted(self.native_compliant)])
        attached=None
        if attachment is not None:
            b=attachment.box_at(start)
            attached=box_message(b,np.linalg.inv(self.robot.named_link_frames(start)["flange"])@b.world_from_local)
            attached["touch_links"]=sorted(self.native_compliant)
            if b.name not in {item["id"] for item in world}: world.append(box_message(b))
        if (not getattr(self, "require_native_motion", False) and target_contact is not None and
                stage in {"contact", "contact_endpoint", "withdrawal"}):
            pairs.extend([[cup, target_contact.name] for cup in sorted(self.native_compliant)])
        if getattr(self, "native_cold", False) and attached is not None:
            world = [item for item in world if item["id"] != attached["id"]]
        request=dict(schema=SCHEMA,op="plan",identity=self.native_identity,q_start=np.asarray(start).tolist(),
            world=world,scene_fingerprint=digest(world),allowed_pairs=pairs,attachment=attached,stage=stage,
            candidate_id=getattr(self,"_candidate_identity",None) or f"directed:{stage}:{seed}",start_velocity=[0.]*6,path_constraints={},
            clearance_mode=os.environ.get("M710_CLEARANCE_MODE","optimized"),
            seed=int(seed),cancelled=False,allowed_planning_time_s=getattr(self,"native_stage_seconds",60.),velocity_scale=.2,acceleration_scale=.2,
            flange_from_task_tcp=self.flange_from_virtual_task_tcp.tolist())
        if goal_pose is None: request["q_goal"]=np.asarray(goal).tolist()
        else: request["goal_pose"]=np.asarray(goal_pose).tolist()
        request["clearance_policy"]=dict(schema="m710_native_free_clearance_v1",stage=stage,
            source_policy=self.collision_policy.to_mapping(),numerical_gap_tolerance_m=1e-9,
            tool_links=sorted(self.native_tools),compliant_tool_links=sorted(self.native_compliant),
            stack_carton_ids=sorted(self.stack_carton_names or []),target_id=target_name,
            conveyor_ids=sorted(b.name for b in obstacles if b.category=="conveyor"),
            payload_id="" if attachment is None else attachment.rigid.name,
            receiver_reserve_m=self.budget.receiver_runtime_clearance_reserve_m,
            edge_resolution_rad=self.budget.edge_resolution_rad,
            allowed_pairs=pairs,attachment_contact_scope="fixed_rigid_attachment_cup_compression_remains_authoritative")
        if getattr(self, "require_native_motion", False):
            request["clearance_policy"]["schema"] = "m710_native_process_clearance_v1"
            request.update(task_id=self.native_task_id, require_native_motion=True,
                history_enabled=False, process_policy=self._native_process_policy(start, stage,
                    attachment=attachment, target_contact=target_contact, support_names=support_names,
                    initial_proximity=initial_proximity))
        return request

    def _native_process_policy(self, start, stage, *, attachment=None, target_contact=None,
                               support_names=(), initial_proximity=None):
        from .independent_cups import m710_cup_array_from_mapping
        context = self._native_contact_context
        target = (attachment.box_at(start) if attachment is not None else target_contact)
        if target is None and context is not None:
            target = context["target"]
        result = dict(schema="m710_native_process_v1", stage=stage,
            target_id=None if target is None else target.name,
            target=None if target is None else box_message(target),
            support_names=list(support_names), stack_carton_ids=sorted(self.stack_carton_names or []),
            collision_policy=self.collision_policy.to_mapping(),
            initial_proximity=None if initial_proximity is None else initial_proximity.evidence(),
            contact_tolerance_m=self.contact_tolerance_m,
            flange_from_physical_contact=self.flange_from_physical_contact.tolist(),
            max_attachment_gap_m=.002, maximum_penetration_m=self.contact_tolerance_m,
            max_normal_misalignment_rad=float(np.deg2rad(5.)),
            require_terminal_seal=stage=="contact", attached=attachment is not None)
        if context is not None and target is not None and context["target"].name == target.name:
            selection=context["selection"]
            array=m710_cup_array_from_mapping(context["suction"])
            result.update(target_face=context["face"],
                suction_edge_margin_m=float(context["suction"].get("suction_edge_margin_m", 0.)),
                cups=[cup.to_dict() for cup in array.cups],
                commanded_active_mask=list(selection["commanded_active_mask"]),
                geometrically_eligible_mask=list(selection["geometrically_eligible_mask"]),
                actual_contact_mask=list(selection["actual_contact_mask"]))
        return result

    def capability_check(self, start_pose, goal_pose, *, stage, location):
        result=linear_capability(start_pose,goal_pose,self.flange_from_virtual_task_tcp,stage=stage,location=location,
            bound_tcp=self.native_identity["flange_from_task_tcp"])
        result["native_planning_calls"]=sum(1 for a in self.native_evidence if a.get("mtc_attempt_index") is not None)
        self.capability_evidence.append(result)
        return None if result["supported"] else result

    def _native_plan(self,start,goal,obstacles,*,seed,attachment=None,support_names=(),target_contact=None,stage,
                     goal_pose=None,initial_proximity=None,allow_ompl=True,max_ompl_attempts=None,purpose=None,
                     try_ptp=True):
        started=perf_counter(); attempts=[]; seen=set()
        strict = getattr(self, "require_native_motion", False)
        constrained = bool(support_names or target_contact is not None or initial_proximity is not None or
            stage in {"contact", "support-release", "extraction", "place", "withdrawal"})
        capabilities = getattr(self, "native_startup", {}).get("capabilities", {})
        if strict and not capabilities.get("native_task_session", False):
            return [],dict(reason="NATIVE_TASK_GENERATION_UNAVAILABLE",stage=stage),dict(backend="moveit2",attempts=[])
        if strict and constrained and capabilities.get("process_policy_schema") != "m710_native_process_v1":
            return [],dict(reason="NATIVE_PROCESS_SEMANTICS_UNAVAILABLE",stage=stage,
                required="ordered_target_compression_cup_mask_support_and_stack_transition"),dict(backend="moveit2",attempts=[])
        if (not self.collision_policy.poc_pair_clearance or (not strict and (support_names or
                stage not in {"pregrasp","transit","residence"}))):
            return [],dict(reason="UNSUPPORTED_POLICY_STAGE",stage=stage),dict(backend="moveit2",attempts=[])
        if goal_pose is not None:
            failure=self.capability_check(self.robot.fk(start),goal_pose,stage=stage,location="before_native_request")
            if failure: return [],failure,dict(backend="moveit2",capability=failure)
        request=self._build_native_request(start,goal,obstacles,seed=seed,attachment=attachment,
            support_names=support_names,target_contact=target_contact,initial_proximity=initial_proximity,
            stage=stage,goal_pose=goal_pose)
        if strict:
            request["parent_stage_id"] = self._native_parent_stage_id(start)
            request["purpose"] = getattr(purpose, "value", purpose)
        endpoint_started=perf_counter();endpoints=[]
        for name,q in (("start",start),("goal",goal)):
            if q is None: continue
            failure=self._state_failure(q,obstacles,attachment=attachment,support_names=support_names,
                target_contact=target_contact,initial_proximity=None if initial_proximity is None else initial_proximity.clone(),stage=stage)
            endpoints.append(dict(endpoint=name,failure=failure))
            if failure:
                detail=dict(reason="INVALID_"+name.upper()+"_AUTHORITY",stage=stage,detail=failure,
                    q_rad=np.asarray(q).tolist(),native_planning_calls=0)
                self.native_evidence.append(detail)
                return [],detail,dict(backend="moveit2",endpoint_authority=endpoints,endpoint_authority_s=perf_counter()-endpoint_started)
        endpoint_s=perf_counter()-endpoint_started
        # No free-space fallback for constrained linear process motion.
        ompl_attempts = self.budget.stage_connection_attempts if max_ompl_attempts is None else max_ompl_attempts
        schedule=[("pilz_industrial_motion_planner","LIN")] if goal_pose is not None else [
            *([("pilz_industrial_motion_planner","PTP")] if try_ptp else []),*( [("ompl","RRTConnectkConfigDefault")]*
                (ompl_attempts if allow_ompl and not constrained else 0))]
        context=self._context_identity(obstacles,attachment=attachment,support_names=support_names,target_contact=target_contact,stage=stage)
        for pipeline,planner in schedule:
            if self._deadline_reached(): break
            submitted={**request,"pipeline_id":pipeline,"planner_id":planner}
            timeout = getattr(self,"native_request_timeout",900.)
            if strict:
                stage_seconds, timeout = self._native_limits()
                submitted["allowed_planning_time_s"] = stage_seconds
                submitted["stage_id"] = self.native_task_id+":"+str(next(self._native_stage_ids))
            request_log=os.environ.get("M710_MOVEIT_REQUEST_LOG")
            if request_log:
                with open(request_log,"a",encoding="utf-8") as stream: stream.write(json.dumps(submitted,allow_nan=False)+"\n")
            raw=self.native.request(submitted,timeout=timeout,
                cancelled=self._native_cancelled if strict else self._deadline_reached)
            attempt={**raw,"stage":stage,"seed":seed,"request_fingerprint":digest(submitted),
                "endpoint_authority":endpoints,"endpoint_authority_s":endpoint_s,
                "ipc_watchdog_s":timeout,"solver":pipeline+"/"+planner,
                "input_state_sha256":digest(np.asarray(start,dtype=float).tolist()),
                "constraints_sha256":digest({k:v for k,v in submitted.items()
                    if k not in {"q_start","request_id"}})}
            if strict:
                attempt["submitted_request"]=deepcopy(submitted)
            if goal_pose is not None:
                from .m710_execution_tcp import make_lin_contract
                attempt['lin_contract']=make_lin_contract(submitted,self.robot.fk(start),
                    position_tolerance=self.ik['position_tolerance_m'],orientation_tolerance=self.ik['orientation_tolerance_rad'],
                    edge_resolution=self.budget.edge_resolution_rad,root=Path(__file__).resolve().parents[2])
                if not strict:
                    attempt['stage_id']='lin-'+digest([attempt['request_fingerprint'],len(self.native_verified)])
            attempts.append(attempt)
            if raw["status"]!="SUCCESS":
                if raw["status"].startswith(("INVALID_START","INVALID_GOAL")): break
                continue
            if strict and (any(raw.get(key) != submitted.get(key) for key in ("task_id", "stage_id", "parent_stage_id"))
                    or raw.get("mtc_generation") is not True
                    or sum(raw.get("native_solver_calls", {}).values()) < 1
                    or (constrained and raw.get("process_semantics_checked") is not True)):
                attempt["authoritative_status"]="PROTOCOL_REJECTED"
                attempt["failure"]={"reason":"NATIVE_GENERATION_EVIDENCE_MISSING","stage":stage}
                self.native_evidence.extend(attempts)
                return [],attempt["failure"],dict(backend="moveit2",success=False,attempts=attempts)
            try:
                path=validate_native_result(raw,start,goal,self.native_joint_names)
            except ValueError as exc:
                attempt["authoritative_status"]="PROTOCOL_REJECTED"
                attempt["failure"]={"reason":str(exc),"stage":stage,"requested_q_goal":request.get("q_goal")}
                self.native_evidence.extend(attempts)
                return [],attempt["failure"],dict(backend="moveit2",success=False,attempts=attempts)
            path_id=digest([q.tolist() for q in path]);attempt["path_sha256"]=path_id
            if path_id in seen:
                attempt["authoritative_status"]="DUPLICATE_REJECTED_PATH"; continue
            if goal_pose is not None:
                from .moveit2_tcp import audit_linear_tcp
                checked=perf_counter()
                audit=audit_linear_tcp(path,self.robot.fk,self.robot.fk(start),goal_pose,
                    position_tolerance=float(self.ik["position_tolerance_m"]),
                    orientation_tolerance=float(self.ik["orientation_tolerance_rad"]),
                    edge_resolution_rad=self.budget.edge_resolution_rad)
                attempt["lin_constraint_audit"]=audit
                attempt["tcp_audit_s"]=perf_counter()-checked
                if not audit["passed"]:
                    attempt["authoritative_status"]="REJECTED"
                    attempt["failure"]={"reason":"LIN_TASK_TCP_CONSTRAINT","stage":stage}
                    self.native_evidence.extend(attempts)
                    return [],attempt["failure"],dict(backend="moveit2",success=False,attempts=attempts)
            seen.add(path_id);checked=perf_counter()
            failure=self._path_failure(path,obstacles,attachment=attachment,support_names=support_names,
                target_contact=target_contact,initial_proximity=initial_proximity,
                stage=stage,diagnostic_origin="moveit2_final_edge_recheck")
            if self._context_identity(obstacles,attachment=attachment,support_names=support_names,target_contact=target_contact,stage=stage)!=context:
                raise MoveItUnavailable("VALIDATION_CONTEXT_CHANGED")
            attempt["authoritative_s"]=perf_counter()-checked
            attempt["authoritative_status"]="REJECTED" if failure else "PASS";attempt["failure"]=failure
            if failure:
                if raw.get("native_output_status")=="PASS":
                    # A new geometry/policy/sampling disagreement needs diagnosis,
                    # not more random seeds. Preserve the actual candidate + gate.
                    self.native_evidence.extend(attempts)
                    detail=dict(reason="NATIVE_AUTHORITY_MISMATCH",stage=stage,authority_failure=failure,
                        path_sha256=path_id,remaining_attempts=len(schedule)-len(attempts))
                    return [],detail,dict(backend="moveit2",success=False,attempts=attempts)
                continue
            attempt["end_to_end_s"]=perf_counter()-started
            self.native_evidence.extend(attempts);self.native_verified.append(deepcopy(attempt))
            return path,None,dict(backend="moveit2",success=True,validation_level="B_STRICT_LOCAL_CONNECTION",attempts=attempts)
        self.native_evidence.extend(attempts)
        reason=attempts[-1]["status"] if attempts and (goal_pose is not None or attempts[-1]["status"].startswith(("INVALID_START","INVALID_GOAL"))) else "MOVEIT2_SEARCH_EXHAUSTED"
        if goal_pose is None and not allow_ompl and reason == "MOVEIT2_SEARCH_EXHAUSTED":
            reason = "DIRECT_CONNECTION_REJECTED"
        return [],dict(reason=reason,stage=stage,attempts=attempts),dict(backend="moveit2",success=False,attempts=attempts)

    def history_linear_suffix_requests(self,hint,target,contact_q,obstacles):
        """Pure current process geometry; no prefix/path validity work or planning."""
        self._forbid_legacy("history_linear_suffix_requests")
        if not self.budget.proof_of_concept: return []
        from .history_adaptation import loaded_prefix_geometry
        from .layout_trajectory import PhysicalContactAttachment
        from .validation_physics import RigidAttachment
        from .geometry import OBB
        from .release_motion import reception_footprint_audit
        old=hint["segment"];names=old["place"]["support_names"]
        supports=[b for b in obstacles if b.name in names and b.category=="conveyor"]
        if len(supports)!=len(names): return []
        desired=np.asarray(old["place"]["actual_box_pose_world"]).copy()
        box=OBB(desired[:3,3],target.half_extents,desired[:3,:3],target.name,target.category)
        top=max(float(b.corners()[:,2].max()) for b in supports)
        desired[2,3]-=max(0.,float(box.corners()[:,2].min())-top)
        payload=OBB(desired[:3,3],target.half_extents,desired[:3,:3],target.name,target.category)
        reception=reception_footprint_audit(payload,supports,edge_tolerance_m=self.placement_policy.edge_tolerance_m)
        if not reception["supported"]: return []
        physical=self.physical_from_virtual(self.robot.fk(contact_q))
        rigid=RigidAttachment.capture(physical,target)
        attachment=PhysicalContactAttachment(self.robot,rigid,self.flange_from_virtual_task_tcp,self.flange_from_physical_contact)
        a,b=old["stage_ranges"]["extraction"]
        extraction_end=np.asarray(old["path"][b]) if b>a else np.asarray(contact_q)
        prefix,_,_=loaded_prefix_geometry(self,hint,extraction_end,obstacles,attachment,reception["receiver_names"])
        requests=[]
        for height in self.budget.release_policy().ideal_heights():
            released=desired.copy();released[2,3]+=height
            preplace=released@np.linalg.inv(rigid.tcp_from_box)
            clearance=(self.collision_policy.pair_clearance("external",self.collision_margin_m)
                +self.budget.receiver_runtime_clearance_reserve_m+self.contact_tolerance_m+2*float(self.ik["position_tolerance_m"]))
            preplace[2,3]+=max(0.,clearance-height)
            goal=self.virtual_from_physical(preplace)
            requests.append(dict(q_start=np.asarray(prefix[-1]).copy(),goal_pose=goal,
                attachment=attachment,release_height_m=height,receiver_names=reception["receiver_names"],
                receiver_intent_pose_world=desired.copy(),physical_contact_pose_world=physical.copy(),
                contact_q=np.asarray(contact_q).copy(),safe_prefix_nodes=len(prefix)))
        return requests

    def history_capability_check(self,hint,target,contact_q,obstacles):
        self._forbid_legacy("history_capability_check")
        failures=[]
        for request in self.history_linear_suffix_requests(hint,target,contact_q,obstacles):
            failure=self.capability_check(self.robot.fk(request["q_start"]),request["goal_pose"],stage="transit",
                location="after_contact_ik_before_contact_or_prefix_collision")
            if failure is None: return None
            failures.append({**failure,"release_height_m":request["release_height_m"]})
        return {**failures[0],"all_release_alternatives_unsupported":failures} if failures else None

    def _path_failure(self,path,obstacles,**kwargs):
        """Observe the unchanged authority gate at path granularity, not each state."""
        started=perf_counter()
        failure=super()._path_failure(path,obstacles,**kwargs)
        row=dict(stage=kwargs.get("stage"),path_nodes=len(path),seconds=perf_counter()-started,
            reason=None if failure is None else failure.get("reason"),
            diagnostic_origin=kwargs.get("diagnostic_origin"),attached=kwargs.get("attachment") is not None)
        self.authority_path_evidence.append(row)
        trace=os.environ.get("M710_AUTHORITY_TRACE")
        if trace:
            with open(trace,"a",encoding="utf-8") as stream: stream.write(json.dumps(row,allow_nan=False)+"\n")
        return failure

    def _improve_free_path(self,path,obstacles,*,planner=None):
        return path,{"adopted":False,"reason":"PRESERVE_NATIVE_OR_VERIFIED_TEMPLATE_EDGES_AND_TIMING"}

    def _transit(self,start,goal,obstacles,*,purpose,seed,iteration_budget,
                 attachment=None,support_names=(),target_contact=None,stage,
                 candidates=(),allow_rrt=True):
        purpose=require_purpose(purpose,free=True)
        if target_contact is not None or support_names:
            raise ValueError("free motion cannot carry local contact/support permissions")
        if (purpose==MotionPurpose.FREE_LOADED_TRANSFER)!=(attachment is not None):
            raise ValueError("motion purpose and attachment disagree")
        if getattr(self,"require_native_motion",False) and np.array_equal(np.asarray(start),np.asarray(goal)):
            failure=self._path_failure([start],obstacles,attachment=attachment,stage=stage)
            event=dict(stage=stage,purpose=purpose.value,q_rad=np.asarray(start).tolist(),event="ZERO_MOTION_CONNECTION")
            self.native_semantic_events.append(event)
            return [np.asarray(start).copy()],failure,dict(generator="zero_motion_semantic_event",**event,
                rrt_called=False,rrt_constructed=False,rrt_expanded=False,planning_iterations_consumed=0,
                selected_method=GenerationMethod.JOINT_DIRECT.value,validation_completed=failure is None)
        options=dict(seed=seed,attachment=attachment,stage=stage,purpose=purpose)
        path,failure,evidence=self._native_plan(start,goal,obstacles,allow_ompl=False,**options)
        evidence.update(purpose=purpose.value,rrt_called=False,rrt_constructed=False,rrt_expanded=False,
            selected_method=GenerationMethod.JOINT_DIRECT.value if failure is None else None,
            native_ompl_allowed=bool(allow_rrt),planning_iterations_consumed=0)
        if failure is None or not allow_rrt or failure.get("reason")!="DIRECT_CONNECTION_REJECTED":
            return path,failure,evidence
        candidate_evidence=[]
        for method,generate in candidates:
            method=GenerationMethod(method)
            if getattr(self,"native_cold",False) and method==GenerationMethod.HISTORY_HINT:
                self._forbid_legacy("transit_history_candidate")
            if method not in {GenerationMethod.LOCAL_CARTESIAN_CANDIDATE,GenerationMethod.VERIFIED_TEMPLATE,
                               GenerationMethod.HISTORY_HINT}:
                raise ValueError("unsupported native free-space candidate method: "+method.value)
            candidate,candidate_failure,details=generate()
            if candidate_failure is None and candidate:
                if not (np.array_equal(np.asarray(candidate[0]),start) and np.array_equal(np.asarray(candidate[-1]),goal)):
                    candidate_failure=dict(reason="NATIVE_CANDIDATE_ENDPOINT_MISMATCH",stage=stage)
                elif getattr(self,"require_native_motion",False):
                    from .moveit2_native_cold import audit_native_motion_coverage
                    coverage=audit_native_motion_coverage(candidate,self.native_verified,task_id=self.native_task_id)
                    if not coverage["passed"]:
                        candidate_failure=dict(reason="NATIVE_CANDIDATE_SOURCE_GAP",stage=stage,coverage=coverage)
                if candidate_failure is None:
                    candidate_failure=self._path_failure(candidate,obstacles,attachment=attachment,stage=stage)
            candidate_evidence.append(dict(method=method.value,failure=candidate_failure,search=details))
            if candidate and candidate_failure is None:
                return candidate,None,{**evidence,"candidate_attempts":candidate_evidence,"success":True,
                    "selected_method":method.value,"validation_completed":True}
        if iteration_budget<=0:
            return [],dict(reason="NATIVE_CONNECTION_ITERATION_BUDGET_EXHAUSTED",stage=stage),{
                **evidence,"candidate_attempts":candidate_evidence}
        path,failure,fallback=self._native_plan(start,goal,obstacles,allow_ompl=True,try_ptp=False,
            max_ompl_attempts=min(int(iteration_budget),self.budget.stage_connection_attempts),**options)
        consumed=len(fallback.get("attempts",[]))
        fallback.update(purpose=purpose.value,candidate_attempts=candidate_evidence,
            direct_attempts=evidence.get("attempts",[]),rrt_called=False,rrt_constructed=False,rrt_expanded=False,
            native_ompl_allowed=True,native_ompl_calls=consumed,planning_iterations_consumed=consumed,
            selected_method=GenerationMethod.RRT_CONNECT.value if failure is None else None)
        return path,failure,fallback

    def _cartesian(self,start,destination,obstacles,*,purpose,**kwargs):
        purpose=require_purpose(purpose)
        strict=getattr(self,"require_native_motion",False)
        if strict:
            origin=self.robot.fk(start)
            if np.array_equal(origin,np.asarray(destination)):
                failure=self._path_failure([start],obstacles,**{key:value for key,value in kwargs.items() if key!="seed"})
                event=dict(stage=kwargs["stage"],purpose=purpose.value,q_rad=np.asarray(start).tolist(),
                    event="ZERO_MOTION_PROCESS",failure=failure)
                self.native_semantic_events.append(event)
                return [np.asarray(start).copy()],failure,dict(generator="zero_motion_semantic_event",**event,
                    cartesian_samples=0,along_path_ik_calls=0,planning_iterations_consumed=0,
                    selected_method=GenerationMethod.PROCESS_WAYPOINT_CANDIDATE.value,
                    rrt_called=False,rrt_constructed=False,rrt_expanded=False,validation_completed=failure is None)
            path,failure,evidence=self._native_plan(start,None,obstacles,goal_pose=destination,
                purpose=purpose,allow_ompl=False,**kwargs)
            count=max(0,len(path)-1)
            self._statistics["cartesian_samples"]+=count
            evidence.update(generator="native_MTC_Pilz_LIN",purpose=purpose.value,
                selected_method=(GenerationMethod.LOCAL_CARTESIAN_CANDIDATE.value if purpose in
                    (MotionPurpose.FREE_APPROACH,MotionPurpose.FREE_LOADED_TRANSFER) else
                    GenerationMethod.PROCESS_WAYPOINT_CANDIDATE.value),
                rrt_called=False,rrt_constructed=False,rrt_expanded=False,cartesian_samples=count,
                along_path_ik_calls=0,native_pipeline_internal_ik_calls_observable=False,
                planning_iterations_consumed=0,validation_completed=failure is None)
            return path,failure,evidence
        # Bounded contact/history semantics cannot be represented by an ACM.
        if (kwargs.get("initial_proximity") is not None or kwargs.get("target_contact") is not None
                or kwargs.get("support_names") or kwargs["stage"] in {"contact","support-release","extraction"}):
            path,failure,evidence=super()._cartesian(start,destination,obstacles,purpose=purpose,**kwargs)
            return path,failure,{**evidence,"generator":"existing_core_contact_process"}
        kwargs.pop("initial_proximity",None)
        return self._native_plan(start,None,obstacles,goal_pose=destination,purpose=purpose,**kwargs)

    def _finalize_task_checked(self,segment,obstacles,target):
        from .moveit2_timing import native_timing_floor
        from .layout_trajectory import TRAJECTORY_STAGES
        floors, records = native_timing_floor(segment["path"], self.native_verified)
        if not records:
            return {"reason":"NO_NATIVE_STAGE_IN_COMPLETE_TASK","stage":"final_validation"}
        segment["native_backend"]={"schema":SCHEMA,"name":"moveit2", "startup":self.native_startup,
            "rest_start_contract":deepcopy(self.native_rest_start),
            "fk_checks":self.native_fk,"stages":records,"minimum_edge_seconds":floors,
            "joint_names":self.native_joint_names,"policy_scope":self.native_identity["policy_scope"],
            "execution_timing":"native_edge_duration_floor_then_existing_C2_quintic_audit",
            "time_law_change_reason":"Existing Isaac controller consumes stopped C2 joint edges, not ROS spline derivatives; all native timestamps retained and durations never shortened"}
        if getattr(self,"require_native_motion",False):
            from .moveit2_native_cold import audit_native_motion_coverage, verify_native_cold_segment
            coverage=audit_native_motion_coverage(segment["path"],records,task_id=self.native_task_id)
            if not coverage["passed"]:
                return dict(reason="NATIVE_COLD_MOTION_SOURCE_GAP",stage="final_validation",coverage=coverage)
            segment.update(native_cold=True,require_native_motion=True)
            cold_audit=self.native_cold_evidence()
            cold_audit["zero_length_semantic_events"].extend(
                dict(stage=name,path_index=a,event="ZERO_LENGTH_STAGE")
                for name,(a,b) in segment["stage_ranges"].items() if a==b)
            cold_audit["zero_length_semantic_events"].extend(deepcopy(segment["events"]))
            _,timeout=self._native_limits()
            task=self.native.request(dict(op="task_audit",identity=self.native_identity,
                task_id=self.native_task_id,stage_ids=[r["stage_id"] for r in coverage["records"]],
                events=segment["events"],stage_ranges=segment["stage_ranges"],target_id=target.name),
                timeout=timeout,cancelled=self._native_cancelled)
            segment["native_backend"].update(native_cold=True,require_native_motion=True,
                task_id=self.native_task_id,cold_audit=cold_audit,source_coverage=coverage,
                mtc_task_audit=task,stages=coverage["records"])
            failure=verify_native_cold_segment(segment)
            if failure is not None:
                return failure
            return super()._finalize_task_checked(segment,obstacles,target)
        path=np.asarray(segment["path"])
        specs=[]
        for name in TRAJECTORY_STAGES:
            if name=="home" or name not in segment["stage_ranges"]: continue
            a,b=segment["stage_ranges"][name]
            spec=dict(name=name,path=path[a:b+1].tolist(),durations=[max(.002,x) for x in floors[a:b]],
                generator="checked_mixed_core_and_native_stage")
            if name=="contact":
                flange=self.robot.named_link_frames(path[b])["flange"]
                attached=box_message(target,np.linalg.inv(flange)@target.world_from_local)
                attached["touch_links"]=sorted(self.native_compliant);spec["attach"]=attached
            if name=="place":
                spec["release"]=box_message(target,segment["place"]["actual_box_pose_world"])
            specs.append(spec)
        world=[box_message(b) for b in obstacles]
        if target.name not in {b["id"] for b in world}: world.append(box_message(target))
        composition=self.native.request(dict(op="compose",identity=self.native_identity,q_start=path[0].tolist(),
            world=world,scene_fingerprint=digest(world),allowed_pairs=[["base_link",self.robot_state_validator.base_support_obstacle_name]],
            attachment=None,start_velocity=[0.]*6,path_constraints={},stages=specs))
        if composition["status"]!="SUCCESS":
            return {"reason":"MTC_COMPOSITION_REJECTED","stage":"final_validation","detail":composition}
        segment["native_backend"]["mtc_composition"]=composition
        return super()._finalize_task_checked(segment,obstacles,target)
