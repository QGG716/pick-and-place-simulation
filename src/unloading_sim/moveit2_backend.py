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
import xml.etree.ElementTree as ET

import numpy as np

from .layout_trajectory import LayoutTrajectoryConnector, OFFICIAL_MODEL_URDF, OFFICIAL_MODEL_SRDF

SCHEMA = "m710_native_stage_v1"
JOINT_NAMES = [f"J{i}" for i in range(1, 7)]


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def box_message(box, pose=None):
    return dict(id=box.name, size=(2*np.asarray(box.half_extents)).tolist(),
                pose=np.asarray(box.world_from_local if pose is None else pose).tolist(),
                category=box.category, geometry_source="frozen_layout_or_audited_tool_OBB")


class MoveItUnavailable(RuntimeError):
    pass


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


def model_request(connector, scene, *, asset_root=None, seed=71070):
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
    tool = list(connector.tool_collision_obbs_provider(q0))
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
    # Owned tool solids form one assembly. Wrist exceptions retain exact ownership.
    for first, second in itertools.combinations(tool_names, 2):
        ET.SubElement(srdf, "disable_collisions", link1=first, link2=second, reason="Owned_tool_assembly")
    for first, second in connector.collision_policy.wrist_tool_pairs(tool_names):
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
    identity = dict(schema=SCHEMA, baseline_sha="4f6e3037aceb47a525711e58360d98c8751fec6b",
        scene_version="m710id70_unloading_layout_v1", scene_fingerprint=scene.snapshot.get("scene_fingerprint"),
        policy_fingerprint=scene.policy.policy_fingerprint, validator_identity=connector.validator_identity,
        model_tool_fingerprint=digest(params), policy_scope="native_intersection_candidates_plus_authoritative_pair_gap_contact_checks")
    return dict(op="init", parameters=params, identity=identity, joint_names=names, seed=seed), tool_names, compliant


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


class MoveItLayoutConnector(LayoutTrajectoryConnector):
    @classmethod
    def from_existing(cls, connector, scene, command=None):
        self = cls.__new__(cls); self.__dict__.update(connector.__dict__)
        measured_velocity=scene.snapshot.get("robot",{}).get("qd_rad_s")
        if measured_velocity is None and scene.snapshot.get("actual_state_context"):
            raise MoveItUnavailable("UNSUPPORTED_MISSING_START_VELOCITY")
        if measured_velocity is not None:
            velocity=np.asarray(measured_velocity,dtype=float)
            if velocity.shape!=(6,) or not np.isfinite(velocity).all() or np.any(velocity!=0):
                raise MoveItUnavailable("UNSUPPORTED_NONZERO_OR_INVALID_START_VELOCITY")
        self.native = ResidentMoveItClient(command or os.environ.get("M710_MOVEIT_COMMAND"),
            log_path=os.environ.get("M710_MOVEIT_LOG"))
        init, self.native_tools, self.native_compliant = model_request(self, scene,
            asset_root=os.environ.get("M710_MOVEIT_ASSET_ROOT"), seed=int(os.environ.get("M710_MOVEIT_SEED", "71070")))
        self.native_identity=init["identity"]; self.native_joint_names=init["joint_names"]
        self.native_stage_seconds=float(os.environ.get("M710_MOVEIT_STAGE_SECONDS", "12"))
        if not np.isfinite(self.native_stage_seconds) or self.native_stage_seconds<=0:
            self.native.close(); raise ValueError("invalid native stage time budget")
        self.native_scene=scene; self.native_evidence=[]; self.native_verified=[]
        try:
            self.native_startup=self.native.request(init)
            self.check_fk()
        except Exception:
            self.native.close(); raise
        return self

    def check_fk(self):
        checks=[]
        for q in (np.zeros(6), np.array([.2,-.3,.4,.5,-.6,.7]), np.array([-.4,.2,-.35,-.6,.4,-.8])):
            links=["base_link", "J3_link", "J6_link", "flange", "fanuc_flange", "tool0"]
            reference=self.robot.named_link_frames(q)
            links=[l for l in links if l in reference]
            result=self.native.request(dict(op="fk",identity=self.native_identity,q_start=q.tolist(),links=links))
            error=max(float(np.max(np.abs(np.asarray(result["frames"][l])-reference[l]))) for l in links)
            tcp=np.asarray(result["frames"]["flange"]) @ self.flange_from_virtual_task_tcp
            error=max(error,float(np.max(np.abs(tcp-self.robot.fk(q)))))
            checks.append(dict(q=q.tolist(),max_matrix_error=error,links=links))
            if error>1e-8: raise MoveItUnavailable(f"MODEL_FK_MISMATCH: {error}")
        self.native_fk=checks

    def _native_plan(self,start,goal,obstacles,*,seed,attachment=None,support_names=(),target_contact=None,stage,
                     goal_pose=None):
        started=perf_counter(); attempts=[]; seen=set()
        if support_names or self.collision_policy.allows_stack_planning_contact(stage):
            return [],dict(reason="UNSUPPORTED_POLICY_STAGE",stage=stage),dict(backend="moveit2",attempts=[])
        world=[box_message(b) for b in obstacles]
        if len({b["id"] for b in world})!=len(world): raise ValueError("DUPLICATE_OBJECT")
        pairs=[["base_link", self.robot_state_validator.base_support_obstacle_name]]
        target_name=attachment.rigid.name if attachment is not None else (target_contact.name if target_contact is not None else None)
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
        if goal_pose is not None:
            origin=self.robot.fk(start)
            if not np.allclose(origin[:3,:3],np.asarray(goal_pose)[:3,:3],atol=1e-8,rtol=0):
                return [],dict(reason="UNSUPPORTED_ROTATING_TASK_TCP_LIN",stage=stage),dict(backend="moveit2")
        request=dict(schema=SCHEMA,op="plan",identity=self.native_identity,q_start=np.asarray(start).tolist(),
            world=world,scene_fingerprint=digest(world),allowed_pairs=pairs,attachment=attached,stage=stage,
            candidate_id=getattr(self,"_candidate_identity",None) or f"directed:{stage}:{seed}",start_velocity=[0.]*6,path_constraints={},
            seed=int(seed),cancelled=False,allowed_planning_time_s=getattr(self,"native_stage_seconds",12.),velocity_scale=.2,acceleration_scale=.2,
            flange_from_task_tcp=self.flange_from_virtual_task_tcp.tolist())
        if goal_pose is None: request["q_goal"]=np.asarray(goal).tolist()
        else: request["goal_pose"]=np.asarray(goal_pose).tolist()
        # No free-space fallback for constrained linear process motion.
        schedule=[("pilz_industrial_motion_planner","LIN")] if goal_pose is not None else [
            ("pilz_industrial_motion_planner","PTP"),*( [("ompl","RRTConnectkConfigDefault")]*self.budget.stage_connection_attempts)]
        context=self._context_identity(obstacles,attachment=attachment,support_names=support_names,target_contact=target_contact,stage=stage)
        for pipeline,planner in schedule:
            if self._deadline_reached(): break
            submitted={**request,"pipeline_id":pipeline,"planner_id":planner}
            request_log=os.environ.get("M710_MOVEIT_REQUEST_LOG")
            if request_log:
                with open(request_log,"a",encoding="utf-8") as stream: stream.write(json.dumps(submitted,allow_nan=False)+"\n")
            raw=self.native.request(submitted,timeout=request["allowed_planning_time_s"]+30.,cancelled=self._deadline_reached)
            attempt={**raw,"stage":stage,"seed":seed,"request_fingerprint":digest(request)}
            attempts.append(attempt)
            if raw["status"]!="SUCCESS":
                if raw["status"].startswith("INVALID_START"): break
                continue
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
            seen.add(path_id);checked=perf_counter()
            failure=self._path_failure(path,obstacles,attachment=attachment,support_names=support_names,
                target_contact=target_contact,stage=stage,diagnostic_origin="moveit2_final_edge_recheck")
            if self._context_identity(obstacles,attachment=attachment,support_names=support_names,target_contact=target_contact,stage=stage)!=context:
                raise MoveItUnavailable("VALIDATION_CONTEXT_CHANGED")
            attempt["authoritative_s"]=perf_counter()-checked
            attempt["authoritative_status"]="REJECTED" if failure else "PASS";attempt["failure"]=failure
            if failure: continue
            if goal_pose is not None:
                from .ik import pose_error
                origin=self.robot.fk(start);direction=np.asarray(goal_pose)[:3,3]-origin[:3,3]
                length2=float(direction@direction)
                maximum_position=maximum_angle=0.
                for a,b in zip(path[:-1],path[1:]):
                    for fraction in (0.,.5,1.):
                        actual=self.robot.fk(a+fraction*(b-a))
                        along=0. if length2<1e-20 else float(np.clip((actual[:3,3]-origin[:3,3])@direction/length2,0.,1.))
                        expected=origin.copy();expected[:3,3]+=along*direction
                        _,distance,angle=pose_error(actual,expected)
                        maximum_position=max(maximum_position,distance);maximum_angle=max(maximum_angle,angle)
                _,end_distance,end_angle=pose_error(self.robot.fk(path[-1]),np.asarray(goal_pose))
                attempt["lin_constraint_audit"]=dict(maximum_line_error_m=maximum_position,maximum_orientation_error_rad=maximum_angle,
                    endpoint_error_m=end_distance,endpoint_error_rad=end_angle,sampling="knots_and_each_joint_edge_midpoint")
                if (max(maximum_position,end_distance)>float(self.ik["position_tolerance_m"]) or
                    max(maximum_angle,end_angle)>float(self.ik["orientation_tolerance_rad"])):
                    attempt["authoritative_status"]="REJECTED";attempt["failure"]={"reason":"LIN_TASK_TCP_CONSTRAINT","stage":stage};continue
            attempt["end_to_end_s"]=perf_counter()-started
            self.native_evidence.extend(attempts);self.native_verified.append(deepcopy(attempt))
            return path,None,dict(backend="moveit2",success=True,validation_level="B_STRICT_LOCAL_CONNECTION",attempts=attempts)
        self.native_evidence.extend(attempts)
        return [],dict(reason="MOVEIT2_SEARCH_EXHAUSTED",stage=stage,attempts=attempts),dict(backend="moveit2",success=False,attempts=attempts)

    def _improve_free_path(self,path,obstacles,*,planner=None):
        return path,{"adopted":False,"reason":"PRESERVE_NATIVE_OR_VERIFIED_TEMPLATE_EDGES_AND_TIMING"}

    def _transit(self,start,goal,obstacles,*,iteration_budget,**kwargs):
        return self._native_plan(start,goal,obstacles,**kwargs)

    def _cartesian(self,start,destination,obstacles,**kwargs):
        # Bounded contact/history semantics cannot be represented by an ACM.
        if (kwargs.get("initial_proximity") is not None or kwargs.get("target_contact") is not None
                or kwargs.get("support_names") or kwargs["stage"] in {"contact","support-release","extraction"}):
            path,failure,evidence=super()._cartesian(start,destination,obstacles,**kwargs)
            return path,failure,{**evidence,"generator":"existing_core_contact_process"}
        kwargs.pop("initial_proximity",None)
        return self._native_plan(start,None,obstacles,goal_pose=destination,**kwargs)

    def _finalize_task_checked(self,segment,obstacles,target):
        from .moveit2_timing import native_timing_floor
        from .layout_trajectory import TRAJECTORY_STAGES
        floors, records = native_timing_floor(segment["path"], self.native_verified)
        if not records:
            return {"reason":"NO_NATIVE_STAGE_IN_COMPLETE_TASK","stage":"final_validation"}
        segment["native_backend"]={"schema":SCHEMA,"name":"moveit2", "startup":self.native_startup,
            "fk_checks":self.native_fk,"stages":records,"minimum_edge_seconds":floors,
            "joint_names":self.native_joint_names,"policy_scope":self.native_identity["policy_scope"],
            "execution_timing":"native_edge_duration_floor_then_existing_C2_quintic_audit",
            "time_law_change_reason":"Existing Isaac controller consumes stopped C2 joint edges, not ROS spline derivatives; all native timestamps retained and durations never shortened"}
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
