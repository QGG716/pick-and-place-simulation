"""Deterministic complete-cycle planner for the V3 acceptance contract."""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import itertools

import numpy as np

from .depalletizing import (LConveyorGeometry, analyze_box_neighborhood,
                           generate_extraction_candidates, minimum_clearance_extraction_distance)
from .fanuc_m710id70 import target_pose, trailer_obstacles
from .geometry import (OBB, make_transform, rotation_matrix_from_rotation_vector,
                       rotation_matrix_from_rpy, rotation_vector_from_matrix)
from .ik import solve_ik_multistart, pose_error
from .planner import RRTConnectPlanner
from .support import SupportRelationGraph
from .timing import time_parameterize_joint_path
from .validation_physics import (ContactState, InitialProximityTracker, RigidAttachment,
                                 contact_separated, external_load, suction_coverage,
                                 support_audit, urdf_collision_shapes, world_link_boxes)


def transformed(box: OBB, pose: np.ndarray) -> OBB:
    world = pose @ box.world_from_local
    return OBB(world[:3,3], box.half_extents, world[:3,:3], box.name, box.category)


def grasp_task_set(nominal_pose, face_offsets_m, tilt_candidates_rad):
    """Return an ordered, deterministic local task set around one face pose.

    Face translations and small tilts are deliberately sparse (a cross, not a
    Cartesian product).  Every returned target still has to pass complete-cup
    coverage and strict actual-FK validation in ``evaluate_task``.
    """
    nominal=np.asarray(nominal_pose,float)
    variants=[('nominal',0.0,0.0,0.0,0.0)]
    # Interleave the two face axes at each radius so a bounded downstream
    # budget does not privilege one direction.
    variants.extend(item for value in face_offsets_m if value for item in (
        (f'offset_local_x_{value:+g}',value,0.0,0.0,0.0),
        (f'offset_local_y_{value:+g}',0.0,value,0.0,0.0)))
    variants.extend((f'tilt_local_x_{value:+g}',0.0,0.0,value,0.0)
                    for value in tilt_candidates_rad if value)
    variants.extend((f'tilt_local_y_{value:+g}',0.0,0.0,0.0,value)
                    for value in tilt_candidates_rad if value)
    result=[]
    for name,x,y,rx,ry in variants:
        pose=nominal.copy()
        pose[:3,3]+=nominal[:3,:2]@np.array([x,y])
        pose[:3,:3]=nominal[:3,:3]@rotation_matrix_from_rpy(rx,ry,0.0)
        result.append((pose,{'variant':name,'face_offset_local_xy_m':[x,y],
                            'orientation_offset_local_xy_rad':[rx,ry]}))
    return result


def grasp_seed_configurations(robot, seeds):
    """Add valid spherical-wrist flip seeds without accepting them as solutions."""
    result=[]
    for original in seeds:
        q=np.asarray(original,float)
        result.append(q)
        if robot.dof != 6:
            continue
        for shift in (-np.pi,np.pi):
            flipped=q.copy()
            flipped[3]+=shift
            flipped[4]*=-1
            flipped[5]+=shift
            if robot.within_limits(flipped):
                result.append(flipped)
    return list({tuple(q):q for q in result}.values())


def residual_failure_detail(ik, planning):
    position=ik.position_error>planning['ik_position_tolerance_m']
    orientation=ik.orientation_error>planning['ik_orientation_tolerance_rad']
    if position and orientation:return 'POSITION_AND_ORIENTATION_RESIDUAL_NOT_CONVERGED'
    if position:return 'POSITION_RESIDUAL_NOT_CONVERGED'
    if orientation:return 'ORIENTATION_RESIDUAL_NOT_CONVERGED'
    return 'STRICT_POSE_REACHED_BUT_VALIDITY_UNRESOLVED'


def escape_path_proposals(target, pure_direction, constraints, pure_distance, planning):
    """Yield deterministic escape candidates across stations and directions.

    Geometry generation is separate from expensive robot validation. Base
    station/direction pairs are scheduled for novelty before any pair receives
    a second rotation, so a small global budget is not consumed by one station
    or one direction. Path length is evidence and a ranking input only; a
    longer-than-straight detour remains a valid candidate.
    """
    pure=np.asarray(pure_direction,float);pure/=np.linalg.norm(pure)
    directions=[('lift',np.array([0.,0.,1.])),('left',np.array([0.,1.,0.])),
                ('right',np.array([0.,-1.,0.])),
                ('diagonal_lift',pure+np.array([0.,0.,1.])),
                ('diagonal_left',pure+np.array([0.,1.,0.])),
                ('diagonal_right',pure+np.array([0.,-1.,0.]))]
    directions=[(name,direction/np.linalg.norm(direction)) for name,direction in directions
                if abs(float(direction@pure))/np.linalg.norm(direction)<1-1e-9]
    step=planning['extraction_scan_step_m']
    stations=list(np.arange(0.0,pure_distance+step/2,step))
    if not stations or stations[-1]<pure_distance-1e-12:
        stations.append(float(pure_distance))
    # Start each direction at a new station, then promptly backfill that new
    # direction at nearer stations. This makes the first four valid proposals
    # cover multiple stations and directions while retaining the important
    # station-zero alternatives. A Latin traversal supplies every remaining
    # pair. Geometry is lazy and cached across later rotation refinement.
    def pair_order():
        emitted=set()
        for direction_index in range(len(directions)):
            anchor=min(direction_index,len(stations)-1)
            for station_index in range(anchor,-1,-1):
                key=(station_index,direction_index)
                if key not in emitted:
                    emitted.add(key);yield key
        for direction_offset in range(len(directions)):
            for station_index in range(len(stations)):
                direction_index=(station_index+direction_offset)%len(directions)
                key=(station_index,direction_index)
                if key not in emitted:
                    emitted.add(key);yield key

    geometry_cache={};geometry_queries=0;schedule_index=0
    for rotation_index,rotation in enumerate(planning['escape_rotation_candidates_rad']):
        for station_index,direction_index in pair_order():
            constrained=stations[station_index];name,direction=directions[direction_index]
            key=(station_index,direction_index)
            if key not in geometry_cache:
                moved=OBB(target.center+pure*constrained,target.half_extents,target.rotation,
                          target.name,target.category)
                geometry_cache[key]=minimum_clearance_extraction_distance(moved,direction,constraints,
                    free_space_clearance_m=planning['extraction_free_clearance_m'],
                    scan_step_m=step,maximum_distance_m=planning['maximum_extraction_m'])
                geometry_queries+=1
            local=geometry_cache[key]
            if local is None:
                continue
            yield {'station_index':station_index,'direction_index':direction_index,
                'constrained_straight_distance_m':float(constrained),
                'escape_direction':name,'escape_direction_world':direction.tolist(),
                'escape_translation_m':float(local),
                'geometric_path_length_m':float(constrained+local),
                'longer_than_pure_straight':bool(constrained+local>pure_distance+1e-12),
                'escape_rotation_world_z_rad':float(rotation),'rotation_index':rotation_index,
                'schedule_index':schedule_index,'geometry_queries_so_far':geometry_queries,
                'scheduler':'station_direction_novelty_then_rotation_v2'}
            schedule_index+=1


def support_relations(target, cartons, fixtures, planning):
    """Return explicit graph/floor supporters for the target carton."""
    graph=SupportRelationGraph.build(cartons,
        contact_tolerance_m=planning['support_relation_tolerance_m'],
        minimum_overlap_ratio=planning['support_relation_minimum_overlap_ratio'])
    names=sorted(graph.supported_by[target.name])
    floor=next((box for box in fixtures if box.name=='floor'),None)
    if floor is not None:
        gap=float(target.corners()[:,2].min()-(floor.center[2]+floor.half_extents[2]))
        if -planning['support_tolerance_m']<=gap<=planning['support_relation_tolerance_m']:
            names.append('floor')
    return list(dict.fromkeys(names)),graph.audit()


class Cell:
    def __init__(self, config, height=None):
        self.config = config
        self.d = config.data
        self.p = self.d["planning"]
        self.robot = config.robot(height)
        self.shapes = urdf_collision_shapes(self.robot)
        self._bounds_cache = {}
        self.support_release_events = []
        self.search_events = {"ik": [], "connection": []}
        s, c = self.d["scene"], self.d["conveyor"]
        self.walls = trailer_obstacles(s["trailer_width_m"], s["trailer_height_m"], s["trailer_x_limits_m"], s["wall_thickness_m"])
        self.chassis_pose = config.world_from_chassis()
        self.chassis = OBB(self.chassis_pose[:3,3], np.asarray(self.d["chassis"]["size_xyz_m"])/2,
                           self.chassis_pose[:3,:3], "chassis", "chassis")
        # The conveyor is fixed to its own chassis-aligned frame; lift motion
        # of the robot mount cannot translate either the chassis or conveyor.
        self.geometry = LConveyorGeometry(
            chassis_size_xyz_m=tuple(self.d["chassis"]["size_xyz_m"]),
            chassis_center_from_robot_base_xyz_m=(0,0,0),
            **{key: c[key] for key in LConveyorGeometry.__dataclass_fields__ if key in c})

    def decks(self, state):
        e, world_z = state
        local_z = world_z - self.chassis_pose[2,3]
        return tuple(transformed(box, self.chassis_pose) for box in self.geometry.obstacles([0,0,0], e, local_z))

    def fixtures(self):
        top = self.chassis.center[2] + self.chassis.half_extents[2]
        base = self.robot.base_transform[:3,3]
        height = base[2] - top
        column = [] if height <= 0 else [OBB(np.array([base[0],base[1],top+height/2]),
            np.r_[np.asarray(self.d["lift"]["column_size_xy_m"])/2,height/2], self.chassis.rotation, "lift_column", "mount")]
        return [*self.walls,self.chassis,*column]

    def broadphase(self, moving, obstacles, margin):
        """Conservative world-AABB rejection before the unchanged OBB SAT.

        Both boxes retain the same local-axis inflation as intersects_obb.
        The extra 1e-7 m numerical slack is larger than SAT's 1e-9 axis
        padding at this cell's metre scale; touching cases always reach SAT.
        Cache holds object references, so reused Python ids cannot alias data.
        """
        if not obstacles:
            return []
        key=(tuple(id(box) for box in obstacles),float(margin))
        cached=self._bounds_cache.get(key)
        if cached is None:
            centers=np.asarray([box.center for box in obstacles])
            radii=np.asarray([np.abs(box.rotation)@np.maximum(box.half_extents+margin,0) for box in obstacles])
            cached=(tuple(obstacles),centers-radii,centers+radii)
            if len(self._bounds_cache)>=32:self._bounds_cache.clear()
            self._bounds_cache[key]=cached
        refs,lower,upper=cached
        radius=np.abs(moving.rotation)@np.maximum(moving.half_extents+margin,0)
        overlap=np.all(lower<=moving.center+radius+1e-7,axis=1)&np.all(upper>=moving.center-radius-1e-7,axis=1)
        return [refs[i] for i in np.flatnonzero(overlap)]

    def state_failure(self, q, obstacles, attachment=None, support_names=(), target_contact=None,
                      initial_proximity=None):
        r,p = self.robot,self.p
        q = np.asarray(q,float)
        if q.shape != (r.dof,) or not np.all(np.isfinite(q)) or not r.within_limits(q):
            return {"reason":"JOINT_LIMIT"}
        if np.min(np.minimum(q-r.joint_limits[:,0],r.joint_limits[:,1]-q)) < p["joint_margin_rad"]:
            return {"reason":"JOINT_MARGIN"}
        frames = r.named_link_frames(q)
        local_flange = r.base_transform[:3,:3].T @ (frames["flange"][:3,3]-r.base_transform[:3,3])
        if np.linalg.norm(local_flange[:2]) > self.config.model["reach_m"] + p["radial_guard_tolerance_m"]:
            return {"reason":"RADIAL_REACH"}
        if np.linalg.cond(r.geometric_jacobian(q)) > p["maximum_jacobian_condition"]:
            return {"reason":"SINGULARITY"}
        # Preserve existing capsule self checks and add previously omitted
        # tool-vs-arm and URDF primitive-vs-environment checks.
        self_hit = r.collision_result(q,[],check_self=True)
        if self_hit.in_collision:
            return {"reason":"SELF_COLLISION", "detail":str(self_hit)}
        links = world_link_boxes(r,q,self.shapes)
        tool = r.tool_collision_obb(q)
        for a in links:
            for b in self.broadphase(a,obstacles,p["collision_margin_m"]):
                if a.name in {"base_link", "J1_link"} and b.name == "lift_column" and contact_separated(a,b,p["contact_tolerance_m"]):
                    continue  # nonpenetrating mounting-plane contact only
                if a.intersects_obb(b, margin=p["collision_margin_m"]):
                    return {"reason":"ROBOT_COLLISION","pair":[a.name,b.name]}
        if tool is not None:
            for b in self.broadphase(tool,obstacles,p["collision_margin_m"]):
                if target_contact is not None and b.name == target_contact.name:
                    # Only the suction working plane is permitted to touch.
                    # Rigid tool remains on the negative TCP-Z side.
                    local = (b.corners()-r.fk(q)[:3,3]) @ r.fk(q)[:3,:3]
                    if np.min(local[:,2]) >= -p["contact_tolerance_m"]:
                        continue
                if tool.intersects_obb(b,margin=p["collision_margin_m"]):
                    return {"reason":"TOOL_COLLISION","pair":[tool.name,b.name]}
            for a in self.broadphase(tool,links,p["collision_margin_m"]):
                if a.name != "J6_link" and tool.intersects_obb(a,margin=p["collision_margin_m"]):
                    return {"reason":"TOOL_SELF_COLLISION","pair":[tool.name,a.name]}
        if attachment is not None:
            box = attachment.box_at(r.fk(q))
            if initial_proximity is not None:
                failure = initial_proximity.state_failure(box, obstacles, support_names)
                if failure:
                    return failure
            else:
                for b in self.broadphase(box,obstacles,p["collision_margin_m"]):
                    if b.name in support_names and contact_separated(box,b,p["support_tolerance_m"]):
                        continue
                    if box.intersects_obb(b,margin=p["collision_margin_m"]):
                        return {"reason":"PAYLOAD_COLLISION","pair":[box.name,b.name]}
            for a in self.broadphase(box,links,p["collision_margin_m"]):
                if box.intersects_obb(a,margin=p["collision_margin_m"]):
                    return {"reason":"PAYLOAD_ROBOT_COLLISION","pair":[box.name,a.name]}
        return None

    def path_failure(self, path, obstacles, attachment=None, support_names=(), target_contact=None,
                     initial_proximity=None):
        count = 0
        for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
            n = max(1,int(np.ceil(np.max(np.abs(b-a))/self.p["edge_resolution_rad"])))
            # Midpoints supplement all endpoint samples, including the start.
            for u in np.linspace(0,1,2*n+1):
                count += 1
                failure = self.state_failure(a+u*(b-a),obstacles,attachment,support_names,
                                             target_contact,initial_proximity)
                if failure:
                    return {**failure,"edge":index,"fraction":float(u),"q_rad":(a+u*(b-a)).tolist()}
        if len(path)==1:
            return self.state_failure(path[0],obstacles,attachment,support_names,
                                      target_contact,initial_proximity)
        return None

    def solve(self, pose, seeds, rng_seed, valid=None, stage="unspecified"):
        p = self.p
        unique = list({tuple(np.asarray(q,float)): np.asarray(q,float) for q in seeds}.values())
        result=solve_ik_multistart(self.robot,pose,unique,random_restarts=p["ik_restarts"],rng=np.random.default_rng(rng_seed),
            max_iterations=p["ik_iterations"],damping=p["ik_damping"],max_step=p["ik_max_step_rad"],
            position_tolerance=p["ik_position_tolerance_m"],orientation_tolerance=p["ik_orientation_tolerance_rad"],
            orientation_weight=p["ik_orientation_weight"],extra_state_valid=valid,
            collision_check_stride=p["ik_iterations"]+1)
        self.search_events["ik"].append({"stage":stage,"rng_seed":int(rng_seed),
                                          **result.search_evidence})
        return result

    def validate_contact_endpoint(self,q,target,face,obstacles):
        """Close the contact contract at the actual endpoint before attaching."""
        q=np.asarray(q,float);actual_tcp=self.robot.fk(q)
        coverage=suction_coverage(actual_tcp,target,face,self.d["tool"],self.p["contact_tolerance_m"])
        if not coverage["geometric_coverage"]:
            return None,{"reason":"FINAL_CONTACT_COVERAGE_FAILED","coverage":coverage,
                         "q_rad":q.tolist(),"actual_tcp_world":actual_tcp.tolist()}
        collision=self.state_failure(q,obstacles,target_contact=target)
        if collision:
            return None,{"reason":"FINAL_CONTACT_COLLISION_FAILED","collision":collision,
                         "coverage":coverage,"q_rad":q.tolist(),
                         "actual_tcp_world":actual_tcp.tolist()}
        # Creation happens only after every endpoint check above has passed.
        attachment=RigidAttachment.capture(actual_tcp,target)
        state=ContactState(q.copy(),actual_tcp.copy(),target.world_from_local.copy(),face,
                           coverage,{"valid":True,"target_contact_rule":True},attachment,
                           self.config.asset_manifest["semantic_fingerprint_sha256"])
        if state.evidence()["attachment_pose_continuity_max_abs"]>1e-12:
            raise RuntimeError("validated contact attachment is not pose-continuous")
        return state,None

    def transit(self,start,goal,obstacles,seed,attachment=None,support_names=(),target_contact=None,
                stage="transit"):
        state = lambda q: self.state_failure(q,obstacles,attachment,support_names,target_contact) is None
        planner = RRTConnectPlanner(self.robot.joint_limits[:,0],self.robot.joint_limits[:,1],state,
            step_size=self.p["rrt_step_rad"],edge_resolution=self.p["edge_resolution_rad"]/2,
            max_iterations=self.p["rrt_iterations"],goal_bias=self.p["rrt_goal_bias"],rng=np.random.default_rng(seed))
        result = planner.plan(start,goal)
        self.search_events["connection"].append({"stage":stage,"rng_seed":int(seed),
                                                  "success":result.success,
                                                  **result.search_evidence})
        if not result.success:
            failure = self.state_failure(start,obstacles,attachment,support_names,target_contact) or self.state_failure(goal,obstacles,attachment,support_names,target_contact)
            return [], failure or {"reason":"PATH_SEARCH_EXHAUSTED","detail":result.message,"iterations":result.iterations}
        failure = self.path_failure(result.path,obstacles,attachment,support_names,target_contact)
        return result.path, failure

    def search_statistics(self, record):
        """Summarize all observed search work for one task, not only its winner."""
        ik_events=list(self.search_events["ik"])
        connection_events=list(self.search_events["connection"])
        candidate_failures=Counter()
        escape_validations=0
        for attempt in record["attempts"]:
            for option in attempt.get("conveyor_attempts",[]):
                escape_validations+=len(option.get("escape_attempts",[]))
                if option.get("stage") in {"approach","contact","handoff","carry","place","withdrawal"} \
                        and option.get("reason")!="OK":
                    candidate_failures[option["stage"]]+=1
        terminations=Counter()
        for event in ik_events:
            if event.get("termination") in {"SEED_STREAM_EXHAUSTED"}:
                terminations[f"ik:{event['stage']}:{event['termination']}"]+=1
        for event in connection_events:
            if event.get("termination") in {"MAXIMUM_ITERATIONS_REACHED","TIME_LIMIT_REACHED"}:
                terminations[f"connection:{event['stage']}:{event['termination']}"]+=1
        for attempt in record["attempts"]:
            for option in attempt.get("conveyor_attempts",[]):
                termination=option.get("escape_search",{}).get("termination")
                if termination and termination!="SUCCESS":
                    terminations[f"escape:{termination}"]+=1
        return {
            "schema_version":"m710_task_search_accounting_v1",
            "task_classification":{
                "grasp_valid_but_not_extracted":bool(record["grasp_reachable"] and not record["extraction_feasible"]),
                "extracted_but_not_complete":bool(record["extraction_feasible"] and not record["geometric_feasible"]),
                "final_stage":record["failure_stage"],"final_reason":record["failure_reason"]},
            "totals":{
                "ik_calls":len(ik_events),
                "ik_seed_pool_available":sum(e.get("seed_pool_available",0) for e in ik_events),
                "ik_seeds_attempted":sum(e.get("seeds_attempted",0) for e in ik_events),
                "ik_iteration_capacity_available":sum(e.get("iteration_capacity_available",0) for e in ik_events),
                "ik_iterations_consumed":sum(e.get("iterations_consumed",0) for e in ik_events),
                "ik_converged_pose_results":sum(e.get("converged_pose_results",0) for e in ik_events),
                "ik_valid_solutions":sum(e.get("valid_solutions",0) for e in ik_events),
                "ik_deduplicated_candidates":sum(e.get("deduplicated_candidates",0) for e in ik_events),
                "connection_attempts":len(connection_events),
                "rrt_iteration_capacity_available":sum(e.get("planning_iteration_budget",0) for e in connection_events),
                "rrt_iterations_consumed":sum(e.get("planning_iterations_consumed",0) for e in connection_events),
                "rrt_extension_attempts":sum(e.get("extension_attempts",0) for e in connection_events),
                "rrt_state_validations":sum(e.get("state_validations",0) for e in connection_events),
                "rrt_edge_validation_calls":sum(e.get("edge_validation_calls",0) for e in connection_events),
                "rrt_edge_state_samples":sum(e.get("edge_state_samples",0) for e in connection_events),
                "escape_robot_validations_all_attempts":escape_validations},
            "candidate_failure_counts_by_stage":dict(candidate_failures),
            "budget_terminations":dict(terminations),
            "budget_scope":{
                "ik":"ik_iterations per seed and ik_restarts per Cell.solve call",
                "grasp_downstream":"grasp_downstream_candidate_limit_per_strategy per extraction strategy",
                "rrt":"rrt_iterations per Cell.transit call",
                "escape":"escape_path_attempt_limit per conveyor option of each downstream grasp candidate; not task-global",
                "state_and_edge_validation":"observed counts only; no separate configured cap"},
            "ik_events":ik_events,"connection_events":connection_events}

    def support_release(self,start,attachment,obstacles,support_names,seed,initial_proximity=None):
        """Lift until every declared support pair satisfies the normal margin."""
        names=list(dict.fromkeys(name for name in support_names if name))
        event={"stage":"SUPPORT_RELEASE","required":bool(names),"support_names":names,
               "collision_margin_per_body_m":self.p["collision_margin_m"]}
        if not names:
            event.update(lift_m=0.0,released=True,q_path=[np.asarray(start).tolist()])
            self.support_release_events.append(event)
            return [np.asarray(start)],None,event
        by_name={box.name:box for box in obstacles}
        missing=sorted(set(names)-set(by_name))
        if missing:
            failure={"reason":"SUPPORT_RELEASE_OBSTACLE_MISSING","support_names":missing}
            event.update(released=False,failure=failure,q_path=[]);self.support_release_events.append(event)
            return [],failure,event
        actual=attachment.box_at(self.robot.fk(start))
        clearance=2*self.p["collision_margin_m"]+2*self.p["support_tolerance_m"]
        bottom=float(actual.corners()[:,2].min())
        lifts={name:max(0.0,by_name[name].center[2]+by_name[name].half_extents[2]+clearance-bottom)
               for name in names}
        lift=max(lifts.values(),default=0.0)
        destination=self.robot.fk(start).copy();destination[2,3]+=lift
        path,failure=self.cartesian(start,destination,obstacles,seed,attachment,names,
                                    initial_proximity=initial_proximity,stage="support_release")
        if failure is None:
            released_box=attachment.box_at(self.robot.fk(path[-1]))
            blocked=[name for name in names if released_box.intersects_obb(
                by_name[name],margin=self.p["collision_margin_m"])]
            if blocked:
                failure={"reason":"SUPPORT_RELEASE_MARGIN_NOT_RESTORED","support_names":blocked}
        event.update(lift_m=lift,lift_by_support_m=lifts,released=failure is None,
            q_path=[q.tolist() for q in path],failure=failure,
            derivation="support top + 2*OBB margin + 2*support tolerance - actual box bottom")
        self.support_release_events.append(event)
        return path,failure,event

    def cartesian(self,start,destination,obstacles,seed,attachment=None,support_names=(),
                  target_contact=None,initial_proximity=None,stage="cartesian"):
        origin = self.robot.fk(start)
        _,dist,angle = pose_error(origin,destination)
        if angle > self.p["cartesian_orientation_tolerance_rad"]:
            return [],{"reason":"CARTESIAN_ORIENTATION_CHANGE_UNSUPPORTED"}
        return self.cartesian_se3(start,destination,obstacles,seed,attachment,support_names,
                                  target_contact,initial_proximity,stage)

    def cartesian_se3(self,start,destination,obstacles,seed,attachment=None,support_names=(),
                      target_contact=None,initial_proximity=None,stage="cartesian_se3"):
        """Interpolate translation and SO(3), validating every strict IK edge."""
        origin = self.robot.fk(start)
        _,dist,angle = pose_error(origin,destination)
        rotation_vector=rotation_vector_from_matrix(destination[:3,:3]@origin[:3,:3].T)
        n = max(1,int(np.ceil(dist/self.p["cartesian_step_m"])),
                int(np.ceil(angle/self.p["cartesian_orientation_step_rad"])))
        path = [np.asarray(start)]
        for i in range(1,n+1):
            pose = origin.copy()
            pose[:3,3] = origin[:3,3]+(destination[:3,3]-origin[:3,3])*i/n
            pose[:3,:3]=rotation_matrix_from_rotation_vector(rotation_vector*i/n)@origin[:3,:3]
            ik = self.solve(pose,[path[-1]],seed+i,stage=f"{stage}_sample")
            if not ik.success:
                return path,{"reason":"NO_IK","sample":i,"position_error_m":ik.position_error,"orientation_error_rad":ik.orientation_error}
            if np.max(np.abs(ik.q-path[-1])) > self.p["cartesian_max_branch_step_rad"]:
                return path,{"reason":"IK_BRANCH_JUMP","sample":i}
            failure = self.path_failure([path[-1],ik.q],obstacles,attachment,support_names,
                                        target_contact,initial_proximity)
            if failure:
                return path,failure
            path.append(ik.q)
        return path,None

    def conveyor_options(self,target,remaining,mode,current):
        c = self.d["conveyor"]
        if mode=="fixed":
            return [tuple(current)]
        front = self.decks((0,c["fixed_z_m"]))[1].corners()[:,0].max()
        extension = float(np.clip(target.corners()[:,0].min()-c["minimum_stack_surface_clearance_m"]-front,0,c["maximum_extension_m"]))
        # Keep V2's upper-stack constraint explicit, including its bottom-layer
        # conflict. No silently relaxed Z bound to make the last box pass.
        highest = max(remaining,key=lambda box:box.corners()[:,2].max())
        upper = min(c["maximum_receiving_surface_z_m"],highest.corners()[:,2].min()-c["upper_stack_bottom_clearance_m"])
        if upper < c["minimum_receiving_surface_z_m"]:
            return []
        desired = float(np.clip(target.corners()[:,2].min(),c["minimum_receiving_surface_z_m"],upper))
        return list(dict.fromkeys([(extension,desired),(0.0,desired),(extension,c["fixed_z_m"]),(0.0,c["fixed_z_m"])]))

    def conveyor_sweep(self,start,end,q,boxes):
        c = self.d["conveyor"]
        n = max(1,int(np.ceil(max(abs(np.asarray(end)-start))/c["movement_step_m"])))
        for u in np.linspace(0,1,2*n+1):
            state = np.asarray(start)+u*(np.asarray(end)-start)
            decks = self.decks(state)
            for deck in decks:
                for obstacle in [*self.walls,*boxes]:
                    if deck.intersects_obb(obstacle,margin=self.p["collision_margin_m"]):
                        return {"reason":"CONVEYOR_SWEEP_COLLISION","pair":[deck.name,obstacle.name],"fraction":float(u)}
                # Conveyor/chassis adjoining edges are a designed interface;
                # penetration of their actual solids is still forbidden.
                if deck.intersects_obb(self.chassis,margin=-self.p["contact_tolerance_m"]):
                    return {"reason":"CONVEYOR_CHASSIS_PENETRATION","fraction":float(u)}
            failure = self.state_failure(q,[*self.fixtures(),*boxes,*decks])
            if failure:
                return {**failure,"conveyor_fraction":float(u)}
        return None


def evaluate_task(cell: Cell,target,remaining,current_q,conveyor_state,*,seed,mode="dynamic",only_face=None,
                  grasp_only=False):
    p,r = cell.p,cell.robot
    others = [b for b in remaining if b.name!=target.name]
    topology = analyze_box_neighborhood(target,others)
    support_names,support_graph_audit=support_relations(target,remaining,cell.fixtures(),p)
    record = {"box":target.name,"seed":int(seed),"mode":mode,"initial_box_pose":target.world_from_local.tolist(),
              "initial_q":np.asarray(current_q).tolist(),"grasp_reachable":False,"extraction_feasible":False,
              "geometric_feasible":False,"payload_qualified":False,"dynamics_verified":False,
              "load_status":"NOT_EVALUATED","failure_stage":"candidate_generation","failure_reason":"NO_EXPOSED_FACE",
              "attempts":[],"selected":None,"conveyor_initial":list(conveyor_state),
              "support_relations":{"support_names":support_names,"graph":support_graph_audit}}
    candidates = generate_extraction_candidates(topology)
    options = cell.conveyor_options(target,remaining,mode,conveyor_state)
    successes=[];downstream_counts={};grasp_task_set_valid=False
    for ci,candidate in enumerate(candidates):
        face=candidate.grasp_face
        if only_face and face!=only_face:
            continue
        targets=[]
        for roll in p["roll_candidates_deg"]:
            nominal,_=target_pose(target.center[0]-target.half_extents[0],target.center[1],target.center[2],2*target.half_extents,face,roll)
            targets.extend((roll,pose,task_set) for pose,task_set in grasp_task_set(
                nominal,p["grasp_face_offset_candidates_m"],p["grasp_tilt_candidates_rad"]))
        for gi,(roll,contact,task_set) in enumerate(targets):
            attempt={"face":face,"roll_deg":roll,"seed":seed+ci*10000+gi*100,"strategy":candidate.strategy,
                     "task_set":task_set,
                     "stage":"coverage","reason":"PENDING","paths":{},"conveyor_attempts":[]}
            record["attempts"].append(attempt)
            coverage=suction_coverage(contact,target,face,cell.d["tool"],p["contact_tolerance_m"])
            attempt["coverage"]=coverage
            # Record extraction constraints even if coverage/IK fails.
            byname={b.name:b for b in others}
            names=sorted({name for name in (topology.left_neighbor,topology.right_neighbor,topology.top_neighbor) if name})
            constraints=[byname[name] for name in names]
            distance=minimum_clearance_extraction_distance(target,candidate.outward_direction_world,constraints,
                free_space_clearance_m=p["extraction_free_clearance_m"],scan_step_m=p["extraction_scan_step_m"],maximum_distance_m=p["maximum_extraction_m"])
            attempt["extraction"]={"distance_m":distance,"direction":candidate.outward_direction_world.tolist(),
                "constraint_names":names,"box_size_xyz_m":(2*target.half_extents).tolist(),"clearance_m":p["extraction_free_clearance_m"],
                "constraint_boxes":[{"name":b.name,"center":b.center.tolist(),"size":(2*b.half_extents).tolist()} for b in constraints],
                "method":"fixed_attitude_projected_OBB_escape_distance_not_path_feasibility"}
            if not coverage["geometric_coverage"]:
                attempt["reason"]="INSUFFICIENT_SEALED_CUPS";continue
            attempt["stage"]="grasp_ik"
            ik_seeds=grasp_seed_configurations(r,[current_q,cell.d["robot"]["home_joints"]])
            attempt["ik_seed_configuration_count"]=len(ik_seeds)
            ik=cell.solve(contact,ik_seeds,attempt["seed"],
                          lambda q: cell.state_failure(q,[*cell.fixtures(),*remaining],target_contact=target) is None,
                          stage="grasp_constrained")
            attempt["ik"]={"success":ik.success,"q":ik.q.tolist(),"position_error_m":ik.position_error,
                           "orientation_error_rad":ik.orientation_error,"message":ik.message,"source":"constrained"}
            if not ik.success:
                diagnostic=cell.solve(contact,ik_seeds,attempt["seed"]+1,stage="grasp_diagnostic")
                attempt["ik"]["unconstrained_diagnostic"]={"success":diagnostic.success,"q":diagnostic.q.tolist(),
                    "position_error_m":diagnostic.position_error,"orientation_error_rad":diagnostic.orientation_error,
                    "message":diagnostic.message}
                if diagnostic.success:
                    failure=cell.state_failure(diagnostic.q,[*cell.fixtures(),*remaining],target_contact=target)
                    if failure is None:
                        ik=diagnostic
                        attempt["ik"].update(success=True,q=ik.q.tolist(),position_error_m=ik.position_error,
                            orientation_error_rad=ik.orientation_error,message=ik.message,source="diagnostic_seed_stream")
                    else:
                        attempt["reason"]="GRASP_CONSTRAINT_FAILED"
                        attempt["failure"]=failure
                        attempt["failure_taxonomy"]={"category":"GRASP_CONSTRAINT_FAILED",
                            "detail":failure["reason"],"strict_fk_pose_reached":True}
                        continue
                else:
                    detail=residual_failure_detail(diagnostic,p)
                    attempt["reason"]="NO_IK"
                    attempt["failure_taxonomy"]={"category":"NO_IK","detail":detail,
                        "strict_fk_pose_reached":False,"search_status":"BUDGET_EXHAUSTED_NOT_INFEASIBILITY_PROOF"}
                    continue
            actual=r.fk(ik.q)
            actual_coverage=suction_coverage(actual,target,face,cell.d["tool"],p["contact_tolerance_m"])
            attempt["actual_coverage"]=actual_coverage
            if not actual_coverage["geometric_coverage"]:
                attempt["reason"]="ACTUAL_CONTACT_COVERAGE_FAILED";continue
            failure=cell.state_failure(ik.q,[*cell.fixtures(),*remaining],target_contact=target)
            if failure:
                attempt.update(stage="grasp_collision",reason=failure["reason"],failure=failure);continue
            # Register only pre-existing target-neighbor proximity. Fixtures,
            # supports and all unregistered pairs retain their original rules.
            initial_supports={"floor",topology.bottom_support}
            eligible_neighbors=[obstacle for obstacle in others if obstacle.name not in initial_supports]
            initial_proximity,attachment_failure=InitialProximityTracker.capture(
                target,eligible_neighbors,p["collision_margin_m"],p["contact_tolerance_m"],
                p["initial_proximity_monotonic_tolerance_m"])
            registered=set(initial_proximity.pairs)
            for obstacle in [*cell.fixtures(),*others]:
                if attachment_failure:
                    break
                if obstacle.name in initial_supports and contact_separated(target,obstacle,p["support_tolerance_m"]):
                    continue
                if obstacle.name in registered:
                    continue
                if target.intersects_obb(obstacle,margin=p["collision_margin_m"]):
                    attachment_failure={"reason":"PAYLOAD_INITIAL_CLEARANCE_FAILED","pair":[target.name,obstacle.name],
                                        "obb_margin_per_body_m":p["collision_margin_m"],"box_pose_world":target.world_from_local.tolist()}
                    break
            attempt["initial_proximity"]=initial_proximity.evidence()
            if attachment_failure:
                attempt.update(stage="attachment_clearance",reason=attachment_failure["reason"],failure=attachment_failure)
                continue
            attempt["strict_grasp_valid"]=True
            record["grasp_reachable"]=True
            if grasp_only:
                attempt.update(stage="grasp_task_set_complete",reason="GRASP_TASK_SET_VALID")
                grasp_task_set_valid=True
                continue
            if downstream_counts.get(ci,0)>=p["grasp_downstream_candidate_limit_per_strategy"]:
                attempt.update(stage="grasp_task_set",reason="DOWNSTREAM_TASK_SET_BUDGET",
                    downstream_budget_per_strategy=p["grasp_downstream_candidate_limit_per_strategy"])
                continue
            downstream_counts[ci]=downstream_counts.get(ci,0)+1
            if distance is None:
                attempt.update(stage="extraction",reason="EXTRACTION_DISTANCE_EXCEEDED");continue
            if not options:
                attempt.update(stage="conveyor",reason="MINIMUM_BELT_HEIGHT_EXCEEDS_UPPER_STACK_BOUND");continue
            for oi,option in enumerate(options):
                sub={"state":list(option),"stage":"conveyor_preposition","reason":"PENDING","paths":{}}
                attempt["conveyor_attempts"].append(sub)
                failure=cell.conveyor_sweep(conveyor_state,option,current_q,remaining)
                if failure:
                    sub.update(reason=failure["reason"],failure=failure);continue
                decks=cell.decks(option)
                obstacles=[*cell.fixtures(),*others,*decks]
                pre=actual.copy();pre[:3,3]+=candidate.outward_direction_world*p["pregrasp_standoff_m"]
                preik=cell.solve(pre,[ik.q,current_q],seed+200+oi,stage="pregrasp")
                if not preik.success:
                    sub.update(stage="approach",reason="PREGRASP_NO_IK");continue
                approach,failure=cell.transit(current_q,preik.q,[*obstacles,target],seed+300+oi,
                                              stage="approach")
                sub["paths"]["approach"]=[q.tolist() for q in approach]
                if failure:
                    sub.update(stage="approach",reason=failure["reason"],failure=failure);continue
                contact_path,failure=cell.cartesian(preik.q,actual,[*obstacles,target],seed+400+oi,
                                                    target_contact=target,stage="contact")
                sub["paths"]["contact"]=[q.tolist() for q in contact_path]
                if failure:
                    sub.update(stage="contact",reason=failure["reason"],failure=failure);continue
                contact_state,failure=cell.validate_contact_endpoint(
                    contact_path[-1],target,face,[*obstacles,target])
                if failure:
                    sub.update(stage="contact_validation",reason=failure["reason"],failure=failure)
                    continue
                sub["contact_state"]=contact_state.evidence()
                attached=contact_state.attachment
                sub["tcp_from_box"]=attached.tcp_from_box.tolist()
                attempt["load_contact"]=external_load(r,cell.config.tool,contact_state.q,attached,
                    cell.d["scene"]["box_mass_kg"],cell.d["scene"]["box_com_fraction"],cell.config.model)
                record["load_status"]=attempt["load_contact"]["qualification"]
                proximity_path=initial_proximity.clone()
                support_path,failure,support_event=cell.support_release(contact_path[-1],attached,obstacles,
                    support_names,attempt["seed"]+4500+oi,proximity_path)
                sub["paths"]["support_release"]=[q.tolist() for q in support_path]
                sub["support_release"]=support_event
                if failure:
                    sub.update(stage="support_release",reason=failure["reason"],failure=failure);continue
                extraction_start=support_path[-1]
                support_proximity=proximity_path
                released_start_box=attached.box_at(r.fk(extraction_start))
                pure_endpoint=r.fk(extraction_start).copy();pure_endpoint[:3,3]+=candidate.outward_direction_world*distance
                extraction=[];failure=None;proximity_path=None;escape_selected=None
                proposal_iterator=escape_path_proposals(
                    released_start_box,candidate.outward_direction_world,constraints,distance,p)
                escape_budget=p["escape_path_attempt_limit"]
                scheduled=list(itertools.islice(proposal_iterator,escape_budget+1))
                proposals=scheduled[:escape_budget]
                candidates_remain=len(scheduled)>escape_budget
                sub["escape_attempts"]=[]
                sub["escape_search"]={"scheduler":"station_direction_novelty_then_rotation_v2",
                    "robot_validation_budget":escape_budget,"scheduled_attempts":len(proposals),
                    "candidate_availability_after_schedule":"MORE_AVAILABLE" if candidates_remain else "EXHAUSTED",
                    "pure_straight_budget_semantics":"mandatory fallback outside escape validation budget",
                    "termination":"NOT_SEARCHED"}
                for pi,proposal in enumerate(proposals):
                    tracker=support_proximity.clone()
                    start_q=extraction_start;prefix=[]
                    if proposal["constrained_straight_distance_m"]>0:
                        station=r.fk(start_q).copy();station[:3,3]+=candidate.outward_direction_world*proposal["constrained_straight_distance_m"]
                        prefix,failure=cell.cartesian(start_q,station,obstacles,attempt["seed"]+5000+pi*100,
                            attached,initial_proximity=tracker,stage="escape_constrained_straight")
                        if failure:
                            sub["escape_attempts"].append({**proposal,"validation_stage":"constrained_straight",
                                "status":"REJECTED_CONSTRAINT","failure":failure});continue
                        start_q=prefix[-1]
                    endpoint=r.fk(start_q).copy()
                    endpoint[:3,3]+=np.asarray(proposal["escape_direction_world"])*proposal["escape_translation_m"]
                    endpoint[:3,:3]=rotation_matrix_from_rpy(0,0,proposal["escape_rotation_world_z_rad"])@endpoint[:3,:3]
                    escaped,failure=cell.cartesian_se3(start_q,endpoint,obstacles,attempt["seed"]+5050+pi*100,
                        attached,initial_proximity=tracker,stage="escape_se3")
                    evidence={**proposal,"validation_stage":"escape_se3",
                        "status":"ACCEPTED" if failure is None and tracker.fully_released else
                                 "REJECTED_CONSTRAINT" if failure else "REJECTED_MARGIN_NOT_RESTORED",
                        "failure":failure,"normal_margin_restored":tracker.fully_released}
                    sub["escape_attempts"].append(evidence)
                    if failure is None and tracker.fully_released:
                        extraction=[*prefix,*escaped[1:]] if prefix else escaped
                        proximity_path=tracker;escape_selected=evidence;break
                if escape_selected is None:
                    sub["escape_search"]["termination"]=("SEARCH_BUDGET_EXHAUSTED" if candidates_remain
                        else "CANDIDATES_EXHAUSTED" if proposals else "NO_GEOMETRIC_CANDIDATES")
                    sub["escape_search"]["pure_straight_attempted"]=True
                    proximity_path=support_proximity.clone()
                    extraction,failure=cell.cartesian(extraction_start,pure_endpoint,obstacles,seed+500+oi,
                        attached,initial_proximity=proximity_path,stage="pure_straight_extraction")
                    sub["pure_straight_attempt"]={"status":"ACCEPTED" if failure is None else "REJECTED_CONSTRAINT",
                                                   "failure":failure,"distance_m":distance}
                else:
                    failure=None
                    sub["escape_search"].update(termination="SUCCESS",pure_straight_attempted=False,
                                                 selected_schedule_index=escape_selected["schedule_index"])
                sub["paths"]["extraction"]=[q.tolist() for q in extraction]
                sub["initial_proximity"]=proximity_path.evidence()
                tcp=[r.fk(q)[:3,3] for q in extraction]
                release_path_length=float(sum(np.linalg.norm(b-a) for a,b in zip(tcp[:-1],tcp[1:])))
                support_tcp=[r.fk(q)[:3,3] for q in support_path]
                support_path_length=float(sum(np.linalg.norm(b-a) for a,b in zip(support_tcp[:-1],support_tcp[1:])))
                sub["extraction_metrics"]={
                    "pure_straight_clearance_distance":distance,
                    "actual_constrained_extraction_distance":release_path_length if failure is None else None,
                    "distance_until_first_escape_path":None if escape_selected is None else escape_selected["constrained_straight_distance_m"],
                    "total_stack_release_distance":support_path_length+release_path_length if failure is None and proximity_path.fully_released else None,
                    "escape_path_used":escape_selected is not None,
                }
                if failure:
                    sub.update(stage="extraction",reason=failure["reason"],failure=failure);continue
                if not proximity_path.fully_released:
                    failure={"reason":"PAYLOAD_PROXIMITY_NOT_RELEASED",
                             "initial_proximity":proximity_path.evidence()}
                    sub.update(stage="extraction",reason=failure["reason"],failure=failure);continue
                record["extraction_feasible"]=True
                deck=decks[1]
                desired_box=target.world_from_local.copy()
                desired_box[:3,3]=[deck.center[0],deck.center[1],option[1]+target.half_extents[2]]
                desired_tcp=desired_box @ np.linalg.inv(attached.tcp_from_box)
                handoff=cell.solve(desired_tcp,[extraction[-1],ik.q,current_q],seed+600+oi,stage="handoff")
                if not handoff.success:
                    sub.update(stage="handoff",reason="HANDOFF_NO_IK");continue
                carry,failure=cell.transit(extraction[-1],handoff.q,obstacles,seed+700+oi,attached,
                                           [deck.name],stage="carry")
                sub["paths"]["carry"]=[q.tolist() for q in carry]
                if failure:
                    sub.update(stage="carry",reason=failure["reason"],failure=failure);continue
                placed=attached.box_at(r.fk(carry[-1]))
                support=support_audit(placed,deck,p["support_tolerance_m"],p["support_edge_clearance_m"])
                sub["support"]=support
                if not support["supported"]:
                    sub.update(stage="place",reason="ACTUAL_FK_SUPPORT_FAILED");continue
                retreat=r.fk(carry[-1]).copy();retreat[:3,3]-=retreat[:3,2]*p["pregrasp_standoff_m"]
                withdrawal,failure=cell.cartesian(carry[-1],retreat,[*obstacles,placed],seed+800+oi,
                                                  target_contact=placed,stage="withdrawal")
                sub["paths"]["withdrawal"]=[q.tolist() for q in withdrawal]
                if failure:
                    sub.update(stage="withdrawal",reason=failure["reason"],failure=failure);continue
                # Placed box remains present until the belt clears the receiving
                # area. Never delete it at vacuum release.
                sub.update(stage="complete",reason="OK",final_q=withdrawal[-1].tolist(),
                           placed_box_pose=placed.world_from_local.tolist(),vacuum_release=True,
                           receiver_state="OCCUPIED",load_status=attempt["load_contact"]["qualification"])
                full=[];stage_indices={}
                for name in ("approach","contact","support_release","extraction","carry","withdrawal"):
                    path=sub["paths"][name]
                    begin=max(0,len(full)-1)
                    full.extend(path if not full else path[1:])
                    stage_indices[name]=[begin,len(full)-1]
                timed=time_parameterize_joint_path(full,cell.config.motion_limits())
                sub["trajectory"]={"q_knots":full,"t_knots_s":timed.time_from_start.tolist(),"stages":stage_indices,"audit":timed.audit(cell.config.motion_limits())}
                loaded=[*support_path,*extraction[1:],*carry[1:]]
                # Integrate the actual FK curve of each interpolated joint
                # edge, not the chord between Cartesian waypoint endpoints.
                tcp_points=[r.fk(loaded[0])[:3,3]]
                for a,b in zip(loaded[:-1],loaded[1:]):
                    n=max(1,int(np.ceil(np.max(np.abs(b-a))/(p["edge_resolution_rad"]/2))))
                    tcp_points.extend(r.fk(a+u*(b-a))[:3,3] for u in np.linspace(0,1,n+1)[1:])
                sub["loaded_tcp_path_m"]=float(sum(np.linalg.norm(b-a) for a,b in zip(tcp_points[:-1],tcp_points[1:])))
                sub["joint_path_rad"]=float(sum(np.linalg.norm(np.asarray(b)-a) for a,b in zip(full[:-1],full[1:])))
                c=cell.d["conveyor"]
                sub["conveyor_action_s"]=abs(option[0]-conveyor_state[0])/c["extension_speed_m_s"]+abs(option[1]-conveyor_state[1])/c["lift_speed_m_s"]
                sub["cycle_s"]=timed.duration_seconds+sub["conveyor_action_s"]+cell.d["execution"]["vacuum_establish_s"]+cell.d["execution"]["release_s"]
                successes.append((sub["cycle_s"],ci,roll,sub,attempt))
            if not any(s["reason"]=="OK" for s in attempt["conveyor_attempts"]):
                first=attempt["conveyor_attempts"][0]
                attempt.update(stage=first["stage"],reason=first["reason"])
            else:
                attempt.update(stage="complete",reason="OK")
    if grasp_only and grasp_task_set_valid:
        record.update(failure_stage="grasp_task_set_complete",failure_reason="GRASP_TASK_SET_VALID")
    elif successes:
        _,_,roll,best,attempt=min(successes,key=lambda s:(s[0],s[1],s[2]))
        record.update(geometric_feasible=True,failure_stage="complete",failure_reason="OK",
            selected={**best,"face":attempt["face"],"roll_deg":roll,
                      "task_set":attempt["task_set"]},load_status=best["load_status"])
    elif record["attempts"]:
        first=record["attempts"][0]
        record["first_failure"]={"stage":first["stage"],"reason":first["reason"]}
        stages=["coverage","grasp_ik","grasp_collision","attachment_clearance","grasp_task_set",
                "grasp_task_set_complete","conveyor","conveyor_preposition","approach","contact","contact_validation",
                "support_release","extraction","handoff","carry","place","withdrawal","complete"]
        furthest=max(record["attempts"],key=lambda a:stages.index(a["stage"]))
        record.update(failure_stage=furthest["stage"],failure_reason=furthest["reason"])
    record["search_statistics"]=cell.search_statistics(record)
    return record
