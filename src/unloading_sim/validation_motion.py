"""Deterministic complete-cycle planner for the V3 acceptance contract."""
from __future__ import annotations

from dataclasses import dataclass
import itertools

import numpy as np

from .depalletizing import (LConveyorGeometry, analyze_box_neighborhood,
                           generate_extraction_candidates, minimum_clearance_extraction_distance)
from .fanuc_m710id70 import target_pose, trailer_obstacles
from .geometry import OBB, make_transform
from .ik import solve_ik_multistart, pose_error
from .planner import RRTConnectPlanner
from .timing import time_parameterize_joint_path
from .validation_physics import (RigidAttachment, contact_separated, external_load,
                                 suction_coverage, support_audit, urdf_collision_shapes, world_link_boxes)


def transformed(box: OBB, pose: np.ndarray) -> OBB:
    world = pose @ box.world_from_local
    return OBB(world[:3,3], box.half_extents, world[:3,:3], box.name, box.category)


class Cell:
    def __init__(self, config, height=None):
        self.config = config
        self.d = config.data
        self.p = self.d["planning"]
        self.robot = config.robot(height)
        self.shapes = urdf_collision_shapes(self.robot)
        self._bounds_cache = {}
        self.support_release_events = []
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

    def state_failure(self, q, obstacles, attachment=None, support_names=(), target_contact=None):
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
            for b in self.broadphase(box,obstacles,p["collision_margin_m"]):
                if b.name in support_names and contact_separated(box,b,p["support_tolerance_m"]):
                    continue
                if box.intersects_obb(b,margin=p["collision_margin_m"]):
                    return {"reason":"PAYLOAD_COLLISION","pair":[box.name,b.name]}
            for a in self.broadphase(box,links,p["collision_margin_m"]):
                if box.intersects_obb(a,margin=p["collision_margin_m"]):
                    return {"reason":"PAYLOAD_ROBOT_COLLISION","pair":[box.name,a.name]}
        return None

    def path_failure(self, path, obstacles, attachment=None, support_names=(), target_contact=None):
        count = 0
        for index,(a,b) in enumerate(zip(path[:-1],path[1:])):
            n = max(1,int(np.ceil(np.max(np.abs(b-a))/self.p["edge_resolution_rad"])))
            # Midpoints supplement all endpoint samples, including the start.
            for u in np.linspace(0,1,2*n+1):
                count += 1
                failure = self.state_failure(a+u*(b-a),obstacles,attachment,support_names,target_contact)
                if failure:
                    return {**failure,"edge":index,"fraction":float(u),"q_rad":(a+u*(b-a)).tolist()}
        if len(path)==1:
            return self.state_failure(path[0],obstacles,attachment,support_names,target_contact)
        return None

    def solve(self, pose, seeds, rng_seed, valid=None):
        p = self.p
        unique = list({tuple(np.asarray(q,float)): np.asarray(q,float) for q in seeds}.values())
        return solve_ik_multistart(self.robot,pose,unique,random_restarts=p["ik_restarts"],rng=np.random.default_rng(rng_seed),
            max_iterations=p["ik_iterations"],damping=p["ik_damping"],max_step=p["ik_max_step_rad"],
            position_tolerance=p["ik_position_tolerance_m"],orientation_tolerance=p["ik_orientation_tolerance_rad"],
            orientation_weight=p["ik_orientation_weight"],extra_state_valid=valid,
            collision_check_stride=p["ik_iterations"]+1)

    def transit(self,start,goal,obstacles,seed,attachment=None,support_names=(),target_contact=None):
        prefix=[]
        if attachment is not None:
            # A zero geometric destacking distance does not imply that free
            # joint-space carry can start at floor contact. Lift clear while
            # preserving the designated support contact, then restore the
            # unchanged full collision margin for the subsequent carry.
            floor=next((b for b in obstacles if b.name=="floor"),None)
            if floor is not None:
                actual=attachment.box_at(self.robot.fk(start))
                clearance=2*self.p["collision_margin_m"]+2*self.p["support_tolerance_m"]
                lift=floor.center[2]+floor.half_extents[2]+clearance-float(actual.corners()[:,2].min())
                if lift>0:
                    destination=self.robot.fk(start).copy();destination[2,3]+=lift
                    prefix,failure=self.cartesian(start,destination,obstacles,seed,attachment,[*support_names,"floor"])
                    self.support_release_events.append({"lift_m":lift,"q_path":[q.tolist() for q in prefix],"failure":failure,
                        "derivation":"floor top + 2*OBB margin + 2*support tolerance - actual box bottom"})
                    if failure:return prefix,failure
                    start=prefix[-1]
        state = lambda q: self.state_failure(q,obstacles,attachment,support_names,target_contact) is None
        planner = RRTConnectPlanner(self.robot.joint_limits[:,0],self.robot.joint_limits[:,1],state,
            step_size=self.p["rrt_step_rad"],edge_resolution=self.p["edge_resolution_rad"]/2,
            max_iterations=self.p["rrt_iterations"],goal_bias=self.p["rrt_goal_bias"],rng=np.random.default_rng(seed))
        result = planner.plan(start,goal)
        if not result.success:
            failure = self.state_failure(start,obstacles,attachment,support_names,target_contact) or self.state_failure(goal,obstacles,attachment,support_names,target_contact)
            return prefix, failure or {"reason":"PATH_SEARCH_EXHAUSTED","detail":result.message,"iterations":result.iterations}
        failure = self.path_failure(result.path,obstacles,attachment,support_names,target_contact)
        return ([*prefix,*result.path[1:]] if prefix else result.path), failure

    def cartesian(self,start,destination,obstacles,seed,attachment=None,support_names=(),target_contact=None):
        origin = self.robot.fk(start)
        _,dist,angle = pose_error(origin,destination)
        if angle > self.p["cartesian_orientation_tolerance_rad"]:
            return [],{"reason":"CARTESIAN_ORIENTATION_CHANGE_UNSUPPORTED"}
        n = max(1,int(np.ceil(dist/self.p["cartesian_step_m"])))
        path = [np.asarray(start)]
        for i in range(1,n+1):
            pose = origin.copy(); pose[:3,3] = origin[:3,3]+(destination[:3,3]-origin[:3,3])*i/n
            ik = self.solve(pose,[path[-1]],seed+i)
            if not ik.success:
                return path,{"reason":"NO_IK","sample":i,"position_error_m":ik.position_error,"orientation_error_rad":ik.orientation_error}
            if np.max(np.abs(ik.q-path[-1])) > self.p["cartesian_max_branch_step_rad"]:
                return path,{"reason":"IK_BRANCH_JUMP","sample":i}
            failure = self.path_failure([path[-1],ik.q],obstacles,attachment,support_names,target_contact)
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


def evaluate_task(cell: Cell,target,remaining,current_q,conveyor_state,*,seed,mode="dynamic",only_face=None):
    p,r = cell.p,cell.robot
    others = [b for b in remaining if b.name!=target.name]
    topology = analyze_box_neighborhood(target,others)
    record = {"box":target.name,"seed":int(seed),"mode":mode,"initial_box_pose":target.world_from_local.tolist(),
              "initial_q":np.asarray(current_q).tolist(),"grasp_reachable":False,"extraction_feasible":False,
              "geometric_feasible":False,"payload_qualified":False,"dynamics_verified":False,
              "load_status":"NOT_EVALUATED","failure_stage":"candidate_generation","failure_reason":"NO_EXPOSED_FACE",
              "attempts":[],"selected":None,"conveyor_initial":list(conveyor_state)}
    candidates = generate_extraction_candidates(topology)
    options = cell.conveyor_options(target,remaining,mode,conveyor_state)
    successes=[]
    for ci,candidate in enumerate(candidates):
        face=candidate.grasp_face
        if only_face and face!=only_face:
            continue
        for roll in p["roll_candidates_deg"]:
            attempt={"face":face,"roll_deg":roll,"seed":seed+ci*100+roll,"strategy":candidate.strategy,
                     "stage":"coverage","reason":"PENDING","paths":{},"conveyor_attempts":[]}
            record["attempts"].append(attempt)
            contact,_=target_pose(target.center[0]-target.half_extents[0],target.center[1],target.center[2],2*target.half_extents,face,roll)
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
            ik=cell.solve(contact,[current_q,cell.d["robot"]["home_joints"]],attempt["seed"],
                          lambda q: cell.state_failure(q,[*cell.fixtures(),*remaining],target_contact=target) is None)
            attempt["ik"]={"success":ik.success,"q":ik.q.tolist(),"position_error_m":ik.position_error,"orientation_error_rad":ik.orientation_error}
            if not ik.success:
                failure=cell.state_failure(ik.q,[*cell.fixtures(),*remaining],target_contact=target)
                attempt["reason"]="NO_IK" if ik.position_error>p["ik_position_tolerance_m"] or ik.orientation_error>p["ik_orientation_tolerance_rad"] else "GRASP_CONSTRAINT_FAILED"
                attempt["failure"]=failure
                continue
            actual=r.fk(ik.q)
            attachment=RigidAttachment.capture(actual,target)
            attempt["tcp_from_box"]=attachment.tcp_from_box.tolist()
            attempt["load_contact"]=external_load(r,cell.config.tool,ik.q,attachment,cell.d["scene"]["box_mass_kg"],cell.d["scene"]["box_com_fraction"],cell.config.model)
            record["load_status"]=attempt["load_contact"]["qualification"]
            actual_coverage=suction_coverage(actual,target,face,cell.d["tool"],p["contact_tolerance_m"])
            attempt["actual_coverage"]=actual_coverage
            if not actual_coverage["geometric_coverage"]:
                attempt["reason"]="ACTUAL_CONTACT_COVERAGE_FAILED";continue
            failure=cell.state_failure(ik.q,[*cell.fixtures(),*remaining],target_contact=target)
            if failure:
                attempt.update(stage="grasp_collision",reason=failure["reason"],failure=failure);continue
            record["grasp_reachable"]=True
            # A robot path cannot fix a clearance violation that already
            # exists between the attached carton and stationary geometry at
            # t=0. Validate this invariant before spending RRT iterations on
            # an approach that would necessarily fail at attachment. The
            # same margin/support predicate is used later along extraction.
            initial_supports={"floor",topology.bottom_support}
            attachment_failure=None
            for obstacle in [*cell.fixtures(),*others]:
                if obstacle.name in initial_supports and contact_separated(target,obstacle,p["support_tolerance_m"]):
                    continue
                if target.intersects_obb(obstacle,margin=p["collision_margin_m"]):
                    attachment_failure={"reason":"PAYLOAD_INITIAL_CLEARANCE_FAILED","pair":[target.name,obstacle.name],
                                        "obb_margin_per_body_m":p["collision_margin_m"],"box_pose_world":target.world_from_local.tolist()}
                    break
            if attachment_failure:
                attempt.update(stage="attachment_clearance",reason=attachment_failure["reason"],failure=attachment_failure)
                continue
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
                preik=cell.solve(pre,[ik.q,current_q],seed+200+oi)
                if not preik.success:
                    sub.update(stage="approach",reason="PREGRASP_NO_IK");continue
                approach,failure=cell.transit(current_q,preik.q,[*obstacles,target],seed+300+oi)
                sub["paths"]["approach"]=[q.tolist() for q in approach]
                if failure:
                    sub.update(stage="approach",reason=failure["reason"],failure=failure);continue
                contact_path,failure=cell.cartesian(preik.q,actual,[*obstacles,target],seed+400+oi,target_contact=target)
                sub["paths"]["contact"]=[q.tolist() for q in contact_path]
                if failure:
                    sub.update(stage="contact",reason=failure["reason"],failure=failure);continue
                # Capture at the endpoint actually reached by the approach.
                attached=RigidAttachment.capture(r.fk(contact_path[-1]),target)
                sub["tcp_from_box"]=attached.tcp_from_box.tolist()
                supports=["floor"]+([topology.bottom_support] if topology.bottom_support else [])
                endpoint=r.fk(contact_path[-1]).copy();endpoint[:3,3]+=candidate.outward_direction_world*distance
                extraction,failure=cell.cartesian(contact_path[-1],endpoint,obstacles,seed+500+oi,attached,supports)
                sub["paths"]["extraction"]=[q.tolist() for q in extraction]
                if failure:
                    sub.update(stage="extraction",reason=failure["reason"],failure=failure);continue
                record["extraction_feasible"]=True
                deck=decks[1]
                desired_box=target.world_from_local.copy()
                desired_box[:3,3]=[deck.center[0],deck.center[1],option[1]+target.half_extents[2]]
                desired_tcp=desired_box @ np.linalg.inv(attached.tcp_from_box)
                handoff=cell.solve(desired_tcp,[extraction[-1],ik.q,current_q],seed+600+oi)
                if not handoff.success:
                    sub.update(stage="handoff",reason="HANDOFF_NO_IK");continue
                lift_begin=len(cell.support_release_events)
                carry,failure=cell.transit(extraction[-1],handoff.q,obstacles,seed+700+oi,attached,[deck.name])
                sub["support_release_lifts"]=cell.support_release_events[lift_begin:]
                sub["paths"]["carry"]=[q.tolist() for q in carry]
                if failure:
                    sub.update(stage="carry",reason=failure["reason"],failure=failure);continue
                placed=attached.box_at(r.fk(carry[-1]))
                support=support_audit(placed,deck,p["support_tolerance_m"],p["support_edge_clearance_m"])
                sub["support"]=support
                if not support["supported"]:
                    sub.update(stage="place",reason="ACTUAL_FK_SUPPORT_FAILED");continue
                retreat=r.fk(carry[-1]).copy();retreat[:3,3]-=retreat[:3,2]*p["pregrasp_standoff_m"]
                withdrawal,failure=cell.cartesian(carry[-1],retreat,[*obstacles,placed],seed+800+oi,target_contact=placed)
                sub["paths"]["withdrawal"]=[q.tolist() for q in withdrawal]
                if failure:
                    sub.update(stage="withdrawal",reason=failure["reason"],failure=failure);continue
                # Placed box remains present until the belt clears the receiving
                # area. Never delete it at vacuum release.
                sub.update(stage="complete",reason="OK",final_q=withdrawal[-1].tolist(),
                           placed_box_pose=placed.world_from_local.tolist(),vacuum_release=True,
                           receiver_state="OCCUPIED",load_status=attempt["load_contact"]["qualification"])
                full=[];stage_indices={}
                for name in ("approach","contact","extraction","carry","withdrawal"):
                    path=sub["paths"][name]
                    begin=max(0,len(full)-1)
                    full.extend(path if not full else path[1:])
                    stage_indices[name]=[begin,len(full)-1]
                timed=time_parameterize_joint_path(full,cell.config.motion_limits())
                sub["trajectory"]={"q_knots":full,"t_knots_s":timed.time_from_start.tolist(),"stages":stage_indices,"audit":timed.audit(cell.config.motion_limits())}
                loaded=[*extraction,*carry[1:]]
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
    if successes:
        _,_,roll,best,attempt=min(successes,key=lambda s:(s[0],s[1],s[2]))
        record.update(geometric_feasible=True,failure_stage="complete",failure_reason="OK",
            selected={**best,"face":attempt["face"],"roll_deg":roll},load_status=best["load_status"])
    elif record["attempts"]:
        first=record["attempts"][0]
        record["first_failure"]={"stage":first["stage"],"reason":first["reason"]}
        stages=["coverage","grasp_ik","grasp_collision","attachment_clearance","conveyor","conveyor_preposition","approach","contact","extraction","handoff","carry","place","withdrawal","complete"]
        furthest=max(record["attempts"],key=lambda a:stages.index(a["stage"]))
        record.update(failure_stage=furthest["stage"],failure_reason=furthest["reason"])
    return record
