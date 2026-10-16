"""One configured receiver-center preplace target. Geometry only, no search."""
from dataclasses import dataclass, replace
import numpy as np

from .geometry import OBB
from .independent_cups import _finite_se3
from .conveyor_placement import (ConveyorSupport, LONGITUDINAL, placement_working_normal,
    support_union_audit, effective_process_and_receiver)
from .release_motion import receiver_footprint_reserve
from .pair_clearance import obb_pair_failure, obb_distance_at_least
from .moveit2_backend import native_tool_collision_boxes, MoveItUnavailable


@dataclass(frozen=True)
class PreplaceTarget:
    target_id: str
    scene_revision: str
    receiver_id: str
    placement_family: str
    landing_box: OBB
    preplace_box: OBB
    physical_contact_pose: np.ndarray
    task_tcp_pose: np.ndarray
    tcp_from_box: np.ndarray
    geometry: dict

    def evidence(self):
        return dict(target_id=self.target_id,scene_revision=self.scene_revision,receiver=self.receiver_id,
            placement_family=self.placement_family,landing_intent_box_pose=self.landing_box.world_from_local.tolist(),
            preplace_box_pose=self.preplace_box.world_from_local.tolist(),
            physical_contact_pose=self.physical_contact_pose.tolist(),task_tcp_pose=self.task_tcp_pose.tolist(),
            captured_contact_from_box=self.tcp_from_box.tolist(),geometry=self.geometry,
            landing_intent_only=True,received=False,physical_execution="NOT_RUN")


def fixed_preplace_target(c, contact, attachment, start, world):
    """The first configured family, one in-plane heading, one receiver center."""
    data=c.native_scene.policy.data
    strategy=data["search_strategy"]
    receiver_name=data["receiver"]["name"]
    supports=tuple(ConveyorSupport(b,c.surface_directions_world.get(b.name))
        for b in world if b.category=="conveyor")
    matches=[s for s in supports if s.name==receiver_name]
    if len(matches)!=1 or attachment.rigid.name!=contact.target_id:
        raise MoveItUnavailable("PREPLACE_RECEIVER_OR_PAYLOAD_IDENTITY_INVALID")
    receiver=matches[0]
    policy=replace(c.placement_policy,
        contact_tolerance_m=c.contact_tolerance_m,
        edge_tolerance_m=float(strategy.get("conveyor_footprint_boundary_tolerance_m",1e-6)),
        occupancy_clearance_m=c.collision_policy.pair_clearance("external",c.collision_margin_m),
        normal_tolerance_rad=np.deg2rad(float(strategy.get("placement_normal_tolerance_deg",5.))),
        process_family_by_support=dict(strategy["surface_process_families"]),
        allowed_families_by_process={k:tuple(v) for k,v in strategy["allowed_placement_families"].items()},
        overlap_process_priority=tuple(strategy["overlap_process_priority"]))
    process,effective=effective_process_and_receiver([receiver.name],supports,policy)
    audit=dict(receiver_pose=receiver.body.world_from_local.tolist(),
        receiver_half_extents_m=receiver.body.half_extents.tolist(),process_family=process,
        position_rule="CONFIGURED_RECEIVER_GEOMETRIC_CENTER",
        heading_rule="CONTACT_Y_LONG_CUP_AXIS_ALONG_PROJECTED_RECEIVER_OUTLET",
        standoff_m=c.budget.preplace_standoff_m,edge_reserve_m=c.budget.receiver_edge_reserve_m)
    def rejected(reason,**detail):
        audit.update(rejection=reason,**detail)
        return None,dict(reason="FIXED_PREPLACE_TARGET_NOT_APPLICABLE",stage="transit",
            scope="FACE_ATTEMPT",geometry=audit)
    if process!=LONGITUDINAL or effective!=receiver_name or not data["receiver"].get("fixed"):
        return rejected("CONFIGURED_LONGITUDINAL_RECEIVER_REQUIRED")
    family=policy.allowed_families_by_process[process][0]
    audit["placement_family"]=family
    normal=placement_working_normal(family)
    up=receiver.body.rotation[:,2]
    direction=np.asarray(receiver.outlet_direction_world,dtype=float)
    if direction.shape!=(3,) or not np.isfinite(direction).all() or not np.allclose(up,[0,0,1],atol=1e-12,rtol=0):
        return rejected("RECEIVER_PLANE_OR_DIRECTION_INVALID")
    y=direction-normal*float(direction@normal)
    if np.linalg.norm(y)<1e-12:
        return rejected("FIXED_HEADING_PARALLEL_TO_WORKING_NORMAL")
    y/=np.linalg.norm(y);x=np.cross(y,normal)
    contact_rotation=np.column_stack([x,y,normal])
    captured=_finite_se3(attachment.rigid.tcp_from_box,"captured_contact_from_box")
    rotation=contact_rotation@captured[:3,:3]
    vertical_half=float(np.abs(rotation[2,:])@attachment.rigid.half_extents)
    center=receiver.body.center.copy()
    center[2]+=receiver.body.half_extents[2]+vertical_half
    landing=OBB(center,attachment.rigid.half_extents,rotation,contact.target_id,"payload")
    generating=support_union_audit(landing,[receiver],contact_tolerance_m=policy.contact_tolerance_m,
        edge_tolerance_m=policy.edge_tolerance_m,engineering_edge_margin_m=policy.engineering_edge_margin_m)
    supported=support_union_audit(landing,supports,contact_tolerance_m=policy.contact_tolerance_m,
        edge_tolerance_m=policy.edge_tolerance_m,engineering_edge_margin_m=policy.engineering_edge_margin_m)
    reserve=receiver_footprint_reserve(landing,[s.body for s in supports],c.budget.receiver_edge_reserve_m,
        contact_tolerance_m=policy.contact_tolerance_m,edge_tolerance_m=policy.edge_tolerance_m)
    audit.update(support= supported,generating_receiver_support=generating,edge_reserve=reserve,
        rotated_vertical_half_extent_m=vertical_half,working_normal_world=normal.tolist())
    if not all(v["supported"] for v in (generating,supported,reserve)):
        return rejected("COMPLETE_BOTTOM_OR_EDGE_RESERVE_REJECTED")
    actual_process,actual_receiver=effective_process_and_receiver(supported["receiver_names"],supports,policy)
    if actual_process!=process or actual_receiver!=receiver_name or family not in policy.allowed_families_by_process[actual_process]:
        return rejected("EFFECTIVE_RECEIVER_OR_WORKING_DIRECTION_REJECTED")
    for b in world:
        if b.category in {"carton","payload"} and b.name!=contact.target_id:
            if landing.signed_distance_obb(b)<policy.occupancy_clearance_m-1e-10:
                return rejected("RECEIVER_OCCUPIED",pair=[landing.name,b.name])
    standoff=float(c.budget.preplace_standoff_m)
    if not np.isfinite(standoff) or standoff<=0:
        return rejected("INVALID_PREPLACE_STANDOFF")
    preplace=OBB(landing.center+up*standoff,landing.half_extents,landing.rotation,landing.name,"payload")
    physical=_finite_se3(preplace.world_from_local@np.linalg.inv(captured),"preplace_contact")
    virtual=_finite_se3(c.virtual_from_physical(physical),"preplace_task_tcp")
    if (not np.allclose(physical@captured,preplace.world_from_local,atol=1e-10,rtol=0)
        or not np.allclose(physical[:3,2],normal,atol=1e-10,rtol=0)
        or not np.allclose(c.physical_from_virtual(virtual),physical,atol=1e-10,rtol=0)):
        raise MoveItUnavailable("PREPLACE_TRANSFORM_ROUNDTRIP_CHANGED")
    # Transform all current native tool solids rigidly to the unique goal, without IK.
    tools,binding=native_tool_collision_boxes(c,start)
    if not tools:
        raise MoveItUnavailable("PREPLACE_TOOL_GEOMETRY_MISSING")
    delta=physical@np.linalg.inv(c.physical_contact_pose(start))
    goal_tools=[]
    for b in tools:
        t=delta@b.world_from_local
        goal_tools.append(OBB(t[:3,3],b.half_extents,t[:3,:3],b.name,b.category))
    for body in [preplace,*goal_tools]:
        for obstacle in world:
            if (body.name in c.native_compliant and obstacle.name in c.stack_carton_names
                and c.collision_policy.compliant_cup_neighbor_contact_mode=="ignore"):
                continue
            failure=obb_pair_failure(body,obstacle,c.collision_policy,c.collision_margin_m,stage="transit")
            if failure:
                return rejected("PREPLACE_BOX_OR_TOOL_CLEARANCE_REJECTED",failure=failure)
            if obstacle.category=="conveyor":
                required=c.collision_policy.pair_clearance("external",c.collision_margin_m)+c.budget.receiver_runtime_clearance_reserve_m
                if not obb_distance_at_least(body,obstacle,required):
                    return rejected("PREPLACE_RECEIVER_RESERVE_REJECTED",pair=[body.name,obstacle.name])
    audit.update(effective_receiver=actual_receiver,occupied_ids=[b.name for b in c.native_scene.occupied],
        tool_solid_count=len(tools),tool_binding=binding,box_and_tool_goal_geometry="CHECKED",
        robot_and_loaded_path="NOT_YET_CHECKED",support_check_scope="FUTURE_LANDING_INTENT_ONLY")
    return PreplaceTarget(contact.target_id,contact.scene_revision,receiver_name,family,
        landing,preplace,physical,virtual,captured,audit),None
