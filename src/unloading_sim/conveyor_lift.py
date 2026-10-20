"""Project-owned scissor geometry and fixed-length conveyor coupling (SI).

No dynamics, actuator ratings, motion approval or implicit change to the old
native pure-Z receiver contract. Every returned collider has the displayed pose.
"""
from dataclasses import dataclass
import numpy as np

from .geometry import OBB
from .workcell_layout import canonical_digest, _obb_record


@dataclass(frozen=True)
class ScissorSpec:
    stages: int
    arm_length_m: float
    arm_depth_m: float = .035
    arm_thickness_m: float = .02

    def validate(self):
        if (type(self.stages) is not int or not 1 <= self.stages <= 8 or
                not np.isfinite([self.arm_length_m, self.arm_depth_m, self.arm_thickness_m]).all() or
                min(self.arm_length_m, self.arm_depth_m, self.arm_thickness_m) <= 0 or
                self.arm_depth_m >= self.arm_length_m):
            raise ValueError("INVALID_SCISSOR_SPEC")


def _box(name, center, size, rotation=None):
    return OBB(np.asarray(center), np.asarray(size)/2,
               np.eye(3) if rotation is None else rotation, name, "lift_mechanism")


def scissor_stack(name, anchor, direction, top_z, spec, *, floor_z=0.):
    """Fixed lower/upper left pins; right pins slide on horizontal rails.

    Two arm lanes avoid intersecting solid bars at the crossing. Hinge pins
    bridge the lanes. The dimensions are concept geometry, not rated hardware.
    """
    spec.validate()
    anchor, direction = np.asarray(anchor, float), np.asarray(direction, float)
    if (anchor.shape != (3,) or direction.shape != (3,) or
            not np.isfinite([*anchor, *direction, top_z, floor_z]).all() or
            abs(direction[2]) > 1e-12 or abs(np.linalg.norm(direction)-1) > 1e-12):
        raise ValueError("INVALID_SCISSOR_FRAME")
    rise = (top_z-anchor[2])/spec.stages
    if not spec.arm_depth_m < rise < spec.arm_length_m-spec.arm_depth_m:
        raise ValueError("SCISSOR_GEOMETRIC_RANGE_EXCEEDED")
    span = float(np.sqrt(spec.arm_length_m**2-rise**2))
    axis = np.cross([0., 0., 1.], direction)
    lane = spec.arm_thickness_m*.65
    boxes, bars, hinges = [], [], []
    for level in range(spec.stages):
        left = anchor+[0, 0, level*rise]
        right = left+span*direction
        for side, (a, b) in enumerate(((left, right+[0,0,rise]), (right, left+[0,0,rise]))):
            a, b = a+axis*lane*(2*side-1), b+axis*lane*(2*side-1)
            tangent = (b-a)/spec.arm_length_m
            rotation = np.column_stack((tangent, axis, np.cross(tangent, axis)))
            key = f"{name}_arm_{level}_{side}"
            boxes.append(_box(key, (a+b)/2,
                              [spec.arm_length_m, spec.arm_thickness_m, spec.arm_depth_m], rotation))
            bars.append(dict(id=key, endpoints=[a.tolist(), b.tolist()], length_m=spec.arm_length_m))
        center = (left+right)/2+[0,0,rise/2]
        pin_rotation = np.column_stack((direction, axis, [0.,0.,1.]))
        boxes.append(_box(f"{name}_pin_{level}", center,
                          [spec.arm_depth_m, 4*lane, spec.arm_depth_m], pin_rotation))
        hinges.append(dict(id=f"{name}_hinge_{level}", type="REVOLUTE",
                           bodies=[f"{name}_arm_{level}_0", f"{name}_arm_{level}_1"],
                           position=center.tolist(), axis=axis.tolist()))
    for level in range(spec.stages+1):
        for side in (0,1):
            pivot = anchor+direction*span*side+[0,0,level*rise]
            boxes.append(_box(f"{name}_end_pin_{level}_{side}", pivot,
                              [spec.arm_depth_m, 4*lane, spec.arm_depth_m], pin_rotation))
    for end, z in (("base", anchor[2]), ("head", top_z)):
        center = anchor+direction*spec.arm_length_m/2
        center[2] = z + (-1 if end == "base" else 1)*spec.arm_depth_m
        boxes.append(_box(f"{name}_{end}_rail", center,
                          [spec.arm_length_m+.08, .08, .03], pin_rotation))
    foot_height=anchor[2]-spec.arm_depth_m-.015-floor_z
    if foot_height <= 0:
        raise ValueError("SCISSOR_BASE_BELOW_FLOOR")
    for side in (0,1):
        foot=anchor+direction*spec.arm_length_m*side
        foot[2]=floor_z+foot_height/2
        boxes.append(_box(f"{name}_foot_{side}",foot,[.10,.10,foot_height]))
    return boxes, dict(id=name, stages=spec.stages, span_m=span, stage_rise_m=rise,
                      angle_rad=float(np.arcsin(rise/spec.arm_length_m)), bars=bars,
                      hinges=hinges, sliding_pin_travel_m=spec.arm_length_m-span,
                      endpoint_joints="LEFT_PIN_RIGHT_SLIDER_WITH_SWIVEL_HEAD")


@dataclass(frozen=True)
class CoupledConveyorState:
    boxes: tuple
    mechanism: dict
    scene_fingerprint: str

    def evidence(self):
        from .moveit2_backend import box_message
        return dict(world=[box_message(b) for b in self.boxes], mechanism=self.mechanism,
                    scene_fingerprint=self.scene_fingerprint,
                    status="COUPLED_CONVEYOR_KINEMATIC_MODEL_NOT_EXECUTABLE",
                    qualification_status="NOT_EVALUATED", continuous_swept_proof=False,
                    physical_execution="NOT_RUN", native_motion_approval="NOT_EVALUATED")


def coupled_conveyor_state(layout, surface_z_world_m, cartons=None, *, legacy_geometry=False):
    """Near longitudinal tip follows height; rear top-surface hinge is fixed.

    Its near tip slides in X, rather than stretching a rigid 2.8 m conveyor.
    Separate scissor supports follow the tilted underside with swivel/sliding
    heads. This explicit opt-in scene does not reuse native pure-Z sweep proof.
    """
    if not legacy_geometry and "coupling" in layout.data["conveyors"]:
        return _seam_coupled_state(layout, surface_z_world_m, cartons)
    fixed = layout.fixed_components()
    transverse = next(b for b in fixed if b.name == "conveyor_transverse")
    longitudinal = next(b for b in fixed if b.name == "conveyor_longitudinal")
    if any(not np.allclose(b.rotation, np.eye(3), atol=1e-12, rtol=0)
           for b in (transverse, longitudinal)):
        raise ValueError("COUPLED_LIFT_REQUIRES_AXIS_ALIGNED_LAYOUT")
    height = float(surface_z_world_m)
    if not np.isfinite(height):
        raise ValueError("NONFINITE_SURFACE_HEIGHT")
    length, width, thickness = 2*longitudinal.half_extents
    rear = longitudinal.center+[-length/2, 0, thickness/2]
    rise = height-rear[2]
    if rise < -1e-12 or rise >= length:
        raise ValueError("LONGITUDINAL_FIXED_LENGTH_RANGE_EXCEEDED")
    theta = float(np.arcsin(max(0., rise)/length))
    c, s = np.cos(theta), np.sin(theta)
    rotation = np.array([[c,0,-s], [0,1,0], [s,0,c]])
    near = rear+rotation@np.array([length,0,0])
    long = OBB(rear+rotation@np.array([length/2,0,-thickness/2]),
               longitudinal.half_extents, rotation, longitudinal.name, longitudinal.category)
    trans = OBB([*transverse.center[:2],height-transverse.half_extents[2]],
                transverse.half_extents, transverse.rotation, transverse.name, transverse.category)
    # All parts originate from the configured assembly/floor, not historical q.
    floor = float(layout.data["world"]["floor_z_m"])
    base_z = floor+.10
    parts, stacks = [], []
    for side in (-1, 1):
        x = transverse.center[0]+side*(transverse.half_extents[0]-.12)
        anchor = [x, transverse.center[1]-.625, base_z]
        items, info = scissor_stack(f"lift_transverse_{'left' if side<0 else 'right'}",
            anchor, [0,1,0], height-2*transverse.half_extents[2]-.07, ScissorSpec(2,1.25),floor_z=floor)
        parts.extend(items); stacks.append(info)
        parts.append(_box(f"{info['id']}_mount",[x,transverse.center[1],height-2*transverse.half_extents[2]-.01],
                          [.08,1.25,.02]))
    # Two narrower four-stage supports under the longitudinal frame. Their
    # different heights follow the real inclined underside, not a flat platform.
    station = rear[0]+length*.70
    for side in (-1, 1):
        x = station+side*.12
        if not rear[0] < x < near[0]:
            raise ValueError("LONGITUDINAL_SUPPORT_OUTSIDE_FRAME")
        underside = rear[2]+(x-rear[0])*np.tan(theta)-thickness/c
        items, info = scissor_stack(f"lift_longitudinal_{'rear' if side<0 else 'near'}",
            [x, rear[1]-(.30 if legacy_geometry else .32), base_z], [0,1,0],
            underside-(.07 if legacy_geometry else .09), ScissorSpec(4,.60,.025,.018),floor_z=floor)
        parts.extend(items); stacks.append(info)
        parts.append(_box(f"{info['id']}_swivel_head",np.array([x,rear[1],underside])-rotation[:,2]*.01,
                          [.08,.60,.02],rotation))
        if not legacy_geometry:
            # Central neck joins the lowered horizontal rail to the swivel.
            # Narrow in X to clear the tilted underside at the maximum angle.
            parts.append(_box(f"{info['id']}_neck", [x,rear[1]-.02,underside-.04], [.02,.50,.05]))
    # Rear stand and bearing keep the same world pose at every lift height.
    for side in (-1,1):
        stand_height = rear[2]-floor-.05
        if stand_height <= 0:
            raise ValueError("INVALID_REAR_STAND_HEIGHT")
        parts.append(_box(f"lift_rear_stand_{side+1}",
            [rear[0]-(0 if legacy_geometry else .12), rear[1]+side*(width/2-.06),
             floor+stand_height/2], [.08,.08,stand_height]))
    parts.append(_box("lift_rear_bearing", rear+([0,0,0] if legacy_geometry else [-.06,0,0]),
                      [.12,width,.10] if legacy_geometry else [.24,width-.02,.10]))
    boxes = tuple(trans if b.name==trans.name else long if b.name==long.name else b for b in fixed)
    boxes += tuple(layout.cartons() if cartons is None else cartons)+tuple(parts)
    if len({b.name for b in boxes}) != len(boxes):
        raise ValueError("DUPLICATE_SCENE_IDENTITY")
    fixed_structure=[b.name for b in parts if "_foot_" in b.name or "_base_rail" in b.name
                     or b.name.startswith("lift_rear_")]
    moving=[trans.name,long.name]+[b.name for b in parts if b.name not in fixed_structure]
    xmin, xmax = transverse.center[0]+np.array([-1,1])*transverse.half_extents[0]
    info = dict(schema="m710_coupled_scissor_v1", geometry_revision="historical_rejected_v1" if legacy_geometry else "rear_offset_neck_v2",
        design_basis=dict(rear_support="80mm posts behind the rotating frame, continuous enlarged bearing housing at unchanged pivot",
            chassis_relation="INDEPENDENT_FLOOR_SUPPORTED_WITH_CLEARANCE", longitudinal_feet_y_offset_m=-.02,
            head="20mm lower rail with central neck to swivel; tilted-frame clearance"), height_m=height, tilt_rad=theta,
        longitudinal_length_m=float(length), rear_hinge_world=rear.tolist(), near_tip_world=near.tolist(),
        near_tip_slide_x_m=float(near[0]-(rear[0]+length)), rear_fixed=True,
        near_tip_within_transverse_x=bool(xmin <= near[0] <= xmax),
        coupling="FIXED_REAR_PIVOT_NEAR_HEIGHT_FOLLOW_WITH_X_SLIDE",
        moving_object_ids=moving, fixed_structure_ids=fixed_structure,
        handoff_qualification="NOT_EVALUATED", stacks=stacks,
        geometry_origin="PROJECT_CONCEPT_NOT_NVIDIA_COMPLETE_SCISSOR_ASSET",
        mechanical_stroke_rating="UNKNOWN", actuator_force_speed="UNKNOWN",
        mass_inertia="NOT_SPECIFIED", full_sweep_collision="NOT_EVALUATED",
        native_receiver_transition="UNSUPPORTED_BY_LEGACY_PURE_Z_CONTRACT")
    identity = canonical_digest(dict(layout=layout.layout_fingerprint, mechanism=info,
                                     geometry=[_obb_record(b) for b in boxes]))
    return CoupledConveyorState(boxes, info, identity)


def _seam_coupled_state(layout, surface_z_world_m, cartons):
    from .concept_solid import relieved_transverse
    fixed=layout.fixed_components()
    transverse=next(b for b in fixed if b.name=="conveyor_transverse")
    longitudinal=next(b for b in fixed if b.name=="conveyor_longitudinal")
    if any(not np.allclose(b.rotation,np.eye(3),atol=1e-12,rtol=0) for b in (transverse,longitudinal)):
        raise ValueError("COUPLED_LIFT_REQUIRES_AXIS_ALIGNED_LAYOUT")
    height=float(surface_z_world_m)
    length,width,thickness=2*longitudinal.half_extents
    low=float(layout.data["assembly"]["world_xyz_m"][2]+layout.data["conveyors"]["surface_z_a_m"])
    limit=float(layout.data["conveyors"]["coupling"]["maximum_surface_height_world_m"])
    rise=height-low
    if (not np.isfinite([height,limit,rise]).all() or rise<0 or height>limit or
            not low<limit<low+length or length*length-rise*rise<=0):
        raise ValueError("LONGITUDINAL_FIXED_LENGTH_RANGE_EXCEEDED")
    dx=float(np.sqrt(length*length-rise*rise))
    c,s=dx/length,rise/length
    theta=float(np.arctan2(rise,dx))
    rotation=np.array([[c,0,-s],[0,1,0],[s,0,c]])
    upper=np.array([transverse.center[0]-transverse.half_extents[0],longitudinal.center[1],height])
    lower=upper-np.array([dx,0,rise])
    if abs((transverse.center[1]-transverse.half_extents[1])-
           (longitudinal.center[1]-width/2))>1e-9:
        raise ValueError("CONVEYOR_YMIN_ALIGNMENT_INVALID")
    trans0=OBB([*transverse.center[:2],height-transverse.half_extents[2]],
        transverse.half_extents,np.eye(3),transverse.name,transverse.category)
    max_rise=limit-low
    slope=max_rise/np.sqrt(length*length-max_rise*max_rise)
    trans=relieved_transverse(trans0,slope)
    long=OBB((upper+lower)/2-rotation[:,2]*thickness/2,
        longitudinal.half_extents,rotation,longitudinal.name,longitudinal.category)
    floor=float(layout.data["world"]["floor_z_m"]);base_z=floor+.10
    parts=[];stacks=[];mates=[]
    for side in (-1,1):
        x=transverse.center[0]+side*(transverse.half_extents[0]-.12)
        items,info=scissor_stack(f"lift_transverse_{'left' if side<0 else 'right'}",
            [x,transverse.center[1]-.625,base_z],[0,1,0],
            height-2*transverse.half_extents[2]-.07,ScissorSpec(2,1.25),floor_z=floor)
        parts.extend(items);stacks.append(info)
        parts.append(_box(f"{info['id']}_mount",[x,transverse.center[1],height-.13],[.08,1.25,.02]))
    # Fixed floor stations; heads swivel and slide against the inclined frame.
    station=upper[0]-length*.40
    for side in (-1,1):
        x=station+side*.12
        if not lower[0]<x<upper[0]:raise ValueError("LONGITUDINAL_SUPPORT_OUTSIDE_FRAME")
        underside=low+(x-lower[0])*s/c-thickness/c
        items,info=scissor_stack(f"lift_longitudinal_{'rear' if side<0 else 'near'}",
            [x,lower[1]-.32,base_z],[0,1,0],underside-.09,ScissorSpec(4,.60,.025,.018),floor_z=floor)
        parts.extend(items);stacks.append(info)
        parts.append(_box(f"{info['id']}_swivel_head",np.array([x,lower[1],underside])-rotation[:,2]*.01,
            [.08,.60,.02],rotation))
        parts.append(_box(f"{info['id']}_neck",[x,lower[1]-.02,underside-.04],[.02,.50,.05]))
    # The rear floor is finite. A fixed cantilever rail reaches outside the
    # opening; both support stations stand on the existing floor inside it.
    low_x=upper[0]-length
    travel=length-np.sqrt(length*length-max_rise*max_rise)
    rail_min=low_x-.25
    floor_min=float(layout.data["trailer"]["opening_x_m"])
    rail_max=max(low_x+travel+.18,floor_min+.30)
    rail_z=low-.20
    if rail_min>=rail_max or rail_z-.02<=floor:raise ValueError("INVALID_SLIDER_RAIL")
    for side in (-1,1):
        y=lower[1]+side*(width/2-.08);key=f"lift_lower_rail_{side+1}"
        parts.append(_box(key,[(rail_min+rail_max)/2,y,rail_z],[rail_max-rail_min,.06,.04]))
        for index,x in enumerate((floor_min+.07,rail_max-.07)):
            foot=f"{key}_stand_{index}";h=rail_z-.02-floor
            parts.append(_box(foot,[x,y,floor+h/2],[.08,.08,h]))
            mates.extend([(foot,key,"FIXED_RAIL_STAND"),(foot,"trailer_floor","FOOT_ON_EXISTING_FLOOR")])
        carriage=f"lift_lower_carriage_{side+1}"
        parts.append(_box(carriage,[lower[0]-.14,y,low-.14],[.16,.10,.08]))
        mates.extend([(carriage,key,"HORIZONTAL_X_SLIDER"),(carriage,"lift_lower_bearing","SLIDER_BEARING")])
    parts.append(_box("lift_lower_bearing",lower+[-.04,0,-.05],[.20,width-.04,.10]))
    mates.append(("lift_lower_bearing","conveyor_longitudinal","LOWER_REVOLUTE_BEARING"))
    boxes=tuple(trans if b.name==trans.name else long if b.name==long.name else b for b in fixed)
    boxes+=tuple(layout.cartons() if cartons is None else cartons)+tuple(parts)
    if len({b.name for b in boxes})!=len(boxes):raise ValueError("DUPLICATE_SCENE_IDENTITY")
    fixed_ids=[b.name for b in parts if "_foot_" in b.name or "_base_rail" in b.name or b.name.startswith("lift_lower_rail_")]
    moving=[trans.name,long.name]+[b.name for b in parts if b.name not in fixed_ids]
    info=dict(schema="m710_coupled_scissor_v3",geometry_revision="upper_seam_lower_slider_v3",
        height_m=height,tilt_rad=theta,longitudinal_length_m=float(length),rear_fixed=False,
        rear_hinge_world=lower.tolist(),near_tip_world=upper.tolist(),near_tip_slide_x_m=0.,
        lower_slide_x_m=float(lower[0]-low_x),near_tip_within_transverse_x=True,
        coupling="UPPER_SEAM_Z_FOLLOW_LOWER_HORIZONTAL_X_SLIDER",
        moving_object_ids=moving,fixed_structure_ids=fixed_ids,stacks=stacks,
        additional_mates=mates,maximum_surface_height_world_m=limit,
        seam=dict(effective_surface_gap_m=0.,upper_edge_world=upper.tolist(),
            ymin_world_m=float(transverse.center[1]-transverse.half_extents[1]),
            underside_relief_slope=float(slope),maximum_underside_cutback_m=float(slope*2*transverse.half_extents[2]),
            definition="COINCIDENT_TOP_EDGE_WITH_FIXED_CONVEX_UNDERSIDE_RELIEF",
            contact_rule="ONLY_NONPENETRATING_DECLARED_SEAM"),
        design_basis=dict(transverse_support="UNCHANGED_1.25M_BARS_AND_CENTRAL_MOUNTS_WITH_CANTILEVER_TOP",
            lower_support="MOVING_CARRIAGE_ON_FIXED_CANTILEVER_RAILS; FEET_ON_FINITE_EXISTING_FLOOR",
            solid_source="SAME_CONVEX_MESH_FOR_DISPLAY_NATIVE_AND_USD; OBB_IS_OUTWARD_BROAD_PHASE"),
        geometry_origin="PROJECT_CONCEPT_NOT_MANUFACTURING_SPECIFICATION",
        mechanical_stroke_rating="UNKNOWN",actuator_force_speed="UNKNOWN",mass_inertia="NOT_SPECIFIED",
        handoff_qualification="NOT_EVALUATED",full_sweep_collision="NOT_EVALUATED",
        native_receiver_transition="DISCRETE_COUPLED_V3_NOT_PURE_Z_SWEEP")
    identity=canonical_digest(dict(layout=layout.layout_fingerprint,mechanism=info,geometry=[_obb_record(b) for b in boxes]))
    return CoupledConveyorState(boxes,info,identity)
