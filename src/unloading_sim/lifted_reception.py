"""Liftable transverse receiver and geometric process templates, never a planner.

All poses are SI/world. Regions describe full-projection reception; swept OBB
screens are conservative and do not qualify a robot path or a physical release.
The original layout, motion entries and release policy are not mutated.
"""
from dataclasses import dataclass
import numpy as np

from .geometry import OBB, make_transform
from .workcell_layout import canonical_digest, _obb_record
from .conveyor_placement import support_union_audit
from .depalletizing import box_is_free_from_stack, minimum_clearance_extraction_distance

MARKERS = dict(status="GEOMETRY_PROCESS_PREVIEW", motion_planning="MOTION_PLANNING_NOT_RUN",
               physical_execution="PHYSICAL_EXECUTION_NOT_RUN", qualification_status="NOT_EVALUATED",
               continuous_swept_proof=False, execution_ready=False)
DEFERRED = {k: "DEFERRED_BY_SCOPE" for k in (
    "raised_belt_docking", "loaded_belt_lowering_sequence", "post_landing_outfeed_length",
    "impact_rebound_tipping_stability", "measured_load_capacity")}
DEFERRED["lift_mechanism_collision_geometry"] = "NOT_MODELED"
HIGH = "HIGH_RECEIVE_SMALL_DROP"
LOW = "LOW_RECEIVE_TOOL_CLEARANCE_DROP"
SCHEMES = {
    "middle_front": ("MIDDLE", [-1., 0., 0.], HIGH),
    "left_front": ("LEFT_END", [-1., 0., 0.], HIGH),
    "left_inner": ("LEFT_END", [0., -1., 0.], LOW),
    "right_inner": ("RIGHT_END", [0., 1., 0.], HIGH),
}


def bounds(box):
    corners = box.corners()
    return corners.min(axis=0), corners.max(axis=0)


def translated(box, delta):
    return OBB(box.center + np.asarray(delta), box.half_extents, box.rotation, box.name, box.category)


@dataclass(frozen=True)
class StartHeight:
    """Value snapshot: moving or mutating the source box cannot change heights."""
    target_id: str
    box_identity: str
    z_b: float
    h: float
    d: float = .05

    @classmethod
    def capture(cls, box, d=.05):
        if (not np.isfinite(box.world_from_local).all() or not np.isfinite(box.half_extents).all()
                or not np.allclose(box.rotation.T@box.rotation,np.eye(3),atol=1e-10,rtol=0)
                or abs(np.linalg.det(box.rotation)-1)>1e-10 or not np.isfinite(d) or d <= 0):
            raise ValueError("INVALID_HEIGHT_INPUT")
        h = 2 * float(np.abs(box.rotation[2]) @ box.half_extents)
        return cls(box.name, canonical_digest(_obb_record(box)), float(box.center[2]-h/2), h, float(d))

    @property
    def hmax(self):
        return self.z_b-self.d

    @property
    def hmin(self):
        return self.z_b-self.h-self.d

    def evidence(self):
        return dict(target_id=self.target_id, start_box_identity=self.box_identity,
                    frozen_at="TASK_START_GEOMETRY", z_b_m=self.z_b, h_m=self.h, d_m=self.d,
                    Hmax_world_m=self.hmax, Hmin_world_m=self.hmin,
                    high_drop_m=self.d, low_drop_m=self.h+self.d,
                    low_first_touch_top_world_m=self.z_b-self.d)


@dataclass(frozen=True)
class LiftedConveyorScene:
    boxes: tuple
    height: dict
    scene_fingerprint: str
    layout_fingerprint: str

    @classmethod
    def build(cls, layout, cartons=None, *, surface_z_world_m=None, mechanical_surface_limits_m=None,
              coupled_lift=False):
        fixed = layout.fixed_components()
        belt = next(b for b in fixed if b.name == "conveyor_transverse")
        if not np.allclose(belt.rotation[:, 2], [0, 0, 1], atol=1e-12, rtol=0):
            raise ValueError("HORIZONTAL_TRANSVERSE_SURFACE_REQUIRED")
        original = float(belt.center[2]+belt.half_extents[2])
        height = original if surface_z_world_m is None else float(surface_z_world_m)
        if not np.isfinite(height):
            raise ValueError("NONFINITE_SURFACE_HEIGHT")
        # First use the established assembly transform, then a WORLD Z translation.
        moved = translated(belt, [0, 0, height-original])
        boxes = tuple(moved if b.name == belt.name else b for b in fixed) + tuple(
            layout.cartons() if cartons is None else cartons)
        if len({b.name for b in boxes}) != len(boxes):
            raise ValueError("DUPLICATE_SCENE_IDENTITY")
        issues = []
        if bounds(moved)[0][2] < layout.data["world"]["floor_z_m"]-1e-12:
            issues.append("BELT_BODY_BELOW_FLOOR")
        limits = mechanical_surface_limits_m
        if limits is not None:
            if len(limits) != 2 or not np.isfinite(limits).all() or limits[0] > limits[1]:
                raise ValueError("INVALID_MECHANICAL_SURFACE_LIMITS")
            if not limits[0] <= height <= limits[1]:
                issues.append("KNOWN_MECHANICAL_OVERTRAVEL")
        evidence = dict(surface_z_world_m=height, original_surface_z_world_m=original,
            lift_displacement_m=height-original, body_thickness_m=2*float(belt.half_extents[2]),
            body_center_z_world_m=float(moved.center[2]), outfeed_direction_world=[0., -1., 0.],
            moving_object_ids=[belt.name], mechanical_travel_status="UNKNOWN" if limits is None else "DECLARED",
            mechanical_surface_limits_m=limits, issues=issues, clamped=False,
            lift_mechanism="NOT_MODELED", scene_version="m710_lifted_reception_geometry_v1")
        if coupled_lift:
            # Explicit new scene; never silently reuse the legacy pure-Z proof.
            from .conveyor_lift import coupled_conveyor_state
            coupled = coupled_conveyor_state(layout, height, cartons)
            boxes = coupled.boxes
            evidence.update(lift_mechanism="SCISSOR_KINEMATIC_CONCEPT",
                scene_version=coupled.mechanism["schema"], coupled_mechanism=coupled.mechanism,
                moving_object_ids=coupled.mechanism["moving_object_ids"],
                native_transition_compatible=False)
        fingerprint = canonical_digest(dict(layout=layout.layout_fingerprint,
            geometry=[_obb_record(b) for b in boxes], height=evidence))
        return cls(boxes, evidence, fingerprint, layout.layout_fingerprint)

    @property
    def belt(self):
        return next(b for b in self.boxes if b.name == "conveyor_transverse")

    def evidence(self):
        # This is also the native adapter's normal OBB serialization, NOT sent to a worker.
        from .moveit2_backend import box_message
        return dict(scene_fingerprint=self.scene_fingerprint, layout_fingerprint=self.layout_fingerprint,
                    height=self.height, world=[box_message(b) for b in self.boxes],
                    render_geometry_source="world", native_world_sent=False)


def layer_role(target, original_cartons, remaining_cartons):
    """Retain original physical boundaries even after middle cartons are removed."""
    originals = {b.name: b for b in original_cartons}
    if target.name not in originals:
        raise ValueError("MISSING_ORIGINAL_LAYER_PROVENANCE")
    original = originals[target.name]
    bottom = bounds(original)[0][2]
    row = sorted((b for b in original_cartons if abs(bounds(b)[0][2]-bottom) < 1e-6),
                 key=lambda b: (b.center[1], b.name))
    ids = [b.name for b in row]
    role = "SINGLE_ORIGINAL" if len(row) == 1 else (
        "RIGHT_END" if target.name == ids[0] else "LEFT_END" if target.name == ids[-1] else "MIDDLE")
    active = {b.name for b in remaining_cartons}
    layers = sorted({round(bounds(b)[0][2], 9) for b in original_cartons})
    return dict(role=role, original_layer_index=layers.index(round(bottom, 9)),
                original_layer_bottom_m=float(bottom), original_y_order=ids,
                remaining_y_order=[n for n in ids if n in active],
                removed_from_original_layer=[n for n in ids if n not in active])


def directional_contact(target, outward):
    """Resolve a WORLD template to a real local box face, including rotated boxes."""
    n = np.asarray(outward, dtype=float)
    projections = target.rotation.T @ n
    axis = int(np.argmax(np.abs(projections)))
    sign = 1 if projections[axis] >= 0 else -1
    if abs(projections[axis]) < 1-1e-10:
        return None
    actual = sign*target.rotation[:, axis]
    x = np.array([0., 0., 1.])
    if abs(x @ actual) > 1e-10:
        return None
    z = -actual
    pose = make_transform(np.column_stack((x, np.cross(z, x), z)),
                          target.center+target.half_extents[axis]*actual)
    return dict(geometric_face_id=f"{'xyz'[axis]}{'+' if sign>0 else '-'}",
                axis=axis, sign=sign, outward_normal=actual.tolist(),
                contact_pose=pose.tolist(), working_axis_into_box=z.tolist(),
                wrist_rule="KEEP_SPATIAL_ORIENTATION_NOT_FIXED_JOINT_VALUES")


def _subtract_rect(rect, cut):
    x0, x1, y0, y1 = rect
    a, b, c, d = max(x0, cut[0]), min(x1, cut[1]), max(y0, cut[2]), min(y1, cut[3])
    if a >= b or c >= d:
        return [rect]
    return [r for r in ([x0, a, y0, y1], [b, x1, y0, y1], [a, b, y0, c], [a, b, d, y1])
            if r[1]-r[0] > 1e-12 and r[3]-r[2] > 1e-12]


def receiving_region(target, belt, occupied=(), *, edge_reserve_m=.01, clearance_m=.005):
    """Exact projection containment in a horizontal rectangular belt.

    For arbitrary carton rotation all eight projected corners are used. Occupancy
    subtraction uses conservative projected bounding rectangles, identified as
    such. No union with the longitudinal belt is used.
    """
    if edge_reserve_m < 0 or clearance_m < 0 or not np.isfinite([edge_reserve_m, clearance_m]).all():
        raise ValueError("INVALID_RECEPTION_MARGIN")
    if not np.allclose(belt.rotation[:, 2], [0, 0, 1], atol=1e-12):
        raise ValueError("HORIZONTAL_RECEIVER_REQUIRED")
    offsets = (target.corners()-target.center) @ belt.rotation
    low, high = offsets.min(axis=0), offsets.max(axis=0)
    limits = [-belt.half_extents[0]+edge_reserve_m-low[0], belt.half_extents[0]-edge_reserve_m-high[0],
              -belt.half_extents[1]+edge_reserve_m-low[1], belt.half_extents[1]-edge_reserve_m-high[1]]
    rects = [list(map(float, limits))] if limits[0] <= limits[1] and limits[2] <= limits[3] else []
    exclusions = []
    h = StartHeight.capture(target)
    surface = belt.center[2]+belt.half_extents[2]
    for other in occupied:
        if other.name in (target.name, belt.name):
            continue
        lo, hi = bounds(other)
        if hi[2] < surface-clearance_m or lo[2] > surface+h.h+clearance_m:
            continue
        pts = (other.corners()-belt.center) @ belt.rotation
        a, b = pts.min(axis=0), pts.max(axis=0)
        cut = [a[0]-high[0]-clearance_m, b[0]-low[0]+clearance_m,
               a[1]-high[1]-clearance_m, b[1]-low[1]+clearance_m]
        updated = [r for rectangle in rects for r in _subtract_rect(rectangle, cut)]
        if updated != rects:
            exclusions.append(dict(id=other.name, rectangle=list(map(float, cut)),
                                   scope="CONSERVATIVE_PROJECTED_OCCUPANCY"))
        rects = updated
    polygons = []
    for a, b, c, d in rects:
        points = np.array([[a,c,0], [b,c,0], [b,d,0], [a,d,0]]) @ belt.rotation.T + belt.center
        points[:, 2] = surface
        polygons.append(points.tolist())
    return dict(rectangles_belt_local_m=rects, polygons_world_m=polygons,
                projection_center_limits_before_occupancy=list(map(float, limits)),
                edge_reserve_m=edge_reserve_m, occupied_exclusions=exclusions,
                geometric_fit=bool(limits[0]<=limits[1] and limits[2]<=limits[3]),
                region_status=("DEFINITE_GEOMETRIC_NO_FIT" if limits[0]>limits[1] or limits[2]>limits[3]
                    else "CONSERVATIVE_OCCUPANCY_NEEDS_CHECK" if exclusions else "GEOMETRIC_REGION_AVAILABLE"),
                robot_reachability="NOT_EVALUATED",sample_pass_does_not_qualify_region=True,
                exists=bool(rects), coverage="ALL_8_CORNERS_HORIZONTAL_PROJECTION",
                scope="RECEPTION_REGION_NOT_ROBOT_REACHABILITY", receiver_id=belt.name)


def swept_envelope(box, delta):
    """Conservative single OBB containing the complete fixed-orientation sweep."""
    delta = np.asarray(delta, dtype=float)
    return OBB(box.center+delta/2, box.half_extents+np.abs(box.rotation.T @ delta)/2,
               box.rotation, box.name, box.category)


def screen_sweep(bodies, delta, obstacles, *, clearance_m=.005, support_ids=(), compliant_ids=(),
                 stack_ids=(), ignore_flexible_neighbors=False, target_id=None, initial_contact=False,
                 maximum_initial_cup_penetration_m=.005):
    issues, contacts = [], []
    for body in bodies:
        envelope = swept_envelope(body, delta)
        for other in obstacles:
            if body.name == other.name:
                continue
            if (ignore_flexible_neighbors and body.name in compliant_ids and other.name in stack_ids
                    and other.name != target_id):
                continue  # Existing named flexible-cup/non-target-stack policy, not rigid geometry.
            if envelope.signed_distance_obb(other) >= clearance_m-1e-10:
                continue
            blo, bhi = bounds(body)
            olo, ohi = bounds(other)
            # Only named, pre-existing planar support and horizontal nonpenetrating
            # extraction. No permission is carried into final separation or drop.
            if (body.name == target_id and other.name in support_ids and abs(delta[2]) < 1e-12
                    and abs(blo[2]-ohi[2]) < 1e-9):
                contacts.append([body.name, other.name, "NAMED_PLANAR_STACK_SUPPORT_HORIZONTAL_ONLY"])
                continue
            # Released carton slides past its already compressed cup plane. This
            # retains only named initial flexible contacts, never rigid inserts.
            if (initial_contact and other.name in compliant_ids and body.name == target_id
                    and body.signed_distance_obb(other) <= .0002
                    and envelope.signed_distance_obb(other) >= -maximum_initial_cup_penetration_m-1e-10
                    and translated(body, delta).signed_distance_obb(other) >= body.signed_distance_obb(other)-1e-10):
                contacts.append([body.name, other.name, "INITIAL_COMPRESSED_CUP_SEPARATION_GEOMETRY_ONLY"])
                continue
            issues.append(dict(pair=[body.name, other.name], status="POSSIBLE_INTERFERENCE",
                               method="CONSERVATIVE_SWEPT_OBB_NOT_EXACT_COLLISION"))
    return dict(status="POSSIBLE_INTERFERENCE" if issues else "CLEAR_CONSERVATIVE_SCREEN",
                issues=issues, named_initial_contacts=contacts)


def side_turn_regions(rectangles, target, tools, obstacles, direction, *, support_ids=(),
                      compliant_ids=(), stack_ids=(), ignore_flexible_neighbors=False, clearance_m=.005):
    """Conservative analytic turn intervals; no fixed box-width lateral step.

    Clip the lateral reachable component containing the initial Y, then subtract
    Y intervals blocked during the complete X leg. AABB enclosures for rotated
    bodies only exclude conservatively; they never certify an exact collision.
    """
    result, blockers = list(rectangles), []
    y0 = target.center[1]
    for body in [target,*tools]:
        lo,hi = bounds(body)
        for obstacle in obstacles:
            if obstacle.name == target.name:
                continue
            if (ignore_flexible_neighbors and body.name in compliant_ids and obstacle.name in stack_ids):
                continue
            a,b = bounds(obstacle)
            if (body.name==target.name and obstacle.name in support_ids and abs(lo[2]-b[2])<1e-9):
                continue
            if lo[2] >= b[2]+clearance_m-1e-10 or hi[2] <= a[2]-clearance_m+1e-10:
                continue
            cut_lo = a[1]-(hi[1]-y0)-clearance_m
            cut_hi = b[1]-(lo[1]-y0)+clearance_m
            before = result
            if lo[0] < b[0]+clearance_m and hi[0] > a[0]-clearance_m:
                clipped=[]
                for x1,x2,y1,y2 in result:
                    if cut_lo < y0 < cut_hi:
                        continue
                    if direction>0 and cut_lo>=y0:
                        y2=min(y2,cut_lo)
                    elif direction<0 and cut_hi<=y0:
                        y1=max(y1,cut_hi)
                    if y1<=y2:
                        clipped.append([x1,x2,y1,y2])
                result=clipped
            clipped=[]
            for x1,x2,y1,y2 in result:
                if x1+(lo[0]-target.center[0]) < b[0]+clearance_m and hi[0]>a[0]-clearance_m:
                    for low,high in ((y1,min(y2,cut_lo)),(max(y1,cut_hi),y2)):
                        if low<high:
                            clipped.append([x1,x2,low,high])
                else:
                    clipped.append([x1,x2,y1,y2])
            result=clipped
            if result != before:
                blockers.append(dict(pair=[body.name,obstacle.name],status="POSSIBLE_INTERFERENCE",
                    method="CONSERVATIVE_CHANNEL_INTERVAL_CLIP"))
    return result,blockers


def drop_geometry(release, tools, belt, obstacles, *, compliant_ids=(), clearance_m=.005,
                  maximum_initial_cup_penetration_m=.005):
    start = StartHeight.capture(release)
    surface = float(belt.center[2]+belt.half_extents[2])
    distance = start.z_b-surface
    if distance < -1e-10:
        return dict(status="RELEASE_BELOW_RECEIVING_SURFACE", drop_m=distance)
    landing = translated(release, [0,0,-distance])
    screen = screen_sweep([release], [0,0,-distance],
        [b for b in obstacles if b.name not in (release.name,belt.name)]+list(tools),
        compliant_ids=compliant_ids, target_id=release.name, initial_contact=True, clearance_m=clearance_m,
        maximum_initial_cup_penetration_m=maximum_initial_cup_penetration_m)
    tool_bottom=min((bounds(b)[0][2] for b in tools), default=None)
    landing_top=float(bounds(landing)[1][2])
    return dict(status=screen["status"], drop_m=distance, release_box=_obb_record(release),
                first_touch_box=_obb_record(landing), tool_motion_world_m=[0.,0.,0.],
                tool_bottom_world_m=tool_bottom, first_touch_top_world_m=landing_top,
                tool_bottom_minus_landing_top_m=None if tool_bottom is None else float(tool_bottom-landing_top),
                vertical_relation_scope="HEIGHT_RELATION_ONLY_NOT_PHYSICAL_HOLD_OR_STABILITY", screen=screen,
                full_bottom_support=support_union_audit(landing,[belt]),
                physics="NOT_EVALUATED", receiver_union_used=False)


def process_scheme(layout, target, originals, remaining, scheme, *, tool_in_contact=(),
                   compliant_ids=(), reference_robot=(), edge_reserve_m=.01, clearance_m=.005,
                   old_maximum_extraction_m=.8, mechanical_surface_limits_m=None,
                   ignore_flexible_neighbors=False, planar_stack_contact_allowed=False,
                   final_separation_m=.0052, maximum_initial_cup_penetration_m=.005):
    role, normal, mode = SCHEMES[scheme]
    snapshot = StartHeight.capture(target)
    height = snapshot.hmin if mode == LOW else snapshot.hmax
    scene = LiftedConveyorScene.build(layout, remaining, surface_z_world_m=height,
                                     mechanical_surface_limits_m=mechanical_surface_limits_m)
    provenance = layer_role(target, originals, remaining)
    face = directional_contact(target, normal)
    region = receiving_region(target, scene.belt, scene.boxes, edge_reserve_m=edge_reserve_m,
                              clearance_m=clearance_m)
    out = dict(**MARKERS, scheme=scheme, target_id=target.name, heights=snapshot.evidence(),
        mode=mode, role=provenance, face=face, scene=scene.evidence(), receiving_region=region,
        deferred=dict(DEFERRED), applicability="INFORMATION_INSUFFICIENT", reasons=[],
        rejection_scope="THIS_GEOMETRIC_TEMPLATE_ONLY_NOT_CARTON_UNLOADABILITY",
        robot_path="NOT_EVALUATED_NO_Q_TRAJECTORY", tool_geometry_count=len(tool_in_contact),
        template_region_world_rectangles=[], display_sample=None, tool_initial=[_obb_record(b) for b in tool_in_contact])
    if role != provenance["role"]:
        out["reasons"].append("ORIGINAL_LAYER_ROLE_MISMATCH")
    if face is None:
        out["reasons"].append("WORLD_DIRECTION_NOT_ALIGNED_WITH_A_PHYSICAL_FACE")
    # The process directions are world X/Y. Arbitrary yaw belts still support
    # full projection regions; this first direction-template clipping is explicit.
    aligned = np.allclose(scene.belt.rotation, np.eye(3), atol=1e-12)
    if not aligned:
        out["reasons"].append("DIRECTION_TEMPLATE_REQUIRES_WORLD_ALIGNED_RECEIVER")
    boxes = [b for b in scene.boxes if b.name != target.name]
    blo = bounds(target)[0]
    supports = [b.name for b in remaining if b.name != target.name and abs(bounds(b)[1][2]-blo[2]) < 1e-9
                and all(min(bounds(target)[1][i],bounds(b)[1][i])-max(blo[i],bounds(b)[0][i]) > 1e-9 for i in (0,1))]
    stack = [b for b in remaining if b.name != target.name]
    out["initial_support_ids"] = supports
    planar = [b.name for b in stack if abs(bounds(b)[1][2]-blo[2])<1e-9] if planar_stack_contact_allowed else supports
    out["intermediate_planar_contact_ids"] = planar
    out["intermediate_contact_policy"] = (
        "EXISTING_PLANNER_RELAXED_PHYSICS_CHECKED; ONLY_ZERO_PENETRATION_PLANAR_HORIZONTAL_CONTACT"
        if planar_stack_contact_allowed else "NAMED_INITIAL_SUPPORT_ONLY")
    # Full final separation is required only after the second leg. Conservative
    # world-X separation from all remaining stack boxes is an explicit bound.
    radius = np.abs(target.rotation) @ target.half_extents
    x_limit = min((bounds(b)[0][0]-radius[0]-clearance_m for b in stack), default=target.center[0])
    if aligned:
        cx, cy = scene.belt.center[:2]
        for a,b,c,d in region["rectangles_belt_local_m"]:
            a,b,c,d = a+cx,min(b+cx,x_limit,target.center[0]),c+cy,d+cy
            if scheme.endswith("front"):
                if not c-1e-12 <= target.center[1] <= d+1e-12:
                    continue
                c=d=float(target.center[1])
            elif normal[1]<0:
                d=min(d,float(target.center[1])-1e-9)
            else:
                c=max(c,float(target.center[1])+1e-9)
            if a<=b and c<=d:
                out["template_region_world_rectangles"].append([float(a),float(b),float(c),float(d)])
    if "inner" in scheme:
        out["reception_direction_region_before_channel"] = list(out["template_region_world_rectangles"])
        intervals,blockers = side_turn_regions(out["template_region_world_rectangles"],target,tool_in_contact,boxes,
            normal[1],support_ids=planar,compliant_ids=compliant_ids,stack_ids={b.name for b in remaining},
            ignore_flexible_neighbors=ignore_flexible_neighbors,clearance_m=clearance_m)
        out["template_region_world_rectangles"]=intervals
        out["channel_screen_blockers"]=blockers
    out["direction_sequence"] = [normal, [-1.,0.,0.]] if "inner" in scheme else [normal]
    out["turn_y_intervals_world_m"] = [r[2:] for r in out["template_region_world_rectangles"]] if "inner" in scheme else []
    out["turn_interval_scope"] = "RECEPTION_AND_CONSERVATIVE_FULL_TWO_LEG_CHANNEL_CLIPPING"
    out["belt_environment_screen"] = screen_sweep([scene.belt],[0,0,0],scene.boxes)
    out["initial_robot_belt_screen"] = screen_sweep(reference_robot,[0,0,0],[scene.belt]) if reference_robot else dict(status="NOT_EVALUATED")
    if not region["exists"]:
        out["reasons"].append("NO_FULL_PROJECTION_RECEIVING_REGION")
    if not out["template_region_world_rectangles"]:
        out["reasons"].append("DIRECTION_CANNOT_ENTER_FULL_RECEIVING_REGION")
    out["reasons"].extend(scene.height["issues"])
    if out["reasons"]:
        conservative=bool(region["occupied_exclusions"] or out.get("channel_screen_blockers"))
        only_screen=set(out["reasons"]) <= {"NO_FULL_PROJECTION_RECEIVING_REGION","DIRECTION_CANNOT_ENTER_FULL_RECEIVING_REGION"}
        out["applicability"]="POSSIBLE_INTERFERENCE_NEEDS_CHECK" if conservative and only_screen and region["geometric_fit"] else "TEMPLATE_NOT_APPLICABLE"
        return out
    # One display sample, no IK target. Retain the entire region above.
    a,b,c,d = out["template_region_world_rectangles"][0]
    sample = np.array([(a+b)/2,(c+d)/2,target.center[2]])
    points = [target.center.copy()]
    if "inner" in scheme:
        points.append(np.array([target.center[0],sample[1],target.center[2]]))
    points.append(sample)
    out["display_sample"] = dict(marker="DISPLAY_SAMPLE_NOT_UNIQUE_TARGET", center_m=sample.tolist(),
                                 geometric_waypoints_m=[p.tolist() for p in points], robot_q=None)
    screens=[]
    bodies=[target,*tool_in_contact]
    for start,end in zip(points,points[1:]):
        moved=[translated(body,start-target.center) for body in bodies]
        screens.append(screen_sweep(moved,end-start,boxes,clearance_m=clearance_m,support_ids=planar,
            target_id=target.name,compliant_ids=compliant_ids,stack_ids={b.name for b in remaining},
            ignore_flexible_neighbors=ignore_flexible_neighbors))
    out["extraction_screens"]=screens
    released = translated(target,sample-target.center)
    tools = [translated(body,sample-target.center) for body in tool_in_contact]
    out["final_stack_separation"] = box_is_free_from_stack(released,stack,free_space_clearance_m=final_separation_m)
    out["final_separation_required_m"] = final_separation_m
    # The rigid tool/held box relationship remains fixed. Flexible contact is
    # covered separately by the existing seal-ring and nominal compression model.
    out["rigid_tool_target_screen"] = screen_sweep([b for b in tool_in_contact if b.name not in compliant_ids],
        [0,0,0],[target],clearance_m=0.)
    out["drop"] = drop_geometry(released,tools,scene.belt,boxes,compliant_ids=compliant_ids,clearance_m=clearance_m,
                               maximum_initial_cup_penetration_m=maximum_initial_cup_penetration_m)
    out["stack_only_normal_separation_m"] = minimum_clearance_extraction_distance(target,normal,stack,
        free_space_clearance_m=clearance_m,maximum_distance_m=old_maximum_extraction_m)
    distance = sum(float(np.linalg.norm(y-x)) for x,y in zip(points,points[1:]))
    out["display_extraction_length_m"]=distance
    out["old_planner_bound"] = dict(maximum_extraction_m=old_maximum_extraction_m,
        sample_exceeds_old_bound=distance>old_maximum_extraction_m, old_planner_changed=False)
    issues = any(s["issues"] for s in screens) or out["drop"]["status"]!="CLEAR_CONSERVATIVE_SCREEN"
    issues |= out["belt_environment_screen"]["status"]!="CLEAR_CONSERVATIVE_SCREEN"
    issues |= out["initial_robot_belt_screen"]["status"]!="CLEAR_CONSERVATIVE_SCREEN"
    issues |= out["rigid_tool_target_screen"]["status"]!="CLEAR_CONSERVATIVE_SCREEN"
    issues |= not out["drop"]["full_bottom_support"]["supported"]
    if not out["final_stack_separation"]:
        out["reasons"].append("FINAL_STACK_SEPARATION_INSUFFICIENT")
        out["applicability"]="TEMPLATE_NOT_APPLICABLE"
    elif issues:
        out["applicability"]="POSSIBLE_INTERFERENCE_NEEDS_CHECK"
    elif not tool_in_contact:
        out["reasons"].append("TOOL_GEOMETRY_NOT_AVAILABLE")
    else:
        out["applicability"]="DISPLAY_SAMPLE_GEOMETRIC_SCREEN_CLEAR"
    return out
