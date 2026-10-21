"""ECO65 measured belts and project-owned fixed-length coupled supports.

``upper_seam_lower_slider_v3`` is the kinematic relation, not a FANUC scale
factor. Source belt convex vertices are baked in D; they are recovered into
the surface-centred part frame once, before any moving rigid transform.
The same native objects are the display and USD solids.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import numpy as np
import yaml

from .model import ROOT, transform
from ..moveit2_backend import digest

CONFIG = ROOT / "configs/workcells/eco65_coupled_carton_v1.yaml"
REVISION = "upper_seam_lower_slider_v3"
OWNERS = {"conveyor_transverse", "conveyor_longitudinal"}


def _pose(center, rotation=None):
    p = transform(center)
    if rotation is not None:
        p[:3, :3] = rotation
    return p.tolist()


def _box(name, center, size, *, rotation=None, role="lift_structure", owner="coupled_receiver"):
    return dict(id=name, pose=_pose(center, rotation), size=list(map(float, size)),
                category="environment", owner=owner, role=role,
                rgba=([.16, .20, .24, 1] if role == "conveyor_frame"
                      else [.34, .44, .53, 1]))


def _hull(vertices):
    from scipy.spatial import ConvexHull
    vertices = np.unique(np.asarray(vertices, float), axis=0)
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    reverse = np.einsum("ij,ij->i", normals, hull.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, ::-1]
    # Qhull preserves all input points in ``points``, including interior and
    # collinear cut-edge points which are not used by any boundary face. FCL's
    # closed Convex contract requires every stored vertex to be connected.
    used, remap = np.unique(faces, return_inverse=True)
    vertices, faces = vertices[used], remap.reshape(faces.shape)
    return dict(vertices=vertices.tolist(), triangles=faces.tolist())


def _clip_convex(mesh, normal, offset):
    """Retain n.v >= offset; actual edge intersections preserve the solid."""
    vertices = np.asarray(mesh["vertices"], float)
    values = vertices @ np.asarray(normal) - offset
    if np.min(values) >= -1e-12:
        return deepcopy(mesh)
    if np.max(values) < -1e-12:
        return None
    points = list(vertices[values >= -1e-12])
    edges = set()
    for a, b, c in mesh["triangles"]:
        edges.update(tuple(sorted(pair)) for pair in ((a, b), (b, c), (c, a)))
    for a, b in edges:
        if values[a] * values[b] < 0:
            points.append(vertices[a] + values[a]/(values[a]-values[b]) * (vertices[b]-vertices[a]))
    return _hull(points)


def _relieved_frame(xsize, ysize, top_depth, depth, slope):
    # Top surface at -top_depth, lower seam edge relieved by the maximum
    # operating slope. This fixed solid is translated, never resized with H.
    x0, x1 = -xsize/2, xsize/2
    vertices = [[x, y, z] for z, xmin in ((-top_depth, x0+slope*top_depth),
                                         (-depth, x0+slope*depth))
                for x in (xmin, x1) for y in (-ysize/2, ysize/2)]
    return _hull(vertices)


def _validate_local_belts(spec):
    """Reject translated/rotated templates, including the old rebuild defect.

    Mesh coordinates have STL float32 precision. Validate actual aggregate
    contours, independently of their placement metadata and frame poses.
    """
    templates = spec['belt_templates']
    ids = [row['id'] for row in templates]
    if len(ids) != len(set(ids)) or {row['owner'] for row in templates} != OWNERS:
        raise ValueError('ECO65_LOCAL_BELT_INVENTORY_INVALID')
    tol = 1e-6
    for owner in sorted(OWNERS):
        v = np.concatenate([np.asarray(row['convex_mesh']['vertices'], float)
                            for row in templates if row['owner'] == owner])
        x, y = ((spec['transverse_width_m'], spec['transverse_length_m'])
                if owner == 'conveyor_transverse' else
                (spec['longitudinal_length_m'], spec['transverse_width_m']))
        bounds = np.array([v.min(axis=0), v.max(axis=0)])
        expected = np.array([[-x/2, -y/2, -spec['belt_depth_m']], [x/2, y/2, 0.]])
        relief = spec['seam_relief_slope']*spec['belt_depth_m'] if owner == 'conveyor_transverse' else 0.
        if (not np.all(np.isfinite(v))
                or not -tol <= bounds[0, 0]-expected[0, 0] <= relief+tol
                or not np.allclose(bounds[0, 1:], expected[0, 1:], atol=tol, rtol=0)
                or not np.allclose(bounds[1], expected[1], atol=tol, rtol=0)):
            raise ValueError('ECO65_BELT_NOT_SURFACE_CENTRED_LOCAL:'+owner)


def make_coupled_spec(source_scene, config=None):
    """Freeze the measured effective belts and replaced structural ownership."""
    if config is None:
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    elif isinstance(config, (str, Path)):
        config = yaml.safe_load(Path(config).read_text(encoding="utf-8"))
    c = deepcopy(config)
    if (c.get("schema") != "eco65_coupled_carton_config_v1"
            or c.get("geometry_revision") != REVISION or c.get("enable_hardware") is not False
            or c.get("box_size_m") != [.15, .1, .1] or c.get("usable_belt_width_m") != .2):
        raise ValueError("ECO65_COUPLED_CONFIG_INVALID")
    items = {item["id"]: item for item in source_scene["conveyors"]["items"]}
    if set(items) != OWNERS:
        raise ValueError("ECO65_COUPLED_SOURCE_BELTS_REQUIRED")
    tb = np.asarray(items["conveyor_transverse"]["belt_bounds_D_m"], float)
    lb = np.asarray(items["conveyor_longitudinal"]["belt_bounds_D_m"], float)
    length = float(lb[1, 0]-lb[0, 0])
    width = float(tb[1, 0]-tb[0, 0])
    transverse_length = float(tb[1, 1]-tb[0, 1])
    low, high = float(c["low_height_m"]), float(c["high_height_m"])
    if (not np.allclose([width, lb[1, 1]-lb[0, 1]], [.2, .2], atol=1e-9, rtol=0)
            or abs(low-.18) > 1e-12 or not low < high < low+length
            or np.any(tb[1] <= tb[0]) or np.any(lb[1] <= lb[0])
            or not np.allclose([tb[1, 2], lb[1, 2]], low, atol=1e-9, rtol=0)):
        raise ValueError("ECO65_MEASURED_BELT_GEOMETRY_INVALID")
    ymin = float(c["transverse_ymin_m"])
    centers = dict(conveyor_transverse=[float(np.mean(tb[:, 0])), ymin+transverse_length/2, low],
                   conveyor_longitudinal=[float(tb[0, 0])-length/2, ymin+width/2, low])
    slope = (high-low)/np.sqrt(length**2-(high-low)**2)
    original_centers = {"conveyor_transverse": [*np.mean(tb[:, :2], axis=0), low],
                        "conveyor_longitudinal": [*np.mean(lb[:, :2], axis=0), low]}
    templates = []
    source_objects = [o for o in source_scene["native_world_objects"] if o.get("owner") in OWNERS]
    previous = source_scene.get('coupled_conveyor_spec')
    if previous:
        # Native objects in a coupled source already carry a moving pose.
        # Reuse their immutable local templates, never convert that pose again.
        identity = dict(previous)
        if identity.pop('fingerprint') != digest(identity):
            raise ValueError('ECO65_COUPLED_SPEC_IDENTITY_INVALID')
        _validate_local_belts(previous)
        if (previous['geometry_revision'] != REVISION
                or abs(previous['seam_relief_slope']-slope) > 1e-12
                or previous['source_belt_bounds_D_m'] != {
                    name: items[name]['belt_bounds_D_m'] for name in sorted(OWNERS)}):
            raise ValueError('ECO65_COUPLED_REBUILD_REQUIRES_ORIGINAL_BELT_SOURCE')
        templates = deepcopy(previous['belt_templates'])
    for row in ([] if previous else source_objects):
        if row.get("role") != "conveyor_belt":
            continue
        owner = row["owner"]
        T = np.asarray(row["pose"], float)
        mesh = deepcopy(row["convex_mesh"])
        vertices = np.asarray(mesh["vertices"]) @ T[:3, :3].T + T[:3, 3]
        vertices -= original_centers[owner]
        mesh["vertices"] = vertices.tolist()
        if owner == "conveyor_transverse":
            # Exact top edge remains; only the underside at the receiving seam
            # is cut. A fixed worst-angle plane avoids belt-body penetration.
            mesh = _clip_convex(mesh, [1., 0., slope], -width/2)
            if mesh is None:
                raise ValueError("ECO65_SEAM_RELIEF_REMOVED_BELT")
        templates.append(dict(id=row["id"], convex_mesh=mesh, owner=owner, role="conveyor_belt",
                              source_id=row["id"], source_geometry_digest=digest(row),
                              coordinate_conversion="D_BAKED_TO_SURFACE_CENTRE_LOCAL_ONCE",
                              rgba=[.17, .20, .21, 1]))
    if {o["owner"] for o in templates} != OWNERS:
        raise ValueError("ECO65_SOURCE_BELT_COLLIDERS_MISSING")
    if any(o.get("owner") in OWNERS for o in source_scene.get("obstacles", [])):
        raise ValueError("ECO65_UNEXPECTED_STATIC_CONVEYOR_PROXY")
    spec = dict(schema="eco65_coupled_spec_v1", geometry_revision=REVISION,
                belt_geometry_revision="surface_centred_local_rebuild_v2",
                source_scene_fingerprint=source_scene["scene_fingerprint"],
                low_height_m=low, high_height_m=high, config=c, longitudinal_length_m=length,
                transverse_width_m=width, transverse_length_m=transverse_length,
                transverse_center_xy_m=centers["conveyor_transverse"][:2],
                longitudinal_center_y_m=centers["conveyor_longitudinal"][1],
                transverse_xmin_m=float(tb[0, 0]), seam_relief_slope=float(slope),
                belt_depth_m=float(max(low-tb[0, 2], low-lb[0, 2])),
                belt_templates=templates,
                replaced_object_ids=[o["id"] for o in source_objects],
                replaced_obstacle_ids=["junction_transition_plate"],
                replaced_visual_ids=sorted(OWNERS),
                source_belt_bounds_D_m={name: deepcopy(items[name]["belt_bounds_D_m"]) for name in sorted(OWNERS)},
                engineering_geometry=dict(
                    belts="ORIGINAL_CONVEX_BELT_CONTOURS; TRANSVERSE_UNDERSIDE_SEAM_RELIEF_ADDED",
                    supports="ALL_OLD_STATIC_CONVEYOR_STRUCTURES_REPLACED_BY_FIXED_LENGTH_SCISSORS_AND_RAILS",
                    frame="NEW_32MM_FRAME_WITH_FIXED_MAXIMUM_ANGLE_SEAM_RELIEF",
                    low_height_design="26MM_LOWER_PINS; SHORT_RAILS_AND_FEET_DESIGNED_FOR_180MM_SURFACE",
                    qualification="PROJECT_GEOMETRY_NOT_MANUFACTURING_OR_LOAD_RATING"))
    spec["fingerprint"] = digest(spec)
    # Fail on an invalid configured endpoint before writing a scene.
    coupled_conveyor_state(spec, low)
    coupled_conveyor_state(spec, high)
    return spec


def _scissor(name, anchor, top_z, config, floor_z=0.):
    """Two disjoint bar lanes, fixed lower rail/feet, moving upper rail."""
    anchor = np.asarray(anchor, float)
    stages, length = int(config["stages"]), float(config["arm_length_m"])
    depth, thickness = float(config["arm_depth_m"]), float(config["arm_thickness_m"])
    rise = (top_z-anchor[2])/stages
    if not depth < rise < length-depth:
        raise ValueError("ECO65_SCISSOR_STROKE_INVALID:" + name)
    span = float(np.sqrt(length**2-rise**2))
    direction, axis = np.array([0., 1., 0.]), np.array([-1., 0., 0.])
    lane = thickness/2 + .001
    rows, bars, joints, mates = [], [], [], []
    for level in range(stages):
        left = anchor + [0, 0, level*rise]
        right = left + direction*span
        names = []
        for side, (a, b) in enumerate(((left, right+[0, 0, rise]),
                                       (right, left+[0, 0, rise]))):
            shift = axis*lane*(2*side-1)
            a, b = a+shift, b+shift
            tangent = (b-a)/length
            R = np.column_stack([tangent, axis, np.cross(tangent, axis)])
            key = f"{name}_arm_{level}_{side}"
            rows.append(_box(key, (a+b)/2, [length, thickness, depth], rotation=R))
            bars.append(dict(id=key, endpoints_m=[a.tolist(), b.tolist()], length_m=length,
                             lane_offset_m=float(lane*(2*side-1))))
            names.append(key)
        point = (left+right)/2 + [0, 0, rise/2]
        pin = f"{name}_cross_pin_{level}"
        rows.append(_box(pin, point, [4*lane, depth, depth], role="lift_hinge"))
        joints.append(dict(id=pin, type="REVOLUTE", bodies=names,
                           position_m=point.tolist(), axis=axis.tolist()))
        mates.extend([[pin, body, "CROSS_PIN"] for body in names])
    for level in range(stages+1):
        for side in (0, 1):
            p = anchor + direction*span*side + [0, 0, level*rise]
            pin = f"{name}_end_pin_{level}_{side}"
            rows.append(_box(pin, p, [4*lane, depth, depth], role="lift_hinge"))
            bodies = []
            if level > 0:
                bodies.append(f"{name}_arm_{level-1}_{1-side}")
            if level < stages:
                bodies.append(f"{name}_arm_{level}_{side}")
            joints.append(dict(id=pin, type="FIXED_XY_PIN" if side == 0 else "Y_SLIDER_PIN",
                               bodies=bodies, position_m=p.tolist(), axis=axis.tolist()))
            mates.extend([[pin, body, "END_PIN"] for body in bodies])
    # Rail span uses the immutable bar length, not the varying horizontal span.
    rail_y = anchor[1]+length/2
    foot_h = anchor[2]-.016-floor_z
    if foot_h <= 0:
        raise ValueError("ECO65_SCISSOR_FOOT_BELOW_FLOOR:" + name)
    rows.append(_box(f"{name}_base_rail", [anchor[0], rail_y, anchor[2]-.012], [.026, length+.024, .008]))
    for side in (0, 1):
        rows.append(_box(f"{name}_foot_{side}",
                        [anchor[0], anchor[1]+length*side, floor_z+foot_h/2], [.030, .030, foot_h]))
        mates.append([f"{name}_foot_{side}", f"{name}_base_rail", "FIXED_FOOT_RAIL"])
    rows.append(_box(f"{name}_head_rail", [anchor[0], rail_y, top_z+.009],
                     [.026, length+.024, .012]))
    for side in (0, 1):
        for end, z in (("base", anchor[2]-.004), ("head", top_z+.003)):
            key = f"{name}_{end}_slider_{side}"
            rows.append(_box(key, [anchor[0], anchor[1]+span*side, z], [.020, .018, .008],
                             role="lift_slider"))
            level = 0 if end == "base" else stages
            mates.extend([[key, f"{name}_{end}_rail", "FIXED_BEARING" if side == 0 else "Y_SLIDER"],
                          [key, f"{name}_end_pin_{level}_{side}", "END_BEARING"]])
    fixed = [f"{name}_base_rail", f"{name}_foot_0", f"{name}_foot_1", f"{name}_base_slider_0"]
    return rows, dict(id=name, stages=stages, span_m=span, stage_rise_m=rise,
                      arm_length_m=length, bars=bars, joints=joints, fixed_object_ids=fixed,
                      slider_travel_m=length-span, additional_mates=mates)


def coupled_conveyor_state(spec, height_m):
    """Pure deterministic geometry; no source scene or robot state is mutated."""
    check = dict(spec)
    expected = check.pop("fingerprint")
    if digest(check) != expected or spec.get("geometry_revision") != REVISION:
        raise ValueError("ECO65_COUPLED_SPEC_IDENTITY_INVALID")
    _validate_local_belts(spec)
    c = spec["config"]
    H, low, high = float(height_m), spec["low_height_m"], spec["high_height_m"]
    if not np.isfinite(H) or not low-1e-12 <= H <= high+1e-12:
        raise ValueError("ECO65_COUPLED_HEIGHT_OUT_OF_RANGE")
    length = spec["longitudinal_length_m"]
    rise = H-low
    dx = float(np.sqrt(length**2-rise**2))
    theta = float(np.arctan2(rise, dx))
    co, si = dx/length, rise/length
    R = np.array([[co, 0, -si], [0, 1, 0], [si, 0, co]])
    upper = np.array([spec["transverse_xmin_m"], spec["longitudinal_center_y_m"], H])
    lower = upper - [dx, 0, rise]
    transverse = np.array([*spec["transverse_center_xy_m"], H])
    longitudinal = (upper+lower)/2
    transforms = {"conveyor_transverse": (transverse, np.eye(3)),
                  "conveyor_longitudinal": (longitudinal, R)}
    world = []
    for item in spec["belt_templates"]:
        row = deepcopy(item)
        center, rotation = transforms[row["owner"]]
        row.update(pose=_pose(center, rotation), category="environment")
        world.append(row)
    width, trans_len, depth = spec["transverse_width_m"], spec["transverse_length_m"], c["frame_depth_m"]
    belt_depth = spec["belt_depth_m"]
    world.append(dict(id="eco65_transverse_relief_frame", pose=_pose(transverse),
                      convex_mesh=_relieved_frame(width, trans_len, belt_depth, depth, spec["seam_relief_slope"]),
                      category="environment", owner="conveyor_transverse", role="conveyor_frame",
                      rgba=[.29, .33, .38, 1]))
    world.append(_box("eco65_longitudinal_rigid_frame",
                      longitudinal - R[:, 2]*(depth+belt_depth)/2,
                      [length, width, depth-belt_depth], rotation=R, role="conveyor_frame",
                      owner="conveyor_longitudinal"))
    stacks, fixed, mates = [], [], []
    trans_spec = c["transverse_scissor"]
    for side in (-1, 1):
        x = transverse[0]+side*c["transverse_support_x_offset_m"]
        anchor = [x, transverse[1]-trans_spec["arm_length_m"]/2, c["scissor_base_pin_z_m"]]
        key = "eco65_lift_transverse_" + ("left" if side < 0 else "right")
        rows, info = _scissor(key, anchor, H-depth-.020, trans_spec, c["floor_z_m"])
        world.extend(rows); stacks.append(info); fixed.extend(info["fixed_object_ids"])
        world.append(_box(key+"_mount", [x, transverse[1], H-depth-.0025],
                          [.026, trans_spec["arm_length_m"]+.024, .005]))
        mates.extend(info["additional_mates"])
    long_spec = c["longitudinal_scissor"]
    # Fixed floor stations. Their heads rotate with the inclined frame and
    # slide against it as its lower endpoint moves on the fixed X rails.
    for side in (-1, 1):
        x = upper[0] - length*c["longitudinal_support_fraction"] + side*c["longitudinal_support_x_offset_m"]
        if not lower[0] < x < upper[0]:
            raise ValueError("ECO65_LONGITUDINAL_SUPPORT_OUTSIDE_FRAME")
        underside = H+(x-upper[0])*si/co-depth/co
        key = "eco65_lift_longitudinal_" + ("far" if side < 0 else "near")
        anchor = [x, lower[1]-long_spec["arm_length_m"]/2, c["scissor_base_pin_z_m"]]
        rows, info = _scissor(key, anchor, underside-.030, long_spec, c["floor_z_m"])
        world.extend(rows); stacks.append(info); fixed.extend(info["fixed_object_ids"])
        world.append(_box(key+"_neck", [x, lower[1], underside-.012], [.014, .15, .012]))
        head = np.array([x, lower[1], underside]) - R[:, 2]*.004
        world.append(_box(key+"_swivel_head", head, [.022, .17, .008], rotation=R, role="lift_hinge"))
        info["frame_contact_local_m"] = (R.T @ (np.array([x, lower[1], underside])-longitudinal)).tolist()
        info["swivel_angle_rad"] = theta
        mates.extend(info["additional_mates"])
        mates.append([key+"_swivel_head", "eco65_longitudinal_rigid_frame", "ROTATING_SLIDING_SUPPORT"])
    low_x = upper[0]-length
    travel = length-np.sqrt(length**2-(high-low)**2)
    rail_min = low_x-c["lower_rail_end_margin_m"]
    rail_max = low_x+travel+c["lower_rail_end_margin_m"]
    # The 180 mm low surface requires short feet, not the reference 200 mm
    # rail offset. Both rails and all four feet remain fixed in D.
    rail_z = low-depth-c["lower_rail_below_frame_m"]
    carriage_z = low-depth-c["lower_carriage_below_frame_m"]
    lower_hinges = []
    for side in (-1, 1):
        y = lower[1]+side*c["lower_rail_y_offset_m"]
        key = "eco65_lower_rail_" + ("left" if side < 0 else "right")
        world.append(_box(key, [(rail_min+rail_max)/2, y, rail_z],
                          [rail_max-rail_min, .018, .012]))
        fixed.append(key)
        for i, x in enumerate((rail_min+.012, rail_max-.012)):
            name = key+f"_foot_{i}"
            foot_h = rail_z-.006-c["floor_z_m"]
            world.append(_box(name, [x, y, c["floor_z_m"]+foot_h/2], [.026, .026, foot_h]))
            fixed.append(name)
        name = key+"_carriage"
        world.append(_box(name, [lower[0], y, carriage_z], [.030, .028, .016], role="lift_slider"))
        mates.append([key, name, "X_SLIDER"])
        # The revolute axis passes through the effective lower endpoint. Its
        # bearing housings are outside the belt, so no axle obstructs the usable
        # belt surface. A moving carriage post connects to each housing at all
        # heights; an underside-normal-offset bearing would disconnect as the
        # conveyor tilts and is deliberately not used.
        bearing = key+"_bearing_housing"
        bearing_bottom = low-.010
        post_bottom = carriage_z+.008
        post = key+"_bearing_post"
        world.append(_box(post, [lower[0], y, (post_bottom+bearing_bottom)/2],
                          [.020, .018, bearing_bottom-post_bottom]))
        world.append(_box(bearing, [lower[0], y, low], [.024, .020, .020], role="lift_hinge"))
        bracket = key+"_rotating_end_bracket"
        local = np.array([-.008, side*(width/2+.002), -.014])
        world.append(_box(bracket, lower+R@local, [.028, .004, .028],
                          rotation=R, role="lift_hinge"))
        pin = key+"_revolute_pin"
        pivot = lower + [0, side*(width/2+.0125), 0]
        world.append(_box(pin, pivot, [.008, .025, .008], role="lift_hinge"))
        mates.extend([[name, post, "CARRIAGE_FIXED_POST"],
                      [post, bearing, "FIXED_BEARING_HOUSING"],
                      [pin, bearing, "LOWER_REVOLUTE_PIN"],
                      [pin, bracket, "LOWER_REVOLUTE_PIN"],
                      [bracket, "eco65_longitudinal_rigid_frame", "FRAME_FIXED_END_BRACKET"]])
        lower_hinges.append(dict(id=pin, type="REVOLUTE", axis=[0., 1., 0.],
                                 position_m=pivot.tolist(), angle_rad=theta,
                                 bodies=[bearing, bracket], effective_surface_endpoint=lower.tolist()))
    ids = [o["id"] for o in world]
    if len(ids) != len(set(ids)):
        raise ValueError("ECO65_COUPLED_DUPLICATE_OBJECT")
    lo = [transverse[0]-width/2, transverse[1]-trans_len/2, H-spec["belt_depth_m"]]
    hi = [transverse[0]+width/2, transverse[1]+trans_len/2, H]
    support_ids = [o["id"] for o in world if o.get("owner") == "conveyor_transverse" and o.get("role") == "conveyor_belt"]
    receiving = dict(region_id="conveyor_transverse", support_ids=support_ids, bounds_D_m=[lo, hi],
                     margin_m=c["receiving_margin_m"],
                     box_pose_D=transform([transverse[0], 0., H+.05]).tolist(),
                     effective_surface_definition="MEASURED_TOP_BELT_RECTANGLE_NOT_WORLD_AABB")
    actual_ymin = {owner: float(min(
        (np.asarray(row['convex_mesh']['vertices']) @ np.asarray(row['pose'])[:3, :3].T
         + np.asarray(row['pose'])[:3, 3])[:, 1].min()
        for row in world if row.get('owner') == owner and row.get('role') == 'conveyor_belt'))
        for owner in sorted(OWNERS)}
    if any(abs(y-lo[1]) > 1e-6 for y in actual_ymin.values()):
        raise ValueError('ECO65_ACTUAL_BELT_YMIN_MISALIGNED')
    mechanism = dict(schema="eco65_coupled_scissor_v1", geometry_revision=REVISION,
                     height_m=H, tilt_rad=theta, longitudinal_length_m=length,
                     upper_endpoint_D_m=upper.tolist(), lower_endpoint_D_m=lower.tolist(),
                     lower_slide_x_m=float(lower[0]-low_x), fixed_structure_ids=sorted(fixed),
                     moving_object_ids=[i for i in ids if i not in fixed], stacks=stacks,
                     lower_hinges=lower_hinges,
                     additional_mates=mates, seam=dict(gap_m=0., ymin_m=lo[1],
                         actual_belt_ymin_m=actual_ymin,
                         upper_edge_D_m=upper.tolist(), relief_slope=spec["seam_relief_slope"]),
                     geometry_origin=spec["engineering_geometry"],
                     physical_execution="NOT_RUN", execution_ready=False)
    result = dict(schema="eco65_coupled_state_v1", spec_fingerprint=spec["fingerprint"],
                  height_m=H, tilt_rad=theta, world_objects=world,
                  receiving=receiving, mechanism=mechanism)
    result["scene_fingerprint"] = digest(result)
    return result


def with_coupled_state(scene, height_m):
    """Materialize one robot-motion world while preserving the static identity.

    ``scene_fingerprint`` remains the frozen model scene identity. The dynamic
    world has its own identity in ``coupled_state.scene_fingerprint`` and must
    be bound to each request; this value is never presented as a new snapshot.
    """
    result = deepcopy(scene)
    state = coupled_conveyor_state(scene["coupled_conveyor_spec"], height_m)
    spec = scene["coupled_conveyor_spec"]
    replaced = set(spec["replaced_object_ids"])
    result["native_world_objects"] = [o for o in result["native_world_objects"]
                                      if o["id"] not in replaced and o.get("owner") not in OWNERS
                                      and o.get("owner") != "coupled_receiver"] + state["world_objects"]
    result["obstacles"] = [o for o in result["obstacles"] if o["id"] not in spec["replaced_obstacle_ids"]]
    result["mesh_obstacles"] = [o for o in result.get("mesh_obstacles", [])
                                if o["id"] not in spec["replaced_visual_ids"]]
    result["native_task"]["receiving"] = deepcopy(state["receiving"])
    result["receiving_surfaces"] = {"conveyor_transverse": deepcopy(state["receiving"])}
    result["support_surface_ids"] = ["cargo_pedestal", *state["receiving"]["support_ids"]]
    result["coupled_state"] = state
    return result


def build_coupled_scene(source_scene, config=None):
    """Construct a new frozen low-state scene; does not accept any robot q."""
    result = deepcopy(source_scene)
    spec = make_coupled_spec(source_scene, config)
    result["coupled_conveyor_spec"] = spec
    result = with_coupled_state(result, spec["config"]["initial_height_m"])
    result["layout_id"] = spec["config"]["layout_id"]
    result["collision_policy_mode"] = spec["config"]["collision_policy_mode"]
    if "extraction_carton_contact_mode" in spec["config"]:
        result["extraction_carton_contact_mode"] = spec["config"]["extraction_carton_contact_mode"]
    if "cup_neighbor_contact_mode" in spec["config"]:
        result["cup_neighbor_contact_mode"] = spec["config"]["cup_neighbor_contact_mode"]
    result["source_native_scene_fingerprint"] = source_scene["scene_fingerprint"]
    result["occupancy"] = {"conveyor_transverse": []}
    result["scene_fingerprint"] = digest({k: v for k, v in result.items() if k != "scene_fingerprint"})
    return result


def receiving_height_candidates(spec, initial_box):
    """Freeze both process heights from the initial target, never carried pose."""
    bottom = float(initial_box["pose"][2]-initial_box["size_m"][2]/2)
    drop = spec["config"]["desired_drop_m"]
    return [dict(name=name, height_m=float(height), initial_box_bottom_m=bottom,
                 desired_drop_m=drop, in_hardware_range=bool(spec["low_height_m"] <= height <= spec["high_height_m"]))
            for name, height in (("receive_high", bottom-drop),
                                 ("receive_low", bottom-initial_box["size_m"][2]-drop))]


def export_state_usd(state, output):
    """Optional USD adapter: exact same size/convex solids as native/viewer."""
    from pxr import Gf, Usd, UsdGeom, Vt
    stage = Usd.Stage.CreateNew(str(Path(output).resolve()))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, "Z")
    root = UsdGeom.Xform.Define(stage, "/CoupledReceiver")
    stage.SetDefaultPrim(root.GetPrim())
    for index, row in enumerate(state["world_objects"]):
        path = "/CoupledReceiver/p" + str(index)
        xform = UsdGeom.Xform.Define(stage, path)
        xform.GetPrim().SetCustomDataByKey("object_id", row["id"])
        xform.AddTransformOp().Set(Gf.Matrix4d(np.asarray(row["pose"]).T.tolist()))
        if "size" in row:
            shape = UsdGeom.Cube.Define(stage, path+"/Solid")
            shape.CreateSizeAttr(1.)
            shape.AddScaleOp().Set(Gf.Vec3f(*row["size"]))
        else:
            shape = UsdGeom.Mesh.Define(stage, path+"/Solid")
            mesh = row["convex_mesh"]
            shape.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(mesh["vertices"], np.float32)))
            shape.CreateFaceVertexCountsAttr([3]*len(mesh["triangles"]))
            shape.CreateFaceVertexIndicesAttr(np.asarray(mesh["triangles"], int).ravel().tolist())
        shape.CreateDisplayColorAttr([Gf.Vec3f(*row.get("rgba", [.4, .5, .6, 1])[:3])])
    stage.GetRootLayer().Save()
    return str(output)
