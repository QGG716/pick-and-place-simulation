"""Small, explicit planning-only static joint graph bindings.

A context match permits querying candidate edges. It never certifies them in
the current scene: the resident worker must check every selected edge with the
actual robot/tool/payload geometry and current world before returning a path.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from time import perf_counter

SCHEMA = "m710_static_prior_v1"
CONTEXT_SCHEMA = "m710_static_prior_context_v1"
MARKER = "PLANNING_ONLY_NOT_EXECUTABLE"
DYNAMIC_CATEGORIES = frozenset(("carton", "payload"))
ATTACHMENT_TRANSLATION_LIMIT_M = .025
ATTACHMENT_ROTATION_LIMIT_RAD = .05


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode()).hexdigest()


def canonical_json(value):
    """Use the transport value domain; tuples become arrays without changing data."""
    return json.loads(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False))


def placement_policy_fingerprint(policy, *, payload_mass_kg=42.5):
    """Bind the existing experiment's receiver, process and load assumptions."""
    layout = policy.layout_validation.layout
    load_path = (layout.config_path.parent / layout.data["tool"]["load_config"]).resolve()
    mass = float(payload_mass_kg)
    if not math.isfinite(mass) or mass <= 0.:
        raise ValueError("STATIC_PRIOR_INVALID_PAYLOAD_MASS")
    return fingerprint(dict(search_strategy=deepcopy(policy.data["search_strategy"]),
        receiver=deepcopy(policy.data["receiver"]), suction=deepcopy(policy.data["suction"]),
        tool_frame_contract=deepcopy(policy.data["tool_frame_contract"]),
        payload_mass_kg=mass,
        tool_load_config_sha256=hashlib.sha256(load_path.read_bytes()).hexdigest()))


def graph_id(mode):
    keys = ("context", "nodes", "edges") + (("accepted_portals",) if "accepted_portals" in mode else ())
    return fingerprint({key: mode[key] for key in keys})


def static_world(world):
    """Retain all fixed bodies, independent of the remaining carton population."""
    result = [deepcopy(b) for b in world if b.get("category") not in DYNAMIC_CATEGORIES]
    if any(not b.get("id") for b in result) or len({b["id"] for b in result}) != len(result):
        raise ValueError("STATIC_PRIOR_INVALID_WORLD_IDENTITY")
    return sorted(result, key=lambda b: b["id"])


def _vector(value, length, name):
    if (not isinstance(value, (list, tuple)) or len(value) != length
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in value)):
        raise ValueError("STATIC_PRIOR_INVALID_" + name)
    return list(value)


def _transform(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("STATIC_PRIOR_INVALID_ATTACHMENT_POSE")
    result = [_vector(row, 4, "ATTACHMENT_POSE") for row in value]
    if result[3] != [0., 0., 0., 1.]:
        raise ValueError("STATIC_PRIOR_INVALID_ATTACHMENT_POSE")
    rot = [row[:3] for row in result[:3]]
    if any(abs(sum(rot[k][i]*rot[k][j] for k in range(3)) - (i == j)) > 1e-7
           for i in range(3) for j in range(3)):
        raise ValueError("STATIC_PRIOR_INVALID_ATTACHMENT_ROTATION")
    det = (rot[0][0]*(rot[1][1]*rot[2][2]-rot[1][2]*rot[2][1])
           - rot[0][1]*(rot[1][0]*rot[2][2]-rot[1][2]*rot[2][0])
           + rot[0][2]*(rot[1][0]*rot[2][1]-rot[1][1]*rot[2][0]))
    if abs(det-1.) > 1e-7:
        raise ValueError("STATIC_PRIOR_INVALID_ATTACHMENT_ROTATION")
    return result


def attachment_binding(attachment):
    if attachment is None:
        return dict(mode="empty")
    size = _vector(attachment.get("size"), 3, "PAYLOAD_SIZE")
    if min(size) <= 0.:
        raise ValueError("STATIC_PRIOR_INVALID_PAYLOAD_SIZE")
    return dict(mode="loaded", size=size, pose=_transform(attachment.get("pose")),
        touch_links=sorted(attachment.get("touch_links", [])))


def request_context(request, *, placement_policy_sha256, joint_limits_rad=None,
                    tool_geometry_binding=None):
    policy = request["clearance_policy"]
    if not placement_policy_sha256 or not request.get("identity"):
        raise ValueError("STATIC_PRIOR_MISSING_CONTEXT")
    dynamic_ids = {b["id"] for b in request["world"] if b.get("category") in DYNAMIC_CATEGORIES}
    if request.get("attachment"):
        dynamic_ids.add(request["attachment"]["id"])
    pairs = sorted(sorted(p) for p in request.get("allowed_pairs", [])
                   if not any(name in dynamic_ids for name in p))
    limits = None
    if joint_limits_rad is not None:
        if len(joint_limits_rad) != 6:
            raise ValueError("STATIC_PRIOR_INVALID_JOINT_LIMITS")
        limits = [_vector(row, 2, "JOINT_LIMITS") for row in joint_limits_rad]
        if any(lo >= hi for lo, hi in limits):
            raise ValueError("STATIC_PRIOR_INVALID_JOINT_LIMITS")
    result = dict(schema=CONTEXT_SCHEMA, identity=deepcopy(request["identity"]),
        flange_from_task_tcp=deepcopy(request["flange_from_task_tcp"]),
        tool_links=sorted(policy["tool_links"]),
        compliant_tool_links=sorted(policy["compliant_tool_links"]),
        collision_policy=deepcopy(policy["source_policy"]),
        static_world=static_world(request["world"]), allowed_pairs=pairs,
        joint_limits_rad=limits,
        interpolation=dict(edge_resolution_rad=policy["edge_resolution_rad"],
            joint_step_l1_rad=min(float(policy["edge_resolution_rad"]), .01), distance_norm="L1",
            waypoint_semantics="joint_linear_between_nodes"),
        receiver_reserve_m=policy.get("receiver_reserve_m", 0.),
        attachment=attachment_binding(request.get("attachment")),
        placement_policy_sha256=placement_policy_sha256)
    if tool_geometry_binding is not None:
        result["tool_geometry_binding"] = deepcopy(tool_geometry_binding)
    return canonical_json(result)  # Exact value sent over the JSON transport.


def applicability(stored_context, current_context):
    """Candidate applicability, with zero inherited current-scene safety claims."""
    result = dict(covered=False, current_geometry_check_required=True,
        static_clearance_inherited=False, reason=None)
    for key in set(stored_context) | set(current_context):
        if key != "attachment" and stored_context.get(key) != current_context.get(key):
            return dict(result, reason="STATIC_PRIOR_CONTEXT_MISMATCH", field=key)
    old, new = stored_context["attachment"], current_context["attachment"]
    if old["mode"] != new["mode"]:
        return dict(result, reason="STATIC_PRIOR_LOAD_CLASS_MISMATCH")
    if old["mode"] == "loaded":
        if old["size"] != new["size"] or old["touch_links"] != new["touch_links"]:
            return dict(result, reason="STATIC_PRIOR_PAYLOAD_MISMATCH")
        a, b = _transform(old["pose"]), _transform(new["pose"])
        distance = math.sqrt(sum((a[i][3]-b[i][3])**2 for i in range(3)))
        cosine = (sum(a[i][j]*b[i][j] for i in range(3) for j in range(3))-1.)/2.
        angle = math.acos(max(-1., min(1., cosine)))
        result.update(attachment_translation_delta_m=distance, attachment_rotation_delta_rad=angle)
        if distance > ATTACHMENT_TRANSLATION_LIMIT_M or angle > ATTACHMENT_ROTATION_LIMIT_RAD:
            return dict(result, reason="STATIC_PRIOR_ATTACHMENT_OUTSIDE_SCOPE")
    return dict(result, covered=True, reason="CANDIDATE_ONLY_RECHECK_REQUIRED")


def validate_mode(mode, *, expected_context=None):
    if not isinstance(mode, dict) or mode.get("status") != "SUCCESS":
        raise ValueError("STATIC_PRIOR_BUILD_NOT_SUCCESSFUL")
    context = mode.get("context")
    if not isinstance(context, dict) or context.get("schema") != CONTEXT_SCHEMA:
        raise ValueError("STATIC_PRIOR_CONTEXT_MISSING")
    if expected_context is not None and context != canonical_json(expected_context):
        raise ValueError("STATIC_PRIOR_BUILD_CONTEXT_CHANGED")
    nodes, edges = mode.get("nodes"), mode.get("edges")
    if not isinstance(nodes, list) or not isinstance(edges, list):
        raise ValueError("STATIC_PRIOR_GRAPH_MISSING")
    limits = context.get("joint_limits_rad")
    for node in nodes:
        q = _vector(node, 6, "NODE")
        if limits is not None and any(v < bounds[0] or v > bounds[1] for v, bounds in zip(q, limits)):
            raise ValueError("STATIC_PRIOR_NODE_OUTSIDE_LIMITS")
    for edge in edges:
        if (not isinstance(edge, list) or len(edge) != 2
                or any(type(i) is not int or not 0 <= i < len(nodes) for i in edge)
                or edge[0] == edge[1]):
            raise ValueError("STATIC_PRIOR_INVALID_EDGE")
    portals = mode.get("accepted_portals", [])
    if not isinstance(portals, list) or len(portals) > 128:
        raise ValueError("STATIC_PRIOR_INVALID_PORTAL_METADATA")
    ids = set()
    for portal in portals:
        if (not isinstance(portal, dict) or not isinstance(portal.get("portal_id"), str)
                or not portal["portal_id"] or portal["portal_id"] in ids
                or type(portal.get("node_index")) is not int
                or not 0 <= portal["node_index"] < len(nodes)):
            raise ValueError("STATIC_PRIOR_INVALID_PORTAL_METADATA")
        _transform(portal.get("pose"))
        ids.add(portal["portal_id"])
    if mode.get("prior_id") is not None and mode["prior_id"] != graph_id(mode):
        raise ValueError("STATIC_PRIOR_GRAPH_ID_MISMATCH")
    # A partial/empty bounded graph can be recorded, but is not a successful query.
    return mode


def make_database(modes, metadata):
    if not modes or any(name not in {"empty", "loaded"} for name in modes):
        raise ValueError("STATIC_PRIOR_INVALID_MODES")
    for name, mode in modes.items():
        validate_mode(mode)
        if mode["context"]["attachment"]["mode"] != name:
            raise ValueError("STATIC_PRIOR_LOAD_CLASS_MISMATCH")
    modes = deepcopy(modes)
    for mode in modes.values():
        mode["prior_id"] = graph_id(mode)
    data = dict(schema=SCHEMA, status=MARKER, qualification_status="NOT_EVALUATED",
        source_kind="OFFLINE_GENERIC_JOINT_GRAPH", history_solution_input=False,
        modes=deepcopy(modes), metadata=deepcopy(metadata))
    data["content_sha256"] = fingerprint(data)
    return data


def load_prior(path):
    started = perf_counter()
    raw = Path(path).read_bytes()
    if len(raw) > 32 * 1024 * 1024:
        raise ValueError("STATIC_PRIOR_DATABASE_TOO_LARGE")
    data = json.loads(raw)
    if (data.get("schema") != SCHEMA or data.get("status") != MARKER
            or data.get("qualification_status") != "NOT_EVALUATED"
            or data.get("history_solution_input") is not False):
        raise ValueError("STATIC_PRIOR_INVALID_DATABASE")
    expected = data.get("content_sha256")
    unsigned = {k: v for k, v in data.items() if k != "content_sha256"}
    if expected != fingerprint(unsigned):
        raise ValueError("STATIC_PRIOR_CONTENT_HASH_MISMATCH")
    make_database(data["modes"], data["metadata"])
    return data, dict(load_s=perf_counter()-started,
        sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw),
        source_kind="STATIC_PRIOR_DATABASE_LOAD")


def candidate_mode(database, request, *, placement_policy_sha256, joint_limits_rad=None):
    context = request_context(request, placement_policy_sha256=placement_policy_sha256,
        joint_limits_rad=joint_limits_rad)
    name = context["attachment"]["mode"]
    mode = database["modes"].get(name)
    if mode is None:
        return None, dict(covered=False, reason="STATIC_PRIOR_LOAD_CLASS_UNCOVERED",
            current_geometry_check_required=True, static_clearance_inherited=False)
    outcome = applicability(mode["context"], context)
    return (mode if outcome["covered"] else None), outcome


def nominal_front_attachment(size_xyz_m):
    """Geometry-derived physical-contact-to-carton transform; no IK/path input."""
    size = _vector(list(size_xyz_m), 3, "PAYLOAD_SIZE")
    if min(size) <= 0.:
        raise ValueError("STATIC_PRIOR_INVALID_PAYLOAD_SIZE")
    return [[0., 0., 1., 0.], [0., -1., 0., 0.],
            [1., 0., 0., size[0]/2.], [0., 0., 0., 1.]]


def layout_portal_poses(scene):
    """Configuration-derived workspace poses, never query endpoints or paths.

    The offsets place samples in plausible transport regions only. They are
    not clearance assertions; the native builder screens every resulting q.
    """
    import numpy as np
    from .geometry import make_tool_rotation, make_transform
    from .conveyor_placement import TOP_DOWN, placement_working_normal

    layout = scene.policy.layout_validation.layout
    data, stack = layout.data, layout.data["carton_stack"]
    size = np.asarray(stack["carton_size_xyz_m"], dtype=float)
    front = float(stack["front_face_x_m"])
    floor = float(data["world"]["floor_z_m"])
    roof = floor + float(data["trailer"]["height_m"])
    stack_top = floor + int(stack["height_layers"])*size[2] + (
        int(stack["height_layers"])-1)*float(stack["layer_gap_m"])
    frames = scene.policy.data["tool_frame_contract"]
    virtual = np.asarray(frames["T_flange_virtual_task_tcp"], dtype=float)
    physical = np.asarray(frames["T_flange_nominal_compressed_contact"], dtype=float)
    tool_reach = float(np.linalg.norm(virtual[:3, 3]))
    base = np.asarray(layout.robot_base_transform(), dtype=float)[:3, 3]
    chassis = next(box for box in scene.fixed_components if box.name == "chassis")
    receiver = scene.receiver
    front_rotation = make_tool_rotation(np.array([1., 0., 0.]))
    top_rotation = make_tool_rotation(placement_working_normal(TOP_DOWN))
    # These fixed sampling offsets are explicit workspace heuristics. They
    # never replace or enlarge the separately bound collision-policy margins.
    front_offset = float(size[0] + tool_reach + .05)
    first_x = front - front_offset
    xs = [first_x, .5*(first_x + float(base[0]))]
    stack_left = max(stack["center_y_m"]) + size[1]/2.
    stack_right = min(stack["center_y_m"]) - size[1]/2.
    left = min(float(data["trailer"]["left_wall_y_m"]), float(stack_left)) - size[1]/2. - .10
    right = max(float(data["trailer"]["right_wall_y_m"]), float(stack_right)) + size[1]/2. + .10
    if right >= left:
        raise ValueError("STATIC_PRIOR_PORTAL_WIDTH_EMPTY")
    ys = [right, (right+left)/2., left]
    heights = [stack_top-2.5*size[2], min(stack_top-.5*size[2], roof-size[2]/2.-.10)]
    virtual_from_box = np.linalg.inv(virtual) @ physical @ np.asarray(nominal_front_attachment(size.tolist()))
    corners = np.array([[x, y, z, 1.] for x in (-size[0]/2., size[0]/2.)
        for y in (-size[1]/2., size[1]/2.) for z in (-size[2]/2., size[2]/2.)])
    top_corners = (top_rotation @ (virtual_from_box @ corners.T)[:3]).T
    below_tcp = max(0., -float(top_corners[:, 2].min()))
    receiver_top = float(receiver.center[2] + np.abs(receiver.rotation[2]) @ receiver.half_extents)
    receiver_heights = [receiver_top+below_tcp+.10, receiver_top+below_tcp+.50]
    poses = []
    def add(label, family, xyz, rotation, formula):
        pose = make_transform(rotation, xyz).tolist()
        _transform(pose)
        poses.append(dict(portal_id=label, family=family, pose=pose, position_formula=formula,
            orientation_source=("geometry.make_tool_rotation(+X)" if family.endswith("front") else
                "geometry.make_tool_rotation(conveyor_placement.placement_working_normal(TOP_DOWN))")))
    for ix, x in enumerate(xs):
        for iy, y in enumerate(ys):
            for iz, z in enumerate(heights):
                add(f"front-clearance-{ix}-{iy}-{iz}", "front", [x, y, z], front_rotation,
                    "x=front-(box_depth+norm(T_flange_virtual_tcp.translation)+0.05), then midpoint(robot_base_x); y=stack/wall span inset by box_half_width+0.10; z=stack_top-{2.5,0.5}*box_height")
    origin = np.array([xs[-1], ys[1]])
    destination = np.asarray(receiver.center[:2])
    for index, fraction in enumerate((.25, .5, .75)):
        xy = (1.-fraction)*origin + fraction*destination
        for iz, z in enumerate(heights):
            for family, rotation in (("transition-front", front_rotation), ("transition-top", top_rotation)):
                add(f"{family}-{index}-{iz}", family, [*xy, z], rotation,
                    "xy=lerp(front-clearance center,receiver center,{0.25,0.5,0.75}); z=front-clearance heights")
    long_axis = int(np.argmax(receiver.half_extents[:2]))
    for index, fraction in enumerate((-.45, 0., .45)):
        xyz = np.asarray(receiver.center) + fraction*receiver.half_extents[long_axis]*receiver.rotation[:, long_axis]
        for iz, z in enumerate(receiver_heights):
            add(f"receiver-overhead-{index}-{iz}", "receiver-top", [*xyz[:2], z], top_rotation,
                "xy=receiver center+{-0.45,0,0.45}*receiver_long_half_extent*long_axis; z=receiver_top+nominal_box_below_task_tcp+{0.10,0.50}")
    if len(poses) != 30 or not np.isfinite([p["pose"] for p in poses]).all():
        raise ValueError("STATIC_PRIOR_INVALID_PORTAL_POSES")
    sources = dict(layout_id=data["layout_id"], no_query_endpoints_used=True,
        robot_base_world_m=base.tolist(), chassis_center_world_m=chassis.center.tolist(),
        receiver_id=receiver.name, receiver_center_world_m=receiver.center.tolist(),
        stack_front_x_m=front, stack_top_z_m=float(stack_top), full_carton_size_m=size.tolist(),
        task_tcp_offset_norm_m=tool_reach, front_sampling_x_m=xs, front_sampling_y_m=ys,
        front_sampling_z_m=[float(x) for x in heights], receiver_top_z_m=receiver_top,
        nominal_loaded_extent_below_top_down_task_tcp_m=below_tcp,
        receiver_sampling_z_m=receiver_heights,
        sampling_offsets_are_clearance_claims=False,
        collision_policy_unchanged=True,
        configuration_fields=["carton_stack", "trailer", "world.floor_z_m",
            "robot_base_transform()", "fixed.chassis", "receiver", "tool_frame_contract"])
    return canonical_json(poses), canonical_json(sources)


def portal_ik_seeds(joint_limits_rad, initial_q, *, seed, count=16):
    """Initial configuration plus bounded full-limit Halton seeds; no graph input."""
    limits = [_vector(row, 2, "JOINT_LIMITS") for row in joint_limits_rad]
    initial = _vector(initial_q, 6, "PORTAL_SEED")
    if len(limits) != 6 or any(lo >= hi for lo, hi in limits):
        raise ValueError("STATIC_PRIOR_INVALID_JOINT_LIMITS")
    if any(q < lo or q > hi for q, (lo, hi) in zip(initial, limits)):
        raise ValueError("STATIC_PRIOR_PORTAL_SEED_OUTSIDE_LIMITS")
    if type(seed) is not int or seed < 0 or type(count) is not int or not 16 <= count <= 24:
        raise ValueError("STATIC_PRIOR_INVALID_PORTAL_SEED_LIMIT")
    def halton(index, base):
        value, factor = 0., 1.
        while index:
            index, digit = divmod(index, base)
            factor /= base
            value += digit*factor
        return value
    return [initial, *[[lo+(hi-lo)*halton(1+(seed % 100000)+i, prime)
        for (lo, hi), prime in zip(limits, (2, 3, 5, 7, 11, 13))] for i in range(count)]]


def diverse_portal_branches(solutions, reference, *, limit=3, tolerance=1e-4):
    """Keep a few distinct seed-directed branches without changing any q."""
    if type(limit) is not int or not 1 <= limit <= 3:
        raise ValueError("STATIC_PRIOR_INVALID_PORTAL_BRANCH_LIMIT")
    reference = _vector(reference, 6, "PORTAL_SEED")
    unique = []
    for raw in solutions:
        q = _vector(raw, 6, "PORTAL_SOLUTION")
        if not any(max(abs(a-b) for a, b in zip(q, other)) <= tolerance for other in unique):
            unique.append(q)
    if not unique:
        return []
    distance = lambda a, b: sum(abs(x-y) for x, y in zip(a, b))
    selected = [min(unique, key=lambda q: distance(q, reference))]
    while len(selected) < min(limit, len(unique)):
        remaining = [q for q in unique if q not in selected]
        selected.append(max(remaining, key=lambda q: min(distance(q, old) for old in selected)))
    return selected
