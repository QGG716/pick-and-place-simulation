"""Optional USD adapter for collider-scoped robot adjacency exclusions."""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import xml.etree.ElementTree as ET


class ActiveContactPairIndex:
    """Exact event index; no contact scope, counters or physical rules change.

    The caller's global set remains available to support-contact consumers.
    FOUND and PERSIST add the same collider key, and LOST discards only that
    exact key.  The auxiliary per-actor-pair set replaces a global-set scan.
    """

    def __init__(self, active_headers: set[tuple[str, str, str, str]]):
        self.active_headers = active_headers
        self._pairs: dict[tuple[str, str], set[tuple[str, str, str, str]]] = {}
        for key in active_headers:
            self._pairs.setdefault(key[:2], set()).add(key)

    def update(self, key: tuple[str, str, str, str], *, lost: bool) -> int:
        pair = key[:2]
        if lost:
            self.active_headers.discard(key)
            keys = self._pairs.get(pair)
            if keys is None:
                return 0
            keys.discard(key)
            count = len(keys)
            if not count:
                del self._pairs[pair]
            return count
        self.active_headers.add(key)
        keys = self._pairs.setdefault(pair, set())
        keys.add(key)
        return len(keys)


class ZeroPointContactResolver:
    """Track missing point evidence without granting any contact permission.

    An empty FOUND/PERSIST header waits for a finite measurement or exact LOST.
    Callers must fail closed on unresolved scope exits/end-of-run and classify
    every resolved measurement normally, including rigid-tool/robot pairs.
    """

    def __init__(self):
        self._pending: dict[tuple[str, str, str, str], object] = {}

    def observe(self, key, separations, lost, scope_token):
        import copy
        import math

        key = tuple(key)
        if lost:
            self._pending.pop(key, None)
            return "lost"
        try:
            values = [float(value) for value in separations]
        except (TypeError, ValueError, OverflowError):
            return "invalid"
        if any(not math.isfinite(value) for value in values):
            return "invalid"
        if values:
            self._pending.pop(key, None)
            return "resolved"
        # Preserve the original unresolved scope: another empty event after a
        # transition cannot overwrite evidence of an unclosed earlier phase.
        if key not in self._pending:
            self._pending[key] = copy.deepcopy(scope_token)
        return "pending"

    @property
    def pending_keys(self):
        return tuple(sorted(self._pending))

    def unresolved_outside(self, scope_token):
        return tuple(key for key in self.pending_keys if self._pending[key] != scope_token)

    def snapshot(self):
        import copy

        return {"pending_count": len(self._pending),
                "pending": [{"key": list(key), "scope_token": copy.deepcopy(self._pending[key])}
                            for key in self.pending_keys],
                "grants_contact_permission": False,
                "unresolved_scope_exit_or_run_end": "FAIL_CLOSED"}


class ContactPathCache:
    """Cache stable PhysX interned path IDs within one unchanged World."""

    def __init__(self, resolver: Callable[[int], object]):
        self._resolver = resolver
        self._paths: dict[int, str] = {}

    def resolve(self, path_id: int) -> str:
        key = int(path_id)
        if key not in self._paths:
            self._paths[key] = str(self._resolver(key))
        return self._paths[key]


class PhysicalContactLedger:
    """Separate geometric contact evidence from the unchanged proximity book."""

    def __init__(self, contact_tolerance_m):
        import math
        self.contact_tolerance_m = float(contact_tolerance_m)
        if not math.isfinite(self.contact_tolerance_m) or self.contact_tolerance_m < 0.0:
            raise ValueError("physical contact tolerance must be finite and nonnegative")
        self.active_headers: set[tuple[str, str, str, str]] = set()
        self._active_index = ActiveContactPairIndex(self.active_headers)

    def observe(self, key, separations, *, lost, time_s, record,
                trajectory_time_s=None):
        import math
        finite = [float(value) for value in separations if math.isfinite(float(value))]
        lower, upper = (min(finite), max(finite)) if finite else (None, None)
        physical = bool(not lost and lower is not None and lower <= self.contact_tolerance_m)
        count = self._active_index.update(key, lost=not physical)
        record["last_header_minimum_separation_m"] = lower
        record["last_header_maximum_separation_m"] = upper
        for name, value, combine in (("minimum_separation_m", lower, min),
                                     ("maximum_separation_m", upper, max)):
            previous = record.get(name)
            record[name] = previous if value is None else value if previous is None else combine(previous, value)
        record["physical_contact_tolerance_m"] = self.contact_tolerance_m
        record["physical_contact_active"] = count > 0
        record["active_physical_collider_pair_count"] = count
        record["physical_contact_observed"] = bool(record.get("physical_contact_observed", False) or physical)
        record["physical_contact_event_count"] = int(record.get("physical_contact_event_count", 0)) + int(physical)
        record.setdefault("first_physical_contact_time_s", None)
        record.setdefault("last_physical_contact_time_s", None)
        record.setdefault("first_physical_contact_trajectory_time_s", None)
        record.setdefault("last_physical_contact_trajectory_time_s", None)
        if physical:
            if record["first_physical_contact_time_s"] is None:
                record["first_physical_contact_time_s"] = float(time_s)
            record["last_physical_contact_time_s"] = float(time_s)
            if trajectory_time_s is not None:
                if record["first_physical_contact_trajectory_time_s"] is None:
                    record["first_physical_contact_trajectory_time_s"] = float(
                        trajectory_time_s
                    )
                record["last_physical_contact_trajectory_time_s"] = float(
                    trajectory_time_s
                )


def physical_support_contact_observed(active_physical_headers, target_path, support_paths):
    def support(path):
        return any(path == root or path.startswith(root + "/") for root in support_paths)
    return any((first == target_path and support(second)) or (second == target_path and support(first))
               for first, second, _shape0, _shape1 in active_physical_headers)


def robot_proximity_is_safety_relevant(record, robot_root_path):
    # Zero impulse or positive separation never bypasses the robot's margin.
    return any(str(record[name]).startswith(robot_root_path) for name in ("actor0", "actor1"))


def classify_compliant_cup_contact(
    *, collider0, collider1, actor0, actor1, compliant_cup_index_by_path,
    commanded_mask, target_path, stack_paths, stage, attached,
    actual_free_space, minimum_separation_m, policy, physical_compression_m,
    release_validation_pending=False,
) -> str | None:
    """Classify one known flexible-cup pair without changing physical collision.

    The path/index map comes from actual compliant shape creation, not naming
    conventions. Rigid inserts, mounts and robot shapes cannot inherit this
    rule. Positive contact-offset proximity uses the same pair/lifecycle scope
    as measured contact; missing separation never supplies permission.
    """
    import math
    import numbers

    paths = (collider0, collider1, actor0, actor1, target_path)
    if any(not isinstance(path, str) or not path.startswith("/") for path in paths):
        return None
    known = (collider0 in compliant_cup_index_by_path, collider1 in compliant_cup_index_by_path)
    if sum(known) != 1:
        return None
    if known[0]:
        cup, cup_actor, other_shape, other_actor = collider0, actor0, collider1, actor1
    else:
        cup, cup_actor, other_shape, other_actor = collider1, actor1, collider0, actor0
    if (cup_actor == other_actor
            or not (cup == cup_actor or cup.startswith(cup_actor.rstrip("/") + "/"))
            or not (other_shape == other_actor or other_shape.startswith(other_actor.rstrip("/") + "/"))):
        return None
    index = compliant_cup_index_by_path[cup]
    if (isinstance(index, bool) or not isinstance(index, numbers.Integral)
            or not 0 <= index < len(commanded_mask)):
        return None
    active = commanded_mask[index]
    if (not isinstance(active, bool) or not isinstance(attached, bool)
            or not isinstance(actual_free_space, bool) or not isinstance(release_validation_pending, bool)):
        return None
    try:
        separation = float(minimum_separation_m)
        physical_limit = float(physical_compression_m)
    except (TypeError, ValueError, OverflowError):
        return None
    if (not math.isfinite(separation) or not math.isfinite(physical_limit)
            or physical_limit < 0 or separation < -physical_limit):
        return None
    try:
        policy_limit = float(policy.maximum_compliant_cup_additional_compression_m)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(policy_limit) or policy_limit < 0 or separation < -min(policy_limit, physical_limit):
        return None
    if release_validation_pending:
        # The caller supplies a bounded release-clearance latch: the constraint
        # has been removed, but known target/cup proximity pairs have not all
        # reported LOST. Independence confirmation alone does not close this
        # interval. It grants no attachment and never applies to a neighbor.
        if other_actor == target_path and not attached:
            return "EXPECTED_TARGET_COMPLIANT_CUP_RELEASE_CLEARANCE"
        return None
    # Actual attachment permits its target contact through loaded motion and
    # support placement. Neither a stale stage nor a commanded cup can extend
    # that permission beyond actual release.
    if stage in {"release", "released", "withdraw", "withdrawal", "post-release"}:
        return None
    if other_actor == target_path:
        if stage == "contact" or attached:
            return "EXPECTED_TARGET_COMPLIANT_CUP_CONTACT"
        return None
    if (other_actor not in stack_paths or active or actual_free_space
            or getattr(policy, "inactive_compliant_cup_stack_contact_mode", None)
            != "physical_contact_within_compression"
            or not (stage == "contact" or (attached and stage in policy.stack_contact_stages))):
        return None
    return "ALLOWED_INACTIVE_COMPLIANT_CUP_STACK_CONTACT"


def premature_physical_conveyor_contacts(records, *, place_start_s, time_tolerance_s):
    return [record for record in records if record.get("physical_contact_observed", False)
            and (record.get("first_physical_contact_trajectory_time_s") is None
                 or float(record["first_physical_contact_trajectory_time_s"])
                 < place_start_s - time_tolerance_s)]


def placement_support_window_start(metadata, release_time_s):
    """Return PLACE start in the exported trajectory-clock time basis."""
    place_window = next(
        (
            window
            for window in metadata.get("stage_windows", [])
            if str(window.get("stage", window.get("name", ""))) == "place"
        ),
        None,
    )
    if place_window is not None:
        return float(place_window["start_time_s"])
    return float(metadata.get("release_arrival_time_seconds", release_time_s))


class ContactReportProbe:
    """Diagnostic-only observations; never filters or changes contact events."""

    def __init__(self):
        self.header_count = 0
        self.event_point_count_distribution: dict[str, int] = {}
        self.actor_pairs: dict[tuple[str, str], dict] = {}

    def observe(self, headers, contact_data, resolve):
        import math

        for header in headers:
            self.header_count += 1
            count = int(header.num_contact_data)
            event = str(getattr(header.type, "name", header.type))
            bucket = f"{event}:num_contact_data={count}"
            self.event_point_count_distribution[bucket] = self.event_point_count_distribution.get(bucket, 0) + 1
            pair = tuple(sorted((resolve(header.actor0), resolve(header.actor1))))
            row = self.actor_pairs.setdefault(pair, {
                "header_count": 0, "zero_point_headers": 0, "point_count": 0,
                "minimum_separation_m": None, "maximum_separation_m": None,
                "nonzero_impulse_points": 0, "maximum_point_impulse_ns": 0.0,
                "invalid_data_points": 0,
            })
            row["header_count"] += 1
            row["zero_point_headers"] += int(count == 0)
            for offset in range(int(header.contact_data_offset), int(header.contact_data_offset) + count):
                try:
                    point = contact_data[offset]
                    separation = float(point.separation)
                    impulse_norm = math.sqrt(sum(float(value) ** 2 for value in point.impulse))
                    if not math.isfinite(separation) or not math.isfinite(impulse_norm):
                        raise ValueError("nonfinite diagnostic point")
                except (AttributeError, IndexError, TypeError, ValueError):
                    row["invalid_data_points"] += 1
                    continue
                row["point_count"] += 1
                row["minimum_separation_m"] = separation if row["minimum_separation_m"] is None else min(row["minimum_separation_m"], separation)
                row["maximum_separation_m"] = separation if row["maximum_separation_m"] is None else max(row["maximum_separation_m"], separation)
                row["nonzero_impulse_points"] += int(impulse_norm > 0.0)
                row["maximum_point_impulse_ns"] = max(row["maximum_point_impulse_ns"], impulse_norm)

    def as_dict(self):
        return {
            "schema": "m710_raw_contact_report_probe_v1",
            "header_count": self.header_count,
            "event_point_count_distribution": self.event_point_count_distribution,
            "actor_pairs": [{"actors": list(pair), **row} for pair, row in sorted(self.actor_pairs.items())],
            "physics_or_event_filtering_changed": False,
        }


def read_effective_collision_offsets(view) -> dict:
    """Read backend values, separately from authored USD/default sentinels."""
    result = {"source": "PhysX tensor backend getters", "read_only": True}
    for name in ("contact_offsets", "rest_offsets"):
        values = getattr(view, f"get_{name}")()
        if hasattr(values, "numpy"):
            values = values.numpy()
        result[f"{name}_m"] = values.tolist()
    result["shape_path_mapping"] = "backend shape order; no unverified path-order assumption"
    return result


def verify_effective_collision_offsets(evidence, *, contact_offset_m, rest_offset_m, expected_shape_count):
    """Exact float32 backend equality, not a loosened numerical tolerance."""
    import math
    import struct

    contact, rest = float(contact_offset_m), float(rest_offset_m)
    if not math.isfinite(contact) or not math.isfinite(rest) or not contact > rest >= 0.0:
        raise ValueError("collision offsets must be finite with contact > rest >= 0")
    for key, expected in (("contact_offsets_m", contact), ("rest_offsets_m", rest)):
        values = [float(value) for row in evidence[key] for value in row]
        expected_f32 = struct.unpack("f", struct.pack("f", expected))[0]
        if len(values) != expected_shape_count or not all(value == expected_f32 for value in values):
            raise ValueError(f"effective {key} differs from the bound collision-offset contract: "
                             f"count={len(values)} expected_count={expected_shape_count} "
                             f"values={sorted(set(values))} expected_float32={expected_f32}")
    return {"status": "PASS", "shape_count": expected_shape_count, **evidence}


def author_explicit_collision_offsets(stage, *, contact_offset_m, rest_offset_m):
    """Author every collider; retain collision APIs, ownership and filters."""
    from pxr import PhysxSchema, Sdf, UsdPhysics

    contact, rest = float(contact_offset_m), float(rest_offset_m)
    # Validate before authoring any stage properties.
    import math
    if not math.isfinite(contact) or not math.isfinite(rest) or not contact > rest >= 0.0:
        raise ValueError("collision offsets must be finite with contact > rest >= 0")
    records = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr(contact)
        api.CreateRestOffsetAttr(rest)
        # Isaac 6 maps contactOffset=contactMargin+contactGap. Explicitly bind
        # both forms so a pre-existing Newton opinion cannot override PhysX.
        if not prim.ApplyAPI("NewtonCollisionAPI"):
            raise RuntimeError("installed Isaac SDK does not expose NewtonCollisionAPI")
        prim.CreateAttribute("newton:contactGap", Sdf.ValueTypeNames.Float, custom=False).Set(contact - rest)
        prim.CreateAttribute("newton:contactMargin", Sdf.ValueTypeNames.Float, custom=False).Set(rest)
        records.append({"collider": str(prim.GetPath()), "contact_offset_m": contact,
                        "rest_offset_m": rest, "newton_contact_gap_m": contact - rest})
    if not records:
        raise ValueError("no collider received explicit collision offsets")
    return records


def verify_authored_collision_offsets(stage, records):
    import struct

    for record in records:
        prim = stage.GetPrimAtPath(record["collider"])
        for name, expected in (("physxCollision:contactOffset", record["contact_offset_m"]),
                               ("physxCollision:restOffset", record["rest_offset_m"]),
                               ("newton:contactGap", record["newton_contact_gap_m"]),
                               ("newton:contactMargin", record["rest_offset_m"])):
            attribute = prim.GetAttribute(name)
            expected_f32 = struct.unpack("f", struct.pack("f", expected))[0]
            if not attribute or not attribute.HasAuthoredValueOpinion() or attribute.Get() != expected_f32:
                raise ValueError(f"authored collider offset mismatch: {record['collider']} {name}")
    return {"status": "PASS", "scope": "all USD collision shapes including static surfaces",
            "collider_count": len(records), "colliders": records}


def colliders_by_physical_body(link_paths: Mapping[str, str], collider_paths: Sequence[str]):
    """USD body hierarchy is nested; only the nearest body owns each shape."""
    result = {name: [] for name in link_paths}
    for collider in collider_paths:
        owners = [(path.count("/"), name) for name, path in link_paths.items()
                  if collider == path or collider.startswith(path + "/")]
        if owners:
            result[max(owners)[1]].append(collider)
    return {name: sorted(set(paths)) for name, paths in result.items()}


def expand_owned_tool_wrist_pairs(
    robot_link_colliders: Mapping[str, Sequence[str]],
    tool_collider_owners: Mapping[str, str],
    *,
    tool_owner: str,
    exempt_links: Sequence[str] = ("J5_link", "J6_link"),
) -> list[tuple[str, str]]:
    """Expand the approved physical-link/owned-tool rule to exact shape pairs.

    Ownership is supplied at shape creation, not inferred from path substrings.
    A tool-like name on an environment shape therefore grants no exemption.
    """
    links = tuple(exempt_links)
    if set(links) - {"J5_link", "J6_link"} or len(set(links)) != len(links):
        raise ValueError("only official J5_link/J6_link tool exemptions are authorized")
    if not tool_owner:
        raise ValueError("tool ownership identity is required")
    tools = sorted(path for path, owner in tool_collider_owners.items() if owner == tool_owner)
    if not tools:
        raise ValueError("owned tool colliders are required")
    pairs = []
    for link in links:
        shapes = tuple(robot_link_colliders.get(link, ()))
        if not shapes:
            raise ValueError(f"official physical wrist link has no robot colliders: {link}")
        if set(shapes) & set(tool_collider_owners):
            raise ValueError("robot and tool shape ownership must be distinct")
        pairs.extend((shape, tool) for shape in shapes for tool in tools)
    return sorted(set(pairs))


def apply_owned_tool_wrist_filters(stage, robot_link_colliders, tool_collider_owners,
                                   *, tool_owner, exempt_links=("J5_link", "J6_link")):
    """Author only CollisionAPI-to-CollisionAPI exclusions; no body filtering."""
    from pxr import UsdPhysics

    pairs = expand_owned_tool_wrist_pairs(
        robot_link_colliders, tool_collider_owners,
        tool_owner=tool_owner, exempt_links=exempt_links,
    )
    for first, second in pairs:
        for path in (first, second):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or not prim.HasAPI(UsdPhysics.CollisionAPI):
                raise ValueError(f"wrist/tool filter target is not a collider: {path}")
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError("wrist/tool filter must not target a rigid-body parent")
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(first)).CreateFilteredPairsRel().AddTarget(second)
    return [{"collider1": first, "collider2": second,
             "reason": "USER_APPROVED_SIMULATION_EXEMPTION",
             "scope": "OFFICIAL_J5_J6_VERSUS_OWNED_TOOL_COLLIDERS"}
            for first, second in pairs]


def expand_robot_only_srdf_pairs(
    link_paths: Mapping[str, str], link_colliders: Mapping[str, Sequence[str]],
    pairs: Sequence[tuple[str, str]],
) -> list[tuple[str, str]]:
    """Expand adjacency to official mesh shapes, never whole rigid bodies.

    The adapter captures the imported robot shapes before adding tool shapes.
    Additional checks prevent accidental use of the later tool descendants.
    """
    result = []
    for first, second in pairs:
        for name in (first, second):
            if name not in link_paths or not link_colliders.get(name):
                raise ValueError(f"SRDF link has no imported robot collision shapes: {name}")
            for path in link_colliders[name]:
                if path == link_paths[name] or not path.startswith(link_paths[name] + "/"):
                    raise ValueError("SRDF exclusions must target robot collision shapes, not whole bodies")
                if "tool" in path.lower() or "gripper" in path.lower():
                    raise ValueError("tool collision shapes cannot inherit robot SRDF exclusions")
        result.extend((a, b) for a in link_colliders[first] for b in link_colliders[second])
    return sorted(set(result))


def apply_robot_only_srdf_filters(stage, robot_root_path: str, srdf_path) -> list[dict]:
    """Call after importing the robot and before authoring mounted tool shapes."""
    from pxr import Usd, UsdPhysics

    # Isaac 6's importer stores the collision meshes below instanceable Xforms.
    # Stage.Traverse omits those instance proxies, and relationships cannot be
    # authored on a proxy. Materialize ONLY collision-bearing robot instances
    # in this stage, preserving the referenced mesh bytes and their transforms.
    # Visual instances and prototypes remain unchanged.
    materialized_instances = set()
    while True:
        instance_roots = set()
        collision_proxies = []
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())
            if (not path.startswith(robot_root_path + "/")
                    or not prim.HasAPI(UsdPhysics.CollisionAPI) or not prim.IsInstanceProxy()):
                continue
            if "tool" in path.lower() or "gripper" in path.lower():
                raise ValueError("tool collision shapes cannot inherit robot SRDF exclusions")
            collision_proxies.append(path)
            ancestor = prim.GetParent()
            while ancestor.IsValid() and str(ancestor.GetPath()).startswith(robot_root_path + "/"):
                if ancestor.IsInstance() and not ancestor.IsInstanceProxy():
                    instance_roots.add(str(ancestor.GetPath()))
                    break
                ancestor = ancestor.GetParent()
        if not collision_proxies:
            break
        if not instance_roots or instance_roots <= materialized_instances:
            raise RuntimeError("cannot author collider-scoped SRDF filters on imported instance proxies")
        for path in sorted(instance_roots):
            stage.GetPrimAtPath(path).SetInstanceable(False)
        materialized_instances.update(instance_roots)

    prims = [p for p in stage.Traverse()
             if str(p.GetPath()) == robot_root_path or str(p.GetPath()).startswith(robot_root_path + "/")]
    link_paths = {}
    for prim in prims:
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if prim.GetName() in link_paths:
                raise ValueError("ambiguous imported robot physical link name")
            link_paths[prim.GetName()] = str(prim.GetPath())
    by_path = {value: key for key, value in link_paths.items()}
    colliders = {name: [] for name in link_paths}
    for prim in prims:
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            ancestor = prim.GetParent()
            while ancestor.IsValid():
                name = by_path.get(str(ancestor.GetPath()))
                if name is not None:
                    colliders[name].append(str(prim.GetPath()))
                    break
                ancestor = ancestor.GetParent()
    elements = ET.parse(srdf_path).getroot().findall("disable_collisions")
    pairs = [(e.attrib["link1"], e.attrib["link2"]) for e in elements]
    expected_links = {"base_link", *(f"J{i}_link" for i in range(1, 7))}
    if set(link_paths) != expected_links or any(len(colliders[name]) != 1 for name in expected_links):
        raise ValueError("official M-710 import must expose exactly seven robot collision meshes, one per physical link")
    expanded = expand_robot_only_srdf_pairs(link_paths, colliders, pairs)
    # A URDF importer may already have whole-body filters. Replace them with
    # the audited mesh-pair exclusions before the tool is inserted into J6.
    for prim in prims:
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            # An explicit empty list overrides filters composed from the
            # imported USD. ClearTargets(removeSpec=True) can instead expose
            # weaker-layer body filters again and hide tool/J5 contacts.
            relation = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
            relation.SetTargets([])
            if relation.GetTargets():
                raise RuntimeError("could not remove imported body-level collision filters")
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateCollisionEnabledAttr(True)
    for first, second in expanded:
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(first)).CreateFilteredPairsRel().AddTarget(second)
    return [{"link1": a, "link2": b, "actor1": link_paths[a], "actor2": link_paths[b],
             "reason": e.attrib.get("reason", "SRDF"), "scope": "OFFICIAL_ROBOT_COLLIDERS_ONLY",
             "materialized_collision_instances": sorted(materialized_instances),
             "collider_pairs": [[x, y] for x in colliders[a] for y in colliders[b]]}
            for (a, b), e in zip(pairs, elements)]
