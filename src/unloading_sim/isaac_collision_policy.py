"""Optional USD adapter for collider-scoped robot adjacency exclusions."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import xml.etree.ElementTree as ET


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
