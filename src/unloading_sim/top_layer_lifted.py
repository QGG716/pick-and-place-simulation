"""Frozen top-layer identity and geometry-driven process dispatch.

This module does not synthesize native paths or claim a carton was delivered.
Lower layers never enter the batch when the current exposed faces change.
"""
from copy import deepcopy
from dataclasses import dataclass

import numpy as np

from .lifted_reception import StartHeight, bounds, directional_contact, layer_role, receiving_region
from .moveit2_backend import box_message, digest
from .support import SupportRelationGraph


@dataclass(frozen=True)
class FrozenTopLayer:
    target_ids: tuple
    original_cartons_json: str
    evidence_json: str

    @classmethod
    def capture(cls, cartons):
        import json
        cartons = tuple(cartons)
        if not cartons or len({b.name for b in cartons}) != len(cartons):
            raise ValueError("INVALID_BATCH_CARTON_IDENTITIES")
        graph = SupportRelationGraph.build(cartons)
        top = max(bounds(b)[1][2] for b in cartons)
        targets = [b for b in cartons if abs(bounds(b)[1][2]-top) <= 1e-6 and not graph.supports[b.name]]
        targets.sort(key=lambda b: (b.center[1], b.center[0], b.name))
        original = [box_message(b) for b in cartons]
        roles = {b.name: layer_role(b, cartons, cartons) for b in targets}
        evidence = dict(target_ids=[b.name for b in targets], original_carton_count=len(cartons),
                        original_cartons_sha256=digest(original), highest_top_z_m=float(top),
                        roles=roles, heights={b.name: StartHeight.capture(b).evidence() for b in targets},
                        support_graph=graph.audit(), selection="INITIAL_HIGHEST_TOP_AND_NO_SUPPORTED_CARTONS")
        return cls(tuple(evidence["target_ids"]), json.dumps(original, sort_keys=True),
                   json.dumps(evidence, sort_keys=True))

    def evidence(self):
        import json
        return json.loads(self.evidence_json)

    def remaining_ids(self, completed):
        completed = tuple(completed)
        if len(set(completed)) != len(completed) or set(completed)-set(self.target_ids):
            raise ValueError("COMPLETION_OUTSIDE_FROZEN_LAYER_OR_DUPLICATE")
        return tuple(name for name in self.target_ids if name not in completed)


def process_options(target, belt, frozen, remaining_cartons, *, occupied=(), longitudinal=None):
    """Front-first: full footprint decides Hmax versus lateral/Hmin.

    A middle carton outside the -Y belt boundary gets the same +Y process as
    the right endpoint. Side alternatives preserve the ORIGINAL end role.
    This is upstream process selection, not a hidden IK/roll/goal scan.
    """
    if target.name not in frozen.target_ids:
        raise ValueError("TARGET_OUTSIDE_FROZEN_TOP_LAYER")
    capture = frozen.evidence()
    expected = capture["heights"][target.name]
    if StartHeight.capture(target).box_identity != expected["start_box_identity"]:
        raise ValueError("FROZEN_TARGET_GEOMETRY_CHANGED")
    names = {b.name for b in remaining_cartons}
    if target.name not in names:
        raise ValueError("TARGET_NOT_IN_REMAINING_STACK")
    region = receiving_region(target, belt, occupied)
    a, b, c, d = region["projection_center_limits_before_occupancy"]
    y = float(((target.center-belt.center) @ belt.rotation)[1])
    options = []
    reception = None
    if longitudinal is not None and a <= b and c <= y <= d:
        from .junction_reception import reception_classification
        from .lifted_reception import translated
        projected = translated(target, [belt.center[0]+(a+b)/2-target.center[0],0,0])
        reception = reception_classification(projected,belt,longitudinal)
    if a <= b and c <= d:
        if c <= y <= d:
            junction = reception is not None and reception["junction"]
            name, height, legs = ("FRONT_DIRECT_HMIN" if junction else "FRONT_DIRECT_HMAX",
                                 expected["Hmin_world_m"] if junction else expected["Hmax_world_m"], [[-1.,0.,0.]])
        else:
            sign = 1. if y < c else -1.
            name, height = "FRONT_LATERAL_HMIN", expected["Hmin_world_m"]
            legs = [[0., sign, 0.], [-1., 0., 0.]]
        options.append(dict(template=name, face=directional_contact(target, [-1., 0., 0.]),
                            height_m=height, segment_directions_world=legs,
                            turn_center_y_interval_world_m=[float(c+belt.center[1]), float(d+belt.center[1])],
                            region=deepcopy(region), reception=deepcopy(reception),
                            requires_open_channel=len(legs) == 2))
        if name=="FRONT_DIRECT_HMAX":
            options.append(dict(options[-1],template="FRONT_DIRECT_HMIN",
                height_m=expected["Hmin_world_m"]))
    role = capture["roles"][target.name]["role"]
    if role in {"LEFT_END", "RIGHT_END"}:
        sign = -1. if role == "LEFT_END" else 1.
        options.append(dict(template="LEFT_INNER_HMIN" if sign < 0 else "RIGHT_INNER_HMAX",
                            face=directional_contact(target, [0., sign, 0.]),
                            height_m=expected["Hmin_world_m"] if sign < 0 else expected["Hmax_world_m"],
                            segment_directions_world=[[0., sign, 0.], [-1., 0., 0.]],
                            region=deepcopy(region), requires_open_channel=True))
    for option in options:
        option.update(original_role=role, target_id=target.name,
                      height_source="FROZEN_INITIAL_TARGET", endpoint="REGION_NOT_FIXED_POINT",
                      acceptance="NOT_PLANNED", spatial_orientation="UPSTREAM_FACE_POSE_HELD",
                      fixed_wrist_joint_values=False)
    return options


def rank_remaining(frozen, cartons, completed, q, robot, belt, *, longitudinal=None):
    """Current q and geometry; prefer direct exits and opening central space."""
    remaining = set(frozen.remaining_ids(completed))
    active = {b.name: b for b in cartons}
    if not remaining <= set(active):
        raise ValueError("UNCOMPLETED_TARGET_MISSING_FROM_WORLD")
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all() or not robot.within_limits(q):
        raise ValueError("INVALID_CURRENT_ROBOT_STATE")
    tcp = robot.fk(q)[:3, 3]
    center_y = float(np.mean([active[n].center[1] for n in remaining])) if remaining else 0.
    graph = SupportRelationGraph.build(cartons)
    rows = []
    for name in remaining:
        target = active[name]
        options = process_options(target, belt, frozen, cartons, longitudinal=longitudinal)
        direct = bool(options and options[0]["template"].startswith("FRONT_DIRECT_"))
        channel=[]
        if options and options[0]['template']=='FRONT_LATERAL_HMIN':
            direction=options[0]['segment_directions_world'][0][1]
            lo,hi=bounds(target)
            for other in cartons:
                if other.name==name:continue
                ol,oh=bounds(other)
                # A same-height neighbor lies between this carton and the belt
                # region. Rank an already open channel first; do not label a
                # blocked current-world process globally unreachable.
                if min(hi[0],oh[0])-max(lo[0],ol[0])>1e-9 and min(hi[2],oh[2])-max(lo[2],ol[2])>1e-9:
                    if direction*(other.center[1]-target.center[1])>0 and direction*(other.center[1]-belt.center[1])<0:
                        channel.append(other.name)
        rows.append(dict(target_id=name, options=options,
                         lateral_channel_blockers=sorted(channel),
                         supported_cartons=sorted(graph.supports[name]),
                         front_blockers=graph.face_blockers(name, "front"),
                         rank=[not direct, bool(channel), abs(float(target.center[1])-center_y),
                               float(np.linalg.norm(target.center-tcp)), name]))
    return sorted(rows, key=lambda row: row["rank"])
