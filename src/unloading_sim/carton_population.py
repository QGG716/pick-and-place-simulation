"""Whole-population dispatch using the existing frozen per-layer processes.

This changes scheduling scope, not grasp permissions, collision or reception.
Failed cartons remain physical obstacles and supports. A task's solver budget is
never reset simply because a different carton later succeeds.
"""
import json

from .lifted_reception import StartHeight, bounds, layer_role
from .moveit2_backend import box_message, digest
from .support import SupportRelationGraph
from .top_layer_lifted import FrozenTopLayer, rank_remaining


class FrozenCartonPopulation(FrozenTopLayer):
    @classmethod
    def capture(cls, cartons):
        cartons = tuple(cartons)
        if not cartons or len({b.name for b in cartons}) != len(cartons):
            raise ValueError("INVALID_BATCH_CARTON_IDENTITIES")
        targets = sorted(cartons, key=lambda b: (-bounds(b)[1][2], b.center[1], b.center[0], b.name))
        original = [box_message(b) for b in cartons]
        evidence = dict(
            target_ids=[b.name for b in targets], original_carton_count=len(cartons),
            original_cartons_sha256=digest(original),
            roles={b.name: layer_role(b, cartons, cartons) for b in targets},
            heights={b.name: StartHeight.capture(b).evidence() for b in targets},
            support_graph=SupportRelationGraph.build(cartons).audit(),
            selection="ALL_ORIGINAL_CARTONS_HIGHEST_REMOVABLE_FIRST",
            eccentric_authority="NONE_ADDED_BY_POPULATION_SCOPE",
        )
        return cls(tuple(evidence["target_ids"]), json.dumps(original, sort_keys=True),
                   json.dumps(evidence, sort_keys=True))


def rank_population(frozen, cartons, completed, attempted, q, robot, belt, *, longitudinal):
    active = {b.name: b for b in cartons}
    rows = rank_remaining(frozen, cartons, completed, q, robot, belt, longitudinal=longitudinal)
    available = [r for r in rows if not r["supported_cartons"] and r["target_id"] not in attempted]
    return sorted(available, key=lambda r: (-bounds(active[r["target_id"]])[1][2], *r["rank"]))


def blocked_population(cartons, attempted):
    graph = SupportRelationGraph.build(cartons)
    return [dict(target_id=b.name, status="BLOCKED_BY_RETAINED_SUPPORT_DEPENDENTS",
                 supported_cartons=sorted(graph.supports[b.name]), native_search_performed=False)
            for b in cartons if b.name not in attempted and graph.supports[b.name]]
