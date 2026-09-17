"""Replay the saved pair classification and pose geometry, without Isaac."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.geometry import OBB
from unloading_sim.isaac_collision_policy import classify_poc_runtime_pair
from unloading_sim.pair_clearance import obb_surface_distance
from unloading_sim.serial_unloading import rotation_from_actual_quaternion


def main():
    evidence = json.loads(Path(__file__).with_name("physical_trial_summary.json").read_text())
    for name, digest in evidence["classifier_source_sha256"].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"classifier implementation changed: {name}")
    event = evidence["first_rejection"]
    observed = classify_poc_runtime_pair(
        collider0=event["colliders"][0], collider1=event["colliders"][1],
        minimum_separation_m=event["minimum_separation_m"],
        policy=SimulationCollisionPolicy.from_mapping(evidence["collision_policy"]),
        robot_link_by_collider={}, owned_tool_colliders=set(),
        stage=event["stage"], permission_source=None,
    )
    if observed != event["pair_evidence"]:
        raise ValueError("recorded PhysX pair classification differs")
    carton, receiver = evidence["carton_state"], evidence["receiver_primitive"]
    payload = OBB(carton["position_m"], np.array(evidence["carton_size_m"]) / 2,
                  rotation_from_actual_quaternion(carton["orientation_wxyz"]),
                  carton["name"], "carton")
    deck = OBB(receiver["center_m"], np.array(receiver["size_m"]) / 2,
               np.array(receiver["rotation_matrix"]), receiver["name"], "conveyor")
    distance = obb_surface_distance(payload, deck)
    expected = evidence["pair_distance_offline_m"]["actual_carton_pose"]
    if abs(distance - expected) > 1e-12 or not 0 < distance < 0.005:
        raise ValueError("saved pose geometry does not reproduce clearance shortfall")
    print(json.dumps(dict(classification_replay_matches=True,
        classification=observed["classification"], stage=event["stage"],
        q_rad=evidence["q_rad"], pair=event["colliders"],
        recorded_physx_separation_m=event["minimum_separation_m"],
        offline_pose_obb_distance_m=distance, required_pair_clearance_m=0.005,
        scope="RECORDED_PAIR_CLASSIFICATION_AND_POSE_GEOMETRY_NOT_NEW_PHYSX_RUN"), indent=2))


if __name__ == "__main__":
    main()
