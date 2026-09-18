"""Offline reproduction using the saved pair and unchanged production queries.
Run from repository root: PYTHONPATH=src python path/to/this_file.py
No simulator is started. This does not reconstruct a PhysX contact manifold.
"""
import json
from pathlib import Path
import numpy as np
from unloading_sim.geometry import OBB
from unloading_sim.serial_unloading import rotation_from_actual_quaternion
from unloading_sim.collision_policy import SimulationCollisionPolicy
from unloading_sim.pair_clearance import obb_surface_distance
from unloading_sim.isaac_collision_policy import classify_poc_runtime_pair
from unloading_sim.m710_replay_physics import ActualStackContactMonitor

p = Path(__file__).parent
f = json.loads((p / 'second_carton_failure_fixture.json').read_text())
expected = json.loads((p / 'second_carton_failure_review.json').read_text())
policy = SimulationCollisionPolicy.from_mapping(f['collision_policy'])
a, b = [OBB(x['position_m'], np.asarray(x['size_m']) / 2,
            rotation_from_actual_quaternion(x['orientation_wxyz']), name=x['name'])
        for x in f['current_pair']]
ia, ib = [OBB(x['center_m'], np.asarray(x['size_m']) / 2,
              x['rotation_matrix'], name=x['name']) for x in f['initial_pair']]
c = f['contact']
result = classify_poc_runtime_pair(
    collider0=c['colliders'][0], collider1=c['colliders'][1],
    minimum_separation_m=c['minimum_separation_m'], policy=policy,
    robot_link_by_collider={}, owned_tool_colliders=set(), stage=c['stage'],
    permission_source=c['pair_evidence']['contact_permission_source'])
gap = obb_surface_distance(a, b)
monitor = ActualStackContactMonitor(ia, [ib], policy)
observation = monitor.observe(f['segment_time_s'], a, [b], commanded_motion=True)
assert result == expected['production_pair_classifier_replay']
assert abs(gap - expected['current_pair_obb_distance_m']) < 1e-12
assert observation['accepted'] and observation['free_space_reached']
assert not result['accepted'] and result['classification'] == 'CLEARANCE_INSUFFICIENT'
print(json.dumps({'offline_replay': 'PASS', 'contact_classification': result['classification'],
                  'contact_point_separation_m': c['minimum_separation_m'],
                  'current_box_minimum_surface_distance_m': gap,
                  'current_pair_monitor_accepted': observation['accepted'],
                  'simulator_started': False}, indent=2))
