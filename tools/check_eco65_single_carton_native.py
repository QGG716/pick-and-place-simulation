"""Offline native component check: never connects hardware or mints a permit.

Uses a fixed regression posture and a synthetic far-away box. This is NOT a
four-stage physical cycle or a substitute for a real frozen observation.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import numpy as np
from unloading_sim.workbench.single_carton.config import load
from unloading_sim.workbench.single_carton.planner import SinglePlanner
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, URDF, URDF_SHA, ROOT
from unloading_sim.workbench.coordinates import Installation, CoordinateModel
from unloading_sim.moveit2_backend import ResidentMoveItClient
from unloading_sim.workbench.single_carton.geometry import corners, contact_candidates


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    folder = args.output or ROOT/'outputs/single_carton_development'/('native-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S'))
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    config = load(args.config)
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    report = dict(scope='OFFLINE_NATIVE_COMPONENT_NOT_FULL_CYCLE', hardware_execution='NOT_RUN',
                  input_source='SYNTHETIC_REGRESSION_FIXTURE', status='RUNNING')
    try:
        planner.build()
        q = np.deg2rad([86.447, 16.414, -69.186, -30.712, 86.945, 77.313])
        pose = np.eye(4); pose[:3, 3] = [.8, .8, .8]
        box = dict(id='offline-target', pose=pose.tolist(), size_m=[.15, .1, .1],
                   corners_m=corners(pose, [.15, .1, .1]).tolist(),
                   faces=[dict(observed=True, center_m=[.8, .8, .85], normal=[0, 0, 1],
                               u=[1, 0, 0], v=[0, 1, 0], size_m=[.15, .1])])
        candidate = contact_candidates(box, planner.source['tool'], config['topology'])[0]
        world = [x for x in planner.world if x.get('category') != 'carton']
        world += [dict(id=box['id'], pose=box['pose'], size=box['size_m'], category='carton')]
        planner.client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(folder),
            str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=folder/'worker.log')
        planner.client.request(planner.init)
        context = planner.context(q, world, box, candidate, 'approach')
        goal = q.copy(); goal[1] += .003
        start = time.monotonic()
        segment = planner.checked_segment(context, goal, 'offline-small-PTP', 10., folder, print)
        report.update(status='PASS', native_plan=segment['evidence']['native_plan'],
                      sent_path=segment['evidence']['sent_path_collision'],
                      points=len(segment['prepared']['points']),
                      duration_s=segment['prepared']['send_duration_s'], elapsed_s=time.monotonic()-start,
                      binding=planner.binding())
    except Exception as error:
        report.update(status='FAILED', reason=str(error))
        raise
    finally:
        (folder/'result.json').write_text(json.dumps(report, indent=2))
        planner.close()
        print(json.dumps({k:v for k,v in report.items() if k!='binding'}, indent=2))


if __name__ == '__main__':
    main()
