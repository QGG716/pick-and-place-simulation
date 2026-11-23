"""Offline check of a single-joint sweep against a saved real-site scene (no hardware, no motion).

Builds the single-carton planner on a saved field record (frozen RGB-D site
scene and measured boxes, ``--frozen-scene`` semantics), takes the approach-
stage context of the first target (no contact permission, no attachment, the
target's full envelope is an obstacle; tool/robot reserve and pair clearances
as configured), and runs the unchanged sent-path check, forced to SAMPLED
(every native sample checked exactly), on the joint-space line from the
record's q_start to the goal. Used before an operator-executed controlled
empty move (2026-11-23: J6 -5.15 -> -185.15 deg, start S). Not a plan, not
executable; the controlled entry plans and checks its own trajectory again.

Usage: python tools/check_joint_sweep_site.py RECORD --config CONFIG --goal-deg J1 .. J6 --output DIR
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('record', type=Path)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--goal-deg', type=float, nargs=6, required=True)
    parser.add_argument('--step-deg', type=float, default=.5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from unloading_sim.workbench.single_carton.config import load
    from unloading_sim.workbench.single_carton.planner import SinglePlanner, SegmentRejected
    from unloading_sim.workbench.single_carton.geometry import contact_candidates
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA, load_mapping
    config = load(args.config)
    config['sent_path_audit'] = 'SAMPLED'          # every sample exact, no proven skips
    config.pop('adaptive_constraints', None); config.pop('adaptive_rule_transforms', None)
    obs = json.loads((args.record/'observation.json').read_text())
    geometry = json.loads((args.record/'geometry.json').read_text())
    scene = json.loads((args.record/'site-scene.json').read_text())
    boxes = geometry['boxes']
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), model)
    planner.build()
    world = json.loads(json.dumps(scene['world']))+[dict(id=b['id'], size=b['size_m'], pose=b['pose'], category='carton') for b in boxes]
    out = args.output; out.mkdir(parents=True, exist_ok=False)
    q0 = np.asarray(obs['q_start'], float); goal = np.deg2rad(args.goal_deg)
    n = max(2, int(np.ceil(np.max(abs(goal-q0))/np.deg2rad(args.step_deg)))+1)
    path = [(q0+(goal-q0)*k/(n-1)).tolist() for k in range(n)]
    report = dict(schema='eco65_joint_sweep_site_check_v1', date=time.strftime('%Y-%m-%d %H:%M:%S'),
                  scope='OFFLINE_SAVED_SITE_SCENE_CHECK_NOT_A_PLAN_NOT_EXECUTABLE', record=str(args.record),
                  observation_id=obs.get('observation_id'), config=str(args.config), q_start_deg=np.rad2deg(q0).tolist(),
                  goal_deg=list(args.goal_deg), probe_states=n, audit='SAMPLED', stage_rules='approach (no contact, no attachment)')
    planner.start_task_services(world, out)
    planner.start_main_worker(out)
    try:
        planner.acquire_workers(out, boxes)
        box = boxes[0]
        candidate = contact_candidates(box, planner.source['tool'], planner.site['topology'], planner.site.get('contact_search'))[0]
        context = planner.context(q0, world, box, candidate, 'approach')
        segment = dict(stage='approach', record='joint-sweep', context=context, planner=dict(note='OFFLINE_SWEEP'), native_result={},
                       prepared=dict(collision_path=path, points=[dict(t=0., q=path[0]), dict(t=1., q=path[-1])]))
        started = time.monotonic()
        try:
            planner.sent_check(segment, out, lambda text: print(text, flush=True))
            checked = segment['evidence']['collision_result']
            report.update(result='PASS', wall_s=time.monotonic()-started,
                          checked_states=checked['sent_path_audit']['checked_states'], parallel_checkers=checked['parallel_checkers'])
        except SegmentRejected as error:
            report.update(result='REJECTED', wall_s=time.monotonic()-started, failure=str(error)[:2000])
    finally:
        planner.release_workers(out)
        planner.close()
    (out/'joint-sweep-check.json').write_text(json.dumps(report, indent=1, ensure_ascii=False, default=str))
    print(json.dumps({k: v for k, v in report.items() if k not in ('q_start_deg',)}, ensure_ascii=False, default=str))
    return 0 if report.get('result') == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
