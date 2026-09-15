"""Analyze archived actual commands and profile exact states without rerunning Isaac."""
import argparse
import cProfile
import hashlib
import json
from pathlib import Path
import pstats
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from unloading_sim.motion_quality import path_quality
from unloading_sim.layout_single_carton import load_layout_motion_policy, build_verified_motion_input, _build_automatic_trajectory_connector
from unloading_sim.serial_unloading import apply_actual_motion_state
from unloading_sim.unloading_sequence import RowUnloadingState


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True)
    ap.add_argument('--segment', type=int, default=3)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    previous = args.run/'isaac'/(f'segment_{args.segment-1:03}' if args.segment > 2 else '')
    current = args.run/'isaac'/f'segment_{args.segment:03}'
    motion_file = args.run/'repo/outputs'/f'ideal_plan_{args.segment:03}'/'motion.json'
    motion = json.loads(motion_file.read_text())
    segment = motion['selected_trajectory_segment']
    policy = load_layout_motion_policy('configs/validation/m710id70_layout_v1_single_carton.yaml')
    scene = build_verified_motion_input(policy)
    rows = RowUnloadingState(); rows.rank(scene.cartons, support_graph=scene.support_graph)
    actual_file = previous/'actual_remaining_state.json'
    scene = apply_actual_motion_state(scene, json.loads(actual_file.read_text()), row_state=rows)
    connector = _build_automatic_trajectory_connector(scene, policy.layout_validation.layout.robot()).connector
    connector.start_planning_request()
    path = np.asarray(segment['path'])
    start, end = segment['stage_ranges'].get('pregrasp', (0, 0))
    if end == start:  # Archived direct mode: recover the exact checked gate, not a guessed stage label.
        success = next(a for a in segment['approach']['attempts'] if a['failure'] is None)
        terminal_samples = success['search']['terminal']['samples']
        end = segment['grasp_index'] - len(terminal_samples)
    assert end > start
    def metrics(q):
        return path_quality(q, fk=connector.robot.fk, joint_limits=connector.robot.joint_limits)
    report = dict(target=segment['target'], baseline_run=str(args.run),
                  archived_execution_source=str(args.run/'execution_source.tar'),
                  motion_sha256=hashlib.sha256(motion_file.read_bytes()).hexdigest(),
                  actual_start_sha256=hashlib.sha256(actual_file.read_bytes()).hexdigest(),
                  free_source_indices=[start, end], geometry=metrics(path[start:end+1]),
                  old_timing_claimed_stops=max(0,end-start-1),
                  old_runtime_interpolation='PIECEWISE_LINEAR_KNOT_VELOCITY_DISCONTINUITIES_NOT_QUINTIC_STOPS',
                  planning_wall_s=motion['planning_performance']['planning_total_wall_seconds'])
    bundle = json.loads((motion_file.parent/'replay_bundle.json').read_text())
    windows = bundle['metadata']['stage_windows']
    free_end = next((w['end_time_s'] for w in windows if w['stage']=='pregrasp'), None) if isinstance(windows, list) else None
    if free_end is None:
        # Source times are explicitly bound in current bundles; archived bundles
        # may only expose the merged contact window. Recover by matching the gate
        # command sample and report the observed timestamp.
        ts = np.asarray(bundle['timestamps_seconds']); qs = np.asarray(bundle['positions_rad'])
        free_end = float(ts[np.argmin(np.max(np.abs(qs-path[end]), axis=1))])
    for label, folder in [('previous', previous), ('next', current)]:
        events = json.loads((folder/'execution_events.json').read_text())['events']
        report[label+'_events'] = [{k:v for k,v in e.items() if k in
            ('event','time_s','simulation_time_s','trajectory_time_s','accepted','target')}
            for e in events if 'release' in e['event'] or 'grasp' in e['event']]
        csv = np.genfromtxt(folder/'joint_tracking.csv', delimiter=',', names=True)
        times = csv['time_s']
        grasp = next((e['simulation_time_s'] for e in events if e['event']=='grasp_contact_attempt' and e.get('accepted')), times[-1])
        if label == 'next':
            spans = [('free', 0, free_end), ('terminal', free_end, grasp)]
        else:
            release = [e.get('simulation_time_s',e.get('time_s')) for e in events
                       if e['event'] == 'release_constraint_removal_confirmed']
            # Accepted release events without times can be corroborated by result metadata.
            release = [t for t in release if t is not None]
            spans = [('withdrawal', max(release) if release else float(times[-1]), float(times[-1]))]
        for span, a, b in spans:
            take = (times >= a-1e-7) & (times <= b+1e-7)
            command = np.column_stack([csv[f'command_J{i}_rad'][take] for i in range(1,7)])
            measured = np.column_stack([csv[f'measured_J{i}_rad'][take] for i in range(1,7)])
            report[span] = dict(start_s=float(a), end_s=float(b), physical_duration_s=float(b-a),
                command=metrics(command), measured=metrics(measured),
                maximum_tracking_error_rad=float(np.max(np.abs(measured-command))),
                rms_tracking_error_rad=float(np.sqrt(np.mean((measured-command)**2))))
        result = json.loads((folder/'result.json').read_text())
        report[label+'_execution'] = {k:v for k,v in result.items() if k in
            ('qualification_checks','qualification_failures','runtime_stop_reason','replay_wall_seconds',
             'replayed_simulation_seconds','physical_cycle_completed','joint_telemetry_semantics')}
    profile = cProfile.Profile(); profile.enable()
    checked = []
    for q in path[start:end+1]:
        checked.append(connector.validate_unloaded_state(q, scene.all_obstacles, stage='pregrasp'))
    profile.disable(); profile.dump_stats(str(args.output/'state_validation.prof'))
    with (args.output/'state_validation_profile.txt').open('w') as stream:
        pstats.Stats(profile, stream=stream).sort_stats('cumulative').print_stats(45)
    report['profile_state_count'] = len(checked)
    report['profile_failures'] = [v for v in checked if v]
    (args.output/'baseline.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k in ('target','geometry','free','terminal','profile_failures')}))


if __name__ == '__main__':
    main()
