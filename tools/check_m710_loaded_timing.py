"""Audit a fully accepted native loaded segment with the existing CPU time law.

This does not create an execution bundle or start Isaac.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input
from unloading_sim.planning_profile import DEFAULT_EXECUTION
from unloading_sim.m710_execution import load_m710_execution_config,_bridge_configuration
from unloading_sim.m710_dynamics import load_m710id70_dynamics
from unloading_sim.moveit2_timing import native_timing_floor,preserve_native_durations
from unloading_sim.timing import motion_limits_from_config,time_parameterize_joint_path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--accepted',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();started=perf_counter();source=json.loads(args.accepted.read_text())
    assert source['authoritative_status']=='PASS' and source['failure'] is None
    execution=load_m710_execution_config(ROOT/DEFAULT_EXECUTION);policy=load_layout_motion_policy(execution.motion_policy_path)
    scene=build_verified_motion_input(policy,ROOT)
    assert scene.snapshot['scene_fingerprint']==source['scene_fingerprint']
    assert policy.policy_fingerprint==source['policy_fingerprint']
    cfg=_bridge_configuration(scene,load_m710id70_dynamics(execution.dynamics_path),execution)
    limits=motion_limits_from_config(cfg,6);path=np.asarray(source['path'])
    records=[a for a in source['evidence']['attempts'] if a.get('authoritative_status')=='PASS']
    floors,used=native_timing_floor(path,records);assert len(used)==1 and used[0]['native_output_status']=='PASS'
    timed=time_parameterize_joint_path(path,limits)
    duration_inputs=np.maximum(np.diff(timed.time_from_start),np.asarray(floors))
    timed=preserve_native_durations(timed,dict(path=path.tolist(),native_backend=dict(minimum_edge_seconds=floors)),list(range(len(path))))
    audit=timed.audit(limits)
    assert audit['within_limits'] and np.array_equal(timed.positions,path)
    # Check the existing floor operation exactly, before cumulative timestamps.
    # Subtracting two accumulated floats need not recover the input bitwise;
    # record that rounding, without adding any collision or timing tolerance.
    assert np.all(duration_inputs>=np.asarray(floors))
    assert np.array_equal(timed.time_from_start,np.r_[0.,np.cumsum(duration_inputs)])
    report=dict(status='PASS',scope='single accepted loaded segment CPU timing contract; no execution readiness or physical execution claim',
        accepted_sha256=hashlib.sha256(args.accepted.read_bytes()).hexdigest(),
        audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        scene_fingerprint=scene.snapshot['scene_fingerprint'],policy_fingerprint=policy.policy_fingerprint,
        execution_config=str(execution.config_path.relative_to(ROOT)),limits=cfg['execution'],
        minimum_edge_seconds=floors,actual_edge_seconds=np.diff(timed.time_from_start).tolist(),audit=audit,
        duration_s=timed.duration_seconds,geometry_unchanged=True,native_duration_floor_preserved=True,
        floor_audit='exact pre-accumulation durations and exact accumulated knots; no added tolerance',
        minimum_timestamp_subtraction_margin_s=float(np.min(np.diff(timed.time_from_start)-np.asarray(floors))),
        wall_s=perf_counter()-started,isaac='NOT_RUN')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(report['status'],report['wall_s'],report['duration_s'])

if __name__=='__main__':main()
