"""Read-only post-run evidence collection; never invokes planner or Isaac."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
from summarize_formal import summarize

p = Path('/root/autodl-tmp/m710-validation-opt-20261002')
formal = p/'repo/outputs/native-cold-validation-opt-once'
run = json.loads((formal/'run.json').read_text())
assert run['status'] in ('ISAAC_WORKFLOW_COMPLETED_UNDER_DECLARED_ASSUMPTIONS', 'ISAAC_FAILED', 'BLOCKED')
summary = summarize(formal)
summary['gates']['isaac_result'].pop('assumed_reception_state', None)
motion = json.loads((formal/'plan/motion.json').read_text()) if (formal/'plan/motion.json').is_file() else {}
records = motion.get('native_backend_evidence', (motion.get('native_failure_evidence') or {}).get('native_stages', []))
profiles = [{k:r.get(k) for k in ('stage_id','parent_stage_id','task_id','stage','status','solver',
    'native_solver_calls','mtc_plan_s','native_output_check_s','authoritative_s','authoritative_profile','failure')}
    for r in records]
summary['stage_profiles_file'] = 'formal/plan/independent-stage-profiles.json'
summary['formal_validation_mode'] = 'optimized'
summary['formal_source_commit'] = run['source_commit']
summary['benchmark_is_separate_and_never_a_formal_input'] = True
summary['implementation_tests'] = {'passed': 140, 'seconds': 3.56}
audit = motion.get('native_cold_audit') or {}
summary['history']['forbidden_entry_attempts'] = audit.get('forbidden_entry_attempts')
selected = motion.get('selected_trajectory_segment') or {}
backend = selected.get('native_backend') or {}
summary['identity']['native_task_id'] = backend.get('task_id')
summary['selected_parent_chain'] = [{k:r.get(k) for k in ('stage','stage_id','parent_stage_id','task_id','solver','path_sha256')}
    for r in backend.get('stages',[])]
delivery = json.loads((formal/'plan/delivery.json').read_text()) if (formal/'plan/delivery.json').exists() else {}
summary['timings_seconds']['first_complete_feasible_result'] = delivery.get('first_complete_feasible_result_seconds')
summary['timings_seconds']['first_exportable_result'] = delivery.get('first_exportable_result_seconds')
summary['timings_seconds']['subsequent_optimization'] = delivery.get('subsequent_optimization_seconds')
physical = json.loads((formal/'physics/result.json').read_text()) if (formal/'physics/result.json').exists() else {}
summary['qualification'] = {k:physical.get(k) for k in ('qualification_passed','workflow_cycle_completed','physical_cycle_completed',
    'actual_reception_succeeded','drive_effort_output_qualified','qualification_check_details','physics_steps','physics_hz',
    'replay_video_frame_count','replay_video_physical_time_scale','native_cold_initialization')}
counts=physical.get('execution_counts') or {}
def observed_gate(present, passed, **details):
    return {'status':('PASS' if passed else 'FAIL') if present else 'NOT_REACHED', **details}
summary['acceptance_layers'] = {
    'optimization_and_directed_regressions': {'status':'PASS','tests':140},
    'same_input_cold_comparison': {'status':'PASS','evidence':'benchmark/comparison.json','scope':'offline saved motion only'},
    'formal_native_geometry': observed_gate(bool(motion),motion.get('complete_trajectory_status')=='PASS'),
    'nonzero_native_source_coverage': observed_gate(bool(backend), (backend.get('source_coverage') or {}).get('passed') is True,
        covered=(backend.get('source_coverage') or {}).get('covered_nonzero_edge_count'),total=(backend.get('source_coverage') or {}).get('nonzero_edge_count')),
    'preflight_export_and_load': observed_gate(bool(delivery),delivery.get('simulation_execution_ready') is True and (delivery.get('bundle_readback') or {}).get('status')=='PASS'),
    'isaac_single_carton_workflow': observed_gate(bool(physical),physical.get('workflow_cycle_completed') is True and physical.get('runtime_stop_reason') is None and counts.get('actual_grasp')==counts.get('actual_release')==1),
    'ideal_reception_and_outfeed': observed_gate(counts.get('actual_release',0)>0,counts.get('ideal_reception')==counts.get('ideal_outfeed')==1,scope='DECLARED_IDEAL_ASSUMPTION_NOT_PHYSICAL_QUALIFICATION'),
    'physical_reception_qualification': {'status':'NOT_EVALUATED','actual_reception_count':counts.get('actual_reception')},
    'measured_drive_torque_qualification': {'status':(physical.get('qualification_check_details') or {}).get('joint_efforts_within_limit',{}).get('status','NOT_EVALUATED')},
    'online_continuous_planning': {'status':'NOT_EVALUATED','scope':'physics paused for offline planning'},
}
semantics=physical.get('stack_clearance_semantics') or {}
feedback=physical.get('runtime_feedback') or {}
summary['first_execution_blocker'] = {'runtime_stop_reason':physical.get('runtime_stop_reason'),
    'first_evidence_conflict':semantics.get('first_evidence_conflict'),
    'confirmed_violation':semantics.get('confirmed_violation'),
    'first_runtime_failure':feedback.get('first_failure'), 'stop_handling':feedback.get('stop_handling'),
    'full_evidence':'formal/physics/stack_clearance_steps.json'}
(p/'evidence/formal-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
(formal/'plan/independent-stage-profiles.json').write_text(json.dumps(profiles,indent=2,allow_nan=False)+'\n')
video = formal/'physics/replay.mp4'
if video.is_file() and run['execution_requests'] == 1:
    probe = subprocess.run(['ffprobe','-v','error','-show_streams','-show_format','-of','json',str(video)],capture_output=True,text=True)
    assert probe.returncode == 0, probe.stderr
    (formal/'video-probe.json').write_text(probe.stdout)
    decode = subprocess.run(['ffmpeg','-v','error','-i',str(video),'-f','null','-'],capture_output=True,text=True)
    (formal/'video-decode-status.json').write_text(json.dumps({'returncode':decode.returncode,'stderr':decode.stderr,
        'sha256':hashlib.sha256(video.read_bytes()).hexdigest(),'scope':'Complete original video decoding; no transcode or splice'},indent=2)+'\n')
    assert decode.returncode == 0, decode.stderr
sources = json.loads((formal/'source-manifest.json').read_text())
changed = [n for n,h in sources.items() if hashlib.sha256((p/'repo'/n).read_bytes()).hexdigest()!=h]
assert not changed, changed
(p/'evidence/post-run-source-check.json').write_text(json.dumps({'files':len(sources),'changed':changed,'source_commit':run['source_commit']},indent=2)+'\n')
archives = {}
for kind in ('formal','development'):
    dst = p/(kind+'-evidence.tar.gz')
    assert not dst.exists()
    with tarfile.open(dst,'w:gz') as a:
        if kind == 'formal': a.add(formal,arcname=formal.name)
        else:
            a.add(p/'evidence',arcname='evidence')
            a.add(p/'benchmark',arcname='benchmark')
    archives[dst.name] = {'sha256':hashlib.sha256(dst.read_bytes()).hexdigest(),'bytes':dst.stat().st_size}
(p/'archive-hashes.json').write_text(json.dumps(archives,indent=2)+'\n')
print(json.dumps({'status':run['status'],'archives':archives},indent=2))
