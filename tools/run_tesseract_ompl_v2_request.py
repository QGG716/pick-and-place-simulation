"""Exactly one fixed engineering request, gated on the known-path audit."""
from __future__ import annotations
import argparse
import json
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
from run_tesseract_ompl_comparison import fixture_context, ROOT
from run_tesseract_ompl_audit import sha, write
from unloading_sim.planning_contract import FreeMotionRequest, PlanningBudget
from unloading_sim.tesseract_scene import export_scene
from unloading_sim.tesseract_ompl_backend import NativeWorker, TesseractOMPLBackend


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--worker',required=True)
    ap.add_argument('--audit',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    audit=json.loads(args.audit.read_text())
    assert audit['worker_sha256']==sha(args.worker)
    assert audit['native_path']['status']=='AUDIT_VALID' and audit['native_path']['audit_complete']
    proof=audit['authority_evidence_reuse']
    assert proof['identical_actual_q_grid'] and proof['authority_path_accepted'] and proof['same_q_mismatches']==0
    args.output.mkdir(parents=True,exist_ok=True)
    marker=args.output/'request.json'
    if marker.exists(): raise SystemExit('This fixed request was already started; refusing an automatic rerun')
    fixtures=ROOT/'tests/fixtures/tesseract_ompl'
    world,segment,c,attachment=fixture_context(fixtures/'historical_state.json',fixtures/'historical_segment.json')
    scene=export_scene(c,world.all_obstacles,stage='pregrasp')
    assert scene['fingerprint']==audit['scene_fingerprint']
    path=segment['path']
    request=FreeMotionRequest('empty_detour',world.snapshot['scene_fingerprint'],scene['fingerprint'],
        scene['model_fingerprint'],scene['tool_fingerprint'],scene['policy_fingerprint'],'pregrasp',
        tuple(scene['joint_names']),tuple(path[78]),tuple(path[208]),scene['constraints'],scene['frames'],
        scene['attachment'],71070,budget=PlanningBudget(max_state_checks=100000,max_attempts=1))
    manifest=dict(schema='fixed_engineering_request_v2',worker_sha256=sha(args.worker),
        audit_sha256=sha(args.audit),sources={str(p.relative_to(ROOT)):sha(p) for p in
        [ROOT/'native/tesseract_ompl/worker.cpp',Path(__file__),*sorted((ROOT/'src/unloading_sim').glob('*.py'))]},
        seed=71070,range_rad=.18,source_path_indices=[78,208],q_start=path[78],q_goal=path[208],
        scene_fingerprint=scene['fingerprint'],model_fingerprint=scene['model_fingerprint'],
        policy_fingerprint=scene['policy_fingerprint'],tool_fingerprint=scene['tool_fingerprint'],
        attachment=scene['attachment'],constraints=scene['constraints'],budget=asdict(request.budget),
        profile_enabled=True,first_exact_only=True,isaac_run=False,full_task_run=False)
    # Exclusive creation also prevents two concurrent invocations.
    with marker.open('x') as f: json.dump(manifest,f,indent=2)
    class RecordingWorker(NativeWorker):
        def call(self,message,cancelled=lambda:False):
            raw=super().call(message,cancelled)
            write(args.output/'native-result.json',raw)
            print(json.dumps(dict(event='native_finished',status=raw['status'],counters=raw.get('counters'))),flush=True)
            return raw
    worker=RecordingWorker(args.worker)
    backend=TesseractOMPLBackend(worker=worker);backend.profile=True
    def authority(points):
        print(json.dumps(dict(event='authority_begin',waypoints=len(points))),flush=True)
        began=perf_counter()
        failure=c._path_failure(points,world.all_obstacles,stage='pregrasp',diagnostic_origin='v2_fixed_request')
        write(args.output/'authority.json',dict(accepted=failure is None,failure=failure,wall_s=perf_counter()-began,
                                               statistics=dict(c._statistics)))
        print(json.dumps(dict(event='authority_finished',accepted=failure is None)),flush=True)
        return failure
    try:
        result=backend.plan(request,scene,authority)
        write(args.output/'result.json',result.to_mapping())
        print(json.dumps(dict(event='request_finished',status=result.status.value,deliverable=result.deliverable)),flush=True)
    finally:
        worker.log.flush();worker.log.seek(0)
        (args.output/'native.log').write_text(worker.log.read())
        worker.close()

if __name__=='__main__': main()
