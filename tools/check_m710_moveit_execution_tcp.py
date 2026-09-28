"""Validate the actual executable bundle; optional explicit legacy LIN migration."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
from time import perf_counter
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.m710_replay_contract import (
    verify_m710_replay_bundle, _verify_m710_replay_bundle_integrity,
    canonical_sha256 as digest, sha256_file, build_replay_input_binding,
    verify_m710_preflight_contract)


def derive_legacy_bundle(bundle, requests_path, source_path):
    """One-time provenance-bearing augmentation; never rewrite a historical file.

    Retains the original motion/authority evidence, refreshes execution-code
    identity, reexports and audits the final reference. Does not claim replanning
    or a new authority collision check.
    """
    from unloading_sim.m710_execution_tcp import make_lin_contract, context, require
    from unloading_sim.m710_execution import _execution_implementation_identity
    from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
    _verify_m710_replay_bundle_integrity(bundle)
    original=deepcopy(bundle['metadata']['m710_execution_preflight'])
    # This migration only updates the execution-reference plumbing. Do not
    # silently refresh unrelated collision, attachment, or monitoring code.
    allowed_updates={'src/unloading_sim/isaac_bridge.py','src/unloading_sim/m710_execution.py',
        'src/unloading_sim/m710_replay_contract.py','src/unloading_sim/moveit2_timing.py',
        'scripts/export_isaac_fanuc_replay.py'}
    for relative,old_hash in original['input_identity']['execution_implementation_source_sha256'].items():
        if relative not in allowed_updates:
            require(sha256_file(ROOT/relative)==old_hash,'LEGACY_UNRELATED_IMPLEMENTATION_CHANGED: '+relative)
    preflight=deepcopy(original);adapter=preflight['replay_adapter_inputs']
    segment=adapter['trajectory_segment'];native=segment['native_backend']
    requests={}
    for line in requests_path.read_text(encoding='utf-8').splitlines():
        request=json.loads(line)
        base={k:v for k,v in request.items() if k not in {'pipeline_id','planner_id'}}
        requests[digest(base)]=base
    policy,_=context(ROOT);robot=policy.layout_validation.layout.robot()
    for index,record in enumerate(native['stages']):
        if record['planner_id']!='LIN':continue
        require(not record.get('lin_contract'),'LEGACY_MIGRATION_REQUIRES_MISSING_CONTRACT')
        request=requests.get(record['request_fingerprint'])
        require(request is not None,'LEGACY_ORIGINAL_REQUEST_MISSING')
        require(request['identity']==native['startup']['identity'],'LEGACY_REQUEST_CONTEXT_MISMATCH')
        audit=record['lin_constraint_audit']
        record['lin_contract']=make_lin_contract(request,robot.fk(request['q_start']),
            position_tolerance=audit['position_tolerance_m'],orientation_tolerance=audit['orientation_tolerance_rad'],
            edge_resolution=request['clearance_policy']['edge_resolution_rad'],root=ROOT)
        record['stage_id']='lin-'+digest([record['request_fingerprint'],index])
    provenance=dict(schema='m710_legacy_lin_derivation_v1',source_bundle_sha256=sha256_file(source_path),
        source_bundle_payload_sha256=bundle['bundle_payload_sha256'],source_preflight_fingerprint=original['preflight_fingerprint'],
        source_requests_sha256=sha256_file(requests_path),contract_source='original_native_request_and_recorded_planner_tolerances',
        original_execution_sources=original['input_identity']['execution_implementation_source_sha256'],
        original_motion_evidence_preserved=True,new_planning=False,new_collision_authority_check=False,new_physical_execution=False)
    preflight['offline_lin_contract_derivation']=provenance
    preflight['input_identity']['execution_implementation_source_sha256']=_execution_implementation_identity(ROOT)
    fp=digest(preflight['input_identity']);preflight['execution_asset_fingerprint_sha256']=fp
    adapter['plan_common']['execution_asset_fingerprint_sha256']=fp
    adapter['input_binding']=build_replay_input_binding(plan_common=adapter['plan_common'],configuration=adapter['configuration'],
        scene_primitives=preflight['scene']['primitives'],trajectory_segment=segment,
        trajectory_segment_status=adapter['trajectory_segment_status'],input_identity=preflight['input_identity'],execution_asset_fingerprint_sha256=fp)
    preflight.pop('preflight_fingerprint',None);preflight['preflight_fingerprint']=digest(preflight)
    verify_m710_preflight_contract(preflight,require_ready=True)
    plan=deepcopy(adapter['plan_common']);plan['segments']=[segment]
    derived=build_fanuc_isaac_replay_bundle(plan,adapter['configuration'],preflight=preflight).to_dict()
    verify_m710_replay_bundle(derived,project_root=ROOT)
    return derived


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--legacy-requests',type=Path,help='one-time migration only; never needed for normal validation')
    p.add_argument('--derive-bundle',type=Path,help='new file; original bundle is never overwritten')
    args=p.parse_args();started=perf_counter();result=dict(status='FAIL',isaac='NOT_RUN')
    try:
        if bool(args.legacy_requests)!=bool(args.derive_bundle):
            raise ValueError('legacy migration requires both --legacy-requests and --derive-bundle')
        bundle=json.loads(args.bundle.read_text(encoding='utf-8'))
        if args.derive_bundle:
            if args.derive_bundle.exists() or args.derive_bundle.resolve()==args.bundle.resolve():
                raise ValueError('derived bundle must be a new file')
            bundle=derive_legacy_bundle(bundle,args.legacy_requests,args.bundle)
        result['entry_verification']=verify_m710_replay_bundle(bundle,project_root=ROOT)
        if args.derive_bundle:
            args.derive_bundle.parent.mkdir(parents=True,exist_ok=True)
            with args.derive_bundle.open('x',encoding='utf-8') as stream:
                json.dump(bundle,stream,indent=2,allow_nan=False)
        result.update(status='PASS',bundle_payload_sha256=bundle['bundle_payload_sha256'])
    except Exception as exc:
        result['reason']=str(exc)
    result['wall_s']=perf_counter()-started
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=True))
    return int(result['status']!='PASS')


if __name__=='__main__':raise SystemExit(main())
