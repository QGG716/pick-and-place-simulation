"""Offline final-reference gate regressions; never import or start real Isaac."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import pytest
from unloading_sim.m710_replay_contract import (
    verify_m710_replay_bundle, _verify_m710_replay_bundle_integrity,
    add_bundle_payload_sha256, canonical_sha256 as digest,
    build_replay_input_binding, build_m710_replay_contract)
from unloading_sim.m710_execution_tcp import bind_reference
from unloading_sim.moveit2_timing import native_timing_floor

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=ROOT/'docs/validation/evidence/m710_execution_gate_20260928/derived-bundle.json'


@pytest.fixture
def bundle():
    return json.loads(Path(os.environ.get('M710_GATE_TEST_BUNDLE',DEFAULT)).read_text(encoding='utf-8'))


def rebind(bundle, *, keep_report=False):
    """Reseal mutated test inputs so tests exercise semantics, not stale SHA only."""
    m=bundle['metadata'];p=m['m710_execution_preflight'];a=p['replay_adapter_inputs'];s=a['trajectory_segment']
    a['input_binding']=build_replay_input_binding(plan_common=a['plan_common'],configuration=a['configuration'],
        scene_primitives=p['scene']['primitives'],trajectory_segment=s,trajectory_segment_status=a['trajectory_segment_status'],
        input_identity=p['input_identity'],execution_asset_fingerprint_sha256=p['execution_asset_fingerprint_sha256'])
    p.pop('preflight_fingerprint',None);p['preflight_fingerprint']=digest(p)
    plan=deepcopy(a['plan_common']);plan['segments']=[s]
    m['m710_replay_contract']=build_m710_replay_contract(p,plan,a['configuration'],s)
    m['native_backend']=deepcopy(s.get('native_backend'))
    for binding in m.get('lin_reference_bindings',[]):
        for stage in (s.get('native_backend') or {}).get('stages',[]):
            if stage.get('stage_id')==binding['stage_id'] and stage.get('lin_contract'):
                binding['contract_sha256']=digest(stage['lin_contract'])
    if not keep_report:m.pop('final_reference_tcp_audit',None)
    return add_bundle_payload_sha256(bundle)


def record(bundle):
    return bundle['metadata']['m710_execution_preflight']['replay_adapter_inputs']['trajectory_segment']['native_backend']['stages'][0]


def test_saved_derived_bundle(bundle):
    result=verify_m710_replay_bundle(bundle,project_root=ROOT)['final_reference_tcp_audit']
    assert result['status']=='PASS'
    assert result['checked_object']=='FINAL_EXECUTOR_JOINT_REFERENCE'
    assert result['report_reuse'] is False


@pytest.mark.parametrize('mutation,reason',[
    ('missing','LIN_CONTRACT_MISSING'),('tcp','LIN_TCP_TRANSFORM_MISMATCH'),
    ('frame','LIN_REFERENCE_FRAME_MISMATCH'),('range','LIN_PLANNING_RANGE_MISMATCH'),
    ('no_range','LIN_PATH_RANGE_MISSING'),('mapping','LIN_BOUNDARY_NOT_RETAINED'),
    ('interior','LIN_TASK_TCP_CONSTRAINT')])
def test_semantic_rejections_after_outer_integrity(bundle,mutation,reason):
    r=record(bundle);m=bundle['metadata']
    if mutation=='missing':r.pop('lin_contract')
    elif mutation=='tcp':r['lin_contract']['flange_from_task_tcp'][0][3]+=.01
    elif mutation=='frame':r['lin_contract']['frame_id']='flange'
    elif mutation=='range':r['path_range'][0]-=1
    elif mutation=='no_range':r.pop('path_range')
    elif mutation=='mapping':
        mapping=m['joint_reference']['source_path_indices'];a=r['path_range'][0]
        mapping[:]=[a-1 if i==a else i for i in mapping]
    elif mutation=='interior':
        a,b=m['lin_reference_bindings'][0]['reference_range']
        m['joint_reference']['positions_rad'][(a+b)//2][0]+=.01
    bundle=rebind(bundle)
    assert _verify_m710_replay_bundle_integrity(bundle)['status']=='PASS'
    with pytest.raises(ValueError,match=reason):verify_m710_replay_bundle(bundle,project_root=ROOT)


@pytest.mark.parametrize('mutation',['contract','reference'])
def test_old_pass_cannot_be_reused(bundle,mutation):
    if mutation=='contract':record(bundle)['lin_contract']['origin'][0][3]+=1e-8
    else:bundle['metadata']['joint_reference']['timestamps_seconds'][-1]+=.001
    bundle=rebind(bundle,keep_report=True)
    assert _verify_m710_replay_bundle_integrity(bundle)['status']=='PASS'
    with pytest.raises(ValueError,match='STALE_FINAL_REFERENCE_TCP_AUDIT'):
        verify_m710_replay_bundle(bundle,project_root=ROOT)


def test_repeated_q_uses_indices_not_last_match():
    a=[0.]*6;b=[.1]*6;x=[.3]*6
    r=dict(planner_id='LIN',stage_id='first-visit',request_fingerprint='source',path_range=[1,2],
        points=[{'q':a},{'q':b}],lin_contract={'request_fingerprint':'source'})
    s=dict(path=[x,a,b,x,a,b],native_backend={'name':'moveit2','stages':[r]})
    refs={'source_path_indices':[0,1,2,3,4,5]}
    assert bind_reference(s,refs)[0]['reference_range']==[1,2]
    with pytest.raises(ValueError,match='LIN_BOUNDARY_NOT_RETAINED'):
        bind_reference(s,{'source_path_indices':[0,3,4,5]})
    with pytest.raises(ValueError,match='AMBIGUOUS_NATIVE_STAGE_PATH_RANGE'):
        native_timing_floor(s['path'],[{'points':[{'q':a,'t':0},{'q':b,'t':1}]}])


def test_core_contract_unaffected(bundle):
    # Existing valid geometric trajectory viewed without native metadata; no new
    # planning/collision claim. Exercises ordinary core artifact gate semantics.
    m=bundle['metadata'];s=m['m710_execution_preflight']['replay_adapter_inputs']['trajectory_segment']
    s.pop('native_backend');m.pop('lin_reference_bindings')
    core=rebind(bundle)
    assert verify_m710_replay_bundle(core,project_root=ROOT)['final_reference_tcp_audit']['status']=='NOT_APPLICABLE'


@pytest.mark.parametrize('mode',['normal','-O','env'])
@pytest.mark.parametrize('mutation',['valid','missing','interior','not_ready','stale'])
def test_actual_supervised_entry_before_physics(bundle,tmp_path,mode,mutation):
    # The REAL entry script runs with a substituted isaacsim module. Its
    # SimulationApp sentinel raises immediately; no real physics imports exist.
    m=bundle['metadata']
    if mutation=='missing':record(bundle).pop('lin_contract');bundle=rebind(bundle)
    elif mutation=='interior':
        a,b=m['lin_reference_bindings'][0]['reference_range'];m['joint_reference']['positions_rad'][(a+b)//2][0]+=.01
        bundle=rebind(bundle)
    elif mutation=='not_ready':m['simulation_execution_ready']=False;bundle=add_bundle_payload_sha256(bundle)
    elif mutation=='stale':
        m['joint_reference']['timestamps_seconds'][-1]+=.001;bundle=rebind(bundle,keep_report=True)
    candidate=tmp_path/'bundle.json';candidate.write_text(json.dumps(bundle),encoding='utf-8')
    marker=tmp_path/'physics-initialization-called'
    (tmp_path/'isaacsim.py').write_text('from pathlib import Path\ndef SimulationApp(*a,**k):\n Path('+repr(str(marker))+').write_text("sentinel only")\n raise RuntimeError("TEST_SENTINEL_NO_REAL_PHYSICS")\n')
    env=dict(os.environ,PYTHONPATH=str(tmp_path)+os.pathsep+str(ROOT/'src'))
    env.pop('PYTHONOPTIMIZE',None)
    if mode=='env':env['PYTHONOPTIMIZE']='1'
    args=[sys.executable]+(['-O'] if mode=='-O' else [])+[str(ROOT/'scripts/isaacsim_fanuc_replay.py'),
        '--project-root',str(ROOT),'--usd-directory',str(tmp_path/'unused-usd'),'--bundle',str(candidate),'--output',str(tmp_path/'output')]
    run=subprocess.run(args,env=env,capture_output=True,text=True,timeout=40)
    assert run.returncode!=0
    if mutation=='valid':
        assert marker.exists(),run.stderr
        assert 'TEST_SENTINEL_NO_REAL_PHYSICS' in run.stderr
    else:
        assert not marker.exists(),run.stderr
        assert not (tmp_path/'output/run_status.json').exists(),run.stderr
        assert 'TEST_SENTINEL_NO_REAL_PHYSICS' not in run.stderr
        reason={'missing':'LIN_CONTRACT_MISSING','interior':'LIN_TASK_TCP_CONSTRAINT','not_ready':'not simulation-execution ready','stale':'STALE_FINAL_REFERENCE_TCP_AUDIT'}[mutation]
        assert reason in run.stderr,run.stderr


def test_exporter_rejects_invalid_contract(bundle):
    from unloading_sim.isaac_bridge import build_fanuc_isaac_replay_bundle
    record(bundle)['lin_contract']['destination'][0][3]+=.03
    bundle=rebind(bundle)
    assert _verify_m710_replay_bundle_integrity(bundle)['status']=='PASS'
    p=bundle['metadata']['m710_execution_preflight'];a=p['replay_adapter_inputs']
    plan=deepcopy(a['plan_common']);plan['segments']=[a['trajectory_segment']]
    with pytest.raises(ValueError,match='LIN_TASK_TCP_CONSTRAINT'):
        build_fanuc_isaac_replay_bundle(plan,a['configuration'],preflight=p)


def test_standalone_gate_import_does_not_load_numerical_runtime():
    code="import importlib.util,sys; p=sys.argv[1]; s=importlib.util.spec_from_file_location('gate',p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); print('numpy' in sys.modules, 'isaacsim' in sys.modules)"
    run=subprocess.run([sys.executable,'-c',code,str(ROOT/'src/unloading_sim/m710_replay_contract.py')],capture_output=True,text=True,check=True)
    assert run.stdout.strip()=='False False'
