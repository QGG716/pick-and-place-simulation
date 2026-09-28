"""CPU boundary tests. Model/geometry doubles are NOT real SAM/solver validation."""
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import workcell_geometry_process as process
from unloading_contracts import canonical_fingerprint, dumps
from unloading_perception.algorithm_artifact import empty_module_observation
from unloading_perception.isaac_payload import load_capture_payload, CapturePayloadError
from test_offline_payload_binding import algorithm_fixture, geometry
from test_metric_thread_policy import state


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def empty_result(tmp_path):
    manifest, _, _ = algorithm_fixture(tmp_path/'input')
    payload = load_capture_payload(tmp_path/'input', manifest, with_instance_masks=True)
    root = tmp_path/'output'
    task = root/'geometry-task'
    config = process.effective_config({}, 'subprocess', 1)
    request = dict(task_id='new-task', run_id='new-run', capture=process.capture_identity(payload),
        config=config, output_directory=str(root), task_directory=str(task), mask_ids=[])
    path = task/'request.json'; write(path, request)
    ref = process.reference(path)
    observation = replace(empty_module_observation(payload), config_identity=canonical_fingerprint(config))
    report = dict(status='COMPLETED', pid=os.getpid(), task_id=request['task_id'],
        policy=config['metric_runtime_policy'], policy_identity=canonical_fingerprint(config['metric_runtime_policy']),
        before_policy=state(), before_geometry=state(1), after_geometry=state(1), applied=['CPU_TEST_SETTER'])
    files = {}
    for key, filename, value in [('thread_report',task/'thread_report.json',report),
            ('face_sets',task/'face_sets.json',[]), ('observation',task/'observation.json',json.loads(dumps(observation))),
            ('geometry',root/'rgbd_cuboids.json',{'instances':[]})]:
        write(filename, value); files[key] = process.reference(filename)
    index = dict(status='COMPLETED', pid=os.getpid(), task_id=request['task_id'], run_id=request['run_id'],
                 capture=request['capture'], request=ref, files=files, result_metadata={})
    write(task/'completed.json', index)
    return request, ref, index


def test_legal_empty_domain_result_is_not_technical_failure(tmp_path):
    request, ref, _ = empty_result(tmp_path)
    result = process.accept_result(request, ref, os.getpid())
    assert result['observed_face_sets'] == ()
    assert result['observation'].unknown_regions[0].reason == 'EMPTY_SEGMENTATION_UNKNOWN_VOLUME'


@pytest.mark.parametrize('fault', ['old_task', 'wrong_module', 'wrong_capture', 'pid', 'request',
    'status', 'missing_marker', 'hash', 'missing_file', 'escape', 'missing_faces', 'thread_pid',
    'thread_count', 'thread_policy', 'module_observation', 'config'])
def test_acceptance_rejects_incomplete_foreign_or_corrupt_results(tmp_path, fault):
    request, ref, index = empty_result(tmp_path)
    task = Path(request['task_directory'])
    if fault == 'old_task': index['task_id'] = 'old'
    elif fault == 'wrong_module': index['capture'] = {**index['capture'], 'module_id':'wrong'}
    elif fault == 'wrong_capture': index['capture'] = {**index['capture'], 'capture_id':'wrong'}
    elif fault == 'pid': index['pid'] += 1
    elif fault == 'request': index['request'] = {**ref,'sha256':'0'*64}
    elif fault == 'status': index['status'] = 'RUNNING'
    elif fault == 'hash': index['files']['geometry']['sha256'] = '0'*64
    elif fault == 'missing_file': Path(index['files']['geometry']['path']).unlink()
    elif fault == 'escape': index['files']['geometry']['path'] = str(tmp_path/'foreign.json')
    elif fault == 'missing_faces': del index['files']['face_sets']
    elif fault.startswith('thread_'):
        path = task/'thread_report.json'; report = process.read(path)
        if fault == 'thread_pid': report['after_geometry']['pid'] += 1
        elif fault == 'thread_count': report['after_geometry']['blas'][1]['num_threads'] = 64
        else: report['policy_identity'] = 'old'
        write(path, report); index['files']['thread_report'] = process.reference(path)
    elif fault in ('module_observation', 'config'):
        path = task/'observation.json'; doc = process.read(path)
        if fault == 'config': doc['payload']['config_identity'] = 'old'
        else: doc['payload']['coverage']['module_binding']['module_id'] = 'wrong'
        write(path, doc); index['files']['observation'] = process.reference(path)
    write(task/'completed.json', index)
    if fault == 'missing_marker': (task/'completed.json').unlink()
    with pytest.raises((ValueError, KeyError, FileNotFoundError)):
        process.accept_result(request, ref, os.getpid())


def test_real_exec_has_distinct_pid_and_scoped_environment(tmp_path, monkeypatch):
    monkeypatch.setenv('PYTHONPATH', 'CPU_TEST_ROS_PATH')
    monkeypatch.setenv('PYTHONHOME', 'CPU_TEST_ROS_HOME')
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '7')
    before = dict(os.environ)
    code = "import os,json; print(json.dumps(dict(pid=os.getpid(),home=os.environ.get('PYTHONHOME'),path=os.environ.get('PYTHONPATH'),blas=os.environ.get('OPENBLAS_NUM_THREADS'))))"
    # Windows venv redirector is itself another process; use the base executable
    # for this stdlib-only OS process test. Real numeric acceptance is Linux.
    pid, timing = process.run_owned_child([sys._base_executable, '-c', code], tmp_path, 10)
    actual = process.read(tmp_path/'stdout.log')
    assert actual['pid'] == pid != os.getpid()
    assert actual['home'] is None and 'ROS' not in actual['path'] and actual['blas'] == '7'
    assert dict(os.environ) == before and timing['launch'] >= 0


@pytest.mark.skipif(sys.platform != 'linux', reason='native OpenBLAS getter evidence uses Linux /proc')
def test_owned_numeric_child_changes_only_its_own_pools(tmp_path):
    pytest.importorskip('scipy'); pytest.importorskip('cv2')
    import metric_thread_policy
    before=metric_thread_policy.snapshot()
    code = "import json,os; from metric_thread_policy import snapshot,configure_blas; b=snapshot(); configure_blas(b['blas'],1); print(json.dumps(snapshot()))"
    pid,_=process.run_owned_child([sys.executable,'-c',code],tmp_path,30)
    child=process.read(tmp_path/'stdout.log')
    assert child['pid']==pid!=os.getpid()
    metric_thread_policy.verify_blas(child['blas'],1)
    after=metric_thread_policy.snapshot()
    assert before['blas']==after['blas']
    metric_thread_policy.verify_non_targets(before,after)


@pytest.mark.parametrize('fault', ['timeout', 'stop', 'interrupt', 'sigterm', 'crash', 'launch'])
def test_owned_child_failure_cleanup_and_stderr(tmp_path, monkeypatch, fault):
    children = []
    original = process.subprocess.Popen
    def popen(*args, **kwargs):
        child = original(*args, **kwargs); children.append(child); return child
    monkeypatch.setattr(process.subprocess, 'Popen', popen)
    def stop():
        if fault == 'interrupt': raise KeyboardInterrupt()
        if fault == 'sigterm': signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
        return fault == 'stop'
    command = [sys.executable, '-c', 'import sys,time; print("child failure evidence",file=sys.stderr,flush=True); time.sleep(30)']
    if fault == 'crash': command = [sys.executable, '-c', 'import sys; print("crash",file=sys.stderr); sys.exit(5)']
    if fault == 'launch': command = [str(tmp_path/'no-interpreter')]
    handler = signal.getsignal(signal.SIGTERM)
    with pytest.raises((TimeoutError, InterruptedError, KeyboardInterrupt, SystemExit, RuntimeError, FileNotFoundError)):
        process.run_owned_child(command, tmp_path, .2 if fault == 'timeout' else 10, stop)
    assert all(child.poll() is not None for child in children)
    assert signal.getsignal(signal.SIGTERM) == handler
    assert (tmp_path/'stderr.log').exists()
    if fault == 'crash': assert 'crash' in (tmp_path/'stderr.log').read_text()


@pytest.mark.parametrize('args', [['--geometry-blas-threads','1'],
    ['--geometry-backend','subprocess','--geometry-blas-threads','0'],
    ['--geometry-backend','subprocess','--geometry-timeout','-1']])
def test_invalid_options_rejected_by_real_once_before_capture(tmp_path, args):
    import run_workcell_perception_once as once
    with pytest.raises(SystemExit) as exc:
        once.main(['--capture','missing','--vision','missing','--models','missing',
                   '--output-directory',str(tmp_path/'unused')] + args)
    assert exc.value.code == 2 and not (tmp_path/'unused').exists()


def install_dispatch_double(monkeypatch, calls):
    """Only the geometry exec boundary is replaced; real once orchestration runs."""
    def dispatch(**kwargs):
        assert kwargs['backend'] == 'subprocess' and kwargs['blas_threads'] == 1
        assert kwargs['geometry_python'] == Path(sys.executable)
        assert kwargs['timeout'] == 23 and kwargs['run_id']
        calls.append(kwargs['payload'].metadata.capture_id)
        from run_isaac_rgbd_geometry import _run_secondary_module
        arguments = {k:v for k,v in kwargs.items() if k not in
            ('backend','blas_threads','geometry_python','run_id','stop_requested')}
        result = _run_secondary_module(**arguments)
        result['observation'] = replace(result['observation'], config_identity=canonical_fingerprint(kwargs['config']))
        result.update(geometry_process_timing_seconds={}, geometry_process={'pid':10000+len(calls)})
        return result
    monkeypatch.setattr(process, 'dispatch_geometry', dispatch)


def test_real_video_loop_forwards_subprocess_and_reuses_one_model_for_two_jobs(tmp_path, monkeypatch):
    from workcell_once_fakes import capture_fixture, install
    import workcell_video_worker as video
    from unloading_perception.finite_sequence import atomic_json
    captures = []
    for i in range(2):
        root = tmp_path/str(i); root.mkdir()
        captures.append(capture_fixture(root, ('normal','normal'), frame=7+i, stamp=1.+i))
    calls = []
    output = tmp_path/'worker'; output.mkdir()
    def request(i):
        capture, models, _ = captures[i]
        return dict(task_id=str(i),frame_sequence=7+i,source_time=1.+i,capture=str(capture),
                    algorithm_output=str(capture/'perception-once'))
    atomic_json(output/'worker-request.json', request(0))
    original = video.run_capture
    caches = []
    def run(argv, **kwargs):
        i = len(caches); caches.append(kwargs['runtime_factory'])
        with monkeypatch.context() as scope:
            install(scope, captures[i][0])
            install_dispatch_double(scope, calls)
            code = original(argv, **kwargs)
        if i == 0: atomic_json(output/'worker-request.json', request(1))
        else: (output/'stop-worker').touch()
        return code
    monkeypatch.setattr(video, 'run_capture', run)
    assert video.main(['--output',str(output),'--models',str(captures[0][1]),'--vision',str(tmp_path),
        '--geometry-backend','subprocess','--geometry-blas-threads','1','--geometry-timeout','23']) == 0
    assert len(calls) == len(set(calls)) == 4
    assert caches[0] is caches[1] and caches[0].loads == 1
    status = process.read(output/'worker-status.json')
    assert status['status'] == 'COMPLETED' and status['resident_model_loads'] == 1


@pytest.mark.parametrize('failure', [RuntimeError('GEOMETRY_CHILD_EXIT'), TimeoutError('GEOMETRY_PROCESS_TIMEOUT'),
                                      FileNotFoundError('completed.json')])
def test_subprocess_failure_blocks_real_group_publication(tmp_path, monkeypatch, failure):
    from test_workcell_perception_once import prepare, report
    capture, _, calls, main, argv = prepare(tmp_path, monkeypatch)
    attempts = []
    def dispatch(**kwargs): attempts.append(kwargs['module_dir']); raise failure
    monkeypatch.setattr(process, 'dispatch_geometry', dispatch)
    assert main(argv+['--geometry-backend','subprocess']) == 1
    assert len(attempts) == 2
    assert not (capture/'perception-once/algorithm_artifact.json').exists()
    assert report(capture)['module_counts']['failed'] == 2


def test_inline_default_keeps_original_configuration():
    config = {'vision': {'legacy_cuboid_diagnostic':False}}
    assert process.effective_config(config, 'inline', None) is config


def prepared_request(tmp_path, monkeypatch):
    import numpy as np
    from unloading_perception.isaac_payload import write_capture_snapshot
    manifest, _, _ = algorithm_fixture(tmp_path/'source')
    payload = write_capture_snapshot(load_capture_payload(tmp_path/'source', manifest, with_instance_masks=True),tmp_path/'module')
    folder = payload.directory
    write(folder/'oracle_proposals.json',{'instances':[]})
    metrics=tmp_path/'metrics.json'; write(metrics,{})
    write(folder/'mode_b1_worker_response.json',{'metrics_reference':process.reference(metrics)})
    mask=tmp_path/'masks.npz'; np.savez(mask,mask_ids=[1])
    artifacts={'cargo_masks.npz':process.reference(mask)}
    command=[]
    def launch(args,*a,**k): command.extend(args); raise RuntimeError('CPU_TEST_LAUNCH_BOUNDARY')
    monkeypatch.setattr(process,'run_owned_child',launch)
    interpreter=tmp_path/'venv/bin/python'
    original=Path.resolve
    def resolve(path,*a,**k):
        if path==interpreter: return tmp_path/'base/python'
        return original(path,*a,**k)
    monkeypatch.setattr(Path,'resolve',resolve)
    with pytest.raises(RuntimeError,match='CPU_TEST_LAUNCH_BOUNDARY'):
        process.dispatch_geometry(backend='subprocess',blas_threads=1,geometry_python=interpreter,
            run_id='new-run',scene='TEST',module_dir=folder,manifest=manifest,payload=payload,
            artifacts=artifacts,config=process.effective_config({},'subprocess',1),vision_root=tmp_path,
            upstream_python=Path(sys.executable),timeout=23)
    assert command[0]==str(interpreter.absolute())
    request_path=folder/'geometry-task/request.json'
    return request_path,process.read(request_path)


def test_actual_dispatch_request_binds_all_inputs_and_preserves_interpreter(tmp_path,monkeypatch):
    path,request=prepared_request(tmp_path,monkeypatch)
    assert request['capture']['K']==[2.,0.,1.,0.,2.,.5,0.,0.,1.]
    assert request['capture']['source_sequence']==7 and request['mask_ids']==[1]
    assert request['run_id']=='new-run' and request['config']['metric_runtime_policy']['requested_blas_threads']==1
    assert {'metric_depth_m.npy','sensor_rgb.png','camera_info.json','capture_binding.json',
            'capture_metadata.json','sam/cargo_masks.npz','oracle_proposals.json','sam_metrics'} <= request['inputs'].keys()
    assert all(process.checked(ref) for ref in request['inputs'].values())


@pytest.mark.parametrize('fault',['hash','depth','K','capture','module','policy'])
def test_child_revalidates_consumed_input_before_geometry(tmp_path,monkeypatch,geometry,fault):
    path,request=prepared_request(tmp_path,monkeypatch)
    import metric_thread_policy
    monkeypatch.setattr(metric_thread_policy,'snapshot',lambda:pytest.fail('must fail before thread initialization'))
    if fault=='hash':request['inputs']['metric_depth_m.npy']['sha256']='0'*64
    elif fault in ('depth','K'):
        ref=request['inputs']['metric_depth_m.npy' if fault=='depth' else 'camera_info.json']
        Path(ref['path']).write_bytes(b'corrupt')
    elif fault in ('capture','module'):request['capture'][fault+'_id']='wrong'
    else:request['config_identity']='wrong'
    write(path,request)
    with pytest.raises((ValueError, CapturePayloadError)):
        process.child_main(['--request',str(path),'--request-sha256',process.reference(path)['sha256']])
    assert not (path.parent/'completed.json').exists()
    assert process.read(path.parent/'child_failure.json')['stage']=='input_validation'


def test_child_new_snapshot_path_calls_existing_geometry_once(tmp_path,monkeypatch,geometry):
    """In-process child protocol test, explicitly fake SAM/BLAS/geometry boundaries."""
    path,request=prepared_request(tmp_path,monkeypatch)
    import metric_thread_policy
    monkeypatch.setattr(geometry,'_worker_artifacts',lambda *a,**k:request['artifacts'])
    calls=[]
    def compute(**kwargs):
        calls.append(kwargs)
        assert kwargs['payload'].directory==Path(request['input_directory'])
        assert kwargs['payload'].source_directory==tmp_path/'source'
        assert kwargs['config']==request['config'] and kwargs['output_directory'] is None
        raise RuntimeError('CPU_TEST_REACHED_EXISTING_GEOMETRY')
    monkeypatch.setattr(geometry,'_run_secondary_module',compute)
    snapshots=iter((state(),state(1)))
    monkeypatch.setattr(metric_thread_policy,'snapshot',lambda:next(snapshots))
    setters=[]
    def configure(rows,n):setters.append(n);return ['CPU_TEST_SETTER']
    monkeypatch.setattr(metric_thread_policy,'configure_blas',configure)
    with pytest.raises(RuntimeError,match='CPU_TEST_REACHED_EXISTING_GEOMETRY'):
        process.child_main(['--request',str(path),'--request-sha256',process.reference(path)['sha256']])
    assert len(calls)==1 and setters==[1]
    assert process.read(path.parent/'child_failure.json')['stage']=='geometry'


def test_subprocess_option_keeps_legal_empty_segmentation_without_child(tmp_path,monkeypatch):
    from test_workcell_perception_once import prepare,report
    capture,_,_,main,argv=prepare(tmp_path,monkeypatch,('empty','empty'))
    monkeypatch.setattr(process,'dispatch_geometry',lambda **k:pytest.fail('empty SAM needs no geometry child'))
    assert main(argv+['--geometry-backend','subprocess','--geometry-blas-threads','1'])==0
    summary=report(capture)
    assert all(row['metric_attempts']==0 and row['metric_not_run_reason']=='EMPTY_SEGMENTATION'
               for row in summary['runs'])


def process_comparison_fixture(tmp_path):
    from test_metric_faces_ab_comparison import fixture
    from test_fusion_face_reduction import batch_fixture
    from unloading_contracts import loads, to_wire
    from unloading_perception.algorithm_artifact import publish_algorithm_run
    from metric_thread_policy import policy
    plan, a, b = fixture(tmp_path, (False,False), [policy(1),policy(1)])
    batches = batch_fixture(False)
    old = process.read(a/'ab_summary.json')
    write(a/'thread_policy_report.json', dict(status='COMPLETED',policy=policy(1),
        policy_identity=canonical_fingerprint(policy(1)),plan_sha256=process.reference(plan)['sha256'],
        pid=os.getpid(),before_policy=state(),before_geometry=state(1),after_geometry=state(1),
        applied=[],artifact=old['artifact']))
    write(a/'function_timings.json', {'production_sha256':{'geometry':'unchanged'}})
    summary = process.read(b/'ab_summary.json')
    config = process.effective_config(summary['effective_config'],'subprocess',1)
    results = {}
    for i,batch in enumerate(batches):
        module = batch.module_id
        task = b/('.geometry-task-'+module)
        obs = replace(loads((b/module/'mode_b_rgbd_observation.json').read_text()),
                      config_identity=canonical_fingerprint(config))
        capture = {**to_wire(obs.coverage['module_binding']), 'source_epoch':obs.source_epoch,
                   'source_sequence':obs.source_sequence,'capture_time':obs.capture_time,'K':[]}
        request = dict(task_id='task-'+module,run_id=b.name,capture=capture,config=config,
                       output_directory=str(b/module),task_directory=str(task),
                       mask_ids=[c.raw_result['instance_lineage']['mask_id'] for c in obs.cargo])
        write(task/'request.json',request)
        ref = process.reference(task/'request.json')
        pid=20000+i
        states = [{**state(n),'pid':pid} for n in (64,1,1)]
        thread = dict(status='COMPLETED',pid=pid,task_id=request['task_id'],policy=policy(1),
            policy_identity=canonical_fingerprint(policy(1)),before_policy=states[0],
            before_geometry=states[1],after_geometry=states[2],applied=[])
        files = {}
        for key,path,value in [('thread_report',task/'thread_report.json',thread),
                ('observation',task/'observation.json',json.loads(dumps(obs))),
                ('face_sets',task/'face_sets.json',[s.to_dict() for s in batch.face_sets])]:
            write(path,value); files[key]=process.reference(path)
        files['geometry']=process.reference(b/module/'rgbd_cuboids.json')
        write(task/'completed.json',dict(status='COMPLETED',pid=pid,task_id=request['task_id'],
            run_id=b.name,capture=capture,request=ref,files=files,result_metadata={}))
        write(task/'parent_acceptance.json',dict(status='ACCEPTED',pid=pid,request=ref))
        results[module]=dict(observation=obs,observed_face_sets=batch.face_sets)
    artifact=publish_algorithm_run(b,results,expected_modules=list(results),run_id=b.name,
        models=process.read(plan)['model_manifest'],config=config,write_json=write)
    summary.update(artifact=artifact,effective_config=config,production_sha256={'geometry':'unchanged'})
    write(b/'ab_summary.json',summary)
    return plan,a,b


def test_process_comparator_exact_fields_and_declared_runtime_only(tmp_path):
    from tools.compare_metric_faces_ab import compare
    plan,a,b=process_comparison_fixture(tmp_path)
    assert compare(a,b,plan,mode='geometry-subprocess')['results_equal']


@pytest.mark.parametrize('fault',['coordinate','cost','missing_instance','hash','thread','config','production'])
def test_process_comparator_rejects_changes_without_ulp_exceptions(tmp_path,fault):
    from tools.compare_metric_faces_ab import main
    plan,a,b=process_comparison_fixture(tmp_path)
    if fault in ('coordinate','cost','missing_instance'):
        path=b/'m0/rgbd_cuboids.json';doc=process.read(path)
        if fault=='coordinate':doc['instances'][0]['camera_facing_faces'][0]['corners_3d_m'][0][0]+=1e-12
        elif fault=='cost':doc['instances'][0]['metric_solver']={'cost':1.0000000000000002}
        else:doc['instances'].pop()
        write(path,doc)
        # Keep transport hash valid: semantic comparison must still fail.
        path=b/'.geometry-task-m0/completed.json';index=process.read(path)
        index['files']['geometry']=process.reference(b/'m0/rgbd_cuboids.json');write(path,index)
    elif fault=='hash':(b/'fusion_result.json').write_text('{}')
    elif fault=='thread':
        path=b/'.geometry-task-m0/thread_report.json';doc=process.read(path)
        doc['after_geometry']['blas'][0]['num_threads']=64;write(path,doc)
    elif fault=='production':
        path=b/'ab_summary.json';doc=process.read(path);doc['production_sha256']['geometry']='changed';write(path,doc)
    else:
        path=b/'.geometry-task-m0/request.json';doc=process.read(path);doc['config']['extra']='unapproved';write(path,doc)
    assert main(['--before',str(a),'--after',str(b),'--plan',str(plan),
                 '--output',str(tmp_path/'comparison.json'),'--mode','geometry-subprocess'])==1
