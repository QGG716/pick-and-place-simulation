"""Real video tick with explicit CPU source/worker substitutes; no SAM invocation."""
from concurrent.futures import Future
from types import SimpleNamespace
from pathlib import Path
import sys
import time
import pytest
import rclpy

from unloading_ros_bridge.video_demo_node import VideoDemoNode

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tests'))


@pytest.fixture
def wired_node(tmp_path,monkeypatch):
    """Real ROS Node initialization, synthetic capture/model and external worker only."""
    import unloading_ros_bridge.video_demo_node as module
    from unloading_perception.finite_sequence import atomic_json
    from unloading_perception.isaac_validation import sha256_file
    from workcell_once_fakes import capture_fixture
    from PIL import Image
    frames=[]
    for i in range(2):
        root=tmp_path/str(i);root.mkdir()
        capture,models,modules=capture_fixture(root,('normal','normal'),frame=7+i,stamp=1.+i)
        for name in modules:
            Image.new('RGB',(3,2)).save(capture/'FULL_STACK_NOMINAL/modules'/name/'preview_rgb.jpg')
        frames.append(dict(path=str(capture.relative_to(tmp_path)),frame_sequence=7+i,source_time=1.+i,
                           manifest_sha256=sha256_file(capture/'manifest.json')))
    record=tmp_path/'index.json'
    atomic_json(record,dict(schema_version='continuous_rgbd_recording_v1',sensor_epoch='synthetic-joint-capture',
                           frames=frames,simulation_real_time_factor=1.))
    output=tmp_path/'output';output.mkdir();(output/'receipts').mkdir()
    atomic_json(output/'progress.json',dict(batch_id='CPU_TEST',groups=[{},{}],target_task=None,delivery=None))
    monkeypatch.setattr(module,'verify_sam_files',lambda _:None)
    calls=[]
    class Worker:
        def poll(self):return None
        def terminate(self):calls.append('terminate')
        def wait(self,**kwargs):calls.append('wait');return 0
    def launch(command,**kwargs):calls.append(command);return Worker()
    monkeypatch.setattr(module.subprocess,'Popen',launch)
    params=dict(recording=record,output=output,project=ROOT,models=models,vision=tmp_path,
                algorithm_python='/TEST_GPU_VENV/bin/python',geometry_backend='subprocess',geometry_blas_threads=1)
    args=['--ros-args']
    for key,value in params.items():args+=['-p',key+':='+str(value)]
    rclpy.init(args=args,domain_id=141)
    node=VideoDemoNode()
    yield node,calls
    node.destroy_node();rclpy.shutdown()
    assert calls[-2:]==['terminate','wait'] and (output/'stop-worker').exists()


def test_real_node_passes_geometry_options_to_worker_and_expected_config(wired_node):
    from geometry_runtime_config import effective_config
    from unloading_perception.finite_sequence import read_json
    import yaml
    node,calls=wired_node
    command=calls[0]
    assert command[0]=='/TEST_GPU_VENV/bin/python'
    assert command[command.index('--geometry-python')+1]=='/TEST_GPU_VENV/bin/python'
    assert command[command.index('--geometry-backend')+1]=='subprocess'
    assert command[command.index('--geometry-blas-threads')+1]=='1'
    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text())
    assert node.config==effective_config(config,'subprocess',1)==read_json(node.output/'expected-config.json')


def test_real_tick_clears_previous_markers_before_new_group_delivery(wired_node,monkeypatch):
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import Marker
    node,_=wired_node
    (node.output/'START').touch()
    now=time.monotonic();node.source_started=now;node.source_finished=now;node.index=len(node.frames)
    node.active={**node.frames[1], 'task_id':'video_01','algorithm_output':str(node.output/'video_01/algorithm'),
                 'submitted_monotonic':now,'preview_monotonic':now}
    node.done=1;node.display_keys={('previous',7),('previous',8)}
    node.future=Future();node.future_kind='accept'
    node.future.set_result(({'path':'CPU_TEST','sha256':'a'*64},{},PointCloud2(),TFMessage()))
    messages=[]
    node.faces_pub=SimpleNamespace(publish=messages.append)
    node.cloud_pub=SimpleNamespace(publish=lambda _:None)
    node.draw=lambda _:None
    node.tick()
    assert node.switching and node.cloud_frame is None
    assert {(m.ns,m.id,m.action) for m in messages[0].markers}=={('previous',7,Marker.DELETE),('previous',8,Marker.DELETE)}
    assert node.state['groups'][1]['status']=='ARTIFACT_READY'
    assert node.state['delivery']['task_id']=='video_01' and node.done==1


def test_source_failure_does_not_abandon_or_replace_running_algorithm(tmp_path):
    (tmp_path/'START').touch()
    original = dict(frame_sequence=20, source_time=1., task_id='video_00')
    from unloading_perception.video_demo import LatestFrameSlot
    slot = LatestFrameSlot()
    slot.offer(dict(frame_sequence=21, source_time=1.1))
    future = Future()  # Explicit in-flight external worker substitute, never completed here.
    calls = []
    node = SimpleNamespace(output=tmp_path, source_started=1., source_finished=None,
        frames=[dict(frame_sequence=22, source_time=1.2, capture=str(tmp_path/'missing-preview'))],
        index=0, slot=slot, future=future, active=original, last=None, done=0, limit=2,
        mode='PROCESSING', error='', source_error=False, current_frame=original,
        depth_pubs={k:SimpleNamespace(get_subscription_count=lambda:0) for k in ('upper','lower')},
        worker=SimpleNamespace(poll=lambda:None), draw=lambda _:calls.append('draw'),
        fail=lambda _:calls.append('unexpected algorithm failure'))
    VideoDemoNode.tick(node)
    assert node.active is original and node.future is future
    assert node.current_frame is original
    assert node.source_error and node.source_finished is not None
    assert 'input_preview: FileNotFoundError' in node.error
    assert node.slot.pending is None and node.done == 0
    assert calls == ['draw']


def test_structured_algorithm_failure_preserves_previous_display_and_no_retry(tmp_path):
    from unloading_perception.finite_sequence import atomic_json
    from unloading_perception.video_demo import LatestFrameSlot
    (tmp_path/'START').touch()
    atomic_json(tmp_path/'worker-status.json',dict(task_id='video_01', status='FAILED', summary={
        'errors':[dict(stage='runtime_initialize',error_type='RuntimeError',error='CPU_TEST_SHARED_FAILURE')]}))
    active=dict(frame_sequence=60, source_time=6., task_id='video_01')
    previous=dict(frame_sequence=20, source_time=2., task_id='video_00')
    node=SimpleNamespace(output=tmp_path, source_started=1., source_finished=2., frames=[], index=0,
        slot=LatestFrameSlot(), future=None, active=active, last=previous, done=1, limit=2,
        mode='PROCESSING', error='', results=[], state={'groups':[{},dict(status='ALGORITHM_RUNNING',stage='algorithm')]},
        depth_pubs={k:SimpleNamespace(get_subscription_count=lambda:0) for k in ('upper','lower')},
        worker=SimpleNamespace(poll=lambda:None), draw=lambda _:None, save=lambda:None)
    node.fail=lambda error:VideoDemoNode.fail(node,error)
    VideoDemoNode.tick(node)
    assert node.last is previous and node.active is None and node.done == 2
    assert node.state['groups'][1]['status'] == 'FAILED'
    assert node.state['groups'][1]['failure_stage'] == 'algorithm'
    assert 'runtime_initialize: RuntimeError: CPU_TEST_SHARED_FAILURE' in node.error
    VideoDemoNode.tick(node)
    assert node.state['exit_code'] == 1 and node.active is None and node.done == 2
