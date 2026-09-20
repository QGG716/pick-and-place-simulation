"""Real video tick with explicit CPU source/worker substitutes; no SAM invocation."""
from concurrent.futures import Future
from types import SimpleNamespace

from unloading_ros_bridge.video_demo_node import VideoDemoNode


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
