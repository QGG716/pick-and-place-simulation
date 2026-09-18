"""Synthetic artifacts, one real persistent graph/DDS; no model or admission substitutes."""
import json
import os
from pathlib import Path
import sys
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from unloading_interfaces.msg import PerceptionObservation, PlanningWorldSnapshot
from visualization_msgs.msg import Marker, MarkerArray
from unloading_perception.algorithm_artifact import algorithm_replay_record
from unloading_perception.finite_sequence import atomic_json,read_json
from unloading_perception.replay import replay_publication
from unloading_ros_bridge.finite_replay_node import FiniteReplayNode
from unloading_ros_bridge.world_bridge_node import WorldBridgeNode
from unloading_ros_bridge.mock_state_publisher import MockStatePublisher
from unloading_ros_bridge.mapping import observation_to_msg,snapshot_from_msg
from unloading_ros_bridge.marker_display import marker_qos

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'tests'))
from test_algorithm_artifact import artifact_fixture


def test_success_corrupt_empty_late_session_and_source_loss_in_one_graph(tmp_path):
    refs={'A':artifact_fixture(tmp_path/'A'),'C':artifact_fixture(tmp_path/'C',empty=True)}
    refs['B']={**refs['A'],'sha256':'0'*64}
    progress={'schema_version':'finite_capture_progress_v1','batch_id':'SYNTHETIC_FINITE_DDS',
        'groups':[{'task_id':k,'status':'PENDING'} for k in ('A','B','C')],
        'target_task':None,'displayed_task':None,'delivery':None}
    path=tmp_path/'progress.json'
    atomic_json(path,progress)
    receipts=tmp_path/'receipts';receipts.mkdir()
    rclpy.init(args=['--ros-args','-p','progress:='+str(path),'-p','receipts:='+str(receipts),
                    '-p','observation_mode:=replay_display_only'],domain_id=100+os.getpid()%30)
    world,source,publisher=WorldBridgeNode(),MockStatePublisher(),FiniteReplayNode()
    observer=rclpy.create_node('synthetic_finite_observer')
    executor=SingleThreadedExecutor()
    for node in (world,source,publisher,observer):executor.add_node(node)
    markers=[];worlds=[]
    observer.create_subscription(MarkerArray,'/unloading/markers',markers.append,marker_qos())
    observer.create_subscription(PlanningWorldSnapshot,'/unloading/world_snapshot',worlds.append,10)
    def wait(predicate,timeout=30.):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.01)
            if predicate():return
        raise AssertionError('finite DDS condition timed out')
    def deliver(name,index):
        progress['target_task']=name
        progress['groups'][index]['status']='ARTIFACT_READY'
        request=dict(batch_id=progress['batch_id'],task_id=name,order=index,session=uuid.uuid4().hex,
                     artifact=refs[name],submitted_monotonic=time.monotonic())
        progress['delivery']=request;atomic_json(path,progress)
        wait(lambda:(receipts/(name+'.json')).exists())
        return request,read_json(receipts/(name+'.json'))
    try:
        wait(lambda:source.joints.get_subscription_count()>0 and world.publisher.get_subscription_count()>=2)
        request_a,a=deliver('A',0)
        assert a['status']=='ROS_ACCEPTED' and a['marker_representatives']>0
        progress['displayed_task']='A'
        _,b=deliver('B',1)
        assert b['status']=='FAILED' and 'HASH_MISMATCH' in b['error']
        assert world.last_observation.source_epoch==request_a['session']
        progress['groups'][1].update(status='FAILED',error='SYNTHETIC_BAD_HASH')
        atomic_json(path,progress)
        wait(lambda:markers and any('TARGET FAILED: SYNTHETIC_BAD_HASH' in m.text for m in markers[-1].markers))
        assert any(m.action==Marker.ADD and m.type==Marker.LINE_STRIP for m in markers[-1].markers)
        request_c,c=deliver('C',2)
        assert c['status']=='ROS_ACCEPTED' and c['objects']==c['marker_representatives']==0
        assert c['unknown_regions']>0 and c['previous_marker_deletes']>0
        assert a['world_publisher_epoch']==c['world_publisher_epoch']==world.publisher_epoch
        assert a['mock_state_epoch']==c['mock_state_epoch']==source.epoch
        assert a['perception_pid']==c['perception_pid']==os.getpid()
        assert a['execution_authorizations']==c['execution_authorizations']==0
        assert world.last_observation.source_epoch==request_c['session']
        assert len(world.heartbeat_templates)<=8
        assert all(t.snapshot.scene_snapshot['perception_source']['source_epoch']==request_c['session'] for _,t in world.heartbeat_templates)
        late=[]
        observer.create_subscription(MarkerArray,'/unloading/markers',late.append,marker_qos())
        wait(lambda:late)
        assert not any(m.action==Marker.ADD and m.type==Marker.LINE_STRIP for m in late[-1].markers)
        assert any(m.action==Marker.DELETE for m in late[-1].markers)
        # Actual late retired-session DDS packet; never reset a source guard.
        rogue=observer.create_publisher(PerceptionObservation,'/unloading/perception',10)
        wait(lambda:rogue.get_subscription_count()>0)
        old=algorithm_replay_record(refs['A']['path'],refs['A']['sha256'],request_a['session'])
        rogue.publish(observation_to_msg(replay_publication(old,sequence=0,elapsed=0.,published_time=100.,clock_domain='ros')))
        count=len(worlds)
        wait(lambda:len(worlds)>count+3)
        assert all(w.source_epoch==request_c['session'] for w in worlds[count:])
        assert world.replay_guard.current_epoch==request_c['session']
        observer.destroy_publisher(rogue)
        source.timer.cancel()
        wait(lambda:worlds and 'MECHANISM_STATE_STALE_OR_TIME_JUMP' in worlds[-1].blocking_reasons)
        frozen=worlds[-1]
        count=len(worlds);wait(lambda:len(worlds)>count+3)
        assert all(w.robot_sample_time==frozen.robot_sample_time and w.mechanism_sample_time==frozen.mechanism_sample_time for w in worlds[count:])
        source.timer.reset()
        wait(lambda:not any('STATE_' in s for s in worlds[-1].blocking_reasons))
        assert 'HISTORICAL_REPLAY_DISPLAY_ONLY' in worlds[-1].blocking_reasons
        assert not worlds[-1].planning_admissible and not snapshot_from_msg(worlds[-1]).scene_snapshot['obstacles']
    finally:
        executor.shutdown()
        for node in (publisher,world,source,observer):node.destroy_node()
        rclpy.shutdown()
