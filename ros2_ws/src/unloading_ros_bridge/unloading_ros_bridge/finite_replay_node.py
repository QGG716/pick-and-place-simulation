"""Finite completed-artifact handoff; models never execute in this ROS process."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import time
from urllib.parse import unquote

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from unloading_interfaces.msg import PerceptionObservation, PlanningWorldSnapshot, ExecutionAuthorization
from unloading_contracts import canonical_fingerprint, to_wire
from unloading_perception.algorithm_artifact import algorithm_replay_record
from unloading_perception.finite_sequence import atomic_json, read_json
from unloading_perception.replay import replay_publication
from .common import require_humble_python310, time_to_float
from .mapping import observation_to_msg, snapshot_from_msg
from .marker_display import marker_qos
from .replay_cache import PUBLICATION


def inspect_delivery(record, message, markers, previous_keys):
    """Full production mapping/identity and geometry verification outside callbacks."""
    world = snapshot_from_msg(message)
    scene = world.scene_snapshot
    coverage = scene['perception_source']['coverage']
    def static(value):
        return {**{k:v for k,v in value.items() if k not in ('replay','source_restart')},
                'replay':{k:v for k,v in value['replay'].items() if k not in PUBLICATION}}
    if (message.source_epoch != record.source_epoch
            or canonical_fingerprint(static(coverage))!=canonical_fingerprint(static(record.coverage))
            or message.planning_admissible or scene['planning_admissible']
            or 'HISTORICAL_REPLAY_DISPLAY_ONLY' not in message.blocking_reasons):
        raise ValueError('world did not accept the selected historical artifact')
    originals = {c.source_instance_id:c for c in record.cargo}
    if {c['source_instance_id'] for c in scene['obstacles']}!=set(originals):
        raise ValueError('world contains missing or foreign objects')
    for c in scene['obstacles']:
        original = originals[c['source_instance_id']]
        for field in ('raw_result','observed_surfaces'):
            if canonical_fingerprint(c[field])!=canonical_fingerprint(getattr(original,field)):
                raise ValueError('world contains mixed artifact evidence')
    if not {r.region_id for r in record.unknown_regions}.issubset(r['region_id'] for r in scene['unknown_regions']):
        raise ValueError('world lost unknown regions')
    current,deleted,patches = set(),set(),0
    surfaces = {(s['module_id'],s['capture_id'],s['source_instance_id'],s['face_id']):s
                for c in record.cargo for s in c.observed_surfaces}
    for marker in markers.markers:
        key = (marker.ns,marker.id)
        if marker.action==Marker.DELETE:
            deleted.add(key)
            continue
        if marker.action!=Marker.ADD: raise ValueError('unexpected marker lifecycle operation')
        identity=json.loads(unquote(marker.ns.split('/evidence/',1)[1]))
        if identity==['ui']: continue
        if identity[0]!=record.source_epoch: raise ValueError('old group remains displayed as current')
        current.add(key)
        if marker.type==Marker.LINE_STRIP:
            source=surfaces[tuple(identity[-4:])]
            expected=[tuple(p) for p in source['corners_3d_m']]
            if [(p.x,p.y,p.z) for p in marker.points]!=expected+[expected[0]]:
                raise ValueError('marker coordinates differ from current algorithm evidence')
            patches+=1
    if not (previous_keys-current).issubset(deleted):
        raise ValueError('previous marker geometry was not deleted')
    expected=sum(len(c.raw_result.get('fusion_diagnostics',{}).get('face_reduction',{}).get('representatives',())) for c in record.cargo)
    if patches!=expected: raise ValueError('marker representative count differs from artifact')
    return {'objects':len(originals),'raw_surfaces':len(surfaces),'marker_representatives':patches,
        'unknown_regions':len(scene['unknown_regions']),'previous_marker_deletes':len(previous_keys-current),
        'world_publisher_epoch':message.publisher_epoch,
        'mock_state_epoch':world.tool_attachment['source_epoch']},current


class FiniteReplayNode(Node):
    def __init__(self):
        super().__init__('unloading_finite_replay')
        self.declare_parameter('progress','')
        self.declare_parameter('receipts','')
        self.progress=Path(str(self.get_parameter('progress').value))
        self.receipts=Path(str(self.get_parameter('receipts').value))
        initial=read_json(self.progress)
        if initial['schema_version']!='finite_capture_progress_v1' or not 1<=len(initial['groups'])<=16:
            raise ValueError('invalid finite progress record')
        self.batch_id=initial['batch_id']
        self.task_ids=[r['task_id'] for r in initial['groups']]
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='artifact-file-validation')
        self.future=None
        self.current=self.request=None
        self.seen=set()
        self.last_order=-1
        self.authorizations=0
        self.sequence=0
        self.first_world=None
        self.latest_world=self.latest_markers=None
        self.current_keys=set()
        self.publisher_epoch=self.mock_epoch=None
        self.rows=[]
        self.preceding=deque(maxlen=10)
        self.receipt_written=False
        self.ownership_seen=False
        reliable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        self.publisher=self.create_publisher(PerceptionObservation,'/unloading/perception',reliable)
        self.status_publisher=self.create_publisher(String,'/unloading/replay_progress',marker_qos())
        self.create_subscription(PlanningWorldSnapshot,'/unloading/world_snapshot',self.on_world,reliable)
        self.create_subscription(MarkerArray,'/unloading/markers',self.on_markers,marker_qos())
        self.create_subscription(ExecutionAuthorization,'/unloading/execution_authorization',self.on_authorization,reliable)
        self.timer=self.create_timer(.1,self.poll)

    def on_world(self,message):
        now=time.monotonic()
        row={'monotonic':now,'ros_time':self.get_clock().now().nanoseconds/1e9,
            'source_epoch':message.source_epoch,'publisher_epoch':message.publisher_epoch,
            'publisher_sequence':message.publisher_sequence,'evaluated':time_to_float(message.published_time),
            'robot_sample':time_to_float(message.robot_sample_time),
            'mechanism_sample':time_to_float(message.mechanism_sample_time),
            'reasons':list(message.blocking_reasons),'planning_admissible':message.planning_admissible}
        self.preceding.append(row)
        if self.request is not None and not self.receipt_written:
            if len(self.rows)>=512:
                self.reject('bounded switch event capacity exceeded')
                return
            self.rows.append(row)
        if self.current is not None and message.source_epoch==self.current.source_epoch:
            self.latest_world=message
            self.ownership_seen=True
            if self.first_world is None: self.first_world=now

    def on_markers(self,message):
        self.latest_markers=message

    def on_authorization(self,message):
        self.authorizations+=1

    def reject(self,error):
        if self.request is not None and not self.receipt_written:
            atomic_json(self.receipts/(self.request['task_id']+'.json'),
                        {**self.request,'status':'FAILED','error':str(error),'world_events':self.rows})
            self.receipt_written=True
        self.get_logger().error(str(error))

    def poll(self):
        try:
            progress=read_json(self.progress)
            if progress['batch_id']!=self.batch_id: raise ValueError('batch identity changed')
            target=next((r for r in progress['groups'] if r['task_id']==progress['target_task']),None)
            display=f"INDEPENDENT CAPTURE GROUPS / READ ONLY\ntarget={progress['target_task']} status={target['status'] if target else 'PENDING'}\nlast confirmed ROS group={progress['displayed_task']}"
            if target and target.get('error'): display+='\nTARGET FAILED: '+target['error'][:400]
            self.status_publisher.publish(String(data=display))
            request=progress['delivery']
            if request is not None and request['task_id'] not in self.seen and self.future is None:
                if (request['batch_id']!=self.batch_id or request['task_id'] not in self.task_ids
                        or type(request['order']) is not int or not self.last_order<request['order']<len(self.task_ids)
                        or self.task_ids[request['order']]!=request['task_id']):
                    raise ValueError('delivery does not belong to finite manifest')
                self.request=request
                self.seen.add(request['task_id'])
                self.last_order=request['order']
                self.rows=list(self.preceding)
                self.receipt_written=False
                self.future=self.pool.submit(algorithm_replay_record,request['artifact']['path'],
                                             request['artifact']['sha256'],request['session'])
                self.future_kind='load'
            if self.future is not None and self.future.done():
                future,self.future=self.future,None
                if self.future_kind=='load':
                    self.current=future.result()
                    self.sequence=0
                    self.started=time.monotonic()
                    self.first_world=None
                    self.latest_world=self.latest_markers=None
                    self.ownership_seen=False
                else:
                    details,keys=future.result()
                    if self.publisher_epoch not in (None,details['world_publisher_epoch']) or self.mock_epoch not in (None,details['mock_state_epoch']):
                        raise ValueError('world/mock process identity changed during finite sequence')
                    self.publisher_epoch=details['world_publisher_epoch']
                    self.mock_epoch=details['mock_state_epoch']
                    steady=[r for r in self.rows if r['source_epoch']==self.current.source_epoch and r['monotonic']>=self.first_world+5.]
                    if len(steady)<3 or any('STATE_' in reason for r in steady for reason in r['reasons']):
                        raise ValueError('current group steady window has state freshness faults')
                    if any(r['planning_admissible'] or 'HISTORICAL_REPLAY_DISPLAY_ONLY' not in r['reasons'] for r in self.rows):
                        raise ValueError('historical read-only boundary lost')
                    if (self.authorizations or self.count_publishers('/unloading/execution_authorization')
                            or self.count_publishers('/unloading/perception')!=1):
                        raise ValueError('unexpected authorization or perception publisher')
                    receipt={**self.request,**details,'status':'ROS_ACCEPTED',
                        'perception_pid':os.getpid(),'execution_authorizations':self.authorizations,
                        'first_world_monotonic':self.first_world,'accepted_monotonic':time.monotonic(),
                        'switch_latency_seconds':self.first_world-self.request['submitted_monotonic'],
                        'fixed_initialization_seconds':5.,'steady_seconds':5.,'world_events':self.rows}
                    atomic_json(self.receipts/(self.request['task_id']+'.world.json'),to_wire(snapshot_from_msg(self.latest_world)))
                    # Raw ROS Marker wire fields preserve ADD/DELETE provenance for audit.
                    from rosidl_runtime_py.convert import message_to_ordereddict
                    atomic_json(self.receipts/(self.request['task_id']+'.markers.json'),message_to_ordereddict(self.latest_markers))
                    atomic_json(self.receipts/(self.request['task_id']+'.json'),receipt)
                    self.current_keys=keys
                    self.receipt_written=True
            if self.current is not None:
                publication=replay_publication(self.current,sequence=self.sequence,
                    elapsed=time.monotonic()-self.started,published_time=self.get_clock().now().nanoseconds/1e9,
                    clock_domain='ros')
                self.publisher.publish(observation_to_msg(publication))
                if self.ownership_seen: self.sequence+=1
            if (self.current is not None and not self.receipt_written and self.first_world is not None
                    and time.monotonic()>=self.first_world+10. and self.latest_markers is not None and self.future is None):
                self.future=self.pool.submit(inspect_delivery,publication,self.latest_world,self.latest_markers,self.current_keys)
                self.future_kind='inspect'
        except Exception as exc:
            self.reject(f'{type(exc).__name__}: {exc}')

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    require_humble_python310()
    rclpy.init(args=args)
    node=FiniteReplayNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
