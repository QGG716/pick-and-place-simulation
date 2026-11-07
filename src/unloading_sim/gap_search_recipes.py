"""Explicit high FRONT drop and fair TOP/FRONT/SIDE carried-lift families."""
from copy import deepcopy
from dataclasses import replace
from time import perf_counter
import numpy as np
from .gap_search_layout import SCHEMA
from .lifted_reception import StartHeight,receiving_region,translated,bounds
from .junction_reception import reception_classification
from .moveit2_backend import digest,MoveItUnavailable
from .coupled_batch import CoupledCycleConnector
from .low_top_receiver import LowTopReceiverConnector
from .single_extraction import SingleExtractionConnector
from .single_lifted_receiver import SingleLiftedReceiverConnector
from .stage_motion_policy import MotionPurpose
from .joint_search_outfeed import WAIT_FIRST,WITHDRAW_FIRST,PARK_FIRST,withdrawal_extra

def validate_receive_contract(contract,target,height):
    h=StartHeight.capture(target);c=contract;upper=h.z_b>=1.5-1e-9
    if (c['schema']!=SCHEMA or c['target_identity']!=h.box_identity or c['target_id']!=target.name
        or c['d_m']!=.05 or c['minimum_m']!=.55 or c['maximum_m']!=1.5
        or not .55-1e-9<=height<=1.5+1e-9 or abs(c['height_m']-height)>1e-9
        or abs(c['original_bottom_m']-h.z_b)>1e-9):raise ValueError('GAP_RECEIVE_CONTRACT')
    if upper:
        if c['face']!='x-' or c['mode']!='FRONT_DIRECT_HIGH_DROP':raise ValueError('UPPER_ORIGINAL_LAYER_FRONT_ONLY')
        release=h.z_b
    else:
        mode='TOP_LIFT_TRANSFER' if c['face']=='z+' else 'EXTRACT_RAISE_TRANSFER'
        if c['mode']!=mode:raise ValueError('LOWER_GAP_PROCESS_MODE')
        release=max(h.z_b+.05,height+.05) if c['face']=='z+' else max(h.z_b,height+.05)
    if abs(c['release_bottom_m']-release)>1e-9 or release-height<.05-1e-9:
        raise ValueError('GAP_RELEASE_HEIGHT_OR_CLEARANCE')

def candidate_options(faces,scene,frozen,root_height,layout,*,initial_q):
    if not .55<=root_height<=1.5:raise ValueError('GAP_INITIAL_HEIGHT_RANGE')
    # One upstream business pose per exposed face. Numerical IK/RRT recovery
    # stays inside the native task budget and is not counted as a new pose.
    groups={role:[] for role in ('TOP','FRONT','SIDE')}
    for face in faces:
        h=StartHeight.capture(face.box)
        original=frozen.evidence()['heights'][face.target_id]
        if h.box_identity!=original['start_box_identity']:raise ValueError('GAP_ORIGINAL_TARGET_CHANGED')
        upper=original['z_b_m']>=1.5-1e-9
        if upper and face.face_role!='FRONT':continue
        heights=sorted(set([min(1.5,h.z_b-.05),1.25,1.,.75,.55]),reverse=True) if upper else [.55,.75,1.,1.25,1.5]
        # Stable partition, then finite successive rounds: TOP -> FRONT -> SIDE.
        for hi,height in enumerate(heights):
            release=h.z_b if upper else max(h.z_b+(.05 if face.face_role=='TOP' else 0.),height+.05)
            if release-height<.05-1e-9:continue
            mode='FRONT_DIRECT_HIGH_DROP' if upper else 'TOP_LIFT_TRANSFER' if face.face_role=='TOP' else 'EXTRACT_RAISE_TRANSFER'
            target=replace(face,source=SCHEMA)
            contract=dict(schema=SCHEMA,target_identity=h.box_identity,target_id=target.target_id,
                d_m=.05,mode=mode,height_m=height,minimum_m=.55,maximum_m=1.5,
                original_bottom_m=h.z_b,face=target.geometric_face_id,contact_pose=target.contact_pose.tolist(),
                release_bottom_m=release,expected_drop_m=release-height)
            validate_receive_contract(contract,target.box,height)
            region=receiving_region(target.box,scene.receiver)
            projected=translated(target.box,[scene.receiver.center[0]-target.box.center[0],0,0])
            long=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal')
            for si,sync in enumerate(('PREGRASP','BEFORE_APPROACH','CONTACT')):
                if face.face_role=='TOP' and sync=='CONTACT':continue
                for ti,timing in enumerate((WAIT_FIRST,WITHDRAW_FIRST,PARK_FIRST)):
                    c=deepcopy(contract)
                    directions=[target.outward_normal.tolist()]
                    if face.face_role!='FRONT' or release>h.z_b+1e-9:directions+=[[0.,0.,1.],[-1.,0.,0.]]
                    if face.face_role=='TOP':directions=[[0.,0.,1.],[-1.,0.,0.]]
                    option=dict(template=mode,target_id=target.target_id,
                        face=dict(geometric_face_id=target.geometric_face_id,working_axis_into_box=(-target.outward_normal).tolist(),contact_pose=target.contact_pose.tolist()),
                        height_m=height,release_bottom_world_m=release,segment_directions_world=directions,
                        synchronization='ALREADY_AT_RECEIVE' if sync=='BEFORE_APPROACH' else sync,
                        pre_approach_height_m=height if sync=='BEFORE_APPROACH' else min(root_height,height),
                        original_role=frozen.evidence()['roles'][target.target_id]['role'],region=region,
                        reception=reception_classification(projected,scene.receiver,long),requires_open_channel=target.face_role=='SIDE',
                        endpoint='REGION_NOT_FIXED_POINT',third_sync_disabled=True,acceptance='NOT_PLANNED',
                        joint_search_contract=c,outfeed_timing=timing,finite_outfeed_support=True,
                        visit_order=[hi,si,ti])
                    if timing==PARK_FIRST:option.update(park_q_rad=np.asarray(initial_q).tolist(),park_goal_source='FRESH40_FROZEN_INITIAL_LEGAL_STATE')
                    option['candidate_key']=digest(option)
                    groups[target.face_role].append((target,option))
    for i in range(max(map(len,groups.values()),default=0)):
        for role in ('TOP','FRONT','SIDE'):
            if i<len(groups[role]):yield groups[role][i]

class GapCycleConnector(CoupledCycleConnector):
    @property
    def planar(self):return False

    def release_bottom_world_m(self,target):return self.option['release_bottom_world_m']

    def _plan_extraction_suffix(self,plan,row,*args):
        # Share the unchanged task/prefix budget across legal IK configurations.
        # A failing LIN branch must not spend that entire budget bisecting the
        # same small release interval before the existing alternate-IK recovery
        # can run. This is a numerical visit slice, not a geometric rejection.
        previous=getattr(self,'_single_connection_deadline',None)
        stage_s,_=self._native_limits();began=perf_counter();slice_s=min(12.,stage_s)
        self._gap_full_suffix_deadline=min(previous,began+stage_s) if previous is not None else began+stage_s
        self._gap_numeric_deadline=min(self._gap_full_suffix_deadline,began+slice_s)
        self._single_connection_deadline=self._gap_numeric_deadline
        try:
            result,detail=super()._plan_extraction_suffix(plan,row,*args)
            detail['regional_numeric_visit']=dict(slice_s=slice_s,elapsed_s=perf_counter()-began,
                shared_task_deadline_unchanged=True,shared_prefix_deadline_unchanged=True,
                remaining_region_and_ik_search='NOT_EXHAUSTED_BY_VISIT_TIMEOUT',
                native_attempts=len(detail.get('region_attempts',[])))
            if result is None and (detail.get('failure') or {}).get('termination')=='STAGE_BUDGET_EXHAUSTED':
                detail['failure']['termination']='LOCAL_NUMERICAL_VISIT_DEADLINE_NOT_UNREACHABLE'
            return result,detail
        finally:
            self._single_connection_deadline=previous
            del self._gap_full_suffix_deadline,self._gap_numeric_deadline

    def region_motion(self,*args):
        if hasattr(self,'_gap_numeric_deadline'):
            self._single_connection_deadline=self._gap_numeric_deadline
        result=super().region_motion(*args)
        if result[1] is None and hasattr(self,'_gap_full_suffix_deadline'):
            # Once native LIN has connected, allow its checks, release, finite
            # outfeed and departure to finish under the ORIGINAL absolute
            # suffix deadline. The numerical visit slice is not a deadline for
            # accepting an otherwise qualified complete cycle.
            self._single_connection_deadline=self._gap_full_suffix_deadline
        return result

    def validate_template(self,target):
        super().validate_template(target)
        withdrawal_extra(self.option)
        validate_receive_contract(self.option['joint_search_contract'],target.box,self.option['height_m'])
        if not np.allclose(target.contact_pose,self.option['joint_search_contract']['contact_pose'],atol=1e-12,rtol=0):
            raise ValueError('GAP_CONTACT_POSE_CHANGED')

class GapTopConnector(LowTopReceiverConnector,GapCycleConnector):
    def validate_template(self,target):
        GapCycleConnector.validate_template(self,target)
        if target.face_role!='TOP':raise ValueError('GAP_TOP_FACE_CHANGED')
        self._top_lift_in_progress=False;self._top_transfer=False

class GapRaiseConnector(LowTopReceiverConnector,GapCycleConnector):
    """Native normal separation, then native +Z, then native regional -X.

    No joint teleport; every extra leg has its actual predecessor receipt and
    remains under the same task deadline. Restored free-space rules follow
    full separation. A blocked lift is a retained failure, never a bypass.
    """
    def validate_template(self,target):
        GapCycleConnector.validate_template(self,target)
        if target.face_role not in ('FRONT','SIDE'):raise ValueError('GAP_RAISE_FACE_CHANGED')
        self._top_lift_in_progress=False;self._top_transfer=False

    def _plan_extraction_suffix(self,plan,row,start,target,obstacles,attachment,payload_world,supports,tracker,seed):
        self._top_transfer=False;self._top_lift_in_progress=False
        # The existing geometric normal extraction computes separation against
        # the actual remaining world, retaining its finite resource bounds.
        path,geometry,failure=self.plan_extraction(start,target,attachment,payload_world,supports,tracker,seed,row)
        if failure:return self._reject_suffix(row,failure)
        self._top_lift_in_progress=True
        try:
            separated,row=SingleExtractionConnector._complete_extraction_candidate(self,plan,row,start,path,geometry,
                target,obstacles,attachment,payload_world,supports,tracker,seed)
        finally:self._top_lift_in_progress=False
        if separated is None:return None,row
        if not tracker.fully_released:raise MoveItUnavailable('GAP_NORMAL_EXTRACTION_NOT_FULLY_SEPARATED')
        separated['normal_separation']=dict(geometry=deepcopy(geometry),receipt=deepcopy(path[-1].native_receipt),
            sample_check=deepcopy(row['extraction_samples']))
        row['normal_separation']=deepcopy(separated['normal_separation'])
        end=path[-1]
        if self.release_bottom_world_m(target)>bounds(attachment.box_at(end))[0][2]+1e-9:
            return LowTopReceiverConnector._plan_extraction_suffix(self,separated,row,end,target,obstacles,attachment,payload_world,(),tracker,seed)
        # A middle-layer fallback may already be above the receiving surface.
        self._top_transfer=True;self._top_lift_receipt=deepcopy(end.native_receipt)
        row['top_lift']=dict(required=False,reason='ALREADY_ABOVE_RECEIVING_SURFACE')
        return SingleLiftedReceiverConnector._plan_extraction_suffix(self,separated,row,end,target,obstacles,attachment,payload_world,(),tracker,seed)
