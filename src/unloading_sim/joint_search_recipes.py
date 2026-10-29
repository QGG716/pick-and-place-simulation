"""Geometry-driven face/pose/reception candidates for the fixed40 contract."""
from dataclasses import replace
from itertools import product
import math
import numpy as np
from .joint_search_layout import SCHEMA
from .lifted_reception import StartHeight,receiving_region
from .moveit2_backend import digest
from .coupled_batch import CoupledCycleConnector
from .low_top_receiver import LowTopReceiverConnector
from .junction_reception import reception_classification
from .lifted_reception import translated
from .joint_search_outfeed import WITHDRAW_FIRST,PARK_FIRST,withdrawal_extra

RAISED_TOP_HEIGHTS_M=(.65,.85)

def pose_target(target,offset,roll):
    pose=target.contact_pose.copy();old=pose[:3,:3].copy()
    pose[:3,3]+=old[:,:2]@np.asarray(offset)
    c,s=math.cos(roll),math.sin(roll)
    pose[:3,:3]=old@np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    approach=pose.copy();approach[:3,3]+=target.standoff_m*target.outward_normal
    return replace(target,source=SCHEMA,contact_pose=pose,approach_pose=approach,in_plane_x=pose[:3,0]).validate()

def validate_receive_contract(contract,target,height):
    h=StartHeight.capture(target)
    if (contract['schema']!=SCHEMA or contract['target_identity']!=h.box_identity
            or contract['d_m']!=.05 or abs(contract['height_m']-height)>1e-9
            or not contract['minimum_m']-1e-9<=height<=contract['maximum_m']+1e-9):raise ValueError('SEARCH_RECEIVE_CONTRACT')
    mode=contract['mode']
    if mode=='Hmax':expected=h.hmax
    elif mode=='Hmin':expected=h.hmin
    elif mode=='LIFTED_TOP_FIXED':expected=contract['minimum_m']
    elif mode=='LIFTED_TOP_EXPLICIT':
        expected=contract.get('explicit_receive_height_m')
        if (expected not in RAISED_TOP_HEIGHTS_M or contract['face']!='z+'
                or not expected>h.hmax+1e-9):raise ValueError('SEARCH_EXPLICIT_TOP_HEIGHT_SCOPE')
    else:raise ValueError('SEARCH_RECEIVE_MODE')
    if abs(expected-height)>1e-9:raise ValueError('SEARCH_HEIGHT_FORMULA_CHANGED_OR_CLAMPED')
    if mode=='LIFTED_TOP_FIXED' and contract['face']!='z+':raise ValueError('LOW_BOX_REQUIRES_EXPLICIT_TOP_LIFT')
    if contract['release_bottom_m']!= (max(h.z_b+.05,height+.05) if contract['face']=='z+' else h.z_b):
        raise ValueError('SEARCH_RELEASE_HEIGHT_CHANGED')

def candidate_options(faces,scene,frozen,root_height,layout,*,initial_q):
    limits=layout.data['conveyors']['coupling'];minimum=limits['minimum_surface_height_world_m'];maximum=limits['maximum_surface_height_world_m']
    variants=[([0.,0.],0.),*([ [0.,0.],a] for a in (math.pi/2,-math.pi/2,math.pi)),
        *([list(offset),a] for offset,a in product(((.048,0),(-.048,0),(0,.096),(0,-.096)),(0.,math.pi/2)))]
    for pose_index,(offset,roll) in enumerate(variants):
        for face_index,face in enumerate(faces):
            h=StartHeight.capture(face.box)
            modes=[('Hmin',h.hmin),('Hmax',h.hmax)] if face.face_role!='TOP' else [('Hmax',h.hmax),('Hmin',h.hmin)]
            if face.face_role=='TOP' and h.hmax<minimum:modes=[('LIFTED_TOP_FIXED',minimum)]
            if face.face_role=='TOP':
                modes.extend(('LIFTED_TOP_EXPLICIT',height) for height in RAISED_TOP_HEIGHTS_M if height>h.hmax+1e-9)
            for mode_index,(mode,height) in enumerate(modes):
                if not minimum-1e-9<=height<=maximum:continue
                try:target=pose_target(face,offset,roll)
                except ValueError:continue
                for sync_index,sync in enumerate(('PREGRASP','CONTACT','BEFORE_APPROACH')):
                    if target.face_role=='TOP' and sync=='CONTACT':continue
                    normal=target.outward_normal.tolist();directions=[normal,[-1.,0.,0.]] if target.face_role!='FRONT' else [[-1.,0.,0.]]
                    contract=dict(schema=SCHEMA,target_identity=h.box_identity,target_id=target.target_id,
                        d_m=.05,mode=mode,height_m=height,minimum_m=minimum,maximum_m=maximum,
                        face=target.geometric_face_id,offset_face_m=offset,roll_rad=roll,
                        contact_pose=target.contact_pose.tolist(),release_bottom_m=max(h.z_b+.05,height+.05) if target.face_role=='TOP' else h.z_b)
                    if mode=='LIFTED_TOP_EXPLICIT':contract['explicit_receive_height_m']=height
                    validate_receive_contract(contract,target.box,height)
                    region=receiving_region(target.box,scene.receiver)
                    limits_x=region['projection_center_limits_before_occupancy'][:2]
                    projected=translated(target.box,[scene.receiver.center[0]+sum(limits_x)/2-target.box.center[0],0,0])
                    longitudinal=next(b for b in scene.fixed_components if b.name=='conveyor_longitudinal')
                    option=dict(template='JOINT_SEARCH_TOP' if target.face_role=='TOP' else 'JOINT_SEARCH_SIDE' if target.face_role=='SIDE' else 'JOINT_SEARCH_FRONT',
                        target_id=target.target_id,face=dict(geometric_face_id=target.geometric_face_id,
                            working_axis_into_box=(-target.outward_normal).tolist(),contact_pose=target.contact_pose.tolist()),
                        height_m=height,segment_directions_world=directions,
                        synchronization='ALREADY_AT_RECEIVE' if sync=='BEFORE_APPROACH' else sync,
                        pre_approach_height_m=height if sync=='BEFORE_APPROACH' else min(root_height,height),
                        original_role=frozen.evidence()['roles'][target.target_id]['role'],
                        region=region,reception=reception_classification(projected,scene.receiver,longitudinal),requires_open_channel=target.face_role=='SIDE',
                        release_bottom_world_m=contract['release_bottom_m'],endpoint='REGION_NOT_FIXED_POINT',
                        third_sync_disabled=True,acceptance='NOT_PLANNED',joint_search_contract=contract,
                        visit_order=[pose_index,sync_index,mode_index,face_index])
                    option['candidate_key']=digest(option)
                    yield target,option
                    park=dict(option,outfeed_timing=PARK_FIRST,visit_order=[*option['visit_order'],.5],
                        park_q_rad=np.asarray(initial_q).tolist(),
                        park_goal_source='FRESH40_FROZEN_INITIAL_LEGAL_STATE')
                    park.pop('candidate_key');park['candidate_key']=digest(park)
                    yield target,park
                    # One additional finite process ordering, with the same
                    # business pose, receiving height and numerical budgets.
                    alternative=dict(option,outfeed_timing=WITHDRAW_FIRST,
                        visit_order=[*option['visit_order'],1])
                    alternative.pop('candidate_key')
                    alternative['candidate_key']=digest(alternative)
                    yield target,alternative
                    for distance_index,extra in enumerate((.10,.30,.50),2):
                        farther=dict(alternative,withdraw_extra_m=extra,visit_order=[*option['visit_order'],distance_index])
                        farther.pop('candidate_key');farther['candidate_key']=digest(farther)
                        yield target,farther

class SearchCycleConnector(CoupledCycleConnector):
    def validate_template(self,target):
        super().validate_template(target)
        withdrawal_extra(self.option)
        contract=self.option['joint_search_contract'];validate_receive_contract(contract,target.box,self.option['height_m'])
        if not np.allclose(target.contact_pose,contract['contact_pose'],atol=1e-12,rtol=0):raise ValueError('SEARCH_CONTACT_POSE_CHANGED')

class SearchTopConnector(LowTopReceiverConnector,SearchCycleConnector):
    def validate_template(self,target):
        SearchCycleConnector.validate_template(self,target)
        if target.face_role!='TOP' or self.option['segment_directions_world']!=[[0.,0.,1.],[-1.,0.,0.]]:
            raise ValueError('SEARCH_TOP_PROCESS_CHANGED')
        self._top_lift_in_progress=False;self._top_transfer=False
