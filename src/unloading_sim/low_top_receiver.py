"""Explicit low-layer top suction with a stationary minimum-height receiver.

One unchanged face pose; native +Z separation, native -X regional transfer,
same-pose release and the existing nominal reception/drain suffix. No physics.
"""
from copy import deepcopy
from time import perf_counter
import numpy as np

from .coupled_batch import CoupledCycleConnector
from .single_extraction import SingleExtractionConnector
from .single_lifted_receiver import SingleLiftedReceiverConnector
from .lifted_reception import StartHeight, directional_contact, receiving_region, translated, bounds
from .junction_reception import reception_classification
from .moveit2_backend import box_message, digest, MoveItUnavailable
from .stage_motion_policy import MotionPurpose
from .l01_selected_routes import TOP_TEMPLATE as SELECTED_TOP_TEMPLATE
from .l01_hmax import TOP_TEMPLATE as L01_HMAX_TOP_TEMPLATE


TEMPLATE = 'TOP_FIXED_MINIMUM_RECEIVER'
LAYER_TEMPLATE = 'TOP_FIXED_CURRENT_RECEIVER'
TOP_TEMPLATES = (TEMPLATE, LAYER_TEMPLATE, SELECTED_TOP_TEMPLATE, L01_HMAX_TOP_TEMPLATE)
LEGACY_LOW_DEFINITION = 'FROZEN_ORIGINAL_HMIN_BELOW_MINIMUM'
BOX_TOP_LOW_DEFINITION = 'FROZEN_BOX_TOP_STRICTLY_BELOW_MINIMUM'


def uses_low_top(height, minimum, definition=LEGACY_LOW_DEFINITION):
    """Explicit policy version; historical Hmin experiments remain reproducible."""
    if definition == LEGACY_LOW_DEFINITION:
        return height.hmin < minimum-1e-9
    if definition == BOX_TOP_LOW_DEFINITION:
        return height.z_b+height.h < minimum
    raise ValueError('UNKNOWN_LOW_TOP_DEFINITION')


def low_top_option(target, belt, longitudinal, frozen, minimum, *, definition=LEGACY_LOW_DEFINITION):
    capture = frozen.evidence()['heights'][target.name]
    height = StartHeight.capture(target)
    if height.box_identity != capture['start_box_identity']:
        raise ValueError('LOW_TOP_FROZEN_TARGET_CHANGED')
    if not uses_low_top(height, minimum, definition):
        return None
    return _fixed_top_option(target,belt,longitudinal,frozen,minimum,height,definition)


def selected_layer_top_permission(frozen, anchor_id):
    """Bind the user-selected layer to original geometry, never parse carton IDs."""
    evidence=frozen.evidence()
    anchor=evidence['heights'][anchor_id]
    ceiling=anchor['z_b_m']+anchor['h_m']
    return dict(anchor_id=anchor_id,anchor_box_identity=anchor['start_box_identity'],
        ceiling_world_m=ceiling,original_cartons_sha256=evidence['original_cartons_sha256'],
        allowed_target_ids=sorted(n for n,h in evidence['heights'].items()
            if h['z_b_m']+h['h_m']<=ceiling+1e-9),
        definition='FROZEN_BOX_TOP_AT_OR_BELOW_SELECTED_LAYER')


def layer_top_option(target,belt,longitudinal,frozen,current_height,permission):
    if permission!=selected_layer_top_permission(frozen,permission['anchor_id']):
        raise ValueError('TOP_LAYER_PERMISSION_CHANGED')
    height=StartHeight.capture(target)
    if height.box_identity!=frozen.evidence()['heights'][target.name]['start_box_identity']:
        raise ValueError('LOW_TOP_FROZEN_TARGET_CHANGED')
    if target.name not in permission['allowed_target_ids']:return None
    option=_fixed_top_option(target,belt,longitudinal,frozen,current_height,height,permission['definition'])
    option.update(template=LAYER_TEMPLATE,layer_top_permission=deepcopy(permission),
        height_source=permission['definition'],low_layer_threshold_m=permission['ceiling_world_m'],
        receiver_height_source='CURRENT_COMMITTED_HEIGHT_FIXED_DURING_TOP_CYCLE')
    return option


def _fixed_top_option(target,belt,longitudinal,frozen,minimum,height,definition):
    if not np.allclose(target.rotation, np.eye(3), atol=1e-12, rtol=0):
        raise ValueError('LOW_TOP_REQUIRES_FROZEN_WORLD_ALIGNED_BOX')
    region = receiving_region(target, belt)
    a,b,c,d = region['projection_center_limits_before_occupancy']
    if not a <= b or not c <= target.center[1]-belt.center[1] <= d:
        raise ValueError('LOW_TOP_FIXED_Y_RECEIVER_FOOTPRINT_UNAVAILABLE')
    projected = translated(target, [belt.center[0]+(a+b)/2-target.center[0],0,0])
    return dict(template=TEMPLATE, face=directional_contact(target,[0.,0.,1.],in_plane_x=[1.,0.,0.]),
        height_m=float(minimum), pre_approach_height_m=float(minimum),
        synchronization='ALREADY_AT_RECEIVE', stationary_receiver_during_target=True,
        # Same existing 50 mm high-drop allowance; at most the existing h+d
        # nominal drop when the box already starts above the minimum receiver.
        release_bottom_world_m=max(height.z_b+height.d, minimum+height.d),
        segment_directions_world=[[0.,0.,1.],[-1.,0.,0.]],
        region=region,reception=reception_classification(projected,belt,longitudinal),
        requires_open_channel=False,original_role=frozen.evidence()['roles'][target.name]['role'],
        target_id=target.name,height_source=('FROZEN_HMIN_BELOW_MECHANISM_MINIMUM'
            if definition==LEGACY_LOW_DEFINITION else BOX_TOP_LOW_DEFINITION),
        low_definition=definition,box_top_world_m=height.z_b+height.h,
        low_layer_threshold_m=float(minimum),endpoint='REGION_NOT_FIXED_POINT',
        acceptance='NOT_PLANNED',spatial_orientation='UPSTREAM_TOP_FACE_POSE_HELD',
        fixed_wrist_joint_values=False,third_sync_disabled=True)


def top_floor_support(target, obstacles, tolerance):
    """Only the named actual floor plane touching the whole bottom footprint."""
    floors=[b for b in obstacles if b.name=='trailer_floor' and b.category=='floor']
    if len(floors)!=1:raise ValueError('LOW_TOP_NAMED_FLOOR_MISSING')
    floor=floors[0]
    if not np.allclose(floor.rotation,np.eye(3),atol=1e-12,rtol=0):
        raise ValueError('LOW_TOP_FLOOR_NOT_HORIZONTAL')
    lo,hi=bounds(target);fl,fh=bounds(floor)
    touching=abs(lo[2]-fh[2])<=tolerance
    inside=np.all(lo[:2]>=fl[:2]) and np.all(hi[:2]<=fh[:2])
    return ['trailer_floor'] if touching and inside else []


class LowTopReceiverConnector(CoupledCycleConnector):
    @property
    def planar(self):return False

    def validate_template(self,target):
        super().validate_template(target)
        if 'world_z_rotation_degrees' in self.option:
            from .bottom_rotated import validate_rotation
            validate_rotation(target,self.option,getattr(self,'rotation_permission',None))
        h=StartHeight.capture(target.box)
        layer=self.option['template']==LAYER_TEMPLATE
        if self.option['template']==L01_HMAX_TOP_TEMPLATE:
            from .l01_hmax import validate_option
            validate_option(target,self.option,getattr(self,'l01_hmax_permission',None),self.batch_root['height_m'],
                c04_side=getattr(self,'c04_side_permission',None),c04_face=getattr(self,'c04_face_permission',None))
            scope_valid=True
        elif self.option['template']==SELECTED_TOP_TEMPLATE:
            from .l01_selected_routes import validate_selected_route
            validate_selected_route(target,self.option,getattr(self,'selected_routes_permission',None),self.batch_root['height_m'])
            scope_valid=True
        elif layer:
            permission=getattr(self,'layer_top_permission',None)
            scope_valid=(permission is not None and self.option.get('layer_top_permission')==permission
                and target.target_id in permission['allowed_target_ids']
                and h.z_b+h.h<=permission['ceiling_world_m']+1e-9
                and self.option['height_m']==self.batch_root['height_m'])
        else:
            scope_valid=(self.option['height_m']==self.option['low_layer_threshold_m']
                and uses_low_top(h,self.option['height_m'],self.option.get('low_definition',LEGACY_LOW_DEFINITION)))
        if (self.option['template'] not in TOP_TEMPLATES or target.face_role!='TOP'
                or not np.allclose(target.outward_normal,[0.,0.,1.],atol=1e-12,rtol=0)
                or target.world_y_offset_m!=0 or target.eccentric_authority is not None
                or not scope_valid
                or self.option['segment_directions_world']!=[[0.,0.,1.],[-1.,0.,0.]]
                or abs(self.option['release_bottom_world_m']-max(h.z_b+h.d,self.option['height_m']+h.d))>1e-9):
            raise MoveItUnavailable('LOW_TOP_PROCESS_SCOPE_CHANGED')
        self._top_lift_in_progress=False
        self._top_transfer=False

    def attachment_support_names(self,target,obstacles):
        return sorted(set(super().attachment_support_names(target,obstacles)+
            top_floor_support(target.box,obstacles,self.contact_tolerance_m)))

    def release_bottom_world_m(self,target):
        return self.option['release_bottom_world_m']

    def _build_native_request(self,*args,**kwargs):
        if getattr(self,'_top_transfer',False) and kwargs.get('stage')=='extraction':
            # After the checked +Z separation, restore ordinary payload/world
            # rules for the entire transfer and release, including the floor.
            kwargs=dict(kwargs,initial_proximity=None,support_names=())
        return super()._build_native_request(*args,**kwargs)

    def _region_context_binding(self,start,target,obstacles,attachment,payload_world,supports,tracker,seed):
        if not self._top_transfer or not tracker.fully_released:
            raise MoveItUnavailable('TOP_TRANSFER_BEFORE_CHECKED_SEPARATION')
        self.check_context(target,obstacles)
        request=self._build_native_request(start,start,payload_world,seed=seed,stage='extraction',attachment=attachment)
        parent,_,transition=self._native_parent_for_request(start,request)
        if transition is not None:raise MoveItUnavailable('TOP_TRANSFER_ATTACHMENT_OWNERSHIP_CHANGED')
        return digest(dict(request=request,parent=parent.evidence(),receiver=box_message(self.high_belt),
            checked_lift_receipt=self._top_lift_receipt))

    def _plan_extraction_suffix(self,plan,row,start,target,obstacles,attachment,payload_world,supports,tracker,seed):
        # A legal local IK recovery starts a fresh prefix/attachment branch.
        self._top_transfer=False
        self._top_lift_in_progress=False
        began=perf_counter();physical=self.physical_contact_pose(start);payload=attachment.box_at(start)
        distance=self.release_bottom_world_m(target)-bounds(payload)[0][2]
        if distance<=0:raise MoveItUnavailable('LOW_TOP_POSITIVE_LIFT_REQUIRED')
        goal=physical.copy();goal[2,3]+=distance
        required=max(self.collision_policy.pair_clearance('external',self.collision_margin_m)+self.contact_tolerance_m,
            self.collision_policy.free_space_clearance_m)+self.budget.extraction_runtime_clearance_reserve_m
        geometry=dict(direction_world=[0.,0.,1.],distance_m=float(distance),
            start_physical_pose=physical.tolist(),goal_physical_pose=goal.tolist(),
            required_terminal_clearance_m=required,support_names=supports,
            method='NATIVE_TOP_VERTICAL_SEPARATION_TO_FIXED_RECEIVER_CLEARANCE')
        path,failure,evidence=self._native_plan(start,None,payload_world,seed=seed,stage='extraction',
            purpose=MotionPurpose.EXTRACTION_PROCESS,goal_pose=self.virtual_from_physical(goal),
            attachment=attachment,support_names=supports,initial_proximity=tracker.clone(),
            allow_ompl=False,max_ompl_attempts=0,try_ptp=False,try_prior=False)
        row['top_lift']=dict(geometry=geometry,native=evidence)
        row['times']['extraction_target_s']=0.;row['times']['extraction_s']=perf_counter()-began
        if failure:return self._reject_suffix(row,failure)
        self._top_lift_in_progress=True
        try:
            lifted,row=SingleExtractionConnector._complete_extraction_candidate(self,plan,row,start,path,geometry,
                target,obstacles,attachment,payload_world,supports,tracker,seed)
        finally:self._top_lift_in_progress=False
        if lifted is None:return None,row
        if not tracker.fully_released:raise MoveItUnavailable('TOP_LIFT_NOT_FULLY_SEPARATED')
        row['top_lift'].update(sample_check=deepcopy(row['extraction_samples']),
            terminal_check=deepcopy(row['terminal_separation']),q_rad=np.asarray(path[-1]).tolist())
        lifted['top_lift']=deepcopy(row['top_lift'])
        self._top_lift_receipt=deepcopy(path[-1].native_receipt)
        self._top_transfer=True;row['complete']=False
        return SingleLiftedReceiverConnector._plan_extraction_suffix(self,lifted,row,path[-1],target,obstacles,
            attachment,payload_world,(),tracker,seed)

    def complete_extraction_plan(self,plan,row,end,target,obstacles,attachment,seed):
        if self._top_lift_in_progress:return plan,row
        result,row=super().complete_extraction_plan(plan,row,end,target,obstacles,attachment,seed)
        if result is not None:
            result['stationary_low_receiver']=dict(height_m=self.option['height_m'],
                throughout_contact_lift_transfer_release=True,top_lift=deepcopy(row['top_lift']))
        return result,row
