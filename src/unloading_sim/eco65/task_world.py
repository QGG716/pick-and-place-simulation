"""Same CAD world, with an isolated single-carton task lifecycle."""
from __future__ import annotations
import copy
import numpy as np
from .model import transform,check_transform
from .unloading_layout import UnloadingWorld,seal_check,support_at,bounds,FACE_AXES

class UnloadingTaskWorld(UnloadingWorld):
    def __init__(self,tool,scene):
        self.attached=False;self.released=False;self.current_q=None;self.phase='initial'
        super().__init__(copy.deepcopy(tool),copy.deepcopy(scene))
        self.initial_scene=copy.deepcopy(self.scene)
        self.box_states={b['id']:'STACKED' for b in self.scene['boxes']}
        self.receiving_region=None;self.release_support=None
    def select_target(self,ident,pose=None,face=None):
        if self.attached or self.released:raise ValueError('Target cannot change during a task')
        super().select_target(ident,pose,face)
        self.initial_scene=copy.deepcopy(self.scene)
    def box_pose(self,q,phase):
        if self.attached:return self.robot.fk(q)@self.payload_relative
        if self.released_pose is not None:return self.released_pose.copy()
        return self.active_pose.copy()
    def set_state(self,q,phase):
        if hasattr(self,'robot'):
            self.active_pose=self.box_pose(q,phase)
        self.current_q=np.asarray(q,float).copy();self.phase=phase
        return super().set_state(q,phase)
    def _support_pair(self,a,b):
        for upper,lower in ((a,b),(b,a)):
            if upper['kind']!='payload':continue
            if lower['kind']!='payload' and lower['name'] not in self.scene['support_surface_ids']:continue
            # Moving target may touch only its original support during extraction,
            # or the selected receiving belt during descent/release/outfeed.
            if upper['name']!=self.box_id and lower['name'] not in self.boxes[upper['name']]['supported_by']:continue
            if upper['name']==self.box_id:
                original=self.boxes[self.box_id]['supported_by']
                receivers=[o['id'] for o in self.scene['obstacles'] if o.get('owner')==self.receiving_region and o['id'] in self.scene['support_surface_ids']]
                if self.phase in ('initial','approach','contact','loaded_lift','front_separation'):
                    allowed=original
                elif self.phase in ('place_contact','retreat','ideal_outfeed'):
                    allowed=receivers if self.phase!='ideal_outfeed' else self.scene['support_surface_ids']
                else:allowed=[]
                if lower['name'] not in allowed:continue
            A=self._pose(upper['name']);B=self._pose(lower['name'])
            ah=np.array(self.object_info[upper['name']]['size_m'])/2;bh=np.array(self.object_info[lower['name']]['size_m'])/2
            bottom=A[2,3]-np.abs(A[2,:3])@ah;top=B[2,3]+np.abs(B[2,:3])@bh
            if not np.allclose(B[:3,:3],np.eye(3),atol=1e-8):continue
            overlap=np.minimum(A[:2,3]+np.abs(A[:2,:3])@ah,B[:2,3]+bh[:2])-np.maximum(A[:2,3]-np.abs(A[:2,:3])@ah,B[:2,3]-bh[:2])
            if bottom>=top-self.scene['contact_tolerance_m'] and np.all(overlap>0):return True
        return False
    def valid(self,q,phase='initial',detail=False):
        self.phase=phase
        if hasattr(self,'initial_scene'):
            for original in self.initial_scene['boxes']:
                if original['id']!=self.box_id and self.boxes[original['id']]['pose']!=original['pose']:raise ValueError('Non-target carton moved')
        # Parent's contact exception tests full rings at actual gap, normal and face.
        return super().valid(q,phase,detail)
    def seal_fit(self,q,box):
        r=seal_check(self.tool,self.robot.fk(q),box,self.boxes[self.box_id]['size_m'],self.active_face)
        return r['fits'],r
    def attach(self,q):
        if self.attached or self.released or self.phase!='contact':raise ValueError('Invalid attach lifecycle')
        if self.boxes[self.box_id]['supports']:raise ValueError('Target supports another carton')
        if not self.valid(q,'contact'):raise ValueError(str(self.last_failure))
        good,report=self.seal_fit(q,self.active_pose)
        if not good:raise ValueError('Actual rings do not seal: '+str(report))
        self.payload_relative=np.linalg.inv(self.robot.fk(q))@self.active_pose
        self.attached=True;self.box_states[self.box_id]='ATTACHED'
        return self.payload_relative.copy()
    def support_report(self,pose):
        half=np.array(self.boxes[self.box_id]['size_m'])/2
        corners=np.array([[x,y,-half[2]] for x in (-half[0],half[0]) for y in (-half[1],half[1])])@pose[:3,:3].T+pose[:3,3]
        z=self.scene['source_config']['conveyors']['surface_z_m'];tol=self.scene['contact_tolerance_m']
        # Conservatively cover the actual rotated bottom's bounding rectangle.
        low=corners[:,:2].min(axis=0);high=corners[:,:2].max(axis=0)
        result=support_at(self.scene,(low+high)/2,[*(high-low),2*half[2]],z)
        ids={o['id'] for o in self.scene['obstacles'] if o.get('owner')==self.receiving_region}
        result['valid']=bool(np.max(np.abs(corners[:,2]-z))<=tol and result['coverage']>=1-1e-8 and set(result['surface_ids'])<=ids)
        result['bottom_height_error_m']=float(np.max(np.abs(corners[:,2]-z)))
        return result
    def release(self,q):
        if not self.attached or self.released:raise ValueError('Release requires attachment')
        if not self.valid(q,'place_contact'):raise ValueError(str(self.last_failure))
        pose=self.box_pose(q,'place_contact');report=self.support_report(pose)
        if not report['valid']:raise ValueError('Release requires selected belt support: '+str(report))
        self.released_pose=pose.copy();self.active_pose=pose.copy();self.attached=False;self.released=True
        self.box_states[self.box_id]='SUPPORTED_RELEASE';self.release_support=report
        self.boxes[self.box_id]['supported_by']=list(report['surface_ids']);self.scene['box']['supported_by']=list(report['surface_ids'])
        self.scene['occupancy'][self.receiving_region]=[self.box_id]
        for b in self.scene['boxes']:
            if self.box_id in b['supports']:b['supports'].remove(self.box_id)
        return pose.copy()
