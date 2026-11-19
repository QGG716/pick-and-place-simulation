"""A confirmed stack relation bounds a static support, not a carton size.

The field statement supplies stacked support topology. Frozen RGB-D must
independently associate the two instances at the upper carton's bottom. The
lower carton may extend arbitrarily far inward/laterally: keep its entire
possible body within the enforced workspace, below that support plane.
"""
from itertools import combinations
import cv2
import numpy as np
from .geometry import rigid,transform


def policy(geometry):
    value=(geometry or {}).get('confirmed_stack_support',{'mode':'DISABLED'})
    if not isinstance(value,dict):raise ValueError('STACK_SUPPORT_POLICY_INVALID')
    if value.get('mode')=='DISABLED':return None
    if (value.get('mode')!='USER_CONFIRMED_TWO_CARTON_STACK' or
        not isinstance(value.get('basis'),str) or not value['basis'].strip()):
        raise ValueError('STACK_SUPPORT_FIELD_CONFIRMATION_REQUIRED')
    return dict(value)


def intersect_vertices(normals,offsets):
    """Vertices of a bounded half-space intersection n.x <= d."""
    A=np.asarray(normals,float);b=np.asarray(offsets,float);vertices=[]
    for index in combinations(range(len(A)),3):
        matrix=A[list(index)]
        if abs(np.linalg.det(matrix))<1e-10:continue
        p=np.linalg.solve(matrix,b[list(index)])
        if np.all(A@p<=b+1e-9):vertices.append(p)
    if len(vertices)<4:raise ValueError('STACK_SUPPORT_EMPTY_WORKSPACE_INTERSECTION')
    return np.unique(np.round(vertices,12),axis=0)


def infer_supports(observation,result,boxes,unresolved,site,geometry,world_points,valid):
    """Return source-bound support bodies and explicit rejected associations."""
    from .site_scene import TARGET_PIXEL_BAND_PX,TARGET_PLANE_BAND_M,SUPPORT_MIN_POINTS,SUPPORT_BELOW_M,SUPPORT_ABOVE_M,NEAR_TARGET_M
    setting=policy(geometry)
    if setting is None:return {},[]
    diagnostics=[];found={}
    if len(result.masks)!=2 or len(boxes)!=1 or len(unresolved)!=1:
        return {},[dict(status='NOT_APPLIED',reason='CONFIRMATION_ONLY_COVERS_TWO_CARTON_STACK')]
    upper=boxes[0];lower=unresolved[0];e=lower.get('evidence') or {};face=e.get('observed_face')
    def reject(reason,**detail):
        return {},[dict(status='NOT_APPLIED',reason=reason,upper_id=upper['id'],lower_index=lower['detection_index'],**detail)]
    if (not face or upper.get('uncertainty') or
        (upper.get('quality') or {}).get('all_dimensions_source')!='RGBD_VISIBLE_BOUNDARIES'):
        return reject('MEASURED_UPPER_BOTTOM_AND_RELIABLE_LOWER_FACE_REQUIRED')
    T=rigid(upper['pose']);R=T[:3,:3];half=np.asarray(upper['size_m'])/2
    axis=int(np.argmax(abs(R[2])));normal=R[:,axis]*np.sign(R[2,axis])
    if normal[2]<np.cos(np.deg2rad(10)):
        return reject('UPPER_BOTTOM_NOT_WITHIN_EXISTING_SUPPORT_ORIENTATION_RANGE')
    if abs(normal@np.asarray(face['normal']))>np.sin(np.deg2rad(10)):
        return reject('LOWER_FACE_NOT_COMPATIBLE_WITH_STACK_SUPPORT_PLANE')
    offset=float(normal@T[:3,3]-half[axis])
    support=np.asarray(face['center_m'])+np.asarray(face['support_points_uv_m'])@np.asarray([face['u'],face['v']])
    signed=support@normal-offset
    if signed.max()>SUPPORT_ABOVE_M or signed.min()>=0:
        return reject('OBSERVED_LOWER_SURFACE_CONFLICTS_WITH_STACK_RELATION',max_above_plane_m=float(signed.max()))
    upper_mask=np.asarray(result.masks[upper['detection_index']],bool)
    lower_mask=np.asarray(result.masks[lower['detection_index']],bool)
    band=cv2.dilate(upper_mask.astype('uint8'),np.ones((2*TARGET_PIXEL_BAND_PX+1,)*2,'uint8')).astype(bool)
    distance=world_points@normal-offset
    local=(world_points-T[:3,3])@R;planar=[i for i in range(3) if i!=axis]
    joint=lower_mask&band&valid&(distance>=-SUPPORT_BELOW_M)&(distance<=SUPPORT_ABOVE_M)
    joint&=np.all(abs(local[...,planar])<=half[planar]+NEAR_TARGET_M,axis=-1)
    if int(joint.sum())<SUPPORT_MIN_POINTS:
        return reject('NO_SAME_FRAME_SUPPORT_INTERFACE',interface_points=int(joint.sum()))
    # Full possible lower body: workspace AND inward side of its observed
    # front plane AND below the confirmed support plane. No depth/width guess.
    lo,hi=np.asarray(site['workspace_bounds_m'],float)
    A=list(np.eye(3))+list(-np.eye(3));b=list(hi)+list(-lo)
    front=np.asarray(face['normal']);A += [front,normal]
    b += [float(front@np.asarray(face['center_m'])+TARGET_PLANE_BAND_M),offset]
    vertices=intersect_vertices(A,b)
    x=front-normal*(normal@front);x/=np.linalg.norm(x)
    frame=np.column_stack((x,np.cross(normal,x),normal));v=vertices@frame
    low,high=v.min(0),v.max(0);high[2]=offset
    pose=np.eye(4);pose[:3,:3]=frame;pose[:3,3]=frame@((low+high)/2)
    body=dict(id=f"rgbd_confirmed_stack_support_{lower['detection_index']}_for_{upper['id']}",
        category='confirmed_stack_support',pose=pose.tolist(),size=(high-low).tolist(),
        supports_target_id=upper['id'],detection_index=lower['detection_index'],geometry_uncertain=True,
        support_plane=dict(normal_world=normal.tolist(),offset_m=offset),
        support_basis=setting['basis'],observation_id=observation.frame.observation_id)
    record=dict(status='APPLIED',upper_id=upper['id'],lower_index=lower['detection_index'],
        object_id=body['id'],basis=setting['basis'],plane=body['support_plane'],
        plane_source='MEASURED_UPPER_BOTTOM_PLUS_USER_CONFIRMED_STACK_TOPOLOGY_NOT_MEASURED_LOWER_TOP',
        interface_points=int(joint.sum()),interface_gap_m=[float(-distance[joint].max()),float(-distance[joint].min())],
        lower_front_max_above_plane_m=float(signed.max()),
        hidden_depth_measured=False,full_dimensions_m=None,carton_end_at_workspace_boundary_assumed=False,
        complete_static_domain_vertices=vertices.tolist(),support_penetration_limit_m=.0004,
        contact_permission_scope='THIS_TARGET_PAYLOAD_DURING_DETACH_ONLY_NO_ROBOT_OR_TOOL_EXCEPTION')
    found[lower['detection_index']]=(body,record)
    return found,[record]
