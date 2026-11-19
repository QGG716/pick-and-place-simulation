"""Source-bound field confirmation for a finite list of pointcloud artefacts.

The original listed volumes stay fixed in base_link, without an added margin.
This is an operator premise, never sensor evidence of free space. Other
instances, supports, fixed geometry and complete payload bounds remain.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from ..coordinates import rigid
from .geometry import transform, corners

ROOT=Path(__file__).resolve().parents[4]


def policy(config):
    value=config.get('upper_carton_pointcloud_artifacts',{'mode':'KEEP'})
    if not isinstance(value,dict) or value.get('mode') not in ('KEEP','USER_CONFIRMED_NAMED_LIST'):
        raise ValueError('UPPER_CARTON_POINTCLOUD_POLICY_INVALID')
    if value['mode']!='KEEP':
        if not isinstance(value.get('basis'),str) or not value['basis'].strip():
            raise ValueError('UPPER_CARTON_POINTCLOUD_FIELD_BASIS_REQUIRED')
        sha=value.get('manifest_sha256')
        if (not isinstance(value.get('manifest'),str) or not value['manifest'].strip()
                or not isinstance(sha,str) or len(sha)!=64 or any(c not in '0123456789abcdef' for c in sha)):
            raise ValueError('UPPER_CARTON_POINTCLOUD_NAMED_MANIFEST_REQUIRED')
    return dict(value)


def _size(value):
    a=np.asarray(value,float)
    if a.shape!=(3,) or not np.isfinite(a).all() or np.any(a<=0):
        raise ValueError('UPPER_CARTON_POINTCLOUD_BOX_SIZE_INVALID')
    return a


def load_manifest(config):
    chosen=policy(config)
    if chosen['mode']=='KEEP':return None
    path=(ROOT/chosen['manifest']).resolve();raw=path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=chosen['manifest_sha256']:
        raise ValueError('UPPER_CARTON_POINTCLOUD_MANIFEST_HASH_CHANGED')
    value=json.loads(raw)
    if (value.get('schema')!='eco65_named_pointcloud_artifacts_v1' or value.get('frame')!='base_link'
            or value.get('evidence')!='USER_FIELD_CONFIRMATION_NOT_SENSOR_FREE_SPACE'
            or value.get('basis')!=chosen['basis']):
        raise ValueError('UPPER_CARTON_POINTCLOUD_MANIFEST_INVALID')
    for key in ('reference_task_id','reference_observation_id'):
        if not isinstance(value.get(key),str) or not value[key].strip():
            raise ValueError('UPPER_CARTON_POINTCLOUD_REFERENCE_REQUIRED:'+key)
    ref=value.get('reference_target',{})
    rigid(ref['measured_pose']);_size(ref['measured_size_m'])
    objects=value.get('objects')
    if not isinstance(objects,list) or not objects:raise ValueError('UPPER_CARTON_POINTCLOUD_EMPTY_LIST')
    ids=set()
    for item in objects:
        ident=item.get('id')
        if (item.get('category')!='observed' or not isinstance(ident,str)
                or not ident.startswith('rgbd_observed_') or ident in ids):
            raise ValueError('UPPER_CARTON_POINTCLOUD_LIST_MUST_NAME_UNIQUE_OBSERVED_OBJECTS')
        ids.add(ident);rigid(item['pose']);_size(item['size_m'])
    return dict(path=str(path),sha256=chosen['manifest_sha256'],data=value)


def exclude_upper_returns(world_points,eligible,masks,boxes,unresolved,config,*,support_above_m,protected_support=None):
    chosen=policy(config);empty=np.zeros(eligible.shape,bool)
    record=dict(policy=chosen['mode'],status='DISABLED',ignored_points=0,
        evidence='USER_FIELD_CONFIRMATION_NOT_SENSOR_FREE_SPACE',basis=chosen.get('basis'),
        scope='NAMED_SOURCE_VOLUMES_UNCLAIMED_POINTS_ONLY',added_margin_m=0.,
        other_instance_masks_protected=True,support_points_protected=True,
        fixed_geometry_unchanged=True,payload_envelope_unchanged=True,seal_evidence_unchanged=True)
    if chosen['mode']=='KEEP':return empty,record
    loaded=load_manifest(config);manifest=loaded['data'];reference=manifest['reference_target']
    record.update(manifest=loaded['path'],manifest_sha256=loaded['sha256'],
        reference_observation_id=manifest['reference_observation_id'],reference_task_id=manifest['reference_task_id'],
        listed_objects=len(manifest['objects']),reference_target=reference,
        association='UPPER_MEASURED_CENTER_INSIDE_REFERENCE_MEASURED_BOX_FIXED_LIST_NOT_MOVED')
    heights={}
    for b in boxes:
        faces=[f for f in b.get('measured_box',b).get('faces',[]) if f.get('observed')]
        if faces:heights[b['detection_index']]=float(faces[0]['center_m'][2])
    for item in unresolved:
        face=(item.get('evidence') or {}).get('observed_face')
        if face:heights[item['detection_index']]=float(face['center_m'][2])
    if not heights or not np.isfinite(list(heights.values())).all():
        record['status']='NOT_APPLIED_NO_OBSERVED_UPPER';return empty,record
    ordered=sorted(heights,key=lambda i:(-heights[i],i));index=ordered[0]
    if len(ordered)>1 and heights[index]==heights[ordered[1]]:
        record['status']='NOT_APPLIED_AMBIGUOUS_UPPER';return empty,record
    target=next((b for b in boxes if b['detection_index']==index),None)
    if target is None or not target.get('measured_box'):
        record['status']='NOT_APPLIED_UPPER_HAS_NO_MEASURED_BOX';return empty,record
    measured=target['measured_box'];T=rigid(measured['pose'])
    center=transform(np.linalg.inv(rigid(reference['measured_pose'])),T[:3,3][None])[0]
    if np.any(abs(center)>_size(reference['measured_size_m'])/2):
        record['status']='NOT_APPLIED_UPPER_MOVED_OUTSIDE_REFERENCE';return empty,record
    # Include the existing support noise window; never erase the lower carton
    # or support returns to manufacture a gap beneath the selected upper box.
    bottom=float(corners(T,measured['size_m'])[:,2].min())
    protected=np.zeros(eligible.shape,bool)
    for i,mask in enumerate(masks):
        if i!=index:protected|=np.asarray(mask,bool)
    if protected_support is not None:protected|=protected_support
    candidates=eligible&~protected&(world_points[...,2]>bottom+support_above_m)
    pixels=np.flatnonzero(candidates);points=world_points.reshape(-1,3)[pixels]
    selected=np.zeros(len(points),bool);items=[]
    for item in manifest['objects']:
        local=transform(np.linalg.inv(np.asarray(item['pose'],float)),points)
        inside=np.all(abs(local)<=np.asarray(item['size_m'])/2,axis=1)
        items.append(dict(id=item['id'],matched_points=int(inside.sum()),newly_ignored_points=int((inside&~selected).sum())))
        selected|=inside
    ignored=empty;ignored.flat[pixels[selected]]=True;points=world_points[ignored]
    record.update(status='APPLIED',target_id=target['id'],detection_index=index,
        measured_pose=T.tolist(),measured_size_m=measured['size_m'],items=items,
        protected_below_world_m=bottom+support_above_m,ignored_points=int(ignored.sum()),
        ignored_mask_shape=list(ignored.shape),
        ignored_mask_sha256=hashlib.sha256(np.packbits(ignored,bitorder='little').tobytes()).hexdigest(),
        ignored_world_bounds_m=[points.min(0).tolist(),points.max(0).tolist()] if len(points) else None)
    return ignored,record
