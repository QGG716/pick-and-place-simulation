"""Private, explicitly registered CAD static layer. No scene or robot guesses."""
import hashlib
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from ..coordinates import rigid
from .geometry import transform


def registration_limits(config):
    c=config.get('layout_registration',{})
    if not isinstance(c,dict):raise ValueError('LAYOUT_REGISTRATION_CONFIG_INVALID')
    limits=dict(max_rms_m=c.get('max_rms_m',.005),max_p95_m=c.get('max_p95_m',.010),
        basis=c.get('basis','USER_DECISION_2026_11_19_RGBD_LAYOUT_REGISTRATION_LIMITS'))
    if not isinstance(limits['basis'],str) or not limits['basis'].strip():raise ValueError('LAYOUT_REGISTRATION_BASIS_REQUIRED')
    for key,bound in [('max_rms_m',.005),('max_p95_m',.010)]:
        v=limits[key]
        if type(v) not in (int,float) or not np.isfinite(v) or not 0<v<=bound:raise ValueError('LAYOUT_REGISTRATION_LIMIT_INVALID:'+key)
    return limits


def load_layout(config,root,frozen_lift=None):
    c=config.get('site_layout')
    if c is None:return None
    root=Path(root).resolve();raw=(root/'assets/layout/raw').resolve();derived=(root/'assets/layout/derived').resolve()
    if not isinstance(c,dict):raise ValueError('SITE_LAYOUT_CONFIG_INVALID')
    manifest=(root/c['manifest']).resolve()
    if not manifest.is_relative_to(derived):raise ValueError('SITE_LAYOUT_MANIFEST_OUTSIDE_PRIVATE_DERIVED')
    report=json.loads(manifest.read_text(encoding='utf-8'))
    if report.get('schema')!='eco65_site_layout_cad_v1':raise ValueError('SITE_LAYOUT_SCHEMA_INVALID')
    inputs=[manifest];source=(root/report['source']).resolve()
    files=sorted(p.resolve() for p in raw.rglob('*') if p.is_file() and p.suffix.lower() in ('.step','.stp'))
    if files!=[source] or not source.is_relative_to(raw):raise ValueError('EXACTLY_ONE_SITE_LAYOUT_STEP_REQUIRED')
    if hashlib.sha256(source.read_bytes()).hexdigest()!=report['source_sha256']:raise ValueError('SITE_LAYOUT_SOURCE_HASH_CHANGED')
    inputs.append(source)
    T=rigid(c['T_base_layout'])
    for key in ('transform_basis','exclusion_basis','association_basis'):
        if not isinstance(c.get(key),str) or not c[key].strip():raise ValueError('SITE_LAYOUT_BASIS_REQUIRED:'+key)
    # Association scale is a field input, not inferred from an old scene.
    distance=c.get('association_distance_m')
    limits=registration_limits(config)
    if type(distance) not in (float,int) or not np.isfinite(distance) or distance<=limits['max_p95_m']:
        raise ValueError('LAYOUT_ASSOCIATION_DISTANCE_REQUIRED_ABOVE_ACCEPTANCE_GATE')
    excluded=c.get('excluded_part_ids')
    ids={p['id'] for p in report['parts']}
    if not isinstance(excluded,list) or any(not isinstance(i,str) for i in excluded) or len(set(excluded))!=len(excluded) or not set(excluded)<=ids:
        raise ValueError('SITE_LAYOUT_EXPLICIT_EXCLUSIONS_INVALID')
    mechanism=lift=None
    if 'roles' in c:
        # Roles + one-DOF lift: the same model the simulation scene uses.
        from ...eco65.site_mechanism import load_roles,LiftMechanism,lift_state,check_range,INTERIOR
        roles_path=(root/c['roles']).resolve()
        if not roles_path.is_relative_to((root/'configs').resolve()):raise ValueError('SITE_LAYOUT_ROLES_OUTSIDE_CONFIGS')
        roles=load_roles(roles_path);mechanism=LiftMechanism(report,roles)
        if isinstance(c.get('lift'),dict) and c['lift'].get('source')=='DRAW_WIRE_SENSOR':
            # h comes only from the read-only sensor state frozen with this observation.
            from .lift_sensor import sensor_config
            sensor=sensor_config(c['lift'])
            if not isinstance(frozen_lift,dict) or frozen_lift.get('source')!='DRAW_WIRE_SENSOR':
                raise ValueError('SITE_LIFT_FROZEN_SENSOR_STATE_REQUIRED')
            if frozen_lift.get('config_sha256')!=sensor['identity_sha256']:raise ValueError('SITE_LIFT_SENSOR_CONFIG_CHANGED')
            lift=dict(lift_state(dict(lift=dict({k:v for k,v in c['lift'].items() if k in ('range_m','range_basis')},
                h_m=frozen_lift['h_m'],h_basis=frozen_lift['h_basis'])),mechanism),
                h_interval_m=list(frozen_lift['h_interval_m']),sensor=frozen_lift)
        else:
            lift=lift_state(c,mechanism);lift['h_interval_m']=[lift['h_m'],lift['h_m']]
        lift_range=check_range(mechanism,lift)
        inputs.append(roles_path);h=lift['h_m'];role_excluded=set(roles['excluded_roles'])
        role_of={k:v['role'] for k,v in mechanism.classification.items()}
    world=[];triangles=[]
    for part in report['parts']:
        paths={}
        for key in ('visual','collision'):
            path=(root/part[key]).resolve()
            if not path.is_relative_to(derived) or hashlib.sha256(path.read_bytes()).hexdigest()!=part[key+'_sha256']:
                raise ValueError('SITE_LAYOUT_PART_HASH_CHANGED:'+part['id'])
            inputs.append(path);paths[key]=path
        if part['id'] in excluded:continue
        if not part.get('topology_valid') or part['collision_type']!='analytic_brep_aabb':
            raise ValueError('SITE_LAYOUT_CONSERVATIVE_COLLISION_REQUIRED')
        lo,hi=np.asarray(part['collision_bounds_m'],float)
        if lo.shape!=(3,) or not np.isfinite([lo,hi]).all() or np.any(hi<=lo):raise ValueError('SITE_LAYOUT_BOUNDS_INVALID')
        if mechanism is not None:
            role=role_of[part['id']]
            if role in role_excluded:continue
            # Interior parts register nothing (they move inside the envelope);
            # their collision is the envelope box added below.
            if role==INTERIOR:continue
            visual=trimesh.load(paths['visual'],force='mesh',process=False)
            triangles.append(transform(T@mechanism.transform(role,h),visual.triangles.reshape(-1,3)).reshape(-1,3,3))
            continue
        pose=T.copy();pose[:3,3]+=T[:3,:3]@((lo+hi)/2)
        world.append(dict(id='site_layout_'+part['id'],pose=pose.tolist(),size=(hi-lo).tolist(),
            category='site_layout_cad',basis='CAD_DESIGN_GEOMETRY_NOT_FIELD_MEASUREMENT',
            source_sha256=report['source_sha256'],part_id=part['id'],name=part['name'],assembly_instance=part['instance']))
        visual=trimesh.load(paths['visual'],force='mesh',process=False)
        triangles.append(transform(T,visual.triangles.reshape(-1,3)).reshape(-1,3,3))
    mechanism_identity=None
    if mechanism is not None:
        for box in mechanism.boxes(*lift['h_interval_m']):
            if box['role']!=INTERIOR and box['part_ids'][0] in excluded:continue
            world.append(dict(id='site_layout_'+box['id'],pose=(T@np.asarray(box['pose'])).tolist(),size=box['size'],
                category='site_layout_cad',basis='CAD_DESIGN_GEOMETRY_NOT_FIELD_MEASUREMENT',source_sha256=report['source_sha256'],
                part_id=box['id'],name=box['name'],role=box['role'],part_ids=box['part_ids'],lift_h_m=h))
        mechanism_identity=dict(roles_sha256=roles['_sha256'],lift=lift,lift_range_states=lift_range,state=mechanism.state(h),
            summary=mechanism.summary(),excluded_roles=roles['excluded_roles'],
            role_counts={r:sum(1 for v in role_of.values() if v==r) for r in sorted(set(role_of.values()))})
    if not world:raise ValueError('SITE_LAYOUT_HAS_NO_STATIC_PARTS')
    identity=dict(source_sha256=report['source_sha256'],manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        excluded_part_ids=excluded,exclusion_basis=c['exclusion_basis'],T_base_layout=T.tolist(),transform_basis=c['transform_basis'],
        registration_limits=limits,association_distance_m=float(distance),association_basis=c['association_basis'])
    if mechanism_identity is not None:identity['mechanism']=mechanism_identity
    return dict(world=world,triangles=np.concatenate(triangles),inputs=inputs,
        identity=identity,
        limits=limits,association_distance_m=float(distance),association_basis=c['association_basis'])


def surface_distances(points,triangles,max_distance):
    """Exact triangle distance in the association band, without an rtree install.

    Nearest triangle-centroid gives an upper bound. All triangles whose sphere
    can improve that bound are checked, so no fixed nearest-K approximation.
    """
    points=np.asarray(points,float);triangles=np.asarray(triangles,float)
    centers=triangles.mean(1);radii=np.linalg.norm(triangles-centers[:,None,:],axis=2).max(1)
    tree=cKDTree(centers);distance,index=tree.query(points);largest=float(radii.max());answer=np.full(len(points),np.inf)
    for i in np.flatnonzero(distance-largest<=max_distance):
        point=points[i];first=triangles[index[i]:index[i]+1]
        near=trimesh.triangles.closest_point(first,point[None])[0];upper=float(np.linalg.norm(near-point))
        candidates=tree.query_ball_point(point,min(upper,max_distance)+largest)
        candidates=[j for j in candidates if np.linalg.norm(centers[j]-point)-radii[j]<=min(upper,max_distance)]
        if not candidates:continue
        closest=trimesh.triangles.closest_point(triangles[candidates],np.repeat(point[None],len(candidates),axis=0))
        answer[i]=np.linalg.norm(closest-point,axis=1).min()
    return answer


def register_layout(layout,world_points):
    p=np.asarray(world_points,float);p=p[np.isfinite(p).all(1)]
    distances=surface_distances(p,layout['triangles'],layout['association_distance_m'])
    selected=distances[np.isfinite(distances)&(distances<=layout['association_distance_m'])]
    rms=float(np.sqrt(np.mean(selected**2))) if len(selected) else None
    p95=float(np.percentile(selected,95)) if len(selected) else None
    valid=bool(len(selected) and rms<=layout['limits']['max_rms_m'] and p95<=layout['limits']['max_p95_m'])
    return dict(status='PASS' if valid else 'LAYOUT_REGISTRATION_MISMATCH',valid=valid,
        rms_m=rms,p95_m=p95,associated_points=len(selected),input_points=len(p),limits=layout['limits'],
        association_distance_m=layout['association_distance_m'],association_basis=layout['association_basis'],
        identity=layout['identity'],basis='FROZEN_RGBD_TO_STATIC_CAD_TRIANGLE_RESIDUALS',
        cad_is_measured=False,hardware_validation='NOT_RUN')


def merge_registered_layout(scene,observation,result,config,planner,self_filter,record,root=None):
    from ..detection import metric_points
    layout=load_layout(config,root or Path(__file__).resolve().parents[4],getattr(observation,'lift_state',None))
    if layout is None:return scene
    frame=observation.frame;points=metric_points(frame)
    valid=np.isfinite(points).all(-1)&(points[...,2]>.1)&(points[...,2]<3.)
    # Movable detections cannot certify a registration to static CAD.
    for mask in result.masks:valid&=~np.asarray(mask,bool)
    world=transform(observation.world_camera,points[valid])
    if self_filter is not None:world=world[~self_filter.contains(world,observation.q_capture)]
    registration=register_layout(layout,world)
    record('layout-registration.json',registration)
    if not registration['valid']:raise ValueError('LAYOUT_REGISTRATION_MISMATCH')
    for path in layout['inputs']:planner.track(path)
    # Retain every RGB-D obstacle. No depth/voxel de-duplication is performed.
    return dict(scene,world=scene['world']+layout['world'],layout_registration=registration,
        imported_site_layout_objects=len(layout['world']),layout_explained_depth_removed=0)


def check_layout_start(planner,q,world,boxes,folder,cancelled=None):
    ids={o['id'] for o in world if o.get('category')=='site_layout_cad'}
    if not ids:return None
    box=boxes[0];cups=planner.site['topology']['connected_cup_ids']
    # Empty-state validation only. No contact permissions, no attached payload,
    # no IK/path search, no seal assumption, and no motion submission.
    candidate=dict(face=box['faces'][0],active_ids=cups,target_lip_contact_ids=cups)
    request=dict(planner.context(q,world,box,candidate,'approach'),op='validate')
    result=planner.native_call(request,folder,'layout-q-start',120,cancelled)
    planner.write_evidence(folder/'layout-start-check.json',dict(q_start=list(q),layout_ids=sorted(ids),
        result=result,hardware_execution='NOT_RUN'),'layout-start-check.json')
    if not result.get('native_valid') or not result.get('legacy_intersection_valid'):
        pair=set((result.get('failure') or {}).get('pair') or [])
        reason='START_POSE_COLLIDES_WITH_LAYOUT' if ids&pair else 'START_POSE_COLLISION_OR_CLEARANCE_FAILED'
        raise ValueError(reason+':'+json.dumps(result.get('failure')))
    return result
