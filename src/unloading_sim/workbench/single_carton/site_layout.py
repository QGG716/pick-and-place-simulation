"""Private, explicitly registered CAD static layer. No scene or robot guesses."""
import hashlib
import json
import os
import threading
import time
from copy import deepcopy
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


TRANSFORM_SOURCES=('RGBD_REGISTRATION','CAD_DESIGN_USER_CONFIRMED')
DEPTH_POLICIES=('KEEP_ALL_RGBD','STEP_PARTS_REPLACE_RGBD')


def transform_source(c):
    """How T_base_layout is established. RGBD_REGISTRATION (default): the
    November 19 frozen RGB-D residual gate per observation. CAD_DESIGN_USER_CONFIRMED
    (user decision 2026-11-27 "直接用 CAD 设计变换"): the roles file's
    cad_robot_base, recorded as field confirmation, not measurement."""
    mode=(c or {}).get('transform_source','RGBD_REGISTRATION')
    if mode not in TRANSFORM_SOURCES:raise ValueError('SITE_LAYOUT_TRANSFORM_SOURCE_INVALID')
    return mode


def depth_policy(c):
    """RGB-D returns inside STEP parts. KEEP_ALL_RGBD (default): every return
    stays an obstacle. STEP_PARTS_REPLACE_RGBD (user decision 2026-11-27
    "由 STEP 代替"): unclaimed returns inside a kept STEP part box grown by
    margin_m build no voxel; the part's CAD box is the obstacle there. Target
    claims and measured supports are decided before and are never removed."""
    p=(c or {}).get('rgbd_depth_policy')
    if p is None:return dict(mode='KEEP_ALL_RGBD')
    if not isinstance(p,dict) or p.get('mode') not in DEPTH_POLICIES:raise ValueError('SITE_LAYOUT_DEPTH_POLICY_INVALID')
    if p['mode']=='KEEP_ALL_RGBD':return dict(mode='KEEP_ALL_RGBD')
    m=p.get('margin_m')
    if type(m) not in (int,float) or not np.isfinite(m) or not 0<=m<=.05:raise ValueError('SITE_LAYOUT_DEPTH_MARGIN_INVALID: 0 .. 0.05 m')
    for key in ('basis','margin_basis'):
        if not isinstance(p.get(key),str) or not p[key].strip():raise ValueError('SITE_LAYOUT_BASIS_REQUIRED:rgbd_depth_policy.'+key)
    return dict(mode=p['mode'],margin_m=float(m),basis=p['basis'],margin_basis=p['margin_basis'])


def replaced_volumes(layout,c):
    """Boxes (pose, half extents incl. margin) whose unclaimed returns are replaced by STEP."""
    p=depth_policy(c)
    if layout is None or p['mode']!='STEP_PARTS_REPLACE_RGBD':return []
    return [dict(id=o['id'],pose=o['pose'],half_m=(np.asarray(o['size'],float)/2+p['margin_m']).tolist())
            for o in layout['world'] if 'size' in o]


# Built once per process (user 2026-11-27: the scene follows the site STEP,
# not a per-task build). Reused while every input keeps device, inode, size,
# mtime_ns and ctime_ns (November 16 3f binding rule); otherwise rebuilt.
_CACHE={};_CACHE_LOCK=threading.Lock();_CACHE_STATS=dict(built=0,reused=0)


def _stat(path):
    s=os.stat(path);return (s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)


def _raw_steps(root):
    raw=(Path(root).resolve()/'assets/layout/raw')
    return sorted(str(p) for p in raw.rglob('*') if p.is_file() and p.suffix.lower() in ('.step','.stp')) if raw.is_dir() else []


def _view(layout,status,seconds):
    out=dict(layout);out['world']=deepcopy(layout['world'])
    out['cache']=dict(status=status,build_s=layout['build_s'],lookup_s=seconds,process_built=_CACHE_STATS['built'],
        process_reused=_CACHE_STATS['reused'],policy='REUSE_WHILE_DEV_INO_SIZE_MTIME_NS_CTIME_NS_UNCHANGED')
    return out


def load_layout(config,root,frozen_lift=None):
    c=config.get('site_layout')
    if c is None:return None
    started=time.monotonic()
    site=config.get('site') if isinstance(config.get('site'),dict) else {}
    key=json.dumps([c,str(Path(root).resolve()),frozen_lift,site.get('workspace_bounds_m')],sort_keys=True,default=str)
    with _CACHE_LOCK:
        entry=_CACHE.get(key)
        if entry is not None:
            try:same=all(_stat(p)==s for p,s in entry['stats']) and _raw_steps(root)==entry['raw']
            except OSError:same=False
            if same:
                _CACHE_STATS['reused']+=1
                return _view(entry['layout'],'REUSED',time.monotonic()-started)
            _CACHE.pop(key,None)
        layout=_load_layout_uncached(config,root,frozen_lift)
        layout['build_s']=time.monotonic()-started;layout['triangles'].setflags(write=False)
        _CACHE[key]=dict(layout=layout,stats=[(p,_stat(p)) for p in layout['inputs']],raw=_raw_steps(root))
        _CACHE_STATS['built']+=1
        return _view(layout,'BUILT',time.monotonic()-started)


def prebuild(config,root,frozen_lift=None):
    """Startup: build the static layer once in the background caller's thread."""
    if (config or {}).get('site_layout') is None:return None
    layout=load_layout(config,root,frozen_lift)
    return dict(objects=len(layout['world']),**layout['cache'])


def _load_layout_uncached(config,root,frozen_lift=None):
    c=config.get('site_layout')
    root=Path(root).resolve();raw=(root/'assets/layout/raw').resolve();derived=(root/'assets/layout/derived').resolve()
    if not isinstance(c,dict):raise ValueError('SITE_LAYOUT_CONFIG_INVALID')
    mode=transform_source(c);depth=depth_policy(c)
    manifest=(root/c['manifest']).resolve()
    if not manifest.is_relative_to(derived):raise ValueError('SITE_LAYOUT_MANIFEST_OUTSIDE_PRIVATE_DERIVED')
    report=json.loads(manifest.read_text(encoding='utf-8'))
    if report.get('schema')!='eco65_site_layout_cad_v1':raise ValueError('SITE_LAYOUT_SCHEMA_INVALID')
    inputs=[manifest];source=(root/report['source']).resolve()
    files=sorted(p.resolve() for p in raw.rglob('*') if p.is_file() and p.suffix.lower() in ('.step','.stp'))
    if files!=[source] or not source.is_relative_to(raw):raise ValueError('EXACTLY_ONE_SITE_LAYOUT_STEP_REQUIRED')
    if hashlib.sha256(source.read_bytes()).hexdigest()!=report['source_sha256']:raise ValueError('SITE_LAYOUT_SOURCE_HASH_CHANGED')
    inputs.append(source)
    limits=registration_limits(config)
    if mode=='CAD_DESIGN_USER_CONFIRMED':
        # The design transform of the roles file; a configured matrix must equal it.
        from ...eco65.site_mechanism import load_roles,cad_robot_base
        if 'roles' not in c:raise ValueError('SITE_LAYOUT_CAD_TRANSFORM_NEEDS_ROLES')
        T=cad_robot_base(load_roles((root/c['roles']).resolve()))
        if c.get('T_base_layout') is not None and not np.allclose(rigid(c['T_base_layout']),T,atol=1e-9):
            raise ValueError('SITE_LAYOUT_CAD_TRANSFORM_DIFFERS_FROM_ROLES')
        keys=('transform_basis','exclusion_basis');distance=None
    else:
        T=rigid(c['T_base_layout']);keys=('transform_basis','exclusion_basis','association_basis')
        # Association scale is a field input, not inferred from an old scene.
        distance=c.get('association_distance_m')
        if type(distance) not in (float,int) or not np.isfinite(distance) or distance<=limits['max_p95_m']:
            raise ValueError('LAYOUT_ASSOCIATION_DISTANCE_REQUIRED_ABOVE_ACCEPTANCE_GATE')
    for key in keys:
        if not isinstance(c.get(key),str) or not c[key].strip():raise ValueError('SITE_LAYOUT_BASIS_REQUIRED:'+key)
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
    # Parts wholly outside the site workspace bounds lie beyond the workspace
    # boundary walls (obstacles 4 cm thick just outside the bounds), so no robot,
    # tool or payload solid can reach them without first meeting a wall: they
    # are not imported. Registration still uses every kept part's surface.
    bounds=(config.get('site') or {}).get('workspace_bounds_m') if isinstance(config.get('site'),dict) else None
    pruned=[]
    if bounds is not None:
        lo,hi=np.asarray(bounds,float);kept=[]
        for o in world:
            P=np.asarray(o['pose'],float);half=abs(P[:3,:3])@(np.asarray(o['size'],float)/2)
            (kept if np.all(P[:3,3]+half>=lo) and np.all(P[:3,3]-half<=hi) else pruned).append(o)
        world=kept
        if not world:raise ValueError('SITE_LAYOUT_HAS_NO_PART_IN_WORKSPACE')
    # Exact fixed mating interfaces (robot base on its STEP mounting plate), named
    # explicitly in the configuration; never inferred from part names.
    mounts=c.get('fixed_mount_interfaces',[])
    ids={o['id'] for o in world};pairs=[]
    if not isinstance(mounts,list):raise ValueError('SITE_LAYOUT_FIXED_MOUNTS_INVALID')
    for m in mounts:
        if (not isinstance(m,dict) or set(m)!={'link','part_id','basis'} or m['link']!='base_link'
                or not isinstance(m['basis'],str) or not m['basis'].strip() or 'site_layout_'+str(m['part_id']) not in ids):
            raise ValueError('SITE_LAYOUT_FIXED_MOUNT_INVALID:'+json.dumps(m,ensure_ascii=False))
        pairs.append(['base_link','site_layout_'+m['part_id']])
    identity=dict(source_sha256=report['source_sha256'],manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
        excluded_part_ids=excluded,exclusion_basis=c['exclusion_basis'],T_base_layout=T.tolist(),transform_basis=c['transform_basis'],
        registration_limits=limits,association_distance_m=None if distance is None else float(distance),
        association_basis=c.get('association_basis'),transform_source=mode,rgbd_depth_policy=depth,fixed_mount_interfaces=mounts,
        workspace_pruning=None if bounds is None else dict(workspace_bounds_m=np.asarray(bounds,float).tolist(),
            kept=len(world),pruned=len(pruned),pruned_ids=sorted(o['id'] for o in pruned),
            basis='WHOLLY_OUTSIDE_WORKSPACE_BOUNDS_BEYOND_BOUNDARY_WALLS'))
    if mechanism_identity is not None:identity['mechanism']=mechanism_identity
    return dict(world=world,triangles=np.concatenate(triangles),inputs=inputs,fixed_mount_pairs=pairs,
        fixed_mount_basis='; '.join(m['basis'] for m in mounts),
        identity=identity,transform_source=mode,rgbd_depth_policy=depth,transform_basis=c['transform_basis'],
        limits=limits,association_distance_m=None if distance is None else float(distance),association_basis=c.get('association_basis'))


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
    if layout['transform_source']=='CAD_DESIGN_USER_CONFIRMED':
        # No per-observation residual gate: the user confirmed the site as designed.
        registration=dict(status='USER_CONFIRMED_CAD_DESIGN_TRANSFORM',valid=True,transform_source=layout['transform_source'],
            transform_basis=layout['transform_basis'],rms_m=None,p95_m=None,rgbd_residual='NOT_COMPUTED',
            identity=layout['identity'],cache=layout['cache'],basis='USER_DECISION_2026_11_27_CAD_DESIGN_TRANSFORM_NOT_MEASUREMENT',
            cad_is_measured=False,hardware_validation='NOT_RUN')
        record('layout-registration.json',registration)
        for path in layout['inputs']:planner.track(path)
        return dict(scene,world=scene['world']+layout['world'],layout_registration=registration,
            imported_site_layout_objects=len(layout['world']),
            layout_explained_depth_removed=scene.get('layout_explained_depth_removed',0))
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
        imported_site_layout_objects=len(layout['world']),
        layout_explained_depth_removed=scene.get('layout_explained_depth_removed',0))


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
