"""Explicit, hashed scene sources for the small calibration adapter only."""
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import ROOT,load_json
from unloading_sim.eco65.native_io import file_hash
from unloading_sim.eco65.asset_paths import resolve_asset
from unloading_sim.touch_calibration import fingerprint


def validate_manifest_identity(snapshot,snapshot_sha256,manifest):
    if (manifest.get('source_snapshot_fingerprint')!=snapshot['fingerprint'] or
        manifest.get('source_snapshot_sha256')!=snapshot_sha256):
        raise ValueError('SNAPSHOT_GEOMETRY_MANIFEST_MISMATCH')
    if manifest.get('source_config_sha256')!=snapshot['config_sha256']:
        raise ValueError('SCENE_CONFIG_MANIFEST_MISMATCH')
    for key,expected in [('T_W_D',snapshot['scene']['T_W_D']),('base_position_D_m',snapshot['scene']['base_position_m'])]:
        if not np.array_equal(manifest[key],expected):raise ValueError('SCENE_FRAME_MANIFEST_MISMATCH:'+key)


def bind_scene_sources(snapshot_path,composed_usd,geometry_manifest,*,root=ROOT):
    """Follow the actual USD reference and compare its collision readback, not names."""
    from pxr import Usd,UsdGeom,Sdf
    root=Path(root).resolve()
    def record(path):
        p=Path(path).resolve();relative=p.relative_to(root).as_posix()
        return dict(path=relative,sha256=file_hash(p))
    sources={key:record(path) for key,path in [('snapshot',snapshot_path),('composed_usd',composed_usd),('geometry_manifest',geometry_manifest)]}
    s=load_json(snapshot_path);m=load_json(geometry_manifest)
    validate_manifest_identity(s,sources['snapshot']['sha256'],m)
    stage=Usd.Stage.Open(str(Path(composed_usd).resolve()))
    if not stage or UsdGeom.GetStageMetersPerUnit(stage)!=1 or UsdGeom.GetStageUpAxis(stage)!='Z':raise ValueError('SCENE_UNITS_OR_AXIS_MISMATCH')
    spec=stage.GetRootLayer().GetPrimAtPath('/World/Workstation')
    refs=[] if spec is None else list(spec.referenceList.GetAppliedItems())
    if len(refs)!=1 or not refs[0].assetPath:raise ValueError('EXPLICIT_WORKSTATION_REFERENCE_REQUIRED')
    resolved=Sdf.ComputeAssetPathRelativeToLayer(stage.GetRootLayer(),refs[0].assetPath)
    sources['workstation_usd']=record(resolved)
    if sources['workstation_usd']['sha256']!=m['workstation_sha256']:raise ValueError('USD_GEOMETRY_MANIFEST_MISMATCH')
    cache=UsdGeom.XformCache();TWD=np.asarray(s['scene']['T_W_D'])
    actual=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Workstation'))).T
    if not np.allclose(actual,TWD,atol=1e-10,rtol=0):raise ValueError('COMPOSED_WORKSTATION_TRANSFORM_MISMATCH')
    max_transform=0.;max_vertex=0.
    for row in m['collision_records']:
        prim=stage.GetPrimAtPath('/World'+row['path'])
        if not prim or not prim.IsA(UsdGeom.Mesh):raise ValueError('COMPOSED_COLLISION_MISSING:'+row['path'])
        a=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get());b=np.asarray(row['points'])
        if a.shape!=b.shape:raise ValueError('COMPOSED_COLLISION_SHAPE_CHANGED:'+row['path'])
        vertex=float(np.max(np.abs(a-b)));delta=float(np.max(np.abs(np.array(cache.GetLocalToWorldTransform(prim)).T-TWD@row['T_D'])))
        if vertex>1e-7 or delta>1e-9:raise ValueError('COMPOSED_COLLISION_GEOMETRY_CHANGED:'+row['path'])
        max_vertex=max(max_vertex,vertex);max_transform=max(max_transform,delta)
    sources['snapshot']['fingerprint']=s['fingerprint']
    binding=dict(schema='eco65_touch_scene_sources_v1',files=sources,
        association=dict(snapshot_to_manifest='fingerprint AND exact file SHA256',
            manifest_to_USD='actual referenced workstation file SHA256 AND composed collision mesh/frame readback',
            collision_records_checked=len(m['collision_records']),max_transform_error=max_transform,max_vertex_error_m=max_vertex))
    binding['binding_sha256']=fingerprint(binding)
    return binding


def derived_scene_identity(c):
    # The exact field set is versioned; no comparison with snapshot or camera hashes.
    return fingerprint(dict(schema='eco65_touch_derived_scene_identity_v1',scene_sources=c['scene_sources']['binding_sha256'],
        **{k:c[k] for k in ('camera','probe','board','kinematics')}))


def verify_scene_sources(c,*,root=ROOT):
    binding=c.get('scene_sources')
    if not binding:raise ValueError('EXPLICIT_SCENE_SOURCES_REQUIRED_NO_LEGACY_FALLBACK')
    files=binding['files']
    for record in files.values():
        if file_hash(resolve_asset(root,record['path']))!=record['sha256']:raise ValueError('BOUND_SCENE_SOURCE_CHANGED:'+record['path'])
    if (c['snapshot']!=files['snapshot']['path'] or c['snapshot_sha256']!=files['snapshot']['sha256'] or
        c['source_fingerprint']!=files['snapshot']['fingerprint']):raise ValueError('CONFIGURATION_SNAPSHOT_SOURCE_MISMATCH')
    fresh=bind_scene_sources(*[resolve_asset(root,files[k]['path']) for k in ('snapshot','composed_usd','geometry_manifest')],root=root)
    if fresh!=binding:raise ValueError('SCENE_SOURCE_ASSOCIATION_CHANGED')
    if c.get('derived_scene_identity')!=derived_scene_identity(c):raise ValueError('DERIVED_SCENE_CONFIGURATION_CHANGED')
    camera=c['camera']
    camera_files={camera['source']:camera['source_sha256'],camera['cad_manifest']:camera['cad_manifest_sha256']}
    for part in camera['parts']:
        for role in ('visual','collision'):camera_files[part[role]]=part[role+'_sha256']
    for path,sha in camera_files.items():
        if file_hash(resolve_asset(root,path))!=sha:raise ValueError('BOUND_CAMERA_ASSET_CHANGED:'+path)
    return binding
