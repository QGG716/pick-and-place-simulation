"""Frozen inputs for native planning, independent review and offline display."""
from pathlib import Path
import hashlib
import numpy as np
from .model import ROOT, load_json, save_json, transform
from .native_scene import combined_scene, verify_scene, draw_layout
from .native_model import asset_inventory, build_native_model
from ..moveit2_backend import digest


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_snapshot(snapshot, root=ROOT):
    check = dict(snapshot); expected = check.pop('fingerprint')
    if digest(check) != expected or snapshot['schema'] != 'eco65_native_snapshot_v1':
        raise ValueError('SNAPSHOT_IDENTITY_INVALID')
    verify_scene(snapshot['scene'])
    for name, sha in {**snapshot['asset_sha256'], **snapshot['config_sha256']}.items():
        path = (Path(root)/name).resolve(); path.relative_to(Path(root).resolve())
        if file_hash(path) != sha: raise ValueError('SNAPSHOT_FILE_CHANGED:'+name)
    model = snapshot['native_model']; identity = model['init']['identity']
    if (identity['scene_fingerprint'] != snapshot['scene']['scene_fingerprint'] or
            digest(model['init']['parameters']) != identity['model_tool_fingerprint'] or
            snapshot['tool'] != model['tool']):
        raise ValueError('SNAPSHOT_MODEL_BINDING_INVALID')
    return snapshot


def build_snapshot(output, environment=None, native_asset_root=None):
    from time import perf_counter
    out = Path(output).resolve(); out.relative_to(ROOT/'outputs')
    out.mkdir(parents=True, exist_ok=False)
    begin = perf_counter(); audit = asset_inventory()
    asset_s = perf_counter()-begin; save_json(out/'assets.json', audit)
    begin = perf_counter(); scene = combined_scene(environment=environment)
    save_json(out/'resolved_scene.json', scene); draw_layout(scene, out/'layout.png')
    scene_s = perf_counter()-begin
    report = dict(status='BLOCKED', assets=audit['status'], environment=scene['environment_status'],
                  timings=dict(asset_initialization_s=asset_s, scene_preparation_s=scene_s),
                  planning='NOT_EVALUATED', independent_geometry='NOT_EVALUATED',
                  replay='NOT_EVALUATED', dynamics='NOT_EVALUATED')
    if audit['status'] != 'PASS' or scene['environment_status'] == 'NOT_EVALUATED':
        save_json(out/'build_report.json', report); return report
    begin = perf_counter()
    model = build_native_model(scene, out/'model', native_asset_root=native_asset_root)
    hashes = {**audit['asset_sha256'], **scene['environment_asset_sha256']}
    hashes.update({b['convex_path']: b['convex_sha256'] for b in model['convex_bindings']})
    snapshot = dict(schema='eco65_native_snapshot_v1', scene=scene, tool=model['tool'],
                    native_model=model, asset_sha256=hashes, config_sha256=scene['config_sha256'])
    snapshot['fingerprint'] = digest(snapshot); verify_snapshot(snapshot)
    save_json(out/'snapshot.json', snapshot)
    report.update(status='BUILT_NOT_RUNTIME_VALIDATED', snapshot_fingerprint=snapshot['fingerprint'])
    report['timings']['model_initialization_s'] = perf_counter()-begin
    save_json(out/'build_report.json', report); return report


def warehouse_bounds(assets):
    """Include every mesh/collider AABB, including instance proxies and floor.

    No HOME-based filtering. May reject paths conservatively. Asset hashes and
    the floor normalization used by appearance are frozen here, once.
    """
    from pxr import Usd, UsdGeom, UsdPhysics
    assets = Path(assets).resolve(); assets.relative_to(ROOT/'outputs')
    pinned = load_json(ROOT/'configs/isaac/eco65_warehouse_assets.json')
    manifest = load_json(assets/'official/manifest.json')
    hashes = {}
    for row in manifest['files']:
        path = (assets/'official'/row['path']).resolve(); path.relative_to(assets/'official')
        if file_hash(path) != row['sha256']: raise ValueError('WAREHOUSE_ASSET_CHANGED:'+row['path'])
        hashes[path.relative_to(ROOT).as_posix()] = row['sha256']
    for row in pinned['verified_sources']:
        if hashes.get((assets/'official'/row['path']).relative_to(ROOT).as_posix()) != row['sha256']:
            raise ValueError('PINNED_WAREHOUSE_SOURCE_MISMATCH')
    paths = {Path(row['path']).name: assets/'official'/row['path'] for row in pinned['verified_sources']}
    source = paths['warehouse.usd']; stage = Usd.Stage.Open(str(source))
    if (not stage or stage.GetCompositionErrors() or UsdGeom.GetStageUpAxis(stage) != 'Z'
            or UsdGeom.GetStageMetersPerUnit(stage) != 1.):
        raise ValueError('WAREHOUSE_COMPOSITION_OR_UNITS_INVALID')
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'], useExtentsHint=False)
    records, floors = [], []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not (prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI)): continue
        bounds = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        if bounds.IsEmpty(): raise ValueError('WAREHOUSE_EMPTY_GEOMETRY:'+str(prim.GetPath()))
        lo, hi = np.array(bounds.GetMin()), np.array(bounds.GetMax()); size = hi-lo
        if prim.IsA(UsdGeom.Mesh) and 'floor' in str(prim.GetPath()).lower() and size[0]>2 and size[1]>2 and size[2]<.5:
            floors.append((float(size[0]*size[1]), float(hi[2])))
        records.append((str(prim.GetPath()), lo, hi))
    if not floors: raise ValueError('WAREHOUSE_FLOOR_NOT_VERIFIED')
    floor_z = max(floors)[1]; objects = []
    for i, (prim, lo, hi) in enumerate(records):
        lo[2] -= floor_z; hi[2] -= floor_z
        # Extend zero-thickness visual surfaces conservatively down/sideways.
        for axis in range(3):
            if hi[axis]-lo[axis] < 1e-6: lo[axis] = hi[axis]-1e-6
        objects.append(dict(id=f'bound_{i:05d}', prim=prim, bounds_W_m=[lo.tolist(), hi.tolist()]))
    result = dict(schema='eco65_warehouse_bounds_v1', T_W_D=transform([.32, -.1, .8]).tolist(),
                  complete_traversal=True, objects=objects, asset_sha256=hashes,
                  warehouse_path=source.relative_to(ROOT).as_posix(),
                  carton_path=paths['SM_CardBoxD_04.usd'].relative_to(ROOT).as_posix(),
                  floor_normalization_m=-floor_z, filtering='none_all_bounds_included')
    result['fingerprint'] = digest(result); return result
