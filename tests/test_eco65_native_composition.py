"""Real USD composition regressions using small, explicitly test-only assets.

These tests exercise source references and the native twelve-carton display
contract. They do not run Isaac, native planning, robot CAD, or physics. Only
the repository root and CLI arguments are redirected to the temporary fixture;
the USD implementation and the composer are real.
"""
import hashlib
import importlib.util
from itertools import product
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest

pytest.importorskip('pxr.Usd', reason='Real USD runtime is required for composition checks')
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from unloading_sim.eco65 import model


SCRIPT = Path(__file__).resolve().parents[1]/'scripts/isaacsim_render_eco65_warehouse.py'
PREFIX = '/World/Workstation'
TARGET = 'carton_r00_l03_c01'
TRANSLATION = np.array([.32, -.10, .80])


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _json(path, value):
    path.write_text(json.dumps(value, indent=2), encoding='utf-8')


def _stage(path, root):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, 'Z')
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, root).GetPrim())
    return stage


def _box_mesh(stage, path, lower, upper):
    points = np.array(list(product(*zip(lower, upper))), dtype=np.float32)
    triangles = [0, 1, 3, 0, 3, 2, 4, 6, 7, 4, 7, 5,
                 0, 4, 5, 0, 5, 1, 2, 3, 7, 2, 7, 6,
                 0, 2, 6, 0, 6, 4, 1, 5, 7, 1, 7, 3]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    mesh.CreateFaceVertexCountsAttr([3]*12)
    mesh.CreateFaceVertexIndicesAttr(triangles)
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateExtentAttr([Gf.Vec3f(*points.min(0).tolist()), Gf.Vec3f(*points.max(0).tolist())])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    return mesh, points


def _cube(stage, path, size, position, *, guide=False):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3d(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if guide:
        cube.CreatePurposeAttr('guide')


def _fixture(root, monkeypatch, *, belt_shift=0.):
    monkeypatch.setattr(model, 'ROOT', root)
    official = root/'assets/official'
    official.mkdir(parents=True)
    warehouse_path, carton_path = official/'warehouse.usda', official/'carton.usda'
    warehouse = _stage(warehouse_path, '/Warehouse')
    _box_mesh(warehouse, '/Warehouse/floor', [-6., -6., -.20], [6., 6., .25])
    for name, lo, hi in [
            ('SM_WallA_West', [-5., -5., .25], [-4.8, 5., 3.25]),
            ('SM_WallA_East', [4.8, -5., .25], [5., 5., 3.25]),
            ('SM_WallB_South', [-5., -5., .25], [5., -4.8, 3.25]),
            ('SM_WallB_North', [-5., 4.8, .25], [5., 5., 3.25])]:
        _box_mesh(warehouse, '/Warehouse/'+name, lo, hi)
    warehouse.GetRootLayer().Save()
    carton = _stage(carton_path, '/Carton')
    # Deliberately off-center, anisotropic source bounds exercise Fit and Pivot.
    _box_mesh(carton, '/Carton/Appearance', [1., -2., .2], [3., 1., 4.2])
    UsdPhysics.RigidBodyAPI.Apply(carton.GetDefaultPrim()).CreateRigidBodyEnabledAttr(True)
    carton.GetRootLayer().Save()
    files = [dict(path=p.name, sha256=_sha(p)) for p in [warehouse_path, carton_path]]
    _json(official/'manifest.json', dict(files=files, file_count=len(files)))
    _json(root/'assets/asset_inspection.json', [
        dict(source='TEST_ONLY_'+name, path=str(path), up_axis='Z', meters_per_unit=1.)
        for name, path in [('warehouse', warehouse_path), ('carton', carton_path)]])

    station_folder, derived = root/'station', root/'derived'
    station_folder.mkdir()
    derived.mkdir()
    station_path = station_folder/'workstation_D.usda'
    station = _stage(station_path, '/Workstation')
    UsdShade.Shader.Define(station, '/Workstation/Materials/glass/Shader').CreateIdAttr('UsdPreviewSurface')
    _cube(station, '/Workstation/trailer_floor', [2., 2., .06], [-.3, 0., -.03])
    _cube(station, '/Workstation/robot_mount_plate', [.15, .15, .04], [-.7, -.5, .16])
    boxes = []
    for layer, column in product(range(4), range(3)):
        ident = f'carton_r00_l{layer:02d}_c{column:02d}'
        position = [.10, [-.22, 0., .22][column], [.23, .33, .43, .53][layer]]
        box = dict(id=ident, size_m=[.25, .21, .10], pose=position)
        boxes.append(box)
        prim = UsdGeom.Xform.Define(station, '/Workstation/Cartons/'+ident)
        T = np.eye(4)
        T[:3, 3] = position
        prim.AddTransformOp().Set(Gf.Matrix4d(T.T.tolist()))
        prim.GetPrim().SetCustomDataByKey('stable_id', ident)
        _cube(station, str(prim.GetPath())+'/Collision', box['size_m'], [0., 0., 0.], guide=True)

    items = []
    for name, lo, hi in [
            ('transverse', [-.55, -.9, .14], [-.3, -.2, .18]),
            ('longitudinal', [-.9, .3, .14], [-.25, .55, .18])]:
        path = derived/(name+'.usda')
        stage = _stage(path, '/Conveyor')
        shift = np.array([0., 0., belt_shift if name == 'longitudinal' else 0.])
        _, points = _box_mesh(stage, '/Conveyor/Belt', np.array(lo)+shift, np.array(hi)+shift)
        stage.GetRootLayer().Save()
        item = dict(id='conveyor_'+name, usd_path=path.relative_to(root).as_posix(), usd_sha256=_sha(path),
            belt_bounds_D_m=[lo, hi], bounds_D_m=[lo, hi], receiving_surface_bounds_D_m=[lo, hi],
            meshes=[dict(prim='/Conveyor/Belt', source_prim='/World/SourceBelt', collision=True,
                bounds_D_m=[lo, hi], points_sha256=hashlib.sha256(points.tobytes()).hexdigest(), faces=12)])
        items.append(item)
        reference = UsdGeom.Xform.Define(station, '/Workstation/'+item['id'])
        reference.GetPrim().GetReferences().AddReference(Path(os.path.relpath(path, station_folder)).as_posix())
    station.GetRootLayer().Save()
    T = np.eye(4)
    T[:3, 3] = TRANSLATION
    manifest = dict(native_snapshot=True, native_12carton=True, frozen_environment=True,
        source_snapshot_fingerprint='TEST_ONLY_COMPOSITION_FIXTURE_NO_PLAN',
        workstation_sha256=_sha(station_path), T_W_D=T.tolist(), cartons=boxes,
        conveyors=dict(source=dict(belt_prim='/World/SourceBelt'), items=items),
        environment=dict(warehouse_path=warehouse_path.relative_to(root).as_posix(),
            carton_path=carton_path.relative_to(root).as_posix(), floor_normalization_m=-.25,
            asset_sha256={p.relative_to(root).as_posix(): _sha(p) for p in [warehouse_path, carton_path]}))
    _json(station_folder/'workstation_manifest.json', manifest)
    return dict(root=root, assets=root/'assets', workstation=station_path,
                output=root/'composition', cartons=boxes, conveyors=items, warehouse=warehouse_path)


def _compose(fixture, monkeypatch):
    spec = importlib.util.spec_from_file_location('test_real_usd_eco65_composer', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(sys, 'argv', [str(SCRIPT), '--assets', str(fixture['assets']),
        '--workstation', str(fixture['workstation']), '--output', str(fixture['output']), '--compose-only'])
    module.main()
    return Usd.Stage.Open(str(fixture['output']/'eco65_desktop_warehouse.usda'))


def _bounds(stage, path):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy', 'guide'])
    bound = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    return np.array(bound.GetMin()), np.array(bound.GetMax())


def test_real_usd_composes_all_twelve_cartons_and_source_conveyors(tmp_path, monkeypatch):
    fixture = _fixture(tmp_path, monkeypatch)
    stage = _compose(fixture, monkeypatch)
    assert stage and not stage.GetCompositionErrors()
    report = json.loads((fixture['output']/'composition_check.json').read_text())
    assert len(report['cartons']) == 12
    assert report['environment_colliders'] == 5
    assert report['intersections'] == []
    np.testing.assert_allclose(report['translation_W_D_m'], [.32, -.10, .80], atol=1e-12, rtol=0)
    assert report['height_checks_m']['trailer_roof_inner']['status'] == 'NOT_EVALUATED_ROOF_DISABLED'
    actual_ids = {p.GetCustomDataByKey('stable_id') for p in stage.GetPrimAtPath(PREFIX+'/Cartons').GetChildren()}
    assert actual_ids == {b['id'] for b in fixture['cartons']}
    for box in fixture['cartons']:
        path = PREFIX+'/Cartons/'+box['id']
        lo, hi = _bounds(stage, path+'/Fit/Pivot/Official')
        np.testing.assert_allclose(hi-lo, [.25, .21, .10], atol=2e-6, rtol=0)
        np.testing.assert_allclose((lo+hi)/2, np.array(box['pose'])+TRANSLATION, atol=2e-6, rtol=0)
        imported = list(Usd.PrimRange(stage.GetPrimAtPath(path+'/Fit/Pivot/Official')))
        assert sum(p.IsA(UsdGeom.Mesh) for p in imported) == 1
        for prim in imported:
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                assert UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get() is False
        assert UsdPhysics.CollisionAPI(stage.GetPrimAtPath(path+'/Collision')).GetCollisionEnabledAttr().Get() is True
    target_lo, target_hi = _bounds(stage, PREFIX+'/Cartons/'+TARGET+'/Fit/Pivot/Official')
    np.testing.assert_allclose((target_lo+target_hi)/2, [.42, -.10, 1.33], atol=2e-6, rtol=0)
    for item in fixture['conveyors']:
        lo, hi = _bounds(stage, PREFIX+'/'+item['id']+'/Belt')
        np.testing.assert_allclose([lo, hi], np.array(item['belt_bounds_D_m'])+TRANSLATION, atol=2e-6, rtol=0)
        assert abs(hi[2]-.98) < 2e-6
        readback = report['belt_geometry_readback'][item['id']]
        np.testing.assert_allclose(readback['bounds_W_m'], [lo, hi], atol=1e-12, rtol=0)
        assert readback['source_sha256'] == item['usd_sha256']
        assert readback['source_mesh_count'] == 1
    warehouse_T = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/Warehouse'))).T
    np.testing.assert_allclose(warehouse_T[:3, 3], [0., 0., -.25], atol=1e-12, rtol=0)


def test_native_composition_rejects_actual_belt_height_different_from_metadata(tmp_path, monkeypatch):
    fixture = _fixture(tmp_path, monkeypatch, belt_shift=.01)
    # The source hash matches its actual bytes; only USD readback exposes that
    # a claimed 0.18 m receiving surface is physically drawn at 0.19 m.
    with pytest.raises(ValueError, match='FROZEN_BELT_GEOMETRY_MISMATCH:conveyor_longitudinal'):
        _compose(fixture, monkeypatch)


def test_native_composition_rejects_changed_frozen_official_source(tmp_path, monkeypatch):
    fixture = _fixture(tmp_path, monkeypatch)
    fixture['warehouse'].write_text(fixture['warehouse'].read_text()+'\n# source changed after binding\n')
    with pytest.raises(ValueError, match='Official cached asset changed'):
        _compose(fixture, monkeypatch)


def test_native_composition_rejects_changed_derived_conveyor_reference(tmp_path, monkeypatch):
    fixture = _fixture(tmp_path, monkeypatch)
    path = fixture['root']/fixture['conveyors'][1]['usd_path']
    path.write_text(path.read_text()+'\n# source changed after export\n')
    with pytest.raises(ValueError, match='FROZEN_CONVEYOR_ASSET_CHANGED:conveyor_longitudinal'):
        _compose(fixture, monkeypatch)
