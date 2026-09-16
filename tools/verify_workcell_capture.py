"""Bounded static USD checks for the roof/mast capture; no trajectory claims."""
import argparse
import json
from pathlib import Path

import numpy as np


def verify(capture, historical):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from verify_carton_capture import verify as verify_cartons
    manifest = json.loads((capture / 'manifest.json').read_text())
    old = json.loads(historical.read_text())
    assert manifest['objects'] == old['objects'], 'frozen carton entities changed'
    assert manifest['robot'] == old['robot'], 'robot mount/model/q changed'
    for key in ('components', 'tool_state', 'payload_state', 'base_state', 'conveyor_state'):
        assert manifest['mechanisms'][key] == old['mechanisms'][key], key
    for now, before in zip(manifest['cameras'], old['cameras']):
        for key in ('K', 'resolution', 'distortion', 'depth_semantics', 'registration_mode', 'publish_rate_hz'):
            assert now[key] == before[key], key
    stage = Usd.Stage.Open(str(capture / 'capture_stage.usda'))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
    xf = UsdGeom.XformCache()

    def bounds(prim):
        b = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        return np.asarray(b.GetMin()), np.asarray(b.GetMax())

    def overlaps(a, b):
        return bool(np.all(np.minimum(a[1], b[1]) - np.maximum(a[0], b[0]) > 1e-7))

    solids = {}
    for definition in manifest['mechanisms']['environment']['solids']:
        prim = next(p for p in stage.Traverse() if p.GetName().endswith('_' + definition['name']))
        assert prim.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
        assert UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible'
        a = bounds(prim)
        center, size = np.asarray(definition['center_m']), np.asarray(definition['size_xyz_m'])
        assert np.allclose(a, [center-size/2, center+size/2], atol=1e-6, rtol=0)
        solids[definition['name']] = {'path': str(prim.GetPath()), 'bounds': [v.tolist() for v in a], 'collision_enabled': True}
    roof = tuple(np.asarray(v) for v in solids['Roof']['bounds'])
    assert abs(roof[0][2] - 2.7) < 1e-6
    rig = stage.GetPrimAtPath('/PerceptionValidation/VisionRig')
    rig_bounds = bounds(rig)
    assert abs(rig_bounds[1][2] - 2.6) < 2e-6, rig_bounds
    clearance = float(roof[0][2] - rig_bounds[1][2])
    assert abs(clearance - .1) < 2e-6
    # Negative fixture uses the exact same enabled USD roof bound as the
    # clearance check, not a separately typed 2.70 constant.
    penetrator = UsdGeom.Cube.Define(stage, '/RoofPenetrationNegativeFixture')
    penetrator.CreateSizeAttr(.04)
    penetrator.AddTranslateOp().Set(Gf.Vec3d(-1.325, .75, 2.7))
    UsdPhysics.CollisionAPI.Apply(penetrator.GetPrim())
    assert overlaps(bounds(penetrator.GetPrim()), roof), 'roof penetration not detected'
    stage.RemovePrim('/RoofPenetrationNegativeFixture')
    optical = []
    tf = []
    rig_cubes = [p for p in Usd.PrimRange(rig) if p.IsA(UsdGeom.Cube)]
    for camera, expected_z in zip(manifest['cameras'], (2.4, 1.6293411964178086)):
        t = np.asarray(camera['T_W_C'])
        assert np.allclose(t[:3, 3], [-1.325, .75, expected_z], atol=1e-9, rtol=0)
        hits = []
        for p in rig_cubes:
            inv = xf.GetLocalToWorldTransform(p).GetInverse()
            origin = np.asarray(inv.Transform(Gf.Vec3d(*t[:3, 3])))
            direction = np.asarray(inv.TransformDir(Gf.Vec3d(*t[:3, 2])))
            half = float(UsdGeom.Cube(p).GetSizeAttr().Get()) / 2
            near, far = 0., .5
            for axis in range(3):
                if abs(direction[axis]) < 1e-12:
                    if abs(origin[axis]) > half: far = -1.; break
                else:
                    limits = sorted(((-half-origin[axis])/direction[axis], (half-origin[axis])/direction[axis]))
                    near, far = max(near, limits[0]), min(far, limits[1])
            if far >= near: hits.append(str(p.GetPath()))
        assert not hits, f'physical rig crosses optical ray: {hits}'
        # Replicator cameras use local -Z forward; verify an actual sensor
        # camera exists at this pose in the exported stage.
        matches = []
        for p in stage.Traverse():
            if not p.IsA(UsdGeom.Camera): continue
            m = xf.GetLocalToWorldTransform(p)
            position = np.asarray(m.ExtractTranslation())
            forward = np.asarray(m.TransformDir(Gf.Vec3d(0,0,-1)))
            forward /= np.linalg.norm(forward)
            if np.allclose(position,t[:3,3],atol=1e-5,rtol=0) and np.allclose(forward,t[:3,2],atol=1e-5,rtol=0):
                matches.append(str(p.GetPath()))
        assert matches, 'manifest camera not found in USD'
        optical.append({'module':camera['module_id'],'USD_cameras':matches,'forward_rig_hits':hits})
        tf.append({'parent':'world','child':camera['frame_id'],'T_parent_child':camera['T_W_C'],
                   'source':'SAME_CAPTURE_METADATA_USED_BY_ISAAC_SENSOR_ADAPTER'})
    config = json.loads((capture/'capture_configuration.json').read_text())
    conveyors = {}
    for name, record in config['conveyors'].items():
        mesh_bounds = [bounds(stage.GetPrimAtPath(s['mesh_path'])) for s in record['segments']]
        low, high = np.min([b[0] for b in mesh_bounds],axis=0), np.max([b[1] for b in mesh_bounds],axis=0)
        center,size = ([-.55,.35,.54],[.7,1.5,.12]) if name.endswith('transverse') else ([-1.6,-.75,.54],[2.8,.7,.12])
        center,size = np.asarray(center),np.asarray(size)
        assert np.allclose(low[:2],(center-size/2)[:2],atol=1e-5,rtol=0)
        assert np.allclose(high[:2],(center+size/2)[:2],atol=1e-5,rtol=0)
        assert abs(high[2]-.6)<1e-6 and low[2]>=.48
        collision=stage.GetPrimAtPath(record['collision_proxy'])
        assert collision.HasAPI(UsdPhysics.CollisionAPI)
        conveyors[name]={'belt_bounds':[low.tolist(),high.tolist()],'source':record['source'], 'collision_proxy_retained':True}
    # Conservative candidates are reported, never called exact contacts.
    candidates=[]
    for p in stage.Traverse():
        if str(p.GetPath()).startswith('/PerceptionValidation') or not p.IsA(UsdGeom.Mesh): continue
        a=bounds(p)
        hit=[name for name in ('Roof','LeftWall','RightWall') if overlaps(a,tuple(np.asarray(v) for v in solids[name]['bounds']))]
        if hit: candidates.append({'mesh':str(p.GetPath()),'surfaces':hit,'world_AABB':[v.tolist() for v in a]})
    result={'status':'STATIC_GEOMETRY_PASS' if not candidates else 'STATIC_RIG_PASS_OTHER_ROBOT_INTERSECTIONS_REQUIRE_REVIEW',
            'rig_world_bounds':[v.tolist() for v in rig_bounds], 'mast_roof_clearance_m':clearance,
            'roof_penetration_negative_fixture':'DETECTED_BY_ENABLED_USD_SOLID_AABB_OVERLAP',
            'solids':solids,'optical':optical,'conveyors':conveyors,
            'robot_wall_roof_AABB_candidates':candidates,'planning_admissible':False,
            'scope':'NOMINAL_STATIC_USD_ONLY_NO_TRAJECTORY_OR_PHYSICS_ACCEPTANCE'}
    (capture/'tf_at_capture.json').write_text(json.dumps(tf,indent=2))
    (capture/'workcell_checks.json').write_text(json.dumps(result,indent=2))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--historical-manifest',type=Path,required=True)
    a=p.parse_args();print(json.dumps(verify(a.capture,a.historical_manifest),indent=2))
