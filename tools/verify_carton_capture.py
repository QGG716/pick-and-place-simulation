"""Check new USD carton captures without running a perception algorithm."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(capture, source_manifest):
    from pxr import Usd, UsdGeom, UsdPhysics
    original=json.loads(source_manifest.read_text())
    current=json.loads((capture/'manifest.json').read_text())
    config=json.loads((capture/'capture_configuration.json').read_text())
    for field in ('objects','cameras','robot','mechanisms','layout','world_fingerprint','dynamic_scene_fingerprint'):
        assert current[field]==original[field], f'Frozen nominal field changed: {field}'
    assert current['manifest_fingerprint']!=original['manifest_fingerprint']
    assert config['sam_run'] is False and config['planning_admissible'] is False
    assert not config['target_highlight'] and not config['mechanical_entities_omitted']
    objects={o['simulation_object_id']:o for o in current['objects']}
    assert set(config['cartons'])==set(objects)
    stage=Usd.Stage.Open(str(capture/'capture_stage.usda'))
    transforms=UsdGeom.XformCache()
    geometry={}
    for object_id,record in config['cartons'].items():
        mesh_path=record['meshes'][0]['path']
        root_path=mesh_path.split('/Visual/')[0]
        root=stage.GetPrimAtPath(root_path)
        assert root.GetTypeName()=='Xform'
        collision=stage.GetPrimAtPath(root_path+'/Collision')
        assert UsdGeom.Imageable(collision).ComputeVisibility()=='invisible'
        assert collision.HasAPI(UsdPhysics.CollisionAPI)
        matrix=np.asarray(transforms.GetLocalToWorldTransform(root),dtype=float).T
        nominal=np.asarray(objects[object_id]['T_W_object'],dtype=float)
        size=np.asarray(objects[object_id]['full_dimensions_m'])
        assert np.allclose(matrix[:3,3],nominal[:3,3],atol=1e-6,rtol=0)
        assert np.allclose(matrix[:3,:3],nominal[:3,:3]@np.diag(size),atol=1e-6,rtol=0)
        # Root-local bound includes its nominal scale; test actual mesh points in entity coordinates.
        inverse=transforms.GetLocalToWorldTransform(root).GetInverse()
        points=[]
        for mesh in record['meshes']:
            prim=stage.GetPrimAtPath(mesh['path'])
            assert prim and not any('physics' in s.lower() or 'physx' in s.lower() for s in prim.GetAppliedSchemas())
            to_root=transforms.GetLocalToWorldTransform(prim)*inverse
            points.extend([list(to_root.Transform(v)) for v in UsdGeom.Mesh(prim).GetPointsAttr().Get()])
        points=np.asarray(points)
        low,high=points.min(0),points.max(0)
        assert np.all(low>=-.50001) and np.all(high<=.50001),f'Visual outside collision: {object_id}'
        assert np.allclose(low,-.5,atol=.001) and np.allclose(high,.5,atol=.001),f'Artificial envelope shrink: {object_id}'
        geometry[object_id]={'asset_id':record['asset_id'],'root_unchanged':True,'unit_mesh_bounds':[low.tolist(),high.tolist()]}
    modules={}
    for camera in current['cameras']:
        p=capture/'FULL_STACK_NOMINAL/modules'/camera['module_id']
        info=json.loads((p/'camera_info.json').read_text())
        binding=json.loads((p/'capture_binding.json').read_text())
        assert info['K']==camera['K'] and info['T_W_C']==camera['T_W_C']
        assert [info['width'],info['height']]==camera['resolution']
        for key,name in [('rgb_sha256','sensor_rgb.png'),('metric_depth_sha256','metric_depth_m.npy'),
                         ('instance_masks_sha256','gt_instance_masks.npz'),('gt_snapshot_sha256','gt_annotations.json'),
                         ('capture_metadata_sha256','capture_metadata.json')]:
            assert binding[key]==digest(p/name),f'Capture hash mismatch: {name}'
        assert binding['simulation_epoch']==current['timing']['simulation_epoch']
        raw=np.load(p/'first_hit_instance_ids.npy')
        depth=np.load(p/'metric_depth_m.npy')
        masks=np.load(p/'gt_instance_masks.npz')
        labels=json.loads((p/'instance_segmentation_info.json').read_text())['idToLabels']
        counts={}
        union=np.zeros(raw.shape,bool)
        for object_id in masks.files:
            assert object_id in objects
            root=config['cartons'][object_id]['meshes'][0]['path'].split('/Visual/')[0]
            ids=[int(k) for k,v in labels.items() if str(v)==root or str(v).startswith(root+'/')]
            expected=np.isin(raw,ids)
            assert np.array_equal(expected,masks[object_id]),f'Multi-mesh instance loss: {object_id}'
            assert not np.any(union & expected),'Shared instance across boxes'
            assert np.all(np.isfinite(depth[expected]) & (depth[expected]>0))
            union|=expected
            counts[object_id]={'render_instance_ids':ids,'pixels':int(expected.sum())}
        assert counts
        modules[camera['module_id']]={'visible_objects':counts,'rgb_depth_instance_binding':'PASS'}
    return {'status':'BASIC_CONSISTENCY_PASS_IMAGE_REVIEW_PENDING','planning_admissible':False,
            'nominal_layout_camera_robot_unchanged':True,'cartons':geometry,'modules':modules,
            'exact_mesh_corners_and_faces':'NOT_EVALUATED'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--source-manifest',type=Path,required=True)
    args=parser.parse_args()
    result=verify(args.capture,args.source_manifest)
    (args.capture/'basic_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print(result['status'])
