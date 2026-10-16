"""Pre-inference scene review against composed render meshes, never a solver input."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.metric_support import backproject_pixels
from unloading_perception.face_quality import support_quality


def read(p): return json.loads(Path(p).read_text(encoding='utf-8'))


def mesh_surfaces(vertices, triangles, degrees=10):
    """Cluster actual mesh triangles around six dominant world directions.

    This scene has unrotated retained cartons. Plane normals/offsets are fitted
    from their actual triangles; nominal sizes are never used as surface truth.
    Non-axis mesh details remain an explicit model discrepancy.
    """
    p = vertices[triangles]; crosses = np.cross(p[:,1]-p[:,0], p[:,2]-p[:,0])
    area = np.linalg.norm(crosses, axis=1)/2
    normals = crosses/np.maximum(2*area[:,None], 1e-30)
    groups = np.argmax(abs(normals), axis=1); result=[]
    for axis in range(3):
        for sign in (-1, 1):
            chosen = (groups==axis)&(normals[:,axis]*sign>np.cos(np.radians(degrees)))&(area>1e-12)
            if not chosen.any(): continue
            points=p[chosen].reshape(-1,3); weights=np.repeat(area[chosen]/3,3)
            center=np.average(points,axis=0,weights=weights)
            covariance=((points-center)*weights[:,None]).T@(points-center)/weights.sum()
            _, vectors=np.linalg.eigh(covariance); n=vectors[:,0]
            if n[axis]*sign<0:n=-n
            errors=abs((points-center)@n)
            result.append(dict(axis=axis,sign=sign,normal=n.tolist(),offset_m=float(-n@center),
                center_world_m=center.tolist(),mesh_area_m2=float(area[chosen].sum()),
                mesh_plane_mean_m=float(np.average(errors,weights=weights)),
                mesh_plane_max_m=float(errors.max()),vertex_bounds=[points.min(0).tolist(),points.max(0).tolist()]))
    return result


def check(capture, scene, output, policy):
    capture,output=Path(capture),Path(output);output.mkdir(parents=True,exist_ok=True)
    reference=read(capture/'rendered_surface_truth.json')
    raw=(capture/reference['mesh_file']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=reference['sha256']:raise ValueError('RENDER_MESH_HASH_MISMATCH')
    manifest=read(capture/'manifest.json'); cfg=policy['visibility_check']; result=[]
    from io import BytesIO
    with np.load(BytesIO(raw)) as meshes:
        surfaces={o['simulation_object_id']:mesh_surfaces(meshes[o['array_prefix']+'_vertices_world_m'],
            meshes[o['array_prefix']+'_triangles'],cfg['mesh_normal_cluster_degrees']) for o in reference['objects']}
    from unloading_perception.isaac_payload import load_capture_payload
    from unloading_perception.isaac_validation import IsaacSceneManifest
    verified_manifest=IsaacSceneManifest.from_dict(manifest)
    for camera in manifest['cameras']:
        module=camera['module_id'];folder=capture/scene/'modules'/module
        payload=load_capture_payload(folder,verified_manifest,expected_module_id=module,with_instance_masks=True)
        depth=payload.depth;K=np.asarray(camera['K']).reshape(3,3);T=np.asarray(camera['T_W_C'])
        for identity,mask in payload.instance_masks.items():
            if identity not in surfaces:continue
            valid=mask&np.isfinite(depth)&(depth>0);y,x=np.nonzero(valid)
            points=backproject_pixels(np.c_[x,y],depth[valid],K)@T[:3,:3].T+T[:3,3]
            neighbours=[]; normal_valid=np.ones(len(x),bool)
            for dx,dy in ((-3,0),(3,0),(0,-3),(0,3)):
                nx=np.clip(x+dx,0,depth.shape[1]-1);ny=np.clip(y+dy,0,depth.shape[0]-1)
                z=depth[ny,nx];ok=np.isfinite(z)&(z>0)&mask[ny,nx]
                normal_valid &= ok
                neighbours.append(backproject_pixels(np.c_[nx,ny],np.where(ok,z,0),K))
            normals=np.cross(neighbours[1]-neighbours[0],neighbours[3]-neighbours[2])@T[:3,:3].T
            length=np.linalg.norm(normals,axis=1);normal_valid &= length>1e-12
            normals/=np.maximum(length[:,None],1e-12)
            faces=[]
            for face in surfaces[identity]:
                n=np.array(face['normal']);d=face['offset_m']
                chosen=(abs(points@n+d)<=cfg['surface_distance_m'])&normal_valid&(abs(normals@n)>np.cos(np.radians(10)))
                support=points[chosen];q=support_quality(support,
                    minimum_short_m=cfg['minimum_short_span_m'],minimum_area_m2=cfg['minimum_area_m2'])
                visible=(len(support)>=cfg['minimum_pixels'] and q['geometry_eligible']
                    and n@(T[:3,3]-np.array(face['center_world_m']))>0
                    and face['mesh_plane_mean_m']<=.003)
                faces.append(dict(**face,visible=bool(visible),pixel_count=len(support),quality=q))
            result.append(dict(module=module,simulation_object_id=identity,visible_pixels=len(points),
                reliable_visible_face_count=sum(f['visible'] for f in faces),surfaces=faces))
        # Display only; original metric_depth_m.npy remains unchanged.
        from PIL import Image
        valid=np.isfinite(depth)&(depth>0);grey=np.zeros(depth.shape,np.uint8)
        grey[valid]=(255*np.clip((depth[valid]-.5)/4,0,1)).astype(np.uint8)
        Image.fromarray(grey).save(output/(module+'-depth.png'))
    counts=[r['reliable_visible_face_count'] for r in result]
    report=dict(stage='PRE_INFERENCE_VISIBILITY',prediction_run=False,policy=policy,
        mesh_truth_sha256=reference['sha256'],capture=str(capture),scene=scene,
        all_objects=result,has_single=1 in counts,has_double=2 in counts,has_triple=3 in counts,
        solver_gt_access=False,depth_display='0.5 to 4.5 m fixed grayscale; invalid black')
    (output/'visibility.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('has_single','has_double','has_triple')}))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--scene',default='STACK_NOTCH_MULTIFACE');p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policy',type=Path,default=ROOT/'configs/perception/notch_geometry_review.json');a=p.parse_args()
    check(a.capture,a.scene,a.output,read(a.policy))
