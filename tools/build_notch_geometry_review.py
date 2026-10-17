"""Portable geometry review/evaluation of all frozen predictions, no inference."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.review_selection import validate_review_capture
from unloading_perception.metric_support import backproject_pixels
from unloading_perception.final_geometry import SIGNS,coherent_cuboid


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,data):Path(path).write_text(json.dumps(data,indent=2,allow_nan=False),encoding='utf-8')


def acceptance_config():
    return read(ROOT/'configs/perception/metric_geometry_acceptance.json')


def review_capture(plan, visibility):
    """Use the bound input inventory, not the producer's former working directory."""
    candidates=[Path(g['capture']) for g in plan['groups'] if g['scene']==visibility['scene']]
    if len(candidates)!=1:raise ValueError('REVIEW_CAPTURE_SCENE_NOT_UNIQUE')
    return candidates[0]


def box_errors(corners,T,vertices):
    from scipy.optimize import linear_sum_assignment
    world=np.asarray(corners)@T[:3,:3].T+T[:3,3]
    center,axes,dimensions,_=coherent_cuboid(world)
    low,high=vertices.min(0),vertices.max(0);gt=low+(SIGNS+1)/2*(high-low)
    distance=np.linalg.norm(world[:,None]-gt[None],axis=-1);a,b=linear_sum_assignment(distance)
    angles=[]
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1,1),repeat=3):
            rotation=axes[list(permutation)]*np.array(signs)[:,None]
            if np.linalg.det(rotation)>0:
                angles.append(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2,-1,1))))
    result=dict(reference='actual composed mesh bounds; ideal cuboid approximation, not nominal config',
        center_error_m=float(np.linalg.norm(center-(low+high)/2)),
        sorted_dimension_error_m=abs(np.sort(dimensions)-np.sort(high-low)).tolist(),
        orientation_symmetry_error_degrees=float(min(angles)),corner_rms_m=float(np.sqrt(np.mean(distance[a,b]**2))),
        corner_max_m=float(distance[a,b].max()),symmetry='all 24 proper signed-axis permutations and one-to-one corner assignment')
    thresholds=acceptance_config()
    result['accuracy_checks'] = dict(center=result['center_error_m']<=thresholds['gt_complete_center_m'],
        dimensions=max(result['sorted_dimension_error_m'])<=thresholds['gt_complete_dimension_m'],
        orientation=result['orientation_symmetry_error_degrees']<=thresholds['gt_complete_orientation_degrees'],
        corners=result['corner_max_m']<=thresholds['gt_certified_corner_m'])
    result['external_accuracy_pass']=all(result['accuracy_checks'].values())
    result['threshold_source']='configs/perception/metric_geometry_acceptance.json; unchanged'
    return result


def evaluate_planes(record,visible,T):
    from scipy.optimize import linear_sum_assignment
    truth=[f for f in visible['surfaces'] if f['visible']];faces=record['camera_facing_faces'];metrics=[]
    costs=np.empty((len(faces),len(truth)))
    pairs={}
    for i,face in enumerate(faces):
        points=np.asarray(face['corners_3d_m'])@T[:3,:3].T+T[:3,3]
        n=np.cross(points[1]-points[0],points[3]-points[0]);n/=np.linalg.norm(n)
        for j,gt in enumerate(truth):
            gn=np.array(gt['normal']);angle=float(np.degrees(np.arccos(np.clip(abs(n@gn),0,1))))
            position=float(abs(points.mean(0)@gn+gt['offset_m']))
            pairs[i,j]=dict(support_label=face['support_label'],gt_surface=[gt['axis'],gt['sign']],
                normal_error_degrees=angle,position_error_m=position,
                predicted_mean_depth_residual_m=face['final_support']['plane_residual_m'],
                predicted_p95_depth_residual_m=face['final_support']['plane_residual_p95_m'],
                mesh_plane_mean_m=gt['mesh_plane_mean_m'],mesh_plane_max_m=gt['mesh_plane_max_m'])
            costs[i,j]=angle/10+position/.006
    a,b=linear_sum_assignment(costs);used=set()
    for i,j in zip(a,b):
        item=pairs[i,j];item['corresponding_surface']=item['normal_error_degrees']<=10 and item['position_error_m']<=.006
        thresholds=acceptance_config()
        item['accuracy_pass']=(item['normal_error_degrees']<=thresholds['gt_normal_degrees']
                               and item['position_error_m']<=thresholds['gt_plane_position_m'])
        if item['corresponding_surface']:used.add(j)
        metrics.append(item)
    return dict(expected_reliable_visible_faces=len(truth),extracted_reliable_planes=record.get('reliable_plane_count'),
        matched_planes=metrics,unmatched_gt_surfaces=[[g['axis'],g['sign']] for j,g in enumerate(truth) if j not in used],
        unmatched_predicted_support_labels=[f['support_label'] for i,f in enumerate(faces) if i not in set(a)],
        policy='one-to-one plane assignment after one-to-one mask matching; 1 degree / 5 mm accuracy gates unchanged')


def build(baseline,final,visibility,output,before_geometry=None):
    from scipy.optimize import linear_sum_assignment
    baseline,final,output=map(Path,(baseline,final,output));output.mkdir(parents=True,exist_ok=False)
    plan=read(baseline/'frozen_plan.json');policy=plan['policy'];visibility=read(visibility)
    visible={(r['module'],r['simulation_object_id']):r for r in visibility['all_objects']}
    capture=review_capture(plan,visibility);mesh=read(capture/'rendered_surface_truth.json')
    if mesh['sha256']!=visibility['mesh_truth_sha256']:raise ValueError('VISIBILITY_MESH_BINDING_MISMATCH')
    meshpath=capture/mesh['mesh_file']
    if hashlib.sha256(meshpath.read_bytes()).hexdigest()!=mesh['sha256']:raise ValueError('MESH_TRUTH_HASH_MISMATCH')
    with np.load(meshpath) as arrays:vertices={o['simulation_object_id']:arrays[o['array_prefix']+'_vertices_world_m'] for o in mesh['objects']}
    pages=[];table=[];evaluations=[]
    for group in plan['groups']:
        name=group['name'];summary=read(baseline/name/'summary.json');manifest=read(Path(group['capture'])/'manifest.json')
        for camera in manifest['cameras']:
            module=camera['module_id'];source=baseline/name/module;geometry=final/name/module
            meta=read(source/'capture_metadata.json');validate_review_capture(meta,module,policy)
            target=output/name/module;target.mkdir(parents=True)
            for filename in ('sensor_rgb.png','detector_boxes.png','sam_boundaries.png','sam_review.html','sam_review_data.js','sam_review.json','detector_proposals.json'):
                shutil.copy2(source/filename,target/filename)
            shutil.copytree(source/'instances',target/'instances')
            review=read(source/'sam_review.json');records=read(geometry/'rgbd_cuboids.json')['instances']
            before_source=Path(before_geometry)/name/module if before_geometry is not None else source
            old={r['mask_id']:r for r in read(before_source/'rgbd_cuboids.json')['instances']}
            response=read(source/'mode_b1_worker_response.json');metrics=read(response['metrics_reference']['path'])
            # Cargo mask path is declared in the worker metrics, checked by the
            # actual runtime before this read-only post-freeze review.
            from run_isaac_rgbd_geometry import _worker_artifacts
            from unloading_perception.isaac_payload import load_capture_payload
            from unloading_perception.isaac_validation import IsaacSceneManifest
            payload=load_capture_payload(source,IsaacSceneManifest.from_dict(manifest),expected_module_id=module,with_instance_masks=True)
            artifacts=_worker_artifacts(source,payload=payload)
            with np.load(artifacts['cargo_masks.npz']['path']) as archive:
                masks={int(i):m.astype(bool) for i,m in zip(archive['mask_ids'],archive['masks'])}
            truth={k:v for k,v in payload.instance_masks.items() if v.any()};gtids=list(truth);mids=list(masks)
            ious=np.array([[np.count_nonzero(masks[mid]&truth[g])/max(1,np.count_nonzero(masks[mid]|truth[g])) for g in gtids] for mid in mids])
            a,b=linear_sum_assignment(-ious);matching={mids[i]:(gtids[j],float(ious[i,j])) for i,j in zip(a,b) if ious[i,j]>=.5}
            depth=payload.depth;T=np.array(camera['T_W_C']);K=np.array(camera['K']).reshape(3,3)
            grey=np.zeros(depth.shape,np.uint8);valid=np.isfinite(depth)&(depth>0);grey[valid]=(255*np.clip((depth[valid]-.5)/4,0,1)).astype(np.uint8)
            Image.fromarray(grey).save(target/'depth.png');instance_data=[]
            for record in records:
                mid=record['mask_id'];match=matching.get(mid);initial=read(geometry/f'v4_validation/stage_trace/mask-{mid}-initial.json')
                with np.load(geometry/f'v4_validation/stage_trace/mask-{mid}-initial.npz') as archive:labels=archive['labels']
                initial_cloud=[]
                for label in np.unique(labels[labels>=0]):
                    y,x=np.nonzero(labels==label);stride=max(1,len(x)//500)
                    xy=np.c_[x[::stride],y[::stride]];points=backproject_pixels(xy,depth[y[::stride],x[::stride]],K)
                    initial_cloud.append(dict(label=int(label),points=points.tolist(),pixels=xy.tolist(),pixel_count=len(x)))
                associated=[p for p in review['proposals'] if p['mask_id']==mid]
                evaluation=dict(mask_id=mid,matched_gt=None if match is None else match[0],mask_iou=None if match is None else match[1],
                    status='UNMATCHED_PREDICTION' if match is None else 'MATCHED',planes='NOT_RUN_NO_COMPOSED_MESH_TRUTH',complete_box='NOT_APPLICABLE')
                if match and name=='stack_notch':
                    evaluation['planes']=evaluate_planes(record,visible[module,match[0]],T)
                    if record['accepted']:evaluation['complete_box']=box_errors(record['corners_3d'],T,vertices[match[0]])
                oldrecord=old[mid];before=dict(accepted=oldrecord['accepted'],patch_count=len(oldrecord['camera_facing_faces']),
                    rejected_boundaries=oldrecord.get('completion_diagnostics',{}).get('rejected_boundaries',[]))
                row=dict(group=name,module=module,mask_id=mid,proposal_ids=[p['proposal_id'] for p in associated],
                    matched_gt=evaluation['matched_gt'],mask_iou=evaluation['mask_iou'],
                    initial_components=len(initial['planes']),final_patches=len(record['camera_facing_faces']),
                    reliable_planes=record['reliable_plane_count'],source_ambiguous=record['source_ambiguous'],
                    cuboid_status=record['cuboid_status'],accepted=record['accepted'],baseline_accepted=oldrecord['accepted'],
                    complete_gt_accuracy_pass=evaluation['complete_box'].get('external_accuracy_pass') if isinstance(evaluation['complete_box'],dict) else None,
                    expected_visible_faces=evaluation['planes'].get('expected_reliable_visible_faces') if isinstance(evaluation['planes'],dict) else None)
                table.append(row);evaluations.append(dict(**row,evaluation=evaluation))
                instance_data.append(dict(**row,record=record,initial=initial,initial_cloud=initial_cloud,
                    proposals=associated,evaluation=evaluation,before=before,before_record=oldrecord,
                    gt_contours=next((g['contours'] for g in review.get('evaluation',{}).get('gt_objects',[]) if match and g['id']==match[0]),[])))
            matched_gt={m[0] for m in matching.values()}
            unmatched=[g for g in gtids if g not in matched_gt]
            missing=[p for p in review['proposals'] if p['mask_id'] is None]
            data=dict(name=name,module=module,capture_id=meta['capture_id'],source_size=review['source_size'],
                instances=instance_data,unmatched_gt=unmatched,proposals_without_mask=missing,
                stages={s:'NOT_RUN' for s in ('contacts','fusion','ros','planning','execution')},
                rgb='sensor_rgb.png',depth='depth.png',policy=policy,detector_summary=review['evaluation']['summary'])
            write(target/'geometry_review.json',data)
            (target/'geometry_review_data.js').write_text('window.GEOMETRY='+json.dumps(data,allow_nan=False).replace('</','<\\/')+';',encoding='utf-8')
            shutil.copy2(ROOT/'tools/notch_geometry_review.html',target/'index.html')
            shutil.copy2(ROOT/'tools/geometry_bounds_review.js',target/'geometry_bounds_review.js')
            pages.append(dict(name=name,module=module,href=f'{name}/{module}/index.html',masks=len(records),
                proposals=len(review['proposals']),complete=sum(r['accepted'] for r in records),unmatched_gt=unmatched))
            pages[-1]['complete_mesh_accuracy_pass']=sum(r['complete_gt_accuracy_pass'] is True for r in instance_data)
    write(output/'all_instances.json',evaluations)
    with (output/'all_instances.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(table[0]));writer.writeheader();writer.writerows(table)
    write(output/'review_manifest.json',dict(pages=pages,policy=policy,all_instances=len(table),mesh_truth_sha256=mesh['sha256']))
    links=''.join(f'<tr><td>{p["name"]}</td><td><a href="{p["href"]}">{p["module"]}</a></td><td>{p["proposals"]}</td><td>{p["masks"]}</td><td>{p["complete"]}</td><td>{len(p["unmatched_gt"])}</td></tr>' for p in pages)
    shutil.copy2(capture/'overview.png',output/'notch-panorama.png')
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>箱堆豁口几何评审</title><style>body{font:16px system-ui;margin:32px;background:#111827;color:#eef}a{color:#8cf}td,th{padding:12px;border-bottom:1px solid #456}img{max-width:100%;max-height:550px}</style><h1>箱堆豁口 · 几何评审</h1><p>DINO / SAM 冻结；真实采集，全部实例。绿色面片不等于完整箱体。</p><p>吸附、融合、ROS、规划、执行：NOT_RUN</p><table><tr><th>场景</th><th>模组 / 进入逐实例评审</th><th>检测</th><th>SAM</th><th>完整体</th><th>未匹配 GT</th></tr>'+links+'</table><p><a href="all_instances.csv">全部实例 CSV</a> · <a href="all_instances.json">独立评价 JSON</a> · <a href="review_manifest.json">冻结清单</a></p><img src="notch-panorama.png"><details><summary>历史诊断说明</summary>历史报告和采集原样保留在仓库归档；不计入本轮统计。</details>',encoding='utf-8')
    return pages


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--final',type=Path,required=True)
    p.add_argument('--visibility',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(json.dumps(build(a.baseline,a.final,a.visibility,a.output),indent=2))
