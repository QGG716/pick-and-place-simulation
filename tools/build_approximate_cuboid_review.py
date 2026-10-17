"""Post-solve evaluation only; never imported by the geometry solver."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image,ImageDraw
from build_notch_geometry_review import ROOT,build,read,write,box_errors,review_capture,acceptance_config
from build_physical_bounds_review import boundary_comparison,render_bound_images
from unloading_perception.metric_support import backproject_pixels


def comparison(record,before,intermediate,T,vertices):
    result=boundary_comparison(record,before,T,vertices)
    model=record.get('approximate_cuboid',{})
    if not result or model.get('status')!='VALIDATED_OBSERVATION_MODEL':return result
    result.update(approximate_model=True,current_box=box_errors(model['corners_3d'],T,vertices),rejected_diagnostic_box=None,
        intermediate_box=box_errors(intermediate['approximate_cuboid']['corners_3d'],T,vertices)
            if intermediate.get('approximate_cuboid',{}).get('corners_3d') else None)
    for bound in result['bounds']:
        measured=next(b for b in model['bounds'] if (b['axis'],b['side'])==(bound['axis'],bound['side']))
        value=measured['value_m']
        bound.update(final_m=value,final_signed_error_m=value-bound['reference_m'],
            constraint_status=measured['interpretation'],constraint_sources=measured['sources'],
            uncertainty_interval_m=measured.get('parameter_stability_interval_m'),
            interval_semantics='algorithmic stability only, not a calibrated physical confidence interval',
            intermediate_m=intermediate.get('approximate_cuboid',{}).get('bounds_'+bound['side']+'_m',[None]*3)[bound['axis']])
    for axis,dimension in enumerate(result['dimensions']):
        value=model['shape_dimensions'][axis]
        dimension.update(final_m=value,final_signed_error_m=value-dimension['reference_m'])
    return result


def static_model_views(folder,instance):
    model=instance['record']['approximate_cuboid'];proposal=instance['proposals'][0]
    box=proposal.get('actual_prompt',{}).get('base_bbox') or proposal['bbox']
    rgb=Image.open(folder/'sensor_rgb.png').convert('RGB');crop=(max(0,int(box[0])-35),max(0,int(box[1])-35),min(rgb.width,int(box[2])+36),min(rgb.height,int(box[3])+36))
    rgb.crop(crop).save(folder/'primary-raw.png')
    edges=((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7))
    draw=ImageDraw.Draw(rgb);points=model['corners_2d']
    for a,b in edges:draw.line([tuple(points[a]),tuple(points[b])],fill='#43ddff',width=2)
    rgb.crop(crop).save(folder/'primary-approximate.png')
    cloud=np.asarray(instance['raw_cloud']['points']);corners=np.asarray(model['corners_3d']);all_points=np.r_[cloud,corners]
    center=(all_points.min(0)+all_points.max(0))/2;extent=np.ptp(all_points,axis=0).max()
    yaw,pitch=-.5,.25
    def pixels(points):
        p=points-center;x,y,z=p.T;u=np.cos(yaw)*x+np.sin(yaw)*z;w=-np.sin(yaw)*x+np.cos(yaw)*z
        return np.c_[400+500/extent*u,310+500/extent*(np.cos(pitch)*y-np.sin(pitch)*w)]
    image=Image.new('RGB',(800,620),'#101822');d=ImageDraw.Draw(image)
    for x,y in pixels(cloud):d.ellipse((x-1,y-1,x+1,y+1),fill='#73bd9d')
    corners2=pixels(corners)
    for a,b in edges:d.line([tuple(corners2[a]),tuple(corners2[b])],fill='#43ddff',width=2)
    d.text((20,20),'Approximate cuboid / NOT a collision envelope',fill='white')
    d.text((20,40),f'Equal metric XYZ scale; view extent {extent:.3f} m',fill='white')
    image.save(folder/'primary-model-cloud.png')


def main():
    p=argparse.ArgumentParser()
    for key in ('frozen','strict','intermediate','final','visibility','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--selection',type=Path,default=ROOT/'configs/perception/approximate_cuboid_review.json')
    args=p.parse_args();selection=read(args.selection);primary=selection['primary']
    source=args.frozen/primary['scene']/primary['module']
    for name,key in [('sensor_rgb.png','rgb_sha256'),('metric_depth_m.npy','depth_sha256')]:
        if hashlib.sha256((source/name).read_bytes()).hexdigest()!=primary[key]:raise ValueError('PRIMARY_CAPTURE_HASH_MISMATCH')
    if read(source/'capture_metadata.json')['capture_id']!=primary['capture_id']:raise ValueError('PRIMARY_CAPTURE_ID_MISMATCH')
    pages=build(args.frozen,args.final,args.visibility,args.output,before_geometry=args.strict)
    capture=review_capture(read(args.frozen/'frozen_plan.json'),read(args.visibility));mesh=read(capture/'rendered_surface_truth.json')
    with np.load(capture/mesh['mesh_file']) as data:vertices={o['simulation_object_id']:data[o['array_prefix']+'_vertices_world_m'] for o in mesh['objects']}
    rows=[];bounds=[];primary_result=None
    for page in pages:
        folder=args.output/page['name']/page['module'];data=read(folder/'geometry_review.json')
        camera=read(args.frozen/page['name']/page['module']/'camera_info.json');T=np.asarray(camera['T_W_C']);K=np.asarray(camera['K']).reshape(3,3)
        depth=np.load(args.frozen/page['name']/page['module']/'metric_depth_m.npy')
        intermediate={r['mask_id']:r for r in read(args.intermediate/page['name']/page['module']/'rgbd_cuboids.json')['instances']}
        for instance in data['instances']:
            record=instance['record'];mid=instance['mask_id'];model=record.get('approximate_cuboid',{})
            identity=dict(group=page['name'],module=page['module'],mask_id=mid,proposal_ids=instance['proposal_ids'])
            c=None
            if page['name']=='stack_notch' and instance['matched_gt']:
                c=comparison(record,instance['before_record'],intermediate[mid],T,vertices[instance['matched_gt']])
                instance['boundary_comparison']=c
                if c:bounds.extend(dict(**identity,**b) for b in c['bounds'])
            if model:
                # Actual raw depth, including transition pixels; display sampling
                # only, not the frozen fitting label or a generated surface.
                sam=next((args.frozen/page['name']/'sam-runs').glob('*'+page['module']+'/cargo_masks.npz'))
                with np.load(sam) as archive:mask=archive['masks'][list(archive['mask_ids']).index(mid)]
                y,x=np.nonzero(mask & np.isfinite(depth) & (depth>0));ids=np.linspace(0,len(x)-1,min(4000,len(x)),dtype=int)
                xy=np.c_[x[ids],y[ids]];points=backproject_pixels(xy,depth[y[ids],x[ids]],K)
                instance['raw_cloud']=dict(points=points.tolist(),pixels=xy.tolist(),total_raw_pixels=len(x),usage='DISPLAY_ONLY_DETERMINISTIC_SAMPLE')
            errors=c.get('current_box') if c and c.get('approximate_model') else None
            rows.append(dict(**identity,reliable_planes=record['reliable_plane_count'],source_ambiguous=record['source_ambiguous'],
                strict_status=record['cuboid_status'],strict_accepted=record['accepted'],
                approximate_status=model.get('status','NOT_ATTEMPTED'),model_stability=model.get('stability',{}).get('status'),
                external_accuracy=None if not errors else errors['external_accuracy_pass'],
                center_error_m=None if not errors else errors['center_error_m'],corner_max_m=None if not errors else errors['corner_max_m'],
                execution_eligible=False))
            if (page['name'],page['module'],mid)==(primary['scene'],primary['module'],primary['mask_id']):
                if instance['proposal_ids']!=[primary['proposal_id']]:raise ValueError('PRIMARY_PROPOSAL_MASK_MAPPING_MISMATCH')
                primary_result=dict(identity=identity,comparison=c,model=model,plane_evaluation=instance['evaluation']['planes'])
                render_bound_images(folder,instance)
                if model.get('status')=='VALIDATED_OBSERVATION_MODEL':static_model_views(folder,instance)
        write(folder/'geometry_review.json',data)
        (folder/'geometry_review_data.js').write_text('window.GEOMETRY='+json.dumps(data,allow_nan=False).replace('</',r'<\/')+';',encoding='utf8')
    if primary_result is None:raise ValueError('PRIMARY_INSTANCE_MISSING')
    write(args.output/'primary_result.json',primary_result);write(args.output/'regression.json',rows);write(args.output/'six_boundaries.json',bounds)
    for name,values in [('regression',rows),('six_boundaries',bounds)]:
        with (args.output/(name+'.csv')).open('w',newline='',encoding='utf-8-sig') as stream:
            fields=list(dict.fromkeys(key for row in values for key in row))
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(values)
    summary=dict(all_predictions=len(rows),approximate_generated=sum(r['approximate_status']=='VALIDATED_OBSERVATION_MODEL' for r in rows),
        approximate_accuracy_pass=sum(r['external_accuracy'] is True for r in rows),selection=selection,
        accuracy_thresholds=acceptance_config(),mesh_truth_sha256=mesh['sha256'],
        detector_sam_capture='NOT_RUN_FROZEN',later_stages='NOT_RUN',execution_eligible=False)
    write(args.output/'summary.json',summary)
    prefix=primary['scene']+'/'+primary['module']+'/'
    links=''.join(f'<li><a href="{v["href"]}">{v["name"]} / {v["module"]} · {v["masks"]} 实例</a></li>' for v in pages)
    (args.output/'index.html').write_text('''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>三面纸箱近似恢复</title><style>body{font:17px system-ui;margin:32px;background:#101822;color:#eef;max-width:1200px}a{color:#8df}img{max-width:31%;vertical-align:top;border:1px solid #456}p{line-height:1.6}.card{padding:20px;background:#203347;border-radius:12px}</style><h1>三面纸箱：有观测依据的近似长方体</h1><p>主对象 mask 6 / proposal 7。近似模型与测量平面分开；虚拟角点不等于实测尖角，非保守碰撞包络，不取得规划或执行资格。</p><div class="card"><a href="'''+prefix+'''index.html?mask='''+str(primary['mask_id'])+'''"><h2>打开逐实例模型、有限边界与点云</h2><img src="'''+prefix+'''primary-raw.png" alt="原始 RGB"><img src="'''+prefix+'''primary-approximate.png" alt="青色近似模型"><img src="'''+prefix+'''primary-model-cloud.png" alt="等米制模型与原始点云"></a></div><p>GT 默认关闭；尖角数值位于折叠诊断。模型生成、独立精度、执行资格分开记录。</p><p><a href="primary_result.json">三路对照与完整模型</a> · <a href="six_boundaries.csv">六边界来源和误差</a> · <a href="regression.csv">全部实例回归</a></p><ul>'''+links+'''</ul><p>DINO / SAM / 采集：本轮不运行，复用冻结输入。接触、吸附、融合、ROS、规划、执行：NOT_RUN。</p></html>''',encoding='utf8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
