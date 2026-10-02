"""Portable read-only stage viewer built only from pinned algorithm artifacts.

No fitting, inference, contact generation or GT geometry is performed here.
"""
import argparse
from collections import Counter
import hashlib
from io import BytesIO
import json
import math
from pathlib import Path
import shutil
import sys
from time import perf_counter
import numpy as np
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_contracts import loads, canonical_fingerprint, to_wire
from unloading_perception.algorithm_artifact import load_algorithm_artifact, _read_reference
from unloading_perception.finite_sequence import atomic_json
from unloading_perception.metric_support import backproject_pixels
from derive_surface_contacts import module_inputs

PALETTE=np.array([[83,197,246],[245,175,76],[184,124,255],[69,214,161],[245,112,151],[211,229,90]],np.uint8)
FILTER_COLORS=np.array([[69,214,161],[30,38,49],[255,75,167],[245,175,76],[184,124,255],[235,85,71]],np.uint8)


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def checked_json(ref, root):
    path=(root/ref['path']).resolve()
    if not path.is_relative_to(root.resolve()):raise ValueError('TRACE_PATH_ESCAPE')
    if hashlib.sha256(path.read_bytes()).hexdigest()!=ref['sha256']:raise ValueError('TRACE_HASH_MISMATCH')
    return read(path)


def validate_trace(folder, ids, capture, required, *, rgb_sha256=None, masks_sha256=None, calibration=None):
    if not (folder/'index.json').exists():
        if not ids:
            return dict(status='NOT_RUN_EMPTY_SEGMENTATION',mask_ids=[],events=[])
        if required:raise ValueError('NEW_RUN_MISSING_INITIAL_TRACE')
        return None
    trace=read(folder/'index.json')
    if ((rgb_sha256 is not None and trace['source']['sha256']!=rgb_sha256)
            or (masks_sha256 is not None and trace['masks']['sha256']!=masks_sha256)
            or (calibration is not None and trace['calibration_identity']!=calibration)):
        raise ValueError('TRACE_INPUT_BINDING_MISMATCH')
    if trace['status']!='COMPLETED' or trace['mask_ids']!=ids or trace['capture_id']!=capture:
        raise ValueError('TRACE_IDENTITY_OR_ROSTER_MISMATCH')
    events=trace['events']
    if [(e['mask_id'],e['stage']) for e in events]!=[(i,s) for i in ids for s in ('initial_planes','fitted_and_independently_validated')]:
        raise ValueError('TRACE_INCOMPLETE_EVENTS')
    for event in events:
        d=checked_json(event['result'],folder)
        if d['mask_id']!=event['mask_id']:raise ValueError('TRACE_INSTANCE_MISMATCH')
        if 'labels' in d:
            ref=d['labels'];path=folder/ref['path']
            if path.name!=ref['path'] or hashlib.sha256(path.read_bytes()).hexdigest()!=ref['sha256']:
                raise ValueError('TRACE_LABEL_HASH_MISMATCH')
    return trace


def crop_bounds(mask):
    y,x=np.nonzero(mask)
    if not len(x):return (0,0,mask.shape[1],mask.shape[0])
    return (max(0,int(x.min())-30),max(0,int(y.min())-30),min(mask.shape[1],int(x.max())+31),min(mask.shape[0],int(y.max())+31))


def blend(rgb,selected,color):
    out=rgb.copy();out[selected]=(out[selected]*.35+np.asarray(color)*.65).astype(np.uint8);return out


def sampled_points(depth,mask,K,maximum=2000):
    y,x=np.nonzero(mask & np.isfinite(depth) & (depth>0));step=max(1,math.ceil(len(x)/maximum))
    xyz=backproject_pixels(np.column_stack((x[::step],y[::step])),depth[y[::step],x[::step]],np.asarray(K).reshape(3,3))
    return dict(points=xyz.round(6).tolist(),total=len(x),stride=step,
                display_only='deterministic stride, coordinates rounded to 1 um for display only; algorithm data unchanged')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--contacts',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--origin',choices=['HISTORICAL_REVIEW','NEW_ALGORITHM_RUN'],required=True)
    p.add_argument('--ros',type=Path)
    a=p.parse_args(argv);started=perf_counter()
    plan=read(a.plan);contacts=read(a.contacts/'summary.json');frozen=read(a.contacts/'frozen_plan.json')
    if plan['groups']!=frozen['plan']['groups']:raise ValueError('CONTACT_GROUP_PLAN_MISMATCH')
    if contacts['status']!='COMPLETED_OFFLINE_ADAPTATION':raise ValueError('CONTACT_ADAPTATION_INCOMPLETE')
    if len(contacts['groups'])!=len(plan['groups']):raise ValueError('CONTACT_GROUP_ROSTER_MISMATCH')
    a.output.mkdir(parents=True,exist_ok=False)
    assets=a.output/'assets';assets.mkdir();groups=[]
    data=dict(schema='perception_stage_viewer_v1',origin=a.origin,groups=groups,tool=frozen['tool'],
        policy='仿真采集、真实算法运行 / ORACLE_PROPOSAL / raw_image_automatic=false / READ ONLY',
        planning_geometry={'status':'NOT_RUN','reason':'本次未显式启用尺寸先验实验；不以包络面代替接触面'},
        moge={'status':'NOT_RUN','reason':'当前 RGB-D 主链未调用 MoGe'},enable_hardware=False,
        legend={'supported':'#45d6a1','rejected':'#eb5547','unknown':'#f5af4c','not_run':'#96a3b7'})
    data['contacts_timing']={k:contacts[k] for k in ('adapter_wall_seconds','adapter_cpu_seconds','timing_scope')}
    atomic_json(a.output/'build-status.json',dict(status='RUNNING',groups=[]))
    for group,cg in zip(plan['groups'],contacts['groups']):
        if cg['group']!=group:raise ValueError('CONTACT_GROUP_IDENTITY_MISMATCH')
        observation,index=load_algorithm_artifact(group['artifact']['path'],group['artifact']['sha256'])
        if cg['original_observation_fingerprint']!=canonical_fingerprint(observation):raise ValueError('CONTACT_OBSERVATION_MISMATCH')
        root=Path(group['artifact']['path']).parent;name=group['name'];base=assets/name;base.mkdir()
        run_summary=read(root/'summary.json')
        fused={c.source_instance_id:c for c in observation.cargo}
        if set(fused)!={r['object_id'] for r in cg['objects']}:raise ValueError('CONTACT_OBJECT_ROSTER_MISMATCH')
        gd=dict(name=name,run_id=index['run_id'],source_sequence=observation.source_sequence,capture_time=observation.capture_time,
            artifact_sha256=group['artifact']['sha256'],unknown_count=len(observation.unknown_regions),
            unknown=[dict(region_id=r.region_id,reason=r.reason) for r in observation.unknown_regions],modules=[],objects=[],
            planning_admissible=False,ros={'status':'NOT_RUN'},clock_domain='ros_sim_time')
        gd['run_stage_seconds']=run_summary.get('stage_seconds',{})
        for module,ref in index['module_observations'].items():
            raw=_read_reference(ref,root);obs=loads(raw.decode());wire=json.loads(raw)
            inputrefs=obs.coverage['input_provenance']['consumed_files']
            rgbraw=_read_reference(inputrefs['sensor_rgb.png'],root)
            rgb=np.asarray(Image.open(BytesIO(rgbraw)).convert('RGB'))
            depth,K,members,masks=module_inputs(index,module,root)
            info=json.loads(_read_reference(inputrefs['camera_info.json'],root));T=info['T_W_C']
            folder=Path(inputrefs['sensor_rgb.png']['path']).parent
            dest=base/module;dest.mkdir()
            def image_file(filename,array):
                path=dest/filename;Image.fromarray(np.asarray(array,np.uint8)).save(path)
                return path.relative_to(a.output).as_posix()
            (dest/'rgb.png').write_bytes(rgbraw)
            valid=np.isfinite(depth)&(depth>0)
            # Fixed 0..5 m scale across both cameras and both groups. Magenta=invalid.
            import matplotlib
            colors=(matplotlib.colormaps['viridis'](np.clip(np.nan_to_num(depth,posinf=5)/5,0,1))[...,:3]*255).astype(np.uint8)
            colors[~valid]=[255,75,167]
            depth_image=image_file('depth.png',colors)
            align=image_file('alignment.png',(.55*rgb+.45*colors).astype(np.uint8))
            proposals=read(folder/'oracle_proposals.json')
            prompt=Image.fromarray(rgb);draw=ImageDraw.Draw(prompt)
            for proposal in proposals['instances']:
                draw.rectangle(proposal['bbox'],outline=(255,196,64),width=3)
                draw.text(tuple(proposal['bbox'][:2]),str(proposal['id']),fill=(255,255,255))
            prompt.save(dest/'prompts.png')
            ids=[int(c.raw_result['instance_lineage']['mask_id']) for c in members.values()]
            trace=validate_trace(folder/'v4_validation/stage_trace',ids,obs.coverage['module_binding']['capture_id'],a.origin=='NEW_ALGORITHM_RUN',
                rgb_sha256=hashlib.sha256(rgbraw).hexdigest(),masks_sha256=next(iter(members.values())).mask_reference.sha256 if members else None,
                calibration=obs.coverage['module_binding']['calibration_identity'])
            timings=read(folder/'rgbd_stage_timing.json')
            response=read(folder/'mode_b1_worker_response.json')
            metrics=read(response['metrics_reference']['path'])
            metrics_hash=hashlib.sha256(Path(response['metrics_reference']['path']).read_bytes()).hexdigest()
            if metrics_hash!=response['metrics_reference']['sha256']:raise ValueError('SAM_METRICS_HASH_MISMATCH')
            maskref=next(iter(members.values())).mask_reference if members else None
            # SAM scores and proposal association from current worker artifact, not guessed.
            sam_entries={}
            if members:
                from urllib.parse import urlparse,unquote
                mp=Path(unquote(urlparse(maskref.uri).path)).parent/'cargo_instances.json'
                sam_entries={int(r['instance_id']):r for r in read(mp).get('instances',[])}
            combined=rgb.copy()
            for ordinal,(sid,mask) in enumerate(masks.items()):combined=blend(combined,mask,PALETTE[ordinal%len(PALETTE)])
            md=dict(module_id=module,capture=obs.coverage['module_binding'],camera=info,input_files=inputrefs,
                rgb=(dest/'rgb.png').relative_to(a.output).as_posix(),depth=depth_image,alignment=align,
                prompts=(dest/'prompts.png').relative_to(a.output).as_posix(),proposal=proposals,
                masks=image_file('masks.png',combined),instances=[],timings=timings,sam_metrics=metrics,
                trace_status='COMPLETED' if trace else 'NOT_RECORDED_IN_HISTORICAL_RUN',trace=trace)
            md['one_shot_stage_seconds']=next(r.get('stage_seconds',{}) for r in run_summary['runs'] if r['module']==module)
            if (folder/'sam_boundaries.png').exists():
                shutil.copyfile(folder/'sam_boundaries.png',dest/'sam-boundaries.png')
                md['sam_boundaries']=(dest/'sam-boundaries.png').relative_to(a.output).as_posix()
            md['independent_mask_evaluation']=read(folder/'mask_evaluation.json')
            for sid,member in members.items():
                mask=masks[sid];mid=int(member.raw_result['instance_lineage']['mask_id']);bounds=crop_bounds(mask)
                record=to_wire(member.raw_result.get('record',{}));prefix=f'mask-{mid}'
                im={}
                def cropped(kind,array):
                    path=dest/f'{prefix}-{kind}.png';Image.fromarray(array).crop(bounds).save(path)
                    im[kind]=path.relative_to(a.output).as_posix()
                cropped('rgb',rgb);cropped('mask',blend(rgb,mask,PALETTE[0]))
                ap=folder/'pointcloud_filter'/f'mask_{mid:04d}.npz'
                audit=next((r for r in read(folder/'metric_pointmap_filter_audit.json') if r['mask_id']==mid),None)
                rawmask=mask&valid;retained=np.zeros_like(mask);point_status='NOT_AVAILABLE'
                if ap.exists():
                    with np.load(ap,allow_pickle=False) as ar:
                        rawmask=ar['raw_valid_mask'];retained=ar['filtered_mask'];reasons=ar['rejected_reason']
                    cropped('filter',FILTER_COLORS[reasons]);point_status='CAPTURED_PRODUCTION_FILTER'
                cropped('raw',blend(rgb,rawmask,PALETTE[0]));cropped('retained',blend(rgb,retained,PALETTE[3]))
                initial=None;initial_points=[]
                if trace:
                    td=folder/'v4_validation/stage_trace';initial=read(td/f'mask-{mid}-initial.json')
                    if canonical_fingerprint(initial['planes'])!=canonical_fingerprint(record['observation_segmentation'].get('initial_planes',[])):
                        raise ValueError('INITIAL_PLANES_DIFFER_FROM_ALGORITHM_RECORD')
                    with np.load(td/f'mask-{mid}-initial.npz',allow_pickle=False) as ar:labels=ar['labels']
                    labelimage=rgb.copy()
                    for i,plane in enumerate(initial['planes']):
                        region=labels==i;labelimage=blend(labelimage,region,PALETTE[i%len(PALETTE)])
                        sample=sampled_points(depth,region,K,350)
                        xyz=np.asarray(sample['points']);n=np.asarray(plane['normal']);d=plane['offset_m']
                        # Display statistic, explicitly sampled and not a replacement acceptance test.
                        initial_points.append(dict(label=i,normal=plane['normal'],offset_m=d,sample=sample,
                            display_sample_mean_abs_mm=float(np.abs(xyz@n+d).mean()*1000) if len(xyz) else None))
                    cropped('labels',labelimage)
                faceimage=Image.fromarray(blend(rgb,rawmask,[70,140,190]));drawing=ImageDraw.Draw(faceimage)
                for face in record.get('camera_facing_faces',[]):
                    xy=np.asarray(face.get('corners_2d',[]))
                    if len(xy):drawing.line([tuple(p) for p in np.vstack([xy,xy[0]])],fill=(69,214,161),width=4)
                cropped('faces',np.asarray(faceimage))
                fp=dest/f'{prefix}-record.json';atomic_json(fp,record)
                md['instances'].append(dict(id=sid,mask_id=mid,bounds=bounds,images=im,sam=sam_entries.get(mid,{}),
                    record_path=fp.relative_to(a.output).as_posix(),record={k:v for k,v in record.items() if k!='frozen_support_regions'},
                    surfaces=list(member.observed_surfaces),point_status=point_status,filter_audit=audit,
                    raw_cloud=sampled_points(depth,rawmask,K),filtered_cloud=sampled_points(depth,retained,K),
                    initial=initial,initial_points=initial_points,
                    fused_objects=[c.source_instance_id for c in observation.cargo if (module,sid) in c.raw_result['source_members']]))
            gd['modules'].append(md)
        for ordinal,row in enumerate(cg['objects']):
            ref=row['candidate_artifact'];raw=Path(ref['path']).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=ref['sha256']:raise ValueError('CONTACT_HASH_MISMATCH')
            doc=json.loads(raw);cargo=fused[row['object_id']]
            if len(doc['candidates'])!=row['candidate_count']:raise ValueError('CONTACT_COUNT_MISMATCH')
            for c in doc['candidates']:
                if c['source']['artifact']!=group['artifact'] or c['source']['object_id']!=row['object_id']:
                    raise ValueError('CONTACT_SOURCE_MISMATCH')
            file=base/f'object-{ordinal}.js'
            payload=json.dumps(doc,ensure_ascii=False,allow_nan=False)
            key=f'{name}/{ordinal}'
            file.write_text('window.installContacts('+json.dumps(key)+','+payload+');',encoding='utf-8')
            gd['objects'].append(dict(id=cargo.source_instance_id,members=cargo.raw_result['source_members'],
                diagnostics=cargo.raw_result['fusion_diagnostics'],surfaces=list(cargo.observed_surfaces),
                summary={k:v for k,v in row.items() if k not in ('best','candidate_artifact')},
                contacts=file.relative_to(a.output).as_posix(),contact_key=key))
        if a.ros:
            receipt=a.ros/'receipts'/f'{name}.json'
            if receipt.exists():
                ros=read(receipt)
                if ros['artifact']!=group['artifact'] or ros['status']!='ROS_ACCEPTED':raise ValueError('ROS_GROUP_MISMATCH')
                gd['ros']=ros
        groups.append(gd)
        atomic_json(a.output/'build-status.json',dict(status='RUNNING',groups=[g['name'] for g in groups]))
    data=to_wire(data)
    data['viewer_build_seconds']=perf_counter()-started
    atomic_json(a.output/'data.json',data)
    (a.output/'data.js').write_text('window.STAGE_DATA='+json.dumps(data,ensure_ascii=False,allow_nan=False)+';',encoding='utf-8')
    shutil.copyfile(ROOT/'tools/perception_stage_viewer.html',a.output/'index.html')
    (a.output/'打开查看器.cmd').write_text('@echo off\r\nstart "" "%~dp0index.html"\r\n',encoding='utf-8')
    atomic_json(a.output/'build-status.json',dict(status='COMPLETED',groups=[g['name'] for g in groups],
        files={p.relative_to(a.output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in a.output.rglob('*') if p.is_file()}))
    return 0


if __name__=='__main__':raise SystemExit(main())
