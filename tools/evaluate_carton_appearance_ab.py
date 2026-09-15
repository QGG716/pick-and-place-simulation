"""Compact post-hoc overlap, physical-boundary and patch coverage A/B report."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from run_carton_appearance_ab import folder
from run_isaac_rgbd_geometry import _worker_artifacts,sha256
from unloading_perception.final_geometry import polygon_pixels,project,SIGNS
from diagnose_metric_calibration import evaluate

# Frozen diagnostic definitions, not geometry acceptance relaxations.
POLICY={'significant_overlap_min_pixels':50,'significant_fraction_each_region':.1,
        'matched_gt_best_mask_iou':.5,'boundary_tolerance_px':2,'minimum_gt_face_pixels':50,
        'physical_corners':'NOT_CERTIFIED_PATCH_CORNERS_ARE_NOT_PHYSICAL_CORNERS'}
BOX_FACES=((0,3,7,4),(1,2,6,5),(0,1,5,4),(3,2,6,7),(0,1,2,3),(4,5,6,7))


def contour(mask):
    return mask & ~cv2.erode(mask.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)


def boundary_metrics(pred,truth):
    dp=cv2.distanceTransform((~pred).astype(np.uint8),cv2.DIST_L2,5)
    dg=cv2.distanceTransform((~truth).astype(np.uint8),cv2.DIST_L2,5)
    return {'prediction_pixels':int(pred.sum()),'gt_visible_physical_edge_pixels':int(truth.sum()),
            'precision_2px':float(np.mean(dg[pred]<=2)) if pred.any() else 0.,
            'recall_2px':float(np.mean(dp[truth]<=2)) if truth.any() else None,
            'gt_to_prediction_mean_px':float(dp[truth].mean()) if truth.any() and pred.any() else None}


def ground_truth(folder,camera):
    depth=np.load(folder/'metric_depth_m.npy');K=np.asarray(camera['K']).reshape(3,3);T=np.asarray(camera['T_W_C'])
    gt=[g for g in json.loads((folder/'gt_annotations.json').read_text())['objects'] if g['visible']]
    archive=np.load(folder/'gt_instance_masks.npz');labels=np.zeros(depth.shape,np.int16);face_labels=np.zeros_like(labels)
    physical=np.zeros(depth.shape,bool);faces={}
    for index,g in enumerate(gt,1):
        mask=archive[g['mask_key']].astype(bool);labels[mask]=index;y,x=np.nonzero(mask);z=depth[y,x]
        G=np.asarray(g['T_W_object']);dims=np.asarray(g['full_dimensions_m'])
        rays=np.column_stack(((x-K[0,2])/K[0,0],(y-K[1,2])/K[1,1],np.ones(len(x))))@T[:3,:3].T@G[:3,:3]
        origin=(T[:3,3]-G[:3,3])@G[:3,:3]
        # Exact ray/slab entry face; nearest noisy point-to-plane distance can
        # mislabel a one-pixel border as a tiny additional visible face.
        with np.errstate(divide='ignore'):
            near=np.minimum((-dims/2-origin)/rays,(dims/2-origin)/rays)
        axis=near.argmax(axis=1);sign=rays[np.arange(len(rays)),axis]<0
        codes=(index-1)*6+axis*2+sign+1;face_labels[y,x]=codes
        corners=((SIGNS*dims/2)@G[:3,:3].T+G[:3,3]-T[:3,3])@T[:3,:3]
        for face_index,ids in enumerate(BOX_FACES):
            code=(index-1)*6+face_index+1;region=face_labels==code
            if region.sum()<POLICY['minimum_gt_face_pixels']:continue
            key=g['simulation_object_id']+f'/{face_index//2}/{1 if face_index%2 else -1}'
            faces[code]={'key':key,'object':g['simulation_object_id'],'visible_pixels':int(region.sum()),'covered':np.zeros(depth.shape,bool)}
            q=project(corners[list(ids)],K);edge=np.zeros(depth.shape,np.uint8)
            cv2.polylines(edge,[q.round().astype(np.int32)],True,1,1)
            physical|=edge.astype(bool)&cv2.dilate(region.astype(np.uint8),np.ones((5,5),np.uint8)).astype(bool)
    return gt,labels,face_labels,faces,physical


def assess(root,group,module,out):
    source=folder(root,group,module);camera=json.loads((source/'camera_info.json').read_text())
    gt,labels,face_labels,gt_faces,physical=ground_truth(source,camera)
    artifacts=_worker_artifacts(source);npz=np.load(artifacts['cargo_masks.npz']['path']);masks=npz['masks'].astype(bool);ids=npz['mask_ids']
    areas=np.bincount(labels.ravel(),minlength=len(gt)+1)[1:]
    intersections=np.array([np.bincount(labels[mask],minlength=len(gt)+1)[1:] for mask in masks])
    pred_area=masks.sum(axis=(1,2));iou=intersections/np.maximum(1,pred_area[:,None]+areas[None,:]-intersections)
    relation=(intersections>=50)&(intersections>=.1*areas[None,:])&(intersections>=.1*pred_area[:,None])
    union=masks.any(axis=0);instances=[]
    for j,g in enumerate(gt):
        best=int(iou[:,j].argmax()) if len(masks) else None
        instances.append({'object':g['simulation_object_id'],'visible_pixels':int(areas[j]),
            'best_mask_iou':float(iou[best,j]) if best is not None else 0.,
            'best_sam_id':int(ids[best]) if best is not None else None,
            'significant_sam_ids':[int(ids[i]) for i in np.flatnonzero(relation[:,j])],
            'visible_gt_covered_by_sam_fraction':float(np.count_nonzero(union&(labels==j+1))/areas[j])})
    predicted=[{'sam_id':int(identity),'gt_overlap_relations':[{'object':gt[j]['simulation_object_id'],
        'intersection_pixels':int(intersections[i,j]),'iou':float(iou[i,j]),'fraction_prediction':float(intersections[i,j]/max(1,pred_area[i])),
        'fraction_gt':float(intersections[i,j]/areas[j])} for j in range(len(gt)) if intersections[i,j]>0],
        'significant_objects':[gt[j]['simulation_object_id'] for j in np.flatnonzero(relation[i])]} for i,identity in enumerate(ids)]
    geometry=json.loads((source/'rgbd_cuboids.json').read_text());face_rows=[];counts=Counter();rejections=Counter()
    image=cv2.imread(str(source/'sensor_rgb.png'));sam_image=image.copy();face_image=image.copy()
    pred_boundary=np.zeros(labels.shape,bool);patch_boundary=np.zeros_like(pred_boundary)
    for identity,mask in zip(ids,masks):
        boundary=contour(mask);pred_boundary|=boundary
        color=tuple(int(v) for v in np.random.default_rng(int(identity)+17).integers(60,250,3))
        sam_image[boundary]=color
        y,x=np.nonzero(mask);cv2.putText(sam_image,str(identity),(int(x.mean()),int(y.mean())),cv2.FONT_HERSHEY_SIMPLEX,.6,color,1)
    face_image[physical]=(255,255,0)
    for record in geometry['instances']:
        if not record['camera_facing_faces']:rejections['NO_ACCEPTED_FACE']+=1
        for check in record['final_face_validation']:
            if check['status']!='PASS':rejections[check.get('reason','REJECTED')]+=1
        for face in record['camera_facing_faces']:
            polygon=polygon_pixels(face['corners_2d'],labels.shape) if 'corners_2d' in face else polygon_pixels(project(face['corners_3d_m'],camera['K']),labels.shape)
            patch_boundary|=contour(polygon)
            overlap=np.bincount(face_labels[polygon],minlength=len(gt)*6+1);overlap[0]=0;code=int(overlap.argmax())
            row={'sam_id':record['mask_id'],'support_label':face['support_label'],'observed_region_coverage':face['final_support']['patch_observation_coverage'],
                 'depth_raw':face['final_support']['raw'],'depth_interior':face['final_support']['interior'],
                 'physical_corner_error':None,'physical_corner_status':'NOT_CERTIFIED',
                 'overlapping_gt_objects':[gt[j]['simulation_object_id'] for j in range(len(gt)) if overlap[j*6+1:j*6+7].sum()>=50]}
            if overlap[code]>0:
                g=gt[(code-1)//6];metric=evaluate({'accepted':False,'camera_facing_faces':[face]},g,camera)['faces'][0]
                row.update({k:metric[k] for k in ('plane_distance_m','normal_error_degrees','boundary_outside_m')})
                matched=(code-1)//6*6+metric['gt_axis']*2+(metric['gt_sign']>0)+1
                row['gt_face']=g['simulation_object_id']+f"/{metric['gt_axis']}/{metric['gt_sign']}"
                if matched in gt_faces:
                    gt_faces[matched]['covered']|=polygon&(face_labels==matched)
                    row['gt_visible_face_coverage']=float(np.count_nonzero(polygon&(face_labels==matched))/gt_faces[matched]['visible_pixels'])
            else:row['evaluation_status']='NO_VISIBLE_GT_SUPPORT'
            boundary_evidence=face.get('boundary_evidence',[])
            counts.update(v['kind'] for v in boundary_evidence)
            q=project(face['corners_3d_m'],camera['K']).round().astype(np.int32)
            for i,(a,b) in enumerate(zip(q,np.roll(q,-1,axis=0))):
                kind=boundary_evidence[i]['kind'] if i<len(boundary_evidence) else 'UNCLASSIFIED_BOUNDARY'
                cv2.line(face_image,tuple(a),tuple(b),(0,220,0) if kind=='PHYSICAL_EDGE_SUPPORTED' else (0,165,255),2)
            face_rows.append(row)
    coverage=[{k:v for k,v in record.items() if k!='covered'}|{'coverage':float(record['covered'].sum()/record['visible_pixels'])} for record in gt_faces.values()]
    valid=[r for r in face_rows if 'boundary_outside_m' in r]
    summary={'group':group,'module':module,'proposal_count':len(json.loads((source/'oracle_proposals.json').read_text())['instances']),
        'sam_count':len(ids),'visible_gt_objects':len(gt),'gt_best_iou_mean':float(np.mean([r['best_mask_iou'] for r in instances])),
        'gt_below_iou_0_5':sum(r['best_mask_iou']<.5 for r in instances),'merged_prediction_count':int((relation.sum(axis=1)>1).sum()),
        'over_split_gt_count':int((relation.sum(axis=0)>1).sum()),'ambiguous_or_no_relation_predictions':int((relation.sum(axis=1)!=1).sum()),
        'face_count':len(face_rows),'gt_evaluable_faces':len(valid),'gt_boundary_over_10mm':sum(r['boundary_outside_m']>.01 for r in valid),
        'mean_boundary_outside_m':float(np.mean([r['boundary_outside_m'] for r in valid])) if valid else None,
        'max_boundary_outside_m':max((r['boundary_outside_m'] for r in valid),default=None),
        'mean_visible_gt_face_coverage':float(np.mean([r['coverage'] for r in coverage])),
        'visible_gt_face_count':len(coverage),
        'pixel_weighted_gt_face_coverage':float(sum(r['coverage']*r['visible_pixels'] for r in coverage)/sum(r['visible_pixels'] for r in coverage)),
        'mean_sam_observation_patch_coverage':float(np.mean([r['observed_region_coverage'] for r in face_rows])) if face_rows else None,
        'max_interior_mean_depth_m':max((r['depth_interior']['mean_m'] for r in face_rows),default=None),
        'max_interior_p95_depth_m':max((r['depth_interior']['p95_m'] for r in face_rows),default=None),
        'sam_physical_boundary':boundary_metrics(pred_boundary,physical),'patch_physical_boundary':boundary_metrics(patch_boundary,physical),
        'boundary_kinds':dict(counts),'rejections':dict(rejections),'certified_physical_corners':0}
    report={'policy':POLICY,'summary':summary,'gt_objects':instances,'predictions':predicted,'faces':face_rows,'gt_visible_faces':coverage}
    (out/f'{group}_{module}.json').write_text(json.dumps(report,indent=2))
    cv2.imwrite(str(out/f'{group}_{module}_sam.png'),sam_image);cv2.imwrite(str(out/f'{group}_{module}_faces.png'),face_image)
    return report,sam_image,face_image


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--historical',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    reports={};rows=[]
    for module in ('module_0_upper','module_1_lower'):
        tiles={}
        for group in ('A','B'):
            report,sam,faces=assess(args.capture,group,module,args.output);reports[group,module]=report;rows.append(report['summary']);tiles[group]=(sam,faces)
        for col,kind in enumerate(('sam','faces')):
            panels=[]
            for group in ('A','B'):
                im=cv2.resize(tiles[group][col],(972,729));cv2.rectangle(im,(0,0),(972,60),(20,20,20),-1)
                cv2.putText(im,group+' '+module+' | '+kind,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1)
                if kind=='faces':cv2.putText(im,'cyan: GT physical edges; green: supported; orange: unknown/occluded',(12,49),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                panels.append(im)
            cv2.imwrite(str(args.output/f'{module}_{kind}_AB.png'),np.hstack(panels))
    paired=[]
    for module in ('module_0_upper','module_1_lower'):
        a={r['key']:r for r in reports['A',module]['gt_visible_faces']};b={r['key']:r for r in reports['B',module]['gt_visible_faces']}
        for key in sorted(a.keys()|b.keys()):
            paired.append({'module':module,'gt_face':key,'A':a.get(key),'B':b.get(key),
                'A_predictions':[f for f in reports['A',module]['faces'] if f.get('gt_face')==key],
                'B_predictions':[f for f in reports['B',module]['faces'] if f.get('gt_face')==key]})
    objects=[]
    for module in ('module_0_upper','module_1_lower'):
        a={r['object']:r for r in reports['A',module]['gt_objects']};b={r['object']:r for r in reports['B',module]['gt_objects']}
        objects.extend({'module':module,'object':key,'A':a.get(key),'B':b.get(key)} for key in sorted(a.keys()|b.keys()))
    (args.output/'summary.json').write_text(json.dumps({'policy':POLICY,'views':rows,'paired_physical_objects':objects,'paired_physical_faces':paired},indent=2))
    fields=[k for k,v in rows[0].items() if not isinstance(v,dict)]
    with (args.output/'summary.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    # Historical mask 10 locates a region, never an algorithm rule.
    historical=args.historical/'FULL_STACK_NOMINAL/modules/module_1_lower'
    old=np.load(_worker_artifacts(historical)['cargo_masks.npz']['path']);index=list(old['mask_ids']).index(10)
    y,x=np.nonzero(old['masks'][index]);x0,x1=max(0,x.min()-30),min(2592,x.max()+31);y0,y1=max(0,y.min()-35),min(1944,y.max()+36)
    for group in ('A','B'):
        raw=cv2.imread(str(folder(args.capture,group,'module_1_lower')/'sensor_rgb.png'))
        sam=cv2.imread(str(args.output/f'{group}_module_1_lower_sam.png'))
        patch=cv2.imread(str(args.output/f'{group}_module_1_lower_faces.png'))
        cv2.imwrite(str(args.output/f'historical_mask10_{group}_native_crop.png'),np.hstack([im[y0:y1,x0:x1] for im in (raw,sam,patch)]))
    (args.output/'historical_case.json').write_text(json.dumps({'historical_mask':10,'module':'module_1_lower','crop_xyxy':[int(x0),int(y0),int(x1),int(y1)],'purpose':'DISPLAY_LOCATION_ONLY_NOT_A_SAM_PAIRING_RULE'},indent=2))
    print(json.dumps(rows),flush=True)


if __name__=='__main__':main()
