"""Summarize persisted final JSONs; never refit or discard failed denominators."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def statistics(faces):
    result={'count':len(faces)}
    for field in ('plane_distance_m','normal_error_degrees','boundary_outside_m'):
        values=[f[field] for f in faces]
        result[field]={'mean':float(np.mean(values)),'max':float(max(values))} if values else None
    result['gt_boundary_over_10mm']=sum(f['boundary_outside_m']>.01 for f in faces)
    return result


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run',type=Path,required=True)
    p.add_argument('--capture',type=Path,required=True); p.add_argument('--legacy',type=Path,required=True)
    args=p.parse_args(); out=args.run/'delivery'; out.mkdir(exist_ok=True)
    full=json.loads((args.run/'frozen-stack-final/paired_stack_report.json').read_text())
    rows=full['paired_instances']; unique=[r for r in rows if len(r['entities'])==1]
    common=[pair for r in unique for pair in r['paired_common_faces']]
    checks=[f for r in rows for f in r['new_final_checks'] if f['status']=='PASS']
    summary={'sam_denominator':len(rows),'unique_identity_denominator':len(unique),
        'ambiguous_source_denominator':len(rows)-len(unique),
        'old_instances_with_faces':sum(r['old_face_count']>0 for r in rows),
        'new_instances_with_faces':sum(r['new_face_count']>0 for r in rows),
        'old_faces':full['old_faces'],'new_faces':full['new_faces'],
        'new_complete_accepted':full['new_complete_accepted'],
        'common_face_pairs':len(common),
        'old_common':statistics([r['old'] for r in common]),
        'new_common':statistics([r['new'] for r in common]),
        'old_all_unique':statistics([f for r in unique for f in r['old']['faces']]),
        'new_all_unique':statistics([f for r in unique for f in r['new']['faces']]),
        'new_patch_coverage_range':[min(f['patch_observation_coverage'] for f in checks),max(f['patch_observation_coverage'] for f in checks)],
        'depth_residual_comparison':'Legacy predicted-polygon support and new frozen observation support differ; use paired GT geometry for symmetric comparison.',
        'mechanical_occlusion':'DEFERRED_MECHANICAL_OCCLUSION',
        'unresolved_boundary_cases':[{'module':r['module'],'mask_id':r['mask_id'],'entities':r['entities'],'faces':[f for f in r['new']['faces'] if f['boundary_outside_m']>.01]} for r in unique if any(f['boundary_outside_m']>.01 for f in r['new']['faces'])]}
    summary['new_raw_depth_max_mean_m']=max(f['raw']['mean_m'] for f in checks)
    summary['new_interior_depth_max_mean_m']=max(f['interior']['mean_m'] for f in checks)
    summary['new_holdout_depth_max_mean_m']=max(f['spatial_holdout']['mean_m'] for f in checks)
    (out/'quantitative_summary.json').write_text(json.dumps(summary,indent=2))
    panels=[]
    for module in ('module_0_upper','module_1_lower'):
        source=args.capture/'FULL_STACK_NOMINAL'
        if module=='module_1_lower': source=source/'modules'/module
        image=cv2.imread(str(source/'sensor_rgb.png'))
        K=np.asarray(json.loads((source/'camera_info.json').read_text())['K']).reshape(3,3)
        tiles=[]
        for name,root,color in [('Legacy accepted patches',args.legacy,(0,0,255)),('Metric accepted patches',args.run/'frozen-stack-final',(0,255,0))]:
            canvas=image.copy(); records=json.loads((root/'FULL_STACK_NOMINAL'/module/'validated_geometry.json').read_text())['instances']
            for record in records:
                for face in record['camera_facing_faces']:
                    points=np.asarray(face['corners_3d_m']) if 'corners_3d_m' in face else np.asarray(record['corners_3d'])[face['corner_indices']]
                    q=points@K.T; q=q[:,:2]/q[:,2:]
                    cv2.polylines(canvas,[np.rint(q).astype(np.int32)],True,color,3)
                    cv2.putText(canvas,str(record['mask_id']),tuple(np.rint(q.mean(axis=0)).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2)
            canvas=cv2.resize(canvas,(972,729)); cv2.rectangle(canvas,(0,0),(972,72),(20,20,20),-1)
            cv2.putText(canvas,name+' | '+module,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            cv2.putText(canvas,'Same frozen input | mechanical occlusion deferred',(12,57),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
            tiles.append(canvas)
        comparison=np.hstack(tiles); cv2.imwrite(str(out/(module+'_stack_comparison.png')),comparison); panels.append(comparison)
    scenes=[]
    for module in ('module_0_upper','module_1_lower'):
        scenes.append(cv2.imread(str(args.run/'calibration-c50ff4c'/module/'single_box_comparison.png')))
    for scene in ('GEOMETRY_ADJACENT_TWO','GEOMETRY_PARTIAL_TWO'):
        for module in ('module_0_upper','module_1_lower'):
            scenes.append(cv2.imread(str(args.run/'small-matrix-final'/scene/module/'two_box_comparison.png')))
    scenes.extend(panels)
    video=cv2.VideoWriter(str(out/'metric_comparison_16s.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1440,960))
    for im in scenes:
        if im is None: raise ValueError('MISSING_FINAL_EVIDENCE_PANEL')
        scale=min(1440/im.shape[1],960/im.shape[0]); resized=cv2.resize(im,(round(im.shape[1]*scale),round(im.shape[0]*scale)))
        canvas=np.zeros((960,1440,3),np.uint8); h,w=resized.shape[:2]; canvas[(960-h)//2:(960-h)//2+h,(1440-w)//2:(1440-w)//2+w]=resized
        for _ in range(20): video.write(canvas)
    video.release()
    capture=cv2.VideoCapture(str(out/'metric_comparison_16s.mp4'))
    assert int(capture.get(cv2.CAP_PROP_FRAME_COUNT))==160
    capture.release()
    print(json.dumps({k:v for k,v in summary.items() if k!='unresolved_boundary_cases'}),flush=True)


if __name__=='__main__': main()
