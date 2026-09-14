"""Collect a bounded acceptance artifact and render the two actual snapshots.

Run on the validation server; input runs are never modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from unloading_perception.geometry import rotation_from_quaternion
from unloading_perception.final_geometry import SIGNS


def main():
    p=argparse.ArgumentParser(); p.add_argument('--run-root',type=Path,required=True); p.add_argument('--frozen-root',type=Path,required=True); p.add_argument('--output',type=Path,required=True); args=p.parse_args()
    out=args.output; out.mkdir(parents=True,exist_ok=False); r=args.run_root
    provenance={}
    def copy(source,target):
        dest=out/target; dest.parent.mkdir(parents=True,exist_ok=True); shutil.copyfile(source,dest)
        provenance[target]={'source':str(source),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'bytes':source.stat().st_size}
    for old,new in [('identity-6f6e2b3','identity'),('visibility-final','visibility'),('ros-final-0507a69','ros'),('ros-missing-module','ros_missing_module')]:
        for path in sorted((r/old).rglob('*')):
            if path.is_file(): copy(path,str(Path(new)/path.relative_to(r/old)))
    metrics=r/'v4-final-6f6e2b3'
    for path in sorted(metrics.rglob('*')):
        if path.is_file() and (path.name in {'v4_path_audit.json','independent_final_face_accuracy.json','association_evaluation.json','fusion.json','fused_algorithm_observation.json','observation.json','command.json','validated_geometry.json','metric_evidence_12s.mp4'} or path.suffix=='.png' and path.name!='worker.png'):
            copy(path,str(Path('metric')/path.relative_to(metrics)))
    for name in ['historical_fusion_corrected_audit.json','final_accuracy_summary.json','pytest-0507a69.xml','pytest-0507a69.log','humble-final-0507a69.log','ci_old_discovery_reproduction.xml','ci_old_discovery_reproduction.log','ci_5861cdb_run.json','ci_5861cdb_jobs.json','latest_ci_runs.json','no-moge-full/no_moge_report.json','no-moge-full.log','ros-final-0507a69.log','ros-missing-module.log']:
        copy(r/name,name)
    for filename in ['pytest.xml','stdout.log','stderr.log']:
        matches=list((r/'ci_diagnostics').rglob(filename))
        match=next(path for path in matches if ('/build/unloading_ros_bridge/' in str(path) if filename=='pytest.xml' else '/test-log/latest/unloading_ros_bridge/' in str(path)))
        copy(match,'ci_failed_bd00dfe/'+filename)
    copy(r/'humble/final-0507a69/build/unloading_ros_bridge/pytest.xml','humble/pytest.xml')
    copy(r/'visibility-capture-v2/summary.json','visibility/capture_summary.json')
    copy(args.frozen_root/'FULL_STACK_NOMINAL/planning_world_snapshot.json','gt/planning_world_snapshot.json')
    copy(args.frozen_root/'FULL_STACK_NOMINAL/feasibility_handoff.json','gt/feasibility_handoff.json')
    figure=plt.figure(figsize=(14,7)); axes=[figure.add_subplot(121,projection='3d'),figure.add_subplot(122,projection='3d')]
    topology=((0,1,2,3),(4,5,6,7),(0,1,5,4),(3,2,6,7),(0,3,7,4),(1,2,6,5))
    for ax,name,title in zip(axes,['ros/perception_world_snapshot.json','gt/planning_world_snapshot.json'],['Actual ROS algorithm snapshot\nObserved surfaces; hidden volume UNKNOWN','Separate ISAAC_GT snapshot\nFull volumes from simulator']):
        snapshot=json.loads((out/name).read_text()); obstacles=snapshot['scene_snapshot']['obstacles']
        for obj in obstacles:
            for face in obj.get('observed_surfaces',[]):
                ax.add_collection3d(Poly3DCollection([face['corners_3d_m']],facecolor='cyan',edgecolor='teal',alpha=.4))
            if obj.get('pose') is not None and obj.get('full_dimensions_m') is not None:
                pose=obj['pose']; rotation=np.asarray(rotation_from_quaternion(pose['orientation_xyzw'])); points=(SIGNS*np.asarray(obj['full_dimensions_m'])/2)@rotation.T+np.asarray(pose['position_m'])
                ax.add_collection3d(Poly3DCollection([points[list(f)] for f in topology],facecolor='orange',edgecolor='brown',alpha=.12))
        ax.set(xlabel='X into trailer (m)',ylabel='Y left (m)',zlabel='Z up (m)',title=title,xlim=(-.2,.8),ylim=(-1.2,1.2),zlim=(0,2.5)); ax.set_box_aspect((1,2.4,2.5)); ax.view_init(elev=20,azim=-150)
    figure.tight_layout(); figure.savefig(out/'actual_ros_vs_gt_snapshot.png',dpi=150,bbox_inches='tight',pad_inches=.25); plt.close(figure)
    source=args.frozen_root/'FULL_STACK_NOMINAL'
    proposals={p['id']:p for p in json.loads((source/'oracle_proposals.json').read_text())['instances']}
    audit=next(a for a in json.loads((out/'identity/instance_lineage_audit.json').read_text()) if a['module_id']=='module_0_upper' and a['mask_id']==3)
    response=json.loads((source/'mode_b1_worker_response.json').read_text()); metrics=json.loads(Path(response['metrics_reference']['path']).read_text())
    with np.load(metrics['artifacts']['cargo_masks.npz']['path'],allow_pickle=False) as archive:
        mask=archive['masks'][list(archive['mask_ids']).index(3)].astype('uint8')
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    picture=cv2.imread(str(source/'sensor_rgb.png')); bounds=np.asarray([proposals[3]['bbox'],audit['original_bbox'],audit['mask_bbox']]); x0,y0=np.maximum(bounds[:,:2].min(axis=0).astype(int)-30,0); x1,y1=np.minimum(bounds[:,2:].max(axis=0).astype(int)+30,[picture.shape[1],picture.shape[0]])
    panels=[]
    for pid,title in [(3,'BEFORE: same-number join'),(audit['proposal_id'],'AFTER: actual SAM lineage')]:
        canvas=picture.copy(); cv2.drawContours(canvas,contours,-1,(0,255,255),3); b=np.asarray(proposals[pid]['bbox'],int); cv2.rectangle(canvas,tuple(b[:2]),tuple(b[2:]),(255,120,0),3)
        tile=cv2.resize(canvas[y0:y1,x0:x1],(900,650)); tile=cv2.copyMakeBorder(tile,100,0,0,0,cv2.BORDER_CONSTANT,value=(15,15,15))
        cv2.putText(tile,title,(15,30),cv2.FONT_HERSHEY_SIMPLEX,.9,(255,255,255),2)
        cv2.putText(tile,f"Yellow: SAM 3 | Blue: proposal {pid} / {proposals[pid]['simulation_object_id']}",(15,67),cv2.FONT_HERSHEY_SIMPLEX,.64,(255,255,255),1); panels.append(tile)
    cv2.imwrite(str(out/'identity_case_sam3.png'),np.hstack(panels))
    manifest={'collector_code_base_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'tested_geometry_sha':'6f6e2b34dd9cbf2dee7ea567e5ebf2b05018b4c5',
              'tested_final_ros_cpu_sha':'0507a69841174346ed92d293c80fcd9f0ea4ee79',
              'upstream_sha':'1d208f2ed380a207e6e46b4a62d2ac640edfe477',
              'artifacts':provenance,'generated_files':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.glob('*.png')},
              'scope':'bounded copies; raw RGB/depth/SAM/first-hit arrays remain under hash-bound server runs; historical inputs preserved'}
    (out/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({'files':len(provenance),'copied_bytes':sum(v['bytes'] for v in provenance.values()),'output':str(out)}))


if __name__=='__main__': main()
