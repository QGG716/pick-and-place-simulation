"""Complete frozen stage evidence and 3D comparisons from final JSON outputs."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.metric_support import check_metric_plane,backproject_pixels
from unloading_perception.final_geometry import SIGNS
from run_isaac_rgbd_geometry import _worker_artifacts


def main():
    p=argparse.ArgumentParser(); p.add_argument('--capture',type=Path,required=True); p.add_argument('--legacy',type=Path,required=True)
    p.add_argument('--result',type=Path,required=True); p.add_argument('--vision',type=Path,required=True)
    args=p.parse_args()
    spec=importlib.util.spec_from_file_location('fixed_v4',args.vision/'pipeline/geometry/refine_multiplane_cuboids.py')
    upstream=importlib.util.module_from_spec(spec); spec.loader.exec_module(upstream)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    manifest=json.loads((args.capture/'scene_bundle/RGBD_CALIBRATION_BOX.manifest.json').read_text())
    for camera in manifest['cameras']:
        module=camera['module_id']; folder=args.capture/'RGBD_CALIBRATION_BOX'
        if module=='module_1_lower': folder=folder/'modules'/module
        dest=args.result/module; legacy=args.legacy/'RGBD_CALIBRATION_BOX'/module
        depth=np.load(folder/'metric_depth_m.npy'); K=camera['K']; T=np.asarray(camera['T_W_C'])
        masks=np.load(_worker_artifacts(folder)['cargo_masks.npz']['path']); mask=masks['masks'][0].astype(bool)
        labels=np.load(dest/'SAM_support.npz')['labels']
        filtered=np.load(folder/'pointcloud_filter/mask_0001.npz')['filtered_mask']
        raw=json.loads((folder/'rgbd_cuboids_baseline_raw.json').read_text())['instances'][0]
        final=json.loads((dest/'SAM.json').read_text()); seeds=final['observation_segmentation']['initial_planes']
        stages=json.loads((dest/'stages.json').read_text()); head=[]
        for name,selected in [('A_RAW_REGISTERED_DEPTH',mask),('B_ORIGINAL_FILTERED_CLOUD',filtered)]:
            rows=[]
            for equation in raw['plane_equations']:
                n=np.asarray(equation[:3]); length=np.linalg.norm(n); n/=length; d=equation[3]/length
                label=int(np.argmax([abs(n@np.asarray(s['normal'])) for s in seeds]))
                rows.append({'normal':n.tolist(),'offset_m':d,'support_label':label,
                             'audit':check_metric_plane(n,d,depth,mask,selected & (labels==label),K,erosion_px=0)})
            head.append({'stage':name,'pixel_count':int(selected.sum()),'residual_reference':'ORIGINAL_INITIAL_PLANE_REEVALUATED_ON_FROZEN_OBSERVATION_LABELS','planes':rows})
        primary=json.loads((legacy/'metric_multiplane_input.json').read_text())['instances'][0]
        candidates=[]
        for f in primary['camera_facing_faces']:
            q=np.asarray(f['corners_2d']); metrics=upstream.mask_metrics(mask,q)
            if f['evidence']=='monocular_depth_plane' and metrics['mask_precision']>=.72 and metrics['mask_coverage']>=.04:
                candidates.append((metrics['mask_coverage'],q))
        polygons=[q for _,q in sorted(candidates,key=lambda v:v[0],reverse=True)[:3]]
        H,metrics=upstream.register_face_union(mask,polygons)
        stage_e={'stage':'E_2D_REGISTERED_TARGET','homography':H.tolist(),'registration':metrics,
                 'target_boundaries_2d':[upstream.transform_points(q,H).tolist() for q in polygons],
                 'metric_plane_residual':'NOT_DEFINED_FOR_2D_TARGET_UNTIL_PNP','scale_change':'PROJECTIVE_2D_TRANSFORM_ONLY'}
        stages.insert(2,stage_e)
        tail={'stage':'H_FINAL_INDEPENDENT_CHECK','legacy':'REJECTED','metric_complete_accepted':final['accepted'],
              'metric_final_checks':final['final_face_validation'],'metric_solver':final['metric_solver']}
        (dest/'stages_complete.json').write_text(json.dumps(head+stages+[tail],indent=2))
        old=json.loads((legacy/'worker.json').read_text())['instances'][0]
        rejection=json.loads((legacy/'validated_geometry.json').read_text())['instances'][0]['v4_path_audit']
        old['accepted']=False; old['independent_final_status']='REJECTED'; old['independent_rejection_audit']=rejection
        (dest/'legacy_rejected_geometry.json').write_text(json.dumps(old,indent=2))
        # Read both final decisions back before rendering; no worker.png is used.
        old=json.loads((dest/'legacy_rejected_geometry.json').read_text()); final=json.loads((dest/'SAM.json').read_text())
        truth=next(g for g in json.loads((folder/'gt_annotations.json').read_text())['objects'] if g['visible'])
        G=np.asarray(truth['T_W_object']); dims=np.asarray(truth['full_dimensions_m']); gt=(SIGNS*dims/2)@G[:3,:3].T+G[:3,3]
        valid=mask & np.isfinite(depth) & (depth>0); y,x=np.nonzero(valid); step=max(1,len(x)//3500); x=x[::step]; y=y[::step]
        cloud=backproject_pixels(np.c_[x,y],depth[y,x],K)@T[:3,:3].T+T[:3,3]
        fig=plt.figure(figsize=(15,6),layout='constrained')
        for column,(title,record,color) in enumerate([('Legacy rejected geometry + raw depth',old,'red'),('Metric accepted patches + raw depth',final,'green')],1):
            ax=fig.add_subplot(1,2,column,projection='3d'); ax.scatter(*cloud.T,s=.5,c='black',alpha=.25)
            for face in record['camera_facing_faces']:
                points=np.asarray(face['corners_3d_m']) if 'corners_3d_m' in face else np.asarray(record['corners_3d'])[face['corner_indices']]
                points=points@T[:3,:3].T+T[:3,3]
                ax.add_collection3d(Poly3DCollection([points],facecolor=color,edgecolor=color,alpha=.15))
            for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
                ax.plot(*gt[[a,b]].T,color='orange',ls='--',lw=1)
            ax.set(xlim=(G[0,3]-.6,G[0,3]+.6),ylim=(G[1,3]-.45,G[1,3]+.45),zlim=(G[2,3]-.3,G[2,3]+.3),
                   xlabel='World X (m)',ylabel='World Y (m)',zlabel='World Z (m)',title=title)
            ax.set_box_aspect((1.2,.9,.6)); ax.view_init(elev=24,azim=-125)
        fig.suptitle('ALGORITHM_ONLY | '+module+' | orange dashed = independent GT; black = raw captured points')
        fig.savefig(dest/'raw_cloud_metric_comparison.png',dpi=160); plt.close(fig)


if __name__=='__main__': main()
