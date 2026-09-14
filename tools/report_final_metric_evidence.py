"""Independent GT plane and bounded-surface audit of frozen final results."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from unloading_contracts import loads


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--capture-directory',type=Path,required=True); parser.add_argument('--result-directory',type=Path,required=True); args=parser.parse_args()
    all_rows=[]; panels=[]
    for scene_dir in sorted(args.result_directory.iterdir()):
        if not scene_dir.is_dir(): continue
        for module in ('module_0_upper','module_1_lower'):
            dest=scene_dir/module
            if not (dest/'observation.json').exists(): continue
            source=args.capture_directory/scene_dir.name
            if module=='module_1_lower': source=source/'modules'/module
            truth={o['simulation_object_id']:o for o in json.loads((source/'gt_annotations.json').read_text())['objects']}
            obs=loads((dest/'observation.json').read_text()); image=cv2.imread(str(source/'sensor_rgb.png'))
            records=json.loads((dest/'validated_geometry.json').read_text())['instances']
            for cargo in obs.cargo:
                lineage=cargo.raw_result.get('instance_lineage',{}); entities=lineage.get('evaluation_entity_ids',[])
                for face in cargo.observed_surfaces:
                    row={'scene':scene_dir.name,'module':module,'instance':cargo.source_instance_id,'mask_id':lineage['mask_id'],'face_id':face['face_id'],'gt_entities':entities,'final_depth_residual_m':face['plane_residual_m'],'point_support_count':face['point_support_count'],'boundary_kind':face['boundary_kind']}
                    if len(entities)==1:
                        gt=truth[entities[0]]; T=np.asarray(gt['T_W_object']); dims=np.asarray(gt['full_dimensions_m']); p=np.asarray(face['corners_3d_m']); n=np.asarray(face['plane_normal']); local=(p-T[:3,3])@T[:3,:3]; candidates=[]
                        for axis in range(3):
                            for sign in (-1,1):
                                distances=np.abs(local[:,axis]-sign*dims[axis]/2); normal_error=np.degrees(np.arccos(np.clip(abs(n@T[:3,axis]),0,1)))
                                candidates.append((float(distances.mean()),float(normal_error),axis,sign))
                        distance,angle,axis,sign=min(candidates)
                        tangential=[i for i in range(3) if i!=axis]
                        overflow=np.maximum(np.abs(local[:,tangential])-dims[tangential]/2,0)
                        row.update(gt_plane_distance_m=distance,gt_normal_error_deg=angle,gt_patch_boundary_outside_face_m=float(np.linalg.norm(overflow,axis=1).max()),gt_face_axis=axis,gt_face_sign=sign,physical_corner_error='NOT_EVALUATED_OBSERVED_PATCH_IS_NOT_PHYSICAL_CORNER')
                    else: row['gt_status']='AMBIGUOUS_SOURCE_IDENTITY'
                    all_rows.append(row)
                    polygon=np.asarray(face['boundary_2d_px']).round().astype('int32')
                    cv2.polylines(image,[polygon],True,(0,255,255),2)
                    cv2.putText(image,f"SAM{lineage['mask_id']} {face['plane_residual_m']*1000:.2f}mm",tuple(polygon[0]),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2)
            for record in records:
                for check in record['final_face_validation']:
                    if check['status']=='REJECTED': all_rows.append({'scene':scene_dir.name,'module':module,'mask_id':record['mask_id'],'rejected_face':check})
            cv2.putText(image,f'{scene_dir.name} {module}: validated metric patches',(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
            cv2.imwrite(str(dest/'final_metric_faces_overlay.png'),image)
            panels.append(cv2.resize(image,(1296,972)))
            if scene_dir.name=='FULL_STACK_NOMINAL':
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                from mpl_toolkits.mplot3d.art3d import Poly3DCollection
                from unloading_perception.final_geometry import polygon_pixels,SIGNS
                meta=json.loads((source/'capture_metadata.json').read_text()); T=np.asarray(meta['T_W_C_at_capture'])
                info=json.loads((source/'camera_info.json').read_text()); K=np.asarray(info['K']).reshape(3,3)
                depth=np.load(source/'metric_depth_m.npy',allow_pickle=False)
                fig=plt.figure(figsize=(14,6)); ax=fig.add_subplot(121,projection='3d'); gt_ax=fig.add_subplot(122,projection='3d')
                for cargo in obs.cargo:
                    for face in cargo.observed_surfaces:
                        p=np.asarray(face['corners_3d_m']); ax.add_collection3d(Poly3DCollection([p],facecolor='cyan',edgecolor='teal',alpha=.22))
                        pixels=polygon_pixels(face['boundary_2d_px'],depth.shape)&np.isfinite(depth)&(depth>0)
                        y,x=np.nonzero(pixels); stride=max(1,len(x)//1500); x=x[::stride]; y=y[::stride]; z=depth[y,x]
                        cloud=np.column_stack(((x-K[0,2])*z/K[0,0],(y-K[1,2])*z/K[1,1],z))@T[:3,:3].T+T[:3,3]
                        ax.scatter(*cloud.T,s=.3,c='black',alpha=.25)
                faces=((0,1,2,3),(4,5,6,7),(0,1,5,4),(3,2,6,7),(0,3,7,4),(1,2,6,5))
                for gt in truth.values():
                    pose=np.asarray(gt['T_W_object']); corners=(SIGNS*np.asarray(gt['full_dimensions_m'])/2)@pose[:3,:3].T+pose[:3,3]
                    gt_ax.add_collection3d(Poly3DCollection([corners[list(f)] for f in faces],facecolor='orange',edgecolor='brown',alpha=.1))
                for axis,title in [(ax,'Algorithm: measured patches + registered depth'),(gt_ax,'Independent GT: full oriented boxes')]:
                    axis.set(xlabel='X into trailer (m)',ylabel='Y left (m)',zlabel='Z up (m)',title=title,xlim=(-.2,.8),ylim=(-1.2,1.2),zlim=(0,2.5)); axis.set_box_aspect((1,2.4,2.5)); axis.view_init(elev=20,azim=-150)
                fig.tight_layout(); fig.savefig(dest/'depth_support_and_gt_world.png',dpi=140); plt.close(fig)
    (args.result_directory/'independent_final_face_accuracy.json').write_text(json.dumps(all_rows,indent=2))
    if panels:
        video=cv2.VideoWriter(str(args.result_directory/'metric_evidence_12s.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1296,972))
        for index in range(120): video.write(panels[min(len(panels)-1,index*len(panels)//120)])
        video.release()
    print(json.dumps({'validated_face_rows':sum('face_id' in row for row in all_rows),'rejected_face_rows':sum('rejected_face' in row for row in all_rows),'report':str(args.result_directory/'independent_final_face_accuracy.json')}))


if __name__=='__main__': main()
