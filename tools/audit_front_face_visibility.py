"""Evaluate equal-area physical front-face samples against rendered first hits.

Evaluation only: simulator entities never feed perception or association.
Reports sampled surface area fractions, not semantic-pixel object counts.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-directory',type=Path,required=True)
    parser.add_argument('--output-directory',type=Path,required=True)
    parser.add_argument('--samples-per-edge',type=int,default=128)
    args=parser.parse_args()
    if args.samples_per_edge<16: raise ValueError('at least 16 samples per edge')
    args.output_directory.mkdir(parents=True,exist_ok=False)
    root=args.scene_directory; n=args.samples_per_edge
    truth=json.loads((root/'gt_annotations.json').read_text())['objects']
    uv=(np.arange(n)+.5)/n-.5; u,v=np.meshgrid(uv,uv)
    rows={}; samples={}; manifest={}; images=[]
    for module,source in [('module_0_upper',root),('module_1_lower',root/'modules/module_1_lower')]:
        info=json.loads((source/'camera_info.json').read_text()); K=np.asarray(info['K']).reshape(3,3)
        meta=json.loads((source/'capture_metadata.json').read_text()); T=np.asarray(meta['T_W_C_at_capture'])
        labels=json.loads((source/'instance_segmentation_info.json').read_text())['idToLabels']
        ids=np.load(source/'first_hit_instance_ids.npy',allow_pickle=False)
        depth=np.load(source/'metric_depth_m.npy',allow_pickle=False)
        if ids.shape!=depth.shape: raise ValueError('first-hit/depth registration shape mismatch')
        image=cv2.imread(str(source/'sensor_rgb.png')); h,w=depth.shape
        for gt in truth:
            entity=gt['simulation_object_id']; M=np.asarray(gt['T_W_object']); dims=np.asarray(gt['full_dimensions_m'])
            # Physical face whose outward normal points most toward world -X.
            axis=int(np.argmax(np.abs(M[0,:3]))); sign=-1 if M[0,axis]>0 else 1
            tangents=[j for j in range(3) if j!=axis]
            local=np.zeros((n*n,3)); local[:,axis]=sign*dims[axis]/2
            local[:,tangents[0]]=u.ravel()*dims[tangents[0]]; local[:,tangents[1]]=v.ravel()*dims[tangents[1]]
            points=local@M[:3,:3].T+M[:3,3]; camera=(points-T[:3,3])@T[:3,:3]
            pixels=camera@K.T; xy=np.rint(pixels[:,:2]/pixels[:,2:]).astype(int)
            inside=(camera[:,2]>0)&(xy[:,0]>=0)&(xy[:,0]<w)&(xy[:,1]>=0)&(xy[:,1]<h)
            measured=np.full(n*n,np.nan); hit=np.zeros(n*n,dtype=np.uint32)
            measured[inside]=depth[xy[inside,1],xy[inside,0]]; hit[inside]=ids[xy[inside,1],xy[inside,0]]
            prims=np.asarray([str(labels.get(str(i),'UNLABELLED_INSTANCE_'+str(i))) for i in hit])
            own=np.asarray([p==gt['prim_path'] or p.startswith(gt['prim_path']+'/') for p in prims])
            visible=inside&own&np.isfinite(measured)&(np.abs(measured-camera[:,2])<=.005)
            blocked=inside&np.isfinite(measured)&(measured<camera[:,2]-.005)
            histogram=Counter(prims[blocked]); top=[{'prim':p,'sample_count':c,'physical_face_area_fraction':c/(n*n)} for p,c in histogram.most_common()]
            rows.setdefault(entity,{'front_axis':axis,'front_sign':sign,'physical_front_area_m2':float(np.prod(dims[tangents])),'modules':{}})['modules'][module]={
                'visible_area_fraction':float(visible.mean()),'in_frustum_area_fraction':float(inside.mean()),
                'blocked_area_fraction':float(blocked.mean()),'unresolved_area_fraction':float((inside&~visible&~blocked).mean()),
                'first_blocking_prims':top,'capture_id':meta['capture_id'],
            }
            samples[entity+'__'+module]=visible.reshape(n,n)
            if entity.startswith('carton_l00_'):
                for flag,color in [(blocked,(0,0,255)),(visible,(0,255,0))]:
                    pts=xy[flag][::8]
                    image[pts[:,1],pts[:,0]]=color
                at=xy[len(xy)//2]; at=np.clip(at,[0,0],[w-350,h-20])
                cv2.putText(image,f'{entity} {visible.mean():.1%}',tuple(at),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2)
        cv2.putText(image,f'{module}: bottom-front first hits; green visible / red blocked',(30,45),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
        cv2.imwrite(str(args.output_directory/f'{module}_bottom_first_hits.png'),image); images.append(image)
        for name in ('camera_info.json','capture_metadata.json','gt_annotations.json','first_hit_instance_ids.npy','metric_depth_m.npy','instance_segmentation_info.json','sensor_rgb.png'):
            path=source/name; manifest[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    for entity,row in rows.items():
        union=np.logical_or(*(samples[entity+'__'+m] for m in ('module_0_upper','module_1_lower')))
        row['union_visible_front_area_fraction']=float(union.mean())
        row['union_remaining_blind_area_fraction']=1-float(union.mean())
    report={'schema_version':'sampled_front_face_first_hit_v1','evaluation_only':True,'sample_grid':[n,n],
            'depth_consistency_tolerance_m':.005,'method':'equal-area physical front samples projected to nearest registered first-hit pixel; uncertainty at silhouette pixels',
            'scene_geometry_hidden':False,'objects':rows,'input_sha256':manifest}
    (args.output_directory/'front_face_visibility.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(args.output_directory/'front_face_visible_samples.npz',**samples)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(12,7))
    for ax,module in zip(axes,('module_0_upper','module_1_lower','union')):
        matrix=np.zeros((8,5))
        for entity,row in rows.items():
            layer=int(entity.split('_l')[1].split('_')[0]); col=int(entity.rsplit('_c',1)[1])
            matrix[layer,col]=row['union_visible_front_area_fraction'] if module=='union' else row['modules'][module]['visible_area_fraction']
        ax.imshow(matrix,origin='lower',vmin=0,vmax=1,cmap='RdYlGn'); ax.set_title(module); ax.set_xlabel('column'); ax.set_ylabel('layer')
        for layer in range(8):
            for col in range(5): ax.text(col,layer,f'{matrix[layer,col]:.0%}',ha='center',va='center',fontsize=9)
    fig.suptitle('Sampled physical FRONT area visibility (all robot and scene entities present)')
    fig.tight_layout(); fig.savefig(args.output_directory/'front_face_area_visibility.png',dpi=160); plt.close(fig)
    print(json.dumps({e:r for e,r in rows.items() if r['union_visible_front_area_fraction']==0},indent=2))


if __name__=='__main__': main()
