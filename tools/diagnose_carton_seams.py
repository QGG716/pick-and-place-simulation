"""Native crops and RGB/depth profiles at GT-located contacts, evaluation only."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from run_carton_appearance_ab import folder
from run_isaac_rgbd_geometry import _worker_artifacts
from unloading_perception.final_geometry import project


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--historical',type=Path,required=True)
    p.add_argument('--evaluation',type=Path,required=True);args=p.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    reports=[]
    for module,identity in [('module_1_lower',10),('module_0_upper',6),('module_1_lower',27)]:
        old=args.historical/'FULL_STACK_NOMINAL'
        if module=='module_1_lower':old=old/'modules'/module
        masks=np.load(_worker_artifacts(old)['cargo_masks.npz']['path']);mask=masks['masks'][list(masks['mask_ids']).index(identity)]
        y,x=np.nonzero(mask);x0,x1=max(0,x.min()-35),min(2592,x.max()+36);y0,y1=max(0,y.min()-45),min(1944,y.max()+46)
        src=folder(args.capture,'A',module);gtmask=np.load(src/'gt_instance_masks.npz')
        truth=json.loads((src/'gt_annotations.json').read_text())['objects'];byid={v['simulation_object_id']:v for v in truth}
        target=max(gtmask.files,key=lambda k:np.count_nonzero(gtmask[k]&mask));g=byid[target];G=np.asarray(g['T_W_object']);dims=np.asarray(g['full_dimensions_m'])
        neighbors=[]
        for h in truth:
            H=np.asarray(h['T_W_object']);delta=(H[:3,3]-G[:3,3])@G[:3,:3]
            if h['simulation_object_id']!=target and np.linalg.norm(delta[:2])<1e-6:
                gap=abs(delta[2])-(dims[2]+h['full_dimensions_m'][2])/2
                if abs(gap)<1e-6:neighbors.append((h,delta[2],gap))
        camera=json.loads((src/'camera_info.json').read_text());T=np.asarray(camera['T_W_C']);K=np.asarray(camera['K'])
        row={'historical_module':module,'historical_mask_id_for_location_only':identity,'target_gt':target,
             'native_crop_xyxy':[int(x0),int(y0),int(x1),int(y1)],'contacts':[{'neighbor':h['simulation_object_id'],'gap_m':float(gap)} for h,delta,gap in neighbors]}
        panels=[];signals={}
        # Select an actual vertical contact whose projection is visible and within this image.
        for h,delta,gap in sorted(neighbors,key=lambda v:v[1]):
            point=G[:3,3]+G[:3,:3]@np.array([-dims[0]/2,0,np.sign(delta)*dims[2]/2])
            q=project([(point-T[:3,3])@T[:3,:3]],K)[0]
            ix,iy=np.rint(q).astype(int)
            both_visible=(h['simulation_object_id'] in gtmask.files and 15<ix<2577 and 15<iy<1929
                          and gtmask[target][iy-5:iy+6,ix-5:ix+6].any()
                          and gtmask[h['simulation_object_id']][iy-5:iy+6,ix-5:ix+6].any())
            if both_visible:
                row['profile_contact']={'neighbor':h['simulation_object_id'],'world_xyz_m':point.tolist(),'pixel_xy':q.tolist(),'gap_m':float(gap)};break
        for group in ('A','B'):
            source=folder(args.capture,group,module);rgb=cv2.imread(str(source/'sensor_rgb.png'));depth=np.load(source/'metric_depth_m.npy')
            sam=cv2.imread(str(args.evaluation/f'{group}_{module}_sam.png'));faces=cv2.imread(str(args.evaluation/f'{group}_{module}_faces.png'))
            # Same fixed depth range in both rows, no independently scaled heatmaps.
            visual=np.zeros(depth.shape,np.uint8);valid=np.isfinite(depth)&(depth>0);visual[valid]=np.clip(depth[valid]/4*255,0,255).astype(np.uint8)
            heat=cv2.applyColorMap(visual,cv2.COLORMAP_VIRIDIS)
            tile=np.hstack([im[y0:y1,x0:x1] for im in (rgb,heat,sam,faces)])
            label=np.zeros((30,tile.shape[1],3),np.uint8);cv2.putText(label,group+' | RAW RGB | depth 0-4m | fresh SAM | final patches + GT',(8,21),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
            panels.append(np.vstack((label,tile)))
            if 'profile_contact' in row:
                point=np.asarray(row['profile_contact']['world_xyz_m']);q=np.asarray(row['profile_contact']['pixel_xy'])
                vertical=project([((point+.02*G[:3,2])-T[:3,3])@T[:3,:3]],K)[0]-q;vertical/=np.linalg.norm(vertical)
                tangent=np.array([-vertical[1],vertical[0]])
                offsets=np.arange(-12,13);xy=q+offsets[:,None,None]*vertical+np.arange(-4,5)[None,:,None]*tangent
                xx=np.rint(xy[...,0]).astype(int);yy=np.rint(xy[...,1]).astype(int)
                colors=rgb[yy,xx,::-1].mean(axis=1);z=depth[yy,xx].astype(float);z[~np.isfinite(z)]=np.nan
                signals[group]={'offset_px':offsets.tolist(),'mean_rgb_0_255':colors.tolist(),'median_optical_z_m':np.nanmedian(z,axis=1).tolist()}
        prefix=module+'_historical_'+str(identity)
        cv2.imwrite(str(args.evaluation/(prefix+'_native_AB.png')),np.vstack(panels))
        if signals:
            fig,axes=plt.subplots(2,1,figsize=(8,6),sharex=True,layout='constrained')
            for group,s in signals.items():
                color=np.asarray(s['mean_rgb_0_255']);luma=color@np.array([.2126,.7152,.0722]);z=np.asarray(s['median_optical_z_m'])
                axes[0].plot(s['offset_px'],luma,label=group);axes[1].plot(s['offset_px'],z*1000,label=group)
                s['luminance_range_0_255']=float(np.ptp(luma));s['depth_range_m']=float(np.nanmax(z)-np.nanmin(z))
                x=np.asarray(s['offset_px']);left=x<=-3;right=x>=3
                s['linear_side_fit_luminance_jump_at_contact']=float(np.polyfit(x[right],luma[right],1)[1]-np.polyfit(x[left],luma[left],1)[1])
                s['linear_side_fit_depth_jump_m']=float(np.polyfit(x[right],z[right],1)[1]-np.polyfit(x[left],z[left],1)[1])
            for ax in axes:ax.axvline(0,color='gray',ls='--');ax.legend();ax.grid(alpha=.3)
            axes[0].set(ylabel='RGB luminance (0-255)',title=prefix+' | GT contact at 0; evaluation only')
            axes[1].set(ylabel='Optical Z (mm)',xlabel='Signed pixel offset across vertical contact')
            fig.savefig(args.evaluation/(prefix+'_profile.png'),dpi=140);plt.close(fig)
        row['profiles']=signals;reports.append(row)
    (args.evaluation/'seam_diagnostics.json').write_text(json.dumps(reports,indent=2))
    print(json.dumps([{k:v for k,v in r.items() if k!='profiles'} for r in reports]),flush=True)


if __name__=='__main__':main()
