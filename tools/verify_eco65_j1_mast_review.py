"""Cross-check four final composed USD readbacks and actual stopped Isaac renders."""
import argparse
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom
from unloading_sim.eco65.model import load_json,save_json
from unloading_sim.eco65.native_io import file_hash


def verify(folder):
    values=[];rows=[];images=[]
    for case in 'ABCD':
        directory=folder/('isaac_'+case);report=load_json(directory/'render_report.json')
        manifest=load_json(folder/'build'/('usd_'+case)/'workstation_manifest.json')
        if report['timeline_play_events'] or report['timeline_playing'] or report['timeline_time_s']!=0:
            raise ValueError('TIMELINE_NOT_STOPPED')
        stage=Usd.Stage.Open(str(directory/'eco65_desktop_warehouse.usda'));cache=UsdGeom.XformCache()
        path=manifest['mast']['usd_path'].replace('/Workstation','/World/Workstation',1)
        def matrix(p):return np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T
        link=matrix('/World/Workstation/Robot/link_1');mast=matrix(path);camera=matrix(path+'/CameraFrame')
        if str(stage.GetPrimAtPath(path).GetParent().GetPath())!='/World/Workstation/Robot/link_1':raise ValueError('WRONG_PARENT')
        local=np.linalg.inv(link)@mast;camera_local=np.linalg.inv(mast)@camera
        if not np.allclose(local,manifest['mast']['T_link1_mast'],atol=1e-10,rtol=0):raise ValueError('MOUNT_TRANSFORM_CHANGED')
        if not np.allclose(camera_local,manifest['mast']['T_mast_camera'],atol=1e-10,rtol=0):raise ValueError('CAMERA_LOCAL_TRANSFORM_CHANGED')
        arm=link[:3,:3]@manifest['mast']['arm_forward_link1'];axis=camera[:3,2].copy();arm[2]=axis[2]=0
        arm/=np.linalg.norm(arm);axis/=np.linalg.norm(axis)
        angle=float(np.degrees(np.arctan2(np.cross(arm,axis)[2],arm@axis)))
        if abs(angle-manifest['mast']['relative_heading_deg'])>1e-8:raise ValueError('USD_ANGLE_CHANGED')
        values.append(dict(link=link,mast=mast,camera=camera))
        rows.append(dict(case=case,joints_rad=manifest['home_q_rad'],signed_angle_deg=angle,
            local_mount_max_error=float(np.max(np.abs(local-manifest['mast']['T_link1_mast']))),
            isaac_version=report['isaac_installed_version'],timeline_time_s=report['timeline_time_s'],timeline_play_events=0,
            usd_path=str(directory/'eco65_desktop_warehouse.usda'),usd_sha256=file_hash(directory/'eco65_desktop_warehouse.usda')))
        for image in report['renders']:
            if file_hash(directory/image['file'])!=image['sha256']:raise ValueError('RENDER_HASH_CHANGED')
        if len(report['renders'])!=(4 if case=='A' else 2):raise ValueError('MISSING_DIAGNOSTIC_VIEW')
        from PIL import Image
        for p in sorted(directory.glob('*.png')):
            size=Image.open(p).size
            if size!=(1920,1080):raise ValueError('RENDER_DIMENSIONS_CHANGED')
            images.append(dict(path=p.relative_to(folder).as_posix(),sha256=file_hash(p),size=list(size)))
    for i,j,degree in [(1,0,15),(2,0,-15),(3,1,10)]:
        expected=np.zeros(6);expected[j]=np.radians(degree)
        if not np.allclose(np.array(rows[i]['joints_rad'])-rows[0]['joints_rad'],expected,atol=1e-10,rtol=0):
            raise ValueError('WRONG_STATIC_PERTURBATION')
    for i in [1,2]:
        delta=values[i]['link']@np.linalg.inv(values[0]['link'])
        for key in ['mast','camera']:
            error=float(np.max(np.abs(delta@values[0][key]-values[i][key])))
            rows[i][key+'_J1_covariance_error']=error
            if error>1e-10:raise ValueError('J1_CO_ROTATION_FAILED')
    for key in ['link','mast','camera']:
        error=float(np.max(np.abs(values[3][key]-values[0][key])))
        rows[3][key+'_J2_invariance_error']=error
        if error>1e-10:raise ValueError('J2_INVARIANCE_FAILED')
    result=dict(status=['CAMERA_AXIS_90DEG_CONSTRAINT_APPLIED','J1_CO_ROTATION_VERIFIED','NO_MOTION_EXECUTED'],
        dynamics='NOT_EVALUATED',scope='Four independent static USD scenes and actual Isaac renders; no transition paths or hardware mounting qualification',
        cases=rows,images=images)
    save_json(folder/'j1_usd_verification.json',result);print('FOUR_COMPOSED_USD_CASES_VERIFIED',len(images))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True)
    verify(p.parse_args().folder)
