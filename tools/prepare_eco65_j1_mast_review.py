"""Bind a fixed accessory and four independent poses to an existing static layout."""
import argparse
import copy
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import ROOT,load_json,save_json,robot_for
from unloading_sim.eco65.native_io import file_hash
from unloading_sim.moveit2_backend import digest
from build_eco65_full_face_layout import verify_static
from eco65_mast_attachment import evaluate


def prepare(snapshot_path,pose_path,config_path,out):
    out=out.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    old=verify_static(load_json(snapshot_path));pose=load_json(pose_path);config=load_json(config_path)
    if pose['status']!='STATIC_POSE_ONLY' or pose['source_snapshot_fingerprint']!=old['fingerprint']:
        raise ValueError('DISPLAY_POSE_SNAPSHOT_MISMATCH')
    if pose['identity']!=old['native_model']['init']['identity']:raise ValueError('DISPLAY_MODEL_IDENTITY_MISMATCH')
    mast=old['scene']['mast']
    for key in ['source_sha256','cad_manifest_sha256']:
        if config[key]!=mast[key]:raise ValueError('MAST_IDENTITY_MISMATCH:'+key)
    new=copy.deepcopy(old);new.pop('fingerprint');scene=new['scene'];scene.pop('scene_fingerprint')
    scene['mast'].pop('T_D_mast',None);scene['mast'].update(config)
    seat=config.get('mount_interface',{}).get('seat')
    if seat:
        path=(ROOT/seat['mesh']).resolve();path.relative_to(ROOT/'assets/mast')
        if file_hash(path)!=seat['sha256']:raise ValueError('MOUNTING_SEAT_CHANGED')
        new['asset_sha256'][seat['mesh']]=seat['sha256']
    scene['scene_fingerprint']=digest(scene)
    new['config_sha256'][config_path.resolve().relative_to(ROOT).as_posix()]=file_hash(config_path)
    new['source_static_snapshot_fingerprint']=old['fingerprint'];new['fingerprint']=digest(new)
    verify_static(new);save_json(out/'snapshot.json',new)
    robot=robot_for(new['tool'],scene);rows=[]
    for label,index,delta in [('A',0,0),('B',0,15),('C',0,-15),('D',1,10)]:
        q=np.array(pose['home_q']);q[index]+=np.radians(delta)
        frames=robot.named_link_frames(q);row=evaluate(frames['link_1'],scene['mast'])
        row.update(case=label,home_q=q.tolist(),changed_joint=index+1,delta_deg=delta)
        item=copy.deepcopy(pose)
        item.update(home_q=q.tolist(),source_snapshot_fingerprint=new['fingerprint'],
            source='Independent static case '+label+' derived from frozen display pose; not a new HOME acceptance',
            source_display_pose=dict(path=pose_path.resolve().relative_to(ROOT).as_posix(),sha256=file_hash(pose_path)),
            static_case=label,new_scene_status='STATIC_INSTALLATION_REVIEW_ONLY')
        item.pop('candidate_report',None)
        save_json(out/('pose_'+label+'.json'),item);rows.append(row)
    a=rows[0]
    for row in rows:
        relative=np.linalg.inv(np.array(row['T_D_link1']))@np.array(row['T_D_mast'])
        row['fixed_attachment_error']=float(np.abs(relative-np.array(config['T_link1_mast'])).max())
        if row['fixed_attachment_error']>1e-10:raise ValueError('ATTACHMENT_NOT_RIGID')
    for row in rows[1:3]:
        delta=np.array(row['T_D_link1'])@np.linalg.inv(a['T_D_link1'])
        row['J1_covariance_error']=float(np.max(np.abs(delta@np.array(a['T_D_camera'])-row['T_D_camera'])))
    rows[3]['J2_invariance_error']=float(np.max(np.abs(np.array(rows[3]['T_D_camera'])-a['T_D_camera'])))
    if rows[3]['J2_invariance_error']>1e-10:raise ValueError('CAMERA_FOLLOWS_J2')
    save_json(out/'four_case_fk.json',dict(status='J1_CO_ROTATION_VERIFIED',method='official URDF FK; independent static states',
        cases=rows,motion='NO_MOTION_EXECUTED',dynamics='NOT_EVALUATED',snapshot=new['fingerprint']))
    print('FOUR_STATIC_CASES_PREPARED',new['fingerprint'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ['snapshot','pose','config','output']:p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();prepare(a.snapshot,a.pose,a.config,a.output)
