"""Check the new rigid mount against retained tool hardware and its own flange.

The other moving robot links are checked at every state by SimCollision. These
additional pairs are rigidly fixed, so their relative fit needs one pose only.
"""
from pathlib import Path
import sys,argparse,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,tool_virtual,save
from unloading_sim.s50_sim_collision import SimCollision,bounds
from unloading_sim.s50_collision_diagnostics import _world_geometry,coal_pair_query
from unloading_sim.pinocchio_backend import _transform


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--config',type=Path);a=p.parse_args()
    c=make_context(ROOT,a.output.parent/'fit-model',a.config);check=SimCollision(c);q=np.deg2rad([0,-90,0,-90,0,0]);V=tool_virtual(c,q)
    parts=[]
    for part in check.tool:
        if part['kind']!='rigid':continue
        T=V.copy();T[:3,3]=V[:3,:3]@part['center']+V[:3,3]
        parts.append(dict(name=part['name'],shape=part['shape'],T=T,bounds=bounds((-part['half'],part['half']),T)))
    flange=[]
    for i,g in enumerate(_world_geometry(check.backend,q)):
        if g['link']=='s50_link_6':flange.append(dict(name=g['geometry'],shape=g['shape'],T=g['world'],bounds=bounds(check.robot_local[i],g['world'])))
    mount=[x for x in parts if x['name'].startswith('tool_s50_mount_')];original=[x for x in parts if not x['name'].startswith('tool_s50_mount_')]
    rows=[]
    pairs=[(x,y) for x in original for y in flange]+[(x,y) for x in mount for y in original if y['name']!='tool_rigid_0']
    for x,y in pairs:
        if not check.near(x,y,.02):continue
        hit=check.query(x,y);design_contact=None
        if x['name']=='tool_s50_mount_side_1' and y['name'] in ['tool_rigid_20','tool_rigid_21']:
            # These two original fittings are seated on the unchanged outer
            # side face at virtual Y=-110 mm. Restrict contact to that face
            # and each original fitting's bounds; do not exempt the whole pair.
            row=np.asarray(c.intrinsic['rigid_local_boxes_virtual_tcp_m'][c.intrinsic['rigid_body_ids'].index(y['name'])])
            def interface(point):
                local=V[:3,:3].T@(point-V[:3,3])
                return abs(local[1]+.11)<1e-5 and np.all(abs(local-row[:3])<=row[3:]/2+1e-5)
            if hit is not None and check.query(x,y,connection=interface) is None:
                design_contact='ORIGINAL_FITTING_SEATED_ON_UNCHANGED_SIDE_FACE';hit=None
        tx=_transform(check.coal,x['T'][:3,:3],x['T'][:3,3]);ty=_transform(check.coal,y['T'][:3,:3],y['T'][:3,3])
        result=coal_pair_query(check.coal,x['shape'],tx,y['shape'],ty)
        rows.append(dict(pair=[x['name'],y['name']],intersection=hit,design_contact=design_contact,measurement=result))
    result=dict(status='PASS' if all(x['intersection'] is None for x in rows) else 'FAIL',standoff_m=c.intrinsic['mount_standoff_m'],
        scope='RIGIDLY_FIXED_MOUNT_HARDWARE_AND_OWN_FLANGE_PAIRS',
        design_contacts=['Compact mount front to original backplate',
            'Coaxial spacer end planes to S50 flange and compact flange plate' if c.intrinsic.get('flange_spacer_m',0.) else 'Compact flange plate to S50 flange',
            'Between the three compact mount parts'],
        checked_near_pairs=rows,far_pair_aabb_exclusion_m=.02,total_pairs=len(pairs))
    save(a.output,result);print(result['status'],len(rows),'near pairs of',len(pairs),flush=True)
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
