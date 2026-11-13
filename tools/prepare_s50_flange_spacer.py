"""Move the existing compact mount away from the wrist with a rigid flange spacer."""
from pathlib import Path
import argparse,json,hashlib,numpy as np,trimesh
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--length',type=float,default=.05);a=p.parse_args()
    if not 0<a.length<=.1 or abs(a.length*1000-round(a.length*1000))>1e-9:raise ValueError('INVALID_SPACER_LENGTH')
    source=ROOT/'assets/grippers/shanghai_wantai_three_zone/s50_short_mount_105mm/mount_meshes.json'
    result=json.loads(source.read_text(encoding='utf8'));mesh=ROOT/result['mesh_file']
    if hashlib.sha256(mesh.read_bytes()).hexdigest()!=result['mesh_sha256']:raise ValueError('SOURCE_MOUNT_CHANGED')
    with np.load(mesh,allow_pickle=False) as data:arrays={key:data[key] for key in data.files}
    # A 118 mm cylindrical envelope matches the S50 flange body's CAD Y span.
    # This is an explicit ideal adapter, with no manufacturing qualification.
    definition=json.loads((ROOT/'assets/robots/huayan/s50_1_kinematic/model.json').read_text(encoding='utf8'))
    bounds=np.asarray(next(x for x in definition['links'] if x['name']=='s50_link_6')['visual_mesh']['local_bounds_m'])
    radius=float((bounds[1,1]-bounds[0,1])/2)
    m=trimesh.creation.cylinder(radius=radius,height=a.length,sections=96)
    m.apply_translation([0,0,-.205-a.length/2]);lo=m.vertices.min(0);hi=m.vertices.max(0)
    arrays['v_spacer']=m.vertices;arrays['f_spacer']=np.asarray(m.faces,dtype=np.int32)
    result['parts'].append(dict(body_id='tool_s50_mount_spacer',vertex_key='v_spacer',face_key='f_spacer',
        local_box_virtual_tcp_m=np.r_[(lo+hi)/2,hi-lo].tolist(),triangles=len(m.faces),
        geometry='IDEAL_COAXIAL_CYLINDER_MATCHING_S50_FLANGE_BODY_DIAMETER',radius_m=radius,length_m=a.length))
    total=.105+a.length
    destination=source.parent.parent/f's50_short_mount_{round(total*1000)}mm_spacer{round(a.length*1000)}mm'
    destination.mkdir(parents=True,exist_ok=True);output=destination/'mount_meshes.npz';np.savez_compressed(output,**arrays)
    result.update(standoff_m=total,flange_spacer_m=a.length,unchanged_compact_mount_standoff_m=.105,
        source_compact_mount_manifest_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        mesh_file=output.relative_to(ROOT).as_posix(),mesh_sha256=hashlib.sha256(output.read_bytes()).hexdigest())
    (destination/'mount_meshes.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(standoff_m=total,flange_spacer_m=a.length,diameter_m=2*radius,path=str(destination))))


if __name__=='__main__':main()
