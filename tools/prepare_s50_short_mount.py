"""Prepare a compact S50 mount while preserving source front-side openings."""
from pathlib import Path
import sys,json,hashlib,argparse,numpy as np,trimesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Pnt,gp_Vec,gp_Trsf
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.tool_geometry import audit_tool_geometry
from unloading_sim.geometry import rotation_matrix_from_rpy


def main():
    p=argparse.ArgumentParser();p.add_argument('--standoff',type=float,default=.105);a=p.parse_args()
    if not .05<=a.standoff<=.15:raise ValueError('SHORT_MOUNT_INTERVAL_OUTSIDE_DESIGN_RANGE')
    coverage=audit_tool_geometry(ROOT);reader=STEPControl_Reader();source=ROOT/'res/上海皖泰真空吸盘三分区.STEP'
    if reader.ReadFile(str(source))!=IFSelect_RetDone or reader.TransferRoots()<=0:raise RuntimeError('STEP_READ_FAILED')
    explorer=TopExp_Explorer(reader.OneShape(),TopAbs_SOLID);solids=[]
    while explorer.More():solids.append(explorer.Current());explorer.Next()
    destination=ROOT/f'assets/grippers/shanghai_wantai_three_zone/s50_short_mount_{round(a.standoff*1000)}mm'
    destination.mkdir(parents=True,exist_ok=True);intermediate=ROOT/'outputs/s50_direct_tool_source_meshes';intermediate.mkdir(parents=True,exist_ok=True)
    R=rotation_matrix_from_rpy(0,np.pi/2,0);rotation=np.asarray(coverage['rotation_step_from_tool']);flange=np.asarray(coverage['flange_origin_step_mm'])
    delta=.15-a.standoff;records=[];arrays={}
    for i,name in [(11,'tool_s50_mount_side_1'),(12,'tool_s50_mount_side_2'),(13,'tool_s50_mount_flange')]:
        solid=solids[coverage['rigid_solid_indices'][i]]
        if i!=13:
            # Trim only the rear of each existing 140 mm side plate. Its front
            # CAD openings and interfaces with the tool remain in place.
            lower_y=flange[1]+(.01+delta)*1000
            box=BRepPrimAPI_MakeBox(gp_Pnt(-1000,lower_y,-1000),2000,2000,2000).Shape()
            operation=BRepAlgoAPI_Common(solid,box);operation.Build()
            if not operation.IsDone():raise RuntimeError('MOUNT_TRIM_FAILED')
            solid=operation.Shape()
        else:
            move=gp_Trsf();move.SetTranslation(gp_Vec(0,delta*1000,0));solid=BRepBuilderAPI_Transform(solid,move,True).Shape()
        mesh=BRepMesh_IncrementalMesh(solid,.05,False,.08,False);mesh.Perform()
        path=intermediate/(name+'.stl');writer=StlAPI_Writer();writer.ASCIIMode=False
        if not writer.Write(solid,str(path)):raise RuntimeError('MOUNT_MESH_FAILED')
        m=trimesh.load_mesh(path,process=True);v=(((m.vertices-flange)@rotation*.001)-[.25,0,0])@R;f=np.asarray(m.faces,dtype=np.int32)
        arrays['v_'+str(i)]=v;arrays['f_'+str(i)]=f
        lo=v.min(0);hi=v.max(0)
        records.append(dict(body_id=name,source_body_id='tool_rigid_'+str(i),vertex_key='v_'+str(i),face_key='f_'+str(i),
            local_box_virtual_tcp_m=np.r_[(lo+hi)/2,hi-lo].tolist(),triangles=len(f)))
    mesh_path=destination/'mount_meshes.npz';np.savez_compressed(mesh_path,**arrays)
    result=dict(schema='s50_short_mount_v1',standoff_m=a.standoff,source_step_sha256=coverage['source_step_sha256'],
        mesh_file=mesh_path.relative_to(ROOT).as_posix(),mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),parts=records,
        source_original_standoff_m=.15,removed_old_extension_m=.2,
        scope='USER_AUTHORIZED_SHORT_IDEAL_RIGID_INSTALLATION_NOT_MANUFACTURING_QUALIFICATION')
    (destination/'mount_meshes.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8');print(json.dumps(result),flush=True)


if __name__=='__main__':main()
