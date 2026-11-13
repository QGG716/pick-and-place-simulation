"""Reuse the supplied Wantai STEP for the S50 direct-mount structures.

Only the three user-removed connector plates are omitted. Cups and inserts
retain their existing representation. No FANUC source geometry is changed.
"""
from pathlib import Path
import hashlib,json,sys
import numpy as np
import trimesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.tool_geometry import audit_tool_geometry
from unloading_sim.geometry import rotation_matrix_from_rpy


def main():
    coverage=audit_tool_geometry(ROOT)
    source=ROOT/'res/上海皖泰真空吸盘三分区.STEP'
    destination=ROOT/'assets/grippers/shanghai_wantai_three_zone/s50_direct_mount'
    intermediate=ROOT/'outputs/s50_direct_tool_source_meshes'
    destination.mkdir(parents=True,exist_ok=True);intermediate.mkdir(parents=True,exist_ok=True)
    reader=STEPControl_Reader()
    if reader.ReadFile(str(source))!=IFSelect_RetDone or reader.TransferRoots()<=0:raise RuntimeError('STEP_READ_FAILED')
    explorer=TopExp_Explorer(reader.OneShape(),TopAbs_SOLID);solids=[]
    while explorer.More():solids.append(explorer.Current());explorer.Next()
    if len(solids)!=coverage['solid_occurrence_count']:raise ValueError('SOURCE_SOLID_COUNT_CHANGED')
    rotation=np.asarray(coverage['rotation_step_from_tool']);flange=np.asarray(coverage['flange_origin_step_mm'])
    R=rotation_matrix_from_rpy(0,np.pi/2,0);arrays={};records=[]
    for i,solid_index in enumerate(coverage['rigid_solid_indices'][:coverage['historical_rigid_solid_count']]):
        if i in (11,12,13):continue
        solid=solids[solid_index];mesher=BRepMesh_IncrementalMesh(solid,.05,False,.08,False);mesher.Perform()
        if not mesher.IsDone():raise RuntimeError('SOURCE_MESH_FAILED:'+str(i))
        path=intermediate/f'tool_rigid_{i}.stl';writer=StlAPI_Writer();writer.ASCIIMode=False
        if not writer.Write(solid,str(path)):raise RuntimeError('SOURCE_MESH_EXPORT_FAILED:'+str(i))
        mesh=trimesh.load_mesh(path,process=True)
        # Exact same STEP -> original flange -> virtual TCP transform as the
        # existing tool bounds. The direct mount only changes flange->TCP.
        vertices=(((mesh.vertices-flange)@rotation*.001)-[.25,0,0])@R
        faces=np.asarray(mesh.faces,dtype=np.int32)
        if len(vertices)<3 or len(faces)==0 or not np.isfinite(vertices).all():raise ValueError('EMPTY_TOOL_PART')
        arrays[f'v_{i}']=vertices.astype(np.float64);arrays[f'f_{i}']=faces
        records.append(dict(body_id=f'tool_rigid_{i}',source_solid_index=solid_index,
            vertex_key=f'v_{i}',face_key=f'f_{i}',vertices=len(vertices),triangles=len(faces),
            local_bounds_m=[vertices.min(0).tolist(),vertices.max(0).tolist()]))
        print(i,len(vertices),len(faces),flush=True)
    mesh_path=destination/'structure_meshes.npz';np.savez_compressed(mesh_path,**arrays)
    result=dict(schema='s50_direct_mount_existing_tool_structure_meshes_v1',source_step_sha256=coverage['source_step_sha256'],
        source_analysis_sha256=coverage['analysis_sha256'],coordinate_frame='ORIGINAL_VIRTUAL_TCP_METRES',
        mesh_file=mesh_path.relative_to(ROOT).as_posix(),mesh_sha256=hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
        parts=records,removed_body_ids=['tool_rigid_11','tool_rigid_12','tool_rigid_13'],
        linear_deflection_m=.00005,angular_deflection_rad=.08,
        scope='BASIC_KINEMATIC_GEOMETRY_REFINEMENT_NOT_CERTIFICATION')
    (destination/'structure_meshes.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print('Prepared',len(records),'retained structure meshes; original 72 cups and 72 rigid inserts retained',flush=True)


if __name__=='__main__':main()
