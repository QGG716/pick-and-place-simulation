"""Inspect source CAD support-plane tolerance; never move or heal the source."""
from pathlib import Path
import argparse
import hashlib
import json
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'assets/robots/huayan_s50_2000/collision_repair'))
from inspect_missing_faces import read_parts, faces, bounds, xyz
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_VERTEX, TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopLoc import TopLoc_Location
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Pnt
from OCP.Precision import Precision
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_cad_model import read_obj,load_definition,transform_vertices


def inspect():
    base = ROOT/'assets/robots/huayan_s50_2000'
    measured = json.loads((base/'derived/interface_measurements.json').read_text())
    original = json.loads((base/'derived/cad_audit.json').read_text())
    part = next(p for p in original['parts'] if p['id']=='base_s50')
    shape = read_parts()[part['name']]
    y = measured['support_plane']['y_CAD_m']
    # Recreate the original conversion settings in memory only, retaining the
    # original source geometry and all historical exported mesh files.
    BRepMesh_IncrementalMesh(shape,.10,False,.15,True)
    rows=[]
    for index, face in enumerate(faces(shape)):
        vertices=[]; tolerances=[]
        ex=TopExp_Explorer(face,TopAbs_VERTEX)
        while ex.More():
            vertex=TopoDS.Vertex_s(ex.Current())
            vertices.append(xyz(BRep_Tool.Pnt_s(vertex)))
            tolerances.append(BRep_Tool.Tolerance_s(vertex)*.001)
            ex.Next()
        edges=[];ex=TopExp_Explorer(face,TopAbs_EDGE)
        while ex.More():
            edges.append(BRep_Tool.Tolerance_s(TopoDS.Edge_s(ex.Current()))*.001);ex.Next()
        bound=np.asarray(bounds(face))
        row=dict(face_index=index, bounds_CAD_m=bound.tolist(),
            exact_lower_y_minus_support_m=float(bound[0,1]-y),
            face_tolerance_m=float(BRep_Tool.Tolerance_s(face)*.001),
            maximum_vertex_tolerance_m=max(tolerances,default=None),
            maximum_edge_tolerance_m=max(edges,default=None),
            minimum_vertex_y_minus_support_m=min((v[1]-y for v in vertices),default=None))
        location=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,location)
        points=[xyz(tri.Node(i).Transformed(location.Transformation())) for i in range(1,tri.NbNodes()+1)] if tri else []
        row['minimum_recreated_mesh_y_minus_support_m']=min((v[1]-y for v in points),default=None)
        below=[v for v in points if v[1]-y < -1e-10]
        if below:row['recreated_mesh_nodes_below_support_CAD_m']=below
        if row['exact_lower_y_minus_support_m'] < 1e-8:
            row['vertices_CAD_m']=vertices
        rows.append(row)
    mesh_vertices,_=read_obj(ROOT/part['mesh'],part['mesh_sha256'])
    below_ids=np.flatnonzero(mesh_vertices[:,1]<y-1e-10)
    attributable=[]
    source_faces=faces(shape)
    for vertex_id in below_ids:
        point=mesh_vertices[vertex_id]
        candidates=[(np.linalg.norm(point-np.asarray(node)),row['face_index'],node)
            for row in rows for node in row.get('recreated_mesh_nodes_below_support_CAD_m',[])]
        if not candidates:raise ValueError('BELOW_SUPPORT_VERTEX_NOT_REPRODUCED')
        delta,face_id,node=min(candidates,key=lambda value:value[0])
        if delta>1e-8:raise ValueError('BELOW_SUPPORT_VERTEX_RECREATION_MISMATCH')
        face=source_faces[face_id];edges=[];ex=TopExp_Explorer(face,TopAbs_EDGE);edge_id=0
        probe=BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float,np.asarray(node)*1000))).Vertex()
        while ex.More():
            edge=TopoDS.Edge_s(ex.Current());query=BRepExtrema_DistShapeShape(probe,edge);query.Perform()
            if query.IsDone():
                edges.append(dict(edge_index_in_face=edge_id,distance_to_recreated_node_m=float(query.Value()*.001),
                    edge_tolerance_m=float(BRep_Tool.Tolerance_s(edge)*.001),
                    nearest_edge_point_CAD_m=xyz(query.PointOnShape2(1))))
            edge_id+=1;ex.Next()
        qualified=[edge for edge in edges if edge['distance_to_recreated_node_m']<=edge['edge_tolerance_m']+1e-12]
        face_query=BRepExtrema_DistShapeShape(probe,face);face_query.Perform()
        whole_query=BRepExtrema_DistShapeShape(probe,shape);whole_query.Perform()
        saved_probe=BRepBuilderAPI_MakeVertex(gp_Pnt(*map(float,point*1000))).Vertex()
        saved_query=BRepExtrema_DistShapeShape(saved_probe,shape);saved_query.Perform()
        support_distances=[]
        for supporting in measured['support_plane']['support_faces']:
            si=supporting['face_index'];sq=BRepExtrema_DistShapeShape(saved_probe,source_faces[si]);sq.Perform()
            if sq.IsDone():support_distances.append(dict(face_index=si,distance_m=float(sq.Value()*.001),
                nearest_point_CAD_m=xyz(sq.PointOnShape2(1))))
        attributable.append(dict(source_obj_vertex_index=int(vertex_id),source_vertex_CAD_m=point.tolist(),
            source_underreach_m=float(y-point[1]),source_face_index=face_id,recreated_node_CAD_m=node,
            recreated_node_underreach_m=float(y-node[1]),source_obj_rounding_error_m=float(delta),
            local_edge_evidence=edges,qualified_edge_indices=[edge['edge_index_in_face'] for edge in qualified],
            within_local_edge_tolerance=bool(qualified),
            distance_to_original_face_m=float(face_query.Value()*.001) if face_query.IsDone() else None,
            nearest_original_face_point_CAD_m=xyz(face_query.PointOnShape2(1)) if face_query.IsDone() else None,
            recreated_node_distance_to_whole_base_m=float(whole_query.Value()*.001) if whole_query.IsDone() else None,
            source_obj_vertex_distance_to_whole_base_m=float(saved_query.Value()*.001) if saved_query.IsDone() else None,
            nearest_whole_base_point_for_source_vertex_CAD_m=xyz(saved_query.PointOnShape2(1)) if saved_query.IsDone() else None,
            support_face_distances=support_distances))
    raw=(base/'source/S50.STEP').read_bytes()
    return dict(schema='s50_source_base_support_tolerance_v1',
        source_step_sha256=hashlib.sha256(raw).hexdigest(),
        method='BRep exact bounds excluding tolerance expansion; original CAD vertices and stored tolerances; metres',
        source_modified=False, support_y_CAD_m=y, faces=rows,
        minimum_bound_y_minus_support_m=min(r['exact_lower_y_minus_support_m'] for r in rows),
        minimum_vertex_y_minus_support_m=min(r['minimum_vertex_y_minus_support_m'] for r in rows),
        minimum_recreated_mesh_y_minus_support_m=min(r['minimum_recreated_mesh_y_minus_support_m'] for r in rows if r['minimum_recreated_mesh_y_minus_support_m'] is not None),
        recreated_meshing=dict(linear_deflection_m=.0001,angular_deflection_rad=.15,original_mesh_file_unchanged=True),
        source_base_mesh_path=part['mesh'],source_base_mesh_sha256=part['mesh_sha256'],
        below_support_vertex_attribution=attributable,
        occt_confusion_precision_m=float(Precision.Confusion_s()*.001),
        support_face_indices=[r['face_index'] for r in measured['support_plane']['support_faces']],
        collision_tolerance_authorized=False,
        scope='Source geometry/tolerance evidence only; does not authorize blanket base penetration')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/s50_task_collision_20261111/base-support-source.json')
    parser.add_argument('--source-output',type=Path,default=ROOT/'assets/robots/huayan_s50_2000/generated/mount_contact_source.json')
    args=parser.parse_args(); report=inspect()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    definition=load_definition(ROOT);link=definition['links'][0]
    local,_=read_obj(ROOT/link['visual_mesh']['path'],link['visual_mesh']['sha256'])
    attribution=report['below_support_vertex_attribution']
    for entry in attribution:
        entry['model_base_vertex_index']=entry['source_obj_vertex_index']
        entry['model_base_vertex_m']=local[entry['model_base_vertex_index']].tolist()
        entry['source_only_allowance_m']=entry['source_underreach_m']
    source=dict(schema='s50_mount_contact_source_v1',source_step_sha256=report['source_step_sha256'],
        source_base_mesh_path=report['source_base_mesh_path'],source_base_mesh_sha256=report['source_base_mesh_sha256'],
        model_base_mesh_path=link['visual_mesh']['path'],model_base_mesh_sha256=link['visual_mesh']['sha256'],
        source_measurements_sha256=hashlib.sha256((ROOT/'assets/robots/huayan_s50_2000/derived/interface_measurements.json').read_bytes()).hexdigest(),
        support_y_CAD_m=report['support_y_CAD_m'],support_model_base_z_m=0.,
        support_face_indices=report['support_face_indices'],below_support_vertices=attribution,
        all_below_plane_vertices_accounted_for=True,source_file_modified=False,
        local_tolerance_qualification_passed=all(e['within_local_edge_tolerance'] for e in attribution),
        source_underreach_is_local_tessellation_artifact_not_physical_mount_translation=True,
        exact_base_lower_y_minus_support_m=report['minimum_bound_y_minus_support_m'],
        occt_confusion_precision_m=report['occt_confusion_precision_m'],
        original_converter_linear_deflection_m=.0001,
        attribution_status='CAD_SOURCE_TESSELLATION_ARTIFACT_REPRODUCED_NOT_EDGE_TOLERANCE_PERMISSION',
        additional_penetration_allowance_m=0.,arbitrary_base_penetration_allowed=False,
        whole_link_collision_exemption_allowed=False,
        method='Original CAD face/node recreation and source-surface distances; reject global edge-tolerance shortcut; original mesh SHA fixed',
        scope='Source evidence only; supporting footprint/contact and all other collisions require separate runtime proof')
    args.source_output.parent.mkdir(parents=True,exist_ok=True)
    args.source_output.write_text(json.dumps(source,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('schema','minimum_bound_y_minus_support_m','minimum_vertex_y_minus_support_m')}))


if __name__=='__main__':main()
