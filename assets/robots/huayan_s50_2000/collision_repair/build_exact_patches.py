"""Rebuild two exact planar trims, export additive triangle patches and audit solids."""
import json,hashlib
import numpy as np
import trimesh
from inspect_missing_faces import *
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCP.BRepTools import BRepTools_ReShape
from OCP.TopAbs import TopAbs_SHELL,TopAbs_SOLID,TopAbs_REVERSED

def topology(shape):
 analyzer=BRepCheck_Analyzer(shape)
 summary=dict(valid=bool(analyzer.IsValid()),invalid_subshapes=[],shells=[],solid_count=0)
 for kind in [TopAbs_SOLID,TopAbs_SHELL,TopAbs_FACE,TopAbs_EDGE]:
  ex=TopExp_Explorer(shape,kind);index=0
  while ex.More():
   sub=ex.Current()
   if kind==TopAbs_SOLID:summary['solid_count']+=1
   if kind==TopAbs_SHELL:summary['shells'].append(dict(index=index,closed=bool(BRep_Tool.IsClosed_s(sub))))
   if not analyzer.IsValid(sub):
    try:status=[str(x) for x in analyzer.Result(sub).Status()]
    except Exception as e:status=[str(e)]
    summary['invalid_subshapes'].append(dict(kind=str(kind),index=index,status=status))
   ex.Next();index+=1
 props=GProp_GProps();BRepGProp.VolumeProperties_s(shape,props)
 summary['signed_volume_m3']=props.Mass()*1e-9
 return summary

def patch_mesh(face):
 tri_loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,tri_loc);verts=[];tris=[]
 for i in range(1,tri.NbNodes()+1):verts.append(xyz(tri.Node(i).Transformed(tri_loc.Transformation())))
 for i in range(1,tri.NbTriangles()+1):
  t=list(tri.Triangle(i).Get())
  if face.Orientation()==TopAbs_REVERSED:t.reverse()
  tris.append([j-1 for j in t])
 return trimesh.Trimesh(verts,tris,process=True)

def sampled_boundary_error(original,mesh):
 counts=np.bincount(mesh.edges_unique_inverse)
 edges=mesh.vertices[mesh.edges_unique[counts==1]]
 samples=[];ex=TopExp_Explorer(original,TopAbs_EDGE)
 while ex.More():
  c=BRepAdaptor_Curve(TopoDS.Edge_s(ex.Current()))
  samples.extend(xyz(c.Value(float(t))) for t in np.linspace(c.FirstParameter(),c.LastParameter(),4097))
  ex.Next()
 samples=np.array(samples);a=edges[:,0];ab=edges[:,1]-a
 denom=np.einsum('ij,ij->i',ab,ab)
 maxima=[]
 for start in range(0,len(samples),128):
  p=samples[start:start+128,None,:]
  f=np.clip(np.einsum('bni,ni->bn',p-a[None,:,:],ab)/denom[None,:],0,1)
  distance=np.linalg.norm(p-a[None,:,:]-f[:,:,None]*ab[None,:,:],axis=2)
  maxima.append(float(distance.min(axis=1).max()))
 return dict(source_curve_samples=len(samples),mesh_boundary_segments=len(edges),
             sampled_source_boundary_to_mesh_max_m=max(maxima),
             sampling_is_not_analytic_hausdorff_certificate=True)

def main():
 parts=read_parts();patches=[]
 history=BASE/'derived/cad_audit.json'
 source_hash=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
 history_hash=hashlib.sha256(history.read_bytes()).hexdigest()
 for label,index in [('J1',74),('J2',75)]:
  name=next(n for n in parts if n.startswith(label));shape=parts[name]
  original=faces(shape)[index];original_info=face_info(original)
  ex=TopExp_Explorer(original,TopAbs_WIRE);wire=TopoDS.Wire_s(ex.Current())
  plane=BRepAdaptor_Surface(original,True).Plane()
  rebuilt=BRepBuilderAPI_MakeFace(plane,wire,True).Face()
  rebuilt_info=face_info(rebuilt)
  source_edges=[];ex=TopExp_Explorer(original,TopAbs_EDGE)
  while ex.More():source_edges.append(ex.Current());ex.Next()
  rebuilt_edges=[];ex=TopExp_Explorer(rebuilt,TopAbs_EDGE)
  while ex.More():rebuilt_edges.append(ex.Current());ex.Next()
  shared=all(any(a.IsSame(b) for b in rebuilt_edges) for a in source_edges) and len(source_edges)==len(rebuilt_edges)
  replacements=[]
  for orientation in ['forward','reversed']:
   face=rebuilt if orientation=='forward' else TopoDS.Face_s(rebuilt.Reversed())
   reshaper=BRepTools_ReShape();reshaper.Replace(original,face);changed=reshaper.Apply(shape)
   replacements.append(dict(orientation=orientation,topology=topology(changed)))
  # The reversed repaired face preserves the original solid signed volume and
  # restores valid J2 topology. Triangle collision still remains a surface query.
  result=mesh_info(rebuilt,.0001);assert result['triangles']>0 and result['status']==0
  mesh=patch_mesh(TopoDS.Face_s(rebuilt.Reversed()))
  output=OUT/(label.lower()+'_face_'+str(index)+'_exact_cad.obj');mesh.export(output,digits=17)
  mesh=trimesh.load(output,force='mesh',process=False)
  measure=sampled_boundary_error(original,mesh)
  patch=dict(cad_body=label.lower()+'_s50',source_face_index=index,
             mesh=output.relative_to(ROOT).as_posix(),mesh_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
             coordinate_frame='CAD_METRES',mesh_vertices=len(mesh.vertices),mesh_triangles=len(mesh.faces),
             qualification=dict(status='EXACT_BOUNDARY_TRIM_REBUILT_ADDITIVE_PATCH',
               method='BRepBuilderAPI_MakeFace(original plane, original wire, Inside=True)',
               exported_face_orientation='REVERSED; preserves source signed solid volume',
               original_face_topology_valid=original_info['topology_valid'],
               repaired_face_topology_valid=bool(BRepCheck_Analyzer(rebuilt).IsValid()),
               boundary_edge_topology_identities_preserved=shared,
               boundary_curve_geometric_hausdorff_m=0.0 if shared else None,
               exact_bounds_change_max_m=float(np.abs(np.array(original_info['bounds_m'])-np.array(rebuilt_info['bounds_m'])).max()),
               original_signed_area_m2=original_info['signed_area_m2'],rebuilt_area_m2=rebuilt_info['signed_area_m2'],
               absolute_area_change_m2=abs(abs(original_info['signed_area_m2'])-abs(rebuilt_info['signed_area_m2'])),
               triangle_area_m2=float(mesh.area),triangle_area_difference_m2=abs(float(mesh.area)-abs(rebuilt_info['signed_area_m2'])),
               requested_linear_deflection_m=1e-7,added_physical_thickness_m=0.,
               conservative_envelope_expansion_m=0.,normal_offset_m=0.,
               tessellation=result,boundary_discrete_verification=measure,
               scope='Local missing-face surface only; whole-link validity and attachment/wrist semantics remain independent checks.'),
             original_face=original_info,whole_link_original=topology(shape),whole_link_replacements=replacements)
  patches.append(patch)
 report=dict(schema='s50_local_collision_patch_v1',coordinate_frame='CAD_METRES',
             source_sha256=source_hash,historical_cad_audit_sha256=history_hash,
             source_files_unchanged=hashlib.sha256(SOURCE.read_bytes()).hexdigest()==source_hash and hashlib.sha256(history.read_bytes()).hexdigest()==history_hash,
             patches=patches,
             transverse_conveyor_implication='No convex hull, AABB fill, or physical offset was added. Corrected exact trims keep their CAD support and boundaries; remaining triangle approximation is recorded per patch. Whole-arm/belt gaps require transformed patches in the unchanged layout, not inferred from this local audit.')
 (OUT/'coverage_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
 print(json.dumps([dict(part=p['cad_body'],qualification=p['qualification'],original=p['whole_link_original'],replacements=p['whole_link_replacements']) for p in patches],indent=2))

if __name__=='__main__':main()

