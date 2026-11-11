"""Locate retained J1 solid containment defect independently from missing faces."""
import json
from inspect_missing_faces import *
from OCP.TopAbs import TopAbs_SHELL,TopAbs_VERTEX,TopAbs_IN,TopAbs_OUT,TopAbs_ON
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCP.BRepClass3d import BRepClass3d_SolidClassifier

def main():
 parts=read_parts();shape=next(s for n,s in parts.items() if n.startswith('J1'))
 shells=[];ex=TopExp_Explorer(shape,TopAbs_SHELL)
 while ex.More():
  shell=TopoDS.Shell_s(ex.Current());props=GProp_GProps();BRepGProp.VolumeProperties_s(shell,props)
  shells.append((shell,dict(index=len(shells),closed=bool(BRep_Tool.IsClosed_s(shell)),
                           shell_topology_valid=bool(BRepCheck_Analyzer(shell).IsValid()),
                           signed_volume_m3=props.Mass()*1e-9,bounds_m=bounds(shell),
                           volume_centroid_m=xyz(props.CentreOfMass()))));ex.Next()
 principal=max(range(len(shells)),key=lambda i:abs(shells[i][1]['signed_volume_m3']))
 outer=BRepBuilderAPI_MakeSolid(shells[principal][0]).Solid()
 classifier=BRepClass3d_SolidClassifier(outer)
 for i,(shell,entry) in enumerate(shells):
  if i==principal:entry['principal_outer_shell']=True;continue
  vertices=[];ex=TopExp_Explorer(shell,TopAbs_VERTEX)
  while ex.More():
   p=BRep_Tool.Pnt_s(TopoDS.Vertex_s(ex.Current()))
   if not any(p.Distance(q)<1e-7 for q in vertices):vertices.append(p)
   ex.Next()
  states={};out=[]
  for p in vertices:
   classifier.Perform(p,1e-6);state=str(classifier.State());states[state]=states.get(state,0)+1
   if classifier.State()==TopAbs_OUT:out.append(xyz(p))
  entry['vertices_against_outer_solid']=states;entry['outside_vertex_samples_m']=out[:8]
 report=dict(schema='s50_j1_shell_containment_v1',source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
             principal_outer_shell=principal,principal_solid_valid=bool(BRepCheck_Analyzer(outer).IsValid()),
             shells=[e for _,e in shells],
             original_solid_valid=bool(BRepCheck_Analyzer(shape).IsValid()),
             note='Vertex membership is diagnostic, not a full geometric containment proof or a repair.')
 (OUT/'j1_shell_containment.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()

