"""Diagnose void-pair overlap in J1; no collision surfaces are removed."""
import json
from inspect_missing_faces import *
from OCP.TopAbs import TopAbs_SHELL
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeSolid
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
def main():
 parts=read_parts();shape=next(s for n,s in parts.items() if n.startswith('J1'))
 shells=[];ex=TopExp_Explorer(shape,TopAbs_SHELL)
 while ex.More():shells.append(TopoDS.Shell_s(ex.Current()));ex.Next()
 checks=[]
 for i in range(1,16):
  builder=BRepBuilderAPI_MakeSolid(shells[0]);builder.Add(shells[i]);builder.Add(shells[16])
  valid=bool(BRepCheck_Analyzer(builder.Solid()).IsValid());item=dict(inner_shells=[i,16],outer_plus_two_voids_valid=valid)
  if not valid:
   a=BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(shells[i].Reversed())).Solid()
   b=BRepBuilderAPI_MakeSolid(TopoDS.Shell_s(shells[16].Reversed())).Solid()
   distance=BRepExtrema_DistShapeShape(a,b);distance.Perform()
   item['void_boundary_distance_m']=distance.Value()*.001 if distance.IsDone() else None
   item['void_bounds_m']=[bounds(a),bounds(b)]
   common=BRepAlgoAPI_Common(a,b);item['boolean_common_done']=bool(common.IsDone())
   if common.IsDone():
    c=common.Shape();props=GProp_GProps();BRepGProp.VolumeProperties_s(c,props)
    item['common_volume_m3']=props.Mass()*1e-9;item['common_topology_valid']=bool(BRepCheck_Analyzer(c).IsValid())
    item['common_subshape_counts']={}
    for kind in [TopAbs_FACE,TopAbs_EDGE,TopAbs_VERTEX]:
     ex=TopExp_Explorer(c,kind);count=0
     while ex.More():count+=1;ex.Next()
     item['common_subshape_counts'][str(kind)]=count
    if props.Mass()>1e-12:item['common_bounds_m']=bounds(c)
  checks.append(item)
 report=dict(schema='s50_j1_void_overlap_v1',checks=checks,
             source_unchanged=True,no_collision_faces_deleted=True,
             conclusion='Pair validity and exact Boolean intersection diagnose internal cavities; this script does not repair or qualify the complete source solid.')
 (OUT/'j1_void_overlap.json').write_text(json.dumps(report,indent=2),encoding='utf8')
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()

