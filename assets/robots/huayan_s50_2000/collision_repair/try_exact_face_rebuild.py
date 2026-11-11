"""Try exact-curve face rebuilding; record outcomes, never overwrite source meshes."""
import json
from inspect_missing_faces import *
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace,BRepBuilderAPI_MakeEdge,BRepBuilderAPI_MakeWire
from OCP.ShapeFix import ShapeFix_Face
from OCP.Geom import Geom_Curve

def main():
 parts=read_parts();result=[]
 for label,index in [('J1',74),('J2',75)]:
  name=next(n for n in parts if n.startswith(label));face=faces(parts[name])[index]
  wex=TopExp_Explorer(face,TopAbs_WIRE);wire=TopoDS.Wire_s(wex.Current())
  plane=BRepAdaptor_Surface(face,True).Plane();attempts=[]
  for variant in ['same_wire_plane','reversed_wire_plane','new_edges_plane']:
   try:
    if variant=='new_edges_plane':
     builder=BRepBuilderAPI_MakeWire();ex=TopExp_Explorer(wire,TopAbs_EDGE)
     while ex.More():
      e=TopoDS.Edge_s(ex.Current());c=BRepAdaptor_Curve(e)
      curve=c.Curve().Curve().Transformed(c.Trsf())
      edge=BRepBuilderAPI_MakeEdge(curve,c.FirstParameter(),c.LastParameter()).Edge()
      edge.Orientation(e.Orientation());builder.Add(edge);ex.Next()
     candidate=builder.Wire()
    else:candidate=wire if variant=='same_wire_plane' else TopoDS.Wire_s(wire.Reversed())
    maker=BRepBuilderAPI_MakeFace(plane,candidate,True)
    if not maker.IsDone():raise ValueError(str(maker.Error()))
    rebuilt=maker.Face()
    info=dict(method=variant,topology_valid=bool(BRepCheck_Analyzer(rebuilt).IsValid()),geometry=face_info(rebuilt))
    info['remesh']=mesh_info(rebuilt,.0001)
    attempts.append(info)
   except Exception as e:attempts.append(dict(method=variant,error=str(e)))
  result.append(dict(part=label,face_index=index,attempts=attempts))
 (OUT/'exact_face_rebuild_attempts.json').write_text(json.dumps(result,indent=2),encoding='utf8')
 print(json.dumps([dict(part=p['part'],attempts=[{k:v for k,v in a.items() if k!='geometry'} for a in p['attempts']]) for p in result],indent=2))

if __name__=='__main__':main()

