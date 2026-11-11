"""Inspect missing S50 planar faces without mutating historical geometry."""
from pathlib import Path
import json, hashlib
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence,TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TopLoc import TopLoc_Location
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_WIRE,TopAbs_EDGE,TopAbs_VERTEX
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepAdaptor import BRepAdaptor_Surface,BRepAdaptor_Curve
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Interface import Interface_Static
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.ShapeFix import ShapeFix_Face
from OCP.BRepTools import BRepTools
import OCP

ROOT=Path(__file__).resolve().parents[4]
BASE=ROOT/'assets/robots/huayan_s50_2000'
OUT=BASE/'collision_repair'
SOURCE=BASE/'source/S50.STEP'

def xyz(p):return [p.X()*.001,p.Y()*.001,p.Z()*.001]
def bounds(shape):
 b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b,False,False)
 return (np.array(b.Get()).reshape(2,3)*.001).tolist()
def read_parts():
 reader=STEPCAFControl_Reader();reader.SetNameMode(True)
 Interface_Static.SetCVal_s('xstep.cascade.unit','MM')
 assert reader.ReadFile(str(SOURCE))==IFSelect_RetDone
 reader.ChangeReader().SetSystemLengthUnit(1.)
 doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'));assert reader.Transfer(doc)
 st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());free=TDF_LabelSequence();st.GetFreeShapes(free)
 parts={}
 def name(l):
  a=TDataStd_Name();return a.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(),a) else ''
 def walk(l,parent):
  world=parent.Multiplied(st.GetLocation_s(l));d=l
  if st.IsReference_s(l):d=TDF_Label();assert st.GetReferredShape_s(l,d)
  if st.IsAssembly_s(d):
   seq=TDF_LabelSequence();st.GetComponents_s(d,seq,False)
   for i in range(1,seq.Length()+1):walk(seq.Value(i),world)
  else:parts[name(d)]=st.GetShape_s(d).Moved(world)
 for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location())
 return parts

def faces(shape):
 ex=TopExp_Explorer(shape,TopAbs_FACE);items=[]
 while ex.More():items.append(TopoDS.Face_s(ex.Current()));ex.Next()
 return items

def face_info(face):
 s=BRepAdaptor_Surface(face,True);pl=s.Plane();p=GProp_GProps();BRepGProp.SurfaceProperties_s(face,p)
 wires=[];ex=TopExp_Explorer(face,TopAbs_WIRE)
 while ex.More():
  wire=TopoDS.Wire_s(ex.Current());edges=[];ee=TopExp_Explorer(wire,TopAbs_EDGE)
  while ee.More():
   e=TopoDS.Edge_s(ee.Current());c=BRepAdaptor_Curve(e)
   u,v=c.FirstParameter(),c.LastParameter()
   samples=[xyz(c.Value(float(t))) for t in np.linspace(u,v,65)]
   edges.append(dict(type=str(c.GetType()),orientation=str(e.Orientation()),bounds_m=bounds(e),parameters=[u,v],samples_m=samples))
   ee.Next()
  wires.append(dict(orientation=str(wire.Orientation()),edges=edges))
  ex.Next()
 return dict(bounds_m=bounds(face),surface_type=str(s.GetType()),signed_area_m2=p.Mass()*1e-6,
             plane_point_m=xyz(pl.Location()),plane_normal=[pl.Axis().Direction().X(),pl.Axis().Direction().Y(),pl.Axis().Direction().Z()],
             orientation=str(face.Orientation()),topology_valid=bool(BRepCheck_Analyzer(face).IsValid()),wires=wires)

def mesh_info(face,deflection_mm):
 BRepTools.Clean_s(face)
 mesh=BRepMesh_IncrementalMesh(face,deflection_mm,False,.1,False)
 loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
 return dict(deflection_mm=deflection_mm,status=int(mesh.GetStatusFlags()),triangles=tri.NbTriangles() if tri else 0)

def main():
 OUT.mkdir(parents=True,exist_ok=True);parts=read_parts();result=[]
 for label,index in [('J1',74),('J2',75)]:
  name=next(n for n in parts if n.startswith(label))
  face=faces(parts[name])[index];info=face_info(face)
  attempts=[]
  for defl in [.1,.01,.001]:attempts.append(dict(method='original_face_remesh',**mesh_info(face,defl)))
  fixer=ShapeFix_Face(face);performed=bool(fixer.Perform());fixed=fixer.Face()
  attempts.append(dict(method='ShapeFix_Face.Perform',performed=performed,topology_valid=bool(BRepCheck_Analyzer(fixed).IsValid()),**mesh_info(fixed,.001)))
  result.append(dict(part=label,face_index=index,original=info,attempts=attempts))
 report=dict(schema='s50_local_face_inspection_v1',source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),reader='OCP '+OCP.__version__,frame='original CAD assembly metres',parts=result)
 (OUT/'local_face_inspection.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
 print(json.dumps([dict(part=r['part'],bounds=r['original']['bounds_m'],valid=r['original']['topology_valid'],wires=len(r['original']['wires']),edges=[len(w['edges']) for w in r['original']['wires']],attempts=r['attempts']) for r in result],indent=2))

if __name__=='__main__':main()

