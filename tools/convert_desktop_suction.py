"""Read-only XCAF STEP conversion. All detailed geometry stays in ignored local paths."""
from __future__ import annotations
import argparse, hashlib, json, re
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT/"assets/tools/desktop_suction/cad/raw"
DERIVED = ROOT/"assets/tools/desktop_suction/derived"

def convert():
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_LabelSequence, TDF_Label, TDF_Tool
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorGen, XCAFDoc_ColorSurf
    from OCP.Quantity import Quantity_Color
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_SHELL, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepBndLib import BRepBndLib
    from OCP.Bnd import Bnd_Box
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane, GeomAbs_Cylinder
    from OCP.Interface import Interface_Static
    import trimesh, OCP
    paths=sorted(p for p in RAW.rglob("*") if p.is_file() and p.suffix.lower() in (".step",".stp"))
    if not paths: raise FileNotFoundError("No private STEP/STP in assets/tools/desktop_suction/cad/raw; no demo fallback.")
    if len(paths)!=1: raise ValueError("Multiple STEP inputs: inspect assembly/version relationships before choosing; no implicit duplication.")
    source=paths[0]; original=source.read_bytes(); digest=hashlib.sha256(original).hexdigest()
    text=original.decode("latin-1")
    declarations=sorted(set(re.findall(r"SI_UNIT\s*\(\s*([^,]+),\s*\.METRE\.\s*\)",text)))
    if declarations!=[".MILLI."]: raise ValueError(f"Unreviewed STEP length units: {declarations}")
    Interface_Static.SetCVal_s("xstep.cascade.unit","MM")
    document=TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(True)
    if reader.ReadFile(str(source))!=IFSelect_RetDone:raise ValueError("STEP reader failed")
    reader.ChangeReader().SetSystemLengthUnit(1.0)
    if not reader.Transfer(document):raise ValueError("XCAF transfer failed")
    st=XCAFDoc_DocumentTool.ShapeTool_s(document.Main());ct=XCAFDoc_DocumentTool.ColorTool_s(document.Main())
    free=TDF_LabelSequence();st.GetFreeShapes(free)
    nodes=[];parts=[]
    def matrix(location):
        t=location.Transformation();m=np.eye(4)
        for i in range(3):
            for j in range(4):m[i,j]=t.Value(i+1,j+1)
        m[:3,3]*=.001
        return m
    def name(label):
        a=TDataStd_Name()
        return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),a) else ""
    def bbox(shape):
        b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b,False,False)
        return (np.array(b.Get()).reshape(2,3)*.001).tolist()
    def count(shape,kind):
        ex=TopExp_Explorer(shape,kind);n=0
        while ex.More():n+=1;ex.Next()
        return n
    def walk(label,loc,path):
        local=st.GetLocation_s(label);world=loc.Multiplied(local);ref=TDF_Label()
        definition=label
        if st.IsReference_s(label):
            if not st.GetReferredShape_s(label,ref):raise ValueError("unresolved XCAF reference")
            definition=ref
        key=path+"/"+str(label.Tag())
        node=dict(instance_id=key,name=name(label),definition_name=name(definition),local_transform_m=matrix(local).tolist(),global_transform_m=matrix(world).tolist(),assembly=st.IsAssembly_s(definition))
        nodes.append(node)
        if node["assembly"]:
            seq=TDF_LabelSequence();st.GetComponents_s(definition,seq,False)
            for i in range(1,seq.Length()+1):walk(seq.Value(i),world,key)
            return
        shape=st.GetShape_s(definition).Moved(world)
        if shape.IsNull():raise ValueError("null/unresolved shape")
        solids=[];ex=TopExp_Explorer(shape,TopAbs_SOLID)
        while ex.More():solids.append(ex.Current());ex.Next()
        if not solids:solids=[shape]
        for solid_index,solid in enumerate(solids):
            pid=f"part_{len(parts):03d}"
            valid=bool(BRepCheck_Analyzer(solid).IsValid())
            BRepMesh_IncrementalMesh(solid,.10,False,.15,True)
            vertices=[];triangles=[];surfaces=[]
            ex=TopExp_Explorer(solid,TopAbs_FACE)
            while ex.More():
                face=TopoDS.Face_s(ex.Current());location=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,location)
                if tri is None:raise ValueError("face without triangulation")
                offset=len(vertices)
                for k in range(1,tri.NbNodes()+1):
                    p=tri.Node(k).Transformed(location.Transformation());vertices.append([p.X()*.001,p.Y()*.001,p.Z()*.001])
                for k in range(1,tri.NbTriangles()+1):
                    inds=list(tri.Triangle(k).Get())
                    if face.Orientation()==TopAbs_REVERSED:inds.reverse()
                    triangles.append([offset+x-1 for x in inds])
                surf=BRepAdaptor_Surface(face,True);typ=surf.GetType()
                props=GProp_GProps();BRepGProp.SurfaceProperties_s(face,props)
                info=dict(area_m2=props.Mass()*1e-6,bounds_m=bbox(face))
                if typ==GeomAbs_Plane:
                    plane=surf.Plane();a=plane.Axis();v=a.Direction();q=a.Location()
                    info.update(type="plane",point_m=[q.X()*.001,q.Y()*.001,q.Z()*.001],axis=[v.X(),v.Y(),v.Z()])
                elif typ==GeomAbs_Cylinder:
                    c=surf.Cylinder();a=c.Axis();v=a.Direction();q=a.Location()
                    info.update(type="cylinder",radius_m=c.Radius()*.001,point_m=[q.X()*.001,q.Y()*.001,q.Z()*.001],axis=[v.X(),v.Y(),v.Z()])
                else:info["type"]=str(typ)
                surfaces.append(info);ex.Next()
            mesh=trimesh.Trimesh(vertices=vertices,faces=triangles,process=True)
            path_visual=DERIVED/"visual"/f"{pid}.obj";path_visual.parent.mkdir(parents=True,exist_ok=True);mesh.export(path_visual)
            hull=mesh.convex_hull;path_hull=DERIVED/"collision"/f"{pid}.obj";path_hull.parent.mkdir(parents=True,exist_ok=True);hull.export(path_hull)
            planes=hull.facets_normal; # actual coverage via all convex-hull half-space equations
            from scipy.spatial import ConvexHull
            eq=ConvexHull(np.asarray(mesh.vertices)).equations
            violation=max(float(np.max(mesh.vertices@row[:3]+row[3])) for row in eq)
            props=GProp_GProps()
            vol=None;center=None
            if valid and count(solid,TopAbs_SOLID)>0:
                BRepGProp.VolumeProperties_s(solid,props);vol=props.Mass()*1e-9
                c=props.CentreOfMass();center=[c.X()*.001,c.Y()*.001,c.Z()*.001]
            color=Quantity_Color();has=ct.GetColor_s(definition,XCAFDoc_ColorGen,color) or ct.GetColor_s(definition,XCAFDoc_ColorSurf,color)
            bounds=bbox(solid)
            parts.append(dict(id=pid,instance_id=key,solid_index=solid_index,name=node["definition_name"] or node["name"],
                bounds_m=bounds,size_m=(np.array(bounds)[1]-bounds[0]).tolist(),topology_valid=valid,
                solids=count(solid,TopAbs_SOLID),shells=count(solid,TopAbs_SHELL),faces=count(solid,TopAbs_FACE),
                geometric_volume_m3=vol,geometric_volume_centroid_m=center,actual_mass_kg=None,
                rgb=[color.Red(),color.Green(),color.Blue()] if has else None,
                visual=str(path_visual.relative_to(ROOT)).replace("\\","/"),collision=str(path_hull.relative_to(ROOT)).replace("\\","/"),
                vertices=len(mesh.vertices),triangles=len(mesh.faces),surface_features=surfaces,
                collision_type="per_solid_convex_hull",max_vertex_outside_hull_m=violation,
                conservative_surface_inflation_m=.0002,semantic="unknown_rigid"))
    for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location(),"root")
    lower=np.min([p["bounds_m"][0] for p in parts],axis=0);upper=np.max([p["bounds_m"][1] for p in parts],axis=0)
    result=dict(schema="eco65_private_cad_v1",source=str(source.relative_to(ROOT)),source_sha256=digest,size_bytes=len(original),
        format="STEP AP214",reader=f"OCP {OCP.__version__} STEPCAFControl_Reader",file_length_units="millimetres",
        reader_system_length_unit_mm=1.0,reader_output_units="millimetres",normalization_scale_to_m=.001,
        transform_convention="T_A_B converts B to A; translations scaled once, rotations unchanged",
        selection_reason="One STEP assembly supplied, instantiate XCAF leaves once, not assembly compound plus children",
        free_shapes=free.Length(),assembly_nodes=nodes,parts=parts,bounds_m=[lower.tolist(),upper.tolist()],size_m=(upper-lower).tolist(),
        tessellation=dict(linear_deflection_m=.0001,angular_deflection_rad=.15,relative=False),
        repairs=[],external_references=list(re.findall(r"EXTERNALLY_DEFINED[^;]+;",text)),unprocessed_parts=[],
        physics=dict(actual_mass_kg=None,material=None,vacuum_circuit=None,real_compression_stroke_m=None))
    assert hashlib.sha256(source.read_bytes()).hexdigest()==digest
    target=DERIVED/"manifests"/"cad.json";target.parent.mkdir(parents=True,exist_ok=True);target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps({k:result[k] for k in ["source","source_sha256","free_shapes","size_m"]},indent=2),flush=True)
    for p in parts:print(p["id"],p["name"],np.round(p["size_m"],5),p["topology_valid"],flush=True)
    return result

if __name__=="__main__":
    convert()
