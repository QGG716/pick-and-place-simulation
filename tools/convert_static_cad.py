"""Independent read-only XCAF converter for a fixed private scene assembly.

Uses the same OCCT leaf/location/tessellation conventions as the existing tool
converter, without its fixed output paths or suction/flange identification.
"""
import argparse,hashlib,json,re
from pathlib import Path
import numpy as np
import trimesh

ROOT=Path(__file__).resolve().parents[1]


def derived_path(path):
    path=Path(path).resolve()
    if not any(path.is_relative_to(ROOT/p) for p in ['assets/mast/derived','assets/camera/derived']):
        raise ValueError('PRIVATE_DERIVED_DIRECTORY_REQUIRED')
    return path


def convert(source,out,part_prefix='mast'):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_LabelSequence,TDF_Label
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_DocumentTool,XCAFDoc_ColorGen,XCAFDoc_ColorSurf
    from OCP.Quantity import Quantity_Color
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE,TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    from OCP.Interface import Interface_Static
    import OCP
    source=Path(source).resolve();source.relative_to(ROOT);out=derived_path(out)
    if not re.fullmatch('[a-z][a-z0-9_]*',part_prefix):raise ValueError('INVALID_PART_PREFIX')
    original=source.read_bytes();sha=hashlib.sha256(original).hexdigest()
    reader=STEPCAFControl_Reader()
    result=reader.ReadFile(str(source))
    if result!=IFSelect_RetDone:raise ValueError('STEP_READ_FAILED:'+str(result))
    declarations=sorted(set(re.findall(r'SI_UNIT\s*\(\s*([^,]+),\s*\.METRE\.\s*\)',original.decode('latin1'))))
    if declarations!=['.MILLI.']:raise ValueError('UNREVIEWED_STEP_UNITS:'+str(declarations))
    out.mkdir(parents=True,exist_ok=False)
    Interface_Static.SetCVal_s('xstep.cascade.unit','MM')
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    reader.SetNameMode(True);reader.SetColorMode(True)
    reader.ChangeReader().SetSystemLengthUnit(1.)
    if not reader.Transfer(doc):raise ValueError('XCAF_TRANSFER_FAILED')
    st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main());ct=XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    free=TDF_LabelSequence();st.GetFreeShapes(free);nodes=[];parts=[]
    def name(label):
        a=TDataStd_Name();return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),a) else ''
    def matrix(loc):
        t=loc.Transformation();m=np.eye(4)
        for i in range(3):
            for j in range(4):m[i,j]=t.Value(i+1,j+1)
        m[:3,3]*=.001;return m.tolist()
    def walk(label,loc,parent):
        world=loc.Multiplied(st.GetLocation_s(label));definition=label
        if st.IsReference_s(label):
            definition=TDF_Label()
            if not st.GetReferredShape_s(label,definition):raise ValueError('XCAF_UNRESOLVED_REFERENCE')
        key=parent+'/'+str(label.Tag());assembly=st.IsAssembly_s(definition)
        nodes.append(dict(id=key,name=name(label),definition_name=name(definition),T_CAD_instance=matrix(world),assembly=assembly))
        if assembly:
            seq=TDF_LabelSequence();st.GetComponents_s(definition,seq,False)
            for i in range(1,seq.Length()+1):walk(seq.Value(i),world,key)
            return
        shape=st.GetShape_s(definition).Moved(world);solids=[];ex=TopExp_Explorer(shape,TopAbs_SOLID)
        while ex.More():solids.append(ex.Current());ex.Next()
        if not solids:solids=[shape]
        for solid in solids:
            id=f'{part_prefix}_part_{len(parts):03}';BRepMesh_IncrementalMesh(solid,.10,False,.15,True)
            vertices=[];faces=[];planes=[];ex=TopExp_Explorer(solid,TopAbs_FACE)
            while ex.More():
                face=TopoDS.Face_s(ex.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
                if tri is None:raise ValueError('MISSING_TRIANGULATION:'+id)
                offset=len(vertices)
                for i in range(1,tri.NbNodes()+1):
                    p=tri.Node(i).Transformed(loc.Transformation());vertices.append([p.X()*.001,p.Y()*.001,p.Z()*.001])
                for i in range(1,tri.NbTriangles()+1):
                    ids=list(tri.Triangle(i).Get())
                    if face.Orientation()==TopAbs_REVERSED:ids.reverse()
                    faces.append([offset+j-1 for j in ids])
                surface=BRepAdaptor_Surface(face,True)
                if surface.GetType()==GeomAbs_Plane:
                    p=surface.Plane();n=p.Axis().Direction();q=p.Location();props=GProp_GProps();BRepGProp.SurfaceProperties_s(face,props)
                    planes.append(dict(point_m=[q.X()*.001,q.Y()*.001,q.Z()*.001],normal=(np.array([n.X(),n.Y(),n.Z()])*(-1 if face.Orientation()==TopAbs_REVERSED else 1)).tolist(),area_m2=props.Mass()*1e-6))
                ex.Next()
            if not vertices:
                nodes[-1].setdefault('non_surface_shapes',[]).append(str(solid.ShapeType()))
                continue
            mesh=trimesh.Trimesh(vertices,faces,process=True);visual=out/(id+'.obj');mesh.export(visual)
            from scipy.spatial import ConvexHull
            ch=ConvexHull(mesh.vertices);triangles=ch.simplices.copy()
            normals=np.cross(mesh.vertices[triangles[:,1]]-mesh.vertices[triangles[:,0]],mesh.vertices[triangles[:,2]]-mesh.vertices[triangles[:,0]])
            reverse=np.einsum('ij,ij->i',normals,ch.equations[:,:3])<0
            triangles[reverse]=triangles[reverse][:,::-1]
            hull=out/(id+'_convex.obj');trimesh.Trimesh(mesh.vertices,triangles,process=False).export(hull)
            color=Quantity_Color();has=ct.GetColor_s(definition,XCAFDoc_ColorGen,color) or ct.GetColor_s(definition,XCAFDoc_ColorSurf,color)
            parts.append(dict(id=id,instance=key,name=name(definition),bounds_m=mesh.bounds.tolist(),size_m=mesh.extents.tolist(),
                rgb=[color.Red(),color.Green(),color.Blue()] if has else None,topology_valid=bool(BRepCheck_Analyzer(solid).IsValid()),
                visual=visual.relative_to(ROOT).as_posix(),visual_sha256=hashlib.sha256(visual.read_bytes()).hexdigest(),
                collision=hull.relative_to(ROOT).as_posix(),collision_sha256=hashlib.sha256(hull.read_bytes()).hexdigest(),
                collision_type='per_solid_convex_hull',planar_faces=planes))
    for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location(),'root')
    b=np.array([p['bounds_m'] for p in parts]);bounds=np.array([b[:,0].min(0),b[:,1].max(0)])
    result=dict(schema='private_static_cad_v1',source=source.relative_to(ROOT).as_posix(),source_sha256=sha,source_bytes=len(original),
        source_units='millimetres',reader_system_unit_mm=1.,normalization_scale_to_m=.001,reader='OCP '+OCP.__version__,
        assembly_nodes=nodes,parts=parts,free_shapes=free.Length(),bounds_m=bounds.tolist(),size_m=(bounds[1]-bounds[0]).tolist(),
        selection='XCAF leaf instances once; no parent compound duplication',repairs=[],dynamics='NOT_EVALUATED')
    assert hashlib.sha256(source.read_bytes()).hexdigest()==sha
    (out/'cad.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(dict(sha256=sha,parts=len(parts),bounds_m=result['bounds_m'],size_m=result['size_m']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--part-prefix',default='mast')
    a=p.parse_args()
    try:convert(a.input,a.output,a.part_prefix)
    except Exception as error:
        destination=derived_path(a.output)
        destination.mkdir(parents=True,exist_ok=True)
        report=destination/'import_failure.json'
        if not report.exists():
            report.write_text(json.dumps(dict(status='IMPORT_FAILED',source=a.input.resolve().relative_to(ROOT).as_posix(),source_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),error=str(error),units=None,bounds_m=None,T_D_mast=None),indent=2),encoding='utf8')
        raise
