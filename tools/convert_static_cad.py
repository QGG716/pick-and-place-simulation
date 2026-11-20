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
    if not any(path.is_relative_to(ROOT/p) for p in ['assets/mast/derived','assets/camera/derived','assets/layout/derived']):
        raise ValueError('PRIVATE_DERIVED_DIRECTORY_REQUIRED')
    return path


def step_string(text):
    """Decode ISO 10303-21 string escapes (\\X2\\, \\X4\\, \\X\\HH, \\S\\)."""
    text=re.sub(r'\\X2\\((?:[0-9A-F]{4})+)\\X0\\',lambda m:''.join(chr(int(m.group(1)[i:i+4],16)) for i in range(0,len(m.group(1)),4)),text)
    text=re.sub(r'\\X4\\((?:[0-9A-F]{8})+)\\X0\\',lambda m:''.join(chr(int(m.group(1)[i:i+8],16)) for i in range(0,len(m.group(1)),8)),text)
    text=re.sub(r'\\X\\([0-9A-F]{2})',lambda m:chr(int(m.group(1),16)),text)
    text=re.sub(r'\\S\\(.)',lambda m:chr(ord(m.group(1))+128),text)
    return text.replace('\\\\','\\')


def surface_model_shells(text):
    """(SHELL_BASED_SURFACE_MODEL name, OPEN_SHELL face count) in file order.

    OPEN_SHELL entities carry no name; OCCT exposes no usable entity identity
    to Python, so callers match this order against transferred shells by face
    count and keep names only when the whole sequence agrees.
    """
    flat=re.sub(r'\r?\n','',text)
    shells={i:len(f.split(',')) for i,f in re.findall(r"#(\d+)\s*=\s*OPEN_SHELL\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)\s*\)",flat)}
    models={i:(n,[x.strip()[1:] for x in b.split(',')]) for i,n,b in re.findall(r"#(\d+)\s*=\s*SHELL_BASED_SURFACE_MODEL\s*\(\s*'([^']*)'\s*,\s*\(([^)]*)\)\s*\)",flat)}
    order=[]
    for items in re.findall(r"=\s*MANIFOLD_SURFACE_SHAPE_REPRESENTATION\s*\(\s*'[^']*'\s*,\s*\(([^)]*)\)",flat):
        for item in items.split(','):
            item=item.strip()[1:]
            if item in models:
                name,bounds=models[item];order.extend((step_string(name),shells.get(b)) for b in bounds)
    return order


def convert(source,out,part_prefix='mast',collision_mode='convex_hull'):
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
    from OCP.TopAbs import TopAbs_SOLID,TopAbs_SHELL,TopAbs_FACE,TopAbs_REVERSED
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
    if collision_mode not in ('convex_hull','cad_aabb'):raise ValueError('UNSUPPORTED_STATIC_COLLISION_MODE')
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
    transfer=reader.ChangeReader().WS().TransferReader()
    def step_body_name(body):
        e=transfer.EntityFromShapeResult(body,1)
        if e is None:return '',None
        owner=e
        label=owner.Name() if hasattr(owner,'Name') else None
        return (step_string(label.ToCString()) if label is not None else ''),e.DynamicType().Name()
    deferred=[];open_shells=[]
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
        base=st.GetShape_s(definition);bodies=[];ex=TopExp_Explorer(base,TopAbs_SOLID)
        while ex.More():bodies.append(('SOLID',ex.Current()));ex.Next()
        if collision_mode=='cad_aabb':
            # Layout mode keeps every surface body: open shells outside any
            # solid and loose faces outside any shell are separate parts.
            for kind,what,avoid in (('OPEN_SHELL',TopAbs_SHELL,TopAbs_SOLID),('FACE',TopAbs_FACE,TopAbs_SHELL)):
                ex=TopExp_Explorer(base,what,avoid)
                while ex.More():bodies.append((kind,ex.Current()));ex.Next()
        if not bodies:bodies=[('SHAPE',base)]
        for kind,body in bodies:
            body_name,entity_type=step_body_name(body) if collision_mode=='cad_aabb' else ('',None)
            if kind=='OPEN_SHELL':
                count=0;fx=TopExp_Explorer(body,TopAbs_FACE)
                while fx.More():count+=1;fx.Next()
                record=dict(faces=count);open_shells.append(record)
            solid=body.Moved(world)
            id=f'{part_prefix}_part_{len(parts):03}';BRepMesh_IncrementalMesh(solid,.10,False,.15,True)
            vertices=[];faces=[];planes=[];untriangulated=0;ex=TopExp_Explorer(solid,TopAbs_FACE)
            while ex.More():
                face=TopoDS.Face_s(ex.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
                if tri is None:
                    # Layout collision uses analytic BRep bounds; a face OCCT
                    # cannot mesh is counted, never silently dropped.
                    if collision_mode!='cad_aabb':raise ValueError('MISSING_TRIANGULATION:'+id)
                    untriangulated+=1;ex.Next();continue
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
            valid=bool(BRepCheck_Analyzer(solid).IsValid())
            if collision_mode=='cad_aabb' and kind!='SOLID' and (not vertices or untriangulated or not valid):
                # A degenerate or invalid surface body is represented only when
                # its analytic bounds lie inside a valid solid part (checked below).
                from OCP.BRepBndLib import BRepBndLib
                from OCP.Bnd import Bnd_Box
                bound=Bnd_Box();BRepBndLib.AddOptimal_s(solid,bound,False,True)
                deferred.append(dict(shell=record,instance=key,body_kind=kind,body_name=body_name,step_entity_type=entity_type,topology_valid=valid,
                    triangulated=bool(vertices),untriangulated_faces=untriangulated,analytic_bounds_m=(np.asarray(bound.Get()).reshape(2,3)*.001).tolist()))
                continue
            if not vertices:
                nodes[-1].setdefault('non_surface_shapes',[]).append(str(solid.ShapeType()))
                continue
            mesh=trimesh.Trimesh(vertices,faces,process=True);visual=out/(id+'.obj');mesh.export(visual)
            hull=out/(id+'_convex.obj')
            if collision_mode=='convex_hull':
                from scipy.spatial import ConvexHull
                ch=ConvexHull(mesh.vertices);triangles=ch.simplices.copy()
                normals=np.cross(mesh.vertices[triangles[:,1]]-mesh.vertices[triangles[:,0]],mesh.vertices[triangles[:,2]]-mesh.vertices[triangles[:,0]])
                reverse=np.einsum('ij,ij->i',normals,ch.equations[:,:3])<0
                triangles[reverse]=triangles[reverse][:,::-1]
                trimesh.Trimesh(mesh.vertices,triangles,process=False).export(hull)
            collision_bounds=mesh.bounds
            if collision_mode=='cad_aabb':
                # Analytic BRep bounds, not extrema of an inscribed triangle
                # approximation. Curved STEP surfaces must remain enclosed.
                from OCP.BRepBndLib import BRepBndLib
                from OCP.Bnd import Bnd_Box
                bound=Bnd_Box();BRepBndLib.AddOptimal_s(solid,bound,False,True)
                collision_bounds=np.asarray(bound.Get()).reshape(2,3)*.001
                extent=collision_bounds[1]-collision_bounds[0]
                if np.any(extent<=0):raise ValueError('NONVOLUMETRIC_LAYOUT_PART:'+id)
                conservative=trimesh.creation.box(extents=extent)
                conservative.apply_translation(collision_bounds.mean(0));conservative.export(hull)
            color=Quantity_Color();has=ct.GetColor_s(definition,XCAFDoc_ColorGen,color) or ct.GetColor_s(definition,XCAFDoc_ColorSurf,color)
            parts.append(dict(id=id,instance=key,name=name(definition),bounds_m=mesh.bounds.tolist(),size_m=mesh.extents.tolist(),
                rgb=[color.Red(),color.Green(),color.Blue()] if has else None,topology_valid=valid,
                visual=visual.relative_to(ROOT).as_posix(),visual_sha256=hashlib.sha256(visual.read_bytes()).hexdigest(),
                collision=hull.relative_to(ROOT).as_posix(),collision_sha256=hashlib.sha256(hull.read_bytes()).hexdigest(),
                collision_type='analytic_brep_aabb' if collision_mode=='cad_aabb' else 'per_solid_convex_hull',
                collision_bounds_m=collision_bounds.tolist(),planar_faces=planes))
            if collision_mode=='cad_aabb':
                parts[-1].update(body_kind=kind,body_name=body_name,step_entity_type=entity_type,untriangulated_faces=untriangulated)
                if kind=='OPEN_SHELL':record['part']=parts[-1]
    for i in range(1,free.Length()+1):walk(free.Value(i),TopLoc_Location(),'root')
    naming='NO_OPEN_SHELLS'
    if collision_mode=='cad_aabb' and open_shells:
        # OCCT may split one disconnected OPEN_SHELL into several shells; the
        # consecutive face counts must refill each STEP shell exactly.
        expected=surface_model_shells(original.decode('latin1'));labels=[];k=0;total=faces=0
        for label,faces in expected:
            total=0
            while k<len(open_shells) and total<(faces or 0):total+=open_shells[k]['faces'];labels.append(label);k+=1
            if total!=faces:break
        naming='STEP_ORDER_FACE_COUNT_VERIFIED' if len(labels)==len(open_shells)==k and total==faces else 'UNVERIFIED_LEFT_UNNAMED'
        if naming=='STEP_ORDER_FACE_COUNT_VERIFIED':
            for label,r in zip(labels,open_shells):
                r['name']=label
                if 'part' in r and not r['part']['body_name']:r['part']['body_name']=label
    for body in deferred:
        record=body.pop('shell',None)
        if record and not body['body_name']:body['body_name']=record.get('name','')
    enclosed=[]
    for body in deferred:
        lo,hi=np.asarray(body['analytic_bounds_m'])
        hosts=[p for p in parts if p.get('body_kind')=='SOLID' and p['topology_valid']
               and np.all(np.asarray(p['collision_bounds_m'][0])<=lo) and np.all(hi<=np.asarray(p['collision_bounds_m'][1]))]
        if hosts:
            host=min(hosts,key=lambda p:np.prod(np.subtract(p['collision_bounds_m'][1],p['collision_bounds_m'][0])))
            body.update(enclosed_by=host['id'],representation='INSIDE_HOST_ANALYTIC_AABB')
        else:body.update(enclosed_by=None,representation='UNREPRESENTED')
        enclosed.append(body)
    b=np.array([p['bounds_m'] for p in parts]);bounds=np.array([b[:,0].min(0),b[:,1].max(0)])
    result=dict(schema='private_static_cad_v1',source=source.relative_to(ROOT).as_posix(),source_sha256=sha,source_bytes=len(original),
        source_units='millimetres',reader_system_unit_mm=1.,normalization_scale_to_m=.001,reader='OCP '+OCP.__version__,
        assembly_nodes=nodes,parts=parts,free_shapes=free.Length(),bounds_m=bounds.tolist(),size_m=(bounds[1]-bounds[0]).tolist(),
        selection='XCAF leaf instances once; no parent compound duplication',repairs=[],dynamics='NOT_EVALUATED')
    if collision_mode=='cad_aabb':result.update(enclosed_surface_bodies=enclosed,open_shell_naming=naming)
    assert hashlib.sha256(source.read_bytes()).hexdigest()==sha
    (out/'cad.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
    print(json.dumps(dict(sha256=sha,parts=len(parts),bounds_m=result['bounds_m'],size_m=result['size_m']),indent=2))
    return result


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
