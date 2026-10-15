"""Static desktop composition of pinned A06; original bytes remain immutable."""
import hashlib
import os
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def clipped_mesh(mesh, matrix, plane_z):
    """Clip polygons and interpolate face-varying UV/normals at the table plane.

    Only lower supports are shortened. All retained parts undergo the same
    uniform scale; rollers, upper frame, guards and decals are not hidden.
    """
    vertices=np.array(mesh.GetPointsAttr().Get());vertices=vertices@matrix[:3,:3].T+matrix[:3,3]
    counts=list(mesh.GetFaceVertexCountsAttr().Get());indices=np.array(mesh.GetFaceVertexIndicesAttr().Get())
    attrs={}
    for p in UsdGeom.PrimvarsAPI(mesh).GetPrimvars():
        if p.GetInterpolation()=='faceVarying': attrs[p.GetPrimvarName()]=np.array(p.ComputeFlattened())
    normals=np.array(mesh.GetNormalsAttr().Get())
    if len(normals):
        assert mesh.GetNormalsInterpolation()=='faceVarying'
        normals=normals@np.linalg.inv(matrix[:3,:3]);normals/=np.linalg.norm(normals,axis=1)[:,None]
        attrs['_normal']=normals
    result=[];newcounts=[];mapping={};values={k:[] for k in attrs};offset=0
    for face,n in enumerate(counts):
        ids=indices[offset:offset+n];v=vertices[ids]
        poly=[(v[i],{k:a[offset+i] for k,a in attrs.items()}) for i in range(n)];offset+=n
        if np.min(v[:,2])<plane_z:
            clipped=[]
            for a,b in zip(poly,poly[1:]+poly[:1]):
                inside_a=a[0][2]>=plane_z;inside_b=b[0][2]>=plane_z
                if inside_a:clipped.append(a)
                if inside_a!=inside_b:
                    t=(plane_z-a[0][2])/(b[0][2]-a[0][2])
                    clipped.append((a[0]+t*(b[0]-a[0]),{k:a[1][k]+t*(b[1][k]-a[1][k]) for k in attrs}))
            poly=clipped
        if len(poly)<3:continue
        mapping[face]=len(newcounts);newcounts.append(len(poly))
        result.extend(v for v,a in poly)
        for k in attrs:values[k].extend(a[k] for v,a in poly)
    return np.array(result),newcounts,values,mapping


def build(cache,lock,config,out):
    cache=Path(cache).resolve();out=Path(out).resolve()
    for row in lock['dependencies']:
        p=cache/row['path'];p.resolve().relative_to(cache)
        if sha(p)!=row['sha256']:raise ValueError('CONVEYOR_SOURCE_CHANGED:'+row['path'])
    # USD's authored percent-encoded local sublayer needs a byte-identical alias.
    material=cache/'Assets/Isaac/5.0/Isaac/Props/Conveyors/Material Library/physics_material.usd'
    alias=material.parent.parent/'Material%20Library/physics_material.usd'
    alias.parent.mkdir(exist_ok=True)
    if alias.exists() and sha(alias)!=sha(material):raise ValueError('CONVEYOR_ALIAS_CHANGED')
    if not alias.exists():alias.write_bytes(material.read_bytes())
    source=cache/lock['a06']['path'];original=Usd.Stage.Open(str(source))
    assert UsdGeom.GetStageMetersPerUnit(original)==1 and UsdGeom.GetStageUpAxis(original)=='Z'
    assert not original.GetCompositionErrors()
    xc=UsdGeom.XformCache();records=[]
    for p in original.Traverse():
        if not p.IsA(UsdGeom.Mesh):continue
        m=UsdGeom.Mesh(p);T=np.array(xc.GetLocalToWorldTransform(p)).T
        v=np.array(m.GetPointsAttr().Get());v=v@T[:3,:3].T+T[:3,3]
        records.append((p,m,T,v))
    belt=next(v for p,m,T,v in records if str(p.GetPath())==lock['a06']['belt_prim'])
    low,high=belt.min(0),belt.max(0);scale=config['conveyor']['usable_width_m']/(high[1]-low[1])
    center=(low+high)/2;top=config['belt_top_D_m']
    allv=np.concatenate([v for p,m,T,v in records]);rawlo,rawhi=allv.min(0),allv.max(0)
    transverse=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
    longitudinal=np.diag([-1.,-1.,1.])
    tx=config['conveyor']['transverse_center_x_D_m'];ty=config['conveyor']['transverse_center_y_D_m']
    tmin_y=ty-(rawhi[0]-center[0])*scale
    ly=tmin_y-config['conveyor']['junction_structure_gap_m']-(center[1]-rawlo[1])*scale
    lx=tx-(center[0]-rawlo[0])*scale
    items=[]
    for name,R,xy in [('transverse',transverse,[tx,ty]),('longitudinal',longitudinal,[lx,ly])]:
        path=out/(name+'.usdc');stage=Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
        root=UsdGeom.Xform.Define(stage,'/Conveyor').GetPrim();stage.SetDefaultPrim(root)
        root.GetReferences().AddReference(Path(os.path.relpath(source,out)).as_posix())
        for p in Usd.PrimRange(root):
            if p.IsInstanceable():p.SetInstanceable(False)
        for p in Usd.PrimRange(root):
            schemas=p.GetMetadata('apiSchemas')
            p.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit([v for v in (schemas.GetAppliedItems() if schemas else []) if not any(k in v.lower() for k in ['physics','physx','conveyor','script'])]))
            for a in p.GetAttributes():
                if a.GetName().startswith(('physics:','physx','physxSurfaceVelocity','omni:graph','conveyor:')):a.Block()
            if p.IsA(UsdGeom.Xformable):UsdGeom.Xformable(p).ClearXformOpOrder()
        meshes=[]
        A=np.eye(4);A[:3,:3]=scale*R
        A[:3,3]=np.array([*xy,top])-scale*R@np.array([center[0],center[1],high[2]])
        for p,m,T,v in records:
            target='/Conveyor'+str(p.GetPath()).removeprefix('/World');dest=UsdGeom.Mesh(stage.GetPrimAtPath(target))
            q,counts,attrs,mapping=clipped_mesh(m,A@T,0.)
            dest.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(q.astype(np.float32)))
            dest.GetFaceVertexCountsAttr().Set(counts);dest.GetFaceVertexIndicesAttr().Set(list(range(len(q))))
            dest.CreateExtentAttr().Set([Gf.Vec3f(*q.min(0)),Gf.Vec3f(*q.max(0))])
            for key,value in attrs.items():
                if key=='_normal':dest.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(np.array(value,np.float32)))
                else:
                    pv=UsdGeom.PrimvarsAPI(dest).GetPrimvar(key);pv.BlockIndices();pv.Set(Vt.Vec2fArray.FromNumpy(np.array(value,np.float32)))
            for child in dest.GetPrim().GetChildren():
                if child.IsA(UsdGeom.Subset):
                    subset=UsdGeom.Subset(child);subset.GetIndicesAttr().Set([mapping[i] for i in subset.GetIndicesAttr().Get() if i in mapping])
            collision='Decal' not in target
            if collision:
                UsdPhysics.CollisionAPI.Apply(dest.GetPrim()).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(dest.GetPrim()).CreateApproximationAttr('none')
            meshes.append(dict(prim=target,bounds_D_m=[q.min(0).tolist(),q.max(0).tolist()],collision=collision,
                               points_sha256=hashlib.sha256(q.astype(np.float32).tobytes()).hexdigest(),source_prim=str(p.GetPath()),faces=len(counts)))
        stage.GetRootLayer().Save()
        b=next(r for r in meshes if r['source_prim']==lock['a06']['belt_prim'])['bounds_D_m']
        bounds=np.array([r['bounds_D_m'] for r in meshes]);outer=[bounds[:,0].min(0).tolist(),bounds[:,1].max(0).tolist()]
        items.append(dict(id='conveyor_'+name,usd_path=str(path),usd_sha256=sha(path),meshes=meshes,belt_bounds_D_m=b,bounds_D_m=outer,
                          uniform_scale=scale,direction_D=[0,-1,0] if name=='transverse' else [-1,0,0],velocity_m_s=0.,
                          collision='single_exact_static_triangle_mesh_per_non_decal_source_mesh',cut_plane_D_z_m=0.))
    return dict(source=lock['a06'],asset_version='5.0',dependencies_verified=len(lock['dependencies']),
                source_units_m=1.,source_axis='Z',source_belt_bounds_m=[low.tolist(),high.tolist()],source_bounds_m=[rawlo.tolist(),rawhi.tolist()],
                adaptation=config['conveyor']['adaptation'],a43_status=lock['a43_status'],items=items,
                transfer_status='NOT_IMPLEMENTED_OR_TESTED',lower_supports='CUT_AT_TABLE_NO_BURIED_LEGS_NO_WHOLE_FRAME_NONUNIFORM_SCALE')
