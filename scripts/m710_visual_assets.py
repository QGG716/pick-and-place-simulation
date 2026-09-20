"""Shared visual-only asset entry for scene initialization and recorded replay."""
from pathlib import Path
import hashlib
import json


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def physical_snapshot(stage):
    """Capture original schemas, geometry, transforms and physical bindings."""
    snapshot = {}
    for prim in stage.Traverse():
        attrs = {}
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if name not in ('visibility','primvars:displayOpacity'):
                attrs[name] = str(attr.Get())
        relations = {r.GetName():str(r.GetTargets()) for r in prim.GetRelationships()
                     if 'physics' in r.GetName().lower() or r.GetName().startswith('physics:')}
        snapshot[str(prim.GetPath())] = dict(apis=list(prim.GetAppliedSchemas()),attrs=attrs,relations=relations)
    return snapshot


def sanitize_visual(root):
    from pxr import Usd, Sdf
    removed = []
    for prim in Usd.PrimRange(root):
        if prim.IsInstanceable(): prim.SetInstanceable(False)
    for prim in list(Usd.PrimRange(root)):
        kind = prim.GetTypeName().lower()
        if any(s in kind for s in ('joint','physicsScene'.lower(),'camera','light','omniGraph'.lower())):
            removed.append([str(prim.GetPath()),kind]); prim.SetActive(False); continue
        schemas = prim.GetMetadata('apiSchemas')
        kept = []
        for api in (schemas.GetAppliedItems() if schemas else []):
            if any(s in api.lower() for s in ('physics','physx','sensor','articulation','semantics','labels')):
                removed.append([str(prim.GetPath()),api])
            else: kept.append(api)
        prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(kept))
        for attr in prim.GetAttributes():
            if attr.GetName().startswith(('physics:','physx','sensor:','omnigraph:')): attr.Block()
        for relation in prim.GetRelationships():
            if 'physics' in relation.GetName().lower(): relation.SetTargets([])
    return removed


def audit_visual(root):
    from pxr import Usd, UsdGeom, UsdShade, Sdf
    meshes, dependencies = [], {}
    for prim in Usd.PrimRange(root):
        assert not any(s in api.lower() for api in prim.GetAppliedSchemas()
                       for s in ('physics','physx','sensor','articulation'))
        assert not any(s in prim.GetTypeName().lower() for s in ('joint','physicsscene','camera','light','omnigraph'))
        if prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible':
            m=UsdGeom.Mesh(prim); uv=UsdGeom.PrimvarsAPI(prim).GetPrimvar('st')
            assert m.GetPointsAttr().Get(), str(prim.GetPath())
            material=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            subsets=UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()
            assert material or subsets, str(prim.GetPath())
            meshes.append(dict(path=str(prim.GetPath()),points=len(m.GetPointsAttr().Get()),
                               uv_count=len(uv.Get()) if uv and uv.Get() else 0))
        for attr in prim.GetAttributes():
            if attr.GetTypeName()!=Sdf.ValueTypeNames.Asset: continue
            value=attr.Get()
            if not value or not value.path: continue
            if value.path in ('OmniPBR.mdl','OmniGlass.mdl','OmniSurface.mdl'): continue
            from urllib.parse import unquote
            decoded_path=unquote(value.path)
            if '<UDIM>' in decoded_path:
                authored=next(p.layer for p in attr.GetPropertyStack() if p.default is not None)
                template=Path(Sdf.ComputeAssetPathRelativeToLayer(authored,decoded_path))
                tiles=list(template.parent.glob(template.name.replace('<UDIM>','[1-9][0-9][0-9][0-9]')))
                assert tiles, 'UNRESOLVED_UDIM '+str(template)
                attr.Set(Sdf.AssetPath(str(template)))
                dependencies.update({str(p):digest(p) for p in tiles})
                continue
            if not value.resolvedPath or not Path(value.resolvedPath).is_file():
                raise ValueError('UNRESOLVED_VISUAL_DEPENDENCY '+str(value))
            dependencies[value.resolvedPath]=digest(value.resolvedPath)
    assert meshes
    return dict(meshes=meshes,dependencies=dependencies,no_physics_control_or_lights=True)


def apply_visual_assets(namespace, config_path, cache):
    import numpy as np
    from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf, Vt
    from carton_appearance import attach_usd_carton
    from workcell_visuals import attach_a06_belts
    config_path,cache=Path(config_path),Path(cache)
    config=json.loads(config_path.read_text())
    cartons=json.loads((config_path.parent/config['cartons']).read_text())
    conveyors=json.loads((config_path.parent/config['conveyors']).read_text())
    stage=namespace['stage'];before=physical_snapshot(stage)
    report=dict(visual_asset_swap=True,new_physics_execution=False,source_config=config,
                cartons={},conveyors={},dependencies=[],chassis={})
    for cfg,subdir in [(cartons,'cartons'),(conveyors,'workcell')]:
        for entry in cfg['dependencies']:
            path=cache/subdir/entry['path']
            assert digest(path)==entry['sha256'],str(path)
            report['dependencies'].append(dict(path=str(path),sha256=entry['sha256'],bytes=path.stat().st_size))
    assets={a['id']:a for a in cartons['assets']}
    belts={}
    invisible=UsdShade.Material.Define(stage,'/Validation/Materials/InvisiblePhysicalProxy')
    shader=UsdShade.Shader.Define(stage,str(invisible.GetPath())+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput('opacityThreshold',Sdf.ValueTypeNames.Float).Set(.5)
    invisible.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    def hide_old(parent):
        for child in parent.GetChildren():
            if child.GetName() in ('TexturedVisual','Decal'):
                UsdGeom.Imageable(child).MakeInvisible()
        # Opacity is local Gprim appearance; visibility remains inherited so the
        # new child meshes stay visible and the original physics body survives.
        UsdGeom.Gprim(parent).CreateDisplayOpacityAttr([0.0])
        UsdShade.MaterialBindingAPI.Apply(parent).Bind(invisible,UsdShade.Tokens.weakerThanDescendants)
    for item in namespace['scene_primitives']:
        name=item['name'];parent=stage.GetPrimAtPath('/Validation/Scene/'+name)
        path=str(parent.GetPath())+'/AssetVisual'
        if item['category']=='carton':
            hide_old(parent)
            entry=assets[cartons['object_overrides'].get(name,cartons['default_asset'])]
            record=attach_usd_carton(stage,path,item['size_m'],entry,cache/'cartons')
            root=stage.GetPrimAtPath(path);sanitize_visual(root)
            record['audit']=audit_visual(root)
            # Vertex envelope in the unit-scaled entity frame: no double scaling.
            xf=UsdGeom.XformCache();inverse=xf.GetLocalToWorldTransform(parent).GetInverse()
            pts=[]
            for p in Usd.PrimRange(root):
                if p.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible':
                    t=xf.GetLocalToWorldTransform(p)*inverse
                    pts.extend(list(t.Transform(v)) for v in UsdGeom.Mesh(p).GetPointsAttr().Get())
            pts=np.asarray(pts);bounds=[pts.min(0).tolist(),pts.max(0).tolist()]
            assert np.allclose(bounds,[[-.5]*3,[.5]*3],atol=1e-6),bounds
            record['entity_local_bounds']=bounds; report['cartons'][name]=record
        elif item['category']=='conveyor':
            hide_old(parent)
            UsdGeom.Xform.Define(stage,path)
            record=attach_a06_belts(stage,path,dict(item,size_xyz_m=item['size_m']),
                                    conveyors,cache/'workcell',namespace['conveyor_frame_material'])
            root=stage.GetPrimAtPath(path);sanitize_visual(root)
            record.pop('velocity_m_s')  # source's static placeholder is NOT a speed command
            record['audit']=audit_visual(root); report['conveyors'][name]=record
            segments=[]
            for segment in record['segments']:
                mesh=UsdGeom.Mesh(stage.GetPrimAtPath(segment['mesh_path']))
                uv=UsdGeom.PrimvarsAPI(mesh).GetPrimvar('st')
                raw=np.asarray(uv.ComputeFlattened(),dtype=float)
                uv.SetIndices(Vt.IntArray())
                indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
                points=np.asarray(mesh.GetPointsAttr().Get())*item['size_m']
                counts=list(mesh.GetFaceVertexCountsAttr().Get());offset=0
                rates=np.zeros_like(raw);direction=np.array([0.,-1.,0.] if name=='conveyor_transverse' else [-1.,0.,0.])
                for count in counts:
                    q=points[indices[offset:offset+count]]
                    tex=raw[offset:offset+count]
                    # Fit each chart in its face plane. Only the top/bottom skin
                    # scrolls; end faces remain a stationary wrap-around edge.
                    if np.ptp(q[:,2])<1e-5 and np.linalg.matrix_rank(q[:,:2]-q[0,:2])==2:
                        coeff=np.linalg.lstsq(np.column_stack([q[:,:2],np.ones(count)]),tex,rcond=None)[0]
                        rates[offset:offset+count]=direction[:2]@coeff[:2]
                    offset+=count
                assert np.any(np.abs(rates)>.01)
                segments.append(dict(uv=uv,original=raw,rates=rates))
            # Small asymmetric belt splice paint, fixed to the moving skin.
            # Non-periodic spacing makes the true UV movement legible at 5 Hz.
            marks=[];size=np.array(item['size_m']);trans=name=='conveyor_transverse'
            length=size[1 if trans else 0]
            for i,fraction in enumerate([.08,.27,.59,.83]):
                mark=UsdGeom.Cube.Define(stage,path+f'/Splice_{i}')
                mark.CreateSizeAttr(1.)
                mark.CreateDisplayColorAttr([Gf.Vec3f(.33,.35,.36)])
                mat=namespace['conveyor_marker_material']
                UsdShade.MaterialBindingAPI.Apply(mark.GetPrim()).Bind(mat)
                xform=UsdGeom.XformCommonAPI(mark.GetPrim())
                dims=np.array([.12+.018*i,.012,.0006] if trans else [.012,.12+.018*i,.0006])
                xform.SetScale(Gf.Vec3f(*(dims/size).tolist()))
                marks.append(dict(xform=xform,offset=fraction*length))
            belts[name]=dict(segments=segments,marks=marks,size=size,length=length,transverse=trans)
    for path in ['/Validation/ConveyorMotionMarkers','/Validation/ConveyorEquipment']:
        prim=stage.GetPrimAtPath(path)
        if prim: UsdGeom.Imageable(prim).MakeInvisible()
    # Reference only default /iw_hub, excluding the source ground and lights.
    parent=stage.GetPrimAtPath('/Validation/Scene/chassis');hide_old(parent)
    item=next(p for p in namespace['scene_primitives'] if p['name']=='chassis')
    size=np.asarray(item['size_m']);path=str(parent.GetPath())+'/AssetVisual'
    root=UsdGeom.Xform.Define(stage,path);ref=stage.DefinePrim(path+'/Official')
    source=cache/config['chassis']['relative_path'];original=Usd.Stage.Open(str(source))
    assert digest(source)==config['chassis']['sha256']
    assert original and str(original.GetDefaultPrim().GetPath())==config['chassis']['subtree']
    assert UsdGeom.GetStageMetersPerUnit(original)==1 and UsdGeom.GetStageUpAxis(original)=='Z'
    ref.GetReferences().AddReference(str(source),config['chassis']['subtree'])
    removed=sanitize_visual(ref)
    xf=UsdGeom.XformCache();inverse=xf.GetLocalToWorldTransform(parent).GetInverse();pts=[]
    for p in Usd.PrimRange(ref):
        if p.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(p).ComputeVisibility()!='invisible':
            t=xf.GetLocalToWorldTransform(p)*inverse
            pts.extend(list(t.Transform(v)) for v in UsdGeom.Mesh(p).GetPointsAttr().Get())
    pts=np.asarray(pts);low,high=pts.min(0),pts.max(0);span=high-low
    scale=float(np.min(size/span));translation=-(low+high)*.5*scale
    translation[2]=-size[2]/2-low[2]*scale
    matrix=Gf.Matrix4d(1);matrix.SetScale(Gf.Vec3d(*(scale/size).tolist()))
    matrix.SetTranslateOnly(Gf.Vec3d(*(translation/size).tolist()));root.AddTransformOp().Set(matrix)
    # Reuse the existing mounting structure's dark material, maintaining its
    # top/interface height. This is a stationary connector above the low AGV.
    top=-size[2]/2+span[2]*scale;gap=size[2]/2-top
    if gap>1e-4:
        support=UsdGeom.Cube.Define(stage,str(parent.GetPath())+'/MountingSupportVisual')
        support.CreateSizeAttr(1);api=UsdGeom.XformCommonAPI(support.GetPrim())
        base=np.asarray(namespace['base_position'])-np.asarray(item['center_m'])
        api.SetTranslate(Gf.Vec3d(float(base[0]/size[0]),float(base[1]/size[1]),float((top+gap/2)/size[2])))
        api.SetScale(Gf.Vec3f(.65/size[0],.65/size[1],gap/size[2]))
        UsdShade.MaterialBindingAPI.Apply(support.GetPrim()).Bind(namespace['_preview_material']('/Validation/Materials/RetainedChassisSupport', {'fallback_rgb': list(namespace['category_colors']['chassis']), 'roughness': .6}))
    report['chassis']=dict(source=config['chassis'],sha256=digest(source),source_bounds=[low.tolist(),high.tolist()],
        uniform_scale=scale,adapted_size_m=(span*scale).tolist(),mounting_gap_m=gap,removed_source_apis=removed,
        audit=audit_visual(ref),wheel_animation=False)
    after=physical_snapshot(stage)
    differences=[p for p,record in before.items() if after.get(p)!=record]
    assert not differences, 'ORIGINAL_PHYSICS_OR_TRANSFORM_CHANGED '+str(differences[:10])
    report['unchanged_original_physics_and_transforms']=dict(status='PASS',original_prim_count=len(before))
    report['asset_mapping_count']=len(report['cartons'])
    assert report['asset_mapping_count']==sum(p['category']=='carton' for p in namespace['scene_primitives'])
    visual=VisualAssets(belts,report)
    zero={name:0. for name in belts}
    visual.update(zero)
    def position(belt):
        prim=belt['marks'][0]['xform'].GetPrim()
        return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation())
    checks={}
    for name,belt in belts.items():
        visual.update(zero);start=position(belt)
        other={n:np.asarray(v['segments'][0]['uv'].Get()).copy() for n,v in belts.items() if n!=name}
        phase=dict(zero);phase[name]=.06;visual.update(phase)
        displacement=position(belt)-start
        expected=[0.,-.06,0.] if belt['transverse'] else [-.06,0.,0.]
        assert np.allclose(displacement,expected,atol=1e-7),displacement
        assert all(np.array_equal(old,np.asarray(belts[n]['segments'][0]['uv'].Get())) for n,old in other.items())
        held=np.asarray(belt['segments'][0]['uv'].Get()).copy();visual.update(phase)
        assert np.array_equal(held,np.asarray(belt['segments'][0]['uv'].Get()))
        checks[name]=dict(world_displacement_for_0_2s_at_0_3m_s=displacement.tolist(),
                         other_belt_uv_unchanged=True,stopped_phase_exactly_held=True)
    visual.update(zero);report['belt_direction_and_independence_checks']=checks
    return visual


class VisualAssets:
    def __init__(self,belts,report): self.belts,self.report=belts,report

    def update(self,phase_m):
        import numpy as np
        from pxr import Gf,Vt
        for name,belt in self.belts.items():
            distance=float(phase_m[name])
            for segment in belt['segments']:
                values=segment['original']-distance*segment['rates']
                segment['uv'].Set(Vt.Vec2fArray([Gf.Vec2f(*v) for v in values]))
            for mark in belt['marks']:
                travel=(mark['offset']-distance)%belt['length']-belt['length']/2
                # World axes: transverse -Y, longitudinal -X. Only belt skin marks move.
                q=np.array([0.,travel,.5*belt['size'][2]+.0004] if belt['transverse']
                           else [travel,0.,.5*belt['size'][2]+.0004])/belt['size']
                mark['xform'].SetTranslate(Gf.Vec3d(*q.tolist()))
