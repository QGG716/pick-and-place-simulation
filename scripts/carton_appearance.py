"""USD carton adapter ported from perception cc64bbd; render-only, no ROS dependencies."""
from pathlib import Path
import hashlib
import math


def merge_instance_mask(masks, identities, object_id, numeric_id, mask):
    """Aggregate visible submeshes without losing earlier parts of a carton."""
    masks[object_id] = masks.get(object_id, False) | mask
    identities.setdefault(object_id, set()).add(int(numeric_id))


def normalization(bounds_min, bounds_max, nominal_size, max_anisotropy=1.35):
    """Map an inspected source envelope to the unit collision box, without gaps."""
    spans = [float(b)-float(a) for a, b in zip(bounds_min, bounds_max)]
    if len(spans) != 3 or any(not math.isfinite(v) or v <= 0 for v in spans):
        raise ValueError('INVALID_ASSET_ENVELOPE')
    if len(nominal_size) != 3 or any(not math.isfinite(v) or v <= 0 for v in nominal_size):
        raise ValueError('INVALID_NOMINAL_SIZE')
    scale = [float(n)/s for n, s in zip(nominal_size, spans)]
    if max(scale)/min(scale) > max_anisotropy:
        raise ValueError('CARTON_ASSET_ASPECT_RATIO_MISMATCH')
    return [1/s for s in spans], [-(a+b)/(2*s) for a,b,s in zip(bounds_min,bounds_max,spans)]


def attach_usd_carton(stage, path, nominal_size, entry, cache):
    """Reference complete mesh/UV/material hierarchy under the unchanged entity."""
    import numpy as np
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    source = (Path(cache)/entry['path']).resolve()
    if not source.is_relative_to(Path(cache).resolve()) or not source.is_file():
        raise ValueError('CARTON_USD_MISSING')
    if hashlib.sha256(source.read_bytes()).hexdigest() != entry['sha256']:
        raise ValueError('CARTON_USD_HASH_MISMATCH')
    original = Usd.Stage.Open(str(source))
    if not original or not original.GetDefaultPrim():
        raise ValueError('CARTON_DEFAULT_PRIM_MISSING')
    if UsdGeom.GetStageUpAxis(original) != 'Z' and not entry.get('rotation_xyz_deg'):
        raise ValueError('CARTON_UP_AXIS_REQUIRES_EXPLICIT_ADAPTER')
    root = UsdGeom.Xform.Define(stage,path)
    orientation = UsdGeom.Xform.Define(stage,path+'/Orientation')
    if entry.get('rotation_xyz_deg'):
        orientation.AddRotateXYZOp().Set(Gf.Vec3f(*entry['rotation_xyz_deg']))
    reference = stage.DefinePrim(path+'/Orientation/Asset')
    reference.GetReferences().AddReference(str(source))
    # Source physics and semantics cannot override project-owned entities.
    removed = []
    for prim in Usd.PrimRange(reference):
        if prim.IsInstanceable():
            prim.SetInstanceable(False)
    for prim in Usd.PrimRange(reference):
        if prim.GetTypeName() in ('PhysicsScene','Camera') or prim.GetTypeName().endswith('Light'):
            raise ValueError('NON_CARTON_CONTENT_IN_ASSET')
        schemas=prim.GetMetadata('apiSchemas')
        kept=[]
        for schema in list(schemas.GetAppliedItems() if schemas else ()):
            if any(x in schema.lower() for x in ('physics','physx','semantics','labels')):
                removed.append([str(prim.GetPath()),schema])
            else:
                kept.append(schema)
        prim.SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(kept))
        for attr in prim.GetAttributes():
            if attr.GetName().startswith(('physics:','physx','semantic:','semantics:')):
                attr.Block()
    # Measure composed transforms (including internal cm->m scaling) once.
    # Rotating an authored AABB overestimates a scan's envelope and creates gaps.
    # Measure the actual visible vertices in the visual adapter's coordinates.
    xforms=UsdGeom.XformCache()
    to_root=xforms.GetLocalToWorldTransform(root.GetPrim()).GetInverse()
    vertices=[]
    for prim in Usd.PrimRange(reference):
        if prim.IsA(UsdGeom.Mesh) and UsdGeom.Imageable(prim).ComputeVisibility()!='invisible':
            transform=xforms.GetLocalToWorldTransform(prim)*to_root
            vertices.extend([list(transform.Transform(v)) for v in UsdGeom.Mesh(prim).GetPointsAttr().Get()])
    if not vertices:
        raise ValueError('CARTON_HAS_NO_RENDER_MESH')
    points=np.asarray(vertices)
    low, high=points.min(0).tolist(),points.max(0).tolist()
    scale, translate = normalization(low,high,nominal_size,entry.get('max_anisotropy',1.35))
    matrix=Gf.Matrix4d(1)
    matrix.SetScale(Gf.Vec3d(*scale));matrix.SetTranslateOnly(Gf.Vec3d(*translate))
    root.AddTransformOp().Set(matrix)
    meshes=[]
    material_files=[]
    for prim in Usd.PrimRange(reference):
        if not prim.IsA(UsdShade.Shader):
            continue
        for attr in prim.GetAttributes():
            if attr.GetTypeName()!=Sdf.ValueTypeNames.Asset:
                continue
            value=attr.Get()
            if not value or not value.path:
                continue
            if value.path in ('OmniPBR.mdl','OmniGlass.mdl'):
                material_files.append({'asset':value.path,'resolver':'ISAAC_RUNTIME_MDL'})
            elif not value.resolvedPath or not Path(value.resolvedPath).is_file():
                raise ValueError('CARTON_MATERIAL_DEPENDENCY_UNRESOLVED: '+value.path)
            else:
                material_files.append({'asset':value.path,'sha256':hashlib.sha256(Path(value.resolvedPath).read_bytes()).hexdigest()})
    for prim in Usd.PrimRange(reference):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh=UsdGeom.Mesh(prim)
        if UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible':
            continue
        points=mesh.GetPointsAttr().Get()
        uv=UsdGeom.PrimvarsAPI(prim).GetPrimvar('st')
        if not points or not uv or not uv.Get():
            raise ValueError('CARTON_MESH_OR_UV_MISSING')
        material=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        subsets=UsdShade.MaterialBindingAPI(prim).GetMaterialBindSubsets()
        if not material and not subsets:
            raise ValueError('CARTON_MATERIAL_MISSING')
        meshes.append({'path':str(prim.GetPath()),'points':len(points),'uv_count':len(uv.Get()),
                       'material':str(material.GetPath()) if material else None,
                       'material_subsets':[str(s.GetPath()) for s in subsets]})
    if not meshes:
        raise ValueError('CARTON_HAS_NO_RENDER_MESH')
    return {'asset_id':entry['id'],'source_sha256':entry['sha256'],'source_path':entry['path'],
            'source_meters_per_unit':UsdGeom.GetStageMetersPerUnit(original),
            'source_up_axis':UsdGeom.GetStageUpAxis(original),'default_prim':str(original.GetDefaultPrim().GetPath()),
            'composed_source_bounds':[low,high],'unit_scale':scale,'unit_translation':translate,
            'nominal_size_m':list(nominal_size),'meshes':meshes,'removed_source_apis':removed,
            'material_files':material_files,'rotation_xyz_deg':entry.get('rotation_xyz_deg',[0,0,0]),
            'collision':'UNCHANGED_INVISIBLE_NOMINAL_CONSERVATIVE_BOX',
            'surface_truth':'RENDERED_DEPTH_AND_INSTANCE_MASK; NOMINAL_CORNERS_NOT_EXACT_SURFACE_TRUTH'}
