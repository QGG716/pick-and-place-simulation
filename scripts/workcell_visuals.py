"""Optional USD adapters for the static perception workcell (not core dependencies)."""
from pathlib import Path
import hashlib
import math


def attach_a06_belts(stage, path, item, config, cache, frame_material):
    """Reuse the official belt subassembly, preserving its thickness and UVs.

    The source's 2.31 m tall integrated frame is deliberately not scaled into a
    0.12 m envelope. Project-owned low frames retain the existing collision box.
    """
    import numpy as np
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt
    entry = config['a06']
    source = (Path(cache) / entry['path']).resolve()
    if not source.is_relative_to(Path(cache).resolve()) or hashlib.sha256(source.read_bytes()).hexdigest() != entry['sha256']:
        raise ValueError('A06 source identity mismatch')
    original = Usd.Stage.Open(str(source))
    mesh = UsdGeom.Mesh(original.GetPrimAtPath(entry['belt_prim']))
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(mesh.GetPrim())
    points = np.asarray([list(transform.Transform(v)) for v in mesh.GetPointsAttr().Get()])
    low, high = points.min(0), points.max(0)
    size = np.asarray(item['size_xyz_m'], float)
    transverse = item['name'] == 'conveyor_transverse'
    length = size[1] if transverse else size[0]
    width = size[0] if transverse else size[1]
    count = math.ceil(length / 2.)
    segment_length = length / count
    thickness = float(high[2] - low[2])
    records = []
    for i in range(count):
        ref_path = f'{path}/A06Segment{i}'
        reference = stage.DefinePrim(ref_path)
        reference.GetReferences().AddReference(str(source))
        for prim in Usd.PrimRange(reference):
            if prim.IsInstanceable(): prim.SetInstanceable(False)
        for prim in Usd.PrimRange(reference):
            schemas = prim.GetMetadata('apiSchemas')
            prim.SetMetadata('apiSchemas', Sdf.TokenListOp.CreateExplicit([
                v for v in (schemas.GetAppliedItems() if schemas else [])
                if not any(k in v.lower() for k in ('physics', 'physx', 'semantics', 'labels'))]))
            for attr in prim.GetAttributes():
                if attr.GetName().startswith(('physics:', 'physx')): attr.Block()
            if prim.IsA(UsdGeom.Mesh): UsdGeom.Imageable(prim).MakeInvisible()
        target_path = ref_path + entry['belt_prim'][len(str(original.GetDefaultPrim().GetPath())):]
        target = UsdGeom.Mesh(stage.GetPrimAtPath(target_path))
        UsdGeom.Imageable(target).MakeVisible()
        # Reference keeps topology, UVs, bindings and material hierarchy. Only
        # this working segment's vertex coordinates and normals are adapted.
        q = points.copy()
        q[:, 0] = (q[:, 0] - low[0]) * segment_length / (high[0] - low[0]) - length / 2 + i * segment_length
        q[:, 1] = (q[:, 1] - (low[1] + high[1]) / 2) * width / (high[1] - low[1])
        q[:, 2] = q[:, 2] - high[2] + size[2] / 2
        linear = np.diag([segment_length / (high[0] - low[0]), width / (high[1] - low[1]), 1.])
        if transverse:
            rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
            q = q @ rotation.T; linear = rotation @ linear
        q /= size
        linear = np.diag(1 / size) @ linear @ np.asarray(transform).T[:3, :3]
        current = target.GetPrim()
        while current.GetPath() != reference.GetParent().GetPath():
            if current.IsA(UsdGeom.Xformable): UsdGeom.Xformable(current).ClearXformOpOrder()
            if current == reference: break
            current = current.GetParent()
        target.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in q]))
        target.CreateExtentAttr().Set([Gf.Vec3f(*q.min(0)), Gf.Vec3f(*q.max(0))])
        normals = target.GetNormalsAttr().Get()
        if normals:
            n = np.asarray(normals) @ np.linalg.inv(linear)
            n /= np.linalg.norm(n, axis=1)[:, None]
            target.GetNormalsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in n]))
        material = UsdShade.MaterialBindingAPI(target.GetPrim()).ComputeBoundMaterial()[0]
        if not material: raise ValueError('A06 belt material missing')
        uv = UsdGeom.PrimvarsAPI(target).GetPrimvar('st')
        if not uv or not uv.Get(): raise ValueError('A06 belt UV missing')
        records.append({'mesh_path': target_path, 'vertices': len(q), 'material': str(material.GetPath()),
                        'unit_bounds': [q.min(0).tolist(), q.max(0).tolist()], 'segment_length_m': segment_length})
    frame = UsdGeom.Cube.Define(stage, path + '/LowFrame')
    frame.CreateSizeAttr(1.)
    frame_api = UsdGeom.XformCommonAPI(frame.GetPrim())
    frame_api.SetScale(Gf.Vec3f(1., 1., 1. - thickness / size[2]))
    frame_api.SetTranslate(Gf.Vec3d(0., 0., -thickness / (2 * size[2])))
    UsdShade.MaterialBindingAPI.Apply(frame.GetPrim()).Bind(frame_material)
    return {'source': entry, 'source_belt_bounds_m': [low.tolist(), high.tolist()],
            'source_belt_thickness_m': thickness, 'segments': records,
            'adaptation': 'OFFICIAL_BELT_SUBASSEMBLY_ON_PROJECT_LOW_FRAME',
            'excluded_source_content': ['tall_integrated_frame', 'frame_decals'],
            'collision_proxy': path + '/Collision', 'velocity_m_s': 0.,
            'turning_function': 'NOT_IMPLEMENTED_OR_TESTED'}
