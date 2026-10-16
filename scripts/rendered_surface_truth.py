"""Export composed render mesh geometry for post-freeze evaluation only."""
import hashlib
import json
from pathlib import Path


def export_surface_truth(stage, records, output):
    import numpy as np
    from pxr import UsdGeom
    output = Path(output); arrays = {}; objects = []
    cache = UsdGeom.XformCache()
    for ordinal, (identity, record) in enumerate(sorted(records.items())):
        vertices, triangles = [], []
        for description in record['meshes']:
            prim = stage.GetPrimAtPath(description['path']); mesh = UsdGeom.Mesh(prim)
            transform = cache.GetLocalToWorldTransform(prim)
            points = np.asarray([list(transform.Transform(p)) for p in mesh.GetPointsAttr().Get()])
            offset = len(vertices); vertices.extend(points.tolist())
            indices = list(mesh.GetFaceVertexIndicesAttr().Get()); start = 0
            for count in mesh.GetFaceVertexCountsAttr().Get():
                polygon = indices[start:start+count]; start += count
                triangles.extend([[offset+polygon[0],offset+polygon[i],offset+polygon[i+1]] for i in range(1,count-1)])
        key = f'object_{ordinal:03d}'
        arrays[key+'_vertices_world_m'] = np.asarray(vertices, np.float64)
        arrays[key+'_triangles'] = np.asarray(triangles, np.int32)
        objects.append(dict(simulation_object_id=identity,array_prefix=key,asset_id=record['asset_id'],
                            source_sha256=record['source_sha256'],mesh_paths=[m['path'] for m in record['meshes']]))
    path = output/'rendered_surface_meshes.npz'; np.savez_compressed(path, **arrays)
    (output/'rendered_surface_truth.json').write_text(json.dumps(dict(
        schema_version='rendered_surface_truth_v1',objects=objects,mesh_file=path.name,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),coordinate_frame='world',units='metres',
        usage='SCENE_VISIBILITY_AND_POSTFREEZE_EVALUATION_ONLY; never solver input',
        geometry_source='actual composed visible USD mesh vertices and triangulated topology; not nominal cuboid'),indent=2),encoding='utf-8')
