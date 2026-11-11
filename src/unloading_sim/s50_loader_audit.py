"""Exact source/loader correspondence, separate from collision qualification.

No mesh is repaired here. A successful audit proves only the reported numeric
coordinate conversion and retained ordered triangle-index multiset.
"""
from __future__ import annotations

from collections import defaultdict, deque
import hashlib
import json

import numpy as np


def array_digest(value):
    value=np.ascontiguousarray(value)
    header=json.dumps(dict(dtype=value.dtype.str,shape=list(value.shape)),sort_keys=True).encode('ascii')
    return hashlib.sha256(header+b'\n'+value.tobytes()).hexdigest()


def _mesh_arrays(vertices,triangles):
    vertices=np.asarray(vertices,dtype=np.float64)
    raw=np.asarray(triangles)
    if (vertices.ndim!=2 or vertices.shape[1:]!=(3,) or not len(vertices)
            or not np.isfinite(vertices).all()):
        raise ValueError('LOADER_AUDIT_INVALID_VERTICES')
    if (raw.ndim!=2 or raw.shape[1:]!=(3,) or not np.issubdtype(raw.dtype,np.integer)
            or not len(raw)):
        raise ValueError('LOADER_AUDIT_INVALID_TRIANGLES')
    triangles=np.asarray(raw,dtype=np.int64)
    if triangles.min()<0 or triangles.max()>=len(vertices):
        raise ValueError('LOADER_AUDIT_TRIANGLE_INDEX_OUT_OF_RANGE')
    return vertices,triangles


def _doubled_area(vertices,triangles):
    points=vertices[triangles]
    return np.linalg.norm(np.cross(points[:,1]-points[:,0],points[:,2]-points[:,0]),axis=1)


def read_actual_coal_mesh(geometry):
    """Read actual collision geometry, not Pin's copied envelope vertex cache."""
    getter=getattr(geometry,'vertices')
    vertices=np.asarray(getter() if callable(getter) else getter,dtype=np.float64).copy()
    nv=getattr(geometry,'num_vertices');nv=int(nv() if callable(nv) else nv)
    nt=getattr(geometry,'num_tris');nt=int(nt() if callable(nt) else nt)
    if len(vertices)!=nv:raise ValueError('COAL_VERTEX_ACCESSOR_COUNT_MISMATCH')
    getter=getattr(geometry,'tri_indices')
    if callable(getter):
        try:raw=getter()
        except TypeError:raw=[getter(i) for i in range(nt)]
    else:raw=getter
    try:triangles=np.asarray(raw,dtype=np.int64)
    except (TypeError,ValueError):
        triangles=np.asarray([[int(t[j]) for j in range(3)] for t in raw],dtype=np.int64)
    if triangles.shape!=(nt,3):raise ValueError('COAL_TRIANGLE_ACCESSOR_COUNT_MISMATCH')
    return _mesh_arrays(vertices,triangles)


def audit_triangle_correspondence(source_vertices,source_triangles,loaded_vertices,loaded_triangles):
    """Prove exact vertex permutation and retained oriented face correspondence.

    Only identity or an exactly observed float64->float32->float64 conversion is
    recognized. Vertex reordering is resolved only by exact unique coordinates;
    unknown conversions or ambiguous duplicate coordinates do not receive an
    approximate tolerance pass. Duplicate source faces use stable occurrence
    order, so deleting a nonzero duplicate cannot hide behind another copy.
    """
    sv,st=_mesh_arrays(source_vertices,source_triangles)
    lv,lt=_mesh_arrays(loaded_vertices,loaded_triangles)
    with np.errstate(over='ignore',invalid='ignore'):
        f32=sv.astype(np.float32).astype(np.float64)
    same_shape=sv.shape==lv.shape
    loaded_order=np.lexsort(lv[:,::-1].T) if same_shape else None
    conversion='UNEXPLAINED_COORDINATE_CHANGE';permutation=None
    for label,candidate in [('FLOAT64_IDENTITY_EXACT',sv),('FLOAT32_ROUNDTRIP_EXACT',f32)]:
        if not same_shape or not np.isfinite(candidate).all():continue
        source_order=np.lexsort(candidate[:,::-1].T)
        ordered=candidate[source_order]
        if not np.array_equal(ordered,lv[loaded_order]):continue
        if len(ordered)>1 and np.any(np.all(ordered[1:]==ordered[:-1],axis=1)):
            conversion='EXACT_COORDINATES_BUT_AMBIGUOUS_DUPLICATE_VERTEX_IDENTITIES';break
        permutation=np.empty(len(lv),dtype=np.int64)
        permutation[loaded_order]=source_order
        conversion=label;break
    coordinate_proven=permutation is not None
    original_area=_doubled_area(sv,st)
    source_zero=np.flatnonzero(original_area==0.)
    base=dict(schema='s50_loader_triangle_correspondence_v1',
        qualification_scope='LOADER_POINT_AND_TRIANGLE_CORRESPONDENCE_ONLY',
        source_vertex_count=len(sv),loaded_vertex_count=len(lv),
        source_triangle_count=len(st),loaded_triangle_count=len(lt),
        source_vertices_sha256=array_digest(sv),source_triangle_indices_sha256=array_digest(st),
        actual_loaded_vertices_sha256=array_digest(lv),actual_loaded_triangle_indices_sha256=array_digest(lt),
        coordinate_conversion=conversion,coordinate_conversion_proven=coordinate_proven,
        source_zero_area_triangle_indices=source_zero.tolist(),
        source_zero_area_triangle_count=len(source_zero),
        actual_loaded_zero_area_triangle_count=int(np.count_nonzero(_doubled_area(lv,lt)==0.)),
        mesh_source_modified=False,geometry_inflation_m=0.,collision_coverage_qualified=False,
        mechanical_limits_evaluated=False,task_acceptance_evaluated=False)
    if same_shape:
        base.update(maximum_source_to_loaded_vertex_distance_m=float(np.linalg.norm(sv-lv,axis=1).max()),
            maximum_source_to_loaded_coordinate_difference_m=float(np.max(np.abs(sv-lv))),
            float32_roundtrip_maximum_coordinate_difference_m=(float(np.max(np.abs(f32-lv)))
                if np.isfinite(f32).all() else None),
            unmapped_indexwise_differences_are_not_geometric_errors=True)
    if not coordinate_proven:
        base.update(passed=False,status='UNEXPLAINED_VERTEX_CONVERSION',removed_source_triangles=[],
            topology_correspondence_attempted=False)
        return base,dict(actual_loaded_vertices=lv,actual_loaded_triangles=lt)
    inverse=np.empty(len(sv),dtype=np.int64);inverse[permutation]=np.arange(len(lv))
    mapped_lt=permutation[lt]
    candidates=defaultdict(deque)
    for index,triangle in enumerate(st):candidates[tuple(map(int,triangle))].append(index)
    loaded_to_source=np.full(len(lt),-1,dtype=np.int64)
    source_to_loaded=np.full(len(st),-1,dtype=np.int64)
    rotations=np.full(len(lt),-1,dtype=np.int64)
    for index,triangle in enumerate(mapped_lt):
        corners=tuple(map(int,triangle))
        for rotation in range(3):
            bucket=candidates.get(corners[rotation:]+corners[:rotation])
            if bucket:
                source_index=bucket.popleft()
                loaded_to_source[index]=source_index;source_to_loaded[source_index]=index
                rotations[index]=rotation;break
    unmatched=np.flatnonzero(loaded_to_source<0)
    removed=np.flatnonzero(source_to_loaded<0)
    removed_rows=[]
    for index in removed:
        vertex_indices=st[index];points=sv[vertex_indices];actual_indices=inverse[vertex_indices]
        converted=lv[actual_indices]
        removed_rows.append(dict(source_triangle_index=int(index),
            source_vertex_indices=vertex_indices.tolist(),source_vertices_m=points.tolist(),
            corresponding_loaded_vertex_indices=actual_indices.tolist(),
            corresponding_loaded_vertices_m=converted.tolist(),
            source_doubled_area_m2=float(original_area[index]),
            converted_doubled_area_m2=float(_doubled_area(lv,actual_indices.reshape(1,3))[0]),
            source_exact_zero_area=bool(original_area[index]==0.)))
    aligned=np.take_along_axis(mapped_lt,(np.arange(3)[None,:]+rotations[:,None])%3,axis=1)
    matches=bool(not len(unmatched) and np.array_equal(st[loaded_to_source],aligned))
    only_zero=bool(all(row['source_exact_zero_area'] for row in removed_rows))
    exact_removed_set=bool(np.array_equal(removed,source_zero))
    passed=bool(matches and only_zero and exact_removed_set)
    base.update(passed=passed,status='EXACT_REMOVED_SOURCE_ZERO_AREA_CORRESPONDENCE' if passed else 'TRIANGLE_DIFFERENCE_RETAINED',
        topology_correspondence_attempted=True,
        source_vertex_index_equals_loaded_vertex_index=bool(np.array_equal(permutation,np.arange(len(lv)))),
        exact_unique_vertex_permutation_proven=True,
        moved_vertex_indices=int(np.count_nonzero(permutation!=np.arange(len(lv)))),
        cyclically_rotated_triangle_corner_count=int(np.count_nonzero(rotations>0)),
        source_to_loaded_maximum_matched_vertex_distance_m=float(np.linalg.norm(sv[permutation]-lv,axis=1).max()),
        retained_ordered_triangle_indices_match=matches,
        retained_source_triangle_order_preserved=bool(not len(unmatched) and np.all(np.diff(loaded_to_source)>0)),
        duplicate_source_triangle_count=int(len(st)-len(candidates)),
        duplicate_matching_rule='EXACT_VERTEX_PERMUTATION_AND_CYCLIC_CORNER_ROTATION_WITH_STABLE_DUPLICATE_OCCURRENCE_NO_WINDING_REVERSAL',
        unmatched_loaded_triangle_count=len(unmatched),unmatched_loaded_triangle_indices=unmatched.tolist(),
        removed_source_triangle_count=len(removed),removed_source_triangles=removed_rows,
        all_removed_source_triangles_exact_zero_area=only_zero,
        removed_source_index_set_equals_all_source_exact_zero_area_indices=exact_removed_set,
        mapping_arrays_sha256=dict(loaded_triangle_to_source_index=array_digest(loaded_to_source),
            source_triangle_to_loaded_index=array_digest(source_to_loaded),
            loaded_vertex_to_source_index=array_digest(permutation),
            source_vertex_to_loaded_index=array_digest(inverse),
            loaded_triangle_corner_rotation_to_source=array_digest(rotations)),
        float32_observation_scope='Exact equality over every loaded coordinate, not a presumed loader implementation')
    return base,dict(actual_loaded_vertices=lv,actual_loaded_triangles=lt,
        loaded_vertex_to_source_index=permutation,source_vertex_to_loaded_index=inverse,
        loaded_triangle_corner_rotation_to_source=rotations,
        loaded_triangle_to_source_index=loaded_to_source,source_triangle_to_loaded_index=source_to_loaded,
        removed_source_triangle_indices=removed)
