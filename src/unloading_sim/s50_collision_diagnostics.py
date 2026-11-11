"""Source-preserving mesh and optional Coal diagnostics, never task acceptance.

No collision pair is disabled here. CAD display configurations are not mechanical
limits, legal homes, or robot trajectories. Heavy dependencies remain optional.
"""
from __future__ import annotations

from itertools import combinations
from pathlib import Path
import hashlib
import json

import numpy as np

from .s50_cad_model import read_obj, load_definition, transform_vertices


SCOPE = 'CAD_DIAGNOSTIC_NOT_TASK'


def diagnostic_environment(layout, conveyor_height_m):
    """Use the same coupled assembly as frozen planning/viewer snapshots."""
    from .conveyor_lift import coupled_conveyor_state
    cartons=tuple(layout.cartons())
    if len(cartons)!=40 or len({box.name for box in cartons})!=40:
        raise ValueError('DIAGNOSTIC_REQUIRES_FRESH40')
    state=coupled_conveyor_state(layout,float(conveyor_height_m),cartons)
    if len({box.name for box in state.boxes})!=len(state.boxes):
        raise ValueError('DIAGNOSTIC_DUPLICATE_ENVIRONMENT_ID')
    return state


def diagnostic_obstacle_record(box):
    """Retain the actual solid as well as its outward broad-phase box."""
    from copy import deepcopy
    return dict(name=box.name,category=box.category,size_m=(2*box.half_extents).tolist(),
        world_from_entity=box.world_from_local.tolist(),solid=deepcopy(getattr(box,'solid',None)))


def diagnostic_solid_triangles(box):
    """Triangulate the declared convex faces without replacing them by an OBB."""
    from .concept_solid import ConceptSolid
    solid=getattr(box,'solid',None)
    if not solid:return None
    if solid.get('kind')!='convex_concept_solid_v1':
        raise ValueError('DIAGNOSTIC_UNSUPPORTED_SOLID_NO_OBB_FALLBACK')
    # Re-run the source solid's finite, planar, convex and outward-cover checks.
    ConceptSolid(box.center,box.half_extents,box.rotation,box.name,box.category,solid)
    vertices=np.asarray(solid['vertices_m'],dtype=float).copy()
    triangles=np.asarray([[face[0],face[i],face[i+1]] for face in solid['faces']
        for i in range(1,len(face)-1)],dtype=np.int64)
    return vertices,triangles


def diagnostic_coal_obstacle(coal,box):
    """Exact declared box/convex solid; unsupported geometry fails closed."""
    surface=diagnostic_solid_triangles(box)
    if surface is None:
        return coal.Box(*map(float,2*box.half_extents)),dict(geometry='frozen_obb',
            representation='DECLARED_OBB_SOLID',is_bounding_proxy=False)
    vertices,triangles=surface
    points=coal.StdVec_Vec3s();faces=coal.StdVec_Triangle()
    for vertex in vertices:points.append(vertex)
    for triangle in triangles:faces.append(coal.Triangle(*map(int,triangle)))
    shape=coal.Convex(points,faces)
    digest=hashlib.sha256(json.dumps(box.solid,sort_keys=True,separators=(',',':'),
        allow_nan=False).encode('utf8')).hexdigest()
    return shape,dict(geometry='frozen_convex_solid',representation='DECLARED_CONVEX_SOLID_FACE_TRIANGULATION',
        is_bounding_proxy=False,solid_sha256=digest,vertex_count=len(vertices),triangle_count=len(triangles))


def mesh_statistics(vertices, triangles, *, weld_grid_m=1e-9, sample_limit=12):
    """Describe an unmodified triangle surface at an explicit quantization grid.

    Welding is analysis only. A grid is not a distance-based weld certificate,
    nor does a closed-edge count prove an embedded, non-self-intersecting solid.
    """
    v=np.asarray(vertices,dtype=float);t=np.asarray(triangles,dtype=np.int64)
    if (v.ndim!=2 or v.shape[1]!=3 or not len(v) or not np.isfinite(v).all()
            or t.ndim!=2 or t.shape[1]!=3 or not len(t) or t.min()<0 or t.max()>=len(v)):
        raise ValueError('INVALID_TRIANGLE_SURFACE')
    if not np.isfinite(weld_grid_m) or weld_grid_m<=0:
        raise ValueError('INVALID_ANALYSIS_WELD_GRID')
    twice_area=np.linalg.norm(np.cross(v[t[:,1]]-v[t[:,0]],v[t[:,2]]-v[t[:,0]]),axis=1)
    _, representative, inverse=np.unique(np.rint(v/weld_grid_m).astype(np.int64),
        axis=0,return_index=True,return_inverse=True)
    wt=inverse[t]
    repeated=(wt[:,0]==wt[:,1])|(wt[:,1]==wt[:,2])|(wt[:,2]==wt[:,0])
    valid=wt[~repeated]
    canonical=np.sort(valid,axis=1)
    _, counts=np.unique(canonical,axis=0,return_counts=True)
    edges=np.concatenate((valid[:,[0,1]],valid[:,[1,2]],valid[:,[2,0]]))
    unique_edges,edge_inverse,edge_counts=np.unique(np.sort(edges,axis=1),axis=0,
        return_inverse=True,return_counts=True)
    directions=np.where(edges[:,0]<edges[:,1],1,-1)
    signs=np.bincount(edge_inverse,weights=directions,minlength=len(unique_edges))
    boundary=unique_edges[edge_counts==1]
    nonmanifold=unique_edges[edge_counts>2]
    signed_volume=np.einsum('ij,ij->i',v[t[:,0]],np.cross(v[t[:,1]],v[t[:,2]])).sum()/6.
    return dict(vertex_count=len(v),triangle_count=len(t),bounds_m=[v.min(0).tolist(),v.max(0).tolist()],
        exact_zero_area_triangles=int(np.count_nonzero(twice_area==0)),
        area_below_1e_20_m2_triangles=int(np.count_nonzero(twice_area<=2e-20)),
        minimum_triangle_area_m2=float(twice_area.min()/2),surface_area_m2=float(twice_area.sum()/2),
        oriented_surface_volume_m3=float(signed_volume),volume_is_not_solid_validity_proof=True,
        unused_vertex_count=int(len(v)-len(np.unique(t))),analysis_weld_grid_m=weld_grid_m,
        analysis_weld_method='ROUND_TO_GRID_NOT_NEAREST_NEIGHBOR_CERTIFICATE',
        welded_vertex_count=len(representative),collapsed_triangles_after_analysis_weld=int(repeated.sum()),
        duplicate_triangles_ignoring_orientation_after_analysis_weld=int(np.maximum(counts-1,0).sum()),
        boundary_edge_count=len(boundary),nonmanifold_edge_count=len(nonmanifold),
        same_direction_two_incident_edge_count=int(np.count_nonzero((edge_counts==2)&(np.abs(signs)==2))),
        boundary_edge_samples_m=v[representative[boundary[:sample_limit]]].tolist(),
        nonmanifold_edge_samples_m=v[representative[nonmanifold[:sample_limit]]].tolist(),
        boundary_count_is_not_geometric_hole_area=True,t_junction_resolution_performed=False,
        mesh_self_intersection_tested=False,inside_outside_solid_winding_tested=False,
        original_mesh_changed=False,solid_coverage_qualified=False)


def audit_model_meshes(root):
    root=Path(root);definition=load_definition(root)
    base=root/'assets/robots/huayan_s50_2000'
    historical=json.loads((base/'derived/cad_audit.json').read_text())
    repair=json.loads((base/'collision_repair/coverage_report.json').read_text())
    repairs={(p['cad_body'],p['source_face_index']):p for p in repair['patches']}
    missing=[(p['id'],f['face_index']) for p in historical['parts'] for f in p['missing_tessellation_faces']]
    rows=[]
    for link in definition['links']:
        meshes=[];vv=[];tt=[];offset=0
        for mesh in link['collision_meshes']:
            v,t=read_obj(root/mesh['path'],mesh['sha256'])
            source_v,source_t=read_obj(root/mesh['source_CAD_mesh_path'],mesh['source_CAD_mesh_sha256'])
            restored=transform_vertices(link['CAD_from_link_zero'],v)
            if not np.array_equal(t,source_t):raise ValueError('COLLISION_SOURCE_TRIANGLES_CHANGED')
            delta=float(np.max(np.abs(restored-source_v)))
            if delta>2e-12:raise ValueError('COLLISION_SOURCE_VERTEX_GEOMETRY_CHANGED')
            meshes.append(dict(path=mesh['path'],sha256=mesh['sha256'],
                source_path=mesh['source_CAD_mesh_path'],source_sha256=mesh['source_CAD_mesh_sha256'],
                source_coordinate_restoration_error_m=delta,
                role=mesh.get('role','ORIGINAL_CAD_SURFACE'),statistics=mesh_statistics(v,t)))
            vv.append(v);tt.append(t+offset);offset+=len(v)
        original=next(p for p in historical['parts'] if p['id']==link['cad_body'])
        merged_v=np.concatenate(vv);merged_t=np.concatenate(tt)
        rows.append(dict(link=link['name'],cad_body=link['cad_body'],meshes=meshes,
            combined_surface=mesh_statistics(merged_v,merged_t),
            historical_brep_valid=original['topology_valid'],
            historical_missing_face_indices=[f['face_index'] for f in original['missing_tessellation_faces']],
            brep_validity_is_not_runtime_query_gate=True))
    return dict(schema='s50_source_triangle_surface_audit_v1',qualification_scope=SCOPE,
        source_step_sha256=definition['source_step_sha256'],model_definition_sha256=definition['definition_sha256'],
        historical_cad_audit_sha256=hashlib.sha256((base/'derived/cad_audit.json').read_bytes()).hexdigest(),
        repair_report_sha256=hashlib.sha256((base/'collision_repair/coverage_report.json').read_bytes()).hexdigest(),
        missing_source_faces=[dict(cad_body=a,face_index=b,patch_present=(a,b) in repairs) for a,b in missing],
        every_previously_unmeshed_face_has_patch=all(pair in repairs for pair in missing),
        missing_face_count=len(missing),original_collision_mesh_count=7,additive_patch_count=len(repairs),
        coverage_scope='SOURCE_FACE_ACCOUNTING_AND_TRIANGLE_TOPOLOGY_NOT_FULL_SOLID_COLLISION_CERTIFICATE',
        collision_acceptance_qualified=False,geometry_inflation_m=0.,links=rows)


def diagnostic_poses(*,include_mount_probes=False):
    poses=[dict(pose_id='cad_q0',label='CAD 装配零位',description='CAD reference, not legal working home',q_rad=[0.]*6)]
    for index in range(6):
        for sign in (-1,1):
            q=np.zeros(6);q[index]=sign*.35
            poses.append(dict(pose_id=f'axis_{index+1}_{"negative" if sign<0 else "positive"}',
                label=f'J{index+1} {sign*.35:+.2f} rad',description='One measured CAD joint; display range only',q_rad=q.tolist()))
    for name,q in [('coupled_a',[0.,.7,-1.1,.25,.4,-.3]),('coupled_b',[.4,-.55,.85,-.35,-.6,.2])]:
        poses.append(dict(pose_id=name,label=name,description='Deterministic mixed CAD articulation, not a motion plan',q_rad=q))
    if include_mount_probes:
        for angle in (2.,2.4,2.8,3.):
            poses.append(dict(pose_id='mount_probe_j2_'+str(angle).replace('.','p'),label=f'J2 mount diagnostic {angle} rad',
                description='Deliberate finite CAD fold diagnostic for active-link/plinth pairs; no mechanical-limit claim',
                q_rad=[0.,angle,0.,0.,0.,0.]))
    return poses


def _finite(value):
    value=float(value)
    return value if np.isfinite(value) else None


def _point(value):
    try:
        value=np.asarray(value,dtype=float)
        return value.tolist() if value.shape==(3,) and np.isfinite(value).all() else None
    except (TypeError,ValueError):return None


def coal_pair_query(coal, first, first_transform, second, second_transform, *, contact_limit=8):
    """Retain actual collide and distance results independently, even on errors."""
    row=dict(status='UNKNOWN',intersects=None,surface_distance_m=None,
        unsigned_separation_m=None,nearest_points_world=None,nearest_points_valid=False,contacts=[])
    errors=[]
    try:
        request=coal.CollisionRequest()
        request.enable_contact=True;request.num_max_contacts=contact_limit
        result=coal.CollisionResult()
        coal.collide(first,first_transform,second,second_transform,request,result)
        row['intersects']=bool(result.isCollision())
        row['reported_contact_count']=int(result.numContacts())
        for contact in list(result.getContacts())[:contact_limit]:
            row['contacts'].append(dict(position_world_m=_point(contact.pos),normal_world=_point(contact.normal),
                reported_penetration_depth_m=_finite(contact.penetration_depth)))
        row['contact_limit']=contact_limit
        row['contacts_truncated']=row['reported_contact_count']>=contact_limit
    except Exception as exc:errors.append(dict(query='collide',type=type(exc).__name__,detail=str(exc)))
    try:
        request=coal.DistanceRequest();request.enable_nearest_points=True
        result=coal.DistanceResult()
        value=coal.distance(first,first_transform,second,second_transform,request,result)
        raw=_finite(getattr(result,'min_distance',value));row['surface_distance_m']=raw
        row['distance_semantics']='RAW_COAL_MIN_DISTANCE_NEGATIVE_MAY_BE_INTERSECTION_SENTINEL_NOT_PENETRATION_DEPTH'
        points=[_point(result.getNearestPoint1()),_point(result.getNearestPoint2())]
        if all(p is not None for p in points):
            row['nearest_points_world']=points
            row['nearest_point_segment_length_m']=float(np.linalg.norm(np.asarray(points[0])-points[1]))
            # Intersecting BVH queries can return a negative sentinel and
            # finite default nearest points. Retain them but never draw them
            # as demonstrated contact positions; actual collide contacts are
            # independently recorded above.
            row['nearest_points_valid']=bool(row['intersects'] is False and raw is not None and raw>=0
                and abs(row['nearest_point_segment_length_m']-raw)<=max(1e-8,abs(raw)*1e-6))
            row['nearest_points_validation']='FINITE_SEPARATED_PAIR_AND_SEGMENT_LENGTH_MATCHES_REPORTED_DISTANCE'
        if row['intersects'] is True:
            row['unsigned_separation_m']=0.
        elif row['intersects'] is False and raw is not None and raw>=0:
            row['unsigned_separation_m']=raw
    except Exception as exc:errors.append(dict(query='distance',type=type(exc).__name__,detail=str(exc)))
    if row['intersects'] is not None and row['unsigned_separation_m'] is not None:
        row['status']='QUERIED'
    if errors:row['query_errors']=errors
    return row


def _world_geometry(backend,q):
    backend._update_geometry(np.asarray(q,dtype=float))
    result=[]
    for index,obj in enumerate(backend.geometry_model.geometryObjects):
        frame=int(obj.parentFrame);name=str(backend.model.frames[frame].name)
        world=backend.base_transform@backend._matrix(backend.geometry_data.oMg[index])
        result.append(dict(index=index,link=name,geometry=str(obj.name),shape=obj.geometry,world=world,
            vertices=backend._geometry_local_vertices[index]))
    return result


def _envelopes(geometry,world_from_mount):
    from .s50_compactness import group_envelopes
    return group_envelopes(geometry,world_from_mount)


def diagnose_pose(backend,q,obstacles,world_from_sim_mount,*,pose_id,contact_limit=8,conveyor_height_m=None):
    """Query every geometry pair without SRDF/adjacency/mount exemptions."""
    from .pinocchio_backend import _transform
    q=np.asarray(q,dtype=float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('DIAGNOSTIC_Q_REQUIRES_SIX_FINITE_VALUES')
    geometry=_world_geometry(backend,q);coal=backend.coal;pairs=[]
    def tf(g):return _transform(coal,g['world'][:3,:3],g['world'][:3,3])
    adjacent={frozenset((f's50_link_{i}',f's50_link_{i+1}')) for i in range(1,6)}
    adjacent.add(frozenset(('s50_base','s50_link_1')))
    for first,second in combinations(geometry,2):
        relation=('SAME_RIGID_LINK' if first['link']==second['link'] else
            'ADJACENT_INTERFACE_NOT_EXEMPTED_OR_REVIEWED' if frozenset((first['link'],second['link'])) in adjacent else 'NONADJACENT_ROBOT')
        row=coal_pair_query(coal,first['shape'],tf(first),second['shape'],tf(second),contact_limit=contact_limit)
        pairs.append(dict(kind='robot_self',relation=relation,
            first={k:first[k] for k in ('link','geometry')},second={k:second[k] for k in ('link','geometry')},**row))
    for obstacle in obstacles:
        shape,shape_evidence=diagnostic_coal_obstacle(coal,obstacle)
        ot=_transform(coal,obstacle.rotation,obstacle.center)
        for first in geometry:
            row=coal_pair_query(coal,first['shape'],tf(first),shape,ot,contact_limit=contact_limit)
            pairs.append(dict(kind='robot_environment',relation='UNFILTERED_DIAGNOSTIC_PAIR',
                first={k:first[k] for k in ('link','geometry')},
                second=dict(object=obstacle.name,category=obstacle.category,**shape_evidence),**row))
    def minimum(selected):
        known=[p for p in selected if p['unsigned_separation_m'] is not None]
        if not known:return None
        p=min(known,key=lambda p:p['unsigned_separation_m'])
        return {k:p[k] for k in ('first','second','intersects','surface_distance_m','unsigned_separation_m','nearest_points_world','nearest_points_valid')}
    env=[p for p in pairs if p['kind']=='robot_environment']
    belt=[p for p in env if p['second']['object']=='conveyor_transverse']
    frames=backend.named_link_frames(q)
    physical_names=['s50_base',*[f's50_link_{i}' for i in range(1,7)],'s50_flange']
    from .s50_compactness import compactness_sample
    compactness = compactness_sample(geometry,pairs,world_from_sim_mount,q_rad=q,pose_id=pose_id,
        conveyor_height_m=conveyor_height_m,safety_evidence=dict(scope=SCOPE,
            queried_pair_count=len(pairs),unknown_pair_count=sum(p['status']!='QUERIED' for p in pairs),
            task_acceptance_evaluated=False)) if belt else None
    return dict(pose_id=pose_id,q_rad=q.tolist(),qualification_scope=SCOPE,conveyor_height_m=conveyor_height_m,
        link_frames_world={name:np.asarray(frames[name]).tolist() for name in physical_names},
        pairs=pairs,queried_pair_count=len(pairs),unknown_pair_count=sum(p['status']!='QUERIED' for p in pairs),
        collision_pairs=[{k:p[k] for k in ('kind','relation','first','second','surface_distance_m','nearest_points_world','nearest_points_valid','contacts')}
            for p in pairs if p['intersects'] is True],
        collision_count_is_not_task_failure=True,mechanical_joint_limits_checked=False,
        minimum_distances=dict(all_environment=minimum(env),transverse_conveyor=minimum(belt),
            by_robot_link_transverse={g['link']:minimum([p for p in belt if p['first']['link']==g['link']]) for g in geometry}),
        elbow_wrist_envelope=_envelopes(geometry,world_from_sim_mount),
        compactness=compactness,
        pair_filtering='NONE_ALL_SHAPE_PAIRS_INCLUDING_OWNED_PATCHES_AND_ADJACENT_JOINTS',
        tool_installed=False,task_acceptance_evaluated=False)
