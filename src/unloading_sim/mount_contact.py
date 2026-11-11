"""Source-bound fixed-base supporting-plane contact, never an arm exemption.

The original CAD's mounting plane and axis are separate from the URDF root.
Preview clocking is explicitly diagnostic. A geometric contact proof does not
make an unverified installation or controller mapping ready for task planning.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .robot_model_binding import rigid_transform

SCHEMA='source_bound_fixed_base_plane_v1'
SOURCE_PATH='assets/robots/huayan_s50_2000/generated/mount_contact_source.json'
NUMERIC_M=1e-10


def _sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _box(box):
    return dict(id=box.name,category=box.category,size=(2*box.half_extents).tolist(),
        pose=box.world_from_local.tolist(),**({'solid':box.solid} if getattr(box,'solid',None) else {}))


def _unit(value,name):
    vector=np.asarray(value,float)
    if vector.shape!=(3,) or not np.isfinite(vector).all() or abs(np.linalg.norm(vector)-1)>1e-8:
        raise ValueError('INVALID_MOUNT_UNIT_VECTOR:'+name)
    return vector/np.linalg.norm(vector)


def _transform_points(transform,vertices):
    return np.asarray(vertices) @ transform[:3,:3].T + transform[:3,3]


@dataclass
class MountContactBinding:
    contract: dict
    base_vertices: np.ndarray
    contact_vertices: np.ndarray
    contact_area_m2: float
    cached_key: str | None = None
    cached_evidence: dict | None = None
    geometry_provider: str = 'SOURCE_HASH_VERIFIED_OBJ'
    # This cache contains immutable geometry copied from the actual Coal model.
    backend_vertices: dict = field(default_factory=dict)
    backend_signatures: dict = field(default_factory=dict)


def _backend_geometry_signature(backend):
    records=[];base=backend.mount_contact_binding.contract['base_link']
    for index,obj in enumerate(backend.geometry_model.geometryObjects):
        if str(backend.model.frames[int(obj.parentFrame)].name)!=base:continue
        geometry=obj.geometry
        serializer=getattr(geometry,'saveToString',None)
        if callable(serializer):
            # Coal serializes the actual BVH mesh (vertices, triangle indices
            # and bounds) in C++; avoid ~92k Python triangle accessor calls.
            saved=serializer()
            if isinstance(saved,str):saved=saved.encode('utf-8')
            if not isinstance(saved,bytes) or not saved:raise ValueError('MOUNT_COAL_EMPTY_MESH_SERIALIZATION')
            digest=hashlib.sha256(saved).hexdigest()
            records.append((str(obj.name),digest,int(obj.parentFrame),int(obj.parentJoint),
                backend._matrix(obj.placement).tobytes(),id(backend._geometry_local_vertices[index])))
            continue
        getter=getattr(geometry,'vertices');vertices=np.asarray(getter() if callable(getter) else getter,dtype=np.float64)
        getter=getattr(geometry,'tri_indices')
        if callable(getter):
            try:raw=getter()
            except TypeError:raw=[getter(i) for i in range(int(geometry.num_tris))]
        else:raw=getter
        try:triangles=np.asarray(raw,dtype=np.int64)
        except (TypeError,ValueError):triangles=np.asarray([[int(t[i]) for i in range(3)] for t in raw],dtype=np.int64)
        if (vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all()
                or triangles.ndim!=2 or triangles.shape[1]!=3 or not len(triangles)
                or triangles.min()<0 or triangles.max()>=len(vertices)):
            raise ValueError('MOUNT_COAL_ACTUAL_MESH_STRUCTURE_UNAVAILABLE')
        # Boost can create a new Python wrapper on each geometry access. Hash
        # actual C++ mesh values/topology, never temporary wrapper addresses.
        digest=hashlib.sha256(vertices.tobytes()+triangles.tobytes()).hexdigest()
        records.append((str(obj.name),digest,int(obj.parentFrame),int(obj.parentJoint),
            backend._matrix(obj.placement).tobytes(),id(backend._geometry_local_vertices[index])))
    if not records:raise ValueError('MOUNT_COAL_ACTUAL_BASE_MESH_MISSING')
    return (getattr(backend,'geometry_revision',0),tuple(records))


def _base_is_fixed(urdf,base):
    by_child={j.find('child').get('link'):j for j in urdf.findall('joint')}
    seen=set();current=base
    while current in by_child:
        if current in seen:return False
        seen.add(current);joint=by_child[current]
        if joint.get('type')!='fixed':return False
        current=joint.find('parent').get('link')
    return True


def bind_mount_contact(robot,definition=None,root=None,*,world_from_physical_mount,
        expected_plinth,source_evidence_path=None):
    """Bind measured CAD support geometry to one immutable scene installation.

    Source tessellation deviations require individual, source-hashed evidence;
    no caller-supplied general penetration tolerance is accepted.
    """
    binding=robot.model_binding;binding.verify_source_unchanged()
    root=Path(root or Path(__file__).resolve().parents[2]).resolve()
    declared=binding.data.get('generated_definition')
    if not declared:raise ValueError('MOUNT_DEFINITION_SOURCE_UNDECLARED')
    path=(binding.config_path.parent/declared).resolve()
    actual=json.loads(path.read_text(encoding='utf-8'))
    if definition is not None:
        compare=deepcopy(definition);compare.pop('definition_sha256',None)
        if compare!=actual:raise ValueError('MOUNT_DEFINITION_SOURCE_CHANGED')
    definition=actual
    if (definition.get('schema') not in ('s50_cad_simulation_6r_v1','s50_task_model_v1')
            or definition['base_link']!=binding.base_link or definition['model_id']!=binding.model_id):
        raise ValueError('MOUNT_DEFINITION_MODEL_BINDING_MISMATCH')
    xml=ET.parse(binding.urdf_path).getroot()
    if not _base_is_fixed(xml,binding.base_link):raise ValueError('MOUNT_LINK_MUST_BE_FIXED_TO_WORLD')
    base_record=next(link for link in definition['links'] if link['name']==binding.base_link)
    frame=definition['frames']['mount'];plane=frame['support_plane_source']
    cad_from_base=rigid_transform(base_record['CAD_from_link_zero'],'CAD_from_base')
    cad_from_mount=rigid_transform(frame['CAD_from_frame_zero'],'CAD_from_mount')
    base_from_mount=np.linalg.inv(cad_from_base) @ cad_from_mount
    up=_unit(np.linalg.inv(cad_from_base)[:3,:3] @ np.asarray(plane['up_CAD'],float),'measured_plane_up')
    point=_transform_points(np.linalg.inv(cad_from_base),np.asarray([plane['axis_intersection_CAD_m']],float))[0]
    joint_origin=rigid_transform(definition['joints'][0]['origin'],'first_joint_origin')
    axis=_unit(joint_origin[:3,:3] @ np.asarray(definition['joints'][0]['axis'],float),'J1_axis_in_base')
    origin=joint_origin[:3,3]
    if (definition['joints'][0]['parent_link']!=binding.base_link
            or abs(abs(float(axis@up))-1)>1e-10 or np.linalg.norm(np.cross(origin-point,axis))>NUMERIC_M
            or np.max(np.abs(point-base_from_mount[:3,3]))>NUMERIC_M):
        raise ValueError('MEASURED_MOUNT_AXIS_OR_PLANE_INCONSISTENT')
    for face in plane['support_faces']:
        n=_unit(np.asarray(face['normal']),'CAD_support_normal')
        if (float(n@np.asarray(plane['up_CAD']))> -1+1e-10 or float(face['area_m2'])<=0
                or abs(float((np.asarray(face['surface_centroid_m'])-np.asarray(plane['axis_intersection_CAD_m']))@np.asarray(plane['up_CAD'])))>NUMERIC_M):
            raise ValueError('MEASURED_SUPPORT_FACE_INCONSISTENT')
    from .s50_cad_model import read_obj
    geometries=xml.find("link[@name='"+binding.base_link+"']").findall('collision')
    meshes=base_record['collision_meshes']
    if len(geometries)!=len(meshes) or not meshes:raise ValueError('MOUNT_BASE_COLLISION_SOURCE_COUNT_CHANGED')
    vertices=[];patches=[];area=0.;sources=[]
    from .geometry import rotation_matrix_from_rpy
    for geometry,record in zip(geometries,meshes):
        mesh=geometry.find('geometry/mesh')
        if mesh is None:raise ValueError('MOUNT_BASE_REQUIRES_ORIGINAL_MESH')
        actual_mesh=binding.resolve_mesh(mesh.get('filename'))
        if actual_mesh!=(root/record['path']).resolve() or _sha(actual_mesh)!=record['sha256']:
            raise ValueError('MOUNT_BASE_MESH_IDENTITY_MISMATCH')
        scale=np.asarray(list(map(float,mesh.get('scale','1 1 1').split())))
        transform=np.eye(4);node=geometry.find('origin')
        if node is not None:
            transform[:3,3]=list(map(float,node.get('xyz','0 0 0').split()))
            transform[:3,:3]=rotation_matrix_from_rpy(*map(float,node.get('rpy','0 0 0').split()))
        local,triangles=read_obj(actual_mesh,record['sha256'])
        local=_transform_points(transform,local*scale)
        tris=local[triangles];normal=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
        doubled=np.linalg.norm(normal,axis=1);unit=np.divide(normal,doubled[:,None],out=np.zeros_like(normal),where=doubled[:,None]>0)
        mask=(np.abs((tris-point)@up).max(axis=1)<=NUMERIC_M)&(unit@up < -1+1e-8)&(doubled>0)
        vertices.append(local);patches.append(tris[mask].reshape(-1,3));area+=float(doubled[mask].sum()/2)
        sources.append(dict(path=record['path'],sha256=record['sha256'],vertex_count=len(local),contact_triangle_count=int(mask.sum())))
    vertices=np.concatenate(vertices);patch=np.concatenate(patches)
    if area<=0 or len(patch)<3:raise ValueError('MOUNT_SUPPORT_PATCH_NOT_PRESENT_IN_ACTUAL_MESH')
    source_path=Path(source_evidence_path or root/SOURCE_PATH)
    source=json.loads(source_path.read_text(encoding='utf-8')) if source_path.is_file() else {}
    allowances=[];source_ready=False
    if source:
        if (source.get('source_step_sha256')!=definition['source_step_sha256']
                or source.get('model_base_mesh_sha256')!=meshes[0]['sha256']):
            raise ValueError('MOUNT_SOURCE_TOLERANCE_IDENTITY_MISMATCH')
        source_ready=(source.get('schema')=='s50_mount_contact_source_v1'
            and source.get('attribution_status')=='CAD_SOURCE_TESSELLATION_ARTIFACT_REPRODUCED_NOT_EDGE_TOLERANCE_PERMISSION'
            and source.get('all_below_plane_vertices_accounted_for') is True
            and source.get('source_file_modified') is False
            and source.get('additional_penetration_allowance_m')==0
            and source.get('arbitrary_base_penetration_allowed') is False
            and source.get('whole_link_collision_exemption_allowed') is False
            and abs(source.get('exact_base_lower_y_minus_support_m',float('inf')))<=1.01*NUMERIC_M)
        if source_ready:
            for row in source.get('below_support_vertices',[]):
                position=np.asarray(row['model_base_vertex_m'],float)
                # Permission is exactly the existing source vertex undershoot,
                # never a reported mesher deflection or global CAD tolerance.
                tolerance=-float((position-point)@up)
                nearest=min(row['support_face_distances'],key=lambda r:r['distance_m'])
                nearest_base=_transform_points(np.linalg.inv(cad_from_base),np.asarray([nearest['nearest_point_CAD_m']]))[0]
                if (position.shape!=(3,) or not np.isfinite(position).all() or not np.isfinite(tolerance)
                        or tolerance<=0 or nearest['face_index'] not in [f['face_index'] for f in plane['support_faces']]
                        or np.max(np.abs(nearest_base-(position+tolerance*up)))>NUMERIC_M
                        or abs(nearest['distance_m']-tolerance)>NUMERIC_M
                        or abs(row['source_only_allowance_m']-tolerance)>NUMERIC_M
                        or row['source_obj_rounding_error_m']>1e-8
                        or not np.any(np.max(np.abs(vertices-position),axis=1)<=NUMERIC_M)):
                    raise ValueError('MOUNT_SOURCE_POINT_ALLOWANCE_INVALID')
                allowances.append(dict(vertex_base_m=position.tolist(),maximum_source_plane_undershoot_m=tolerance,
                    source_vertex_index=row['model_base_vertex_index'],support_face_index=nearest['face_index'],
                    source_evidence='HASH_BOUND_REPRODUCED_TESSELLATION_VERTEX_WITH_EXACT_SOURCE_SURFACE_PROJECTION'))
    raw_min=float(np.min((vertices-point)@up))
    if raw_min>=-NUMERIC_M:source_ready=True
    install=binding.data.get('installation') or {}
    clocking=install.get('mount_clocking') or {}
    physical=install.get('physical_mount_frame') or {}
    formal=clocking.get('verified') is True and bool(clocking.get('source')) and physical.get('verified') is True and bool(physical.get('source'))
    contract=dict(schema=SCHEMA,model_id=binding.model_id,base_link=binding.base_link,support_object='mount_plinth',
        model_binding=binding.identity(),definition_sha256=_sha(path),source_step_sha256=definition['source_step_sha256'],
        expected_world_from_physical_mount=rigid_transform(world_from_physical_mount,'world_from_physical_mount').tolist(),
        base_from_measured_mount=base_from_mount.tolist(),plane_point_base_m=point.tolist(),plane_up_base=up.tolist(),
        first_joint_axis_base=axis.tolist(),expected_plinth=_box(expected_plinth),base_mesh_sources=sources,
        source_contact_area_m2=area,source_support_face_indices=[f['face_index'] for f in plane['support_faces']],
        source_geometry_verified=source_ready,source_vertex_allowances=allowances,
        source_tolerance_evidence_sha256=_sha(source_path) if source_path.is_file() else None,
        source_raw_minimum_plane_distance_m=raw_min,installation_verified=formal,
        installation_scope='VERIFIED_PHYSICAL_INSTALLATION' if formal else 'CAD_SIMULATION_CONVENTION_ONLY',
        geometry_permission_scope='FIXED_BASE_SUPPORTING_SURFACE_ONLY',numerical_tolerance_m=NUMERIC_M)
    for array in (vertices,patch):array.setflags(write=False)
    bound=MountContactBinding(contract,vertices,patch,area)
    robot.mount_contact_binding=bound
    return bound


def use_actual_mesh_backend(lightweight_robot,backend):
    """Use Coal's actual loaded base vertices for the CPU half-space proof."""
    bound=getattr(lightweight_robot,'mount_contact_binding',None)
    if bound is None:return
    if lightweight_robot.model_binding.identity()!=backend.model_binding.identity():
        raise ValueError('MOUNT_COAL_SOURCE_BINDING_MISMATCH')
    backend.mount_contact_binding=bound
    q=backend.model_binding.fk_probe_configurations()[0]
    backend._update_geometry(q)
    base=backend.named_link_frames(q)[bound.contract['base_link']]
    result=[]
    for index,obj in enumerate(backend.geometry_model.geometryObjects):
        if str(backend.model.frames[int(obj.parentFrame)].name)!=bound.contract['base_link']:continue
        vertices=backend._geometry_local_vertices[index]
        if vertices is None:raise ValueError('MOUNT_COAL_BASE_REQUIRES_ACTUAL_MESH_VERTICES')
        transform=np.linalg.inv(base) @ backend.base_transform @ backend._matrix(backend.geometry_data.oMg[index])
        result.append(_transform_points(transform,vertices))
    if not result:raise ValueError('MOUNT_COAL_BASE_GEOMETRY_MISSING')
    actual=np.concatenate(result);actual.setflags(write=False)
    bound.backend_vertices[id(backend)]=actual
    bound.backend_signatures[id(backend)]=_backend_geometry_signature(backend)
    lightweight_robot.mount_contact_mesh_backend=backend


def evaluate_mount_contact(contract,base_vertices,contact_vertices,contact_area_m2,world_from_base,support):
    """Numeric predicate shared with the native implementation, without masks."""
    base_vertices=np.asarray(base_vertices,float);contact_vertices=np.asarray(contact_vertices,float)
    if any(v.ndim!=2 or v.shape[1]!=3 or len(v)==0 or not np.isfinite(v).all()
            for v in (base_vertices,contact_vertices)):
        raise ValueError('MOUNT_MESH_VERTICES_MUST_BE_NONEMPTY_FINITE_N_BY_3')
    base=rigid_transform(world_from_base,'actual_world_from_base')
    expected=rigid_transform(contract['expected_world_from_physical_mount'],'expected_mount')
    mount=base @ rigid_transform(contract['base_from_measured_mount'],'base_from_mount')
    support_pose=rigid_transform(support.world_from_local,'support_pose')
    local_from_world=np.linalg.inv(support_pose)
    local_from_base=local_from_world @ base
    points=_transform_points(local_from_base,base_vertices)
    patch=_transform_points(local_from_base,contact_vertices)
    gap=points[:,2]-support.half_extents[2]
    limits=np.zeros(len(points));matched=[]
    for allowance in contract.get('source_vertex_allowances',[]):
        source=np.asarray(allowance['vertex_base_m'],float)
        depth=float(allowance['maximum_source_plane_undershoot_m'])
        if (source.shape!=(3,) or not np.isfinite(source).all() or not np.isfinite(depth) or depth<=0
                or abs(depth+float((source-np.asarray(contract['plane_point_base_m']))@np.asarray(contract['plane_up_base'])))>NUMERIC_M):
            raise ValueError('MOUNT_SOURCE_POINT_ALLOWANCE_INVALID')
        # Assimp may expose float32 vertices. This matches only this immutable
        # source point, and is not an engineering penetration allowance.
        match_error=4*np.finfo(np.float32).eps*np.maximum(np.abs(source),1e-6)+1e-12
        mask=np.all(np.abs(np.asarray(base_vertices)-source)<=match_error,axis=1)
        limits[mask]=depth
        if mask.any():matched.append(dict(source_vertex_base_m=source.tolist(),matched_vertices=int(mask.sum()),
            observed_minimum_support_distance_m=float(gap[mask].min()),source_allowance_m=float(limits[mask].max())))
    expected_box=contract['expected_plinth']
    normal=base[:3,:3] @ np.asarray(contract['plane_up_base'])
    measured=_transform_points(local_from_base,np.asarray([contract['plane_point_base_m']]))[0]
    checks=dict(fixed_base_role=contract['base_link']==contract['model_binding']['base_link'],
        source_geometry_verified=contract.get('source_geometry_verified') is True,
        named_original_support=support.name==contract['support_object']=='mount_plinth' and support.category=='mount_structure' and not getattr(support,'solid',None),
        support_geometry_unchanged=np.allclose(support.world_from_local,expected_box['pose'],atol=NUMERIC_M,rtol=0)
            and np.allclose(2*support.half_extents,expected_box['size'],atol=NUMERIC_M,rtol=0),
        mounting_axis_and_frame_unchanged=np.allclose(mount,expected,atol=NUMERIC_M,rtol=0),
        opposed_support_normals=bool(float(normal@support_pose[:3,2])>=1-1e-10),
        measured_plane_is_support_top=abs(float(measured[2]-support.half_extents[2]))<=NUMERIC_M,
        full_contact_region_supported=len(patch)>=3 and bool(np.all(np.abs(patch[:,:2])<=support.half_extents[:2]+NUMERIC_M))
            and bool(np.all(np.abs(patch[:,2]-support.half_extents[2])<=NUMERIC_M)),
        positive_source_contact_area=bool(np.isfinite(contact_area_m2) and contact_area_m2>0),
        no_unexplained_base_penetration=bool(np.all(gap>=-limits-NUMERIC_M)))
    checks={name:bool(value) for name,value in checks.items()};passed=all(checks.values())
    return dict(schema=SCHEMA,status='QUALIFIED_FIXED_BASE_CONTACT' if passed else 'MOUNT_CONTACT_NOT_QUALIFIED',
        passed=passed,checks=checks,failures=[name for name,value in checks.items() if not value],
        allowed_pairs=[[contract['base_link'],contract['support_object']]] if passed else [],
        actual_world_from_base=base.tolist(),actual_world_from_measured_mount=mount.tolist(),
        minimum_raw_mesh_support_distance_m=float(gap.min()),matched_source_deviations=matched,
        contact_area_m2=float(contact_area_m2),contact_region_support_local_bounds_m=[patch.min(axis=0).tolist(),patch.max(axis=0).tolist()],
        installation_verified=contract['installation_verified'],installation_scope=contract['installation_scope'],
        formal_task_readiness_not_implied=True,active_links_exempted=[])


def mount_contact_evidence(robot,obstacles):
    bound=getattr(robot,'mount_contact_binding',None)
    if bound is None:return dict(status='MOUNT_CONTACT_MODEL_NOT_BOUND',passed=False,allowed_pairs=[],active_links_exempted=[])
    support=[b for b in obstacles if b.name==bound.contract['support_object']]
    if len(support)!=1:return dict(status='MOUNT_SUPPORT_MISSING_OR_DUPLICATED',passed=False,allowed_pairs=[],active_links_exempted=[])
    backend=getattr(robot,'mount_contact_mesh_backend',robot)
    if id(backend) in bound.backend_signatures:
        try:unchanged=bound.backend_signatures[id(backend)]==_backend_geometry_signature(backend)
        except (AttributeError,TypeError,ValueError,RuntimeError):unchanged=False
        if not unchanged:return dict(status='MOUNT_COLLISION_GEOMETRY_CHANGED',passed=False,allowed_pairs=[],active_links_exempted=[])
    if not np.allclose(robot.base_transform,backend.base_transform,atol=NUMERIC_M,rtol=0):
        raise ValueError('MOUNT_CORE_COAL_BASE_TRANSFORM_MISMATCH')
    vertices=bound.backend_vertices.get(id(backend),bound.base_vertices)
    key=json.dumps(dict(base=np.asarray(robot.base_transform).tolist(),support=_box(support[0]),
        geometry=id(vertices),contract=bound.contract),sort_keys=True)
    if bound.cached_key==key:return deepcopy(bound.cached_evidence)
    result=evaluate_mount_contact(bound.contract,vertices,bound.contact_vertices,bound.contact_area_m2,
        robot.base_transform,support[0])
    result['geometry_provider']='SAME_COAL_COLLISION_MESH_VERTICES' if id(backend) in bound.backend_vertices else bound.geometry_provider
    bound.cached_key=key;bound.cached_evidence=deepcopy(result)
    return result


def mount_contact_contract(robot):
    bound=getattr(robot,'mount_contact_binding',None)
    return None if bound is None else deepcopy(bound.contract)
