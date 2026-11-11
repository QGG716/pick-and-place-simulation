from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import json
import numpy as np
import pytest
from types import SimpleNamespace

from unloading_sim.s50_cad_model import load_preview_robot
from unloading_sim.robot_replacement_layout import frozen_baseline_layout
from unloading_sim.mount_contact import bind_mount_contact,mount_contact_evidence,evaluate_mount_contact
from unloading_sim.joint_search_layout import qualified_mount_pairs

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture(scope='module')
def installed():
    layout=frozen_baseline_layout(ROOT)
    robot=load_preview_robot(ROOT,world_from_mount=layout.robot_base_transform())
    parts=layout.fixed_components();plinth=next(p for p in parts if p.name=='mount_plinth')
    bound=bind_mount_contact(robot,root=ROOT,world_from_physical_mount=layout.robot_base_transform(),expected_plinth=plinth)
    return robot,parts,plinth,bound

def test_actual_source_contact_is_local_and_not_installation_qualification(installed):
    robot,parts,_,bound=installed
    evidence=mount_contact_evidence(robot,parts)
    assert evidence['passed'],evidence
    assert evidence['minimum_raw_mesh_support_distance_m']==pytest.approx(-.54e-6,abs=1e-12)
    assert .0152<evidence['contact_area_m2']<.0153
    assert evidence['installation_verified'] is False
    assert evidence['installation_scope']=='CAD_SIMULATION_CONVENTION_ONLY'
    assert evidence['active_links_exempted']==[]
    assert qualified_mount_pairs(robot,parts)=={('s50_base','mount_plinth')}
    assert len(bound.contract['source_vertex_allowances'])==1

@pytest.mark.parametrize('delta',[[-.001,0,0],[0,0,-.001],[0,0,-1e-7],[0,0,.001]])
def test_real_mount_movement_never_uses_source_mesh_allowance(installed,delta):
    robot,_,plinth,bound=installed
    shifted=robot.base_transform.copy();shifted[:3,3]+=delta
    result=evaluate_mount_contact(bound.contract,bound.base_vertices,bound.contact_vertices,bound.contact_area_m2,shifted,plinth)
    assert not result['passed']
    assert not result['checks']['mounting_axis_and_frame_unchanged']
    assert not result['allowed_pairs']

def test_new_unexplained_mesh_penetration_fails(installed):
    robot,_,plinth,bound=installed
    vertices=np.vstack([bound.base_vertices,[0,0,-1e-6]])
    result=evaluate_mount_contact(bound.contract,vertices,bound.contact_vertices,bound.contact_area_m2,robot.base_transform,plinth)
    assert not result['checks']['no_unexplained_base_penetration']
    assert not result['passed']

def test_support_change_duplicate_and_missing_fail_closed(installed):
    robot,parts,plinth,bound=installed
    smaller=replace(plinth,half_extents=np.array([.02,.02,plinth.half_extents[2]]))
    result=evaluate_mount_contact(bound.contract,bound.base_vertices,bound.contact_vertices,bound.contact_area_m2,robot.base_transform,smaller)
    assert not result['checks']['full_contact_region_supported']
    assert not mount_contact_evidence(robot,[*parts,plinth])['passed']
    assert not mount_contact_evidence(robot,[])['passed']

def test_missing_source_artifact_does_not_qualify_raw_overlap(installed,tmp_path):
    robot,parts,plinth,_=installed
    separate=load_preview_robot(ROOT,world_from_mount=robot.base_transform)
    bind_mount_contact(separate,root=ROOT,world_from_physical_mount=robot.base_transform,
        expected_plinth=plinth,source_evidence_path=tmp_path/'absent.json')
    assert qualified_mount_pairs(separate,parts)==set()

def test_source_allowance_cannot_be_increased(installed,tmp_path):
    robot,_,plinth,_=installed
    source=json.loads((ROOT/'assets/robots/huayan_s50_2000/generated/mount_contact_source.json').read_text())
    source['below_support_vertices'][0]['source_only_allowance_m']=.001
    path=tmp_path/'changed.json';path.write_text(json.dumps(source))
    with pytest.raises(ValueError,match='SOURCE_POINT_ALLOWANCE_INVALID'):
        bind_mount_contact(robot,root=ROOT,world_from_physical_mount=robot.base_transform,expected_plinth=plinth,source_evidence_path=path)

@pytest.mark.parametrize('value',[np.nan,np.inf,-np.inf])
def test_nonfinite_vertices_fail_closed(installed,value):
    robot,_,plinth,bound=installed
    vertices=bound.base_vertices.copy();vertices[0,2]=value
    with pytest.raises(ValueError,match='NONEMPTY_FINITE'):
        evaluate_mount_contact(bound.contract,vertices,bound.contact_vertices,bound.contact_area_m2,robot.base_transform,plinth)

@pytest.mark.parametrize('mutation',['revision','shape','placement','topology','vertices'])
@pytest.mark.parametrize('serializer',[False,True])
def test_actual_geometry_change_invalidates_permission(installed,mutation,serializer):
    from unloading_sim.mount_contact import _backend_geometry_signature
    robot,_,plinth,bound=installed
    local=deepcopy(bound)
    actual=SimpleNamespace(vertices=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]),tri_indices=np.array([[0,1,2]]))
    if serializer:actual.saveToString=lambda:json.dumps(dict(vertices=actual.vertices.tolist(),triangles=actual.tri_indices.tolist()))
    geometry=SimpleNamespace(name='base',geometry=actual,parentFrame=0,parentJoint=0,placement=np.eye(4))
    backend=SimpleNamespace(base_transform=robot.base_transform.copy(),mount_contact_binding=local,
        geometry_revision=0,geometry_model=SimpleNamespace(geometryObjects=[geometry]),
        model=SimpleNamespace(frames=[SimpleNamespace(name='s50_base')]),
        _matrix=lambda x:x,_geometry_local_vertices=[bound.base_vertices])
    local.backend_vertices[id(backend)]=bound.base_vertices
    local.backend_signatures[id(backend)]=_backend_geometry_signature(backend)
    assert mount_contact_evidence(backend,[plinth])['passed']
    if mutation=='revision':backend.geometry_revision+=1
    if mutation=='shape':geometry.geometry=object()
    if mutation=='placement':geometry.placement[2,3]=-.01
    if mutation=='topology':actual.tri_indices[0]=[1,0,2]
    if mutation=='vertices':actual.vertices[0,2]=-.01
    result=mount_contact_evidence(backend,[plinth])
    assert result['status']=='MOUNT_COLLISION_GEOMETRY_CHANGED'
    assert result['allowed_pairs']==[]

def test_temporary_mesh_wrapper_is_not_a_geometry_change(installed):
    from unloading_sim.mount_contact import _backend_geometry_signature
    robot,_,_,bound=installed
    class TemporaryWrapper:
        name='base';parentFrame=0;parentJoint=0;placement=np.eye(4)
        @property
        def geometry(self):
            return SimpleNamespace(vertices=lambda:np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]),
                tri_indices=lambda:np.array([[0,1,2]]),saveToString=lambda:'immutable fixture mesh v1')
    backend=SimpleNamespace(mount_contact_binding=bound,model=SimpleNamespace(frames=[SimpleNamespace(name='s50_base')]),
        geometry_model=SimpleNamespace(geometryObjects=[TemporaryWrapper()]),_matrix=lambda x:x,
        _geometry_local_vertices=[bound.base_vertices])
    assert _backend_geometry_signature(backend)==_backend_geometry_signature(backend)
