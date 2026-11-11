"""Surface defects and truthful optional Coal reporting, without a task bypass."""
from types import SimpleNamespace
from pathlib import Path
import hashlib
import json

import numpy as np
import pytest

from unloading_sim.s50_collision_diagnostics import (mesh_statistics,coal_pair_query,diagnostic_poses,SCOPE,
    diagnostic_environment,diagnostic_obstacle_record,diagnostic_solid_triangles,diagnostic_coal_obstacle)
from unloading_sim.s50_cad_model import read_mesh_bytes


def tetrahedron():
    return np.array([[0.,0,0],[1,0,0],[0,1,0],[0,0,1]]),np.array([[0,2,1],[0,1,3],[0,3,2],[1,2,3]])


def test_closed_tetrahedron_and_missing_face_are_distinguished():
    v,t=tetrahedron();complete=mesh_statistics(v,t);hole=mesh_statistics(v,t[:-1])
    assert complete['boundary_edge_count']==0
    assert complete['nonmanifold_edge_count']==0
    assert complete['same_direction_two_incident_edge_count']==0
    assert complete['oriented_surface_volume_m3']==pytest.approx(1/6)
    assert hole['boundary_edge_count']==3
    assert hole['solid_coverage_qualified'] is False


def test_duplicate_degenerate_and_nonmanifold_triangles_are_retained():
    v,t=tetrahedron();extra=np.concatenate((t,t[:1],[[0,0,1]]))
    original=extra.copy();result=mesh_statistics(v,extra)
    assert result['triangle_count']==6
    assert result['exact_zero_area_triangles']==1
    assert result['collapsed_triangles_after_analysis_weld']==1
    assert result['duplicate_triangles_ignoring_orientation_after_analysis_weld']==1
    assert result['nonmanifold_edge_count']==3
    np.testing.assert_array_equal(extra,original)


def test_orientation_error_is_separate_from_open_surface():
    v,t=tetrahedron();t[-1]=t[-1,::-1];result=mesh_statistics(v,t)
    assert result['boundary_edge_count']==0
    assert result['same_direction_two_incident_edge_count']==3
    assert result['solid_coverage_qualified'] is False


def test_weld_grid_is_analysis_only_and_not_silent_repair():
    v,t=tetrahedron();v=np.vstack((v,v[0]+[2e-10,0,0]));t[0,0]=4
    original=v.copy();coarse=mesh_statistics(v,t,weld_grid_m=1e-9);fine=mesh_statistics(v,t,weld_grid_m=1e-12)
    assert coarse['boundary_edge_count']==0
    assert fine['boundary_edge_count']>0
    assert coarse['original_mesh_changed'] is False
    np.testing.assert_array_equal(v,original)


@pytest.mark.parametrize('v,t',[(np.array([[np.nan,0,0]]),np.array([[0,0,0]])),
    (np.zeros((3,3)),np.array([[0,1,5]]))])
def test_invalid_surface_never_looks_clear(v,t):
    with pytest.raises(ValueError):mesh_statistics(v,t)


def fake_coal(*,intersection=False,distance=.2,points=None,collision_error=False):
    class CollisionResult:
        def isCollision(self):return intersection
        def numContacts(self):return 0
        def getContacts(self):return []
    class DistanceResult:
        min_distance=distance
        def getNearestPoint1(self):return (points or [[0.,0,0],[.2,0,0]])[0]
        def getNearestPoint2(self):return (points or [[0.,0,0],[.2,0,0]])[1]
    def collide(*args):
        if collision_error:raise RuntimeError('retained collision backend failure')
    return SimpleNamespace(CollisionRequest=SimpleNamespace,CollisionResult=CollisionResult,
        DistanceRequest=SimpleNamespace,DistanceResult=DistanceResult,collide=collide,distance=lambda *args:distance)


def test_empty_contacts_do_not_erase_real_collision_or_negative_raw_distance():
    result=coal_pair_query(fake_coal(intersection=True,distance=-1.),None,None,None,None)
    assert result['intersects'] is True and result['contacts']==[]
    assert result['surface_distance_m']==-1.
    assert result['unsigned_separation_m']==0.
    assert result['status']=='QUERIED'
    assert result['nearest_points_valid'] is False


def test_negative_distance_without_intersection_and_nan_nearest_points_are_unknown():
    result=coal_pair_query(fake_coal(distance=-1.,points=[[np.nan,0,0],[0,0,0]]),None,None,None,None)
    assert result['status']=='UNKNOWN'
    assert result['nearest_points_valid'] is False
    assert result['nearest_points_world'] is None


def test_missing_collision_query_cannot_be_replaced_by_distance_clearance():
    result=coal_pair_query(fake_coal(collision_error=True),None,None,None,None)
    assert result['status']=='UNKNOWN' and result['intersects'] is None
    assert result['surface_distance_m']==.2
    assert result['query_errors'][0]['query']=='collide'


def test_finite_but_inconsistent_nearest_points_cannot_be_presented_as_measured_line():
    result=coal_pair_query(fake_coal(distance=5.),None,None,None,None)
    assert result['status']=='QUERIED' and result['surface_distance_m']==5.
    assert result['nearest_points_valid'] is False


def test_diagnostic_population_is_finite_deterministic_and_not_a_path():
    poses=diagnostic_poses()
    assert poses==diagnostic_poses() and len(poses)==15
    assert poses[0]['q_rad']==[0.]*6
    assert len({p['pose_id'] for p in poses})==15
    assert all(len(p['q_rad'])==6 and np.isfinite(p['q_rad']).all() for p in poses)
    assert SCOPE=='CAD_DIAGNOSTIC_NOT_TASK'


def test_local_mount_artifact_is_bound_to_one_real_source_vertex_not_global_epsilon():
    root=Path(__file__).resolve().parents[1]
    data=json.loads((root/'assets/robots/huayan_s50_2000/generated/mount_contact_source.json').read_text())
    assert len(data['below_support_vertices'])==1
    row=data['below_support_vertices'][0]
    assert row['source_obj_vertex_index']==14085 and row['source_face_index']==137
    np.testing.assert_allclose(row['model_base_vertex_m'],[.14679515,-.05360862,-.00000054],atol=1e-14,rtol=0)
    assert row['source_obj_vertex_distance_to_whole_base_m']==pytest.approx(.00000054,abs=1e-14)
    closest=min(row['support_face_distances'],key=lambda value:value['distance_m'])
    assert closest['face_index']==19
    np.testing.assert_allclose(closest['nearest_point_CAD_m'],[.14679515,-.003,.05360862],atol=1e-14,rtol=0)
    assert data['local_tolerance_qualification_passed'] is False
    assert data['additional_penetration_allowance_m']==0.
    assert data['arbitrary_base_penetration_allowed'] is False
    assert data['whole_link_collision_exemption_allowed'] is False
    for prefix in ('source_base_mesh','model_base_mesh'):
        assert hashlib.sha256(read_mesh_bytes(root/data[prefix+'_path'])).hexdigest()==data[prefix+'_sha256']


def test_diagnostic_environment_contains_complete_frozen_148_entities_and_solid():
    from unloading_sim.robot_replacement_layout import frozen_baseline_layout,freeze_non_robot_scene
    root=Path(__file__).resolve().parents[1]
    layout=frozen_baseline_layout(root);state=diagnostic_environment(layout,.6)
    assert len(state.boxes)==148
    assert state.scene_fingerprint=='319e091c0924be0ec730724f123f2c2c8d08150ca2087652adcdf6b222f6a206'
    actual={b.name:diagnostic_obstacle_record(b) for b in state.boxes}
    frozen=freeze_non_robot_scene(layout)
    expected=next(s for s in frozen['height_samples'] if s['height_m']==.6)['entities']
    assert set(actual)=={row['id'] for row in expected}
    assert sum(row['category']=='carton' for row in actual.values())==40
    for row in expected:
        translated=dict(row);translated['name']=translated.pop('id')
        assert actual[row['id']]==translated
    solids=[box for box in state.boxes if getattr(box,'solid',None)]
    assert len(solids)==1 and solids[0].name=='conveyor_transverse'
    vertices,triangles=diagnostic_solid_triangles(solids[0])
    assert vertices.shape==(8,3) and triangles.shape==(12,3)
    np.testing.assert_allclose(vertices[0],[-.2738681118011935,-1.1,-.06],rtol=0,atol=1e-14)
    np.testing.assert_allclose(vertices[1],[-.35,-1.1,.06],rtol=0,atol=1e-14)
    assert vertices[0,0]>-solids[0].half_extents[0]  # real underside relief, not its outer box


def test_declared_convex_solid_is_sent_to_coal_without_box_fallback():
    from unloading_sim.geometry import OBB
    from unloading_sim.concept_solid import relieved_transverse
    source=relieved_transverse(OBB(np.zeros(3),[.35,1.1,.06],np.eye(3),'conveyor_transverse'),.5)
    calls=[]
    def convex(vertices,triangles):
        calls.append((np.array(vertices),np.array(triangles)));return 'real_convex'
    def forbidden_box(*args):raise AssertionError('solid was replaced by OBB')
    fake=SimpleNamespace(StdVec_Vec3s=list,StdVec_Triangle=list,Triangle=lambda *args:args,
        Convex=convex,Box=forbidden_box)
    geometry,evidence=diagnostic_coal_obstacle(fake,source)
    assert geometry=='real_convex' and evidence['is_bounding_proxy'] is False
    np.testing.assert_array_equal(calls[0][0],source.solid['vertices_m'])
    assert calls[0][1].shape==(12,3) and evidence['geometry']=='frozen_convex_solid'
    broken=SimpleNamespace(solid={'kind':'unrecognized'})
    with pytest.raises(ValueError,match='NO_OBB_FALLBACK'):diagnostic_coal_obstacle(fake,broken)
