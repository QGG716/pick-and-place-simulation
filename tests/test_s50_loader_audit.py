"""Index-level loss accounting must reject unexplained geometry differences."""
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.s50_loader_audit import audit_triangle_correspondence,read_actual_coal_mesh


def fixture():
    v=np.array([[.1,.2,.3],[1.1,.2,.3],[.1,1.2,.3]])
    t=np.array([[0,1,2],[0,0,1],[0,2,1],[2,2,2]],dtype=np.int64)
    return v,t


def test_every_removed_source_index_and_coordinate_is_accounted():
    v,t=fixture();original=v.copy();loaded=v.astype(np.float32).astype(float)
    report,arrays=audit_triangle_correspondence(v,t,loaded,t[[0,2]])
    assert report['passed'] is True and report['coordinate_conversion']=='FLOAT32_ROUNDTRIP_EXACT'
    assert report['removed_source_index_set_equals_all_source_exact_zero_area_indices'] is True
    assert [row['source_triangle_index'] for row in report['removed_source_triangles']]==[1,3]
    np.testing.assert_array_equal(arrays['loaded_triangle_to_source_index'],[0,2])
    np.testing.assert_array_equal(arrays['source_triangle_to_loaded_index'],[0,-1,1,-1])
    for row in report['removed_source_triangles']:
        np.testing.assert_array_equal(row['source_vertices_m'],v[t[row['source_triangle_index']]])
        np.testing.assert_array_equal(row['corresponding_loaded_vertices_m'],loaded[t[row['source_triangle_index']]])
        assert row['source_doubled_area_m2']==0.
    assert report['collision_coverage_qualified'] is False
    np.testing.assert_array_equal(original,v)


def test_reordered_faces_retain_exact_indices_but_do_not_claim_original_order():
    v,t=fixture();report,arrays=audit_triangle_correspondence(v,t,v,t[[2,0]])
    assert report['passed'] and report['coordinate_conversion']=='FLOAT64_IDENTITY_EXACT'
    assert not report['retained_source_triangle_order_preserved']
    np.testing.assert_array_equal(arrays['loaded_triangle_to_source_index'],[2,0])


def test_deleting_nonzero_duplicate_face_is_not_hidden_by_an_identical_copy():
    v,_=fixture();t=np.array([[0,1,2],[0,1,2],[0,0,1]])
    report,_=audit_triangle_correspondence(v,t,v,t[:1])
    assert not report['passed'] and report['duplicate_source_triangle_count']==1
    assert not report['all_removed_source_triangles_exact_zero_area']
    assert [x['source_triangle_index'] for x in report['removed_source_triangles']]==[1,2]


def test_same_face_count_with_changed_winding_is_rejected():
    v,_=fixture();source=np.array([[0,1,2],[0,0,1]])
    report,_=audit_triangle_correspondence(v,source,v,np.array([[0,2,1]]))
    assert not report['passed'] and report['unmatched_loaded_triangle_count']==1


def test_float32_collapse_of_nonzero_source_face_is_not_source_zero_area():
    v=np.array([[1.,0.,0.],[1.+1e-9,0.,0.],[1.,1.,0.],[0.,0.,1.]])
    t=np.array([[0,1,2],[0,2,3]])
    report,_=audit_triangle_correspondence(v,t,v.astype(np.float32).astype(float),t[1:])
    assert not report['passed']
    assert report['coordinate_conversion']=='EXACT_COORDINATES_BUT_AMBIGUOUS_DUPLICATE_VERTEX_IDENTITIES'
    assert report['source_zero_area_triangle_count']==0
    assert not report['topology_correspondence_attempted']


@pytest.mark.parametrize('change',['scale','tiny_unexplained'])
def test_vertex_conversion_must_be_observed_exactly_not_guessed_from_counts(change):
    v,t=fixture();loaded=v.astype(np.float32).astype(float)
    if change=='scale':loaded*=2
    else:loaded[0,0]+=1e-12
    report,_=audit_triangle_correspondence(v,t,loaded,t[[0,2]])
    assert not report['passed'] and not report['topology_correspondence_attempted']
    assert report['status']=='UNEXPLAINED_VERTEX_CONVERSION'


def test_exact_vertex_reordering_and_cyclic_face_corners_have_explicit_maps():
    v,t=fixture();order=np.array([1,2,0]);inverse=np.argsort(order)
    loaded=v[order].astype(np.float32).astype(float)
    triangles=inverse[t[[0,2]]][:,[1,2,0]]
    report,arrays=audit_triangle_correspondence(v,t,loaded,triangles)
    assert report['passed'] and report['exact_unique_vertex_permutation_proven']
    assert report['source_vertex_index_equals_loaded_vertex_index'] is False
    assert report['moved_vertex_indices']==3
    assert report['cyclically_rotated_triangle_corner_count']==2
    np.testing.assert_array_equal(arrays['loaded_vertex_to_source_index'],order)
    np.testing.assert_array_equal(arrays['source_vertex_to_loaded_index'],inverse)
    for row in report['removed_source_triangles']:
        np.testing.assert_array_equal(row['corresponding_loaded_vertices_m'],
            loaded[row['corresponding_loaded_vertex_indices']])


def test_ambiguous_duplicate_coordinates_do_not_get_arbitrary_vertex_identity():
    v,t=fixture();v=np.vstack([v,v[0]])
    report,_=audit_triangle_correspondence(v,t,v,t[[0,2]])
    assert not report['passed']
    assert report['coordinate_conversion']=='EXACT_COORDINATES_BUT_AMBIGUOUS_DUPLICATE_VERTEX_IDENTITIES'


def test_actual_geometry_accessor_uses_vertices_and_indexed_triangles():
    v,t=fixture()
    geometry=SimpleNamespace(vertices=lambda:v,num_vertices=3,num_tris=4,tri_indices=lambda i:t[i])
    actual_v,actual_t=read_actual_coal_mesh(geometry)
    np.testing.assert_array_equal(actual_v,v);np.testing.assert_array_equal(actual_t,t)
    actual_v[0,0]=100.
    assert v[0,0]==.1


@pytest.mark.parametrize('bad',[np.nan,np.inf])
def test_nonfinite_vertices_fail_closed(bad):
    v,t=fixture();v[0,0]=bad
    with pytest.raises(ValueError,match='INVALID_VERTICES'):audit_triangle_correspondence(v,t,v,t)
