"""Real metric fit on deterministic CPU depths; no model/fit/validator substitutes."""
import numpy as np
import pytest

from unloading_perception.final_geometry import project
from unloading_perception.metric_faces import fit_metric_faces, validate_metric_record


def test_unprojectable_candidate_is_retained_as_geometric_rejection():
    K = np.array([[30., 0, 50], [0, 30., 50], [0, 0, 1]])
    y, x = np.indices((100, 100))
    region = (x >= 30) & (x < 80) & (y >= 20) & (y < 80)
    # A slanted plane plus a thin contradictory observed strip. The strip is
    # excluded by the existing fit support but affects boundary construction.
    region[33:36, 8:31] = True
    z = 1/(1+(x-50)/30+1e-10)
    depth = np.where(region & (z > 0), z, np.inf)
    depth[33:36, 8:31] = 1.
    labels = np.where(region, 0, -1)
    normal = np.array([1., 0, 1.])/np.sqrt(2.)
    record = fit_metric_faces(depth, region, K, labels,
        [{'normal': normal.tolist(), 'offset_m': -1/np.sqrt(2.)}], mask_id=7)
    assert record['mask_id'] == 7 and not record['accepted']
    assert not record['camera_facing_faces']
    assert record['frozen_support_regions']['0']  # Instance/support were not deleted.
    rejection, = record['patch_construction_rejections']
    assert rejection['status'] == 'REJECTED' and rejection['reason'] == 'FACE_BEHIND_CAMERA'
    assert rejection['support']['status'] == 'PASS'
    with pytest.raises(ValueError, match='FACE_BEHIND_CAMERA'):
        project(rejection['corners_3d_m'], K)  # The former unhandled projection.
    again = validate_metric_record(record, depth, region, K)
    assert again['final_face_validation'] == record['final_face_validation'] == [rejection]
    assert not again['accepted'] and not again['camera_facing_faces']


def test_valid_observed_patch_still_has_no_construction_rejection():
    K = np.array([[100., 0, 40], [0, 100., 40], [0, 0, 1]])
    region = np.zeros((80, 80), bool)
    region[10:70, 10:70] = True
    depth = np.where(region, 2., np.inf)
    record = fit_metric_faces(depth, region, K, np.where(region, 0, -1),
        [{'normal': [0., 0., 1.], 'offset_m': -2.}], mask_id=8)
    assert record['mask_id'] == 8 and len(record['camera_facing_faces']) == 1
    assert not record.get('patch_construction_rejections')
    assert not record['accepted']  # A single observed surface is not a complete box.
