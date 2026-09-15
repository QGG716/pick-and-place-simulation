import math
import pytest
from scripts.carton_appearance import FACES,atlas_uv,face_dimensions
from scripts.carton_appearance import normalization
from scripts.carton_appearance import merge_instance_mask


def test_cardboard_tape_and_label_remain_one_independent_instance():
    import numpy as np
    masks,identities={},{}
    raster=np.array([[11,12,13],[21,21,0]])
    for numeric_id in (11,12,13,11):
        merge_instance_mask(masks,identities,'box_a',numeric_id,raster==numeric_id)
    merge_instance_mask(masks,identities,'box_b',21,raster==21)
    assert np.array_equal(masks['box_a'],np.array([[1,1,1],[0,0,0]],dtype=bool))
    assert np.array_equal(masks['box_b'],raster==21)
    assert identities=={'box_a':{11,12,13},'box_b':{21}}


def test_asset_normalization_preserves_full_envelope_and_offcenter_origin():
    low,high=(-.19,-.125,0),(.19,.125,.14875)
    scale,offset=normalization(low,high,(.6,.4,.3))
    for i in range(3):
        assert math.isclose(low[i]*scale[i]+offset[i],-.5)
        assert math.isclose(high[i]*scale[i]+offset[i],.5)


def test_asset_normalization_rejects_flat_box_and_invalid_bounds():
    with pytest.raises(ValueError,match='ASPECT_RATIO'):
        normalization((0,0,0),(.6,.4,.05),(.6,.4,.3))
    with pytest.raises(ValueError,match='ENVELOPE'):
        normalization((0,0,0),(1,1,0),(.6,.4,.3))


def test_visual_box_preserves_six_outward_planar_cube_surfaces():
    seen=set()
    for face in FACES:
        fixed=[i for i in range(3) if len({p[i] for p in face})==1]
        assert len(fixed)==1
        axis=fixed[0]; seen.add((axis,face[0][axis]))
        a=[face[1][i]-face[0][i] for i in range(3)]
        b=[face[3][i]-face[0][i] for i in range(3)]
        n=[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]]
        assert n[axis]*face[0][axis]>0
        assert all(abs(v)==.5 for p in face for v in p)
    assert seen=={(i,s) for i in range(3) for s in (-.5,.5)}


def test_uv_atlas_and_physical_tile_scale_for_unequal_dimensions():
    uv=atlas_uv();assert len(uv)==24
    assert all(0<=v<=1 for p in uv for v in p)
    dimensions=[face_dimensions((.6,.4,.3),face) for face in FACES]
    assert math.isclose(sum(w*h for w,h in dimensions),2*(.6*.4+.6*.3+.4*.3))
    for i in range(6):
        tile=uv[4*i:4*i+4]
        assert math.isclose(max(p[0] for p in tile)-min(p[0] for p in tile),1/3)
        assert math.isclose(max(p[1] for p in tile)-min(p[1] for p in tile),1/2)
