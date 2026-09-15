import math
from scripts.carton_appearance import FACES,atlas_uv,face_dimensions


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
