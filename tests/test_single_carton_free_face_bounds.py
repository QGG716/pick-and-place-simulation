"""Free-ray face bounds include independent cuboid truth and refuse unknowns."""
import cv2
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.geometry import recover,corners
from unloading_sim.workbench.single_carton.measured_envelope import conservative_envelope,front_free_witnesses
from test_single_carton_variable_geometry import measured_box


@pytest.mark.parametrize('gap',[1,2,3,6])
def test_missing_edge_band_uses_distant_observed_background_without_inventing_edge(gap):
    p,m,c,R=measured_box();inner=cv2.erode(m.astype('uint8'),np.ones((2*gap+1,)*2,'uint8')).astype(bool)
    p[m&~inner]=np.nan;box=recover(p,m,np.eye(4),conservative_measured=True)
    intervals=box['uncertainty']['intervals'];bounded=[i for i in intervals if i['method']=='SENSOR_FREE_RECTANGLE_SPAN_BOUND']
    assert bounded and max(box['size_m'][:2])<.4
    for i in bounded:
        w=i['free_space_witness'];assert w['pixel_guard']==2 and w['free_margin_m']==.006
        assert w['observed_depth_m']>w['plane_depth_m']+.006
        assert not w['mask_boundary_used_as_carton_edge']
    truth=np.eye(4);truth[:3,:3]=R;truth[:3,3]=c;T=np.asarray(box['pose'])
    assert np.max(abs((corners(truth,[.24,.18,.12])-T[:3,3])@T[:3,:3])-np.asarray(box['size_m'])/2)<1e-7


@pytest.mark.parametrize('outside',['missing','foreground'])
def test_unobserved_or_occluding_background_does_not_bound_a_face(outside):
    p,m,_,_=measured_box();measured=recover(p,m,np.eye(4))
    # A visible side itself is a valid free-space witness for the front plane.
    # Remove it too so this negative fixture has ONLY front-plane returns.
    T=np.asarray(measured['pose']);front=T[:3,3]+T[:3,2]*measured['size_m'][2]/2
    p[m&(abs((p-front)@T[:3,2])>.002)]=np.nan
    inner=cv2.erode(m.astype('uint8'),np.ones((7,7),'uint8')).astype(bool)
    p[m&~inner]=np.nan
    if outside=='missing':p[~m]=np.nan
    else:p[~m]*=.25/p[~m,2,None]
    box=conservative_envelope(measured,p,m,np.eye(4))
    assert all(i['method']!='SENSOR_FREE_RECTANGLE_SPAN_BOUND' for i in box['uncertainty']['intervals'])


def test_image_edge_cannot_supply_missing_pixel_guard():
    yy,xx=np.indices((9,9));rays=np.stack(((xx-4)/20,(yy-4)/20,np.ones(xx.shape)),-1)
    p=rays*.5;p[:2]=rays[:2]*1.4
    T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=[0,0,.55]
    assert front_free_witnesses(p,T,np.array([-.04,-.04,-.05]),np.array([.04,.04,.05]),np.full(3,.003))=={}
