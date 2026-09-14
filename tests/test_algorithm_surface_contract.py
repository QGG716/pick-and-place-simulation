from dataclasses import replace
import pytest
from unloading_contracts import dumps,loads,canonical_fingerprint
from unloading_perception.demo import _synthetic_observation
from unloading_perception.algorithm_handoff import validate_algorithm_capture


def observation():
    source=_synthetic_observation(); T=((1.,0.,0.,0.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.))
    surface={'schema_version':'observed_surface_v1','face_id':'upper/capture/1/face0','frame_id':'world','module_id':'upper','capture_id':'capture','source_instance_id':'upper/capture/1','sensor_epoch':source.source_epoch,'clock_domain':source.clock_domain,'capture_time':source.capture_time,'calibration_identity':'cal','T_W_C_at_capture':T,'plane_normal':(0.,0.,1.),'plane_offset_m':-2.,'corners_3d_m':((0.,0.,2.),(1.,0.,2.),(1.,1.,2.),(0.,1.,2.)),'point_support_count':100,'plane_residual_m':.001,'volume_status':'UNKNOWN'}
    cargo=replace(source.cargo[0],observed_surfaces=(surface,),pose=None,full_dimensions_m=None,candidate_eligible=False,eligibility_reasons=('UNKNOWN_VOLUME',))
    return replace(source,cargo=(cargo,),coverage={'expected_modules':('upper','lower'),'received_modules':('upper',),'module_bindings':{'upper':{'capture_id':'capture','calibration_identity':'cal','T_W_C_at_capture':T}}})


def test_formal_surface_roundtrip_and_fingerprint():
    obs=observation(); restored=loads(dumps(obs)); validate_algorithm_capture(restored)
    assert canonical_fingerprint(obs)==canonical_fingerprint(restored)
    assert restored.cargo[0].full_dimensions_m is None
    assert restored.cargo[0].observed_surfaces[0]['point_support_count']==100


@pytest.mark.parametrize('field,value',[('capture_id','old'),('calibration_identity','wrong'),('sensor_epoch','retired'),('clock_domain','wrong'),('T_W_C_at_capture',((1.,0.,0.,1.),(0.,1.,0.,0.),(0.,0.,1.,0.),(0.,0.,0.,1.)))])
def test_invalid_capture_surface_binding_rejected(field,value):
    obs=observation(); cargo=obs.cargo[0]; face={**cargo.observed_surfaces[0],field:value}
    with pytest.raises(ValueError): validate_algorithm_capture(replace(obs,cargo=(replace(cargo,observed_surfaces=(face,)),)))
