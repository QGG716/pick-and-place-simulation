"""Directed tests for scaled visual adapters and source-time phase semantics."""
import importlib.util
from pathlib import Path
import pytest


def load(name):
    path=Path(__file__).resolve().parents[1]/'scripts'/f'{name}.py'
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


cartons=load('carton_appearance')
belts=load('m710_belt_visual')


def test_parent_scaled_carton_maps_to_unit_extent_once():
    scale,shift=cartons.normalization([2,3,4],[2.6,3.4,4.3],[.6,.4,.3])
    for low,high,s,t,nominal in zip([2,3,4],[2.6,3.4,4.3],scale,shift,[.6,.4,.3]):
        assert (low*s+t)*nominal == pytest.approx(-nominal/2)
        assert (high*s+t)*nominal == pytest.approx(nominal/2)


def test_reject_unsuitable_asset_aspect():
    with pytest.raises(ValueError,match='ASPECT_RATIO'):
        cartons.normalization([0,0,0],[1,1,1],[.6,.4,.3])


def result():
    return dict(conveyor_speed_command_m_s=.3,conveyor_started_time_s=0,
        conveyor_stopped_time_s=None,conveyor_active_surface_history=[
            dict(time_s=0,active_surfaces=['transverse','longitudinal']),
            dict(time_s=2,active_surfaces=['longitudinal']),
            dict(time_s=5,active_surfaces=['transverse','longitudinal'])],
        physics_steps=2400,physics_hz=240,conveyor_visual_motion=dict(
            independent_phase_accumulators_m={'transverse':2.1,'longitudinal':3.0}))


def test_stop_hold_resume_and_independent_surfaces():
    r=result()
    assert belts.distance_at(r,'transverse',2)==pytest.approx(.6)
    assert belts.distance_at(r,'transverse',4.9)==pytest.approx(.6)
    assert belts.distance_at(r,'transverse',6)==pytest.approx(.9)
    assert belts.distance_at(r,'longitudinal',6)==pytest.approx(1.8)
    r['conveyor_stopped_time_s']=7
    assert belts.distance_at(r,'longitudinal',10)==pytest.approx(2.1)


def test_phase_continuity_within_world_and_reset_between_worlds():
    first=result(); second=result(); third=result()
    second['conveyor_visual_motion']['independent_phase_accumulators_m']={'transverse':4.2,'longitudinal':6.}
    samples=[dict(result=r,clip={'world_session_id':w}) for r,w in [(first,'A'),(second,'A'),(third,'B')]]
    schedule=belts.build_schedules(samples)
    assert schedule[1]['initial_phase_m']=={'transverse':2.1,'longitudinal':3.}
    assert schedule[2]['initial_phase_m']=={'transverse':0.,'longitudinal':0.}


def test_phase_reconstruction_must_match_recorded_evidence():
    r=result();r['conveyor_speed_command_m_s']*=16
    with pytest.raises(AssertionError):
        belts.build_schedules([dict(result=r,clip={'world_session_id':'A'})])
