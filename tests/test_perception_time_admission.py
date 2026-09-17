"""Time-only contrasts use otherwise complete, plannable synthetic geometry."""
from dataclasses import replace
import math

import pytest

from unloading_perception.demo import _synthetic_observation
from unloading_perception.scene import build_scene_update


def observation(capture):
    return replace(_synthetic_observation(), capture_time=capture, processed_time=capture+1,
                   clock_domain='ros_sim_time')


def test_original_future_counterexample_blocks_otherwise_plannable_geometry():
    valid = build_scene_update(observation(100), now=100, max_age_seconds=2)
    assert valid.planning_admissible
    future = build_scene_update(observation(110), now=100, max_age_seconds=2)
    assert not future.planning_admissible
    assert future.blocking_reasons == ('OBSERVATION_TIME_IN_FUTURE',)
    assert future.observation.capture_time == 110
    assert future.observation.processed_time == 111


@pytest.mark.parametrize('capture,tolerance,reasons', [
    (99., 0., ()), (100., 0., ()), (98., 0., ()),
    (math.nextafter(98., -math.inf), 0., ('OBSERVATION_STALE',)),
    (100.125, .125, ()),
    (math.nextafter(100.125, math.inf), .125, ('OBSERVATION_TIME_IN_FUTURE',)),
    (math.nextafter(100., math.inf), 0., ('OBSERVATION_TIME_IN_FUTURE',)),
])
def test_exact_float_boundaries(capture, tolerance, reasons):
    update = build_scene_update(observation(capture), now=100., max_age_seconds=2.,
        future_tolerance_seconds=tolerance, clock_domain='ros_sim_time', clock_initialized=True)
    assert update.blocking_reasons == reasons
    assert update.planning_admissible == (not reasons)


@pytest.mark.parametrize('patch', [
    {'now': math.nan}, {'now': math.inf}, {'max_age_seconds': math.nan},
    {'max_age_seconds': math.inf}, {'max_age_seconds': 0.}, {'max_age_seconds': -1.},
    {'future_tolerance_seconds': math.nan}, {'future_tolerance_seconds': math.inf},
    {'future_tolerance_seconds': -.1},
])
def test_invalid_time_or_configuration_rejected(patch):
    args = dict(now=100., max_age_seconds=2., future_tolerance_seconds=0.,
                clock_domain='ros_sim_time', clock_initialized=True)
    args.update(patch)
    with pytest.raises(ValueError): build_scene_update(observation(100), **args)


@pytest.mark.parametrize('domain,initialized,reason', [
    ('ros', True, 'OBSERVATION_CLOCK_DOMAIN_MISMATCH'),
    ('', True, 'OBSERVATION_CLOCK_DOMAIN_MISMATCH'),
    (None, True, 'OBSERVATION_CLOCK_DOMAIN_MISMATCH'),
    ('ros_sim_time', False, 'CLOCK_NOT_INITIALIZED'),
    ('ros_sim_time', None, 'CLOCK_NOT_INITIALIZED'),
])
def test_explicit_online_clock_context(domain, initialized, reason):
    update = build_scene_update(observation(100), now=100, max_age_seconds=2,
        clock_domain=domain, clock_initialized=initialized)
    assert reason in update.blocking_reasons
    assert not update.planning_admissible


def test_offline_zero_is_not_ros_initialization_and_online_context_cannot_skip_age():
    assert build_scene_update(observation(0), now=0, max_age_seconds=2).planning_admissible
    online = build_scene_update(observation(0), now=0, max_age_seconds=2,
        clock_domain='ros_sim_time', clock_initialized=False)
    assert online.blocking_reasons == ('CLOCK_NOT_INITIALIZED',)
    with pytest.raises(ValueError):
        build_scene_update(observation(0), clock_domain='ros_sim_time', clock_initialized=True)


def test_illegal_observation_times_and_missing_domain_rejected_by_contract():
    for patch in ({'capture_time': math.nan}, {'processed_time': math.inf}, {'clock_domain': ''}):
        with pytest.raises(ValueError): replace(observation(100), **patch)


def test_processed_time_is_not_age_and_wrong_domains_are_not_subtracted():
    obs = replace(observation(100), processed_time=1e6)
    assert build_scene_update(obs, now=100, max_age_seconds=2).planning_admissible
    update = build_scene_update(observation(110), now=100, max_age_seconds=2,
        clock_domain='ros', clock_initialized=True)
    assert update.blocking_reasons == ('OBSERVATION_CLOCK_DOMAIN_MISMATCH',)
