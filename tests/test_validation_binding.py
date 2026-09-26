"""Input ownership regressions through the official production connector."""
from copy import deepcopy
from dataclasses import replace

import pytest

from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.motion_validation import Status
from unloading_sim.validation_physics import RigidAttachment
from test_lookahead_contact import START_Q
from test_validation_kernel_official import official_scene


@pytest.fixture
def binding_scene(official_scene):
    scene, connector = official_scene
    connector.start_planning_request()
    connector._deadline_monotonic = None
    connector.validation_budget.deadline = None
    return scene, connector, list(deepcopy(scene.all_obstacles))


def edge(validator):
    return validator.check_motion(START_Q, START_Q + .000001)


def changed(validator):
    samples = validator.statistics['state_samples']
    state_keys, edge_keys = list(validator.states), list(validator.edges)
    result = edge(validator)
    assert result.status == Status.INDETERMINATE, result.evidence()
    assert result.failure['reason'] == 'VALIDATION_CONTEXT_CHANGED'
    assert not result.cache_hit
    assert validator.statistics['state_samples'] == samples
    # Previously checked semantic evidence may remain, but this stale binding
    # must neither publish it nor add unchecked/new results to either cache.
    assert list(validator.states) == state_keys and list(validator.edges) == edge_keys


def test_equal_scene_new_binding_watches_current_scene(binding_scene, monkeypatch):
    _, c, a = binding_scene
    va = c._motion_validator(a, stage='pregrasp')
    assert edge(va).valid
    b = deepcopy(a)
    vb = c._motion_validator(b, stage='pregrasp')
    assert vb.context.context_id == va.context.context_id
    assert edge(vb).valid
    b[-1].center[0] += .001
    changed(vb)
    # Getting B must not rebind the still-held A validator.
    assert edge(va).valid and edge(va).cache_hit
    assert vb is not va
    seen = []
    original = c._state_failure
    def record(q, obstacles, **options):
        seen.append(obstacles)
        return original(q, obstacles, **options)
    monkeypatch.setattr(c, '_state_failure', record)
    current = c._motion_validator(b, stage='pregrasp')
    assert current is not vb
    assert current.context.context_id != vb.context.context_id
    # Moving a remote carton is not necessarily a collision. Separately verify
    # invalidation above and the actual, current geometry verdict here.
    result = edge(current)
    assert result.status in (Status.VALID, Status.INVALID)
    assert seen and all(obstacles is b for obstacles in seen)
    assert (original(START_Q, b, stage='pregrasp') is None) == result.valid
    changed(vb)


@pytest.mark.parametrize('operation', ['append', 'delete', 'replace'])
def test_same_elements_different_container_invalidate(binding_scene, operation):
    _, c, a = binding_scene
    va = c._motion_validator(a, stage='pregrasp')
    assert edge(va).valid
    b = list(a)
    vb = c._motion_validator(b, stage='pregrasp')
    assert vb.context.context_id == va.context.context_id
    assert edge(vb).valid
    if operation == 'delete':
        b.pop()
    else:
        box = deepcopy(b[-1])
        box.center[0] += .001
        if operation == 'append':
            b.append(box)
        else:
            b[-1] = box
    changed(vb)
    assert edge(va).valid and edge(va).cache_hit


@pytest.mark.parametrize('dependency', ['attachment', 'target_contact', 'support_names'])
def test_equal_optional_dependencies_have_independent_leases(binding_scene, monkeypatch, dependency):
    scene, c, obstacles = binding_scene
    if dependency == 'attachment':
        rigid = RigidAttachment.capture(c.physical_from_virtual(c.robot.fk(START_Q)), scene.cartons[0])
        old = PhysicalContactAttachment(c.robot, rigid, c.flange_from_virtual_task_tcp.copy(),
                                        c.flange_from_physical_contact.copy())
        new = replace(old, rigid=deepcopy(rigid))
        mutate = lambda: new.rigid.tcp_from_box.__setitem__((0, 3), new.rigid.tcp_from_box[0, 3] + .001)
    elif dependency == 'target_contact':
        old = deepcopy(scene.cartons[0]); new = deepcopy(old)
        mutate = lambda: new.center.__setitem__(0, new.center[0] + .001)
    else:
        old = []; new = []
        mutate = lambda: new.append('new_support')
    options = dict(stage='transit' if dependency == 'attachment' else 'pregrasp')
    va = c._motion_validator(obstacles, **options, **{dependency: old})
    before = edge(va)
    assert before.status in (Status.VALID, Status.INVALID)
    vb = c._motion_validator(obstacles, **options, **{dependency: new})
    assert vb.context.context_id == va.context.context_id
    assert edge(vb).status == before.status
    mutate()
    changed(vb)
    assert edge(va).status == before.status and edge(va).cache_hit
    seen = []
    original = c._state_failure
    def record(q, obstacles, **kwargs):
        seen.append(kwargs[dependency])
        return original(q, obstacles, **kwargs)
    monkeypatch.setattr(c, '_state_failure', record)
    current = c._motion_validator(obstacles, **options, **{dependency: new})
    assert current.context.context_id != vb.context.context_id
    assert edge(current).status in (Status.VALID, Status.INVALID)
    assert seen and all(value is new for value in seen)


def test_same_binding_prepared_and_edge_caches_remain_hot(binding_scene, monkeypatch):
    _, c, obstacles = binding_scene
    supports = []
    v = c._motion_validator(obstacles, stage='pregrasp', support_names=supports)
    assert edge(v).valid
    builds = c.context_statistics['full_context_builds']
    hits = c.context_statistics['prepared_context_hits']
    def unexpected(*args, **kwargs):
        pytest.fail('hot binding must not rebuild context or dispatch state checks')
    monkeypatch.setattr(c, '_validation_context', unexpected)
    monkeypatch.setattr(c, '_state_failure', unexpected)
    assert c._motion_validator(obstacles, stage='pregrasp', support_names=supports) is v
    assert edge(v).cache_hit
    assert c.context_statistics['full_context_builds'] == builds
    assert c.context_statistics['prepared_context_hits'] == hits + 1


def test_second_level_reuse_checks_binding_after_prepared_eviction(binding_scene):
    _, c, a = binding_scene
    va = c._motion_validator(a, stage='pregrasp')
    c._prepared_contexts.clear()
    assert c._motion_validator(a, stage='pregrasp') is va
    b = list(a)
    c._prepared_contexts.clear()
    vb = c._motion_validator(b, stage='pregrasp')
    assert edge(vb).valid
    b.pop()
    changed(vb)
    assert edge(va).valid


def test_prepared_cache_checks_identity_not_just_lookup_key(binding_scene):
    _, c, a = binding_scene
    va = c._motion_validator(a, stage='pregrasp')
    b = list(a)
    vb = c._motion_validator(b, stage='pregrasp')
    # Retain strong references and reject even a candidate found under a wrong
    # lookup key; correctness must not rely on uniqueness of bare integer IDs.
    assert va._layout_input_binding[0] is a
    assert vb._layout_input_binding[0] is b
    b_key = next(key for key, value in c._prepared_contexts.items() if value is vb)
    c._prepared_contexts[b_key] = va
    assert c._motion_validator(b, stage='pregrasp') is vb


def test_new_binding_changed_during_preparation_cannot_reuse_old_result(binding_scene, monkeypatch):
    _, c, a = binding_scene
    va = c._motion_validator(a, stage='pregrasp')
    assert edge(va).valid
    b = deepcopy(a)
    original = c._validation_context
    def mutate(obstacles, **options):
        result = original(obstacles, **options)
        assert obstacles is b
        b[-1].center[0] += .001
        return result
    monkeypatch.setattr(c, '_validation_context', mutate)
    vb = c._motion_validator(b, stage='pregrasp')
    changed(vb)
    assert edge(va).valid and edge(va).cache_hit
