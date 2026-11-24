import numpy as np
import pytest

from unloading_sim.workbench.single_carton import debug_obstacles as dbg


def pose(x, y=0., z=0.):
    T = np.eye(4); T[:3, 3] = (x, y, z); return T.tolist()


BOX = dict(id='t', pose=pose(0.), size_m=[0.2, 0.2, 0.2])
SITE = dict(debug_near_target_obstacles=dict(mode=dbg.MODE, near_m=0.01, basis='test'))


def world():
    return [
        dict(id='t', pose=pose(0.), size=[0.2, 0.2, 0.2], category='carton'),
        dict(id='near_obs', pose=pose(0.2 + 0.005), size=[0.2, 0.2, 0.2], category='observed'),
        dict(id='far_obs', pose=pose(0.2 + 0.05), size=[0.2, 0.2, 0.2], category='observed'),
        dict(id='shadow', pose=pose(0., 0.19), size=[0.2, 0.2, 0.2], category='neighbor_shadow'),
        dict(id='other_carton', pose=pose(-0.2), size=[0.2, 0.2, 0.2], category='carton'),
        dict(id='support', pose=pose(0., 0., -0.2), size=[0.2, 0.2, 0.2], category='observed'),
        dict(id='bound', pose=pose(0., -0.2), size=[0.2, 0.2, 0.2], category='workspace_boundary'),
        dict(id='adj', pose=pose(0.), category='adjacent_contact',
             convex_mesh=dict(vertices=[[-0.1, 0.104, 0.], [0.1, 0.104, 0.], [0., 0.15, 0.1]], triangles=[[0, 1, 2]])),
    ]


def test_absent_or_disabled_is_unchanged():
    w = world()
    assert dbg.apply({}, w, [BOX], lambda *_: [])[0] is w
    assert dbg.apply(dict(debug_near_target_obstacles=dict(mode='DISABLED')), w, [BOX], lambda *_: [])[1] is None


def test_removes_only_rgbd_objects_within_near_m():
    kept, evidence = dbg.apply(SITE, world(), [BOX], lambda w, b: [o for o in w if o['id'] == 'support'])
    removed = {o['id'] for o in evidence['removed']}
    assert removed == {'near_obs', 'shadow', 'adj'}
    assert {o['id'] for o in kept} == {'t', 'far_obs', 'other_carton', 'support', 'bound'}
    assert evidence['status'] == 'USER_DEBUG_PREMISE_NOT_SENSOR_FREE_SPACE'


@pytest.mark.parametrize('value', [dict(mode='X', near_m=0.01, basis='b'), dict(mode=dbg.MODE, near_m=0.2, basis='b'),
                                   dict(mode=dbg.MODE, near_m=0.01, basis=' '), dict(mode=dbg.MODE, basis='b')])
def test_invalid_policy(value):
    with pytest.raises(ValueError):
        dbg.policy(dict(debug_near_target_obstacles=value))


def test_front_corridor_removes_only_objects_in_front_of_the_face():
    w = [dict(id='front', pose=pose(0., 0., 0.17), size=[0.02, 0.02, 0.02], category='observed'),
         dict(id='beside', pose=pose(0.2, 0., 0.17), size=[0.02, 0.02, 0.02], category='observed'),
         dict(id='beyond', pose=pose(0., 0., 0.35), size=[0.02, 0.02, 0.02], category='observed')]
    site = dict(debug_near_target_obstacles=dict(mode=dbg.MODE, near_m=0.01, front_corridor_m=0.1, basis='b'))
    kept, evidence = dbg.apply(site, w, [BOX], lambda *_: [])
    assert [o['id'] for o in evidence['removed']] == ['front']
    assert evidence['removed'][0]['near_targets'][0]['rule'] == 'FRONT_CORRIDOR'
    assert dbg.apply(SITE, w, [BOX], lambda *_: [])[1]['removed'] == []
