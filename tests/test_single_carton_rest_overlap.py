"""Unresolved volumes at a target's rest pose (user decision 2026-11-26
"重叠部分都忽略"): the overlap is ignored, the rest is bound to the target's
bottom (support) or sides (adjacent contact); above the top stays an obstacle.
Measured cartons and the workspace limits are not touched. Fake data only."""
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.rest_overlap import target_world, policy, RULE, MODE
from unloading_sim.workbench.single_carton.geometry import corners
from unloading_sim.workbench.single_carton.optimization_defaults import apply

SETTING = dict(mode=MODE, basis='SYNTHETIC')


def target():
    T = np.eye(4); T[:3, 3] = [-.7, .1, .37]
    return dict(id='t', pose=T.tolist(), size_m=[.2, .14, .14], corners_m=corners(T, [.2, .14, .14]).tolist())


def box(ident, center, size, category):
    T = np.eye(4); T[:3, 3] = center
    return dict(id=ident, pose=T.tolist(), size=list(size), category=category)


def verts(o):
    return np.asarray(o['convex_mesh']['vertices'], float)


def test_lower_shadow_overlapping_the_bottom_becomes_a_bound_support():
    t = target()   # bottom at z = 0.30
    shadow = box('s', [-.7, .1, .25], [.18, .12, .12], 'neighbor_shadow')   # top at 0.31: 10 mm into the target
    world, records = target_world([t, shadow], t, SETTING)
    assert records[0]['overlapped'] and records[0]['ignored_inside_envelope']
    pieces = [o for o in world if o.get('original_id') == 's']
    assert len(pieces) == 1 and pieces[0]['category'] == 'confirmed_stack_support' and pieces[0]['rule'] == RULE
    plane = pieces[0]['support_plane']
    assert np.allclose(plane['normal_world'], [0, 0, 1]) and abs(plane['offset_m'] - .30) < 1e-9
    assert abs(verts(pieces[0])[:, 2].max() - .30) < 1e-9         # nothing left inside the envelope
    assert pieces[0]['supports_target_id'] == 't' and not any(o['id'] == 's' for o in world)


def test_side_neighbour_too_close_is_extended_to_the_face_and_bound_as_adjacent():
    t = target()   # +Y face at y = 0.17
    shadow = box('n', [-.7, .2205, .37], [.1, .1, .1], 'neighbor_shadow')   # y from 0.1705: 0.5 mm gap
    world, _ = target_world([t, shadow], t, SETTING)
    piece = next(o for o in world if o.get('original_id') == 'n')
    assert piece['category'] == 'adjacent_contact'
    n = np.asarray(piece['support_plane']['normal_world'])
    assert np.allclose(n, [0, -1, 0]) and abs(piece['support_plane']['offset_m'] - (-.17)) < 1e-9
    assert abs(verts(piece)[:, 1].min() - .17) < 1e-9             # extended to the face, never shrunk
    assert verts(piece)[:, 1].max() >= .2705 - 1e-9


def test_volume_above_the_top_stays_an_obstacle_but_loses_the_overlap():
    t = target()   # top at z = 0.44
    above = box('a', [-.7, .1, .48], [.1, .1, .1], 'unresolved_carton_volume')   # bottom 0.43: overlaps 10 mm
    world, records = target_world([t, above], t, SETTING)
    piece = next(o for o in world if o.get('original_id') == 'a')
    assert piece['category'] == 'unresolved_carton_volume' and 'supports_target_id' not in piece
    assert abs(verts(piece)[:, 2].min() - .44) < 1e-9 and records[0]['bound'] == []


def test_edge_piece_beside_and_above_is_a_side_contact():
    t = target()
    edge = box('e', [-.7, .19, .45], [.1, .04, .04], 'neighbor_shadow')     # beyond +Y and +Z faces only
    world, _ = target_world([t, edge], t, SETTING)
    piece = next(o for o in world if o.get('original_id') == 'e')
    assert piece['category'] == 'adjacent_contact'


def test_measured_cartons_far_objects_and_workspace_limits_are_untouched():
    t = target()
    carton = box('c', [-.7, .2, .37], [.2, .1, .14], 'carton')                  # overlapping measured carton
    far = box('f', [-.7, .5, .37], [.1, .1, .1], 'neighbor_shadow')
    wall = box('w', [-1.02, 0, .5], [.04, 1.6, 1.45], 'workspace_boundary')
    world_in = [t, carton, far, wall]
    world, records = target_world(world_in, t, SETTING)
    assert world is world_in and records == []


def test_own_support_and_other_targets_contacts():
    t = target()
    own = dict(box('o', [-.7, .1, .25], [.18, .12, .12], 'confirmed_stack_support'), supports_target_id='t')
    other = dict(box('x', [-.7, .1, .25], [.18, .12, .12], 'adjacent_contact'), supports_target_id='u',
                 original_category='neighbor_shadow', geometry_uncertain=True)
    world, records = target_world([t, own, other], t, SETTING)
    assert any(o['id'] == 'o' for o in world)                              # own support kept as is
    assert [r['original_id'] for r in records] == ['x']


def test_policy_validation_and_code_default():
    assert policy({}) is None and policy(dict(unresolved_rest_overlap_policy=dict(mode='DISABLED'))) is None
    with pytest.raises(ValueError):
        policy(dict(unresolved_rest_overlap_policy=dict(mode=MODE)))
    with pytest.raises(ValueError):
        policy(dict(unresolved_rest_overlap_policy=dict(mode='OTHER', basis='x')))
    c = dict(geometry={}); apply(c)
    assert c['geometry']['unresolved_rest_overlap_policy']['mode'] == MODE
    c = dict(geometry=dict(unresolved_rest_overlap_policy=dict(mode='DISABLED'))); apply(c)
    assert c['geometry']['unresolved_rest_overlap_policy'] == dict(mode='DISABLED')


def test_disabled_policy_changes_nothing():
    t = target(); shadow = box('s', [-.7, .1, .25], [.18, .12, .12], 'neighbor_shadow')
    world_in = [t, shadow]
    assert target_world(world_in, t, None) == (world_in, [])


def test_objects_already_accepted_as_supports_are_left_unchanged():
    t = target()
    support = box('sup', [-.7, .1, .29], [.4, .3, .02], 'observed_support')      # top 0.30: wider than the target
    world_in = [t, support]
    assert target_world(world_in, t, SETTING, keep_ids={'sup'}) == (world_in, [])
