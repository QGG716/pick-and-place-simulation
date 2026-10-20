"""Directed policy scope checks; these synthetic inventories are not CAD proof."""
from copy import deepcopy
import pytest

from unloading_sim.eco65.collision_policy import (
    effective_collision_policy, LEGACY_POLICY, MAST_MODE, ROBOT_BODY_LINKS)


def inventory():
    sha = 'a'*64
    mast = dict(id='mast_part_019', collision='mast.obj', collision_sha256=sha)
    camera = dict(id='camera_part_001', collision='camera.obj', collision_sha256=sha)
    adapter = dict(name='pitch_saddle', size_m=[.083, .049, .003], T_mast='synthetic')
    mount = dict(schema='synthetic_pitch_mount')
    def row(ident, role, source):
        return dict(id=ident, role=role, source=source, parent_link='link_1',
                    mesh=ident+'.obj', mesh_sha256=sha)
    return dict(collision_policy_mode=MAST_MODE,
        mast=dict(attachment='link_1', parts=[mast],
                  mount_interface=dict(seat=dict(mesh='seat.obj', sha256=sha))),
        permanent_camera=dict(parts=[camera], replaced_mast_parts=[],
                              pitch_adapter=[adapter], mount=mount),
        moving_attachments=[
            row('eco65_mast_mast_part_019', 'permanent_mast',
                dict(part_id='mast_part_019', mesh='mast.obj', sha256=sha)),
            row('eco65_mast_mounting_seat__convex_000', 'permanent_mounting_seat',
                dict(mesh='seat.obj', sha256=sha)),
            row('eco65_camera_camera_part_001', 'permanent_camera',
                dict(part_id='camera_part_001', mesh='camera.obj', sha256=sha)),
            row('eco65_camera_pitch_saddle', 'permanent_camera_adapter',
                dict(definition=adapter, camera_mount=mount))])


def test_exact_assembly_cartesian_pairs_preserve_every_other_collision():
    scene = inventory(); before = deepcopy(scene)
    policy = effective_collision_policy(scene); exemption = policy['robot_mast_exemption']
    assert policy['schema'] == 'eco65_pair_collision_policy_v2'
    assert len(exemption['exempt_pairs']) == 28
    assert exemption['robot_body_links'] == ROBOT_BODY_LINKS
    assert ['link_2', 'eco65_mast_mast_part_019'] in exemption['exempt_pairs']
    assert ['base_link', 'eco65_mast_mounting_seat__convex_000'] in exemption['exempt_pairs']
    assert ['link_6', 'eco65_camera_pitch_saddle'] in exemption['exempt_pairs']
    assert not any('robot_mount_plate' in p or 'part_024_seal' in p or 'carton_r00_l03_c01' in p
                   for p in exemption['exempt_pairs'])
    assert scene == before
    assert exemption['geometry_retained'] is True
    assert {k: policy[k] for k in LEGACY_POLICY if k != 'schema'} == {
        k: v for k, v in LEGACY_POLICY.items() if k != 'schema'}


@pytest.mark.parametrize('change', [
    lambda s: s['moving_attachments'][0].update(id='eco65_mast_looks_similar'),
    lambda s: s['moving_attachments'][0]['source'].update(part_id='unknown'),
    lambda s: s['moving_attachments'][0].update(parent_link='link_2'),
    lambda s: s['moving_attachments'][1].update(id='robot_mount_plate'),
    lambda s: s['moving_attachments'][2]['source'].update(sha256='b'*64),
    lambda s: s['moving_attachments'][3]['source'].update(camera_mount={'unbound': True}),
    lambda s: s['moving_attachments'].append(dict(id='tool', parent_link='link_1',
                                               role='conservative_rigid', source={})),
])
def test_similar_names_and_unbound_assets_cannot_widen_scope(change):
    scene = inventory(); change(scene)
    with pytest.raises(ValueError, match='MAST_EXEMPTION'):
        effective_collision_policy(scene)


def test_historical_entries_keep_v1_and_unknown_modes_fail_closed():
    assert effective_collision_policy({}) == LEGACY_POLICY
    with pytest.raises(ValueError, match='UNKNOWN_ECO65_COLLISION_POLICY_MODE'):
        effective_collision_policy(dict(collision_policy_mode='ignore_everything'))
