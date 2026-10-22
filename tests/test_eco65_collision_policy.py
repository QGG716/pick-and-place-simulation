"""Directed policy scope checks; these synthetic inventories are not CAD proof."""
from copy import deepcopy
import pytest

from unloading_sim.eco65.collision_policy import (
    effective_collision_policy, LEGACY_POLICY, MAST_MODE, ROBOT_BODY_LINKS,
    EXTRACTION_CARTON_MODE, extraction_carton_pairs)
from unloading_sim.eco65.collision_policy import CUP_NEIGHBOR_MODE, CUP_LIP_LINKS, cup_neighbor_pairs
from unloading_sim.eco65.collision_policy import ROBOT_TOOL_MODE
from unloading_sim.eco65.collision_policy import RIGID_CARTON_MODE, RIGID_TOOL_LINKS, rigid_tool_carton_pairs


def test_rigid_carton_policy_follows_world_attached_released_and_drained_ownership():
    scene=coupled_inventory()
    scene.update(cup_neighbor_contact_mode=CUP_NEIGHBOR_MODE,robot_tool_exemption_mode=ROBOT_TOOL_MODE,
        rigid_tool_carton_contact_mode=RIGID_CARTON_MODE,
        rigid_tool_carton_inventory=[dict(id=i,semantic='conservative_rigid',mesh_sha256='a'*64) for i in RIGID_TOOL_LINKS])
    scene['coupled_conveyor_spec']['config']['cup_neighbor_contact_mode']=CUP_NEIGHBOR_MODE
    policy=effective_collision_policy(scene)
    assert policy['schema']=='eco65_pair_collision_policy_v6'
    assert len(RIGID_TOOL_LINKS)==45 and 'part_006_rigid' in RIGID_TOOL_LINKS
    ids=[b['id'] for b in scene['boxes']];target=scene['target_box_id']
    args=dict(continuous_task_mode='eco65_coupled_cycle_v1',world_carton_ids=ids)
    world=rigid_tool_carton_pairs(policy,**args)
    attached=rigid_tool_carton_pairs(policy,**dict(args,world_carton_ids=[i for i in ids if i!=target],attached_target_id=target))
    assert world==attached and len(world)==540
    assert ['part_000',target] in world and ['part_006_rigid',target] in world
    assert not any('link_3' in p or 'part_006_lip' in p or 'robot_mount_plate' in p for p in world)
    drained=rigid_tool_carton_pairs(policy,**dict(args,world_carton_ids=[i for i in ids if i!=target],completed_ids=[target]))
    assert len(drained)==495 and not any(target in p for p in drained)
    for change in (dict(attached_target_id=target),dict(world_carton_ids=ids[:-1]),
                   dict(world_carton_ids=ids+['unknown']),dict(completed_ids=[target])):
        with pytest.raises(ValueError,match='OWNERSHIP'):
            rigid_tool_carton_pairs(policy,**dict(args,**change))
    scene['rigid_tool_carton_inventory'][0]['id']='link_3'
    with pytest.raises(ValueError,match='EXACT_INVENTORY'):effective_collision_policy(scene)


def test_explicit_link3_part026_permission_preserves_existing_rules():
    scene = coupled_inventory()
    scene['cup_neighbor_contact_mode'] = CUP_NEIGHBOR_MODE
    scene['coupled_conveyor_spec']['config']['cup_neighbor_contact_mode'] = CUP_NEIGHBOR_MODE
    before = effective_collision_policy(scene)
    scene['robot_tool_exemption_mode'] = ROBOT_TOOL_MODE
    policy = effective_collision_policy(scene)
    assert policy['schema'] == 'eco65_pair_collision_policy_v5'
    assert policy['robot_tool_exemption']['exempt_pairs'] == [['link_3', 'part_026']]
    assert {k:policy[k] for k in before if k!='schema'} == {k:v for k,v in before.items() if k!='schema'}
    ids = [b['id'] for b in scene['boxes']]; target = scene['target_box_id']
    assert len(cup_neighbor_pairs(policy, continuous_task_mode='eco65_coupled_cycle_v1',
        target_id=target, world_carton_ids=ids)) == 132
    assert len(extraction_carton_pairs(policy, continuous_task_mode='eco65_coupled_cycle_v1',
        stage='extraction', target_id=target, attached_target_id=target,
        world_carton_ids=[i for i in ids if i!=target])) == 11
    scene['robot_tool_exemption_mode'] = 'all_robot_tool'
    with pytest.raises(ValueError, match='EXACT_SCOPE'):
        effective_collision_policy(scene)


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


def coupled_inventory():
    scene = inventory()
    scene['extraction_carton_contact_mode'] = EXTRACTION_CARTON_MODE
    scene['coupled_conveyor_spec'] = dict(schema='eco65_coupled_spec_v1',
        config=dict(extraction_carton_contact_mode=EXTRACTION_CARTON_MODE))
    scene['target_box_id'] = 'carton_r00_l03_c01'
    scene['boxes'] = [dict(id=f'carton_r00_l{layer:02d}_c{column:02d}',
        pose=[.205, (column-1)*.11, .23+layer*.1], size_m=[.15, .1, .1])
        for layer in range(4) for column in range(3)]
    return scene


def test_coupled_extraction_authorization_is_versioned_and_frozen():
    scene = coupled_inventory()
    policy = effective_collision_policy(scene)
    rule = policy['extraction_carton_contact']
    assert policy['schema'] == 'eco65_pair_collision_policy_v3'
    assert policy['stack_contact_mode'] == EXTRACTION_CARTON_MODE
    assert rule['stage'] == 'extraction' and rule['allow_intersection'] is True
    assert rule['continuous_task_mode'] == 'eco65_coupled_cycle_v1'
    assert rule['original_carton_ids'] == sorted(box['id'] for box in scene['boxes'])
    assert rule['geometry_retained'] is True
    assert rule['physical_execution'] == 'NOT_RUN' and rule['execution_ready'] is False
    assert policy['robot_mast_exemption'] == effective_collision_policy(inventory())['robot_mast_exemption']
    changed = deepcopy(scene)
    changed['boxes'][0]['pose'][0] += .001
    assert (effective_collision_policy(changed)['extraction_carton_contact']['original_carton_inventory_fingerprint']
            != rule['original_carton_inventory_fingerprint'])


def test_extraction_only_current_attached_target_and_remaining_cartons():
    scene = coupled_inventory()
    policy = effective_collision_policy(scene)
    first = 'carton_r00_l03_c01'
    second = 'carton_r00_l03_c00'
    ids = [box['id'] for box in scene['boxes']]
    args = dict(continuous_task_mode='eco65_coupled_cycle_v1', stage='extraction',
                target_id=first, attached_target_id=first,
                world_carton_ids=[name for name in ids if name != first])
    pairs = extraction_carton_pairs(policy, **args)
    assert len(pairs) == 11
    assert all(pair[0] == first for pair in pairs)
    assert [first, 'carton_r00_l02_c01'] in pairs
    assert not any('robot_mount_plate' in pair or 'link_2' in pair for pair in pairs)
    # After the first ideal drain, the second target can only name ten
    # remaining world cartons. The completed carton gains no ghost permission.
    pairs = extraction_carton_pairs(policy, **dict(args, target_id=second,
        attached_target_id=second, world_carton_ids=[x for x in ids if x not in (first, second)],
        completed_ids=[first]))
    assert len(pairs) == 10
    assert all(first not in pair and pair[0] == second for pair in pairs)
    for stage in ('pregrasp', 'contact', 'transit', 'place', 'withdrawal', 'residence'):
        assert extraction_carton_pairs(policy, **dict(args, stage=stage)) == []
    assert extraction_carton_pairs(policy, **dict(args, attached_target_id=None)) == []
    assert extraction_carton_pairs(policy, **dict(args, continuous_task_mode='historical')) == []
    assert extraction_carton_pairs(effective_collision_policy(inventory()), **args) == []


@pytest.mark.parametrize('change', [
    lambda scene: scene.pop('coupled_conveyor_spec'),
    lambda scene: scene['coupled_conveyor_spec']['config'].clear(),
    lambda scene: scene['boxes'][0].update(id='warehouse_carton_unknown'),
    lambda scene: scene['boxes'][0].update(size_m=[.25, .21, .1]),
    lambda scene: scene['boxes'].append(deepcopy(scene['boxes'][0])),
])
def test_extraction_mode_rejects_non_coupled_or_changed_inventory(change):
    scene = coupled_inventory()
    change(scene)
    with pytest.raises(ValueError, match='EXTRACTION_CARTON'):
        effective_collision_policy(scene)


def test_extraction_pairs_fail_closed_on_unknown_or_inconsistent_ownership():
    scene = coupled_inventory()
    policy = effective_collision_policy(scene)
    target = scene['target_box_id']
    world = [b['id'] for b in scene['boxes'] if b['id'] != target]
    args = dict(continuous_task_mode='eco65_coupled_cycle_v1', stage='extraction',
                target_id=target, attached_target_id=target, world_carton_ids=world)
    for updates in (dict(world_carton_ids=[*world, 'robot_mount_plate']),
                    dict(world_carton_ids=[*world, target]),
                    dict(world_carton_ids=world[:-1]),
                    dict(attached_target_id='carton_r00_l03_c00'),
                    dict(completed_ids=[world[0]])):
        with pytest.raises(ValueError, match='RUNTIME_OWNERSHIP'):
            extraction_carton_pairs(policy, **dict(args, **updates))


def test_cup_neighbor_permission_names_all_lips_but_never_target_or_rigid_parts():
    scene = coupled_inventory()
    scene['cup_neighbor_contact_mode'] = CUP_NEIGHBOR_MODE
    scene['coupled_conveyor_spec']['config']['cup_neighbor_contact_mode'] = CUP_NEIGHBOR_MODE
    policy = effective_collision_policy(scene)
    assert policy['schema'] == 'eco65_pair_collision_policy_v4'
    target = scene['target_box_id']
    world = [b['id'] for b in scene['boxes']]
    args = dict(continuous_task_mode='eco65_coupled_cycle_v1',target_id=target,world_carton_ids=world)
    pairs = cup_neighbor_pairs(policy, **args)
    assert len(pairs) == 132
    assert ['part_006_lip','carton_r00_l03_c02'] in pairs
    assert {p[0] for p in pairs} == set(CUP_LIP_LINKS)
    assert all(p[1] != target and '_rigid' not in p[0] for p in pairs)
    # Released target still world-owned: no cup waiver for tool withdrawal.
    assert not any(target in p for p in pairs)
    nxt = 'carton_r00_l03_c00'
    later = cup_neighbor_pairs(policy, **dict(args,target_id=nxt,
        world_carton_ids=[i for i in world if i != target],completed_ids=[target]))
    assert len(later) == 120 and not any(target in p or nxt in p for p in later)
    assert cup_neighbor_pairs(policy, **dict(args,continuous_task_mode='legacy')) == []
    with pytest.raises(ValueError, match='OWNERSHIP'):
        cup_neighbor_pairs(policy, **dict(args,world_carton_ids=world+['conveyor']))
    changed = deepcopy(policy)
    changed['cup_neighbor_contact']['terminal_band_links'][0] = 'part_002_rigid'
    with pytest.raises(ValueError, match='BINDING'):
        cup_neighbor_pairs(changed, **args)
