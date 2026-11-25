"""User decisions of 2026-11-25: whole flexible cups, passive cups off neighbours,
lower two cup rows on the face, and the camera pitch mount turned 90 deg (collision model only)."""
import json

import numpy as np
import pytest

from test_single_carton_planner import case  # noqa: F401
from test_single_carton_candidate_strategy import BASIS, example, front_box, single, tool
from unloading_sim.workbench.single_carton import passive_cups as pc
from unloading_sim.workbench.single_carton.config import issues
from unloading_sim.workbench.ui import ROOT


def test_cup_bodies_bind_the_other_collision_of_each_cup_part():
    t = tool(); c = example()
    connected = c['topology']['connected_cup_ids']
    lips = {s['id']: s['contact_band'] for s in t['seals']}
    links = [x['id'] for x in t['collisions']]
    bodies = pc.cup_bodies(t, connected, lips, links)
    assert bodies['part_006_seal'] == 'part_006_rigid' and len(set(bodies.values())) == 12
    assert all(v.endswith('_rigid') for v in bodies.values())
    with pytest.raises(ValueError, match='CUP_BODY_LIP_BINDING_CHANGED'):
        pc.cup_bodies(t, connected, dict(lips, part_006_seal='part_008_lip'), links)
    with pytest.raises(ValueError, match='CUP_BODY_NOT_A_TOOL_LINK'):
        pc.cup_bodies(t, connected, lips, [x for x in links if x != 'part_006_rigid'])


def test_cup_body_and_passive_policies_are_validated():
    c = example(); lip = dict(mode='CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS', basis=BASIS)
    assert not any('cup_body_basis' in e for e in issues(dict(c, neighbor_lip_contact=dict(lip, cup_body_basis=BASIS))))
    for bad in (dict(lip, cup_body_basis=' '), dict(lip, cup_body_basis=1), dict(mode='DISABLED', cup_body_basis=BASIS)):
        assert any('cup_body_basis' in e for e in issues(dict(c, neighbor_lip_contact=bad))), bad
    assert any('cup_body_basis' in e for e in issues(dict(c, planning_module='NATIVE', neighbor_lip_contact=dict(lip, cup_body_basis=BASIS))))
    good = dict(mode=pc.MODE, basis=BASIS)
    assert pc.policy(dict(passive_cup_neighbour=good))['toward_tool_m'] == .02
    assert pc.policy(dict(passive_cup_neighbour=dict(mode='DISABLED'))) is None and pc.policy({}) is None
    for bad in (dict(good, basis=''), dict(good, mode='X'), dict(good, toward_tool_m=.2), dict(good, extra=1)):
        with pytest.raises(ValueError):
            pc.policy(dict(passive_cup_neighbour=bad))
    assert any('passive_cup_neighbour' in e for e in issues(dict(c, passive_cup_neighbour=dict(good, basis=''))))


def test_cup_bodies_join_the_lips_per_object_and_need_the_v3_worker(case):
    p, obs, box, folder, calls, opts = case
    t = p.source['tool']; connected = p.site['topology']['connected_cup_ids']
    world = [dict(id=box['id'], pose=box['pose'], size=box['size_m'], category='carton'),
             dict(id='other', pose=np.eye(4).tolist(), size=[.1, .1, .1], category='carton')]
    lip = dict(mode='CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS', basis=BASIS, cup_body_basis=BASIS)
    p.site = dict(p.site, neighbor_lip_contact=lip)
    p.contact_geometry = dict(p.contact_geometry, connected_cup_ids=connected,
                              cup_body_by_contour=pc.cup_bodies(t, connected, p.contact_geometry['terminal_band_by_contour'],
                                                                [x['id'] for x in t['collisions']]))
    candidate = dict(target_lip_contact_ids=connected[:2], active_ids=connected[:1], face=box['faces'][0])
    ctx = p.context([0.]*6, world, box, candidate, 'approach')
    neighbour = [b for a, b in ctx['allowed_pairs'] if a == 'other']
    assert neighbour == [s.replace('_seal', '_lip') for s in connected] + [s.replace('_seal', '_rigid') for s in connected]
    assert ctx['single_carton_contact']['neighbor_lip_contact']['cup_body_basis'] == BASIS
    ready = dict(capabilities=dict(bounded_single_carton_planning=True, neighbor_lip_contact_v1=True, neighbor_lip_contact_v2=True))
    with pytest.raises(ValueError, match='NATIVE_WORKER_UPDATE_REQUIRED_FOR_NEIGHBOR_CUP_BODY'):
        p.worker_ready(ready)


def test_passive_cups_facing_a_neighbour_are_counted_literal_geometry():
    ring = [[.02*np.cos(a), .02*np.sin(a), 0.] for a in np.linspace(0, 2*np.pi, 12, endpoint=False)]
    shift = lambda r, dy: [[x, y+dy, z] for x, y, z in r]
    candidate = dict(tcp_pose=np.eye(4).tolist(),               # suction axis +Z, ring plane z=0
                     target_lip_rings_tcp=dict(a=ring, b=shift(ring, .1), c=shift(ring, .2)),
                     expected_passively_closed_ids=['b', 'c'])
    box = dict(id='target')
    lower = np.eye(4); lower[:3, 3] = [0., .1, -.015]            # 10 mm thick slab 1-2 cm toward the tool, under cup b
    far = np.eye(4); far[:3, 3] = [0., .2, -.05]                 # 4.5 cm toward the tool: beyond the 2 cm sweep
    world = [dict(id='target', pose=np.eye(4).tolist(), size=[1., 1., 1.]),
             dict(id='lower', pose=lower.tolist(), size=[.1, .05, .01]),
             dict(id='far', pose=far.tolist(), size=[.1, .05, .01])]
    out = pc.facing(candidate, world, box, pc.policy(dict(passive_cup_neighbour=dict(mode=pc.MODE, basis=BASIS))))
    assert out['count'] == 1 and out['facing_neighbour'] == [dict(cup='b', objects=['lower'])]


def test_single_contact_lower_two_rows_centred_on_the_face():
    from unloading_sim.workbench.single_carton.geometry import contact_candidates, contact_search_settings, transform
    c, search = single(vertical_placement='LOWER_TWO_ROWS_ON_FACE', vertical_placement_basis=BASIS)
    box = front_box()
    cand = contact_candidates(box, tool(), c['topology'], search)[0]
    T = np.asarray(cand['tcp_pose'])
    z = {k: transform(T, np.asarray(r)).mean(0)[2] for k, r in cand['target_lip_rings_tcp'].items()}
    rows = sorted(set(np.round(list(z.values()), 4)))
    assert len(rows) == 3
    lower = [v for v in z.values() if round(v, 4) <= rows[1]]
    assert np.mean(lower) == pytest.approx(box['faces'][0]['center_m'][2], abs=1e-9)
    info = cand['contact_search']['single_contact']
    assert info['vertical_placement'] == 'LOWER_TWO_ROWS_ON_FACE'
    assert info['up_shift_in_face_m'] == pytest.approx((rows[2]-rows[1])/2, abs=1e-6) and info['up_shift_in_face_m'] > 0
    centred = contact_candidates(box, tool(), c['topology'], single()[1])[0]
    assert centred['contact_search']['single_contact']['vertical_placement'] == 'FACE_CENTER'
    with pytest.raises(ValueError):
        contact_search_settings(dict(search, single_contact=dict(search['single_contact'], vertical_placement_basis='')))
    with pytest.raises(ValueError):
        contact_search_settings(dict(search, single_contact=dict(search['single_contact'], vertical_placement='TOP')))


def test_pitch_mount_turns_90_cw_with_the_camera_pole_and_bar_unchanged():
    import yaml
    from unloading_sim.eco65.camera_installation import apply_pitch_mount_yaw, pole_axis, installation_fingerprint
    scene = json.loads((ROOT/'outputs/workbench/scene_snapshot.json').read_text())['scene']
    config = yaml.safe_load((ROOT/'configs/workcells/eco65_camera_pitch_mount_cw90_v1.yaml').read_text())
    out = apply_pitch_mount_yaw(scene, config)
    assert out['mast'] == {**scene['mast'], 'T_mast_camera': out['mast']['T_mast_camera'],
                           'relative_heading_deg': out['mast']['relative_heading_deg']}   # every mast part unchanged
    A = np.asarray(out['permanent_camera']['pitch_mount_yaw']['T_new_from_old_mast'])
    assert np.allclose(A[:3, :3], [[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]], atol=1e-12)   # -90 deg about +Z
    p = np.asarray(pole_axis(scene['mast'])['point_mast_m'])
    assert np.allclose(A[:3, :3]@p+A[:3, 3], p, atol=1e-12)                          # about the pole axis
    old = np.asarray(scene['permanent_camera']['T_link1_optical']); new = np.asarray(out['permanent_camera']['T_link1_optical'])
    assert np.allclose(old[:3, 2], [0., -np.sqrt(3)/2, -.5], atol=1e-3)                # looked -Y of link_1
    assert np.allclose(new[:3, 2], [-np.sqrt(3)/2, 0., -.5], atol=1e-3)                # now -X (arm forward), pitch kept
    assert np.isclose(new[2, 3], old[2, 3])
    rows = {r['id']: r for r in out['moving_attachments']}; before = {r['id']: r for r in scene['moving_attachments']}
    moved = [k for k in rows if rows[k]['T_parent_local'] != before[k]['T_parent_local']]
    assert len(moved) == 24 and all(k.startswith('eco65_camera_') for k in moved)       # 19 camera parts + 5 pitch adapters
    assert out['camera_extrinsics']['installation_fingerprint'] == installation_fingerprint(out['permanent_camera'])
    with pytest.raises(ValueError, match='PITCH_MOUNT_YAW_ALREADY_APPLIED'):
        apply_pitch_mount_yaw(out, config)
    with pytest.raises(ValueError, match='PITCH_MOUNT_YAW_CONFIG_INVALID'):
        apply_pitch_mount_yaw(scene, dict(config, basis=' '))


def test_bar_yaw_turns_only_part_001_about_j1_and_regenerates_its_seat():
    import shutil, sys, uuid, yaml
    sys.path.insert(0, str(ROOT/'tools'))
    from unloading_sim.eco65.layout_installation import apply_bar_yaw
    scene = json.loads((ROOT/'outputs/workbench/scene_snapshot.json').read_text())['scene']
    config = yaml.safe_load((ROOT/'configs/workcells/eco65_mast_bar_ccw90_v1.yaml').read_text())
    for bad in (dict(config, part_ids=['mast_part_019']), dict(config, pivot='POLE'), dict(config, basis=''), dict(config, yaw_deg='90')):
        with pytest.raises(ValueError, match='MAST_BAR_YAW_CONFIG_INVALID'):
            apply_bar_yaw(scene, bad, ROOT/'outputs'/('test-bar-yaw-'+uuid.uuid4().hex))
    out_dir = ROOT/'outputs'/('test-bar-yaw-'+uuid.uuid4().hex)
    try:
        out, assets = apply_bar_yaw(scene, config, out_dir)
    finally:
        shutil.rmtree(out_dir, ignore_errors=True)
    parts = {p['id']: p for p in out['mast']['parts']}; old = {p['id']: p for p in scene['mast']['parts']}
    assert np.allclose(np.asarray(parts['mast_part_001']['T_mast'])[:3, :3], [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]], atol=1e-12)
    assert np.allclose(np.asarray(parts['mast_part_001']['T_mast'])[:3, 3], 0., atol=1e-12)    # about the J1 line
    assert all(parts[k] == old[k] for k in parts if k != 'mast_part_001')                      # pole and clamp unchanged
    assert out['permanent_camera'] == scene['permanent_camera'] and out['mast']['T_link1_mast'] == scene['mast']['T_link1_mast']
    rows = {r['id']: r for r in out['moving_attachments']}; before = {r['id']: r for r in scene['moving_attachments']}
    changed = [k for k in rows if k in before and rows[k]['T_parent_local'] != before[k]['T_parent_local']]
    assert changed and all(before[k]['source']['part_id'] == 'mast_part_001' for k in changed)
    seats = [k for k in rows if rows[k]['role'] == 'permanent_mounting_seat']
    assert seats and set(seats) == set(out['mast']['bar_yaw']['new_seat_collision_ids'])
    ifaces = {r['attachment_id'] for r in out['fixed_attachment_interfaces']}
    assert set(seats) <= ifaces and not (set(out['mast']['bar_yaw']['replaced_seat_collision_ids'])-set(seats)) & ifaces
    with pytest.raises(ValueError, match='MAST_BAR_YAW_ALREADY_APPLIED'):
        apply_bar_yaw(out, config, out_dir)


def test_pole_with_pitch_mount_and_camera_moves_to_the_turned_bar_end():
    import shutil, sys, uuid, yaml
    sys.path.insert(0, str(ROOT/'tools'))
    from unloading_sim.eco65.layout_installation import apply_bar_yaw, apply_pole_to_bar_end
    scene = json.loads((ROOT/'outputs/workbench/scene_snapshot.json').read_text())['scene']
    bar = yaml.safe_load((ROOT/'configs/workcells/eco65_mast_bar_ccw90_v1.yaml').read_text())
    pole = yaml.safe_load((ROOT/'configs/workcells/eco65_mast_pole_to_bar_end_v1.yaml').read_text())
    with pytest.raises(ValueError, match='MAST_POLE_TO_BAR_END_REQUIRES_BAR_YAW'):
        apply_pole_to_bar_end(scene, pole)
    out_dir = ROOT/'outputs'/('test-pole-'+uuid.uuid4().hex)
    try:
        turned, _ = apply_bar_yaw(scene, bar, out_dir)
    finally:
        shutil.rmtree(out_dir, ignore_errors=True)
    out = apply_pole_to_bar_end(turned, pole)
    move = out['mast']['pole_to_bar_end']
    assert np.allclose(move['pole_axis_before_mast_m'][:2], [.1045, -.00405], atol=1e-9)
    assert np.allclose(move['pole_axis_after_mast_m'][:2], [.00405, .1045], atol=1e-9)     # bar far end after +90 deg
    parts = {p['id']: p for p in out['mast']['parts']}; before = {p['id']: p for p in turned['mast']['parts']}
    assert np.allclose(np.asarray(parts['mast_part_019']['T_mast'])[:3, :3],
                       np.asarray(before['mast_part_019'].get('T_mast', np.eye(4)))[:3, :3])  # pole orientation kept
    assert np.allclose(np.asarray(parts['mast_part_010']['T_mast'])[:3, :3], [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]], atol=1e-12)
    assert parts['mast_part_001'] == before['mast_part_001']                                  # bar unchanged here
    old = np.asarray(turned['permanent_camera']['T_link1_optical']); new = np.asarray(out['permanent_camera']['T_link1_optical'])
    assert np.allclose(old[:3, :3], new[:3, :3]) and np.allclose(new[:3, 3]-old[:3, 3], [-.10045, .10855, 0.], atol=1e-6)
    assert move['moved_attachments'] == {'mast_part_010': 1, 'mast_part_019': 1, 'camera': 24}
    with pytest.raises(ValueError, match='MAST_POLE_TO_BAR_END_ALREADY_APPLIED'):
        apply_pole_to_bar_end(out, pole)


def test_contact_failure_moves_on_to_the_next_ik_branch_only_with_the_switch(case, monkeypatch):
    from unloading_sim.workbench.single_carton.planner import SegmentRejected
    p, obs, box, folder, calls, opts = case
    calls_made = []

    def fake_plan(q, world, box_, candidate, sub, screened, *a, **k):
        calls_made.append(list(screened['screen']['approach_q']))
        if len(calls_made) == 1:
            raise SegmentRejected('contact', 'contact:NATIVE_REJECTED:NATIVE_PLANNING_FAILED')
        return dict(segments=[], qpre=screened['screen']['approach_q'])
    monkeypatch.setattr(p, '_plan_candidate_segments', fake_plan)
    monkeypatch.setattr(p, 'endpoint_precheck', lambda q, w, b, c, att, sub, cancelled, sol, branches, label:
                        dict(conclusion='CLEAR', approach_q=list(branches[0]), contact_q=list(branches[0])))
    candidate = dict(tcp_pose=np.eye(4).tolist())
    screened = dict(screen=dict(approach_q=[1.]*6, contact_q=[1.]*6, pending_branches=[[2.]*6]))
    with pytest.raises(SegmentRejected):                                      # switch off: unchanged, contact failure is final
        p.plan_candidate_segments([0.]*6, [], box, candidate, folder, screened)
    calls_made.clear()
    p.site = dict(p.site, contact_failure_next_branch=dict(mode='NEXT_IK_BRANCH', basis=BASIS))
    planned = p.plan_candidate_segments([0.]*6, [], box, candidate, folder, screened)
    assert calls_made == [[1.]*6, [2.]*6]
    assert planned['ik_branch']['fallback_index'] == 1 and planned['ik_branch']['earlier_failures'][0]['stage'] == 'contact'
    # A contact sent-path audit rejection is handed back as a branch retry.
    retry = p.replan_after(dict(planned, pending_branches=[[3.]*6]), SegmentRejected('contact', 'contact:SENT_PATH_REJECTED:x'))
    assert retry['branch_retry']['pending'] == [[3.]*6] and retry['branch_retry']['tried'][-1]['stage'] == 'contact'
    p.site = dict(p.site, contact_failure_next_branch=dict(mode='DISABLED'))
    with pytest.raises(SegmentRejected):
        p.replan_after(dict(planned, pending_branches=[[3.]*6]), SegmentRejected('contact', 'contact:SENT_PATH_REJECTED:x'))
    c = example()
    assert not any('contact_failure_next_branch' in e for e in issues(dict(c, contact_failure_next_branch=dict(mode='NEXT_IK_BRANCH', basis=BASIS))))
    for bad in (dict(mode='NEXT_IK_BRANCH'), dict(mode='X', basis=BASIS), dict(mode='NEXT_IK_BRANCH', basis=BASIS, extra=1)):
        assert any('contact_failure_next_branch' in e for e in issues(dict(c, contact_failure_next_branch=bad))), bad
    assert any('contact_failure_next_branch' in e for e in issues(dict(c, planning_module='NATIVE',
                                                                       contact_failure_next_branch=dict(mode='NEXT_IK_BRANCH', basis=BASIS))))
