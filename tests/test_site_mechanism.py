"""Site layout roles and one-DOF lift chain. Synthetic manifest with literal
expected values; the real private manifest is checked only when present."""
from pathlib import Path

import numpy as np
import pytest

from unloading_sim.eco65.site_mechanism import (LiftMechanism, check_range, classify, lift_state, load_manifest,
                                                load_roles)

ROOT = Path(__file__).resolve().parents[1]
ROLES = ROOT / 'configs/workcells/eco65_site_layout_roles_v1.yaml'
REAL = ROOT / 'assets/layout/derived/a4b6ed1298f7491b/site-layout.json'


def part(i, name, lo_mm, hi_mm):
    return dict(id=f'p{i:03}', body_name=name, collision_bounds_m=(np.array([lo_mm, hi_mm], float) / 1000).tolist())


@pytest.fixture
def synthetic():
    roles = load_roles(ROLES)
    parts = [
        # hinges: centres (y, z) = (444.5, -801.8) and (702.8, 146.7) mm, coaxial along X
        part(0, '转轴', [44, 429.5, -817.8], [74, 459.5, -785.8]),
        part(1, '转轴', [324, 429.5, -817.8], [354, 459.5, -785.8]),
        part(2, '转轴2', [14, 687.8, 131.7], [74, 717.8, 161.7]),
        part(3, '转轴2', [324, 687.8, 131.7], [384, 717.8, 161.7]),
        part(4, 'YLYJ-JC00-01_底板', [-194.5, 30.8, -1110], [365.5, 40.8, -810]),
        part(5, 'YLYJ-JC00-07_顶板', [-194.5, 366.8, -1110], [365.5, 376.8, -810]),
        part(6, 'YLYJ-JC00-04_剪叉臂(前)', [-194.5, 73.8, -870], [355.4, 333.8, -830]),
        part(7, 'HG30-NEW.stp', [12, 630, -320], [40, 656, 180]),
        part(8, 'QHH30C_左封.stp', [-4, 636, 106.1], [40, 675, 203.6]),
        part(9, '_BEZ01-210-10000_kj^BEZ01', [74, 408, -832], [324, 738, 178]),
        part(10, '箱子模型', [-272, -111, -1541], [-72, 89, -1341]),
        part(11, 'liangan3', [-211, 798, -789], [-64, 1019, -492]),
        part(12, '6060-L1440(Default<按加工>)', [342, -30, -720], [402, 30, 720]),
        part(13, 'GBT 70.1-M4X20', [92, 356, -854], [99, 376.8, -847]),
    ]
    manifest = dict(source_sha256=roles['source_sha256'], parts=parts)
    return manifest, roles


def test_roles_follow_rules_and_unmatched_parts_stay_fixed(synthetic):
    manifest, roles = synthetic
    c = classify(manifest, roles)
    assert [c[f'p{i:03}']['role'] for i in range(14)] == [
        'LIFT_TOP', 'LIFT_TOP', 'BELT_SLIDER', 'BELT_SLIDER', 'FIXED', 'LIFT_TOP', 'LIFT_INTERIOR', 'FIXED',
        'BELT_SLIDER', 'BELT_INCLINED', 'CARTON', 'ROBOT_LINK', 'FIXED', 'LIFT_TOP']
    assert c['p012']['basis'] == 'NO_RULE_MATCHED_KEPT_AS_FIXED_OBSTACLE'
    with pytest.raises(ValueError, match='SITE_ROLES_SOURCE_MISMATCH'):
        classify(dict(manifest, source_sha256='0' * 64), roles)


def test_lift_chain_literal_values(synthetic):
    m = LiftMechanism(*synthetic)
    assert m.length_m == pytest.approx(0.9830417793766448, abs=1e-9)
    assert np.degrees(m.angle0_rad) == pytest.approx(15.233648889106476, abs=1e-6)
    assert (m.base_top_y, m.top_under_y) == pytest.approx((0.0408, 0.3668))
    s = m.state(-0.1)
    assert s['slider_shift_m'] == pytest.approx(-0.033080833716051083, abs=1e-9)
    assert s['belt_angle_deg'] == pytest.approx(21.375642873849543, abs=1e-6)
    assert s['plate_gap_m'] == pytest.approx(0.226)
    s = m.state(0.05)
    assert s['slider_shift_m'] == pytest.approx(0.01221965213583509, abs=1e-9)
    assert s['belt_angle_deg'] == pytest.approx(12.23332956521795, abs=1e-6)
    # Slider end 203.6 mm vs rail end 180 mm at the CAD pose: reported, not hidden.
    assert m.state(0.)['slider_overhang_past_rail_end_m'] == pytest.approx(0.0236)


def test_group_motions_keep_the_chain_closed(synthetic):
    m = LiftMechanism(*synthetic)
    for h in (-0.25, -0.1, 0., 0.05):
        s = m.state(h)
        upper = m.transform('BELT_INCLINED', h) @ np.r_[m.upper, 1.]
        np.testing.assert_allclose(upper[:3], m.upper + [0., 0., s['slider_shift_m']], atol=1e-12)
        lower = m.transform('BELT_INCLINED', h) @ np.r_[m.lower, 1.]
        np.testing.assert_allclose(lower[:3], m.lower + [0., h, 0.], atol=1e-12)
        np.testing.assert_allclose(m.transform('LIFT_TOP', h)[:3, 3], [0., h, 0.])
        np.testing.assert_allclose(m.transform('FIXED', h), np.eye(4))
    with pytest.raises(ValueError, match='SITE_MECHANISM_LIFT_PLATES_OVERLAP'):
        m.state(-0.33)


def test_boxes_exclude_duplicates_and_bound_the_interior(synthetic):
    m = LiftMechanism(*synthetic)
    boxes = {b['id']: b for b in m.boxes(-0.1)}
    assert 'p010' not in boxes and 'p011' not in boxes and 'p006' not in boxes
    env = boxes['lift_interior_envelope']
    centre, size = np.array(env['pose'])[:3, 3], np.array(env['size'])
    np.testing.assert_allclose(centre - size / 2, [-0.1945, 0.0408, -0.870], atol=1e-12)
    np.testing.assert_allclose(centre + size / 2, [0.3554, 0.2668, -0.830], atol=1e-12)
    np.testing.assert_allclose(np.array(boxes['p005']['pose'])[:3, 3], [0.0855, 0.2718, -0.960], atol=1e-12)


def test_machine_lift_range_from_user_overall_height(synthetic):
    m = LiftMechanism(*synthetic)
    # Overall height 160-300 mm (base plate bottom to top plate top); CAD 346 mm.
    assert m.overall_height_cad_m == pytest.approx(0.346)
    assert m.range_h_m == pytest.approx([-0.186, -0.046])
    lo, hi = m.state(-0.186), m.state(-0.046)
    assert lo['belt_angle_deg'] == pytest.approx(26.86979515134415, abs=1e-6)
    assert lo['slider_shift_m'] == pytest.approx(-0.07159142437766064, abs=1e-9)
    assert hi['belt_angle_deg'] == pytest.approx(18.032077779097367, abs=1e-6)
    assert hi['slider_shift_m'] == pytest.approx(-0.01374192969517507, abs=1e-9)
    manifest, roles = synthetic
    roles = dict(roles, mechanism=dict(roles['mechanism'], lift_range=dict(roles['mechanism']['lift_range'], basis='')))
    with pytest.raises(ValueError, match='SITE_MECHANISM_LIFT_RANGE_REQUIRED'):
        LiftMechanism(manifest, roles)


def test_lift_state_requires_field_h_and_only_tightens_the_machine_range(synthetic):
    m = LiftMechanism(*synthetic)
    with pytest.raises(ValueError, match='SITE_LIFT_STATE_REQUIRED'):
        lift_state({}, m)
    good = dict(lift=dict(h_m=-0.1, h_basis='operator entry'))
    state = lift_state(good, m)
    assert state['range_m'] == pytest.approx([-0.186, -0.046]) and state['range_basis'].startswith('USER_2026_11_21')
    tight = dict(lift=dict(good['lift'], range_m=[-0.15, -0.05], range_basis='field tighter'))
    assert lift_state(tight, m)['range_m'] == [-0.15, -0.05]
    for bad, code in [(dict(h_m=-0.03), 'SITE_LIFT_H_OUTSIDE_CONFIGURED_RANGE'), (dict(h_basis=''), 'SITE_LIFT_BASIS_REQUIRED'),
                      (dict(range_m=[-0.2, -0.05], range_basis='wider'), 'SITE_LIFT_RANGE_WIDENS_MACHINE_RANGE'),
                      (dict(range_m=[-0.05, -0.15], range_basis='x'), 'SITE_LIFT_RANGE_INVALID'),
                      (dict(range_m=[-0.15, -0.05]), 'SITE_LIFT_BASIS_REQUIRED')]:
        with pytest.raises(ValueError, match=code):
            lift_state(dict(lift={**good['lift'], **bad}), m)


@pytest.mark.skipif(not REAL.exists(), reason='private site layout manifest not converted here')
def test_real_site_manifest_literal_mechanism():
    manifest, roles = load_manifest(REAL), load_roles(ROLES)
    m = LiftMechanism(manifest, roles)
    roles_count = {}
    for v in m.classification.values():
        roles_count[v['role']] = roles_count.get(v['role'], 0) + 1
    assert roles_count == dict(LIFT_INTERIOR=407, FIXED=74, LIFT_TOP=71, CARTON=48, TOOL=43, BELT_INCLINED=27,
                               BELT_SLIDER=20, ROBOT_LINK=9, ROBOT_CAMERA_MAST=3)
    assert m.length_m == pytest.approx(0.983, abs=1e-6)
    assert np.degrees(m.angle0_rad) == pytest.approx(15.230, abs=1e-3)
    assert m.base_top_y == pytest.approx(0.0408, abs=1e-4) and m.top_under_y == pytest.approx(0.3668, abs=1e-4)
    assert m.rail_z == pytest.approx((-0.320, 0.180), abs=1e-4)
    assert m.range_h_m == pytest.approx([-0.186, -0.046], abs=1e-6)
    check_range(m, dict(range_m=m.range_h_m))


def test_cad_robot_base_maps_cad_up_to_base_up_and_stack_to_minus_x():
    from unloading_sim.eco65.site_mechanism import cad_robot_base
    T = cad_robot_base(load_roles(ROLES))
    np.testing.assert_allclose(T[:3, :3] @ [0, 1, 0], [0, 0, 1], atol=3e-4)
    np.testing.assert_allclose(T[:3, :3] @ [0, 0, -1], [-1, 0, 0], atol=3e-4)
    # CAD body '11' bottom box centre (mm -84.0, 678.0, -670.7) lands on the official base_link.STL
    # bottom box centre (x -0.055..0.0613, y -0.055..0.055, z 0) within 1 mm.
    np.testing.assert_allclose(T @ [-0.0840, 0.6780, -0.6707, 1.], [0.00315, 0, 0, 1], atol=1e-3)


@pytest.mark.skipif(not REAL.exists(), reason='private site layout manifest not converted here')
def test_real_site_loader_uses_roles_and_field_lift_state():
    from unloading_sim.workbench.single_carton.site_layout import load_layout
    from unloading_sim.eco65.site_mechanism import cad_robot_base
    T = cad_robot_base(load_roles(ROLES))
    c = dict(site_layout=dict(manifest=str(REAL.relative_to(ROOT)), roles=str(ROLES.relative_to(ROOT)),
                              T_base_layout=T.tolist(), transform_basis='TEST_CAD_FIT', excluded_part_ids=[],
                              exclusion_basis='TEST_ROLES_ONLY', association_distance_m=.05, association_basis='TEST_50_MM',
                              lift=dict(h_m=-0.1, h_basis='TEST')))
    loaded = load_layout(c, ROOT)
    roles = {}
    for o in loaded['world']:
        roles[o['role']] = roles.get(o['role'], 0) + 1
    assert roles == dict(FIXED=74, LIFT_TOP=71, BELT_INCLINED=27, BELT_SLIDER=20, LIFT_INTERIOR=1)
    top = next(o for o in loaded['world'] if o['name'] == 'YLYJ-JC00-07_顶板')
    # Top plate centre CAD Y 371.8 mm lowered by 100 mm -> base z = 0.2718 - 0.6779 = -0.4061 m.
    assert np.asarray(top['pose'])[2, 3] == pytest.approx(-0.4061, abs=1e-3)
    assert loaded['identity']['mechanism']['state']['belt_angle_deg'] == pytest.approx(21.37, abs=0.01)
    del c['site_layout']['lift']
    with pytest.raises(ValueError, match='SITE_LIFT_STATE_REQUIRED'):
        load_layout(c, ROOT)


@pytest.mark.skipif(not REAL.exists(), reason='private site layout manifest not converted here')
def test_real_site_simulation_scene_shares_the_layout_and_adds_cartons():
    from unloading_sim.eco65.site_layout_scene import build_site_scene
    s = build_site_scene()
    assert s['execution_mode'] == 'simulation' and s['enable_hardware'] is False
    assert len(s['world_objects']) == 193 and len(s['cartons']) == 48
    assert s['excluded_roles'] == dict(CARTON=48, ROBOT_LINK=9, ROBOT_CAMERA_MAST=3, TOOL=43)
    assert all(c['size_m'] == pytest.approx([0.2, 0.2, 0.2], abs=1e-3) for c in s['cartons'])
    # Highest, nearest carton centre in base_link (CAD stack lies towards base -X).
    np.testing.assert_allclose(np.asarray(s['cartons'][0]['pose'])[:3, 3], [-0.561, 0.315, 0.746], atol=2e-3)
    assert s['lift']['h_m'] == -0.046 and s['lift']['range_m'] == pytest.approx([-0.186, -0.046], abs=1e-6)
    cfg = dict(schema='eco65_site_layout_sim_v1', execution_mode='simulation', manifest=str(REAL.relative_to(ROOT)),
               roles=str(ROLES.relative_to(ROOT)), carton_mass_kg=2., lift=dict(h_m=0., h_basis='CAD pose'))
    with pytest.raises(ValueError, match='SIM_SITE_LIFT_H_OUTSIDE_MACHINE_RANGE'):
        build_site_scene(cfg)
    with pytest.raises(ValueError, match='SIM_SITE_LIFT_H_BASIS_REQUIRED'):
        build_site_scene(dict(schema='eco65_site_layout_sim_v1', execution_mode='simulation', manifest=str(REAL.relative_to(ROOT)),
                              roles=str(ROLES.relative_to(ROOT)), carton_mass_kg=2., lift=dict(h_m=0.)))


def test_interval_boxes_contain_every_intermediate_pose(synthetic):
    m = LiftMechanism(*synthetic)
    swept = {b['id']: b for b in m.boxes(-0.12, -0.08)}
    nominal = {b['id']: b for b in m.boxes(-0.1)}
    # Pure translation: the 10 mm top plate grows exactly by the 40 mm interval.
    assert swept['p005']['size'] == pytest.approx([0.560, 0.050, 0.300])
    assert np.array(swept['p005']['pose'])[1, 3] == pytest.approx(np.array(nominal['p005']['pose'])[1, 3])
    for pid in ('p009', 'p008', 'p005'):
        lo, hi = np.array(next(p for p in synthetic[0]['parts'] if p['id'] == pid)['collision_bounds_m'])
        corners = (lo + hi) / 2 + np.array([[i, j, k] for i in (-.5, .5) for j in (-.5, .5) for k in (-.5, .5)]) * (hi - lo)
        inv, half = np.linalg.inv(np.array(swept[pid]['pose'])), np.array(swept[pid]['size']) / 2
        role = swept[pid]['role']
        for h in np.linspace(-0.12, -0.08, 801):
            local = (inv @ m.transform(role, h) @ np.c_[corners, np.ones(8)].T).T[:, :3]
            assert np.all(np.abs(local) <= half + 1e-12)
    env = swept['lift_interior_envelope']
    assert np.array(env['pose'])[1, 3] + env['size'][1] / 2 == pytest.approx(0.3668 - 0.08)
