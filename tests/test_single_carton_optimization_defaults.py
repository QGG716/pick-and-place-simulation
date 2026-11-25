"""Optimisation switches default ON in code (user decision 2026-11-25)."""
import json
from pathlib import Path

from unloading_sim.workbench.single_carton import optimization_defaults as D
from unloading_sim.workbench.single_carton.config import load, issues

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT/'configs/workbench/single_carton.example.json'


def test_example_gets_every_switch_and_stays_valid():
    c = load(EXAMPLE)
    assert issues(c) == []
    applied = set(c['code_defaults']['applied'])
    for key in ('parallel_screening', 'defer_pilz_solution_check', 'screen_approach_path', 'cross_cycle_inheritance',
                'contact_failure_next_branch', 'pilz_sent_path_ompl_fallback', 'audit_chunking',
                'contact_search.single_contact', 'neighbor_lip_contact', 'geometry.resting_contact_policy'):
        assert key in applied, key
    assert c['sent_path_audit'] == 'ADAPTIVE_CONTACT' and c['collision_check_mode'] == 'TWO_LEVEL'
    assert c['contact_search']['single_contact']['vertical_placement'] == 'LOWER_TWO_ROWS_ON_FACE'
    assert c['neighbor_lip_contact']['cup_body_basis'] and c['neighbor_lip_contact']['observed_support_basis']
    # The example keeps its explicit numbers; reserved audit workers follow its pool (6 + 1 - 3).
    assert c['native_check_workers'] == 6 and c['pipelined_audit_workers'] == 4


def test_explicit_values_win():
    c = json.loads(EXAMPLE.read_text())
    c.update(parallel_screening=False, defer_pilz_solution_check=False, sent_path_audit='SAMPLED')
    c['geometry']['coplanar_seam_policy'] = {'mode': 'DISABLED'}
    D.apply(c)
    assert c['parallel_screening'] is False and c['sent_path_audit'] == 'SAMPLED'
    assert c['geometry']['coplanar_seam_policy'] == {'mode': 'DISABLED'}
    # Dependent switches are not forced on against an explicit prerequisite.
    assert 'pilz_sent_path_ompl_fallback' not in c
    assert 'adaptive_constraints' not in c and 'adaptive_rule_transforms' not in c


def test_native_planning_keeps_previous_defaults():
    c = json.loads(EXAMPLE.read_text()); c['planning_module'] = 'NATIVE'
    assert D.apply(c) == [] and 'parallel_screening' not in c
    assert c['code_defaults']['skipped'] == 'PLANNING_MODULE_NATIVE_KEEPS_PREVIOUS_DEFAULTS'


def test_debug_and_check_premises_are_not_defaults():
    c = json.loads(EXAMPLE.read_text()); D.apply(c)
    for key in ('debug_near_target_obstacles', 'dry_run_no_vacuum', 'cycle_structure'):
        assert key not in c
    assert c.get('holding_capacity', {}).get('check_mode', 'ENFORCED') == 'ENFORCED'


# target_cup_approach_contact (user decision 2026-11-25, "吸盘是柔性的，所以只记录，不拒绝").
from test_single_carton_planner import case  # noqa: F401,E402  (fixture reuse)
from unloading_sim.workbench.single_carton.geometry import contact_candidates  # noqa: E402

TARGET_CUP = dict(mode='CUPS_MAY_TOUCH_TARGET_DURING_APPROACH_RECORD_ONLY', basis='TEST')


def test_target_cup_config_validation():
    c = load(EXAMPLE)
    assert c['target_cup_approach_contact']['mode'] == TARGET_CUP['mode'] and issues(c) == []
    for bad in (dict(mode='CUPS_MAY_TOUCH_TARGET_DURING_APPROACH_RECORD_ONLY'), dict(TARGET_CUP, extra=1), dict(mode='ON', basis='x')):
        c['target_cup_approach_contact'] = bad
        assert any(e.startswith('target_cup_approach_contact') for e in issues(c))
    c['target_cup_approach_contact'] = TARGET_CUP; c['planning_module'] = 'NATIVE'
    assert any('target_cup_approach_contact 仅' in e for e in issues(c))


def test_target_cup_pairs_only_in_the_approach_stage(case):
    p, obs, box, folder, calls, opts = case
    p.site['target_cup_approach_contact'] = TARGET_CUP
    lips = p.contact_geometry['terminal_band_by_contour']
    p.contact_geometry['connected_cup_ids'] = list(lips)
    candidate = contact_candidates(box, p.source['tool'], p.site['topology'])[0]
    q = [0.]*6
    approach = p.context(q, p.world, box, candidate, 'approach')
    expected = [[box['id'], lips[i]] for i in lips]
    assert approach['allowed_pairs'][-len(expected):] == expected
    assert approach['single_carton_contact']['target_cup_approach_contact'] == dict(basis='TEST')
    contact = p.context(q, p.world, box, candidate, 'contact')
    assert 'target_cup_approach_contact' not in contact['single_carton_contact']
    assert not any(pair in contact['allowed_pairs'] for pair in expected if pair[1] not in
                   [lips[i] for i in candidate['target_lip_contact_ids']])
    # With cup bodies every body joins after the lips.
    p.contact_geometry['cup_body_by_contour'] = {i: lips[i]+'_body' for i in lips}
    bodies = p.context(q, p.world, box, candidate, 'approach')['allowed_pairs']
    assert bodies[-len(lips):] == [[box['id'], lips[i]+'_body'] for i in lips]
    p.site.pop('target_cup_approach_contact')
    assert 'target_cup_approach_contact' not in p.context(q, p.world, box, candidate, 'approach')['single_carton_contact']
