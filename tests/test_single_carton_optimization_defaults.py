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
