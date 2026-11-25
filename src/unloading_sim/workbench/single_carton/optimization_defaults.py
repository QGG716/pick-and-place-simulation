"""Code defaults of the optimisation switches (user decision 2026-11-25).

User: "所有的优化在代码里都把开关打开，而不是做到现场配置里". Every planning,
candidate-search, collision-criterion and vision switch that was added default
off since 2026-11-16 now defaults ON in the PYTHON planning flow. ``apply``
fills only keys a configuration does not set: an explicit value in a file,
including an explicit ``mode: DISABLED``, still wins. ``planning_module=NATIVE``
keeps the previous defaults, because these switches are PYTHON-flow only and
NATIVE fails closed on them.

Not optimisations, so not defaulted here (they stay explicit per configuration):
debug_near_target_obstacles, dry_run_no_vacuum, holding_capacity.check_mode,
start_joint_check, pre_execute_visual_recheck, execution_lock, cycle_structure,
confirmed_stack_support, wrist_branch_limits, tool_robot_clearance,
planning_deadline_s, detector thresholds and site parameters such as
geometry.max_carton_extent_m or holding_capacity.cup_suction_force_n.

audit_chunking=WORK_QUEUE was measured as no gain on 2026-11-23; the 2026-11-25
prewarmed A/B on 38ab81b1 (single contact) gave 2.41/2.41 s against SUBSETS
2.43/2.51/2.65 s, so it is on like the rest.
"""
from copy import deepcopy

BASIS = 'CODE_DEFAULT_USER_DECISION_2026_11_25: "所有的优化在代码里都把开关打开，而不是做到现场配置里"'

# Scheduling / identical-decision speed mechanisms (2026-11-16 .. 11-23).
PLANNING = {
    'resident_workers': True, 'scene_handles': True, 'async_evidence': True,
    'path_endpoint_reuse': True, 'binding_hash_cache': True,
    'native_check_workers': 12, 'parallel_candidate_lanes': 4,
    'collision_check_mode': 'TWO_LEVEL', 'sent_path_audit': 'ADAPTIVE_CONTACT',
    'parallel_screening': True, 'lane_clients': 3, 'concurrent_audits': True,
    'sent_path_probe': 'GEOMETRIC', 'solution_path_check': 'CLEARANCE_PREDICATE',
    'native_ik_batch': True, 'compact_requests': True, 'binary_transport': True,
    'early_scene_upload': True, 'process_geometry_ref': True, 'native_lin_ladder': True,
    'structured_plan_digest': True, 'binary_plan_record': True, 'prepare_processes': 4,
    'early_holding_postcheck': True, 'adaptive_constraints': 'PER_BODY',
    'workspace_pair_cache': True, 'python_switch_interval_s': 0.0005,
    'defer_pilz_solution_check': True, 'deadline_ompl_fallback': 'NOT_STARTED_WHEN_SHORT',
    'digest_memo': True, 'analytic_send_slowdown': True,
    'adaptive_rule_transforms': 'LINK_TRANSFORMS_ONLY', 'audit_chunking': 'WORK_QUEUE',
    'lin_time_parametrization': {'mode': 'PATH_TOPP', 'joint_limit_fraction': 0.95,
                                 'basis': BASIS+' (user decision D 2026-11-23)'},
    'native_ompl_budget_s': 5.0,
    'native_ompl_budget_basis': BASIS+' (user decision 2026-11-21 "降到 5 s")',
}

# Candidate search (user decisions 2026-11-25).
CANDIDATES = {
    'candidate_family_order': {'mode': 'INTERLEAVE_LIFT_SIBLINGS', 'basis': BASIS},
    'ik_winding_twins': {'mode': 'AFTER_DISTINCT_CONFIGURATIONS', 'basis': BASIS},
    'screen_approach_path': {'mode': 'APPROACH_JOINT_LINE', 'basis': BASIS},
    'cross_cycle_inheritance': {'mode': 'IK_SEEDS_AND_FAMILY_FIRST', 'basis': BASIS, 'max_entries': 3,
                                'family_tolerance_rad': 0.5, 'max_q_start_deviation_deg': 1.0,
                                'store': 'cross-cycle-inheritance.json'},
    'pilz_sent_path_ompl_fallback': True,
    'pilz_sent_path_ompl_fallback_basis': BASIS+' ("加 OMPL 回退")',
    'contact_failure_next_branch': {'mode': 'NEXT_IK_BRANCH', 'basis': BASIS+' ("加上换分支")'},
    'passive_cup_neighbour': {'mode': 'PREFER_PASSIVE_CUPS_OFF_NEIGHBOURS', 'basis': BASIS,
                              'toward_tool_m': 0.02, 'into_face_m': 0.005},
}

SINGLE_CONTACT = {
    'mode': 'FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', 'long_side_toward': '+Y', 'max_normal_from_x_deg': 45.0,
    'basis': BASIS+' ("吸盘就只要一种选项（对准箱子待吸面的中心，吸盘长边平行于Y轴，短边平行于Z轴）"; '
                   '"不用再搜32个点了")',
    'vertical_placement': 'LOWER_TWO_ROWS_ON_FACE',
    'vertical_placement_basis': BASIS+' ("用下两排吸盘来吸，最上面一排吸盘空着")',
}
CONTACT_SEARCH = {'reference_face_roll_deg': 90, 'j6_rotation_deg': 180, 'lift_offsets_m': [0.0],
                  'max_candidates': 1, 'precontact_m': 0.06, 'basis': BASIS, 'single_contact': SINGLE_CONTACT}

POSTURE = {'wrist_down_weight': 2.0, 'tolerance_deg': {'upper_up': 20.0, 'forearm_horizontal': 20.0, 'wrist_down': 20.0}}

# Collision criteria for the flexible cups (user decisions 2026-11-24/25).
NEIGHBOR_LIP_CONTACT = {
    'mode': 'CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS', 'basis': BASIS+' (2026-11-24 flexible lips may touch neighbours)',
    'observed_support_basis': BASIS+' ("允许唇边碰下方箱")',
    'cup_body_basis': BASIS+' ("吸盘从唇边到最后到吸具的连接器，都是柔性对，刚性芯是不对的")',
}

# Vision policies (user decisions 2026-11-19 .. 11-24); site parameters stay in the configuration.
GEOMETRY = {
    'coplanar_seam_policy': {'mode': 'MASK_RGB_EDGE_COINCIDENT', 'uncertainty_m': 0.005, 'basis': BASIS},
    'stereo_shadow_boundary_policy': {'mode': 'INVALID_BAND_BEFORE_FARTHER_RETURN_IS_BACKGROUND', 'basis': BASIS},
    'rgb_color_boundary_policy': {'mode': 'CARTON_COLOUR_DIFFERS_FROM_SURROUNDINGS', 'uncertainty_m': 0.005, 'basis': BASIS},
    'single_face_depth_policy': {'mode': 'BOUND_BY_MAX_CARTON_EXTENT', 'basis': BASIS},
    'coplanar_continuation_policy': {'mode': 'EXTEND_ENVELOPE_TO_NEXT_BOUNDARY', 'basis': BASIS},
    'non_interpenetration_policy': {'mode': 'OTHER_INSTANCE_SURFACES_CUT_ENVELOPE', 'basis': BASIS},
    'resting_contact_policy': {'mode': 'GENERAL_RESTING_CONTACT', 'basis': BASIS,
                               'lower_top_beside': 'SUPPORT_SURFACE', 'lower_top_beside_basis': BASIS},
    'own_mask_claim_policy': {'mode': 'OWN_MASK_FIRST_BRIDGE_NO_RETURN', 'basis': BASIS},
    'adjacent_contact_policy': {'mode': 'LATERAL_CONTACT_PLANES', 'max_gap_m': 0.01, 'basis': BASIS},
    'edge_mixed_pixel_policy': {'mode': 'REMOVE_INTERPOLATED_SILHOUETTE_RETURNS', 'basis': BASIS},
}


def apply(c):
    """Fill the absent optimisation keys of a loaded configuration in place.

    Returns the list of dotted keys that came from these code defaults; the
    list is also stored as ``c['code_defaults']`` for the evidence records.
    """
    applied = []
    if c.get('planning_module', 'PYTHON') == 'NATIVE':
        c['code_defaults'] = dict(basis=BASIS, applied=[], skipped='PLANNING_MODULE_NATIVE_KEEPS_PREVIOUS_DEFAULTS')
        return applied

    def put(target, key, value, prefix=''):
        if key not in target:
            target[key] = deepcopy(value); applied.append(prefix+key)

    # A default never contradicts an explicit setting it depends on.
    requires = {
        'early_scene_upload': lambda: c.get('scene_handles'),
        'digest_memo': lambda: c.get('structured_plan_digest'),
        'adaptive_constraints': lambda: c.get('sent_path_audit') in ('ADAPTIVE', 'ADAPTIVE_CONTACT'),
        'adaptive_rule_transforms': lambda: c.get('sent_path_audit') in ('ADAPTIVE', 'ADAPTIVE_CONTACT'),
        'prepare_processes': lambda: c.get('prepare_implementation', 'PYTHON') != 'NATIVE_LIBRARY',
        'pilz_sent_path_ompl_fallback': lambda: c.get('defer_pilz_solution_check'),
        'pilz_sent_path_ompl_fallback_basis': lambda: c.get('pilz_sent_path_ompl_fallback'),
        'native_ompl_budget_basis': lambda: 'native_ompl_budget_s' in c,
        'audit_chunking': lambda: c.get('concurrent_audits'),
    }
    for table in (PLANNING, CANDIDATES):
        for key, value in table.items():
            if key in requires and not requires[key]():
                continue
            put(c, key, value)
    # Reserved audit workers: all but three of the pool (planning lane of the
    # single candidate), as in the field variant (13 processes, 10 reserved).
    pool = int(c.get('native_check_workers', 0)) + 1
    if 'pipelined_audits' not in c and pool >= 4:
        put(c, 'pipelined_audits', True); put(c, 'pipelined_audit_mode', 'RESERVED_WORKERS')
        put(c, 'pipelined_audit_workers', pool - 3)
    if c.get('contact_search') is None:
        c.pop('contact_search', None); put(c, 'contact_search', CONTACT_SEARCH)
    elif isinstance(c['contact_search'], dict):
        put(c['contact_search'], 'single_contact', SINGLE_CONTACT, 'contact_search.')
    posture = c.get('posture_preference')
    if isinstance(posture, dict) and posture.get('mode') == 'SOFT_LINK_DIRECTIONS':
        for key, value in POSTURE.items():
            put(posture, key, value, 'posture_preference.')
    lip = c.get('neighbor_lip_contact')
    if lip is None:
        put(c, 'neighbor_lip_contact', NEIGHBOR_LIP_CONTACT)
    elif isinstance(lip, dict) and lip.get('mode') == 'CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS':
        for key in ('observed_support_basis', 'cup_body_basis'):
            put(lip, key, NEIGHBOR_LIP_CONTACT[key], 'neighbor_lip_contact.')
    geometry = c.setdefault('geometry', {})
    for key, value in GEOMETRY.items():
        put(geometry, key, value, 'geometry.')
    c['code_defaults'] = dict(basis=BASIS, applied=list(applied))
    return applied
