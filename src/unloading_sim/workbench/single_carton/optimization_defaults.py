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
    # Same functions and thresholds, mask-independent part computed beside
    # detection/SAM (switch of 2026-11-25; defaulted 2026-11-26 with the
    # request "构建当前帧同构仿真环境那里要提速").
    'scene_build': 'SPLIT_PARALLEL',
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
    'front_branch_first': {'mode': 'PLAN_BLOCKED_FRONT_BRANCH_FIRST', 'basis': BASIS+' (2026-11-26 "先规划朝前分支")'},
    'approach_via_point': {'mode': 'J1_AND_WRIST_FIRST', 'basis': BASIS+' (2026-11-26 "加入流程"; offline 7b07fd06 via test)'},
    'target_cup_approach_contact': {'mode': 'CUPS_MAY_TOUCH_TARGET_DURING_APPROACH_RECORD_ONLY',
                                    'basis': BASIS+' ("吸盘是柔性的，所以只记录，不拒绝")'},
    'mast_contact': {'mode': 'ROBOT_MAST_CONTACT_RECORD_ONLY',
                     'basis': BASIS+' ("所有与视觉桅杆的碰撞都只记录，不否决"; depth bound declined: "全部只记录，不设上限")'},
    'passive_cup_neighbour': {'mode': 'PREFER_PASSIVE_CUPS_OFF_NEIGHBOURS', 'basis': BASIS,
                              'toward_tool_m': 0.02, 'into_face_m': 0.005},
    # Implementer's extension 2026-11-28 of the user decisions "加 OMPL 回退" (2026-11-25)
    # and approach_via_point "加入流程" (2026-11-26) to the loaded return: with straight
    # front detach only there is no other detach profile left after a rejected Pilz return.
    'loaded_return_retry': {'mode': 'VIA_THEN_OMPL_AFTER_PILZ_SENT_PATH_REJECTED',
                            'basis': 'CODE_DEFAULT_IMPLEMENTER_2026_11_28_PENDING_USER_REVIEW: a rejected Pilz loaded return is '
                                     'planned once more, mirrored approach via points first ("加入流程"), else OMPL ("加 OMPL 回退"); '
                                     'same goal, budget and full audit'},
    # Implementer's choice 2026-11-28 pending user review: the tool-down release
    # IK branch nearest the target's grasp branch (no wrist flip on the loaded return).
    'release_branch': {'mode': 'NEAREST_GRASP_BRANCH',
                       'basis': 'CODE_DEFAULT_IMPLEMENTER_2026_11_28_PENDING_USER_REVIEW: with the standing array the posture-first '
                                'release branch flipped the wrist (J5 -90 / J6 +91 against the grasp J5 +79 / J6 -90) and every '
                                'layer-12 loaded return folded the tool into link_3; same release pose, least raw joint travel'},
    # User decision 2026-11-28: "正吸的脱离方案只能是走直线".
    'detach_profile_policy': {'mode': 'FRONT_FACE_STRAIGHT_OUTWARD_ONLY', 'max_normal_from_horizontal_deg': 45.0,
                              'basis': 'CODE_DEFAULT_USER_DECISION_2026_11_28: "正吸的脱离方案只能是走直线" (front-face grips: '
                                       'only the OUTWARD straight LIN along the face normal; top-face grips unchanged)'},
}

# User decision 2026-11-28 (face-unloading review): "吸具没有对吸盘进行分区，而吸具横着
# 的面积会大于单个箱子的正面 ... 要么一次吸两箱，要么把吸具竖起来吸一箱（即吸具的长边平行
# 于z轴）" - one carton, so the array stands up (two 2 kg cartons + 1.5 kg tool would
# exceed the 5 kg rating); "在吸某一箱时 ... 会尝试很多个吸取点，我觉得这样很耗时" and
# "吸取点太靠右，可以适当往左挪一点" - one sideways placement toward the robot instead
# of the 2026-11-26 cup-grid shifts. Supersedes the 2026-11-25 "吸盘长边平行于Y轴"
# and the 2026-11-26 CUP_GRID default; both stay selectable in a configuration.
BASIS_1128 = 'CODE_DEFAULT_USER_DECISION_2026_11_28'
SINGLE_CONTACT = {
    'mode': 'FACE_CENTER_LONG_SIDE_ALONG_BASE_Z', 'long_side_toward': '+Z', 'max_normal_from_x_deg': 45.0,
    'basis': BASIS_1128+': "把吸具竖起来吸一箱（即吸具的长边平行于z轴）" (the array is wider than one carton front and the '
                        'common valve has no zones: "要么一次吸两箱"); one contact, no lift or search variants (2026-11-25 '
                        '"吸盘就只要一种选项", "不用再搜32个点了")',
    'vertical_placement': 'LOWER_TWO_ROWS_ON_FACE',
    'vertical_placement_basis': BASIS+' ("用下两排吸盘来吸，最上面一排吸盘空着"); standing array: the two lowest of its four rows '
                                      'on the face, the upper rows above the carton top',
    'in_face_shift': 'LATERAL_TOWARD_ROBOT_MAX',
    'in_face_shift_basis': BASIS_1128+': "在吸某一箱时 ... 会尝试很多个吸取点，我觉得这样很耗时"; "吸取点太靠右，可以适当往左挪一点" '
                                      '(one sideways placement: most anticipated seals, then the TCP nearest the J1 axis, checked '
                                      'every 1 mm; the cup-column grid alone left C5 22.5 mm over and link_3 hit the side panel)',
}
CONTACT_SEARCH = {'reference_face_roll_deg': 90, 'j6_rotation_deg': 180, 'lift_offsets_m': [0.0],
                  'max_candidates': 1, 'precontact_m': 0.06, 'basis': BASIS, 'single_contact': SINGLE_CONTACT}

POSTURE = {'wrist_down_weight': 2.0, 'tolerance_deg': {'upper_up': 20.0, 'forearm_horizontal': 20.0, 'wrist_down': 20.0},
           # User decision 2026-11-25 ("降为次要"): the J4->J5 term only orders
           # branches whose arm scores are within the band (implementer's 0.05).
           'wrist_down_priority': 'SECONDARY', 'wrist_down_secondary_band': 0.05,
           # User decision 2026-11-25 ("加上朝向偏好"): reach-over branches last;
           # 2026-11-26 ("取消掉朝后分支"): reach-over branches are not tried.
           'arm_facing': 'FRONT_ONLY',
           # User decision 2026-11-25 ("不好的逆解为什么要尝试"; "偏竖直 >45° 不尝试").
           'upper_arm_max_from_vertical_deg': 45.0}

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
    'single_face_depth_policy': {'mode': 'BOUND_BY_MAX_CARTON_EXTENT', 'basis': BASIS,
                                 'thin_second_plane': 'AS_SINGLE_FACE',
                                 'thin_second_plane_basis': BASIS+' ("改成按单面处理", field 1b7de01d)'},
    'coplanar_continuation_policy': {'mode': 'EXTEND_ENVELOPE_TO_NEXT_BOUNDARY', 'basis': BASIS},
    'non_interpenetration_policy': {'mode': 'OTHER_INSTANCE_SURFACES_CUT_ENVELOPE', 'basis': BASIS},
    'resting_contact_policy': {'mode': 'GENERAL_RESTING_CONTACT', 'basis': BASIS,
                               'lower_top_beside': 'SUPPORT_SURFACE', 'lower_top_beside_basis': BASIS},
    'own_mask_claim_policy': {'mode': 'OWN_MASK_FIRST_BRIDGE_NO_RETURN', 'basis': BASIS},
    'adjacent_contact_policy': {'mode': 'LATERAL_CONTACT_PLANES', 'max_gap_m': 0.01, 'basis': BASIS},
    'edge_mixed_pixel_policy': {'mode': 'REMOVE_INTERPOLATED_SILHOUETTE_RETURNS', 'basis': BASIS},
    # User decision 2026-11-26 ("只要...能看到完整的一个面...都让该箱能进入运动规划流程";
    # overlap option "重叠部分都忽略"): see rest_overlap.py.
    'unresolved_rest_overlap_policy': {'mode': 'IGNORE_OVERLAP_PAYLOAD_DETACH',
                                       'basis': 'CODE_DEFAULT_USER_DECISION_2026_11_26: "重叠部分都忽略" (unresolved volumes at '
                                                'the target rest pose: overlap ignored, rest bound as payload-only detach contacts)'},
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
