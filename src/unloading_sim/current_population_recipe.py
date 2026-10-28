"""Latest explicitly selected recipes, applied to a fresh frozen population.

Only the existing bottom-layer grants are combined here. No old trajectory,
solver state, completed carton, or joint-rate diagnostic is imported.
"""
import math

SCOPE = 'CURRENT_RECIPES_NEW_WORLD_40'
LAYOUT = 'm710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100_min250_lowprofile_tool_extension200.yaml'
PREPARATION = 'CHECKED_UNLOADED_LOWERING_BEFORE_FIXED_RECEIVER_CYCLE_V1'


def recipe_permission(frozen):
    from .low_top_receiver import selected_layer_top_permission, BOX_TOP_LOW_DEFINITION
    from .l01_hmax import layer_permission
    from .bottom_rotated import rotation_permission
    evidence = frozen.evidence()
    if evidence['original_carton_count'] != 40:
        raise ValueError('CURRENT_RECIPE_REQUIRES_FROZEN_40')
    return dict(scope=SCOPE, original_cartons_sha256=evidence['original_cartons_sha256'],
                low_definition=BOX_TOP_LOW_DEFINITION, minimum_height_m=.25,
                bottom_top_permission=selected_layer_top_permission(frozen, 'carton_l01_c02'),
                l01_hmax_permission=layer_permission(frozen),
                rotation_permission=rotation_permission(frozen),
                c04_face_order=['z+', 'y-', 'x-'],
                include_receive_before_approach=True, all_joint_rate_limits_retained=True)


def preparation(option, initial_height, target):
    height = float(option['height_m'])
    if not math.isfinite(initial_height) or initial_height < height-1e-9 or abs(height-.25)>1e-9:
        raise ValueError('FIXED_RECEIVER_PREPARATION_HEIGHT')
    from .lifted_reception import StartHeight
    return dict(schema=PREPARATION, initial_height_m=float(initial_height), cycle_height_m=height,
                target_id=target.name, target_identity=StartHeight.capture(target).box_identity,
                timing='BEFORE_PREGRASP', robot_motion='STATIONARY', native_check_required=True)


def fixed_cycle_height(option, root_height, target=None):
    """Validate a planned lowering; this is not a native sweep acceptance."""
    grant = option.get('fixed_receiver_preparation')
    if grant is None:
        return root_height
    if (grant.get('schema') != PREPARATION or grant.get('initial_height_m') != root_height
            or grant.get('cycle_height_m') != option['height_m']
            or option.get('pre_approach_height_m') != option['height_m']
            or grant.get('timing') != 'BEFORE_PREGRASP' or grant.get('robot_motion') != 'STATIONARY'
            or grant.get('native_check_required') is not True
            or not math.isfinite(root_height) or root_height < option['height_m']-1e-9
            or abs(option['height_m']-.25)>1e-9):
        raise ValueError('FIXED_RECEIVER_PREPARATION_CHANGED')
    if target is not None and grant != preparation(option, root_height, target):
        raise ValueError('FIXED_RECEIVER_PREPARATION_TARGET_CHANGED')
    return option['height_m']


def validate_preparation_binding(option, root, binding):
    if 'fixed_receiver_preparation' not in option:
        return
    fixed_cycle_height(option, root['height_m'])
    pre = binding.get('pre_approach_transition')
    if (not pre or pre['target_id'] != option['fixed_receiver_preparation']['target_id']
            or pre['samples'][0]['height_m'] != root['height_m']
            or pre['samples'][-1]['height_m'] != option['height_m']
            or pre['samples'][-1]['world'] != binding['samples'][0]['world']
            or binding['samples'][0]['world'] != binding['samples'][-1]['world']
            or pre['maximum_height_step_m'] > .010000001 or pre['continuous_swept_proof']):
        raise ValueError('FIXED_RECEIVER_NATIVE_BINDING_MISSING_OR_CHANGED')


def current_options(permission, target, frozen, layout, cartons, current_height):
    """Return None for unchanged upper recipes; explicit options for bottom two layers."""
    if permission != recipe_permission(frozen):
        raise ValueError('CURRENT_RECIPE_PERMISSION_CHANGED')
    top = permission['bottom_top_permission']
    if target.name not in top['allowed_target_ids']:
        return None, None, None, None
    from .conveyor_lift import coupled_conveyor_state
    from .low_top_receiver import layer_top_option
    from .l01_hmax import options, FACES, c04_side_permission
    world = {b.name: b for b in coupled_conveyor_state(layout, .25, cartons).boxes}
    belt, longitudinal = world['conveyor_transverse'], world['conveyor_longitudinal']
    hmax = permission['l01_hmax_permission'] if target.name in FACES else None
    if hmax:
        choices = options(target, belt, longitudinal, frozen, .25, hmax)
        if target.name == 'carton_l01_c04':
            side = options(target, belt, longitudinal, frozen, .25, hmax,
                           c04_side=c04_side_permission(frozen))
            choices = [choices[1], *side, choices[0]]
    else:
        choices = [layer_top_option(target, belt, longitudinal, frozen, .25, top)]
    for option in choices:
        option['fixed_receiver_preparation'] = preparation(option, current_height, target)
    rotation = permission['rotation_permission'] if target.name == permission['rotation_permission']['target_id'] else None
    return choices, top if hmax is None else None, hmax, rotation
