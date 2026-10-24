"""Four requested bottom cartons from the unchanged accepted 34-carton root."""
from unloading_sim.low_top_receiver import selected_layer_top_permission

SCOPE = 'XY_TOP_L00_FOUR_TOP_EXTENSION200'
ORDER = ('carton_l00_c00', 'carton_l00_c01', 'carton_l00_c02', 'carton_l00_c03')
RETAINED = ('carton_l00_c04', 'carton_l01_c04')


def bind_scope(frozen, state):
    permission = selected_layer_top_permission(frozen, ORDER[0])
    if (abs(permission['ceiling_world_m']-.3) > 1e-9
            or set(permission['allowed_target_ids']) != set(ORDER) | {RETAINED[0]}
            or {b.name for b in state.cartons} != set(ORDER) | set(RETAINED)
            or len(state.ledger) != 34 or abs(state.height-.25) > 1e-9):
        raise ValueError('L00_FOUR_TOP_ROOT_CHANGED')
    from unloading_sim.lifted_reception import StartHeight
    for box in state.cartons:
        if StartHeight.capture(box).box_identity != frozen.evidence()['heights'][box.name]['start_box_identity']:
            raise ValueError('L00_FOUR_TOP_FROZEN_GEOMETRY_CHANGED')
    return permission
