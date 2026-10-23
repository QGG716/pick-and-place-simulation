"""User-approved l01-only planning diagnostic, bound to original layer geometry."""
from unloading_sim.carton_population import FrozenCartonPopulation

SCOPE = 'XY_TOP_LOWPROFILE250_L01_TOOL_CHASSIS'


def frozen_l01_scope(cartons):
    frozen = FrozenCartonPopulation.capture(cartons)
    heights = frozen.evidence()['heights']
    # Selection uses the original geometry, never an ID prefix or remaining boxes.
    ids = sorted(n for n, h in heights.items()
                 if abs(h['z_b_m']-.3) < 1e-9 and abs(h['z_b_m']+h['h_m']-.6) < 1e-9)
    if len(heights) != 40 or len(ids) != 5:
        raise ValueError('L01_ORIGINAL_LAYER_GEOMETRY_CHANGED')
    return dict(allowed_target_ids=ids, bottom_world_m=.3, top_world_m=.6,
                original_box_identities={n: heights[n]['start_box_identity'] for n in ids},
                planning_only=True, other_collision_and_process_checks_retained=True)
