"""One explicitly requested scene assumption, separate from accepted cycles."""
from copy import deepcopy
from .moveit2_backend import box_message
from .support import SupportRelationGraph

TARGET='carton_l05_c03'
SCOPE='CURRENT_RECIPES_ASSUMED_L05_REMOVAL_CONTINUATION'


def assume_l05_removed(state):
    before=deepcopy(state.root())
    cartons={b.name:b for b in state.cartons}
    if (state.assumed_removed or TARGET not in cartons or TARGET in before['completed']
            or len(state.cartons)!=16 or len(state.ledger)!=24 or state.cycles):
        raise ValueError('ASSUMPTION_REQUIRES_ACCEPTED_24_BOX_TERMINAL_ROOT')
    if SupportRelationGraph.build(state.cartons).supports[TARGET]:
        raise ValueError('ASSUMED_REMOVAL_SUPPORTS_ANOTHER_CARTON')
    box=box_message(cartons[TARGET])
    state.cartons=tuple(b for b in state.cartons if b.name!=TARGET)
    state.assumed_removed=(TARGET,)
    return dict(event_type='USER_ASSUMED_REMOVAL',target_id=TARGET,box=box,
        source='EXPLICIT_USER_INSTRUCTION',counted_as_planning_success=False,
        physical_execution='NOT_RUN',motion_generated=False,robot_and_receiver_unchanged=True,
        before=before,after=deepcopy(state.root()))


def validate_assumption(event,original,initial,historical_ledger):
    before=event['before'];target=event['target_id']
    assert event['event_type']=='USER_ASSUMED_REMOVAL' and target==TARGET
    assert event['source']=='EXPLICIT_USER_INSTRUCTION' and not event['counted_as_planning_success']
    assert event['physical_execution']=='NOT_RUN' and not event['motion_generated']
    assert event['robot_and_receiver_unchanged']
    assert len(before['completed'])==24 and len(before['world_cartons'])==16
    assert before['predecessor_final_state_receipt'] is None and 'assumed_removed' not in before
    assert before['completed']==[e['target_id'] for e in historical_ledger]
    assert TARGET not in before['completed'] and event['box']==original[TARGET]
    assert {b['id']:b for b in before['world_cartons']}=={n:b for n,b in original.items() if n not in before['completed']}
    expected=dict(before,world_cartons=[b for b in before['world_cartons'] if b['id']!=TARGET],assumed_removed=[TARGET])
    assert initial==event['after']==expected
