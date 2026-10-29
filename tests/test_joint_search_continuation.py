from copy import deepcopy
import pytest
from plan_m710_joint_search import bind_best_continuation,continuation_active_nodes
from plan_m710_joint_search import coverage_order,continuation_coverage_order


def checkpoint():
    def node(name,parent,completed):
        return dict(id=name,parent=parent,root=dict(completed=list(range(completed)),
            world_cartons=[dict(id=i) for i in range(completed,40)],q_rad=[.1]*6,height_m=.25),
            children=[],closed=False,cursor=36,attempted=['old-failure'])
    nodes={name:node(name,parent,count) for name,parent,count in
        [('earlier',None,28),('accepted','earlier',29),('sibling','earlier',29),('next','accepted',30)]}
    return dict(best='fixed',layouts={'fixed':dict(nodes=nodes,best_node='accepted')},
        pending=None,trials=[dict(complete=False)],elapsed_search_s=5928.)


def test_continuation_preserves_current_world_and_failures_and_filters_older_branches():
    db=checkpoint();before=deepcopy(db);scope=bind_best_continuation(db,True)
    assert scope['accepted_prefix']==list(range(29)) and scope['prior_elapsed_search_s']==5928.
    assert scope['prior_trials']==1 and scope['failed_cartons_remain_in_world']
    assert db['layouts']==before['layouts'] and db['trials']==before['trials']
    assert [n['id'] for n in continuation_active_nodes(db['layouts']['fixed'],scope['anchor_node'])]==['accepted','next']


def test_resume_keeps_original_anchor_when_best_has_advanced():
    db=checkpoint();scope=deepcopy(bind_best_continuation(db,True))
    db['layouts']['fixed']['best_node']='next'
    assert bind_best_continuation(db,False)==scope
    assert bind_best_continuation(db,True)==scope


def test_failed_box_remains_after_another_box_becomes_a_valid_next_target():
    db=checkpoint();record=db['layouts']['fixed'];scope=bind_best_continuation(db,True)
    assert record['nodes']['accepted']['root']['world_cartons']==[dict(id=i) for i in range(29,40)]
    record['nodes']['accepted']['closed']=True
    assert [n['id'] for n in continuation_active_nodes(record,scope['anchor_node'])]==['next']
    assert record['nodes']['accepted']['attempted']==['old-failure']


@pytest.mark.parametrize('mutation',['assumed_removed','pending','q_change'])
def test_continuation_rejects_manual_removal_open_attempt_and_changed_anchor(mutation):
    db=checkpoint()
    if mutation=='assumed_removed':db['layouts']['fixed']['nodes']['accepted']['root']['assumed_removed']=['x']
    elif mutation=='pending':db['pending']=dict(id=1)
    else:
        bind_best_continuation(db,True)
        db['layouts']['fixed']['nodes']['accepted']['root']['q_rad'][0]=.2
    with pytest.raises(ValueError):bind_best_continuation(db,True)


def test_ordinary_search_is_unchanged_and_exhausted_continuation_does_not_rewind():
    db=checkpoint();record=db['layouts']['fixed']
    assert bind_best_continuation(db) is None and len(continuation_active_nodes(record))==4
    scope=bind_best_continuation(db,True)
    for name in ('accepted','next'):record['nodes'][name]['closed']=True
    assert continuation_active_nodes(record,scope['anchor_node'])==[]


def test_departure_representative_uses_front_suffix_without_new_candidate_or_pose():
    timings=['WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW',
        'WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED','WITHDRAW_THEN_WAIT_FULL_OUTFEED']
    items=[]
    for face_index,face in enumerate(('z+','x-')):
        for timing_index,timing in enumerate(timings):
            key=face+timing
            option=dict(candidate_key=key,visit_order=[0,0,0,face_index,*(() if timing_index==0 else (.5 if timing_index==1 else 1,))],
                height_m=.25,synchronization='PREGRASP',outfeed_timing=timing,
                joint_search_contract=dict(roll_rad=0.,offset_face_m=[0.,0.],face=face,mode='Hmax'))
            items.append((object(),option))
    result=coverage_order(items)
    assert [o['candidate_key'] for _,o in result[:3]]==['z+'+timings[0],'x-'+timings[0],'x-'+timings[2]]
    assert {id(o) for _,o in result}=={id(o) for _,o in items}


def test_current_continuation_prioritizes_one_short_drop_park_then_preserves_all_choices():
    items=[]
    for i,(face,timing,gap) in enumerate([
        ('z+','WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW',.05),
        ('x-','WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW',.05),
        ('x-','WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED',.35),
        ('x-','WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED',.05)]):
        option=dict(candidate_key=str(i),visit_order=[0,0,i,0],height_m=.25,
            synchronization='PREGRASP',outfeed_timing=timing,
            joint_search_contract=dict(roll_rad=0.,offset_face_m=[0.,0.],face=face,mode='Hmax',release_bottom_m=.25+gap))
        items.append((object(),option))
    before=deepcopy([o for _,o in items]);ordered=continuation_coverage_order(items)
    assert ordered[0] is items[3]
    assert len(ordered)==len(items) and {id(o) for _,o in ordered}=={id(o) for _,o in items}
    assert [o for _,o in items]==before
    assert continuation_coverage_order(items[:3])==coverage_order(items[:3])


def test_extra_clearance_park_is_first_without_losing_zero_clearance_park():
    option=dict(candidate_key='original',visit_order=[0,0,0,0,.5],height_m=.25,
        outfeed_timing='WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED',synchronization='PREGRASP',
        joint_search_contract=dict(roll_rad=0.,offset_face_m=[0.,0.],face='x-',mode='Hmax',release_bottom_m=.3))
    original=(object(),option)
    extra=(object(),dict(deepcopy(option),candidate_key='extra100',withdraw_extra_m=.10,visit_order=[0,0,0,0,5]))
    assert continuation_coverage_order([original,extra])==[extra,original]


def test_four_explicit_top_front_poses_get_early_visits_and_keep_other_options():
    import math
    items=[]
    for i,(shift,roll) in enumerate([(-.12,math.pi/2),(-.12,-math.pi/2),(-.24,math.pi/2),(-.24,-math.pi/2)]):
        items.append((object(),dict(candidate_key=str(i),visit_order=[12+i,0,0,0],height_m=.2,synchronization='PREGRASP',
            joint_search_contract=dict(roll_rad=roll,offset_face_m=[shift,0.],face='z+',mode='LIFTED_TOP_FIXED',
                contact_pose_family='TOP_FRONT_EDGE_120_240_MM_QUARTER_TURN_V1'))))
    old=(object(),dict(candidate_key='old',visit_order=[0,0,0,0],height_m=.2,synchronization='PREGRASP',
        joint_search_contract=dict(roll_rad=0.,offset_face_m=[0.,0.],face='z+',mode='LIFTED_TOP_FIXED')))
    assert continuation_coverage_order([old,*items])==[items[2],items[3],items[0],items[1],old]


def test_existing_raised_top_near_front_combination_precedes_low_transfer_without_duplicates():
    import math
    c=dict(roll_rad=math.pi/2,offset_face_m=[-.12,0.],face='z+',mode='LIFTED_TOP_EXPLICIT',
        contact_pose_family='TOP_FRONT_EDGE_120_240_MM_QUARTER_TURN_V1')
    high=(object(),dict(candidate_key='raised',visit_order=[12,0,1,0],height_m=.65,synchronization='PREGRASP',joint_search_contract=c))
    low=(object(),dict(candidate_key='low',visit_order=[12,0,0,0],height_m=.2,synchronization='PREGRASP',
        joint_search_contract=dict(c,mode='LIFTED_TOP_FIXED')))
    assert continuation_coverage_order([low,high])==[high,low]
    assert continuation_coverage_order([high])==[high]


def test_two_midpoint_raised_pose_representatives_are_finite_and_preserve_other_receivers():
    import math
    items=[]
    for i,(roll,height) in enumerate([(math.pi/2,.2),(math.pi/2,.65),(-math.pi/2,.65)]):
        items.append((object(),dict(candidate_key=str(i),visit_order=[16+i,0,0,0],height_m=height,synchronization='PREGRASP',
            joint_search_contract=dict(roll_rad=roll,offset_face_m=[-.18,0.],face='z+',
                mode='LIFTED_TOP_FIXED' if height==.2 else 'LIFTED_TOP_EXPLICIT',
                contact_pose_family='TOP_FRONT_EDGE_180_MM_QUARTER_TURN_V1'))))
    assert continuation_coverage_order(items)==[items[1],items[2],items[0]]


def test_short_axis_top_representatives_keep_other_receive_and_departure_choices():
    import math
    items=[]
    for i,(roll,height,timing) in enumerate([(0.,.65,'WAIT_AT_RELEASE'),(0.,.2,'WAIT_AT_RELEASE'),
            (math.pi,.2,'WAIT_AT_RELEASE'),(0.,.2,'WITHDRAW_THEN_WAIT_FULL_OUTFEED')]):
        items.append((object(),dict(candidate_key=str(i),visit_order=[18+int(roll!=0),0,int(height>.2),0,*([1] if i==3 else [])],
            height_m=height,synchronization='PREGRASP',outfeed_timing=timing,
            joint_search_contract=dict(roll_rad=roll,offset_face_m=[-.24,0.],face='z+',
                mode='LIFTED_TOP_FIXED' if height==.2 else 'LIFTED_TOP_EXPLICIT',
                contact_pose_family='TOP_FRONT_EDGE_240_MM_SHORT_AXIS_V1'))))
    ordered=continuation_coverage_order(items)
    assert ordered[:2]==[items[1],items[2]]
    assert len(ordered)==len(items) and {id(o) for _,o in ordered}=={id(o) for _,o in items}
