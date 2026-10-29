from plan_m710_joint_search import admit_candidate,option_visit_priority,coverage_order,interrupted_evidence

def test_refinements_cannot_starve_distinct_coarse_regions():
    db=dict(candidates=list(range(100)),next_candidate=0,admissions=0,
        refinement_queue=[dict(candidate='fine-'+str(i),parent='first') for i in range(100)])
    visits=[admit_candidate(db) for _ in range(12)]
    assert [e for e,kind,parent in visits if kind=='COARSE']==list(range(8))
    assert sum(kind=='REFINEMENT' for e,kind,parent in visits)==4

def test_queue_resume_consumes_each_candidate_once():
    db=dict(candidates=[1,2],next_candidate=0,admissions=0,refinement_queue=[dict(candidate=3,parent='base')])
    assert [admit_candidate(db)[0] for _ in range(3)]==[1,2,3]
    assert admit_candidate(db) is None

def test_more_exit_distances_cannot_starve_other_faces_or_receive_modes():
    options=[dict(name='front-Hmin-'+str(t),visit_order=[0,0,0,0,t]) for t in (.5,1,2,3,4)]
    options += [dict(name='front-Hmin',visit_order=[0,0,0,0]),
        dict(name='top-Hmax',visit_order=[0,0,0,1]),dict(name='front-Hmax',visit_order=[0,0,1,0]),
        dict(name='front-Hmax-park',visit_order=[0,0,1,0,.5])]
    result=sorted(options,key=option_visit_priority)
    assert [o['name'] for o in result[:3]]==['front-Hmin','top-Hmax','front-Hmax']
    assert result[4]['name']=='front-Hmax-park'
    assert result[-1]['name']=='front-Hmin-4'


def test_pose_and_sync_receive_early_visits_before_exhausting_departure_distances():
    options=[dict(visit_order=[pose,sync,mode,face,*timing])
        for pose in range(12) for sync in range(3) for mode in range(2) for face in range(2)
        for timing in ([],[.5],[1],[2],[3],[4])]
    ordered=sorted(options,key=option_visit_priority)
    first=[o['visit_order'] for o in ordered[:16]]
    assert any(o[0]==1 for o in first) and any(o[1]==1 for o in first)
    assert not any(len(o)==5 and o[4]>=2 for o in first)
    assert len(ordered)==len({tuple(o['visit_order']) for o in ordered})==864


def test_short_node_slice_covers_offsets_rotations_faces_and_modes_without_new_candidates():
    from itertools import product
    poses=[(0.,[0.,0.]),(1.5707963267948966,[0.,0.]),
        (-1.5707963267948966,[0.,0.]),(0.,[.048,0.]),(0.,[-.048,0.])]
    items=[]
    for pose,face,mode,sync,timing in product(range(5),range(2),range(2),range(2),range(3)):
        roll,offset=poses[pose]
        key=str((pose,face,mode,sync,timing))
        option=dict(candidate_key=key,visit_order=[pose,sync,mode,face,*(() if timing==0 else (.5 if timing==1 else 1,))],
            joint_search_contract=dict(roll_rad=roll,offset_face_m=offset,face=('x-','z+')[face],mode=('Hmin','Hmax')[mode]),
            height_m=(.55,.85)[mode],synchronization=('PREGRASP','BEFORE_APPROACH')[sync],
            outfeed_timing=('WAIT_AT_RELEASE_UNTIL_FULL_ENVELOPE_OUTFED_THEN_WITHDRAW',
                'WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED','WITHDRAW_THEN_WAIT_FULL_OUTFEED')[timing])
        items.append((object(),option))
    ordered=coverage_order(items);first=[o for _,o in ordered[:7]]
    assert first[0]['candidate_key']==str((0,0,0,0,0))
    assert any(o['joint_search_contract']['roll_rad']!=0 for o in first)
    assert any(any(o['joint_search_contract']['offset_face_m']) for o in first)
    assert any(o['joint_search_contract']['face']=='z+' for o in first)
    assert any(o['joint_search_contract']['mode']=='Hmax' for o in first)
    assert any(o['synchronization']=='BEFORE_APPROACH' for o in first)
    assert any(o['outfeed_timing']=='WITHDRAW_PARK_THEN_WAIT_FULL_OUTFEED' for o in first)
    assert len(ordered)==len(items)==len({o['candidate_key'] for _,o in ordered})
    assert {id(o) for _,o in ordered}=={id(o) for _,o in items}


def test_coverage_order_retains_empty_and_single_legal_option():
    assert coverage_order([])==[]
    item=(object(),dict(candidate_key='only',visit_order=[0,0,0,0],height_m=.65,
        joint_search_contract=dict(roll_rad=0.,offset_face_m=[0.,0.],face='z+',mode='LIFTED_TOP_EXPLICIT'),
        synchronization='PREGRASP'))
    assert coverage_order([item])==[item]


def test_interruption_after_archive_keeps_evidence_and_call_cost_without_commit(tmp_path):
    import json
    import pytest
    from plan_m710_all_remaining import archive_task
    folder=tmp_path/'task-000001';folder.mkdir()
    calls=[dict(op='ik',stage='pregrasp',round_trip_s=.125)]
    (folder/'requests.json').write_text(json.dumps(calls))
    (folder/'plan.json').write_text(json.dumps(dict(status='UNCOMMITTED_CANDIDATE')))
    expected=archive_task(folder,tmp_path)
    assert not folder.exists()
    evidence,recovered=interrupted_evidence(folder,tmp_path)
    assert evidence==expected and recovered==calls
    # An existing but altered archive must fail closed, not silently disappear.
    (tmp_path/evidence['archive']).write_bytes(b'changed')
    with pytest.raises(AssertionError):interrupted_evidence(folder,tmp_path)
