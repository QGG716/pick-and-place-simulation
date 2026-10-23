from unloading_sim.eco65.transverse_process import fair_process_candidates


def candidate(name, option, cups):
    return dict(candidate_id=name,process_option=dict(option_id=option),active_ids=list(range(cups)))


def test_bounded_pool_reaches_low_before_more_high_grasps():
    high=[candidate('h'+str(i),'high',4) for i in range(20)]
    low=[candidate('l'+str(i),'low',4) for i in range(20)]
    side=[candidate('s0','side',2)]
    ranked=high+low+side
    selected=fair_process_candidates(ranked,8)
    assert [g['candidate_id'] for g in selected]==['h0','l0','s0','h1','l1','h2','l2','h3']
    assert all(any(g is original for original in ranked) for g in selected)
    assert len(high)==20 and len(low)==20


def test_single_option_keeps_cup_and_ik_ranking_and_empty_pool():
    ranked=[candidate('four','high',4),candidate('two','high',2),candidate('one','high',1)]
    assert fair_process_candidates(ranked,2)==ranked[:2]
    assert fair_process_candidates([],8)==[]
