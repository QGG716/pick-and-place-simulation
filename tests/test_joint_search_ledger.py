from copy import deepcopy
import pytest
from verify_m710_joint_search import selected_branch

def evidence():
    initial=dict(world_cartons=[{'id':str(i)} for i in range(40)],completed=[],predecessor_final_state_receipt=None)
    after=dict(world_cartons=initial['world_cartons'][1:],completed=['0'],predecessor_final_state_receipt={'receipt_id':'real-required-in-cycle-verifier'})
    return dict(layouts={'fixed':dict(initial=initial,best_node='child',nodes={
        'root':dict(id='root',incoming=None,parent=None,root=initial),
        'child':dict(id='child',incoming=0,parent='root',root=after)})},
        trials=[dict(layout='fixed',diagnostic=False,complete=True,node='root',target_id='0')])

def test_valid_single_branch_structure():assert len(selected_branch(evidence(),'fixed'))==1

@pytest.mark.parametrize('tamper',['assumed','cross_layout','duplicate','missing','diagnostic','world_changed'])
def test_reject_fabricated_ledger(tamper):
    db=evidence();r=db['layouts']['fixed']
    if tamper=='assumed':r['initial']['assumed_removed']=['removed-box']
    if tamper=='cross_layout':db['trials'][0]['layout']='another-layout'
    if tamper=='duplicate':r['nodes']['child']['root']['completed']=['0','0']
    if tamper=='missing':r['initial']['world_cartons'].pop()
    if tamper=='diagnostic':db['trials'][0]['diagnostic']=True
    if tamper=='world_changed':r['nodes']['child']['root']['world_cartons'][0]={'id':'moved'}
    with pytest.raises(AssertionError):selected_branch(db,'fixed')
