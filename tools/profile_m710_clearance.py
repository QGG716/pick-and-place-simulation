"""Actual native/authority differential checks; no task search or execution."""
import argparse
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.layout_trajectory import PhysicalContactAttachment
from unloading_sim.validation_physics import RigidAttachment
from unloading_sim.moveit2_backend import MoveItLayoutConnector


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.parent.mkdir(parents=True,exist_ok=True);started=perf_counter()
    policy=load_layout_motion_policy(ROOT/'configs/validation/m710id70_proof_of_concept.yaml')
    scene=build_verified_motion_input(policy,ROOT)
    core=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot()).connector
    c=MoveItLayoutConnector.from_existing(core,scene);c.start_planning_request()
    old=json.loads((ROOT/'tests/fixtures/moveit2/frozen_candidate.json').read_text())['selected_trajectory_segment']
    historical=json.loads((ROOT/'tests/fixtures/moveit2/clearance_rejections.json').read_text())
    assert historical['scene_fingerprint']==scene.snapshot['scene_fingerprint']
    target=next(b for b in scene.cartons if b.name==old['target'])
    c.stack_carton_names={b.name for b in scene.cartons};c.robot_state_validator.stack_carton_names=frozenset(c.stack_carton_names)
    c.robot_state_validator.contact_target_name=target.name
    q_grasp=np.asarray(old['path'][old['grasp_index']]);physical=c.physical_from_virtual(c.robot.fk(q_grasp))
    attachment=PhysicalContactAttachment(c.robot,RigidAttachment.capture(physical,target),c.flange_from_virtual_task_tcp,c.flange_from_physical_contact)
    obstacles=[b for b in scene.all_obstacles if b.name!=target.name]
    request=c._build_native_request(historical['q_start'],historical['q_goal'],obstacles,seed=71072,attachment=attachment,stage='transit')

    from hashlib import sha256
    states=[]; labels=[]
    def add(q,label): states.append(np.asarray(q).tolist());labels.append(label)
    for k in ('q_start','q_goal'): add(historical[k],k)
    for i,case in enumerate(historical['cases']):
        add(case['original_failure']['q_rad'],f'historical_failure_{i}')
        a,b=np.asarray(case['edge'])
        for f in np.linspace(0,1,9): add(a+f*(b-a),f'historical_edge_{i}_{f}')
    a,b=old['stage_ranges']['transit']
    for i,q in enumerate(old['path'][a:b+1]): add(q,f'frozen_transit_{i}')
    for j in range(6):
        for delta in (-.16,-.08,-.02,.02,.08,.16):
            q=np.asarray(historical['q_start']).copy();q[j]+=delta;add(q,f'start_axis_{j}_{delta}')
    # Fixed finite grid, never a random scan or a planning obstacle change.
    report=dict(schema='m710_clearance_fixed_set_v1',request=request,startup=c.native_startup,
        states=states,labels=labels,input_sha256=sha256(json.dumps(states,separators=(',',':')).encode()).hexdigest(),
        unique_states=len({tuple(q) for q in states}),state_count=len(states),isaac='NOT_RUN')
    try:
        report['native']=c.native.request({**request,'op':'validate','probe_states':states})
        report['authority']=[]
        for q in states:
            t=perf_counter();failure=c._state_failure(np.asarray(q),obstacles,attachment=attachment,stage='transit')
            report['authority'].append(dict(valid=failure is None,failure=failure,seconds=perf_counter()-t))
        report['boolean_mismatches']=[i for i,(n,a) in enumerate(zip(report['native']['states'],report['authority'])) if n['valid']!=a['valid']]
        report['status']='PASS' if not report['boolean_mismatches'] else 'MISMATCH'
    finally:
        c.native.close();report['end_to_end_s']=perf_counter()-started
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print(report['status'],report['state_count'],report['native']['clearance']['profile'],report['native']['clearance'].get('workspace'),flush=True)

if __name__=='__main__':main()
