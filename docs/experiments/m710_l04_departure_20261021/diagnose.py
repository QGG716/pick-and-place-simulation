"""Read-only diagnosis of the two recorded l04_c03 tool-departure endpoints.

Arm inverse geometry reuses the existing official-model branch audit, and every
branch is verified by production Pinocchio FK. No planning, limit changes,
acceptance changes, alternative targets, or state commits are performed.
"""
from pathlib import Path
import hashlib,json,math,sys
import xml.etree.ElementTree as ET
import numpy as np

work=Path('/work-population-x300-cb6eb01')
folder=Path('/population-x300-20261021/l04-departure-diagnosis')
sys.path[:0]=[str(work/'tools'),str(work/'src')]
from plan_m710_mount_comparison import make_context,FLAGS
from unloading_sim.ik import pose_error
_,connector,_=make_context('x_minus_300_y_minus_100',layout_path=work/
    'configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_300_y_minus_100.yaml')
robot=connector.robot
assert type(robot).__name__=='PinocchioHppFclBackend'
data=json.loads((folder/'input.json').read_text())
kernel=(folder/'arm-branch-kernel.py').read_text()
# This is the unchanged inverse-geometry/FK-check block from the prior audit.
original=(work/'tools/audit_m710_endpoint_arm_branches.py').read_text()
block=original[original.index('    limits=robot.joint_limits'):original.index('    args.output.mkdir')]
expected='\n'.join(line[4:] if line.startswith('    ') else line for line in block.splitlines())+'\n'
assert kernel==expected
exec(compile(kernel,str(folder/'arm-branch-kernel.py'),'exec'))
urdf=ET.fromstring(data['model']['parameters']['robot_description'])
for i in range(6):
    limit=urdf.find(f"joint[@name='J{i+1}']/limit")
    assert np.allclose(limits[i],[float(limit.get('lower')),float(limit.get('upper'))],atol=1e-12,rtol=0)
mount=urdf.find("joint[@name='world_to_base_link']/origin")
if mount is None:
    mount=next(j.find('origin') for j in urdf.findall('joint')
               if j.find('parent').get('link')=='world' and j.find('child').get('link')=='base_link')
assert np.allclose(list(map(float,mount.get('xyz').split())),robot.base_transform[:3,3],atol=1e-12,rtol=0)
cases=[]
for request in data['requests']:
    q=np.asarray(request['q_start']);goal=np.asarray(request['goal_pose'])
    start=robot.fk(q)
    assert np.max(np.abs(start-robot.named_link_frames(q)['flange']@np.asarray(request['flange_from_task_tcp'])))<1e-9
    control=audit(dict(request,goal_pose=start.tolist(),stage_id='recorded_start_positive_control'))
    matching=[b for b in control['branches'] if b.get('arm_within_limits') and
        np.linalg.norm(np.arctan2(np.sin(np.asarray(b['q123_rad'])-q[:3]),
                                 np.cos(np.asarray(b['q123_rad'])-q[:3])))<1e-8]
    assert len(matching)==1 and robot.within_limits(q)
    endpoint=audit(request)
    matching_key=(matching[0]['shoulder_sign'],matching[0]['elbow_sign'])
    selected=next(b for b in endpoint['branches'] if (b.get('shoulder_sign'),b.get('elbow_sign'))==matching_key)
    displacement=goal[:3,3]-start[:3,3]
    assert np.max(np.abs(goal[:3,:3]-start[:3,:3]))<1e-8
    assert displacement[0]<0 and np.linalg.norm(displacement[1:])<1e-8
    # Counterfactual FK fit identifies all six endpoint angles without ever
    # modifying production limits or sending the result as a motion request.
    fit=q.copy()
    for iteration in range(100):
        error=pose_error(robot.fk(fit),goal)[0]
        if np.linalg.norm(error)<1e-11:break
        jac=robot.geometric_jacobian(fit)
        step=np.linalg.solve(jac,error)
        fit+=step*min(1.,.1/max(float(np.linalg.norm(step)),1e-12))
    error=pose_error(robot.fk(fit),goal)[0]
    assert np.linalg.norm(error)<1e-9
    assert np.linalg.norm(fit[:3]-selected['q123_rad'])<1e-7
    # Limit crossing along precisely the requested straight translation. This
    # gives only an arm-limit upper bound, never a collision-qualified motion.
    lo,hi=0.,1.
    assert selected['q123_rad'][2]<limits[2,0]
    for _ in range(45):
        fraction=(lo+hi)/2;pose=start.copy();pose[:3,3]+=fraction*displacement
        sample=audit(dict(request,goal_pose=pose.tolist()))
        branch=next(b for b in sample['branches'] if (b.get('shoulder_sign'),b.get('elbow_sign'))==matching_key)
        if branch['q123_rad'][2]>=limits[2,0]:lo=fraction
        else:hi=fraction
    singular=np.linalg.svd(robot.geometric_jacobian(q),compute_uv=False)
    cases.append(dict(stage_id=request['stage_id'],start_q_rad=q.tolist(),start_q_deg=np.rad2deg(q).tolist(),
        start_tcp_pose=start.tolist(),goal_pose=goal.tolist(),displacement_world_m=displacement.tolist(),
        requested_distance_m=float(np.linalg.norm(displacement)),
        start_J3_lower_margin_deg=float(np.rad2deg(q[2]-limits[2,0])),
        same_arm_branch_goal=selected,all_endpoint_arm_branches=endpoint,
        start_positive_control=control,actual_start_arm_branch_recovered=True,
        counterfactual_q_rad=fit.tolist(),counterfactual_q_deg=np.rad2deg(fit).tolist(),
        counterfactual_position_error_m=float(np.linalg.norm(error[:3])),
        counterfactual_orientation_error_rad=float(np.linalg.norm(error[3:])),
        counterfactual_within_limits=robot.within_limits(fit),
        counterfactual_violating_joints=[f'J{i+1}' for i in range(6) if not limits[i,0]<=fit[i]<=limits[i,1]],
        required_J3_below_lower_deg=float(np.rad2deg(limits[2,0]-fit[2])),
        distance_to_J3_lower_bound_m=float(np.linalg.norm(displacement)*(lo+hi)/2),
        distance_bound_scope='SAME_ARM_BRANCH_KINEMATIC_LIMIT_ONLY_NOT_COLLISION_QUALIFIED',
        start_jacobian_singular_values=singular.tolist(),
        start_jacobian_condition_number=float(singular[0]/singular[-1])))
result=dict(source_commit=data['source_commit'],source_trace_sha256=data['source_trace_sha256'],
    diagnostic_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    kernel_sha256=hashlib.sha256(kernel.encode()).hexdigest(),backend=type(robot).__name__,
    base_transform=robot.base_transform.tolist(),official_limits_deg=np.rad2deg(limits).tolist(),
    cases=cases,scope='TWO_ACTUALLY_REQUESTED_FIXED_POSE_DEPARTURE_ENDPOINTS_ONLY',
    new_native_planning_requests=0,new_physical_execution=False,**FLAGS)
(folder/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
for c in cases:
    print(json.dumps({k:c[k] for k in ('stage_id','requested_distance_m','start_J3_lower_margin_deg',
        'counterfactual_q_deg','required_J3_below_lower_deg','distance_to_J3_lower_bound_m',
        'counterfactual_violating_joints','start_jacobian_condition_number')}))
print('DIAGNOSIS_COMPLETE')
