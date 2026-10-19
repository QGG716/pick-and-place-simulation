"""Finite current-scene transfer choices and raw J1 direction, no motion synthesis."""
import numpy as np
from scipy.spatial.transform import Rotation

J1_POLICY='ECO65_RAW_J1_OUTBOUND_NONINCREASING_RETURN_NONDECREASING_V1'
J1_EPSILON=1e-7


def j1_direction(stage):
    return -1 if stage in ('contact','extraction','transit','place') else 1 if stage in ('withdrawal','residence') else 0


def j1_valid(q,direction):
    q=np.asarray(q,float)
    return bool(np.isfinite(q).all() and np.all(direction*np.diff(q[:,0])>=-J1_EPSILON))


def stage_chain_valid(stages):
    names=[s['stage'] for s in stages]
    return (7<=len(names)<=10 and names[:3]==['pregrasp','contact','extraction'] and
            names[-3:]==['place','withdrawal','residence'] and all(n=='transit' for n in names[3:-3]))


def endpoint_fk(robot,q,goal):
    actual=robot.fk(np.asarray(q));goal=np.asarray(goal)
    return dict(position_error_m=float(np.linalg.norm(actual[:3,3]-goal[:3,3])),
                orientation_error_rad=float(np.linalg.norm(Rotation.from_matrix(actual[:3,:3].T@goal[:3,:3]).as_rotvec())))


def request_budgets(remaining,stage,rpc):
    budget=min(remaining,rpc)
    if not np.isfinite([budget,stage]).all() or budget<=10:raise ValueError('NATIVE_REQUEST_BUDGET_EXHAUSTED')
    output=min(120.,max(5.,budget*.35));solver=min(stage,budget-output-5.)
    return dict(request_budget_s=budget,allowed_planning_time_s=solver,
                output_check_timeout_s=output,return_reserve_s=5.)


def clear_stack_extractions(scene,target_pose,size,tcp,face):
    """Keep target orientation; move the full carton in front of the stack before turning."""
    if face!='top':return []
    # Geometry-derived front plane, with unchanged physical gap plus search reserve.
    others=[b for b in scene['boxes'] if b['id']!=scene['target_box_id']]
    front=min(b['pose'][0]-b['size_m'][0]/2 for b in others)
    rear=float(np.asarray(target_pose)[0,3]+np.abs(np.asarray(target_pose)[0,:3])@(np.asarray(size)/2))
    outward=max(0.,rear-front+.0024+.001+.01)
    result=[]
    # A third finite option includes the carton half-diagonal turning envelope.
    # The first two remain for comparison with the fixed failure investigation.
    turn_extra=float(np.ceil((np.linalg.norm(np.asarray(size)/2)-size[0]/2+.02)/.005)*.005)
    for height,extra in ((.01,0.),(.04,0.),(.04,turn_extra)):
        distance=outward+extra
        goal=np.array(tcp,copy=True);goal[0,3]-=distance;goal[2,3]+=height
        result.append((goal,dict(lift_m=height,outward_m=distance,neighbor_front_x_m=front,
            target_initial_rear_x_m=rear,target_final_rear_x_m=rear-distance,turn_envelope_extra_m=extra,
            lift_basis='FULL_CARTON_FRONT_CLEARANCE_BEFORE_TURN',
            transit_requirement='ORDINARY_PAYLOAD_NEIGHBOR_CLEARANCE_WITH_SEARCH_RESERVE')))
    return result


def transfer_waypoints(scene,size,flange_from_target,flange_from_tcp):
    """Small scene-derived front staging poses; actual native IK/path decides usability."""
    receiving=np.asarray(scene['native_task']['receiving']['box_pose_D'])
    base=np.asarray(scene['base_position_m']);half=np.asarray(size)/2
    result=[]
    # Low front poses hit base/link_1; simply raising the same orientation hit
    # link_2 or exceeded reach. Turn the box in front so its tool offset points
    # toward the robot instead of extending across the box. Native IK and motion
    # checks remain the only acceptance criteria for this finite alternative.
    clearance=.08
    for above_base in (.21,.31):
        box=receiving.copy()
        box[:3,:3]=Rotation.from_euler('z',np.pi).as_matrix()@box[:3,:3]
        box[:3,3]=[base[0]-half[0]-.0034-clearance,0.,max(receiving[2,3]+.10,base[2]+above_base)]
        tcp=box@np.linalg.inv(flange_from_target)@flange_from_tcp
        result.append(dict(tcp_pose=tcp,box_pose=box,basis='FRONT_STAGING_WITH_TOOL_OFFSET_TOWARD_BASE',front_clearance_m=clearance,above_base_m=above_base))
    return result
