"""Optional native adapter for explicit, unloaded XY relocation samples.

An exact countertranslation of the world lets the existing native robot model
check each external-axis sample. It is never used as an IK hint or as a claim
that MoveIt generated a six-axis trajectory for moving the base.
"""
from copy import deepcopy
import hashlib,json,time
from pathlib import Path
import numpy as np
from .coupled_batch import batch_scene,coupled_binding,CoupledBatchClient
from .moveit2_backend import ResidentMoveItClient,box_message,digest
from .single_contact import SingleContactConnector
from .xy_base_continuation import XY_SCHEMA,xy_path,xy_candidate,relocation_world,check_plinth_world


def check_xy_relocation(args,context,state,source,destination,baseline,folder,*,order=(0,1)):
    folder=Path(folder);folder.mkdir(exist_ok=False);started=time.monotonic()
    original,built,frozen,initial=context;layout=original.policy.layout_validation.layout
    if (source.z<=.6 or source.z!=destination.z or source.minimum!=destination.minimum
            or source.maximum!=destination.maximum or source.profile!=destination.profile
            or not state.cartons or state.assumed_removed):
        raise ValueError('XY_UNLOADED_SOLID_PLINTH_SCOPE')
    scene=batch_scene(original,state,built.robot,layout);before=deepcopy(state.root())
    path=xy_path([source.x,source.y],[destination.x,destination.y],order=order)
    target=min(state.cartons,key=lambda b:(b.center[2],b.name))
    # Choose an actual lowest carton solely as the check_start protocol's
    # unattached reference object; no grasp, cup command or contact is claimed.
    if target.name in built.collision_policy.tool_chassis_contact_target_ids:
        raise ValueError('XY_REQUIRES_REFERENCE_WITHOUT_TOOL_CHASSIS_EXCEPTION')
    row=dict(schema=XY_SCHEMA,operation='UNLOADED_BASE_XY_SAMPLED_PATH',from_layout=source.evidence(),to_layout=destination.evidence(),
        batch_root=before,path_xy_a_m=path.tolist(),axis_order=list(order),maximum_step_m=.005,
        reference_object_id=target.name,reference_is_not_grasp_target=True,commanded_active_mask=[False]*72,
        robot_q_constant=state.q.tolist(),receiver_height_constant_m=state.height,
        native_evaluation='ROBOT_MODEL_AT_SOURCE_BASE_WORLD_COUNTERTRANSLATED_BY_NEGATIVE_BASE_DELTA',
        base_actuation='KINEMATIC_ASSUMPTION_NOT_PHYSICAL_EXECUTION',continuous_swept_proof=False,physical_execution='NOT_RUN',
        samples=[],passed=False,failure=None)
    raw=[];client=None;c=None
    validator=built.robot_state_validator
    old={k:getattr(validator,k,None) for k in ('contact_target_name','commanded_cup_mask','stack_carton_names')}
    try:
        binding=coupled_binding(layout,state.cartons,state.height,state.height,target.name,scene.snapshot['assembly']['designed_contacts'])
        binding['joint_search_layout']=source.evidence()
        client=CoupledBatchClient(ResidentMoveItClient(args.worker_command,log_path=str(folder/'worker.log')),binding=binding)
        c=SingleContactConnector.from_existing(built,scene,native_client=client,planning_mode='cold_from_scratch',
            native_budget=dict(stage_wall_time_s=30.,ipc_timeout_s=40.))
        c.stack_carton_names={b.name for b in state.cartons};validator.stack_carton_names=set(c.stack_carton_names)
        validator.contact_target_name=target.name;validator.commanded_cup_mask=(False,)*72
        c._native_contact_context=dict(target=target,face='top',suction=deepcopy(scene.policy.data['suction']),
            selection={k:[False]*72 for k in ('geometrically_eligible_mask','commanded_active_mask','actual_contact_mask')})
        c._candidate_identity='xy-unloaded:'+digest(dict(root=before,destination=destination.evidence(),order=list(order)))
        for index,xy in enumerate(path):
            xy_candidate(source,xy,baseline)
            delta=np.r_[xy-np.array([source.x,source.y]),0.]
            canonical=relocation_world(scene.all_obstacles,delta)
            support=check_plinth_world(canonical)
            sample=dict(index=index,base_xy_a_m=xy.tolist(),delta_world_m=delta.tolist(),plinth=support,
                canonical_world_sha256=digest([box_message(b) for b in canonical]))
            row['samples'].append(sample)
            if not support['passed']:
                row['failure']=dict(reason='XY_PLINTH_PATH_REJECTED',sample=index,detail=support['failures']);break
            evaluation=relocation_world(scene.all_obstacles,delta,evaluation_frame=True)
            cpu=built.validate_unloaded_state(state.q,evaluation,stage='unloaded')
            sample['cpu_robot_failure']=cpu
            if cpu is not None:
                row['failure']=dict(reason='XY_ROBOT_PATH_REJECTED',sample=index,detail=cpu);break
            reference=next(b for b in evaluation if b.name==target.name)
            request=c._build_native_request(state.q,state.q,evaluation,seed=args.seed,stage='pregrasp',target_contact=reference)
            request.update(op='check_start')
            try:response=client.request(request,timeout=40.)
            except Exception as exc:
                raw.append(dict(sample=index,request=request,error=dict(reason=type(exc).__name__,detail=str(exc))))
                raise
            raw.append(dict(sample=index,request=request,response=response))
            sample.update(native_status=response.get('status'),evaluation_world_sha256=digest(request['world']),
                native_request_sha256=digest(request),native_response_sha256=digest(response))
            if response.get('status')!='SUCCESS':
                row['failure']=dict(reason='XY_NATIVE_PATH_REJECTED',sample=index,detail=response);break
        else:row['passed']=True
    except Exception as exc:
        row['failure']=dict(reason=type(exc).__name__,detail=str(exc),scope='XY_RELOCATION_ATTEMPT')
        import traceback
        (folder/'exception.txt').write_text(traceback.format_exc())
    finally:
        for k,v in old.items():setattr(validator,k,v)
        if client:
            (folder/'native-model.json').write_text(json.dumps(client.trace_inputs.get('model'),indent=2))
            (folder/'native-records.json').write_text(json.dumps(client.records))
            client.close()
        (folder/'native-checks.json').write_text(json.dumps(raw))
        row['native_checks_sha256']=hashlib.sha256((folder/'native-checks.json').read_bytes()).hexdigest()
        row['wall_s']=time.monotonic()-started
        assert state.root()==before,'XY_CHECK_MUTATED_ACCEPTED_WORLD'
        (folder/'xy-motion.json').write_text(json.dumps(row,indent=2))
    return row
