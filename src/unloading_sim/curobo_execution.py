"""Check the exported controller geometry through the existing stage validator.

No planning, retiming, collision policy or dynamics implementation lives here.
The exporter may remove collinear samples. Its actual retained edges are checked
again; an authority_accepted flag on a native candidate grants no permission.
"""
from time import perf_counter
import numpy as np
from .stage_backend import fingerprint


def execution_knots(segment, bundle):
    """Bind actual C2 reference knots (including holds) to source stage indices."""
    from .m710_replay_physics import replay_command_arrays
    reference = bundle['metadata']['joint_reference']
    replay_command_arrays(bundle, bundle['metadata']['joint_names'])
    if reference['interpolation'] != 'C2_piecewise_quintic_rest_to_rest':
        raise ValueError('unsupported execution interpolation')
    path = np.asarray(segment['path'], float)
    indices = reference['source_retained_indices']
    if (not indices or any(type(i) is not int for i in indices)
            or indices[0] != 0 or indices[-1] != len(path)-1
            or any(b <= a for a,b in zip(indices, indices[1:]))):
        raise ValueError('invalid retained source indices')
    protected = {int(segment[k]) for k in ('grasp_index','release_index','release_retreat_index')}
    protected.update(i for pair in segment['stage_ranges'].values() for i in pair)
    gate = segment.get('approach',{}).get('free_connection_end_index')
    if gate is not None: protected.add(gate)
    if not protected.issubset(indices):
        raise ValueError('execution removed a stage or attachment/release boundary')
    def moving_rows(rows):
        rows = np.asarray(rows,float)
        return rows[np.r_[True, np.any(np.diff(rows,axis=0)!=0,axis=1)]]
    # Holds are valid timing changes. Any unbound spatial knot is rejected.
    if not np.array_equal(moving_rows(reference['positions_rad']), moving_rows(path[indices])):
        raise ValueError('execution geometry differs from its retained source indices')
    return path, indices


def validate_execution_handoff(scene, connector, motion, bundle):
    """Recheck actual exported edges, attachment events and the release envelope."""
    from .layout_trajectory import PhysicalContactAttachment, validate_layout_trajectory_stage_contract
    from .validation_physics import RigidAttachment
    from .history_adaptation import recheck_current_release
    started = perf_counter()
    report = dict(schema='curobo_complete_execution_handoff_v1', accepted=False,
        failure=None, stages=[], native_spline_derivatives_used=False,
        physical_execution_completed=False)
    def finish(failure=None):
        report.update(accepted=failure is None, failure=failure, wall_s=perf_counter()-started)
        return report
    try:
        segment=motion['selected_trajectory_segment']
        validate_layout_trajectory_stage_contract(segment)
        if bundle['metadata']['joint_names'] != scene.snapshot['robot']['joint_names']:
            raise ValueError('execution joint order mismatch')
        path, indices=execution_knots(segment,bundle)
        metadata=bundle['metadata']
        if not metadata['timing_audit']['within_limits']:
            raise ValueError('final timing audit failed')
        effort=np.asarray(metadata['joint_effort_limits_nm'],float)
        if effort.shape!=(6,) or not np.isfinite(effort).all() or np.any(effort<=0):
            raise ValueError('finite execution effort limits required')
        report.update(source_points=len(path),retained_points=len(indices),
            actual_reference_sha256=fingerprint(metadata['joint_reference']),
            timing_audit=metadata['timing_audit'],
            geometry_rule='REVALIDATE_ACTUAL_RETAINED_EDGES_WITH_LEGACY_SPATIAL_GRID',
            time_rule='MONOTONE_C2_REST_TO_REST_ON_STRAIGHT_JOINT_EDGES',
            runtime_effort_inertia_and_feedback_checks_required=True)
        target=next(b for b in scene.cartons if b.name==segment['target'])
        world=list(scene.all_obstacles); unloaded=[b for b in world if b.name!=target.name]
        supports=tuple(scene.support_graph.supported_by[target.name])
        rigid=RigidAttachment(np.asarray(segment['contact']['physical_contact_from_box'],float),
                              target.half_extents,target.name)
        attachment=PhysicalContactAttachment(connector.robot,rigid,
            connector.flange_from_virtual_task_tcp,connector.flange_from_physical_contact)
        if not np.allclose(attachment.box_at(path[segment['grasp_index']]).world_from_local,
                           target.world_from_local,atol=1e-10,rtol=0):
            raise ValueError('execution attachment discontinuity')
        def check(stage,first,last,obstacles,**options):
            chosen=[i for i in indices if first<=i<=last]
            if not chosen or chosen[0]!=first or chosen[-1]!=last:
                raise ValueError('execution stage boundary missing')
            t=perf_counter()
            failure=connector._path_failure(path[chosen],obstacles,stage=stage,**options)
            report['stages'].append(dict(stage=stage,source_range=[first,last],
                retained_points=len(chosen),wall_s=perf_counter()-t,failure=failure))
            return failure
        with connector._contact_context():
            selection=connector._contact_selection(path[segment['grasp_index']],target,
                segment['face'],scene.policy.data['suction'])
            if selection['commanded_active_mask']!=segment['contact']['cup_selection']['commanded_active_mask']:
                raise ValueError('execution commanded contact mask changed')
            tracker=None
            for stage,(first,last) in segment['stage_ranges'].items():
                if stage in ('home','pregrasp'):
                    failure=check(stage,first,last,world)
                elif stage=='contact':
                    gate=segment.get('approach',{}).get('free_connection_end_index',first)
                    if gate>first:
                        failure=check('pregrasp',first,gate,world)
                        if failure:return finish(failure)
                        first=gate
                    failure=check(stage,first,last,world,target_contact=target)
                elif stage in ('support-release','extraction'):
                    if tracker is None:
                        tracker,failure=connector._initial_proximity(target,unloaded,supports)
                        if failure:return finish(failure)
                    failure=check(stage,first,last,unloaded,attachment=attachment,
                                  support_names=supports,initial_proximity=tracker)
                    if failure is None and stage=='extraction' and not tracker.fully_released:
                        failure=dict(reason='ACTUAL_EXTRACTION_CLEARANCE_NOT_REACHED',stage=stage)
                elif stage=='transit':
                    failure=check(stage,first,last,unloaded,attachment=attachment)
                elif stage=='place':
                    receivers=tuple(segment['place']['selection']['receiver_names'])
                    contacts=tuple(n for n in segment['place']['support_names'] if n in receivers) or receivers
                    failure=check(stage,first,last,unloaded,attachment=attachment,support_names=contacts)
                elif stage=='withdrawal':
                    placed=attachment.box_at(path[segment['release_index']])
                    _,sweep=recheck_current_release(segment,connector,unloaded,placed)
                    failure=check(stage,first,last,[*unloaded,placed],target_contact=placed)
                    if failure:return finish(failure)
                    for predicted in sweep:
                        failure=check(stage,first,last,[*unloaded,predicted],target_contact=predicted)
                        if failure:break
                        failure=connector._state_failure(path[last],[*unloaded,predicted],
                            target_contact=predicted,stage='residence')
                        if failure:break
                else:
                    raise ValueError('unsupported complete-task execution stage: '+stage)
                if failure:return finish(failure)
        return finish()
    except (ValueError,KeyError,IndexError,TypeError) as exc:
        return finish(dict(reason='EXECUTION_HANDOFF_INVALID',detail=str(exc)))
