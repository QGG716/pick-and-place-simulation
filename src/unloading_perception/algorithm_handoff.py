"""Normalize fused observed patches for the existing ROS/world contracts."""
from dataclasses import replace
from unloading_contracts import ObservationStatus,UnknownRegion,canonical_fingerprint
import numpy as np


def validate_algorithm_capture(observation):
    bindings=observation.coverage.get('module_bindings',{})
    received=set(observation.coverage['received_modules'])
    if set(bindings)!=received or not received.issubset(observation.coverage['expected_modules']):
        raise ValueError('ALGORITHM_MODULE_BINDINGS_MISMATCH')
    for cargo in observation.cargo:
        for surface in cargo.observed_surfaces:
            binding=bindings[surface['module_id']]
            for field in ('capture_id','calibration_identity'):
                if surface[field]!=binding[field]: raise ValueError('ALGORITHM_CAPTURE_CALIBRATION_MISMATCH')
            if surface['sensor_epoch']!=observation.source_epoch or surface['clock_domain']!=observation.clock_domain:
                raise ValueError('ALGORITHM_EPOCH_CLOCK_MISMATCH')
            if not np.allclose(surface['T_W_C_at_capture'],binding['T_W_C_at_capture'],atol=1e-9,rtol=0):
                raise ValueError('ALGORITHM_CAPTURE_TF_MISMATCH')


def fused_algorithm_observation(observations,fusion):
    observations=tuple(sorted(observations, key=lambda o: o.observation_id))
    if not observations: raise ValueError('EMPTY_MODULE_OBSERVATIONS')
    if len({o.clock_domain for o in observations})!=1 or len({o.source_epoch for o in observations})!=1 or len({o.source_sequence for o in observations})!=1:
        raise ValueError('INCOMPATIBLE_MODULE_CAPTURE_GROUP')
    originals={c.source_instance_id:c for o in observations for c in o.cargo}
    if len(originals)!=sum(len(o.cargo) for o in observations): raise ValueError('DUPLICATE_MODULE_INSTANCE_IDENTITY')
    cargo=[]; consumed=set()
    for obj in fusion.objects:
        members=[originals[i] for _,i in obj.source_members]; consumed.update(i for _,i in obj.source_members)
        # Keep all contributing patches as formal source evidence, including
        # partial boundaries even when the display chose a duplicate face.
        surfaces=tuple(sorted((s for member in members for s in member.observed_surfaces),
                              key=lambda s: (s['module_id'], s['capture_id'], s['source_instance_id'], s['face_id'])))
        cargo.append(replace(members[0],source_instance_id=obj.fusion_id,object_id=None,track_id=None,pose=None,full_dimensions_m=None,corners_3d_m=None,axes_3d_rows=None,candidate_eligible=False,eligibility_reasons=('OBSERVED_SURFACES_WITH_UNKNOWN_VOLUME',),association_status=obj.association_status,observed_surfaces=surfaces,raw_result={'source_members':obj.source_members,'fusion_id':obj.fusion_id,'track_id':None,'fusion_diagnostics':obj.diagnostics}))
    # Instances without certified faces must not disappear from the world.
    cargo.extend(replace(c,pose=None,full_dimensions_m=None,corners_3d_m=None,axes_3d_rows=None,candidate_eligible=False,eligibility_reasons=tuple(dict.fromkeys(c.eligibility_reasons+('UNKNOWN_VOLUME',)))) for i,c in sorted(originals.items()) if i not in consumed)
    unknown=tuple(r for o in observations for r in o.unknown_regions) + tuple(UnknownRegion('unknown-volume-'+c.source_instance_id,'world','UNKNOWN_VOLUME_BEHIND_OBSERVED_PATCH') for c in cargo)
    if not cargo and not unknown:
        unknown=(UnknownRegion('empty-algorithm-coverage','world','EMPTY_OBSERVATION_DOES_NOT_PROVE_FREE_SPACE'),)
    result = replace(observations[0],observation_id='algorithm-fusion-'+canonical_fingerprint([o.observation_id for o in observations]),provider='registered-rgbd-fused-algorithm',processed_time=max(o.processed_time for o in observations),status=ObservationStatus.PARTIAL,cargo=tuple(cargo),unknown_regions=unknown,coverage={'source_kind':'ALGORITHM_FROM_ISAAC_RENDERED_RGBD','direct_simulator_truth':False,'expected_modules':fusion.expected_modules,'received_modules':fusion.received_modules,'coverage_status':fusion.coverage_status,'capture_time_range':fusion.capture_time_range,'clock_domain':observations[0].clock_domain,'absence_means_free_space':False,'historical_replay':True,'module_bindings':{o.coverage['module_binding']['module_id']:o.coverage['module_binding'] for o in observations}},synthetic=False)
    # Fusion must not erase the acquisition/proposal disclosure or source evidence.
    coverage = dict(result.coverage)
    coverage['module_coverage'] = {o.coverage['module_binding']['module_id']: o.coverage for o in observations}
    for key in ('proposal_source', 'raw_image_automatic'):
        values = [o.coverage[key] for o in observations if key in o.coverage]
        if values:
            if len(values) != len(observations) or any(v != values[0] for v in values):
                raise ValueError('INCONSISTENT_MODULE_PROVENANCE')
            coverage[key] = values[0]
    result = replace(result, coverage=coverage)
    validate_algorithm_capture(result)
    return result
