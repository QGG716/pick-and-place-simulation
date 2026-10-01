"""Mandatory final-reference LIN validation; no ROS, planning, or physics startup."""
from copy import deepcopy
from pathlib import Path
import numpy as np
from .m710_replay_contract import canonical_sha256 as digest, sha256_file
from .moveit2_tcp import audit_linear_tcp

SCHEMA = 'm710_task_tcp_lin_v1'
AUDIT_VERSION = 'm710_final_reference_tcp_v1'
PROGRESS = 'linear_position_and_shortest_rotation_shared_progress'


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def context(root):
    from .layout_single_carton import load_layout_motion_policy
    from .layout_trajectory import OFFICIAL_MODEL_URDF, LayoutTrajectoryBudget
    policy = load_layout_motion_policy(Path(root)/'configs/validation/m710id70_proof_of_concept.yaml')
    return policy, dict(urdf_path=str(OFFICIAL_MODEL_URDF),
        urdf_sha256=sha256_file(Path(root)/OFFICIAL_MODEL_URDF),
        edge_resolution_rad=LayoutTrajectoryBudget().edge_resolution_rad,
        base_transform=policy.layout_validation.layout.robot_base_transform().tolist(),
        flange_from_task_tcp=policy.tool_frames.flange_from_virtual_task_tcp.tolist())


def make_lin_contract(request, origin, *, position_tolerance, orientation_tolerance, edge_resolution, root):
    """Called from the original request, before export or reference processing."""
    _, kin = context(root)
    return dict(schema=SCHEMA, frame_id='world', origin=np.asarray(origin).tolist(),
        destination=deepcopy(request['goal_pose']), q_start=deepcopy(request['q_start']),
        flange_from_task_tcp=deepcopy(request['flange_from_task_tcp']),
        identity=deepcopy(request['identity']), request_fingerprint=digest(request),
        kinematics=kin, position_tolerance_m=float(position_tolerance),
        orientation_tolerance_rad=float(orientation_tolerance),
        edge_resolution_rad=float(edge_resolution), progress=PROGRESS)


def lin_records(segment):
    native = segment.get('native_backend')
    if not native:
        return []
    require(native.get('name') == 'moveit2', 'UNKNOWN_NATIVE_BACKEND')
    records = native.get('stages')
    require(isinstance(records, list), 'NATIVE_STAGES_MISSING')
    result = []
    seen = set()
    path = np.asarray(segment['path'])
    for record in records:
        if record.get('planner_id') != 'LIN':
            require(not record.get('lin_contract'), 'LIN_PLANNER_CONTRACT_MISMATCH')
            continue
        contract = record.get('lin_contract')
        require(isinstance(contract, dict), 'LIN_CONTRACT_MISSING_REQUIRES_MIGRATION_OR_REAUDIT')
        stage_id = record.get('stage_id')
        require(isinstance(stage_id, str) and stage_id and stage_id not in seen, 'LIN_STAGE_ID_INVALID')
        seen.add(stage_id)
        interval = record.get('path_range')
        require(isinstance(interval, list) and len(interval) == 2 and all(type(i) is int for i in interval), 'LIN_PATH_RANGE_MISSING')
        a, b = interval
        require(0 <= a < b < len(path), 'LIN_PATH_RANGE_INVALID')
        points = np.asarray([point['q'] for point in record['points']])
        require(np.array_equal(path[a:b+1], points), 'LIN_PLANNING_RANGE_MISMATCH')
        require(contract.get('request_fingerprint') == record.get('request_fingerprint'), 'LIN_REQUEST_IDENTITY_MISMATCH')
        result.append(record)
    return result


def bind_reference(segment, reference):
    """Use exporter-carried source indices, never infer a range from repeated q."""
    mapping = reference.get('source_path_indices')
    result = []
    for record in lin_records(segment):
        require(isinstance(mapping, list), 'LIN_REFERENCE_MAP_MISSING')
        a, b = record['path_range']
        starts = [i for i, source in enumerate(mapping) if source == a]
        ends = [i for i, source in enumerate(mapping) if source == b]
        require(bool(starts) and bool(ends), 'LIN_BOUNDARY_NOT_RETAINED')
        # Duplicate mapped indices are explicit stationary event holds, not q matches.
        first, last = starts[-1], ends[0]
        require(first < last, 'LIN_REFERENCE_RANGE_INVALID')
        result.append(dict(stage_id=record['stage_id'], path_range=[a,b],
            reference_range=[first,last], contract_sha256=digest(record['lin_contract'])))
    return result


def audit_bundle_tcp(bundle, *, project_root=None):
    metadata = bundle['metadata']
    segment = metadata['m710_execution_preflight']['replay_adapter_inputs']['trajectory_segment']
    require(metadata.get('native_backend') == segment.get('native_backend'), 'NATIVE_METADATA_BINDING_MISMATCH')
    records = lin_records(segment)
    if not records:
        require(not metadata.get('lin_reference_bindings'), 'ORPHAN_LIN_BINDING')
        return dict(status='NOT_APPLICABLE', reason='NO_NATIVE_LIN_STAGES')
    root = Path(project_root) if project_root else Path(__file__).resolve().parents[2]
    reference = metadata['joint_reference']
    require(reference.get('interpolation') == 'C2_piecewise_quintic_rest_to_rest', 'LIN_REFERENCE_INTERPOLATION_UNSUPPORTED')
    mapping = reference.get('source_path_indices')
    joints = np.asarray(reference['positions_rad'], dtype=float)
    require(joints.ndim == 2 and joints.shape[1] == 6 and np.isfinite(joints).all(), 'LIN_REFERENCE_INVALID')
    require(isinstance(mapping, list) and len(mapping) == len(joints) and
        all(type(i) is int and 0 <= i < len(segment['path']) for i in mapping) and
        all(a <= b for a,b in zip(mapping,mapping[1:])), 'LIN_REFERENCE_MAP_INVALID')
    bindings = bind_reference(segment, reference)
    require(metadata.get('lin_reference_bindings') == bindings, 'LIN_REFERENCE_BINDING_MISMATCH')
    policy, kin = context(root)
    robot = policy.layout_validation.layout.robot()
    native_identity = segment['native_backend']['startup']['identity']
    require(native_identity.get('scene_fingerprint') == metadata['m710_execution_preflight']['input_identity']['scene_fingerprint'], 'LIN_SCENE_IDENTITY_MISMATCH')
    require(metadata['gripper']['frame_contract']['T_flange_virtual_task_tcp'] == kin['flange_from_task_tcp'], 'LIN_EXECUTOR_TCP_CONTEXT_MISMATCH')
    audits = []
    for record, binding in zip(records, bindings):
        c = record['lin_contract']
        require(c.get('schema') == SCHEMA and c.get('progress') == PROGRESS, 'LIN_CONTRACT_VERSION_OR_PROGRESS')
        require(c.get('frame_id') == 'world', 'LIN_REFERENCE_FRAME_MISMATCH')
        require(c.get('identity') == native_identity and c.get('kinematics') == kin, 'LIN_CONTEXT_MISMATCH')
        require(c.get('flange_from_task_tcp') == kin['flange_from_task_tcp'] == native_identity.get('flange_from_task_tcp'), 'LIN_TCP_TRANSFORM_MISMATCH')
        require(digest(c['flange_from_task_tcp']) == native_identity.get('task_tcp_fingerprint'), 'LIN_TCP_IDENTITY_MISMATCH')
        require(c.get('position_tolerance_m') == float(policy.data['ik']['position_tolerance_m']) and
            c.get('orientation_tolerance_rad') == float(policy.data['ik']['orientation_tolerance_rad']), 'LIN_TOLERANCE_MISMATCH')
        # Same conservative edge rule as the original planner; never coarsen it.
        require(c.get('edge_resolution_rad') == kin['edge_resolution_rad'], 'LIN_RESOLUTION_MISMATCH')
        a,b = binding['reference_range']; p,q = record['path_range']
        require(np.array_equal(joints[a],segment['path'][p]) and np.array_equal(joints[b],segment['path'][q]), 'LIN_REFERENCE_BOUNDARY_CHANGED')
        require(np.array_equal(c.get('q_start'),segment['path'][p]), 'LIN_ORIGINAL_START_CHANGED')
        for key in ('origin','destination','flange_from_task_tcp'):
            value=np.asarray(c[key],dtype=float)
            require(value.shape==(4,4) and np.isfinite(value).all() and np.allclose(value[3],[0,0,0,1],atol=1e-12,rtol=0) and
                np.allclose(value[:3,:3].T@value[:3,:3],np.eye(3),atol=1e-12,rtol=0) and np.linalg.det(value[:3,:3])>0, 'LIN_INVALID_TRANSFORM')
        audit = audit_linear_tcp(joints[a:b+1], robot.fk, c['origin'], c['destination'],
            position_tolerance=c['position_tolerance_m'], orientation_tolerance=c['orientation_tolerance_rad'], edge_resolution_rad=c['edge_resolution_rad'])
        require(audit['passed'], 'LIN_TASK_TCP_CONSTRAINT: '+str(audit['first_failure']))
        audits.append(dict(stage_id=record['stage_id'], reference_range=[a,b], audit=audit))
    # Every mapped joint must remain its explicit source node (including holds).
    require(np.array_equal(joints,np.asarray(segment['path'])[mapping]), 'FINAL_REFERENCE_SOURCE_PATH_CHANGED')
    identity = metadata['m710_execution_preflight']['input_identity']
    binding = dict(version=AUDIT_VERSION, reference=reference, stages=records,
        lin_reference_bindings=bindings, execution_input_identity=identity,
        audit_implementation_sha256={str(Path(p).name):sha256_file(Path(__file__).with_name(p))
            for p in ('m710_execution_tcp.py','moveit2_tcp.py','robot.py','geometry.py')})
    return dict(status='PASS', binding_sha256=digest(binding), audit_version=AUDIT_VERSION,
        checked_object='FINAL_EXECUTOR_JOINT_REFERENCE', audits=audits,
        continuous_sweep_proof=False, report_reuse=False)
