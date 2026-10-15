"""Cold native single-carton coordinator. Every motion is generated in MTC.

This module has no Python IK/RRT import or hardware execution provider.
Independent MuJoCo geometry is constructed only for the selected final plan.
"""
from __future__ import annotations
from copy import deepcopy
from time import perf_counter
import uuid
import numpy as np

from .model import robot_for, transform, check_transform
from .native_model import POLICY
from .native_scene import verify_scene
from .native_contact import check_contours
from .home_candidate import verify_candidate
from .mount_alignment import check_alignment
from ..moveit2_backend import ResidentMoveItClient, MoveItUnavailable, digest, validate_native_result, native_generation_evidence
from ..timing import JointMotionLimits, time_parameterize_joint_path, sample_quintic_knots
from ..moveit2_tcp import audit_linear_tcp

STATUS = 'PLANNING_ONLY_NOT_EXECUTABLE'
PHASES = ['pregrasp', 'contact', 'extraction', 'transit', 'place', 'withdrawal', 'residence']
INDEPENDENT_PHASE = dict(pregrasp='approach', contact='contact', extraction='loaded_lift',
    transit='loaded_transfer', place='place_contact', withdrawal='retreat', residence='retreat')


def extraction_goal(scene, target_pose, size, tcp):
    """Retain the 10 mm lift clearance above crossed support surfaces."""
    half=np.asarray(size)/2; start=np.asarray(target_pose)[:3,3]
    receive=np.asarray(scene['source_equipment_config']['conveyors']['longitudinal']['receive_center_xy_m'])
    lo=np.minimum(start[:2],receive)-half[:2]; hi=np.maximum(start[:2],receive)+half[:2]
    bottom=float(start[2]-np.abs(np.asarray(target_pose)[2,:3])@half)
    crossed=[]
    for row in scene['obstacles']:
        if row['id'] not in scene['support_surface_ids']: continue
        p=np.asarray(row['position_m']); h=np.asarray(row['size_m'])/2
        if np.all(p[:2]+h[:2]>lo) and np.all(p[:2]-h[:2]<hi):
            crossed.append(dict(id=row['id'],top_m=float(p[2]+h[2])))
    highest=max([bottom,*[r['top_m'] for r in crossed]])
    lift=highest+.01-bottom; goal=np.array(tcp,copy=True);goal[2,3]+=lift
    return goal,dict(initial_bottom_m=bottom,crossed_supports=crossed,highest_support_m=highest,
        retained_lift_clearance_m=.01,lift_m=lift,goal_bottom_m=bottom+lift)


class NativeTask:
    def __init__(self, snapshot, command, *, task_seconds=900., stage_seconds=60., attempt=0, cancelled=None, home_candidate=None):
        from .native_io import verify_snapshot
        verify_snapshot(snapshot)
        self.snapshot = snapshot; self.scene = verify_scene(snapshot['scene'])
        self.model = snapshot['native_model']; self.init = self.model['init']; self.identity = self.init['identity']
        if self.identity['scene_fingerprint'] != self.scene['scene_fingerprint']: raise ValueError('MODEL_SCENE_MISMATCH')
        if not np.isfinite([task_seconds, stage_seconds]).all() or min(task_seconds, stage_seconds) <= 0:
            raise ValueError('INVALID_NATIVE_BUDGET')
        self.robot = robot_for(snapshot['tool'], self.scene)
        self.home_candidate = None if home_candidate is None else deepcopy(verify_candidate(home_candidate, snapshot['tool'], self.scene))
        self.home_checks = []; self.native_calls = []
        self.extraction_evidence = None
        self.cancelled = cancelled or (lambda: False)
        self.stage_seconds = stage_seconds; self.deadline = perf_counter()+task_seconds
        self.task_id = 'eco65-'+uuid.uuid4().hex
        self.counter = 10000*attempt; self.parent = ''; self.attachment = None
        self.target = deepcopy(self.scene['box']); self.target_pose = transform(self.target['pose'])
        self.records = []; self.ik_records = []; self.stages = []; self.events = []
        started = perf_counter(); self.native = ResidentMoveItClient(command)
        try:
            self.ready = self.call(self.init)
            if self.ready.get('status') != 'READY' or self.ready.get('identity') != self.identity:
                raise MoveItUnavailable('NATIVE_MODEL_IDENTITY_REJECTED')
            self.initialization_s = perf_counter()-started
        except Exception as error:
            error.native_calls = self.native_calls
            self.native.close(); raise

    def close(self): self.native.close()

    def call(self, request):
        remaining = self.deadline-perf_counter()
        if self.cancelled(): raise MoveItUnavailable('CANCELLED')
        if remaining <= 0: raise MoveItUnavailable('NATIVE_TASK_BUDGET_EXHAUSTED')
        request = deepcopy(request)
        if request['op'] != 'init': request['allowed_planning_time_s'] = min(self.stage_seconds, remaining)
        started = perf_counter()
        record = dict(request=request)
        self.native_calls.append(record)
        try:
            result = self.native.request(request, timeout=min(remaining+5, self.stage_seconds+30), cancelled=self.cancelled)
            record['result'] = result
            return result
        except Exception as error:
            record['error'] = str(error)
            raise
        finally: record['wall_s'] = perf_counter()-started

    def request(self, q, stage, *, op='check_state_offline', endpoint=False):
        world = [dict(id=o['id'], size=o['size_m'], pose=transform(o['position_m']).tolist(), category='environment')
                 for o in self.scene['obstacles']]
        for b in self.scene['boxes']:
            if b['id'] == self.target['id'] and self.attachment is not None: continue
            pose = self.target_pose if b['id'] == self.target['id'] else transform(b['pose'])
            world.append(dict(id=b['id'], size=b['size_m'], pose=pose.tolist(), category='carton'))
        target_pose = (self.robot.named_link_frames(q)['link_6'] @ self.attachment
                       if self.attachment is not None else self.target_pose)
        target = dict(id=self.target['id'], size=self.target['size_m'], pose=target_pose.tolist())
        attached = None if self.attachment is None else {**target, 'pose': self.attachment.tolist(),
                    'touch_links': self.init['compliant_tool_links']}
        supports = self.target['supported_by'] if stage == 'extraction' else ['conveyor_longitudinal_belt'] if stage == 'place' else []
        process = dict(self.init['process_geometry'], schema='eco65_native_process_v1', stage=stage,
            target_id=self.target['id'], target=target, target_face='top', support_names=supports,
            stack_carton_ids=[b['id'] for b in self.scene['boxes']], initial_proximity=None,
            max_attachment_gap_m=.0004, maximum_penetration_m=.0004, suction_edge_margin_m=.003,
            max_normal_misalignment_rad=self.snapshot['tool']['contact']['normal_tolerance_rad'], collision_policy=POLICY)
        clearance = dict(schema='eco65_native_process_clearance_v1', source_policy=POLICY, stage=stage,
            tool_links=self.init['tool_links'], compliant_tool_links=self.init['compliant_tool_links'],
            target_id=self.target['id'], payload_id='' if attached is None else self.target['id'],
            conveyor_ids=[o['id'] for o in self.scene['obstacles'] if o['owner'].startswith('conveyor')],
            receiver_reserve_m=0., numerical_gap_tolerance_m=1e-9, edge_resolution_rad=.002)
        return dict(op=op, identity=self.identity, q_start=np.asarray(q).tolist(), world=world,
            scene_fingerprint=digest(world), attachment=attached, stage=stage, task_id=self.task_id,
            parent_stage_id=self.parent, seed=6518+self.counter, start_velocity=[0.]*6, path_constraints={},
            allowed_pairs=[['base_link', 'robot_mount_plate']], process_policy=process, clearance_policy=clearance,
            clearance_mode='optimized', endpoint=endpoint, cancelled=False,
            flange_from_task_tcp=self.identity['flange_from_task_tcp'], planning_only=True,
            planning_mode='cold_from_scratch', history_enabled=False, velocity_scale=.5, acceleration_scale=.5)

    def check_fk(self, home):
        reports = []
        for q in [np.zeros(6), home, [.2, -.3, .4, .5, -.6, .7], [-.4, .2, -.35, -.6, .4, -.8]]:
            frames = self.robot.named_link_frames(q)
            reply = self.call(dict(op='fk', identity=self.identity, q_start=np.asarray(q).tolist(),
                links=list(frames)+[self.identity['task_tcp_link']]))
            expected = {**frames, self.identity['task_tcp_link']: self.robot.fk(q)}
            error = max(float(np.max(np.abs(np.asarray(reply['frames'][name])-value))) for name, value in expected.items())
            if error > 1e-8: raise ValueError('ECO65_NATIVE_FK_MISMATCH')
            reports.append(dict(q_rad=np.asarray(q).tolist(), max_matrix_error=error))
        return reports

    def ik(self, pose, seed, stage, count=32):
        rng = np.random.default_rng(6518+self.counter)
        for q in [np.asarray(seed), *rng.uniform(self.robot.joint_limits[:, 0], self.robot.joint_limits[:, 1], (count-1, 6))]:
            self.counter += 1
            req = dict(op='ik', identity=self.identity, q_start=q.tolist(), goal_pose=check_transform(pose).tolist(),
                seed=6518+self.counter, planning_only=True, planning_mode='cold_from_scratch',
                position_tolerance_m=3e-5, orientation_tolerance_rad=3e-4)
            result = self.call(req); self.ik_records.append(dict(stage=stage, request=req, result=result))
            if result.get('native_ik_calls') != 1: raise ValueError('NATIVE_IK_EVIDENCE_MISSING')
            if result['status'] == 'SUCCESS':
                value = np.asarray(result['q'], float)
                if value.shape != (6,) or not np.isfinite(value).all(): continue
                if np.any(value < self.robot.joint_limits[:, 0]) or np.any(value > self.robot.joint_limits[:, 1]): continue
                checked = self.call(self.request(value, stage))
                if checked['status'] == 'SUCCESS': yield value

    def motion(self, q, stage, *, pose=None, goal=None):
        constrained = stage in {'contact', 'extraction', 'place', 'withdrawal'}
        solvers = [('pilz_industrial_motion_planner', 'LIN' if constrained else 'PTP')]
        if not constrained: solvers.append(('ompl', 'RRTConnectkConfigDefault'))
        goals = [None] if constrained else ([np.asarray(goal)] if goal is not None else list(self.ik(pose, q, stage)))
        if not constrained:
            goals.sort(key=lambda candidate: float(np.abs(candidate-q).sum()))
            unique=[]
            for candidate in goals:
                if all(np.max(np.abs(candidate-other))>1e-5 for other in unique): unique.append(candidate)
            goals=unique
        # A first valid IK branch can require a long turn through a wall. Give
        # the existing bounded native IK set its PTP attempts before spending
        # the OMPL budget on any branch; never substitute Python motion search.
        for pipeline, planner in solvers:
            for joint_goal in goals:
                self.counter += 1; req = self.request(q, stage, op='plan')
                req.update(stage_id=f'{stage}-{self.counter}', pipeline_id=pipeline, planner_id=planner, require_native_motion=True)
                if constrained: req['goal_pose'] = check_transform(pose).tolist()
                else: req['q_goal'] = joint_goal.tolist()
                started = perf_counter(); raw = self.call(req)
                self.records.append(dict(request=req, result=raw, wall_s=perf_counter()-started))
                if raw['status'] != 'SUCCESS': continue
                if (not native_generation_evidence(raw, req) or raw.get('mtc_generation') is not True
                        or raw.get('generated_during_task') is not True or raw.get('task_id') != self.task_id
                        or raw.get('parent_stage_id') != self.parent or raw.get('stage_id') != req['stage_id']):
                    raise ValueError('NATIVE_MTC_PROVENANCE_MISSING')
                path = validate_native_result(raw, q, joint_goal, self.init['joint_names'])
                if constrained:
                    audit = audit_linear_tcp(path, self.robot.fk, self.robot.fk(q), pose,
                        position_tolerance=.0001, orientation_tolerance=.001, edge_resolution_rad=.002)
                    if not audit['passed']: continue
                limits = self.model['simulation_limits']
                velocity = [min(float(self.model['official_limits'][name]['velocity']), limits['joint_velocity_rad_s']) for name in self.init['joint_names']]
                timed = time_parameterize_joint_path(path, JointMotionLimits(np.array(velocity),
                    np.full(6, limits['joint_acceleration_rad_s2']), np.full(6, limits['joint_jerk_rad_s3'])))
                self.stages.append(dict(stage=stage, stage_id=req['stage_id'], parent_stage_id=self.parent,
                    q=timed.positions.tolist(), times=timed.time_from_start.tolist(), native_result=raw,
                    native_request=req, attachment=None if self.attachment is None else self.attachment.tolist(),
                    target_pose_start=self.target_pose.tolist()))
                self.parent = req['stage_id']
                return np.asarray(path[-1])
        raise ValueError('NATIVE_STAGE_CANDIDATES_EXHAUSTED:'+stage)

    def select_home(self, pre):
        q = np.asarray(self.home_candidate['q_rad'] if self.home_candidate else
                       self.scene['source_equipment_config']['task']['initial_seed_q_rad'], float)
        row = dict(source='EXPLICIT_CANDIDATE' if self.home_candidate else 'CONFIGURED_SEED', q_rad=q.tolist(),
                   scene_fingerprint=self.scene['scene_fingerprint'], candidate=self.home_candidate)
        self.home_checks.append(row)
        e = self.snapshot['tool']['mounting_evidence']
        row['mount_alignment'] = check_alignment(e['official_bolt_centers_xy_m'], e['bolt_centers_cad_xz_m'],
            e['mount_point_cad_m'][1], self.snapshot['tool']['T_flange_tool_cad'])
        if row['mount_alignment']['status'] != 'PASS': raise ValueError('HOME_MOUNT_INTERFACE_INVALID')
        in_limits = (q.shape == (6,) and np.isfinite(q).all() and
            np.all(q >= self.robot.joint_limits[:, 0]) and np.all(q <= self.robot.joint_limits[:, 1]))
        row['joint_limits'] = 'PASS' if in_limits else 'FAIL'
        if in_limits:
            row['native_state'] = self.call(self.request(q, 'pregrasp'))
        if not in_limits or row['native_state']['status'] != 'SUCCESS':
            row['status'] = 'REJECTED'
            q = next(self.ik(pre, q if in_limits else np.mean(self.robot.joint_limits, axis=1), 'pregrasp', 64), None)
            if q is None: raise ValueError('NATIVE_HOME_CANDIDATES_EXHAUSTED')
            row = dict(source='BOUNDED_NATIVE_IK_SEARCH', q_rad=q.tolist(), max_candidates=64,
                       scene_fingerprint=self.scene['scene_fingerprint'])
            self.home_checks.append(row)
        try: row['fk_crosscheck'] = self.check_fk(q)
        except Exception as error:
            row.update(status='REJECTED', reason=str(error))
            raise  # Inconsistent FK is a model/worker failure, not an IK search miss.
        row['status'] = 'ACCEPTED_SIMULATION_INITIAL_STATE'
        return q, row['fk_crosscheck']

    def run(self):
        started = perf_counter()
        top = self.target_pose @ transform([0, 0, self.target['size_m'][2]/2], np.diag([1., -1., -1.]))
        pre = top.copy(); pre[2, 3] += .03
        q, fk = self.select_home(pre)
        home = q.copy()
        # A legal HOME already at the approach pose is still a state, not a
        # fabricated nonzero motion. Choose a distinct nearby checked approach.
        if np.linalg.norm(self.robot.fk(q)[:3, 3]-pre[:3, 3]) < 1e-4: pre[2, 3] += .005
        q = self.motion(q, 'pregrasp', pose=pre)
        q = self.motion(q, 'contact', pose=top)
        seal = check_contours(self.init['process_geometry'], self.robot.named_link_frames(q)['link_6'],
                              self.target_pose, self.target['size_m'], gap=.0004, penetration=.0004)
        if seal['status'] != 'PASS': raise ValueError('CONTACT_ENDPOINT_CONTOURS_REJECTED')
        self.attachment = np.linalg.inv(self.robot.named_link_frames(q)['link_6']) @ self.target_pose
        self.events.append(dict(event='ATTACH_PREDICTED', after_stage=self.parent,
            target_id=self.target['id'], flange_from_target=self.attachment.tolist(), q=q.tolist()))
        lift, self.extraction_evidence = extraction_goal(self.scene,self.target_pose,self.target['size_m'],self.robot.fk(q))
        q = self.motion(q, 'extraction', pose=lift)
        cfg = self.scene['source_equipment_config']['conveyors']
        center = [*cfg['longitudinal']['receive_center_xy_m'], cfg['surface_z_m']+self.target['size_m'][2]/2]
        place_box = transform(center)
        place_tcp = place_box @ np.linalg.inv(self.attachment) @ np.asarray(self.identity['flange_from_task_tcp'])
        preplace = place_tcp.copy(); preplace[2, 3] += .03
        q = self.motion(q, 'transit', pose=preplace)
        q = self.motion(q, 'place', pose=place_tcp)
        self.target_pose = self.robot.named_link_frames(q)['link_6'] @ self.attachment
        if not supported(self.scene, self.target_pose): raise ValueError('SUPPORTED_RELEASE_REJECTED')
        self.events.append(dict(event='SUPPORTED_RELEASE_PREDICTED', after_stage=self.parent,
            target_id=self.target['id'], pose=self.target_pose.tolist(), q=q.tolist()))
        self.attachment = None
        retreat = self.robot.fk(q).copy(); retreat[2, 3] += .03
        q = self.motion(q, 'withdrawal', pose=retreat)
        q = self.motion(q, 'residence', goal=home)
        result = dict(schema='eco65_native_plan_v1', status=STATUS, task_id=self.task_id,
            identity=self.identity, snapshot_fingerprint=self.snapshot['fingerprint'], home_q=home.tolist(),
            stages=self.stages, events=self.events, fk_crosscheck=fk,
            native_records=self.records, native_ik_records=self.ik_records,
            native_calls=self.native_calls, initial_state_checks=self.home_checks,
            extraction_geometry=self.extraction_evidence,
            initial_state_semantics='SIMULATION_INITIAL_STATE_NOT_A_VERIFIED_HOMING_MOTION',
            native_initialization_s=self.initialization_s,
            planning_wall_s=perf_counter()-started, simulated_duration_s=sum(s['times'][-1] for s in self.stages),
            dynamics='NOT_EVALUATED', hardware_enabled=False, independent_geometry='NOT_EVALUATED')
        result['fingerprint'] = digest(result)
        validate_plan_contract(result, self.snapshot)
        return result


def supported(scene, pose):
    b = scene['box']; half = np.array(b['size_m'])/2
    belt = next(o for o in scene['obstacles'] if o['id'] == 'conveyor_longitudinal_belt')
    center = np.array(belt['position_m']); bh = np.array(belt['size_m'])/2
    corners = np.array([[x, y, -half[2]] for x in (-half[0], half[0]) for y in (-half[1], half[1])])
    corners = corners@pose[:3, :3].T+pose[:3, 3]
    return bool(np.max(np.abs(corners[:, 2]-(center[2]+bh[2]))) <= .0004
                and np.all(np.abs(corners[:, :2]-center[:2]) <= bh[:2]+1e-8))


def validate_plan_contract(plan, snapshot):
    expected = dict(plan); fingerprint = expected.pop('fingerprint')
    if digest(expected) != fingerprint or plan['status'] != STATUS or plan['hardware_enabled']:
        raise ValueError('PLAN_IDENTITY_OR_MODE_MISMATCH')
    if plan['identity'] != snapshot['native_model']['init']['identity'] or plan['snapshot_fingerprint'] != snapshot['fingerprint']:
        raise ValueError('PLAN_MODEL_TOOL_SCENE_MISMATCH')
    if [s['stage'] for s in plan['stages']] != PHASES: raise ValueError('PLAN_STAGE_CHAIN_INCOMPLETE')
    if [e['event'] for e in plan['events']] != ['ATTACH_PREDICTED', 'SUPPORTED_RELEASE_PREDICTED']:
        raise ValueError('PLAN_EVENT_CHAIN_INCOMPLETE')
    previous = np.asarray(plan['home_q']); parent = ''; seen = set()
    model = snapshot['native_model']; joints = model['init']['joint_names']; limits = model['simulation_limits']
    bounds = np.array([[float(model['official_limits'][j][k]) for k in ('lower', 'upper')] for j in joints])
    velocity = np.array([min(float(model['official_limits'][j]['velocity']), limits['joint_velocity_rad_s']) for j in joints])
    attach, release = plan['events']; target = snapshot['scene']['target_box_id']
    for event, phase in [(attach, 'contact'), (release, 'place')]:
        s = next(s for s in plan['stages'] if s['stage'] == phase)
        if event['target_id'] != target or event['after_stage'] != s['stage_id'] or not np.array_equal(event['q'], s['q'][-1]):
            raise ValueError('PLAN_EVENT_IDENTITY_MISMATCH')
    check_transform(attach['flange_from_target']); check_transform(release['pose'])
    for s in plan['stages']:
        q, t = np.asarray(s['q']), np.asarray(s['times'])
        if (q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or len(t) != len(q) or
                not np.isfinite(q).all() or not np.isfinite(t).all() or t[0] != 0 or np.any(np.diff(t) <= 0)):
            raise ValueError('EMPTY_OR_INVALID_TRAJECTORY')
        if not np.any(np.abs(np.diff(q, axis=0)) > 1e-12): raise ValueError('STATIONARY_MOTION_NOT_AN_EVENT')
        if np.max(np.abs(previous-q[0])) > 1e-9 or s['parent_stage_id'] != parent:
            raise ValueError('PLAN_STAGE_DISCONTINUITY')
        if s['stage_id'] in seen: raise ValueError('PLAN_STAGE_ID_REUSED')
        seen.add(s['stage_id'])
        if np.any(q < bounds[:, 0]-1e-9) or np.any(q > bounds[:, 1]+1e-9): raise ValueError('PLAN_JOINT_LIMIT')
        d, dt = np.abs(np.diff(q, axis=0)), np.diff(t)[:, None]
        if (np.any(1.875*d/dt > velocity+1e-8) or
                np.any(10/np.sqrt(3)*d/dt**2 > limits['joint_acceleration_rad_s2']+1e-8) or
                np.any(60*d/dt**3 > limits['joint_jerk_rad_s3']+1e-8)):
            raise ValueError('PLAN_SIMULATION_LIMIT')
        raw, req = s['native_result'], s['native_request']
        if s['stage'] in {'contact', 'extraction', 'place', 'withdrawal'} and (
                req.get('pipeline_id') != 'pilz_industrial_motion_planner' or req.get('planner_id') != 'LIN'
                or 'goal_pose' not in req): raise ValueError('PLAN_PROCESS_REQUIRES_LIN')
        if req['clearance_policy']['source_policy'] != POLICY or any(
                req['process_policy'].get(k) != v for k, v in model['init']['process_geometry'].items()):
            raise ValueError('PLAN_POLICY_OR_CONTACT_GEOMETRY_CHANGED')
        if (req['identity'] != plan['identity'] or req['task_id'] != plan['task_id'] or
                raw.get('task_id') != plan['task_id'] or req['stage_id'] != s['stage_id'] or req['stage'] != s['stage'] or
                req['planning_only'] is not True or req['planning_mode'] != 'cold_from_scratch' or
                raw.get('planning_only_status') != STATUS or raw.get('native_output_status') != 'PASS'):
            raise ValueError('PLAN_NATIVE_IDENTITY_MISMATCH')
        loaded = s['stage'] in {'extraction', 'transit', 'place'}
        expected_attachment = attach['flange_from_target'] if loaded else None
        if s['attachment'] != expected_attachment or (req['attachment'] is not None) != loaded:
            raise ValueError('PLAN_ATTACHMENT_LIFECYCLE_INVALID')
        expected_world = {o['id']: (o['size_m'], transform(o['position_m']).tolist()) for o in snapshot['scene']['obstacles']}
        expected_world.update({b['id']: (b['size_m'], transform(b['pose']).tolist()) for b in snapshot['scene']['boxes'] if b['id'] != target})
        expected_target = release['pose'] if s['stage'] in {'withdrawal', 'residence'} else transform(snapshot['scene']['box']['pose']).tolist()
        if not loaded: expected_world[target] = (snapshot['scene']['box']['size_m'], expected_target)
        actual_world = {b['id']: (b['size'], b['pose']) for b in req['world']}
        if len(actual_world) != len(req['world']) or actual_world != expected_world: raise ValueError('PLAN_WORLD_CHANGED')
        if loaded and (req['attachment']['id'] != target or req['attachment']['pose'] != expected_attachment or
                       req['attachment']['touch_links'] != model['init']['compliant_tool_links']):
            raise ValueError('PLAN_ATTACHMENT_IDENTITY_INVALID')
        path = validate_native_result(raw, q[0], None, snapshot['native_model']['init']['joint_names'])
        if not np.array_equal(path, q) or not native_generation_evidence(raw, req) or raw.get('generation_source') != 'NATIVE_GENERATED':
            raise ValueError('PLAN_NATIVE_PATH_CHANGED')
        if (raw.get('status') != 'SUCCESS' or raw.get('generated_during_task') is not True or
                raw.get('mtc_generation') is not True or raw.get('stage_id') != s['stage_id'] or raw.get('parent_stage_id') != parent):
            raise ValueError('PLAN_MTC_CHAIN_MISMATCH')
        previous = q[-1]; parent = s['stage_id']
    if not np.allclose(previous, plan['home_q'], atol=1e-9, rtol=0): raise ValueError('PLAN_FINAL_HOME_CHANGED')
    if abs(sum(s['times'][-1] for s in plan['stages'])-plan['simulated_duration_s']) > 1e-8:
        raise ValueError('PLAN_DURATION_MISMATCH')
    return True


def review_plan(plan, snapshot):
    """One independent world and exactly the same monotonic joint-edge replay."""
    from .task_world import UnloadingTaskWorld
    from .native_io import verify_snapshot
    verify_snapshot(snapshot)
    validate_plan_contract(plan, snapshot); started = perf_counter()
    world = UnloadingTaskWorld(snapshot['tool'], snapshot['scene'])
    world.select_target(snapshot['scene']['target_box_id'], face='top')
    world.receiving_region = 'conveyor_longitudinal'; checks = 0
    # Recompute contact against the extracted outer CAD loops, not the source
    # script's fitted circles. This also governs the source contact permission.
    world.tool = deepcopy(snapshot['tool'])
    world.tool['native_contact_geometry'] = snapshot['native_model']['init']['process_geometry']
    try:
        for stage in plan['stages']:
            phase = INDEPENDENT_PHASE[stage['stage']]
            q, ts = np.asarray(stage['q']), np.asarray(stage['times'])
            if stage['stage'] in {'contact', 'extraction', 'place', 'withdrawal'}:
                audit = audit_linear_tcp(q, world.robot.fk, world.robot.fk(q[0]), stage['native_request']['goal_pose'],
                    position_tolerance=.0001, orientation_tolerance=.001, edge_resolution_rad=.0008)
                if not audit['passed']: raise ValueError('INDEPENDENT_TCP_LIN_REJECTED:'+str(audit))
            for i, (a, b) in enumerate(zip(q[:-1], q[1:])):
                n = max(1, int(np.ceil((ts[i+1]-ts[i])/.02)), int(np.ceil(1.875*np.abs(b-a).sum()/.0008)))
                values, _ = sample_quintic_knots(ts[i:i+2], q[i:i+2], np.linspace(ts[i], ts[i+1], n+1))
                for value in values:
                    checks += 1
                    if not world.valid(value, phase):
                        return dict(status='FAIL', stage=stage['stage'], edge=i, q=value.tolist(),
                                    failure=world.last_failure, checked_states=checks, wall_s=perf_counter()-started,
                                    plan_fingerprint=plan['fingerprint'], snapshot_fingerprint=snapshot['fingerprint'])
            if stage['stage'] == 'contact':
                relative = world.attach(q[-1])
                expected = np.asarray(snapshot['tool']['T_flange_tcp'])@relative
                if not np.allclose(expected, plan['events'][0]['flange_from_target'], atol=1e-8, rtol=0):
                    raise ValueError('INDEPENDENT_ATTACHMENT_TRANSFORM_MISMATCH')
            if stage['stage'] == 'place':
                pose = world.release(q[-1])
                if not np.allclose(pose, plan['events'][1]['pose'], atol=1e-8, rtol=0):
                    raise ValueError('INDEPENDENT_RELEASE_TRANSFORM_MISMATCH')
        return dict(status='PASS', checked_states=checks, wall_s=perf_counter()-started,
                    plan_fingerprint=plan['fingerprint'], snapshot_fingerprint=snapshot['fingerprint'],
                    target_id=world.box_id, release_pose=world.released_pose.tolist(),
                    other_cartons_preserved=True, continuous_sweep_proof=False, dynamics='NOT_EVALUATED')
    finally: world.close()
