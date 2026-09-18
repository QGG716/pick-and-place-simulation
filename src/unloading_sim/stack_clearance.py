"""Current PhysX box geometry, separate from within-step contact features.

Only a task's target and named stack boxes with adapter-verified native Cube
colliders enter this path. Robot, tool and receiver classification is unchanged.
"""
from collections import Counter, deque
from copy import deepcopy
import hashlib
import json

import numpy as np

from .geometry import OBB
from .pair_clearance import obb_surface_distance
from .serial_unloading import rotation_from_actual_quaternion


def paired_contact_key(actor0, actor1, collider0, collider1):
    """Canonicalize the pair without detaching a collider from its actor."""
    a, b = sorted(((actor0, collider0), (actor1, collider1)))
    return a[0], b[0], a[1], b[1]


def copy_contact_points(header, data):
    """Copy callback-owned values during their one-step lifetime."""
    points = []
    for index in range(int(header.contact_data_offset),
                       int(header.contact_data_offset) + int(header.num_contact_data)):
        try:
            p = data[index]
            points.append(dict(contact_point_separation_m=float(p.separation),
                position_m=[float(v) for v in p.position], normal=[float(v) for v in p.normal],
                impulse_ns=[float(v) for v in p.impulse],
                face_index0=int(p.face_index0), face_index1=int(p.face_index1)))
        except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
            points.append({'invalid_contact_data': True})
    return points


class StackClearanceStep:
    """One task, one pre/post-step measurement chain and bounded evidence ring.

    Unresolved evidence holds trajectory time using the caller's existing hold.
    It never grants a new contact permission and cannot unlatch free space.
    """

    def __init__(self, *, shapes, target, neighbors, policy, world_id, task_id,
                 maximum_wait_s):
        self.names = {target, *neighbors}
        self.target = target
        self.neighbors = self.names - {target}
        self.shapes = {name: deepcopy(shapes[name]) for name in sorted(self.names)}
        self.policy = policy
        self.world_id, self.task_id = str(world_id), str(task_id)
        self.maximum_wait_s = float(maximum_wait_s)
        if not policy.poc_pair_clearance or not np.isfinite(self.maximum_wait_s) or self.maximum_wait_s < 0:
            raise ValueError('invalid stack clearance policy/wait')
        for name, shape in self.shapes.items():
            if (shape.get('source') != 'VERIFIED_NATIVE_PHYSX_CUBE'
                    or shape.get('name') != name or shape.get('actor') != shape.get('collider')
                    or not str(shape.get('actor', '')).startswith('/')
                    or np.asarray(shape.get('half_extents_m')).shape != (3,)
                    or not np.all(np.isfinite(shape['half_extents_m']))
                    or min(shape['half_extents_m']) <= 0):
                raise ValueError('stack shape identity is not verified')
        self.actors = {s['actor']: name for name, s in self.shapes.items()}
        if len(self.actors) != len(self.names):
            raise ValueError('duplicate stack actor')
        self.shape_fingerprint = hashlib.sha256(json.dumps(self.shapes, sort_keys=True).encode()).hexdigest()
        self.step = -1
        self.open = False
        self.pending = {}
        self.transferred_pending = []
        self.hold_started = None
        self.hold = False
        self.counts = Counter()
        self.ring = deque(maxlen=12)
        self.transition_window = []
        self.transition_tail = 0
        self.transitions = []
        self.first_conflict = None  # immutable historical first issue, not an active latch
        self.confirmed_violation = None
        self.active_issues = {}
        self.issue_history = []
        self.lifecycles = {}
        self.previous_post = None
        self.conflict_window = []
        self.conflict_tail = 0
        self.last = None

    def contains_pair(self, actor0, actor1):
        names = {self.actors.get(actor0), self.actors.get(actor1)}
        return self.target in names and len(names) == 2 and names <= self.names

    def _frame(self, states, *, phase, time_s, distances):
        source = {x['name']: x for x in states}
        if len(source) != len(states) or not self.names <= source.keys():
            raise ValueError('missing/duplicate current stack body')
        boxes, copied = {}, {}
        for name in sorted(self.names):
            x, shape = source[name], self.shapes[name]
            if x.get('prim_path') != shape['actor']:
                raise ValueError('current stack actor mismatch')
            for field in ('center_m', 'linear_velocity_m_s', 'angular_velocity_rad_s'):
                if np.asarray(x[field]).shape != (3,) or not np.all(np.isfinite(x[field])):
                    raise ValueError('invalid actual rigid body measurement')
            rotation = rotation_from_actual_quaternion(x['quaternion_wxyz'])
            center = np.asarray(x['center_m']) + rotation @ np.asarray(shape['local_center_m'])
            boxes[name] = OBB(center, shape['half_extents_m'],
                              rotation @ np.asarray(shape['local_rotation']), name, 'carton')
            copied[name] = deepcopy(x)
        pairs = {}
        if distances:
            for name in sorted(self.neighbors):
                sat = boxes[self.target].signed_distance_obb(boxes[name])
                pairs[name] = dict(surface_distance_m=obb_surface_distance(boxes[self.target], boxes[name]),
                    intersection=bool(sat <= 0), sat_overlap_bound_m=max(0., -float(sat)))
        return dict(step=self.step, time_s=float(time_s), sample_phase=phase,
                    policy_fingerprint=self.policy.fingerprint, shape_fingerprint=self.shape_fingerprint,
                    boxes=boxes, states=copied, pairs=pairs)

    def begin_step(self, *, step, time_s, trajectory_time_s, context, states, world_id=None, task_id=None):
        if (world_id is not None and str(world_id) != self.world_id
                or task_id is not None and str(task_id) != self.task_id):
            raise ValueError('stack measurement world/task mismatch')
        if self.open or step <= self.step or not np.isfinite([time_s, trajectory_time_s]).all():
            raise ValueError('stack geometry step is stale or unclosed')
        self.step, self.context = int(step), deepcopy(context)
        self.trajectory_time_s = float(trajectory_time_s)
        self.pre = self._frame(states, phase='PRE_WORLD_STEP_PHYSX_TENSOR', time_s=time_s, distances=False)
        self.contacts = []
        self.open = True

    def collect(self, *, actor0, actor1, collider0, collider1, event, points):
        if not self.contains_pair(actor0, actor1):
            return False
        if not self.open:
            raise ValueError('stack contact outside an open physical step')
        self.contacts.append(dict(actor0=actor0, actor1=actor1, collider0=collider0,
            collider1=collider1, event=event, points=deepcopy(points), step=self.step,
            context=deepcopy(self.context),
            sample_phase='CONTACT_GENERATION_WITHIN_WORLD_STEP_SUBTIME_NOT_EXPOSED',
            manifold_id='NOT_EXPOSED_BY_INSTALLED_API'))
        return True

    def _point_audit(self, points, neighbor, post):
        seps = [float(p['contact_point_separation_m']) for p in points]
        vectors = [np.asarray(p[k], float) for p in points for k in ('position_m','normal','impulse_ns')]
        if not np.isfinite(seps).all() or any(v.shape != (3,) or not np.isfinite(v).all() for v in vectors):
            raise ValueError('invalid point')
        if any(abs(np.linalg.norm(p['normal'])-1) > 1e-4 for p in points):
            raise ValueError('invalid contact normal')
        for p in points:
            for name in (self.target, neighbor):
                offset = self.shapes[name]['contact_generation_offset_m']
                motion = np.linalg.norm(post['boxes'][name].center-self.pre['boxes'][name].center)
                radius = np.linalg.norm(post['boxes'][name].half_extents)
                rotation_motion = radius*np.linalg.norm(post['boxes'][name].rotation-self.pre['boxes'][name].rotation)
                bound = 2*offset + abs(p['contact_point_separation_m']) + motion + rotation_motion
                distance = min(frame['boxes'][name].point_distance_squared(np.asarray(p['position_m']))
                               for frame in (self.pre, post))**.5
                if not np.isfinite(bound) or bound < 0 or distance > bound + 1e-9:
                    raise ValueError('contact point does not bind either measured box pose')
        return min(seps), any(np.linalg.norm(p['impulse_ns']) > 0 for p in points)

    def _continuous_clearance(self, neighbor, post):
        """Measured endpoint clearance and continuous backend readback chain.

        Endpoint velocities and float32 representation bounds reject pose jumps.
        This does not claim an unexposed exact contact-generation subtime or a
        reconstructed solver trajectory. Both measured endpoints keep the same
        5 mm gate; observable feature history is checked separately.
        """
        if self.previous_post is None or self.previous_post['time_s'] != self.pre['time_s']:
            return False
        dt = post['time_s'] - self.pre['time_s']
        for name in (self.target, neighbor):
            a, b = self.pre['states'][name], post['states'][name]
            prev = self.previous_post['states'][name]
            if any(a[k] != prev[k] for k in ('center_m','quaternion_wxyz','linear_velocity_m_s','angular_velocity_rad_s')):
                return False
            ca, cb = np.asarray(a['center_m']), np.asarray(b['center_m'])
            distance = np.linalg.norm(cb-ca)
            # Bounds derive from the actual tensor representation, not a new
            # configurable physical epsilon or a contact/impulse threshold.
            rounding = np.linalg.norm(np.abs(np.spacing(ca.astype(np.float32)))
                                      + np.abs(np.spacing(cb.astype(np.float32))))
            velocity_bound = dt * (np.linalg.norm(a['linear_velocity_m_s']) + np.linalg.norm(b['linear_velocity_m_s']))
            if distance > velocity_bound + rounding + 1e-9:
                return False
            qa, qb = np.asarray(a['quaternion_wxyz']), np.asarray(b['quaternion_wxyz'])
            angular_motion = min(np.linalg.norm(qa-qb), np.linalg.norm(qa+qb)) * 2
            angular_rounding = 2*np.linalg.norm(np.abs(np.spacing(qa.astype(np.float32)))
                                                + np.abs(np.spacing(qb.astype(np.float32))))
            if angular_motion > dt*(np.linalg.norm(a['angular_velocity_rad_s'])+np.linalg.norm(b['angular_velocity_rad_s'])) + angular_rounding + 1e-9:
                return False
        before = obb_surface_distance(self.pre['boxes'][self.target], self.pre['boxes'][neighbor])
        after = post['pairs'][neighbor]['surface_distance_m']
        return before >= self.policy.required_pair_clearance_m and after >= self.policy.required_pair_clearance_m

    def _features(self, contact, frame):
        a, b = contact['actor0'], contact['actor1']
        flip = a > b
        actor = min(a, b)
        rotation = frame['boxes'][self.actors[actor]].rotation
        result = []
        first, second = (frame['boxes'][self.actors[x]] for x in sorted((a,b)))
        for p in contact['points']:
            normal = np.asarray(p['normal'])*(-1 if flip else 1)
            gap = self._normal_gap(first,second,normal)
            result.append(dict(normal_local=(rotation.T@normal).tolist(),
                faces=[p['face_index1'],p['face_index0']] if flip else [p['face_index0'],p['face_index1']],
                separation_m=p['contact_point_separation_m'], normal_gap_m=gap))
        return result

    @staticmethod
    def _normal_gap(first, second, normal):
        return float((first.center-second.center)@normal
            - np.abs(first.rotation.T@normal)@first.half_extents
            - np.abs(second.rotation.T@normal)@second.half_extents)

    def _separation_explained(self, old, feature, key, post):
        values, rounding = [], 0.
        for frame in (self.pre,post):
            first,second = (frame['boxes'][self.actors[a]] for a in key[:2])
            normal = first.rotation@np.asarray(feature['normal_local'])
            values.append(old['separation_m']+self._normal_gap(first,second,normal)-old['normal_gap_m'])
            # Sum represented-input ULP bounds for this projection. This does
            # not enlarge the physical clearance or allowed penetration.
            for box in (first,second):
                for x in (box.center,box.half_extents,box.rotation):
                    rounding += float(np.abs(np.spacing(np.asarray(x,dtype=np.float32))).sum())
        return min(values)-rounding <= feature['separation_m'] <= max(values)+rounding

    def _persistent_feature(self, key, contact, post, neighbor):
        life = self.lifecycles.get(key)
        if (life is None or life['closed'] or contact['event'] not in {'CONTACT_PERSIST','CONTACT_PERSISTS'}
                or life['last_seen_step'] not in {self.step-1,self.step}
                or not self._continuous_clearance(neighbor, post)):
            return False
        features = self._features(contact, post)
        # Match observable normal/face features from the licensed contact
        # process. These are NOT invented manifold IDs. Original points are
        # retained and separately checked against both measured native boxes.
        return all(any(f['faces']==old['faces'] and np.linalg.norm(
            np.asarray(f['normal_local'])-old['normal_local']) <= 1e-4
            and self._separation_explained(old,f,key,post)
            for old in life['features']) for f in features)

    def _issue(self, key, reason, contact, time_s, *, confirmed=False):
        if key not in self.active_issues:
            issue = dict(id=len(self.issue_history), key=list(key), reason=reason, step=self.step,
                time_s=float(time_s), status='CONFIRMED_VIOLATION' if confirmed else 'PENDING_REVIEW',
                original_contact=deepcopy(contact), review_action=(
                    'STOP_CONFIRMED_VIOLATION' if confirmed else
                    'VERIFY_FINITE_FEATURES_OF_SAME_UNBROKEN_CONTACT_PROCESS_AND_RETAINED_GEOMETRY'),
                history_explainable=True)
            self.issue_history.append(issue)
            self.active_issues[key] = issue
            if self.first_conflict is None:
                self.first_conflict = deepcopy(issue)
        if confirmed:
            self.active_issues[key]['status'] = 'CONFIRMED_VIOLATION'
            self.confirmed_violation = self.confirmed_violation or dict(step=self.step,reason=reason)
        self.pending[key] = reason

    def _resolve(self, key, explanation, time_s):
        issue = self.active_issues.pop(key, None)
        if issue is None and key in self.pending:
            issue = dict(id=len(self.issue_history), key=list(key), reason=self.pending[key],
                status='PENDING_REVIEW', source='EXPLICITLY_TRANSFERRED_INITIALIZATION_PENDING',
                original_scope=next((deepcopy(r['original_scope']) for r in self.transferred_pending
                                     if r['key']==list(key)), None))
            self.issue_history.append(issue)
        if issue is not None:
            assert issue['status'] == 'PENDING_REVIEW'
            issue.update(status='EXPLAINED', resolved_step=self.step, resolved_time_s=float(time_s),
                         resolution=explanation)
            self.counts['issues_explained'] += 1
        self.pending.pop(key, None)

    @property
    def active_reason(self):
        return (self.confirmed_violation['reason'] if self.confirmed_violation else
                next(iter(self.pending.values()), None))

    def finish_step(self, *, states, time_s, monitor, commanded_motion):
        if not self.open or not np.isfinite(time_s) or time_s <= self.pre['time_s']:
            raise ValueError('post-step measurement is not aligned')
        self.open = False
        post = self._frame(states, phase='POST_WORLD_STEP_PHYSX_TENSOR', time_s=time_s, distances=True)
        prior_free = bool(monitor.free_space_reached)
        if prior_free != self.context['actual_free_space']:
            raise ValueError('stack contact context changed within step')
        permits = bool(not prior_free and (self.context['stage'] in {'settling','pregrasp','approach','contact'}
            or self.context['attached'] and self.policy.allows_stack_planning_contact(self.context['stage'])))
        verdicts = []
        for contact in self.contacts:
            a,b,c,d = (contact[k] for k in ('actor0','actor1','collider0','collider1'))
            key = paired_contact_key(a,b,c,d)
            neighbor = next(n for n in (self.actors[a],self.actors[b]) if n != self.target)
            geometry = post['pairs'][neighbor]
            reason, explanation, confirmed = None, None, False
            response, lower = None, None
            points = contact['points']
            life = self.lifecycles.get(key)
            if c != self.shapes[self.actors[a]]['collider'] or d != self.shapes[self.actors[b]]['collider']:
                reason, confirmed = 'STACK_CONTACT_SHAPE_IDENTITY_MISMATCH', True
            elif contact['event'] not in {'CONTACT_FOUND','CONTACT_PERSIST','CONTACT_PERSISTS','CONTACT_LOST'}:
                reason, confirmed = 'STACK_CONTACT_UNKNOWN_EVENT', True
            elif contact['event'] == 'CONTACT_LOST':
                if life is not None:
                    life['closed'] = True
                # LOST is evidence of an ended report lifecycle, not a proof
                # that an earlier missing/contradictory report was safe.
            elif not points:
                reason = 'STACK_CONTACT_POINTS_PENDING'
                if life is not None and not life['closed'] and contact['event'] != 'CONTACT_FOUND':
                    life['last_seen_step'] = self.step
            else:
                try:
                    lower,response = self._point_audit(points,neighbor,post)
                    self.counts['headers_with_physical_response'] += int(response)
                    self.counts['headers_with_negative_point_separation'] += int(lower < 0)
                    self.counts['small_positive_point_separation_with_clear_box_geometry'] += int(
                        0 < lower < self.policy.required_pair_clearance_m
                        and geometry['surface_distance_m'] >= self.policy.required_pair_clearance_m)
                    if lower < -self.policy.maximum_actual_penetration_m:
                        reason,confirmed = 'STACK_CONTACT_SEPARATION_EXCEEDS_ALLOWED_RANGE',True
                    elif permits:
                        # A licensed within-step contact can end in separated
                        # post-solve geometry. Neither sign nor nonzero impulse
                        # alone contradicts that allowed transition.
                        explanation = 'LICENSED_STEP_CONTACT_CURRENT_GEOMETRY_DECIDES_EXIT'
                        self.lifecycles[key] = dict(last_permitted_step=self.step, last_permitted_time_s=float(time_s),
                            context=deepcopy(self.context), features=self._features(contact,post),
                            last_seen_step=self.step, closed=False)
                    elif lower < 0 or response:
                        if self._persistent_feature(key,contact,post,neighbor):
                            explanation = 'PERSISTING_LICENSED_FEATURE_WITH_CONTINUOUS_CLEAR_SOLIDS'
                            life['last_seen_step'] = self.step
                        else:
                            reason = 'STACK_CONTACT_ORIGIN_OR_CONTINUITY_UNRESOLVED'
                    else:
                        explanation = 'VALID_POSITIVE_FEATURE_CURRENT_BOX_CLEARANCE'
                        if life is not None and not life['closed'] and contact['event'] != 'CONTACT_FOUND':
                            life['last_seen_step'] = self.step
                    if contact['event'] == 'CONTACT_FOUND' and not permits and life is not None:
                        life['closed'] = True
                except (KeyError,ValueError,TypeError,OverflowError):
                    reason,confirmed = 'STACK_CONTACT_INVALID_MEASUREMENT',True
            if reason:
                self._issue(key,reason,contact,time_s,confirmed=confirmed)
            elif explanation and key in self.pending:
                issue = self.active_issues.get(key)
                # Only missing finite data can be completed by subsequent
                # matching features; contradictory finite evidence is never
                # overwritten by a newer favorable report.
                if ((issue is None or issue['reason']=='STACK_CONTACT_POINTS_PENDING'
                     and issue['status']=='PENDING_REVIEW' and issue['history_explainable'])
                        and (permits or explanation=='PERSISTING_LICENSED_FEATURE_WITH_CONTINUOUS_CLEAR_SOLIDS')):
                    self._resolve(key,explanation,time_s)
            issue = self.active_issues.get(key)
            if issue is not None and issue['status']=='PENDING_REVIEW':
                issue['history_explainable'] &= bool(permits or self._continuous_clearance(neighbor,post)
                    and life is not None and not life['closed'])
            verdicts.append(dict(**contact, geometry=geometry, evidence_reason=reason, explanation=explanation,
                contact_response_observed=response, minimum_contact_point_separation_m=lower,
                engineering_distance_source='CURRENT_VERIFIED_BOX_EUCLIDEAN',
                phase_contact_permission='EXISTING_TARGET_STACK_STAGE' if permits else None))
        unresolved = self.active_reason
        observation = monitor.observe(time_s,post['boxes'][self.target],list(post['boxes'].values()),
            commanded_motion=commanded_motion,geometry=post,allow_free_space_transition=not unresolved)
        stop_reason = observation['reason'] if not observation['accepted'] else None
        if not permits:
            if any(p['intersection'] for p in post['pairs'].values()):
                stop_reason = stop_reason or 'STACK_GEOMETRY_INTERSECTION_OUTSIDE_CONTACT_PHASE'
            elif any(p['surface_distance_m']+1e-9 < self.policy.required_pair_clearance_m for p in post['pairs'].values()):
                stop_reason = stop_reason or 'ACTUAL_FREE_TRANSIT_STACK_CLEARANCE_LOST'
        if stop_reason:
            self.confirmed_violation = self.confirmed_violation or dict(step=self.step,reason=stop_reason)
        if self.confirmed_violation:
            stop_reason = self.confirmed_violation['reason']
        unresolved = self.active_reason
        self.hold = bool(unresolved)
        if self.hold:
            if self.hold_started is None:
                self.hold_started = float(time_s)
            if time_s-self.hold_started >= self.maximum_wait_s:
                stop_reason = stop_reason or unresolved
        else:
            self.hold_started = None

        nearest = min(post['pairs'], key=lambda n: post['pairs'][n]['surface_distance_m'], default=None)
        evidence_names = {self.target, nearest} - {None}
        evidence_names.update(self.actors[c[k]] for c in self.contacts for k in ('actor0','actor1'))
        row = dict(step=self.step, time_s=float(time_s), trajectory_time_s=self.trajectory_time_s,
            world_id=self.world_id, task_id=self.task_id, target=self.target, context=self.context,
            pre_time_s=self.pre['time_s'], pre_sample_phase=self.pre['sample_phase'],
            post_sample_phase=post['sample_phase'],
            pre_states={n:self.pre['states'][n] for n in evidence_names},
            post_states={n:post['states'][n] for n in evidence_names},
            pair_geometry={n:post['pairs'][n] for n in evidence_names if n != self.target},
            contacts=verdicts, nearest_neighbor=nearest, observation=observation,
            geometry_free_before=prior_free, geometry_free_after=monitor.free_space_reached,
            hold=self.hold, evidence_status=unresolved or 'RESOLVED', stop_reason=stop_reason)
        self.ring.append(row)
        self.counts['physical_steps'] += 1
        self.counts['contact_headers'] += len(self.contacts)
        self.counts[unresolved or 'RESOLVED'] += 1
        if self.first_conflict is not None and self.first_conflict['step'] == self.step:
            self.conflict_window = deepcopy(list(self.ring))
            self.conflict_tail = 12
        elif self.conflict_tail:
            self.conflict_window.append(deepcopy(row)); self.conflict_tail -= 1
        if not prior_free and monitor.free_space_reached:
            self.transitions.append(dict(step=self.step, time_s=float(time_s), from_free=False,to_free=True,
                clearance_m=observation['minimum_stack_clearance_m'],nearest_neighbor=nearest,
                shape_fingerprint=self.shape_fingerprint, policy_fingerprint=self.policy.fingerprint,
                contact_evidence_status=row['evidence_status'], basis='POST_STEP_VERIFIED_BOX_GEOMETRY'))
            self.transition_window = deepcopy(list(self.ring))
            self.transition_tail = 12
        elif self.transition_tail:
            self.transition_window.append(deepcopy(row)); self.transition_tail -= 1
        self.last = row
        self.previous_post = post
        return dict(observation=observation, hold=self.hold, stop_reason=stop_reason)

    def evidence(self):
        return deepcopy(dict(schema='m710_stack_clearance_step_v2', world_id=self.world_id,
            task_id=self.task_id, target=self.target, shapes=self.shapes, shape_fingerprint=self.shape_fingerprint,
            counts=dict(self.counts), transitions=self.transitions, transition_window=self.transition_window,
            conflict_window=self.conflict_window,
            last_steps=list(self.ring), first_conflict=self.first_conflict,
            confirmed_violation=self.confirmed_violation, issue_history=self.issue_history,
            active_issues=list(self.active_issues.values()), lifecycles=[dict(key=list(k),**v) for k,v in self.lifecycles.items()],
            transferred_pending=self.transferred_pending,
            pending=[{'key':list(k),'reason':v} for k,v in self.pending.items()]))
