"""Continuous multi-carton job: settings, target order and the job loop with fakes.

No device, no motion and no valve command exist in these tests.
"""
import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.workbench.continuous import job as J
from unloading_sim.workbench.single_carton.budget import selected_targets


ORDER = dict(mode='TOP_LAYER_FIRST_CENTRE_OUT', layer_tolerance_m=0.05, basis='test')


def config(tmp_path, **job):
    value = dict(max_cartons=3, receiver_clearing=dict(mode='USER_CONFIRMED_SELF_CLEARING', clearing_time_s=0, basis='test'))
    value.update(job)
    return dict(record_root=str(tmp_path), start_joint_check='ENABLED', continuous_job=value)


def box(index, centre, size=(0.15, 0.10, 0.10)):
    c = np.asarray(centre, float)
    h = np.asarray(size, float) / 2
    corners = [list(c + np.array([sx, sy, sz]) * h) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return dict(detection_index=index, id=f'carton-{index}', corners_m=corners, measured_box=dict(corners_m=corners))


def front_face(index, centre, width=0.15, height=0.10):
    # A face looking toward the robot (-X from the stack), camera at the origin.
    return dict(detection_index=index, reason='HIDDEN_EXTENT_NOT_BOUNDED_BY_CURRENT_RGBD',
                evidence=dict(observed_face=dict(center_m=list(centre), normal=[1., 0., 0.], u=[0., 1., 0.], v=[0., 0., 1.],
                                                 size_m=[width, height])))


# -- settings ---------------------------------------------------------------------

def test_settings_require_receiver_policy_and_basis(tmp_path):
    c = config(tmp_path)
    del c['continuous_job']['receiver_clearing']
    with pytest.raises(ValueError, match='RECEIVER_CLEARING_POLICY_REQUIRED'):
        J.settings(c)
    c = config(tmp_path, receiver_clearing=dict(mode='USER_CONFIRMED_SELF_CLEARING', clearing_time_s=2))
    with pytest.raises(ValueError, match='RECEIVER_CLEARING_POLICY_REQUIRED'):
        J.settings(c)
    with pytest.raises(ValueError, match='SENSOR_OBSERVED_CLEAR_UNAVAILABLE'):
        J.settings(config(tmp_path, receiver_clearing=dict(mode='SENSOR_OBSERVED_CLEAR', basis='x')))
    with pytest.raises(ValueError, match='RECEIVER_CLEARING_TIME_INVALID'):
        J.settings(config(tmp_path, receiver_clearing=dict(mode='USER_CONFIRMED_SELF_CLEARING', basis='x')))


def test_settings_defaults_and_validity_bound(tmp_path):
    s = J.settings(config(tmp_path))
    assert s['plan_validity_s'] == 30.0 and '30 s' in s['plan_validity_basis']
    assert s['target_order']['mode'] == 'TOP_LAYER_FIRST_CENTRE_OUT'
    with pytest.raises(ValueError, match='CONTINUOUS_PLAN_VALIDITY_INVALID'):
        J.settings(config(tmp_path, plan_validity_s=31))
    with pytest.raises(ValueError, match='UNKNOWN_KEYS'):
        J.settings(config(tmp_path, lookahead=True))


def test_apply_to_config_only_tightens_plan_validity(tmp_path):
    c = config(tmp_path)
    J.apply_to_config(c)
    assert c['plan_validity_s'] == 30.0 and c['plan_validity_basis']
    c = config(tmp_path, plan_validity_s=20)
    c['plan_validity_s'] = 10
    J.apply_to_config(c)
    assert c['plan_validity_s'] == 10


def test_start_joint_check_must_be_enabled(tmp_path):
    c = config(tmp_path)
    c['start_joint_check'] = 'DISABLED_BY_USER'
    assert any('START_JOINT_CHECK_MUST_BE_ENABLED' in e for e in J.config_issues(c))
    assert J.config_issues(config(tmp_path)) == []


# -- target order -------------------------------------------------------------------

def test_top_layer_first_then_centre_out():
    # Stack in front of the robot along -X; lateral axis is Y.
    boxes = [box(0, (-0.5, 0.30, 0.05)), box(1, (-0.5, 0.15, 0.05)), box(2, (-0.5, 0.0, 0.05)),
             box(3, (-0.5, -0.15, 0.05)), box(4, (-0.5, -0.30, 0.05)), box(5, (-0.5, 0.60, 0.15))]
    r = J.target_order(boxes, [], ORDER)
    # 5 is the only higher box and rests on nothing measured; then the bottom row centre-out.
    assert r['order'][0] == 5
    assert r['order'][1] == 2
    assert set(r['order'][2:4]) == {1, 3} and set(r['order'][4:]) == {0, 4}
    assert r['order'][2:4] == [1, 3]  # equal offsets: lower detection index first
    assert r['skipped'] == []


def test_box_under_another_is_not_searched():
    lower = box(0, (-0.5, 0.0, 0.05))
    upper = box(1, (-0.5, 0.02, 0.15))
    r = J.target_order([lower, upper], [], ORDER)
    assert r['order'] == [1]
    assert r['skipped'] == [dict(detection_index=0, reason='CARTON_RESTING_ON_TOP', above=[1])]


def test_unresolved_face_above_blocks_lower_box_with_hidden_extent():
    lower = box(0, (-0.5, 0.0, 0.05))
    # Only the upper carton's front face is seen, flush with the lower front (x = -0.425, robot side).
    upper = front_face(1, (-0.425, 0.0, 0.15))
    r = J.target_order([lower], [upper], ORDER, camera_position=np.array([0., 0., 0.4]))
    assert r['order'] == [] and r['skipped'][0]['above'] == [1]
    # Workspace-ignored detections carry no face and never block.
    ignored = dict(detection_index=2, reason='OUTSIDE_WORKSPACE_IGNORED', evidence=dict(valid_depth_points=0))
    assert J.target_order([lower], [ignored], ORDER)['order'] == [0]


def test_side_by_side_neighbours_do_not_block_each_other():
    r = J.target_order([box(0, (-0.5, 0.076, 0.05)), box(1, (-0.5, -0.076, 0.05))], [front_face(2, (-0.425, 0, -0.05), 0.3)],
                       ORDER, camera_position=np.array([0., 0., 0.4]))
    assert r['order'] == [0, 1] and r['skipped'] == []


def test_selected_targets_keeps_order_and_rejects_unknown_index():
    boxes = [dict(detection_index=i, id=str(i)) for i in range(3)]
    assert [b['detection_index'] for b in selected_targets(boxes, [2, 0])] == [2, 0]
    assert [b['detection_index'] for b in selected_targets(boxes, 1)] == [1]
    assert len(selected_targets(boxes, None)) == 3
    with pytest.raises(ValueError, match='TARGET_ORDER_INVALID'):
        selected_targets(boxes, [3])
    with pytest.raises(ValueError, match='TARGET_ORDER_INVALID'):
        selected_targets(boxes, [1, 1])


# -- the job loop ---------------------------------------------------------------------

class FakeCycle:
    def __init__(self, final='COMPLETE', dry_run=None):
        self.state = 'APPROACH'
        self.final = final
        self.reason = '' if final == 'COMPLETE' else 'SIMULATED_FAULT'
        self.dry_run = dry_run
        self.release_confirmed = False

    def tick(self):
        if self.state not in ('COMPLETE', 'FAILED', 'CANCELLED', 'STOP_UNCONFIRMED'):
            self.state = self.final

    def status(self):
        return dict(state=self.state, physical_result='NOT_COMPLETE', release_confirmed=self.release_confirmed,
                    cycle_release_command_sent=self.final == 'COMPLETE' and self.dry_run is None)


class FakeManager:
    """Scripted stand-in for TaskManager: frames are lists of (boxes, rejected, detections)."""

    def __init__(self, frames, *, plan_fail=(), cycle_final=None, expire=(), q_offsets=None):
        self.frames = list(frames)
        self.plan_fail = set(plan_fail)
        self.cycle_final = dict(cycle_final or {})
        self.expire = set(expire)
        self.q_offsets = dict(q_offsets or {})
        self.backend = SimpleNamespace(observation_generation=1, device_state=lambda k: '数据有效')
        self.vacuum = SimpleNamespace(holding=False)
        self.cycle = None
        self.observation = None
        self.result = None
        self.boxes = []
        self.rejected = []
        self.phase = 'IDLE'
        self.reason = ''
        self.plan = None
        self.task_id = None
        self.active = False
        self.observed = 0
        self.planned = []
        self.executed = 0
        self.stopped = 0
        self.standing_observer = None
        self.stop_event = threading.Event()

    def observe(self):
        self.observed += 1
        boxes, rejected, detections = self.frames.pop(0)
        offset = self.q_offsets.get(self.observed, 0.)
        frame = SimpleNamespace(stamp=SimpleNamespace(received_monotonic_ns=time.monotonic_ns()),
                                observation_id=f'obs-{self.observed}')
        self.observation = SimpleNamespace(frame=frame, q_start=[offset, 0, 0, 0, 0, 0], world_camera=np.eye(4).tolist())
        self.result = SimpleNamespace(detections=[{}] * detections)
        self.boxes, self.rejected = boxes, rejected
        self.phase = 'TARGET_READY' if boxes else 'GEOMETRY_UNRESOLVED'
        self.task_id = None

    def prepare_plan(self, selected):
        self.task_id = f'task-{self.observed}'
        self.planned.append(list(selected))
        if self.observed in self.plan_fail:
            raise ValueError('NO_PLAN_FOR_ANY_TARGET')
        target = next(b for b in self.boxes if b['detection_index'] == selected[0])
        self.plan = dict(target=target)
        self.phase = 'PLAN_READY'

    def plan_remaining_s(self):
        return 10.

    def execute(self):
        if self.observed in self.expire:
            self.expire.discard(self.observed)
            raise ValueError('PLAN_EXPIRED_REPLAN')
        self.executed += 1
        self.cycle = FakeCycle(self.cycle_final.get(self.executed, 'COMPLETE'))
        threading.Timer(0.01, self.cycle.tick).start()

    def stop(self):
        self.stopped += 1


def frame(*indices):
    return ([box(i, (-0.5, 0.15 * i, 0.05)) for i in indices], [], len(indices))


def run(job, timeout=5.):
    J.ContinuousJob.POLL_S = 0.002
    job.start()
    job.thread.join(timeout)
    assert not job.running
    return job.status()


def events(job):
    return [json.loads(line) for line in (job.folder / 'events.jsonl').read_text().splitlines()]


def test_job_runs_until_count_and_records_each_round(tmp_path):
    m = FakeManager([frame(0, 1, 2), frame(1, 2), frame(2)])
    job = J.ContinuousJob(m, config(tmp_path, max_cartons=2))
    status = run(job)
    assert status['end']['reason'] == 'CARTON_COUNT_REACHED'
    assert status['completed'] == 2 and m.executed == 2 and m.observed == 2
    assert m.planned == [[1, 0, 2], [1, 2]]
    record = json.loads((job.folder / 'job.json').read_text())
    assert record['task_ids'] == ['task-1', 'task-2']
    assert record['physical_result'] == 'NOT_RUN'
    assert record['rounds'][0]['target_order']['order'] == [1, 0, 2]
    kinds = [e['kind'] for e in events(job)]
    assert kinds[0] == 'job_started' and kinds[-1] == 'job_ended'
    assert kinds.count('receiver_clearing') == 1


def test_job_ends_when_nothing_is_detected(tmp_path):
    m = FakeManager([frame(0), ([], [], 0)])
    status = run(J.ContinuousJob(m, config(tmp_path)))
    assert status['end']['reason'] == 'NO_TARGET_DETECTED' and status['completed'] == 1


def test_job_ends_when_detections_are_not_measurable(tmp_path):
    m = FakeManager([([], [front_face(0, (-0.575, 0, 0.05))], 1)])
    status = run(J.ContinuousJob(m, config(tmp_path)))
    assert status['end']['reason'] == 'NO_PLANNABLE_TARGET' and m.executed == 0


def test_planning_failure_ends_the_job_without_motion(tmp_path):
    m = FakeManager([frame(0, 1)], plan_fail={1})
    job = J.ContinuousJob(m, config(tmp_path))
    status = run(job)
    assert status['end']['reason'] == 'NO_COMPLETE_PLAN' and m.executed == 0
    assert status['rounds'][0]['planning'] == 'FAILED'
    assert any(e['kind'] == 'round_planning_failed' for e in events(job))


def test_failed_cycle_ends_the_job(tmp_path):
    m = FakeManager([frame(0, 1), frame(1)], cycle_final={1: 'FAILED'})
    status = run(J.ContinuousJob(m, config(tmp_path)))
    assert status['end']['reason'] == 'CYCLE_NOT_COMPLETE' and m.observed == 1


def test_moved_start_ends_the_job(tmp_path):
    m = FakeManager([frame(0, 1), frame(1)], q_offsets={2: 0.01})
    status = run(J.ContinuousJob(m, config(tmp_path)))
    assert status['end']['reason'] == 'START_MOVED' and m.executed == 1


def test_expired_plan_reobserves_and_replans(tmp_path):
    m = FakeManager([frame(0), frame(0), frame(1)], expire={1})
    status = run(J.ContinuousJob(m, config(tmp_path, max_cartons=2)))
    assert status['end']['reason'] == 'CARTON_COUNT_REACHED'
    assert m.observed == 3 and m.executed == 2
    assert [r['round'] for r in status['rounds']] == [1, 2]


def test_execute_gate_blocks_motion(tmp_path):
    m = FakeManager([frame(0)])
    status = run(J.ContinuousJob(m, config(tmp_path), execute_gate=lambda: ['控制器报警']))
    assert status['end']['reason'] == 'EXECUTION_REFUSED' and m.executed == 0


def test_session_change_between_rounds_ends_the_job(tmp_path):
    m = FakeManager([frame(0, 1), frame(1)])
    original = m.execute

    def execute_then_reconnect():
        original()
        m.backend.observation_generation = 2
    m.execute = execute_then_reconnect
    status = run(J.ContinuousJob(m, config(tmp_path)))
    assert status['end']['reason'] == 'SESSION_CHANGED' and m.observed == 1


def test_operator_stop_during_clearing_wait(tmp_path):
    m = FakeManager([frame(0, 1), frame(1)])
    J.ContinuousJob.POLL_S = 0.002
    job = J.ContinuousJob(m, config(tmp_path, receiver_clearing=dict(mode='USER_CONFIRMED_SELF_CLEARING', clearing_time_s=30, basis='t')))
    job.start()
    deadline = time.monotonic() + 5
    while job.status()['state'] != 'RECEIVER_CLEARING' and time.monotonic() < deadline:
        time.sleep(0.005)
    job.stop()
    job.thread.join(5)
    status = job.status()
    assert status['end']['reason'] == 'OPERATOR_STOP' and m.observed == 1 and m.stopped == 1


def test_operator_confirm_each_waits_for_the_click(tmp_path):
    m = FakeManager([frame(0, 1), frame(1)])
    J.ContinuousJob.POLL_S = 0.002
    job = J.ContinuousJob(m, config(tmp_path, max_cartons=2, receiver_clearing=dict(mode='OPERATOR_CONFIRM_EACH', basis='t')))
    job.start()
    deadline = time.monotonic() + 5
    while job.status()['state'] != 'WAIT_OPERATOR_CLEAR' and time.monotonic() < deadline:
        time.sleep(0.005)
    assert m.observed == 1
    job.confirm_receiver_clear()
    job.thread.join(5)
    assert job.status()['end']['reason'] == 'CARTON_COUNT_REACHED' and m.observed == 2


def test_dry_run_ends_after_one_round(tmp_path):
    m = FakeManager([frame(0, 1), frame(1)])
    original = m.execute

    def dry_execute():
        original()
        m.cycle.dry_run = dict(mode='NO_VACUUM')
    m.execute = dry_execute
    status = run(J.ContinuousJob(m, config(tmp_path)))
    assert status['end']['reason'] == 'DRY_RUN_NO_RELEASE' and m.executed == 1


def test_start_refuses_held_load_and_active_task(tmp_path):
    m = FakeManager([frame(0)])
    m.vacuum.holding = True
    with pytest.raises(ValueError, match='持箱'):
        J.ContinuousJob(m, config(tmp_path)).start()
    m.vacuum.holding = False
    m.active = True
    with pytest.raises(ValueError, match='活动任务'):
        J.ContinuousJob(m, config(tmp_path)).start()
