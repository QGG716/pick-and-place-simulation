"""The actual numeric acceptance CLI must fail closed and keep mismatch reports."""
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import check_native_prepare as tool
import check_planning_deadline as setup
from unloading_sim.workbench.controlled.trajectory import Limits


@pytest.mark.parametrize('mode', ['identical', 'numeric_mismatch', 'decision_mismatch', 'worker_error'])
def test_prepare_cli_reports_failure_and_closes_worker(tmp_path, monkeypatch, mode):
    point = dict(t=0., q=[0.]*6, v=[0.]*6, a=[0.]*6)
    points = [point, dict(point, t=1.)]
    py = dict(status='PREPARED', t=[0., 1.], q=[[0.]*6]*2, v=[[0.]*6]*2, a=[[0.]*6]*2,
              collision_path=[[0.]*6]*2, analytic_speed_bounds_rad_s=[0.]*6,
              subdivisions_per_send=1, send_duration_s=1., scaled_duration_s=1.,
              source_duration_s=1., terminal_hold_s=0.)
    native = dict(status='PREPARED', prepared={k:v for k,v in copy.deepcopy(py).items() if k!='status'})
    if mode == 'numeric_mismatch':native['prepared']['a'][0] = [2e-12]+[0.]*5
    if mode == 'decision_mismatch':native = dict(status='PREPARE_REJECTED', error='INTERPOLATION_LIMIT:1:2')
    closed = []
    class Client:
        def __init__(self, *a, **k):pass
        def request(self, request, **kwargs):
            if request.get('op') != 'sc_prepare':return dict(capabilities=dict(native_prepare=True))
            if mode == 'worker_error':raise RuntimeError('WORKER_FAILED_INJECTED')
            return native
        def close(self):closed.append(True)
    planner = SimpleNamespace(init=dict(op='init', identity={}), limits=Limits((-3.,)*6,(3.,)*6,(.1,)*6,(.2,)*6),
                              config=dict(period_s=.005), site=dict(time_scale=1.), reach_bound=1.)
    monkeypatch.setattr(setup, 'make_planner', lambda *a: (planner, None))
    monkeypatch.setattr('unloading_sim.moveit2_backend.ResidentMoveItClient', Client)
    monkeypatch.setattr(tool, 'collect', lambda *a: [dict(source='fake-record', points=points, request={})])
    monkeypatch.setattr(tool, 'python_prepare', lambda *a: (py, .01))
    output = tmp_path/'report.json'
    monkeypatch.setattr(sys, 'argv', ['check_native_prepare', '--records', str(tmp_path), '--output', str(output)])
    if mode == 'worker_error':
        with pytest.raises(RuntimeError, match='WORKER_FAILED_INJECTED'):tool.main()
    else:
        code = tool.main()
        report = json.loads(output.read_text())
        assert code == (0 if mode == 'identical' else 1)
        assert report['acceptance']['all_identical'] == (mode == 'identical')
        assert report['tolerance'] == 1e-12
        assert all(r['time_scale'] >= 1. and r['period_s'] == .005 for r in report['rows'])
        if mode == 'decision_mismatch':
            assert report['numeric_compared'] == 0
            assert report['worst_max_abs']['a'] is None
    assert closed == [True]
