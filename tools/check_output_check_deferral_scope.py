"""Real native worker: output-check deferral is refused outside physical single carton.

Starts the controlled-empty planner's native worker (policy scope
CONTROLLED_EMPTY_NO_CONTACT), plans one tiny residence motion from a fixed
regression posture with and without defer_output_check_to_sent_path, and
requires: without the field, the worker's own waypoint check runs (PASS);
with it, the worker refuses the request (OUTPUT_CHECK_DEFERRAL_SCOPE_INVALID).
Planning only: no hardware import, no permit, no motion or valve command.
"""
import json
from pathlib import Path
import sys
import numpy as np
from unloading_sim.workbench.controlled.planner import EmptyPlanner, load_config
from unloading_sim.workbench.controlled.trajectory import Authority
from unloading_sim.workbench.ui import load_mapping, ROOT
from unloading_sim.moveit2_backend import ResidentMoveItClient

out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT/'outputs/single_carton_development/deferral-scope-check'
out.mkdir(parents=True, exist_ok=True)
config = load_config(ROOT/'configs/workbench/controlled.example.json')
planner = EmptyPlanner(config, load_mapping(config['mapping']), Authority('eco65_controlled_empty_package_v1'))
planner.build()
client = ResidentMoveItClient(['env', 'ROS_LOG_DIR='+str(out), str(ROOT/'ros2/m710_moveit_backend/worker.sh')], log_path=out/'moveit.log')
report = dict(scope='NATIVE_DEFERRAL_SCOPE_CHECK_PLANNING_ONLY_NOT_EXECUTABLE', hardware_execution='NOT_RUN')
try:
    ready = client.request(planner.init)
    report['capability_single_carton_output_check_deferral'] = ready['capabilities'].get('single_carton_output_check_deferral')
    report['policy_scope'] = planner.init['identity']['policy_scope']
    q = np.array([1.5112456927168503, 0.27872908154349446, -1.207785295672596, -0.536165146212658, 1.5139334997649214, 1.3463469849884258])
    goal = q.copy(); goal[0] += .01
    base = planner.request(q, op='plan', q_goal=goal.tolist(), pipeline_id='pilz_industrial_motion_planner', planner_id='PTP',
                           velocity_scale=1., acceleration_scale=1., allowed_planning_time_s=20., request_budget_s=110.)
    normal = client.request(base, timeout=120)
    report['without_field'] = dict(status=normal.get('status'), native_output_status=normal.get('native_output_status'))
    try:
        refused = client.request(dict(base, defer_output_check_to_sent_path=True), timeout=120)
        report['with_field'] = dict(status=refused.get('status'), native_output_status=refused.get('native_output_status'))
    except Exception as error:
        report['with_field'] = dict(error=str(error))
finally:
    client.close()
ok = (report.get('capability_single_carton_output_check_deferral') is False and report['without_field']['native_output_status'] == 'PASS'
      and 'OUTPUT_CHECK_DEFERRAL_SCOPE_INVALID' in report['with_field'].get('error', ''))
report['result'] = 'PASS' if ok else 'FAIL'
(out/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False))
print(json.dumps(report, ensure_ascii=False))
sys.exit(0 if ok else 1)
