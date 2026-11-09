"""Read-only regression of the existing ROS/MoveIt worker startup and model FK."""
import argparse
import json
from pathlib import Path
import numpy as np
from unloading_sim.eco65.model import robot_for
from unloading_sim.workbench.scene import load_snapshot
from unloading_sim.moveit2_backend import ResidentMoveItClient


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot', type=Path, required=True)
    p.add_argument('--worker-command', required=True)
    p.add_argument('--scene-input-root', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    snapshot = load_snapshot(a.snapshot, a.scene_input_root)
    init = snapshot['native_model']['init']
    robot = robot_for(snapshot['tool'], snapshot['scene'])
    a.output.parent.mkdir(parents=True, exist_ok=True)
    client = ResidentMoveItClient(a.worker_command, log_path=a.output.with_suffix('.log'))
    report = {'hardware_validation':'NOT_RUN','physical_execution':'UNAVAILABLE','checks':[]}
    try:
        ready = client.request(init, timeout=50)
        assert ready['status'] == 'READY' and ready['identity'] == init['identity']
        report['ready'] = ready
        for degrees in ([0,0,0,0,0,0], [20,-30,45,10,25,-15], [-35,15,-40,20,-30,45]):
            q = np.deg2rad(degrees)
            expected = {**robot.named_link_frames(q), init['identity']['task_tcp_link']:robot.fk(q)}
            got = client.request({'op':'fk','identity':init['identity'],'q_start':q.tolist(),'links':list(expected)},timeout=30)
            error = max(float(np.max(np.abs(np.asarray(got['frames'][name])-value))) for name,value in expected.items())
            assert error < 1e-8, error
            report['checks'].append({'q_deg':degrees,'max_matrix_error':error})
        report['status'] = 'PASS'
    finally:
        client.close()
        a.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({'status':report['status'],'checks':report['checks']}))


if __name__ == '__main__': main()
