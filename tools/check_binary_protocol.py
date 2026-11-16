"""Real worker: binary frames carry the same content as JSON lines; binary cancel frames work (no hardware).

1. One recorded sent-path validate request is sent to a JSON-line worker and to
   a binary-frame worker: replies must be identical apart from timing fields.
2. A resident (M710_WORKER_CANCEL=1) binary worker gets the same request with a
   cancel callback that fires after 0.2 s: the reply must come back cancelled
   by request id, and the worker must answer the next request normally.

Usage: python tools/check_binary_protocol.py --record FILE --output OUT.json
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
VOLATILE = {'request_id', 'transport_and_worker_s', 'request_wall_s', 'budget', 'native_timing', 'seconds', 'profile'}


def strip(value):
    if isinstance(value, dict):
        return {k: strip(v) for k, v in value.items() if k not in VOLATILE and not k.endswith('_s')}
    if isinstance(value, list):
        return [strip(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--record', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import check_planning_deadline as tool
    from unloading_sim.moveit2_backend import ResidentMoveItClient, MoveItUnavailable
    from unloading_sim.workbench.single_carton.resident import CancellableWorkerClient
    planner, _ = tool.make_planner(ROOT/'configs/workbench/single_carton.example.json')
    request = json.loads(args.record.read_text())['request']
    request = {k: v for k, v in request.items() if k not in ('request_id', 'absolute_deadline_monotonic_s',
                                                             'deadline_clock_reference_monotonic_s')}
    request['request_budget_s'] = 600.
    command = ['env', 'ROS_LOG_DIR=/tmp/binary-protocol', str(ROOT/'ros2/m710_moveit_backend/worker.sh')]
    report = dict(schema='eco65_binary_protocol_check_v1', record=str(args.record), probe_states=len(request.get('probe_path', [])))
    text = ResidentMoveItClient(command, log_path='/tmp/binary-protocol-text.log')
    binary = ResidentMoveItClient(command, log_path='/tmp/binary-protocol-binary.log', binary=True)
    try:
        text.request(planner.init, timeout=180); binary.request(planner.init, timeout=180)
        report['binary_active'] = binary.binary_active
        t0 = time.perf_counter(); a = text.request(request, timeout=900); ta = time.perf_counter()-t0
        t0 = time.perf_counter(); b = binary.request(request, timeout=900); tb = time.perf_counter()-t0
        report.update(identical_replies=strip(a) == strip(b), json_round_trip_s=ta, binary_round_trip_s=tb,
                      json_bytes=text.transfer_bytes(), binary_bytes=binary.transfer_bytes(),
                      native_valid=(a.get('native_valid'), b.get('native_valid')))
    finally:
        text.close(); binary.close()
    resident = CancellableWorkerClient(command, log_path='/tmp/binary-protocol-resident.log', name='binary-check', binary=True)
    try:
        resident.request(planner.init, timeout=180)
        report['resident_binary_active'] = resident.binary_active
        started = time.perf_counter(); fire = started+.2
        try:
            resident.request(request, timeout=900, cancelled=lambda: time.perf_counter() >= fire)
            report['cancel'] = 'NOT_CANCELLED_COMPLETED_FIRST'
        except MoveItUnavailable as error:
            response = getattr(error, 'response', None) or {}
            report['cancel'] = dict(error=str(error), cancelled_by_request=response.get('cancelled_by_request'),
                                    reply_after_s=time.perf_counter()-started)
        small = dict(request, probe_path=request['probe_path'][:3]) if 'probe_path' in request else request
        after = resident.request(small, timeout=120)
        report['after_cancel'] = dict(status=after.get('status'), native_valid=after.get('native_valid'), alive=resident.alive())
    finally:
        resident.close()
    report['acceptance'] = dict(identical_replies=report.get('identical_replies'),
                                cancelled_by_request=isinstance(report.get('cancel'), dict) and report['cancel'].get('cancelled_by_request') is True,
                                usable_after_cancel=report.get('after_cancel', {}).get('alive') is True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=str))
    print(json.dumps(report, indent=1, default=str))


if __name__ == '__main__':
    main()
