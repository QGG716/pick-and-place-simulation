#!/usr/bin/env python3
"""Offline native-plan diagnostic. NO hardware connection, permit or execution."""
import argparse
import json
from pathlib import Path
from unloading_sim.workbench.controlled.planner import ROOT,EmptyPlanner,load_config
from unloading_sim.workbench.controlled.trajectory import Authority,prepare
from unloading_sim.workbench.ui import load_mapping
from unloading_sim.moveit2_backend import ResidentMoveItClient

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'configs/workbench/controlled.example.json')
    p.add_argument('--start-rad',type=float,nargs=6,required=True)
    p.add_argument('--goal-rad',type=float,nargs=6,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();c=load_config(a.config);planner=EmptyPlanner(c,load_mapping(c['mapping']),Authority());planner.build()
    a.output.mkdir(parents=True,exist_ok=True)
    client=ResidentMoveItClient(['env','ROS_LOG_DIR='+str(a.output.resolve()),str(ROOT/'ros2/m710_moveit_backend/worker.sh')],log_path=a.output/'native.log')
    report=dict(scope='OFFLINE_EXPLICIT_JOINT_INPUT_NOT_LIVE',hardware_execution='NOT_RUN',binding=planner.binding())
    try:
        report['init']=client.request(planner.init)
        result=client.request(planner.request(a.start_rad,op='plan',q_goal=a.goal_rad,pipeline_id='pilz_industrial_motion_planner',planner_id='PTP',velocity_scale=1.,acceleration_scale=1.,allowed_planning_time_s=20.),timeout=120)
        report['native_plan']=result
        if result.get('status')!='SUCCESS':raise RuntimeError(result.get('status'))
        prepared=prepare(result['points'],planner.limits,10,reach_bound_m=planner.reach_bound)
        (a.output/'sent-trajectory-NOT-EXECUTABLE.json').write_text(json.dumps(prepared))
        checked=client.request(planner.request(a.start_rad,op='validate',probe_path=prepared['collision_path']),timeout=600)
        report['sent_path_check']=checked
        report['status']='PASS' if checked.get('native_valid') and checked.get('legacy_intersection_valid') and checked.get('path_failure') is None else 'FAIL'
        print(json.dumps(dict(status=report['status'],source_points=len(result['points']),sent_points=len(prepared['points']),duration_s=prepared['send_duration_s'],hardware_execution='NOT_RUN')))
        if report['status']!='PASS':raise RuntimeError('SENT_PATH_REJECTED')
    except Exception as e:report['error']=str(e);raise
    finally:
        client.close();(a.output/'report.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
