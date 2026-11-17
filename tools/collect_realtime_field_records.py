"""Observation/planning-only field collector for the November 17 review.

Timing is enabled before importing the workbench. Motion submission, task
execution and the common valve's command sequence are blocked independently.
No experimental planning option is enabled. --ui-smoke never connects IO.
"""
import argparse
import json
import os
import subprocess
from pathlib import Path
os.environ['ECO65_TIMING']='1'
from unloading_sim.workbench.single_carton import ui

BLOCK='REVIEW_COLLECTION_ONLY: 本入口仅采集观测及规划；运动与阀门指令已禁用'


def refuse(*args,**kwargs):
    raise RuntimeError(BLOCK)


class CollectionService(ui.ExecutionService):
    submit=refuse
    # No trajectory can be submitted by this service. Cancellation affects
    # planning only, so it must never send the robot a physical stop command.
    def request_stop(self):pass


class CollectionVacuum(ui.VacuumOwner):
    _sequence=refuse
    establish=refuse
    release=refuse


class CollectionTaskManager(ui.TaskManager):
    execute=refuse


class CollectionWorkbench(ui.SingleWorkbench):
    execute_cycle=refuse
    open_config=refuse
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.root.title('ECO65 第 0–7 步现场核验采集 · 只观测与规划 · 禁止执行')
        self.execute_button.configure(text='采集模式：执行禁用',state='disabled')
        self.notice.set(BLOCK)
    def _update_task_once(self):
        super()._update_task_once()
        if self._closed:return
        self.execute_button.configure(state='disabled')
        if '核验采集：禁止执行' not in self.execute_blockers:
            self.execute_blockers.append('核验采集：禁止执行')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ui.ROOT/'configs/workbench/single_carton.example.json')
    parser.add_argument('--record-root',type=Path,default=ui.ROOT/'outputs/realtime-field-20261117')
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--ui-smoke',action='store_true')
    args=parser.parse_args()
    config=ui.load(args.config)
    experimental=('planning_deadline_s','resident_workers','scene_handles','async_evidence','path_endpoint_reuse',
                  'binding_hash_cache','collision_check_mode','sent_path_audit','planning_module')
    enabled=[k for k in experimental if config.get(k) not in (None,False,'PYTHON','EXACT','SAMPLED')]
    if enabled:raise ValueError('COLLECTION_REQUIRES_DEFAULT_PLANNING_OPTIONS:'+str(enabled))
    config['record_root']=str(args.record_root.resolve());config['execution']['record_root']=config['record_root']
    state=dict(source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ui.ROOT,text=True).strip(),timing=True,
               record_root=config['record_root'],experimental_options_enabled=enabled,
               motion_submission='BLOCKED',vacuum_commands='BLOCKED',physical_acceptance='NOT_RUN',issues=ui.issues(config))
    print(json.dumps(state,ensure_ascii=False,indent=2),flush=True)
    if args.check_only:return
    if state['issues']:raise ValueError('COLLECTION_CONFIG_INVALID:'+str(state['issues']))
    installation=ui.Installation.load(config['execution']['installation'])
    model=ui.CoordinateModel(ui.URDF,ui.URDF_SHA,installation.world_from_base,installation.tool)
    calibration=ui.Extrinsics.load(config['camera_extrinsics']);installation.check_camera(calibration)
    mapping=ui.load_mapping(config['execution']['mapping'])
    authority=ui.Authority('eco65_single_carton_segment_v1');planner=ui.SinglePlanner(config,mapping,authority,model)
    service=CollectionService(mapping,config['execution'],authority,planner.binding);vacuum=CollectionVacuum(config['vacuum'])
    backend=ui.SingleBackend(ui.D435iObserver(),ui.NetworkedRobotView(service,mapping,config.get('robot_network',{})),ui.VacuumView(vacuum),service)
    ui.TaskManager=CollectionTaskManager
    root=ui.tk.Tk();workbench=CollectionWorkbench(root,backend,planner,service,vacuum,config,model,calibration,auto_start=not args.ui_smoke)
    if args.ui_smoke:root.after(700,workbench.close)
    root.mainloop()


if __name__=='__main__':main()
