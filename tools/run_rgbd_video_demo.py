"""Launch real RViz + read-only ROS + isolated resident algorithm, optionally record X11."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.finite_sequence import atomic_json,read_json
from unloading_perception.video_demo import recording_frames


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('recording','output','models','vision','algorithm-python'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--domain-id',type=int,default=177)
    p.add_argument('--record',action='store_true',help='Record the actual X11 desktop at original wall-clock speed')
    p.add_argument('--auto-start',action='store_true')
    p.add_argument('--auto-close',action='store_true',help='Close 15 seconds after finite processing completes')
    args=p.parse_args()
    if args.record and not args.auto_start:
        p.error('--record requires --auto-start so recording starts before the START gate')
    recording_frames(args.recording)
    args.output=args.output.resolve()
    if args.output.is_relative_to(args.recording.resolve().parent):raise ValueError('output must be outside the original recording')
    args.output.mkdir(exist_ok=False)
    (args.output/'receipts').mkdir()
    atomic_json(args.output/'progress.json',{'schema_version':'finite_capture_progress_v1','batch_id':uuid.uuid4().hex,
        'sequence_kind':'CONTINUOUS_RGBD_RECORDING','status':'RUNNING','exit_code':1,'target_task':None,'displayed_task':None,
        'delivery':None,'groups':[{'task_id':f'video_{i:02d}','status':'PENDING'} for i in range(2)]})
    env={**os.environ,'ROS_DOMAIN_ID':str(args.domain_id)}
    processes=[];logs=[];recorder=None;finished=None;exit_code=1
    def launch(name,command):
        log=(args.output/(name+'.log')).open('w',encoding='utf-8');logs.append(log)
        process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process);return process
    try:
        launch('mock',['ros2','run','unloading_ros_bridge','mock_state_publisher'])
        launch('world',['ros2','run','unloading_ros_bridge','world_bridge_node','--ros-args','-p','observation_mode:=replay_display_only'])
        launch('handoff',['ros2','run','unloading_ros_bridge','finite_replay_node','--ros-args',
            '-p','progress:='+str(args.output/'progress.json'),'-p','receipts:='+str(args.output/'receipts')])
        command=['ros2','run','unloading_ros_bridge','video_demo_node','--ros-args']
        # Resolving this symlink would bypass pyvenv.cfg and lose the GPU environment.
        for key,value in {'recording':args.recording.resolve(),'output':args.output,'project':ROOT,
                          'models':args.models.resolve(),'vision':args.vision.resolve(),'algorithm_python':args.algorithm_python.absolute()}.items():
            command+=['-p',key+':='+str(value)]
        launch('video',command)
        launch('rviz',['rviz2','-d',str(ROOT/'ros2_ws/src/unloading_bringup/rviz/rgbd_video_demo.rviz')])
        print('RViz ready/start gate:',args.output/'START',flush=True)
        started=False
        while True:
            if any(process.poll() is not None for process in processes):raise RuntimeError('a demo process exited; inspect its log')
            if recorder is not None and recorder.poll() is not None:raise RuntimeError('screen recorder exited unexpectedly')
            status=args.output/'display-status.json'
            if args.auto_start and not started and status.exists() and read_json(status)['image_subscribers']>0:
                if args.record:
                    log=(args.output/'record.log').open('w',encoding='utf-8');logs.append(log)
                    recorder=subprocess.Popen(['ffmpeg','-nostdin','-f','x11grab','-video_size','1920x1080',
                        '-framerate','10','-i',os.environ['DISPLAY'],'-c:v','libx264','-preset','veryfast','-crf','25',
                        '-pix_fmt','yuv420p','-progress',str(args.output/'record-progress.txt'),'-stats_period','0.1',
                        str(args.output/'demo-original-speed.mp4')],stdout=log,stderr=log)
                    deadline=time.monotonic()+20
                    progress=args.output/'record-progress.txt'
                    while not (progress.exists() and 'frame=' in progress.read_text(encoding='utf-8')):
                        if recorder.poll() is not None or time.monotonic()>deadline:raise RuntimeError('screen capture did not start')
                        time.sleep(.05)
                (args.output/'START').touch();started=True
            result=args.output/'demo-finished.json'
            if result.exists() and finished is None:
                finished=time.monotonic();exit_code=read_json(result)['exit_code']
                print('Finite processing finished, exit status',exit_code,flush=True)
            if finished is not None and time.monotonic()-finished>15:
                if recorder is not None:
                    recorder.send_signal(signal.SIGINT);recorder.wait(timeout=20);recorder=None
                    subprocess.run(['ffmpeg','-nostdin','-f','x11grab','-video_size','1920x1080','-i',os.environ['DISPLAY'],
                        '-frames:v','1',str(args.output/'rviz-final.png')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                if args.auto_close:break
            time.sleep(.1)
    except KeyboardInterrupt:exit_code=130
    finally:
        if recorder is not None:recorder.send_signal(signal.SIGINT);recorder.wait(timeout=20)
        for process in reversed(processes):
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for log in logs:log.close()
    return exit_code


if __name__=='__main__':raise SystemExit(main())
