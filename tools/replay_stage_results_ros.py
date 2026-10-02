"""Read-only display of current completed stage runs using existing ROS admission.

Run with ROS Python after sourcing Humble and the existing bridge install.
No algorithm worker, execution bridge or action publisher is started here.
"""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src'),
              str(ROOT/'ros2_ws/src/unloading_ros_bridge')]
from unloading_perception.finite_sequence import atomic_json,verify_result,read_json
from run_finite_workcell_sequence import RosDelivery


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--domain',type=int,default=181)
    parser.add_argument('--font',type=Path,help='existing local CJK font; never installed globally')
    args=parser.parse_args(argv)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    frozen=read_json(args.run/'frozen_plan.json');finished=read_json(args.run/'progress.json')
    if finished['status']!='COMPLETED':raise ValueError('NEW_RUN_NOT_COMPLETED')
    args.output.mkdir(exist_ok=False);(args.output/'receipts').mkdir()
    rows=[dict(task_id=r['name'],status='ARTIFACT_READY',capture=r['capture'],artifact=r['artifact']) for r in finished['groups']]
    for row in rows:
        verify_result(args.run/row['task_id']/'algorithm/summary.json',frozen['inputs'][row['task_id']],
                      frozen['models'],frozen['effective_config'],expected_output=args.run/row['task_id']/'algorithm')
    state=dict(schema_version='finite_capture_progress_v1',batch_id=uuid.uuid4().hex,
        sequence_kind='FIXED_NEW_ALGORITHM_RESULTS_HISTORICAL_SOURCE',groups=rows,target_task=None,
        displayed_task=None,delivery=None,status='RUNNING',enable_hardware=False)
    atomic_json(args.output/'progress.json',state)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image,PointCloud2
    from tf2_msgs.msg import TFMessage
    from visualization_msgs.msg import MarkerArray,Marker
    from unloading_ros_bridge.marker_display import marker_qos
    from unloading_ros_bridge.video_demo_node import VideoDemoNode
    from PIL import Image as PILImage,ImageDraw,ImageFont
    import subprocess
    fontpath=str(args.font) if args.font else subprocess.check_output(['fc-match','-f','%{file}',':lang=zh'],text=True).strip()
    font=ImageFont.truetype(fontpath,22);small=ImageFont.truetype(fontpath,17)
    class Display(Node):
        image=VideoDemoNode.image
        frame_cloud=VideoDemoNode.frame_cloud
        markers=VideoDemoNode.markers
        def __init__(self):
            super().__init__('stage_results_display')
            self.switching=True;self.latest_markers=None;self.display_keys=set();self.current=None;self.dashboard=None
            self.image_pub=self.create_publisher(Image,'/demo/dashboard',1)
            self.cloud_pub=self.create_publisher(PointCloud2,'/demo/result/depth_cloud',marker_qos())
            self.tf_pub=self.create_publisher(TFMessage,'/tf_static',marker_qos())
            self.faces_pub=self.create_publisher(MarkerArray,'/demo/result/faces',marker_qos())
            self.create_subscription(MarkerArray,'/unloading/markers',self.markers,marker_qos())
            self.images_seen=[];self.create_subscription(Image,'/demo/dashboard',self.image_seen,1)
            self.create_timer(.2,self.tick)
        def image_seen(self,msg):
            self.images_seen.append(dict(frame_id=msg.header.frame_id,stamp_sec=msg.header.stamp.sec,
                stamp_nanosec=msg.header.stamp.nanosec,bytes=len(msg.data),received_monotonic=time.monotonic()))
            self.images_seen=self.images_seen[-100:]
        def tick(self):
            if self.dashboard is not None:
                self.image_pub.publish(self.image(self.dashboard,self.current['task_id'],self.current['source_time']))
        def switch(self,row):
            self.switching=True
            self.faces_pub.publish(MarkerArray(markers=[Marker(ns=ns,id=i,action=Marker.DELETE) for ns,i in self.display_keys]))
            manifest=read_json(Path(row['capture'])/'manifest.json')
            source_time=manifest['timing']['simulation_time']
            cloud,tf=self.frame_cloud(dict(capture=row['capture'],source_time=source_time))
            self.cloud_pub.publish(cloud);self.tf_pub.publish(tf)
            canvas=PILImage.new('RGB',(1400,1000),'#14202b');draw=ImageDraw.Draw(canvas)
            draw.text((20,15),'RGB-D RECORDING · 本轮新算法结果回放 · READ ONLY',font=font,fill='#ffffff')
            draw.text((20,52),f"{row['task_id']} / ROS_ACCEPTED / ORACLE_PROPOSAL / planning_admissible=false",font=font,fill='#ffcf7a')
            draw.text((20,90),f'原采集时间 {source_time:.6f} ros_sim_time；显示不是实时相机，禁止执行',font=small,fill='#dce7f1')
            for j,camera in enumerate(manifest['cameras']):
                folder=args.run/row['task_id']/'algorithm'/camera['module_id']
                for k,name in enumerate(('sensor_rgb.png','sam_boundaries.png')):
                    image=PILImage.open(folder/name).convert('RGB');image.thumbnail((660,390))
                    x,y=20+j*690,155+k*415;canvas.paste(image,(x,y))
                    draw.text((x,y-27),camera['module_id']+' / '+('原 RGB' if k==0 else '本次 SAM 边界'),font=small,fill='#ffffff')
            self.dashboard=canvas;self.current=dict(task_id=row['task_id'],source_time=source_time)
            self.switching=False
            if self.latest_markers is not None:self.markers(self.latest_markers)
            canvas.save(args.output/(row['task_id']+'-dashboard.png'))
    rclpy.init();node=Display();thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
    delivery=RosDelivery(args.output,args.domain);evidence=[]
    try:
        delivery.start()
        for order,row in enumerate(rows):
            # Suppress new geometry until its own ROS receipt; the previous image
            # retains its explicit previous task ID during this transition.
            node.switching=True
            node.faces_pub.publish(MarkerArray(markers=[Marker(ns=ns,id=i,action=Marker.DELETE) for ns,i in node.display_keys]))
            state['target_task']=row['task_id']
            request=dict(batch_id=state['batch_id'],task_id=row['task_id'],order=order,session=uuid.uuid4().hex,
                         artifact=row['artifact'],submitted_monotonic=time.monotonic())
            state['delivery']=request;atomic_json(args.output/'progress.json',state)
            receipt=delivery.await_receipt(request,timeout=50.)
            node.switch(row)
            row['status']='ROS_ACCEPTED';state['displayed_task']=row['task_id'];atomic_json(args.output/'progress.json',state)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:time.sleep(.1)
            observed=node.images_seen[-10:]
            if not observed or any(i['frame_id']!=row['task_id'] for i in observed):raise ValueError('OLD_IMAGE_REMAINS_AFTER_SWITCH')
            evidence.append(dict(task_id=row['task_id'],artifact=row['artifact'],image_receipts=observed,
                marker_deletes=receipt['previous_marker_deletes'],display_cloud_source_time=node.current['source_time']))
        state['status']='COMPLETED';atomic_json(args.output/'image-switch-checks.json',evidence)
    except Exception as exc:
        state.update(status='FAILED',error=str(exc));raise
    finally:
        atomic_json(args.output/'progress.json',state)
        delivery.close();rclpy.shutdown();thread.join(timeout=5);node.destroy_node()
    return 0


if __name__=='__main__':raise SystemExit(main())
