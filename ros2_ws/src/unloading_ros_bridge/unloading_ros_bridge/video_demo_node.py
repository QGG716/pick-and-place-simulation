"""Read-only RViz video preview, latest paired input, and asynchronous actual algorithms."""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
import os
from pathlib import Path
import subprocess
import time
import uuid

import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont, ImageOps
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,PointCloud2,PointField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray,Marker
from unloading_perception.finite_sequence import atomic_json,read_json,verify_capture,verify_result,verify_sam_files
from unloading_perception.video_demo import LatestFrameSlot,recording_frames,worker_failure
from unloading_perception.isaac_payload import load_capture_payload
from unloading_perception.geometry import quaternion_from_rotation
from .common import float_to_time,require_humble_python310
from .marker_display import marker_qos


class VideoDemoNode(Node):
    def __init__(self):
        super().__init__('rgbd_video_demo')
        for name,default in (('recording',''),('output',''),('project',''),('models',''),('vision',''),
                             ('algorithm_python',''),('max_results',2),('fps',10.)):
            self.declare_parameter(name,default)
        def value(name): return self.get_parameter(name).value
        self.output=Path(value('output')); self.project=Path(value('project'))
        self.record,self.frames=recording_frames(value('recording'))
        self.models=read_json(value('models'));verify_sam_files(self.models)
        import yaml
        self.config=yaml.safe_load((self.project/'configs/isaac/perception_validation.yaml').read_text(encoding='utf-8'))
        self.limit=int(value('max_results'))
        if not 2<=self.limit<=16: raise ValueError('finite demo requires 2..16 results')
        self.state=read_json(self.output/'progress.json')
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.future=None; self.future_kind=None
        self.slot=LatestFrameSlot(); self.index=0; self.active=None; self.last=None; self.done=0
        self.started=time.monotonic();self.source_started=None;self.source_finished=None
        self.source_error=False;self.current_frame=self.frames[0];self.depth_cache={}
        self.samples=deque(maxlen=100);self.results=[];self.error='';self.mode='READY'
        self.preview={};self.panels={};self.panel_cache={};self.last_panel_refresh=0.
        for module in ('module_0_upper','module_1_lower'):
            with PILImage.open(Path(self.frames[0]['capture'])/'FULL_STACK_NOMINAL/modules'/module/'preview_rgb.jpg') as im:
                self.preview[module]=im.convert('RGB')
        self.switching=False;self.latest_markers=None;self.display_keys=set()
        self.cloud_frame=None
        self.font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',18)
        self.small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',15)
        self.image_pub=self.create_publisher(Image,'/demo/dashboard',1)
        self.rgb_pubs={m:self.create_publisher(Image,f'/demo/input/{m}/rgb',1) for m in ('upper','lower')}
        self.depth_pubs={m:self.create_publisher(Image,f'/demo/input/{m}/depth',1) for m in ('upper','lower')}
        self.cloud_pub=self.create_publisher(PointCloud2,'/demo/result/depth_cloud',marker_qos())
        self.tf_pub=self.create_publisher(TFMessage,'/tf_static',marker_qos())
        self.faces_pub=self.create_publisher(MarkerArray,'/demo/result/faces',marker_qos())
        self.create_subscription(MarkerArray,'/unloading/markers',self.markers,marker_qos())
        env={k:v for k,v in os.environ.items() if k not in ('PYTHONPATH','PYTHONHOME')}
        env['PYTHONPATH']=os.pathsep.join((str(self.project/'src'),str(self.project/'packages/unloading_contracts/src'),str(self.project/'tools')))
        self.log=(self.output/'algorithm-worker.log').open('w',encoding='utf-8')
        self.worker=subprocess.Popen([str(value('algorithm_python')),str(self.project/'tools/workcell_video_worker.py'),
            '--output',str(self.output),'--models',str(value('models')),'--vision',str(value('vision'))],
            env=env,stdout=self.log,stderr=subprocess.STDOUT)
        self.timer=self.create_timer(1./float(value('fps')),self.tick)

    def markers(self,msg):
        # Keep geometry byte-for-byte; the lengthy world legend is shown in the dashboard instead.
        self.latest_markers=msg
        if self.switching:return
        selected=[m for m in msg.markers if m.action==Marker.DELETE or m.type!=Marker.TEXT_VIEW_FACING]
        self.faces_pub.publish(MarkerArray(markers=selected))
        self.display_keys={(m.ns,m.id) for m in selected if m.action==Marker.ADD}

    def save(self): atomic_json(self.output/'progress.json',self.state)

    def fail(self,exc):
        self.error=str(exc);self.mode='FAILED'
        if self.active:
            row=self.state['groups'][self.done]
            row.update(status='FAILED',error=self.error,failure_stage=row.get('stage','unknown'))
            self.results.append({**self.active,'status':'FAILED','error':self.error})
            self.done+=1;self.active=None
        self.save()

    def prepare(self,frame):
        inputs=verify_capture(frame['capture'])
        return inputs,self.frame_cloud(frame) if self.last is None else None

    def accept(self,request,summary_path,inputs):
        summary,reference,counts=verify_result(summary_path,inputs,self.models,self.config)
        return reference,counts,*self.frame_cloud(request)

    def frame_cloud(self,request):
        clouds=[];transforms=[]
        for camera in read_json(Path(request['capture'])/'manifest.json')['cameras']:
            payload=load_capture_payload(Path(request['capture'])/'FULL_STACK_NOMINAL/modules'/camera['module_id'],
                Path(request['capture'])/'manifest.json',expected_module_id=camera['module_id'])
            depth=payload.depth[::10,::10];rgb=payload.rgb[::10,::10]
            y,x=np.indices(depth.shape);K=np.asarray(camera['K']).reshape(3,3)
            valid=np.isfinite(depth)&(depth>0)
            z=depth[valid]; optical=np.column_stack(((x[valid]*10-K[0,2])*z/K[0,0],(y[valid]*10-K[1,2])*z/K[1,1],z))
            T=np.asarray(camera['T_W_C']);points=optical@T[:3,:3].T+T[:3,3]
            packed=np.zeros(len(z),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
            for i,k in enumerate(('x','y','z')):packed[k]=points[:,i]
            colors=rgb[valid].astype(np.uint32);packed['rgb']=(colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]
            clouds.append(packed)
            tf=TransformStamped();tf.header.frame_id='world';tf.child_frame_id='result_'+camera['module_id']
            tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=T[:3,3].tolist()
            q=quaternion_from_rotation(T[:3,:3])
            tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=q
            transforms.append(tf)
        points=np.concatenate(clouds)
        cloud=PointCloud2(height=1,width=len(points),is_bigendian=False,point_step=16,row_step=16*len(points),
            is_dense=True,data=points.tobytes())
        cloud.header.frame_id='world';cloud.header.stamp=float_to_time(request['source_time'])
        cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','rgb'))]
        return cloud,TFMessage(transforms=transforms)

    def image(self,image,frame_id='demo_preview',stamp=0.):
        a=np.asarray(image.convert('RGB'))
        msg=Image(height=a.shape[0],width=a.shape[1],encoding='rgb8',step=a.shape[1]*3,data=a.tobytes())
        msg.header.frame_id=frame_id;msg.header.stamp=float_to_time(stamp)
        return msg

    def draw(self,now):
        canvas=PILImage.new('RGB',(1200,940),(17,23,31));draw=ImageDraw.Draw(canvas)
        def text(x,y,s,color='#dce7f1',small=False):draw.text((x,y),str(s),font=self.small if small else self.font,fill=color)
        text(15,10,'RGB-D RECORDING  |  ORACLE-PROMPTED SAM  |  READ ONLY', '#5edbd1')
        text(15,43,'CURRENT INPUT (preview 640x480)')
        text(440,43,'ACTUAL PROCESSED FRAME / staged results')
        source=self.current_frame
        text(15,72,f"frame {source['frame_sequence']}  source {source['source_time']:.3f}s")
        shown=self.last or self.active
        text(440,72,'No algorithm result yet' if shown is None else f"frame {shown['frame_sequence']}  source {shown['source_time']:.3f}s",'#ffd174')
        for row,module in enumerate(('module_0_upper','module_1_lower')):
            y=110+row*290
            text(15,y,module,small=True)
            if module in self.preview:canvas.paste(ImageOps.contain(self.preview[module],(400,250)),(15,y+25))
            for col,(name,label) in enumerate((('sensor_rgb.png','original input'),('sam_boundaries.png','SAM boundary'),('v4_validation/final_metric_faces_overlay.png','metric faces'))):
                x=440+col*250;text(x,y,label,small=True)
                if shown:
                    file=Path(shown['algorithm_output'])/module/name
                    # Output writers finish before their production summary stage advances.
                    key=str(file)
                    if file.exists() and now-self.last_panel_refresh>.5:
                        try:
                            with PILImage.open(file) as im:self.panel_cache[key]=im.convert('RGB').resize((245,245*3//4))
                        except OSError:pass
                    if key in self.panel_cache:canvas.paste(self.panel_cache[key],(x,y+25))
                    else:text(x,y+95,'PENDING',small=True)
        if now-self.last_panel_refresh>.5:self.last_panel_refresh=now
        fps=(len(self.samples)-1)/(self.samples[-1]-self.samples[0]) if len(self.samples)>1 else 0.
        elapsed=0. if self.active is None else now-self.active['submitted_monotonic']
        completed=[r for r in self.results if r['status']=='ROS_ACCEPTED']
        hz=len(completed)/max(now-(self.source_started or now),1.)
        last='NONE' if self.last is None else str(self.last['frame_sequence'])
        lag='N/A' if self.last is None else f"{source['source_time']-self.last['source_time']:.3f}s SOURCE TIME"
        state=(('SOURCE FAILED / ' if self.source_error else 'SOURCE ENDED / ')+self.mode) if self.source_finished else self.mode
        text(15,710,f"{state} | input measured {fps:.2f} FPS | new complete results {len(completed)} ({hz:.4f} Hz)")
        text(15,740,f"processing frame: {self.active['frame_sequence'] if self.active else 'NONE'} | elapsed {elapsed:.1f}s | last ROS frame: {last}")
        text(15,770,f"result lag: {lag} | overwritten pending pairs: {self.slot.dropped} | queue <= 1")
        timings='';quality=''
        processing=self.active or self.last
        if processing:
            p=Path(processing['algorithm_output'])/'summary.json'
            if p.exists():
                try:
                    report=read_json(p);times=report.get('stage_seconds',{})
                    timings=f"stage={report['stage']}  SAM={times.get('sam_inference',0.):.1f}s geometry={times.get('metric_geometry',0.):.1f}s"
                    rejected=[x for r in report['runs'] for x in r.get('patch_construction_rejections',[])]
                    if rejected:quality=f"{len(rejected)} patch candidate(s) REJECTED: {rejected[0]['reason']}; instances retained"
                except (ValueError,OSError):pass
        text(15,800,timings)
        unknown=self.results[-1].get('ros',{}).get('unknown_regions','N/A') if self.results else 'N/A'
        text(15,830,f'3D depth frame: {self.cloud_frame} | faces: {last} | unknown: {unknown} | no cuboids', '#5edbd1')
        text(15,860,'HISTORICAL_REPLAY_DISPLAY_ONLY | planning_admissible=false | NO EXECUTION','#ffd174')
        text(15,890,(self.error or quality or f"simulation RTF={self.record['simulation_real_time_factor']:.4f}; wall durations and source time are separate")[:105],'#ff8d8d' if self.error else '#ffd174' if quality else '#a8b7c8',True)
        if self.record.get('source_warning'):text(15,915,'SOURCE: verified continuous prefix; acquisition interrupted at the next incomplete pair','#ffd174',True)
        self.image_pub.publish(self.image(canvas))
        atomic_json(self.output/'display-status.json',{'mode':state,'input_fps':fps,'new_result_hz':hz,
            'input_frame':source['frame_sequence'],'processing_frame':None if not self.active else self.active['frame_sequence'],
            'last_completed_frame':None if not self.last else self.last['frame_sequence'],'dropped_pending_pairs':self.slot.dropped,
            'error':self.error,'results':self.results,'wall_elapsed':now-(self.source_started or now),
            'image_subscribers':self.image_pub.get_subscription_count()})

    def tick(self):
        now=time.monotonic()
        try:
            # Explicit user start allows RViz and screen recording to be ready first.
            if not (self.output/'START').exists():self.draw(now);return
            if self.source_started is None:self.source_started=now;self.mode='PROCESSING'
            if self.index<len(self.frames):
                try:
                    frame=self.frames[self.index]
                    previews={}
                    for module in ('module_0_upper','module_1_lower'):
                        folder=Path(frame['capture'])/'FULL_STACK_NOMINAL/modules'/module
                        with PILImage.open(folder/'preview_rgb.jpg') as im:previews[module]=im.convert('RGB')
                    self.preview=previews;self.current_frame=frame
                    self.index+=1;self.samples.append(now);self.slot.offer(frame)
                    for module,short in (('module_0_upper','upper'),('module_1_lower','lower')):
                        self.rgb_pubs[short].publish(self.image(self.preview[module],module,frame['source_time']))
                    if self.index==len(self.frames):self.source_finished=now
                except Exception as exc:
                    # A broken source stops acquisition, never abandons/replaces the in-flight job.
                    self.source_error=True;self.source_finished=now;self.index=len(self.frames)
                    self.error=f'input_preview: {type(exc).__name__}: {exc}'
                    self.slot.take()
            for module,short in (('module_0_upper','upper'),('module_1_lower','lower')):
                if self.depth_pubs[short].get_subscription_count():
                    try:
                        frame=self.current_frame;key=(short,frame['frame_sequence'])
                        if key not in self.depth_cache:
                            folder=Path(frame['capture'])/'FULL_STACK_NOMINAL/modules'/module
                            d=np.load(folder/'metric_depth_m.npy',allow_pickle=False)[::4,::4].copy()
                            msg=Image(height=d.shape[0],width=d.shape[1],encoding='32FC1',step=d.shape[1]*4,data=d.astype('<f4').tobytes())
                            msg.header.frame_id=module;msg.header.stamp=float_to_time(frame['source_time'])
                            self.depth_cache={k:v for k,v in self.depth_cache.items() if k[0]!=short}
                            self.depth_cache[key]=msg
                        self.depth_pubs[short].publish(self.depth_cache[key])
                    except Exception as exc:
                        self.error=f'depth_preview: {type(exc).__name__}: {exc}'
            if self.future is not None and self.future.done():
                future,self.future=self.future,None
                value=future.result()
                if self.future_kind=='prepare':
                    self.inputs,preview_cloud=value
                    if preview_cloud is not None:
                        self.cloud_pub.publish(preview_cloud[0]);self.tf_pub.publish(preview_cloud[1])
                        self.cloud_frame=self.active['frame_sequence']
                    self.state['groups'][self.done].update(status='ALGORITHM_RUNNING',stage='algorithm')
                    atomic_json(self.output/'worker-request.json',self.active);self.save()
                else:
                    ref,counts,self.cloud,self.transforms=value
                    self.switching=True
                    self.faces_pub.publish(MarkerArray(markers=[Marker(ns=ns,id=i,action=Marker.DELETE) for ns,i in self.display_keys]))
                    empty=PointCloud2(height=1,width=0,point_step=16,row_step=0,fields=self.cloud.fields)
                    empty.header.frame_id='world';self.cloud_pub.publish(empty)
                    self.cloud_frame=None
                    request=dict(batch_id=self.state['batch_id'],task_id=self.active['task_id'],order=self.done,
                        session=uuid.uuid4().hex,artifact=ref,submitted_monotonic=time.monotonic())
                    self.state['groups'][self.done].update(status='ARTIFACT_READY',stage='ros_delivery',counts=counts)
                    self.state['delivery']=request;self.save()
            if self.active is not None and self.future is None:
                row=self.state['groups'][self.done]
                worker=self.output/'worker-status.json'
                if row['status']=='ALGORITHM_RUNNING' and worker.exists():
                    result=read_json(worker)
                    if result['task_id']==self.active['task_id'] and result['status']!='RUNNING':
                        if result['status']=='FAILED':
                            raise RuntimeError(worker_failure(result))
                        self.active['algorithm_seconds']=result['wall_seconds'];self.active['resident_model_loads']=result['resident_model_loads']
                        self.future=self.pool.submit(self.accept,self.active,Path(self.active['algorithm_output'])/'summary.json',self.inputs)
                        self.future_kind='accept';row['stage']='artifact_validation'
                if row['status']=='ARTIFACT_READY':
                    receipt=self.output/'receipts'/(self.active['task_id']+'.json')
                    if receipt.exists():
                        received=read_json(receipt)
                        if received['status']!='ROS_ACCEPTED':raise RuntimeError(received.get('error','ROS rejected'))
                        if any(received[k]!=self.state['delivery'][k] for k in ('session','artifact','task_id')):raise ValueError('wrong ROS receipt')
                        self.cloud_pub.publish(self.cloud);self.tf_pub.publish(self.transforms)
                        self.cloud_frame=self.active['frame_sequence']
                        self.switching=False
                        if self.latest_markers is not None:self.markers(self.latest_markers)
                        row.update(status='ROS_ACCEPTED',stage='completed');self.state['displayed_task']=self.active['task_id']
                        self.last=self.active;self.results.append({**self.active,'status':'ROS_ACCEPTED','ros':{k:v for k,v in received.items() if k!='world_events'}})
                        self.active=None;self.done+=1;self.save()
                    elif now-self.state['delivery']['submitted_monotonic']>35:raise TimeoutError('ROS delivery timeout')
            if self.active is None and self.future is None and self.slot.pending is not None and self.done<self.limit:
                frame=self.slot.take();task=f'video_{self.done:02d}'
                self.active={**frame,'task_id':task,'algorithm_output':str(self.output/task/'algorithm'),'submitted_monotonic':now}
                (self.output/task).mkdir(exist_ok=False)
                self.state['target_task']=task;self.state['delivery']=None
                self.state['groups'][self.done].update(status='VALIDATING_INPUT',stage='capture_validation',frame=frame)
                self.save();self.mode='PROCESSING'
                self.future=self.pool.submit(self.prepare,frame);self.future_kind='prepare'
            if self.done>=self.limit or (self.source_finished and self.active is None and self.slot.pending is None):
                success=len([r for r in self.results if r['status']=='ROS_ACCEPTED'])>=2 and not self.error
                self.mode='COMPLETE' if success else 'FAILED'
                self.state.update(status='COMPLETED' if success else 'PARTIAL_OR_FAILED',exit_code=0 if success else 1)
                self.save()
                atomic_json(self.output/'demo-finished.json',{'exit_code':self.state['exit_code'],'results':self.results})
            if self.worker.poll() is not None and self.active:raise RuntimeError('algorithm worker exited')
        except Exception as exc:self.fail(f'{type(exc).__name__}: {exc}')
        self.draw(now)

    def destroy_node(self):
        (self.output/'stop-worker').touch()
        if self.worker.poll() is None:
            self.worker.terminate()
            try:self.worker.wait(timeout=10)
            except subprocess.TimeoutExpired:self.worker.kill();self.worker.wait()
        self.pool.shutdown(wait=True,cancel_futures=True);self.log.close()
        return super().destroy_node()


def main(args=None):
    require_humble_python310();rclpy.init(args=args);node=VideoDemoNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
