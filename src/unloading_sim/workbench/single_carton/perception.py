"""Local Hugging Face checkpoints and immutable stopped RGB-D associations."""
from dataclasses import dataclass, asdict
from pathlib import Path
from time import monotonic, monotonic_ns
import json
import time
import numpy as np
from ..contracts import DetectionResult, Source, HOST_SESSION
from ..detection import metric_points
from .geometry import recover


class HuggingFaceGroundingSAM:
    def __init__(self, dino_directory, sam_directory, device='cpu', prompt='carton . box .',
                 box_threshold=.35, text_threshold=.30):
        import torch
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection, SamModel, SamProcessor
        self.torch=torch;self.device=device;self.prompt=prompt
        self.box_threshold=box_threshold;self.text_threshold=text_threshold
        for path, kind in ((dino_directory,'grounding-dino'),(sam_directory,'sam')):
            p=Path(path)
            if not (p/'model.safetensors').is_file() or kind not in json.loads((p/'config.json').read_text()).get('model_type',''):
                raise ValueError('CHECKPOINT_FORMAT_OR_MODEL_MISMATCH:'+str(p))
        # These are HF directories, never renamed upstream .pth checkpoints.
        self.dino_processor=AutoProcessor.from_pretrained(dino_directory,local_files_only=True)
        self.dino=AutoModelForZeroShotObjectDetection.from_pretrained(dino_directory,local_files_only=True).to(device).eval()
        self.sam_processor=SamProcessor.from_pretrained(sam_directory,local_files_only=True)
        self.sam=SamModel.from_pretrained(sam_directory,local_files_only=True).to(device).eval()
    def detect(self,frame):
        torch=self.torch
        with torch.inference_mode():
            inputs=self.dino_processor(images=frame.rgb,text=self.prompt,return_tensors='pt').to(self.device)
            outputs=self.dino(**inputs)
            r=self.dino_processor.post_process_grounded_object_detection(outputs,inputs.input_ids,
                box_threshold=self.box_threshold,text_threshold=self.text_threshold,target_sizes=[frame.rgb.shape[:2]])[0]
            boxes=r['boxes'].cpu().numpy();scores=r['scores'].cpu().numpy()
            if not len(boxes):return DetectionResult(frame.observation_id,frame.stamp.source,monotonic_ns(),(),(), 'HF_GROUNDING_DINO_SAM_ACTUAL_INFERENCE')
            if len(boxes)>20:raise ValueError('TOO_MANY_TARGETS_FOR_SINGLE_CARTON_OBSERVATION')
            inp=self.sam_processor(images=frame.rgb,input_boxes=[boxes.tolist()],return_tensors='pt').to(self.device)
            out=self.sam(**inp,multimask_output=True)
            masks=self.sam_processor.image_processor.post_process_masks(out.pred_masks.cpu(),inp['original_sizes'].cpu(),inp['reshaped_input_sizes'].cpu())[0]
            choices=out.iou_scores[0].argmax(-1).cpu().numpy()
            selected=tuple(masks[i,int(j)].numpy().astype(bool) for i,j in enumerate(choices))
            items=tuple(dict(label=str(label),score=float(score),bbox_xyxy=box.tolist(),center_m=None,
                frame_id='camera_optical',geometry_kind='SEGMENTATION_REQUIRES_CUBOID_RECOVERY',
                sam_predicted_iou=float(out.iou_scores[0,i,int(choices[i])]))
                for i,(label,score,box) in enumerate(zip(r['labels'],scores,boxes)))
        return DetectionResult(frame.observation_id,frame.stamp.source,monotonic_ns(),items,selected,'HF_GROUNDING_DINO_SAM_ACTUAL_INFERENCE')


@dataclass(frozen=True)
class Observation:
    frame: object
    q_start: tuple
    q_capture: tuple
    robot_sample: dict
    stationary_samples: tuple
    generation: int
    world_camera: tuple
    world_tcp_start: tuple

    def metadata(self):
        f=self.frame
        return dict(observation_id=f.observation_id,serial=f.device_id,rgb_number=f.rgb_number,
            depth_number=f.depth_number,rgb_device_time=asdict(f.rgb_device_time),depth_device_time=asdict(f.depth_device_time),
            stamp=asdict(f.stamp),intrinsics=f.intrinsics,depth_scale_m=f.depth_scale_m,stream=f.stream,
            q_start=self.q_start,q_capture=self.q_capture,robot_sample=self.robot_sample,
            stationary_samples=self.stationary_samples,generation=self.generation,
            T_world_camera=self.world_camera,T_world_tcp_start=self.world_tcp_start,
            association='HOST_RECEIPT_PROXIMITY_WITH_MEASURED_STATIONARY_WINDOW_NOT_HARDWARE_TRIGGER')


def validate_robot(sample,now,max_age=.1):
    if (not sample or sample.get('source')!='LIVE' or not 0<=now-sample.get('received_s',-1)<=max_age
            or sample.get('alarm',True) or not sample.get('enabled',False)):
        raise ValueError('LIVE_ROBOT_STALE_DISCONNECTED_OR_FAULT')
    q=np.asarray(sample.get('q'),float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('INVALID_REAL_JOINTS')
    return q


def freeze(backend,service,model,calibration,*,cancelled=lambda:False,clock=monotonic,sleep=time.sleep):
    if calibration is None or calibration.parent_frame!='link_1':raise ValueError('FIELD_LINK1_CAMERA_CALIBRATION_REQUIRED')
    generation=backend.observation_generation;start=clock();history=[];last=None
    while clock()-start<4:
        if cancelled():raise ValueError('CANCELLED')
        sample=service.snapshot();now=clock();q=validate_robot(sample,now)
        if sample['sequence']!=last:
            history.append(dict(sample));last=sample['sequence']
        history=[h for h in history if sample['received_s']-h['received_s']<=.55]
        if history and sample['received_s']-history[0]['received_s']>=.4:
            values=np.asarray([h['q'] for h in history]);times=np.array([h['received_s'] for h in history])
            speed=np.max(abs(np.diff(values,axis=0)/np.diff(times)[:,None]))
            if np.max(np.ptp(values,axis=0))<=np.deg2rad(.02) and speed<=np.deg2rad(.5):
                frame=backend.frame
                if frame is not None and backend.sample_state(frame.stamp,'camera')=='LIVE_VALID' and frame.stamp.host_session==HOST_SESSION:
                    if frame.stamp.source!=Source.LIVE:raise ValueError('REAL_OBSERVATION_REQUIRED')
                    rt,dt=frame.rgb_device_time,frame.depth_device_time
                    if (not frame.device_id or frame.rgb_number is None or frame.depth_number is None or
                            rt.value_ms is None or dt.value_ms is None or rt.domain!=dt.domain or
                            abs(rt.value_ms-dt.value_ms)>1000/float(frame.stream.get('fps',30))+1):
                        raise ValueError('UNTRACEABLE_OR_UNPAIRED_RGB_DEPTH_FRAMESET')
                    age=(monotonic_ns()-frame.stamp.received_monotonic_ns)/1e9
                    paired=min(history,key=lambda h:abs(h['received_s']-frame.stamp.received_monotonic_ns/1e9))
                    if age<=.10 and abs(paired['received_s']-frame.stamp.received_monotonic_ns/1e9)<=.03:
                        if generation!=backend.observation_generation:raise ValueError('CONNECTION_CHANGED_DURING_CAPTURE')
                        capture=tuple(paired['q']);T=model.camera(capture,calibration)['T_world_camera']
                        return Observation(frame,tuple(q),capture,dict(paired),tuple(history),generation,
                            tuple(map(tuple,T)),tuple(map(tuple,model.tcp(q))))
        sleep(.01)
    raise ValueError('NO_FRESH_STATIONARY_RGBD_JOINT_PAIR_WITHIN_30_MS')


def reconstruct(observation,result):
    f=observation.frame
    if result.source!=Source.LIVE or result.observation_id!=f.observation_id or len(result.masks)!=len(result.detections):
        raise ValueError('SAME_FRAME_REAL_SEGMENTATION_REQUIRED')
    points=metric_points(f);boxes=[];rejected=[]
    for i,(item,mask) in enumerate(zip(result.detections,result.masks)):
        try:
            box=recover(points,mask,observation.world_camera,rgb=f.rgb)
            box.update(id=f.observation_id+'-carton-'+str(i),detection_index=i,detection=item)
            boxes.append(box)
        except ValueError as e:rejected.append(dict(detection_index=i,reason=str(e)))
    return boxes,rejected


def unchanged(original,current,*,max_joint_rad=np.deg2rad(.01)):
    if current.generation!=original.generation:raise ValueError('CONNECTION_OR_CONFIGURATION_CHANGED_REPLAN')
    if np.max(abs(np.asarray(current.q_start)-original.q_start))>max_joint_rad:raise ValueError('START_CHANGED_REPLAN')
    a,b=original.frame,current.frame
    if a.device_id!=b.device_id or a.intrinsics!=b.intrinsics or a.depth_scale_m!=b.depth_scale_m:
        raise ValueError('CAMERA_IDENTITY_CHANGED_REPLAN')
    # The view is stopped at the same q; compare scene depth directly, including
    # surrounding geometry. Never freshen the old exposure timestamp.
    good=(a.depth>0)&(b.depth>0)
    if np.mean(good)<.5:raise ValueError('SCENE_RECHECK_INSUFFICIENT_DEPTH')
    change=(abs(a.depth.astype(float)-b.depth)*a.depth_scale_m>.005)&good
    if change.sum()/good.sum()>.01:raise ValueError('TARGET_OR_SCENE_CHANGED_REPLAN')
    return dict(status='PASS',original_observation_id=a.observation_id,new_observation_id=b.observation_id,
                changed_depth_fraction=float(change.sum()/good.sum()),original_stamp=asdict(a.stamp),new_stamp=asdict(b.stamp))
