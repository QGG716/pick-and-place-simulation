"""Hugging Face detector backends and immutable stopped RGB-D associations."""
from dataclasses import dataclass, asdict
from pathlib import Path
from time import monotonic, monotonic_ns
import json
import time
import numpy as np
from ..contracts import DetectionResult, Source, HOST_SESSION
from ..detection import metric_points
from .geometry import recover


from .hf_detector_core import carton_nms, MAX_TARGETS  # noqa: F401  (re-exported)


def detection_result(frame,boxes,inference=None):
    """Bind detector boxes to the frame they were computed on."""
    if len(boxes)>MAX_TARGETS:raise ValueError('TOO_MANY_TARGETS_FOR_SINGLE_CARTON_OBSERVATION')
    extra={} if inference is None else dict(inference=inference)
    items=tuple(dict(label=str(b['label']),score=float(b['score']),bbox_xyxy=[float(x) for x in b['bbox_xyxy']],center_m=None,
        frame_id='camera_optical',geometry_kind='DETECTION_ONLY',**extra) for b in boxes)
    return DetectionResult(frame.observation_id,frame.stamp.source,monotonic_ns(),items,(),
                           'HF_GROUNDING_DINO_ACTUAL_INFERENCE'+('' if inference is None else '_'+inference['backend']))


def segmentation_result(frame,detection,masks,ious,inference=None):
    """Bind one same-frame mask per detection; shapes and counts are checked."""
    if detection.observation_id!=frame.observation_id:raise ValueError('SAM_FRAME_MISMATCH')
    if len(masks)!=len(detection.detections) or len(ious)!=len(masks):raise ValueError('SAM_RESULT_COUNT_MISMATCH')
    shape=frame.rgb.shape[:2]
    if any(np.asarray(m).shape!=shape or np.asarray(m).dtype!=bool for m in masks):raise ValueError('SAM_MASK_SHAPE_MISMATCH')
    extra={} if inference is None else dict(inference=inference)
    items=tuple(dict(item,geometry_kind='SEGMENTATION_REQUIRES_CUBOID_RECOVERY',sam_predicted_iou=float(iou),**extra)
                for item,iou in zip(detection.detections,ious))
    return DetectionResult(frame.observation_id,frame.stamp.source,monotonic_ns(),items,tuple(masks),
                           'HF_GROUNDING_DINO_SAM_ACTUAL_INFERENCE'+('' if inference is None else '_'+inference['backend']))


class HuggingFaceGroundingSAM:
    """Local in-process inference (CPU or a local CUDA device)."""
    loading_status='正在加载本地模型'
    def __init__(self, dino_directory, sam_directory, device='cpu', prompt='carton . box .',
                 box_threshold=.35, text_threshold=.30, cpu_threads=2, **_):
        from .hf_detector_core import GroundingSamCore
        self.core=GroundingSamCore(dino_directory,sam_directory,device,prompt,box_threshold,text_threshold,cpu_threads)
        self.description=f'本地 {device}'
    def detect_boxes(self,frame):
        return detection_result(frame,self.core.detect(frame.rgb))
    def segment(self,frame,detection):
        if detection.observation_id!=frame.observation_id:raise ValueError('SAM_FRAME_MISMATCH')
        masks,ious=self.core.segment(frame.rgb,[item['bbox_xyxy'] for item in detection.detections])
        return segmentation_result(frame,detection,masks,ious)
    def detect(self,frame):
        return self.segment(frame,self.detect_boxes(frame))


def make_detector(**config):
    """Factory selected by detector.backend; no silent fallback between them."""
    backend=config.get('backend','local')
    if backend=='local':return HuggingFaceGroundingSAM(**config)
    if backend=='remote_ssh':
        from .remote_detector import RemoteGroundingSAM
        return RemoteGroundingSAM(**config)
    raise ValueError('UNKNOWN_DETECTOR_BACKEND:'+str(backend))
make_detector.loading_status='正在加载检测模型'


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
    capture_checks: dict | None = None

    def metadata(self):
        f=self.frame
        return dict(observation_id=f.observation_id,serial=f.device_id,rgb_number=f.rgb_number,
            depth_number=f.depth_number,rgb_device_time=asdict(f.rgb_device_time),depth_device_time=asdict(f.depth_device_time),
            stamp=asdict(f.stamp),intrinsics=f.intrinsics,depth_scale_m=f.depth_scale_m,stream=f.stream,
            q_start=self.q_start,q_capture=self.q_capture,robot_sample=self.robot_sample,
            stationary_samples=self.stationary_samples,generation=self.generation,
            T_world_camera=self.world_camera,T_world_tcp_start=self.world_tcp_start,
            capture_checks=self.capture_checks,
            association='HOST_RECEIPT_PROXIMITY_WITH_MEASURED_STATIONARY_WINDOW_NOT_HARDWARE_TRIGGER')


def validate_robot(sample,now,max_age=.1):
    if (not sample or sample.get('source')!='LIVE' or not 0<=now-sample.get('received_s',-1)<=max_age
            or sample.get('alarm',True) or not sample.get('enabled',False)):
        raise ValueError('LIVE_ROBOT_STALE_DISCONNECTED_OR_FAULT')
    q=np.asarray(sample.get('q'),float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise ValueError('INVALID_REAL_JOINTS')
    return q


class CaptureRejected(ValueError):
    def __init__(self,reason,evidence):
        super().__init__(reason);self.evidence=dict(evidence)


def stationary_checks(history):
    """Use controller-measured speed when present, retaining the position bound.

    Differentiating quantized encoder positions at irregular RPC intervals can
    invent speed spikes on a stationary robot. Missing measured speed still
    uses the conservative finite-difference check, never an assumed zero.
    """
    if len(history)<2:return dict(stationary=False,window_s=0.,samples=len(history))
    values=np.asarray([h['q'] for h in history]);times=np.array([h['received_s'] for h in history])
    dt=np.diff(times)
    if np.any(dt<=0):raise ValueError('NONMONOTONIC_STATIONARY_FEEDBACK')
    measured=all(h.get('velocity_source')=='MEASURED_UDP_SPEED' and
        np.asarray(h.get('velocity')).shape==(6,) and np.isfinite(h['velocity']).all() for h in history)
    speed=float(np.max(abs(np.asarray([h['velocity'] for h in history])))) if measured else float(np.max(abs(np.diff(values,axis=0)/dt[:,None])))
    spread=float(np.max(np.ptp(values,axis=0)));window=float(times[-1]-times[0]);gap=float(dt.max())
    return dict(stationary=bool(window>=.4 and gap<=.1 and spread<=np.deg2rad(.02) and speed<=np.deg2rad(.5)),
        window_s=window,samples=len(history),max_gap_s=gap,range_deg=float(np.rad2deg(spread)),
        speed_deg_s=float(np.rad2deg(speed)),speed_source='MEASURED_UDP_SPEED' if measured else 'FINITE_DIFFERENCE_OF_ACTUAL_SAMPLES')


def capture_status(d):
    def value(key,scale=1):return '—' if d.get(key) is None else f'{d[key]*scale:.1f}'
    return (f"准备取景 {d.get('elapsed_s',0):.1f}/4.0 s · {d.get('waiting','等待数据')}\n"
            f"停稳窗口 {d.get('window_s',0):.3f}/0.400 s · 位置范围 {d.get('range_deg',0):.4f}/0.0200° · "
            f"速度 {d.get('speed_deg_s',0):.3f}/0.500°/s ({d.get('speed_source','等待反馈')})\n"
            f"RGB-D 年龄 {value('frame_age_s',1000)}/100 ms · 关节配对差 {value('pair_skew_s',1000)}/30 ms")


def freeze(backend,service,model,calibration,*,cancelled=lambda:False,clock=monotonic,sleep=time.sleep,progress=lambda _:None):
    if calibration is None or calibration.parent_frame!='link_1':raise ValueError('FIELD_LINK1_CAMERA_CALIBRATION_REQUIRED')
    generation=backend.observation_generation;start=clock();history=[];last=None;reported=-1.
    diagnostic=dict(waiting='等待新反馈')
    while clock()-start<4:
        if cancelled():raise ValueError('CANCELLED')
        sample=service.snapshot();now=clock();q=validate_robot(sample,now)
        if sample['sequence']!=last:
            history.append(dict(sample));last=sample['sequence']
        history=[h for h in history if sample['received_s']-h['received_s']<=.55]
        diagnostic=dict(stationary_checks(history),elapsed_s=clock()-start,waiting='等待连续停稳反馈')
        if diagnostic['stationary']:
            diagnostic['waiting']='等待本会话新 RGB-D'
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
                skew=abs(paired['received_s']-frame.stamp.received_monotonic_ns/1e9)
                diagnostic.update(frame_age_s=age,pair_skew_s=skew,waiting='等待帧年龄及关节配对达标',
                                  rgb_number=frame.rgb_number,depth_number=frame.depth_number)
                if not 0<=age<=.10:diagnostic['waiting']='RGB-D 年龄不合格'
                elif skew>.03:diagnostic['waiting']='RGB-D 与关节配对差超过 30 ms'
                else:
                    if generation!=backend.observation_generation:raise ValueError('CONNECTION_CHANGED_DURING_CAPTURE')
                    capture=tuple(paired['q']);T=model.camera(capture,calibration)['T_world_camera']
                    return Observation(frame,tuple(q),capture,dict(paired),tuple(history),generation,
                        tuple(map(tuple,T)),tuple(map(tuple,model.tcp(q))),dict(diagnostic,waiting='已锁定'))
        elif diagnostic.get('range_deg',0)>.02:diagnostic['waiting']='关节位置变化超过 0.02°'
        elif diagnostic.get('speed_deg_s',0)>.5:diagnostic['waiting']='关节速度超过 0.5°/s'
        elif diagnostic.get('max_gap_s',0)>.1:diagnostic['waiting']='连续关节反馈间隔超过 100 ms'
        if clock()-reported>=.25:
            progress(capture_status(diagnostic));reported=clock()
        sleep(.01)
    diagnostic['elapsed_s']=clock()-start
    raise CaptureRejected('准备观测未完成（停稳取景超时）：\n'+capture_status(diagnostic)+
                          '\nNO_FRESH_STATIONARY_RGBD_JOINT_PAIR_WITHIN_30_MS',diagnostic)


def reconstruct(observation,result,geometry=None):
    f=observation.frame
    if result.source!=Source.LIVE or result.observation_id!=f.observation_id or len(result.masks)!=len(result.detections):
        raise ValueError('SAME_FRAME_REAL_SEGMENTATION_REQUIRED')
    return recover_boxes(f,observation.world_camera,result.detections,result.masks,geometry)


def recover_boxes(f,world_camera,detections,masks,geometry=None):
    """Recover boxes from one frame's masks; callers own the frame provenance."""
    points=metric_points(f);boxes=[];rejected=[];seen=[]
    for i,(item,mask) in enumerate(zip(detections,masks)):
        # Also handle old saved observations produced before detector NMS.
        if any(np.count_nonzero(mask&previous)/max(1,np.count_nonzero(mask|previous))>=.95 for previous in seen):continue
        seen.append(mask)
        try:
            if geometry and geometry.get('prior_size_m'):
                from .partial_geometry import recover_partial
                box=recover_partial(points,mask,world_camera,rgb=f.rgb,
                    prior=geometry['prior_size_m'],dimension_tolerance_m=geometry.get('dimension_tolerance_m',.008))
            else:box=recover(points,mask,world_camera,rgb=f.rgb)
            box.update(id=f.observation_id+'-carton-'+str(i),detection_index=i,detection=item)
            boxes.append(box)
        except ValueError as e:rejected.append(dict(detection_index=i,reason=str(e),evidence=getattr(e,'evidence',None)))
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
    lost=(a.depth>0)&(b.depth==0)
    if lost.sum()/max(1,(a.depth>0).sum())>.01:raise ValueError('SCENE_DEPTH_COVERAGE_LOST_REPLAN')
    if np.mean(good)<.5:raise ValueError('SCENE_RECHECK_INSUFFICIENT_DEPTH')
    change=(abs(a.depth.astype(float)-b.depth)*a.depth_scale_m>.005)&good
    if change.sum()/good.sum()>.01:raise ValueError('TARGET_OR_SCENE_CHANGED_REPLAN')
    # A small newly introduced obstacle must not be diluted by a large stable
    # background. Ignore isolated depth speckles, but reject coherent local
    # changes (including newly missing depth) independently of image fraction.
    import cv2
    _,_,stats,_=cv2.connectedComponentsWithStats((change|lost).astype('uint8'),connectivity=8)
    local_changed=int(stats[1:,cv2.CC_STAT_AREA].max()) if len(stats)>1 else 0
    if local_changed>=9:raise ValueError('LOCAL_SCENE_CHANGED_REPLAN')
    return dict(status='PASS',original_observation_id=a.observation_id,new_observation_id=b.observation_id,
                changed_depth_fraction=float(change.sum()/good.sum()),max_local_changed_pixels=local_changed,
                original_stamp=asdict(a.stamp),new_stamp=asdict(b.stamp))


def target_unchanged(original,current,mask):
    a,b=original.frame,current.frame
    valid=mask&(a.depth>0)&(b.depth>0)
    if valid.sum()<120 or valid.sum()/max(1,(mask&(a.depth>0)).sum())<.95:
        raise ValueError('LOCKED_TARGET_DEPTH_COVERAGE_LOST_REPLAN')
    changed=abs(a.depth.astype(float)*a.depth_scale_m-b.depth.astype(float)*b.depth_scale_m)>.004
    # Colour changes also invalidate a replacement carton at the same depth.
    # This conservative check can reject lighting changes; it never asserts ID
    # from a fresh mask or updates the frozen plan in place.
    appearance=np.mean(abs(a.rgb.astype(float)-b.rgb.astype(float)),axis=-1)>35
    if changed[valid].mean()>.05 or appearance[valid].mean()>.10:
        raise ValueError('LOCKED_TARGET_CHANGED_REPLAN')
