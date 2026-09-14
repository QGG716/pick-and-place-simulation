"""Finite actual SAM/2D inference with MoGe deliberately unconfigured."""
import argparse,json,sys
from pathlib import Path
from time import perf_counter
from vision_resident_worker import ResidentRuntime,parser as worker_parser,sha256


def main():
    p=argparse.ArgumentParser(); p.add_argument('--scene-directory',type=Path,required=True); p.add_argument('--model-manifest',type=Path,required=True); p.add_argument('--vision-root',type=Path,required=True); p.add_argument('--output-directory',type=Path,required=True); args=p.parse_args()
    scene=args.scene_directory; binding=json.loads((scene/'capture_metadata.json').read_text()); models=json.loads(args.model_manifest.read_text())
    worker=worker_parser().parse_args(['--upstream-root',str(args.vision_root),'--output-root',str(args.output_directory),'--input-root',str(scene),'--sam-model',models['sam']['snapshot_path']])
    started=perf_counter(); runtime=ResidentRuntime(worker)
    source=scene/'sensor_rgb.png'; proposal=scene/'oracle_proposals.json'
    import cv2
    h,w=cv2.imread(str(source)).shape[:2]
    request={'schema_version':'1.1.0','op':'infer','request_id':'rgbd-without-moge','worker_epoch':'no-moge-validation','proposal_reference':{'uri':proposal.resolve().as_uri(),'sha256':sha256(proposal)},'frame':{'source':'isaac-sim-6.0.1','stream':'perception_validation','epoch':binding['sensor_epoch'],'sequence':binding['frame_sequence'],'capture_time':binding['capture_center_time'],'receive_time':binding['capture_center_time'],'clock_domain':binding['clock_domain'],'frame_id':binding['rgb_frame_id'],'width':w,'height':h,'encoding':'rgb8','rgb_uri':source.resolve().as_uri(),'rgb_sha256':sha256(source)}}
    result=runtime.infer(request)
    assert runtime.moge_model is None and not any(k=='moge' or k.startswith('moge.') for k in sys.modules)
    assert result['status']=='COMPLETE'
    report={'status':'SAM_AND_2D_COMPLETE_WITHOUT_MOGE','moge_model_configured':False,'moge_imported':False,'timing_scope':'model_startup_image_read_SAM_2D_and_artifact_output; metric_geometry_and_ROS_are_separate','wall_seconds':perf_counter()-started,'response':result}
    (args.output_directory/'no_moge_report.json').write_text(json.dumps(report,indent=2)); print(json.dumps(report,indent=2))


if __name__=='__main__': main()
