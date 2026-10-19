"""Same profile-checked native path, offline warehouse display at 640x360 / 5 fps.

Timeline remains stopped. Pose updates are KINEMATIC_REPLAY, never dynamics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from time import perf_counter


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['snapshot','plan','assets','workstation','output']:
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--review',type=Path,help='Required for strict plans only')
    p.add_argument('--evaluation-profile',choices=['strict','chain_integration_v1'],help='If supplied, must match the plan')
    p.add_argument('--preview',action='store_true',help='Representative stage frames, four RGB samples per pose')
    a=p.parse_args()
    from unloading_sim.eco65.model import ROOT,load_json,save_json
    from unloading_sim.eco65.native_replay import replay_inputs,state_at,apply_display_state
    from unloading_sim.eco65.evaluation import validate_export_reference
    snapshot,plan=map(load_json,[a.snapshot,a.plan])
    review=load_json(a.review) if a.review else None
    if a.evaluation_profile and a.evaluation_profile!=plan.get('evaluation_profile','strict'):
        raise ValueError('EVALUATION_PROFILE_MISMATCH')
    robot=replay_inputs(snapshot,plan,review)
    out=a.output.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    started=perf_counter();app=None
    try:
        ffmpeg=shutil.which('ffmpeg')
        if not ffmpeg:raise ValueError('EXISTING_FFMPEG_ENCODER_REQUIRED')
        encoder_version=subprocess.check_output([ffmpeg,'-version'],text=True).splitlines()[0]
        encoders=subprocess.run([ffmpeg,'-hide_banner','-encoders'],capture_output=True,text=True,check=True).stdout
        if 'libx264 ' not in encoders:raise ValueError('EXISTING_FFMPEG_LIBX264_REQUIRED')
        from isaacsim import SimulationApp
        # This installed runtime has no working NGX/OptiX denoiser. Average
        # actual RGB samples at each fixed pose; never mix trajectory times.
        samples_per_pose=4 if a.preview else 64
        render_config=dict(headless=True,renderer='RealTimePathTracing',
                           width=640,height=360,denoiser=False,anti_aliasing=2,
                           extra_args=['--/persistent/rtx/modes/rt2/enabled=true',
                                       '--/persistent/rtx/modes/pt/enabled=false',
                                       '--/persistent/rtx/modes/rt/enabled=false'])
        app=SimulationApp(render_config)
        import numpy as np
        from PIL import Image,ImageDraw
        from pxr import UsdGeom,Gf
        import omni.usd
        import omni.replicator.core as rep
        import omni.timeline
        import isaacsim_render_eco65_warehouse as compose
        manifest=load_json(a.workstation.with_name('workstation_manifest.json'))
        validate_export_reference(plan,manifest)
        if manifest.get('source_snapshot_fingerprint')!=snapshot['fingerprint'] or not manifest.get('native_snapshot'):
            raise ValueError('WORKSTATION_SNAPSHOT_MISMATCH')
        if manifest['home_q_rad']!=plan['home_q']:raise ValueError('WORKSTATION_HOME_CHANGED')
        sys.argv=['compose','--assets',str(a.assets),'--workstation',str(a.workstation),
                  '--output',str(out/'composition'),'--compose-only']
        compose.main()
        omni.usd.get_context().open_stage(str(out/'composition/eco65_desktop_warehouse.usda'))
        app.reset_render_settings()
        for _ in range(30):app.update()
        timeline=omni.timeline.get_timeline_interface();timeline.stop()
        stage=omni.usd.get_context().get_stage();checks=[]
        import carb
        settings=carb.settings.get_settings()
        renderer_readback={key:settings.get(key) for key in ['/rtx/rendermode','/rtx/post/aa/op','/rtx/pathtracing/spp',
            '/rtx/pathtracing/optixDenoiser/enabled']}
        save_json(out/'renderer_settings.json',dict(requested=render_config,actual=renderer_readback))
        # Same nonzero wrist/zero/HOME FK cases as the current native model.
        for row in plan['fk_crosscheck']:
            state=state_at(snapshot,plan,robot,0)
            state['frames']=robot.named_link_frames(row['q_rad'] if 'q_rad' in row else row['q'])
            checks.append(apply_display_state(stage,snapshot,state))
        camera=UsdGeom.Camera.Define(stage,'/World/OfflineReplayCamera')
        camera.CreateHorizontalApertureAttr(36.);camera.CreateVerticalApertureAttr(20.25)
        camera.CreateFocalLengthAttr(45.);camera.CreateClippingRangeAttr(Gf.Vec2f(.01,200.))
        camera.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(-1.98,-2.6,2.7),Gf.Vec3d(.02,-.1,1.02),Gf.Vec3d(0,0,1)).GetInverse())
        product=rep.create.render_product(str(camera.GetPath()),(640,360))
        rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(product)
        preparation_s=perf_counter()-started;render_started=perf_counter();records=[]
        target=snapshot['scene']['target_box_id']
        duration=plan['simulated_duration_s'];samples=np.unique(np.r_[np.arange(0,duration,.2),duration])
        if a.preview:
            ends=np.cumsum([s['times'][-1] for s in plan['stages']]);starts=np.r_[0,ends[:-1]]
            samples=np.unique(np.r_[0,(starts+ends)/2,ends])
        command=[ffmpeg,'-hide_banner','-loglevel','error','-nostdin','-f','rawvideo','-pixel_format','rgb24',
                 '-video_size','640x360','-framerate','5','-i','pipe:0','-an','-c:v','libx264','-crf','18',
                 '-pix_fmt','yuv420p','-movflags','+faststart',str(out/'kinematic_replay.mp4')]
        seen_phases=set()
        with (out/'encoder.log').open('w') as encoder_log, (out/'replay_progress.jsonl').open('w') as progress, \
                subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=encoder_log) as writer:
            for t in samples:
                state=state_at(snapshot,plan,robot,float(t));readback=apply_display_state(stage,snapshot,state)
                # Let render products become ready at the fixed pose. This
                # bounded warmup advances neither physical nor trajectory time.
                for render_attempt in range(40 if not records else 8):
                    rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
                    if timeline.is_playing() or timeline.get_current_time()!=0:raise ValueError('OFFLINE_TIMELINE_ADVANCED')
                    pixels=rgb.get_data();pixels=pixels['data'] if isinstance(pixels,dict) else pixels
                    pixels=np.asarray(pixels)[...,:3]
                    if render_attempt>=3 and pixels.shape==(360,640,3) and np.std(pixels)>=5:break
                if pixels.shape!=(360,640,3) or np.std(pixels)<5:
                    save_json(out/'invalid_frame.json',dict(time_s=float(t),shape=list(pixels.shape),
                        dtype=str(pixels.dtype),std=float(np.std(pixels)) if pixels.size else None))
                    if pixels.shape==(360,640,3):Image.fromarray(pixels).save(out/'invalid_frame.png')
                    raise ValueError('INVALID_REPLAY_RENDER')
                accumulated=np.zeros((360,640,3),dtype=np.float64)
                for _ in range(samples_per_pose):
                    rep.orchestrator.step(delta_time=0.,rt_subframes=1,pause_timeline=True)
                    if timeline.is_playing() or timeline.get_current_time()!=0:raise ValueError('OFFLINE_TIMELINE_ADVANCED')
                    sample=rgb.get_data();sample=sample['data'] if isinstance(sample,dict) else sample
                    sample=np.asarray(sample)[...,:3]
                    if sample.shape!=(360,640,3) or np.std(sample)<5:raise ValueError('INVALID_REPLAY_ACCUMULATION_SAMPLE')
                    accumulated+=sample
                pixels=np.rint(accumulated/samples_per_pose).astype(np.uint8)
                im=Image.fromarray(pixels);draw=ImageDraw.Draw(im)
                draw.rectangle([0,302,640,360],fill='#172432')
                draw.text((8,305),f"{target} | {state['phase']} | {t:.2f}s | cups={len(state['active_ids'])}",fill='white')
                draw.text((8,321),'KINEMATIC_REPLAY | PLANNING_ONLY_NOT_EXECUTABLE',fill='white')
                draw.text((8,337),f"attached={state['attached']} placed={state['released']} | Dynamics: NOT_EVALUATED",fill='white')
                writer.stdin.write(np.asarray(im).tobytes())
                if not records:im.save(out/'warehouse_first_frame.png')
                if state['phase'] not in seen_phases:
                    im.save(out/('phase_'+state['phase']+'.png'));seen_phases.add(state['phase'])
                records.append(dict(time_s=float(t),q_rad=state['q'].tolist(),phase=state['phase'],
                    warmup_render_steps=render_attempt+1,rgb_samples=samples_per_pose,readback=readback))
                if len(records)==1 or len(records)%25==0:
                    progress.write(json.dumps(dict(frames=len(records),time_s=float(t),phase=state['phase'],
                        render_wall_s=perf_counter()-render_started,timeline_time_s=timeline.get_current_time()))+'\n');progress.flush()
            im.save(out/'warehouse_last_frame.png')
        if writer.returncode!=0:raise ValueError('FFMPEG_ENCODING_FAILED:'+str(writer.returncode))
        save_json(out/'replay_report.json',dict(status='KINEMATIC_REPLAY',planning_status=plan['status'],
            evaluation_profile=plan.get('evaluation_profile','strict'),
            sampling='REPRESENTATIVE_STAGE_PREVIEW' if a.preview else 'NORMAL_TIME_5_FPS',
            dynamics='NOT_EVALUATED',hardware=False,plan_fingerprint=plan['fingerprint'],snapshot_fingerprint=snapshot['fingerprint'],
            target_id=target,carton_count=len(snapshot['scene']['boxes']),
            frame_rate=5,frame_count=len(records),simulated_duration_s=duration,preparation_wall_s=preparation_s,render_wall_s=perf_counter()-render_started,
            encoder_version=encoder_version,encoder_command=command,
            renderer_settings=renderer_readback,
            accumulation=dict(method='ARITHMETIC_MEAN_OF_ACTUAL_ISAAC_RGB_AT_IDENTICAL_POSE',
                samples_per_pose=samples_per_pose,cross_pose_blending=False,linear_radiance_claim=False),
            video_sha256=hashlib.sha256((out/'kinematic_replay.mp4').read_bytes()).hexdigest(),
            timeline=dict(playing=timeline.is_playing(),time_s=timeline.get_current_time(),physics_steps=0),
            fk_readback=checks,frames=records,ideal_outfeed_count=0))
    except Exception as error:
        import traceback
        save_json(out/'failure.json',dict(error=str(error),traceback=traceback.format_exc(),dynamics='NOT_EVALUATED'))
        raise
    finally:
        if app:app.close()


if __name__=='__main__':main()
