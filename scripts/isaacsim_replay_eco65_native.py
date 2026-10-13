"""Same reviewed native path, offline warehouse display at 640x360 / 5 fps.

Timeline remains stopped. Pose updates are KINEMATIC_REPLAY, never dynamics.
"""
import argparse
from pathlib import Path
import sys
from time import perf_counter


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['snapshot','plan','review','assets','workstation','output']:
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    from unloading_sim.eco65.model import ROOT,load_json,save_json
    from unloading_sim.eco65.native_replay import replay_inputs,state_at,apply_display_state
    snapshot,plan,review=map(load_json,[a.snapshot,a.plan,a.review])
    robot=replay_inputs(snapshot,plan,review)
    out=a.output.resolve();out.relative_to(ROOT/'outputs');out.mkdir(parents=True,exist_ok=False)
    started=perf_counter();app=None
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','width':640,'height':360})
        import numpy as np
        import imageio.v2 as imageio
        from PIL import Image,ImageDraw
        from pxr import UsdGeom,Gf
        import omni.usd
        import omni.replicator.core as rep
        import omni.timeline
        import isaacsim_render_eco65_warehouse as compose
        manifest=load_json(a.workstation.with_name('workstation_manifest.json'))
        if manifest.get('source_snapshot_fingerprint')!=snapshot['fingerprint'] or not manifest.get('native_snapshot'):
            raise ValueError('WORKSTATION_SNAPSHOT_MISMATCH')
        if manifest['home_q_rad']!=plan['home_q']:raise ValueError('WORKSTATION_HOME_CHANGED')
        sys.argv=['compose','--assets',str(a.assets),'--workstation',str(a.workstation),
                  '--output',str(out/'composition'),'--compose-only']
        compose.main()
        omni.usd.get_context().open_stage(str(out/'composition/eco65_desktop_warehouse.usda'))
        for _ in range(30):app.update()
        timeline=omni.timeline.get_timeline_interface();timeline.stop()
        stage=omni.usd.get_context().get_stage();checks=[]
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
        duration=plan['simulated_duration_s'];samples=np.unique(np.r_[np.arange(0,duration,.2),duration])
        with imageio.get_writer(str(out/'kinematic_replay.mp4'),fps=5,codec='libx264',macro_block_size=1) as writer:
            for t in samples:
                state=state_at(snapshot,plan,robot,float(t));readback=apply_display_state(stage,snapshot,state)
                for _ in range(2):rep.orchestrator.step(delta_time=0.,rt_subframes=4,pause_timeline=True)
                if timeline.is_playing() or timeline.get_current_time()!=0:raise ValueError('OFFLINE_TIMELINE_ADVANCED')
                pixels=rgb.get_data();pixels=pixels['data'] if isinstance(pixels,dict) else pixels
                pixels=np.asarray(pixels)[...,:3]
                if pixels.shape!=(360,640,3) or np.std(pixels)<5:raise ValueError('INVALID_REPLAY_RENDER')
                im=Image.fromarray(pixels);draw=ImageDraw.Draw(im)
                draw.rectangle([0,302,640,360],fill='#172432')
                draw.text((8,305),f"carton_r00_l01_c00 | {state['phase']} | {t:.2f}s | attached={state['attached']}",fill='white')
                draw.text((8,321),'KINEMATIC_REPLAY | PLANNING_ONLY_NOT_EXECUTABLE',fill='white')
                draw.text((8,337),'Dynamics: NOT_EVALUATED | ideal outfeed: 0',fill='white')
                writer.append_data(np.asarray(im))
                if not records:im.save(out/'warehouse_first_frame.png')
                records.append(dict(time_s=float(t),q_rad=state['q'].tolist(),phase=state['phase'],readback=readback))
        save_json(out/'replay_report.json',dict(status='KINEMATIC_REPLAY',planning_status=plan['status'],
            dynamics='NOT_EVALUATED',hardware=False,plan_fingerprint=plan['fingerprint'],snapshot_fingerprint=snapshot['fingerprint'],
            frame_rate=5,simulated_duration_s=duration,preparation_wall_s=preparation_s,render_wall_s=perf_counter()-render_started,
            fk_readback=checks,frames=records,ideal_outfeed_count=0))
    except Exception as error:
        import traceback
        save_json(out/'failure.json',dict(error=str(error),traceback=traceback.format_exc(),dynamics='NOT_EVALUATED'))
        raise
    finally:
        if app:app.close()


if __name__=='__main__':main()
