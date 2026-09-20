"""Explicitly labeled visual-only checks; no changes to the main camera/lights."""
import json
import subprocess


def render_updated_state(rep):
    """Reset stopped-time DLSS history, then settle without advancing physics."""
    rep.settings.carb_settings('/rtx-transient/post/dlss/forceParamReset', True)
    rep.orchestrator.step(rt_subframes=1,pause_timeline=True,delta_time=0.,wait_for_render=True)
    rep.settings.carb_settings('/rtx-transient/post/dlss/forceParamReset', False)
    rep.orchestrator.step(rt_subframes=8,pause_timeline=True,delta_time=0.,wait_for_render=True)


def render_review(ns,visuals,inputs,schedules,options):
    from m710_belt_visual import distance_at
    import numpy as np
    from PIL import Image,ImageDraw,ImageFont
    rep=ns['rep']
    views={
        'carton_detail':((-.8,-1.0,2.6),(.25,0.,1.9)),
        'conveyor_detail':((-1.8,-1.0,2.5),(-1.0,-.25,.55)),
        'chassis_detail':((-3.1,1.0,.85),(-1.95,.35,.20)),
    }
    annotators={}
    for name,(eye,target) in views.items():
        camera=rep.create.camera(position=eye,look_at=target,focal_length=18,clipping_range=(.03,10))
        product=rep.create.render_product(camera,(1280,720));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(product)
        annotators[name]=ann
    rep.orchestrator.step(rt_subframes=8,pause_timeline=True,delta_time=0.,wait_for_render=True)
    for name,ann in annotators.items():
        rgb=np.asarray(ann.get_data())[:,:,:3]
        assert rgb.std()>10 and rgb.mean()>10, name+' inspection camera occluded'
        Image.fromarray(rgb).save(options.output/(name+'.png'))
    output=options.output/'belt_motion_1x_visual_check.mp4'
    encoder=subprocess.Popen([str(options.ffmpeg),'-v','error','-f','rawvideo','-pixel_format','rgb24',
        '-video_size','1280x720','-framerate','5','-i','pipe:0','-an','-c:v','libx264','-threads','2',
        '-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    samples=list(np.arange(22.4,26.41,.2))+list(np.arange(47.0,51.01,.2))
    schedule=schedules[1];data=inputs[1];records=[]
    for index,t in enumerate(samples):
        phases={name:initial+distance_at(data['result'],name,t) for name,initial in schedule['initial_phase_m'].items()}
        visuals.update(phases)
        render_updated_state(rep)
        rgb=np.asarray(annotators['conveyor_detail'].get_data())[:,:,:3]
        assert rgb.std()>10 and rgb.mean()>10
        frame=Image.fromarray(rgb);draw=ImageDraw.Draw(frame)
        draw.rectangle((12,646,850,710),fill='#14202b')
        draw.text((24,654),f'1x BELT VISUAL CHECK | source clip 2 | t={t:.1f}s',
                  font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20),fill='white')
        draw.text((24,683),'Two source windows; robot held still; no physics execution',
                  font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16),fill='#b7d5e8')
        encoder.stdin.write(np.asarray(frame).tobytes())
        if index in (0,1,10,11,29,30): frame.save(options.output/f'belt_check_{index:02d}.png')
        records.append(dict(source_time_s=float(t),phase_m=phases))
    encoder.stdin.close();assert encoder.wait()==0
    (options.output/'belt_motion_review.json').write_text(json.dumps(dict(
        mode='visual_only',fps=5,playback_speed=1,source_clip=2,source_windows_s=[[22.4,26.4],[47,51]],
        samples=records,framework_static=True,physics_timeline_stopped=True),indent=2))
