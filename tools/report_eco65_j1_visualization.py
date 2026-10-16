"""Annotate actual Isaac originals using saved USD camera/readback coordinates."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def report(out):
    layout=json.loads((out/'layout.json').read_text(encoding='utf-8'))
    views=json.loads((out/'view_cameras.json').read_text(encoding='utf-8'))
    check=json.loads((out/'check_report.json').read_text(encoding='utf-8'))
    TWD=np.array(layout['T_W_D_readback']);C=np.array(layout['camera']['T_W_C'])
    TDW=np.linalg.inv(TWD);camD=(TDW@C)[:3,3];axisD=(TDW@C)[:3,2]
    base=np.array(layout['base_T_D'])[:3,3];stack=np.array(layout['stack_bounds_D_m']).mean(axis=0)
    font=ImageFont.truetype('DejaVuSans.ttf',24)
    annotations=[]
    for view in views:
        im=Image.open(out/(view['name']+'.png')).convert('RGB');d=ImageDraw.Draw(im)
        inv=np.linalg.inv(view['T_W_view']);w,h=im.size
        def project(p):
            a=inv@TWD@[*p,1]
            ha=view['horizontal_aperture'];va=view['vertical_aperture']
            if view['projection']=='orthographic':x,y=a[0]/(.1*ha),a[1]/(.1*va)
            else:x,y=a[0]/(-a[2])*view['focal_length']/ha,a[1]/(-a[2])*view['focal_length']/va
            return np.array([w*(.5+x),h*(.5-y)])
        def text(pos,label,color=(255,255,255)):
            pos=tuple(np.asarray(pos,dtype=int));bb=d.textbbox(pos,label,font=font)
            d.rectangle((bb[0]-5,bb[1]-3,bb[2]+5,bb[3]+3),fill=(23,28,33));d.text(pos,label,font=font,fill=color)
        def arrow(a,b,label,color,label_offset=(8,8)):
            p=project(a);q=project(b);d.line([tuple(p),tuple(q)],fill=color,width=5)
            u=q-p;u/=max(np.linalg.norm(u),1e-12);v=np.array([-u[1],u[0]])
            d.polygon([tuple(q),tuple(q-16*u+7*v),tuple(q-16*u-7*v)],fill=color)
            text(q+label_offset,label,color)
            annotations.append(dict(view=view['name'],label=label,start_D=np.asarray(a).tolist(),end_D=np.asarray(b).tolist(),pixels=[p.tolist(),q.tolist()]))
        text([24,20],view['name']+' | USD readback | KINEMATIC VISUALIZATION')
        if view['name']=='layout_top':
            origin=np.array([-.95,.55,.02])
            arrow(origin,origin+[.3,0,0],'D +X (into stack)',(255,100,85))
            arrow(origin,origin+[0,.3,0],'D +Y',(100,230,130))
            arrow(base,base+[0,0,.2],'J1 vertical axis',(255,220,70))
            arrow(base+[0,0,.35],stack+[0,0,.35],'stack direction',(230,150,80))
            arrow(camD,camD+axisD*.28,'camera optical axis',(80,210,255),(-300,-42))
            for belt in layout['conveyors']:
                p=np.mean(belt['belt_bounds_D_m'],axis=0)
                arrow(p,p+np.array(belt['nominal_direction_D'])*.22,'belt -Y' if belt['nominal_direction_D'][1] else 'belt -X',(220,220,80))
        elif view['name']=='camera_mount_closeup':
            arrow(camD,camD+axisD*.18,'horizontal optical axis',(80,210,255))
            text([24,62],f"Optical height W: {C[2,3]:.4f} m; rigid parent: link_1")
            text([24,98],'XD500 actual CAD; existing silver mount and J1 seat retained')
        elif view['name']=='layout_front':
            columns=len({round(b['center_D_m'][1],6) for b in layout['boxes']})
            layers=len({round(b['center_D_m'][2],6) for b in layout['boxes']})
            size=' x '.join(str(round(x*1000)) for x in layout['boxes'][0]['size_m'])
            text([24,62],f"{columns} columns x {layers} layers | {len(layout['boxes'])} cartons | {size} mm")
            text([24,98],f"Stack W height: {layout['stack_bounds_D_m'][0][2]+TWD[2,3]:.3f} to {layout['stack_bounds_D_m'][1][2]+TWD[2,3]:.3f} m; roof off")
            text([24,134],'All occluders retained; front view can obscure lower layers')
        elif view['name']=='layout_side':
            text([24,62],f"USD W heights: tabletop {layout['table']['height_W_m']:.3f} m; belt {layout['conveyors'][0]['belt_height_W_m']:.3f} m")
            text([24,98],f"Robot base {TWD[2,3]+base[2]:.3f} m; optical centre {C[2,3]:.4f} m")
            text([24,134],'Temporary probe and floating calibration board retained')
        else:
            text([24,62],f"J1 face {np.degrees(layout['q_face']):.3f} deg; other joints fixed")
            text([24,98],'Known bracket/tool clearance shortfall unresolved; motion safety NOT EVALUATED')
        im.save(out/'annotated'/(view['name']+'_annotated.png'))
    (out/'annotation_projection.json').write_text(json.dumps(annotations,indent=2),encoding='utf-8')
    preview=Image.new('RGB',(1920,1200),(25,29,34));d=ImageDraw.Draw(preview)
    for i,view in enumerate(views):
        tile=Image.open(out/(view['name']+'.png'));tile.thumbnail((640,360))
        x=(i%3)*640;y=(i//3)*400;preview.paste(tile,(x,y));d.text((x+8,y+365),view['name'],font=font,fill='white')
    tile=Image.open(out/'keyframes/face_1.png');tile.thumbnail((480,360));preview.paste(tile,(1360,400))
    d.text((1288,765),'Actual mast camera: face_1',font=font,fill='white')
    d.text((20,840),'Same composed scene; no objects moved for framing.',font=font,fill='white')
    d.text((20,885),'KINEMATIC_VISUALIZATION | MOTION_SAFETY_NOT_EVALUATED | Dynamics: NOT_EVALUATED',font=font,fill='white')
    d.text((20,930),'The level mast camera sees only the upper part of the stack; review original sensor frames.',font=font,fill='white')
    preview.crop((0,0,1920,1000)).save(out/'layout_preview.jpg',quality=94)
    keys=Image.new('RGB',(1280,1040),(25,29,34));kd=ImageDraw.Draw(keys)
    for i,name in enumerate(['face_1','away_1','face_2','away_2']):
        x=i%2*640;y=i//2*520;keys.paste(Image.open(out/'keyframes'/(name+'.png')),(x,y));kd.text((x+10,y+487),name,font=font,fill='white')
    keys.save(out/'camera_keyframes_preview.jpg',quality=95)
    md=['# Current layout and J1 camera visualization','',
        'KINEMATIC_VISUALIZATION · MOTION_SAFETY_NOT_EVALUATED · Dynamics: NOT_EVALUATED','',
        f"Source commit: `{layout['source_commit']}` (code changes are bound separately after commit).",
        f"Snapshot: `{layout['scene_sources']['files']['snapshot']['path']}`.",
        f"Snapshot fingerprint: `{layout['snapshot_fingerprint']}`.",
        f"Selected camera scene: `{layout['selected_camera_USD']['path']}`; SHA256 `{layout['selected_camera_USD']['sha256']}`.",'',
        'All geometry was reused. Only official FK transforms change in the replay session. The saved derivative is frozen at q_face.',
        'USD readback: metres, Z up. D +X into trailer, +Y left. T_W_D = '+str(layout['T_W_D_readback']),
        'Table D bounds: '+str(layout['table']['bounds_D_m'])+f"; top W={layout['table']['height_W_m']:.6f} m.",
        'Base D translation: '+str(base.tolist()),
        'q_face joint radians: '+str(layout['q_face_rad']),
        'q2-q6 source: existing configuration.display_q, unchanged throughout.',
        f"q_face={np.degrees(layout['q_face']):.9f} deg; q_away={np.degrees(layout['q_away']):.9f} deg; sequence face / away / face / away.",
        'The scalar J1 solve includes the optical-centre orbit; no IK/path planner was called.','',
        '## Actual USD layout','', '| Carton | Centre D (m) | Size (m) | Snapshot delta (m) |','|---|---|---|---|']
    for b in layout['boxes']:md.append(f"| {b['id']} | {b['center_D_m']} | {b['size_m']} | {b['max_snapshot_delta_m']:.3g} |")
    md+=['','Stack envelope D: '+str(layout['stack_bounds_D_m']), 'Roof present: '+str(layout['roof_present']),
         'Carton appearance/proxy maximum bound difference: '+str(max(b['visual_proxy_bound_delta_m'] for b in layout['boxes']))+' m (AABB diagnostic, not penetration evidence).','']
    for b in layout['conveyors']:md += [b['id']+': USD D bounds '+str(b['bounds_D_m'])+f"; belt W top {b['belt_height_W_m']:.6f} m; nominal direction {b['nominal_direction_D']}; snapshot delta {b['snapshot_delta_m']:.3g} m."]
    md += ['', 'Mast parent: `'+layout['mast_parent']+'`. Camera prim: `'+layout['camera']['prim']+'`.',
           'Camera face W position: '+str(C[:3,3].tolist())+'; optical axis: '+str(C[:3,2].tolist())+'.',
           'Camera output: 640x480, 64 H x 50 V degrees; actual 1024x1024 64x64-degree render, ray-preserving remap.',
           'Optical assumption: '+layout['camera']['optical_assumption'],
           'Temporary board W transform: '+str(layout['temporary_attachments']['board_T_W']),
           'Temporary probe remains rigid under link_6. Board is the existing fixed simulation fixture.','',
           '## Checks and limits','',
           'Actual Isaac version: '+check['isaac_version']+'. Physical timeline stopped, no PLAY events; replay_time_s is independent.',
           f"Both MP4 files fully decoded: {check['timing']['frames']} frames, {check['timing']['fps']} fps, {check['timing']['duration_s']} seconds.",
           'Maximum transform/check errors: '+str(check['max_readback_errors']),
           'q2-q6 maximum change: '+str(check['q2_q6_max_change'])+' rad.',
           'Repeated hold-image comparisons: '+str(check['repeat_image_comparison']),
           f"Unique freshly rendered camera PNGs: {check['unique_camera_images']}.",
           f"Rendering/check time: {check['render_seconds']:.2f} s; total with encoding/decode: {check['total_seconds']:.2f} s (not replay time).",'',
           'Known bracket/tool clearance is about 1 mm against the prior 2.4 mm criterion. It remains unresolved; this entry does not change policy or calibration acceptance.',
           'The level camera is higher than the stack. Only partial stack coverage is possible. Real occluders and the board were preserved. Image visibility and any suspected interference require visual review; no new collision qualification was run.',
           'All positions/sizes above are actual composed USD readback; nominal conveyor directions and replay timing are configuration. Optical pupil and fixed board support are simulation assumptions.','']
    (out/'layout_summary.md').write_text('\n'.join(md),encoding='utf-8')
    files={p.relative_to(out).as_posix():dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in out.rglob('*') if p.is_file() and p.name!='delivery_manifest.json'}
    (out/'delivery_manifest.json').write_text(json.dumps(files,indent=2),encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path);report(p.parse_args().output)
