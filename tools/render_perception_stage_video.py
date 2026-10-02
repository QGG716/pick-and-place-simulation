"""1920x1080 result-review video from the current portable viewer data, no inference."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from time import perf_counter
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))

TITLES=['原始 RGB-D 与标定','实际提示输入与 SAM','实例深度支持与点云','初始平面与冻结标签',
        '米制拟合与独立验证','认证面片与完整体判定','双模组融合与来源','显式尺寸先验实验',
        '局部吸附姿态与逐杯支持','真实 ROS / RViz 接纳']


def contact_document(folder,obj):
    text=(folder/obj['contacts']).read_text(encoding='utf-8')
    return json.loads(text[text.index(',')+1:-2])


def metric_limits(ax,points):
    points=np.asarray(points,float)
    if not len(points):return
    center=(points.min(0)+points.max(0))/2;radius=max(.1,float(np.ptp(points,axis=0).max())*.65)
    ax.set(xlim=(center[0]-radius,center[0]+radius),ylim=(center[1]-radius,center[1]+radius),zlim=(center[2]-radius,center[2]+radius))
    ax.set_box_aspect((1,1,1))


def spatial_plot(path,instance,initial=False):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(10,7));ax=fig.add_subplot(projection='3d')
    xyz=np.asarray(instance['raw_cloud']['points'])
    if len(xyz):ax.scatter(*xyz.T,s=1,c='steelblue',alpha=.3)
    if initial:
        for p in instance['initial_points']:
            pts=np.asarray(p['sample']['points'])
            if not len(pts):continue
            n=np.asarray(p['normal']);center=pts.mean(0);center-=n*(center@n+p['offset_m'])
            ax.quiver(*center,*(n*.1),color='darkorange')
            helper=np.eye(3)[np.argmin(abs(n))];u=np.cross(n,helper);u/=np.linalg.norm(u);v=np.cross(n,u)
            uv=(pts-center)@np.array([u,v]).T;lo=uv.min(0);hi=uv.max(0)
            corners=center+np.array([[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]],[lo[0],lo[1]]])@np.array([u,v])
            ax.plot(*corners.T,color='darkorange',label='initial plane display extent')
    for f in instance['record'].get('camera_facing_faces',[]):
        if initial:continue
        q=np.asarray(f['corners_3d_m']);ax.plot(*np.vstack([q,q[0]]).T,color='green')
        ax.quiver(*q.mean(0),*(np.asarray(f['plane_normal'])*.1),color='green')
    ax.set(xlabel='camera X (m)',ylabel='camera Y (m)',zlabel='camera Z (m)',title='ACTUAL INITIAL PLANES' if initial else 'FINAL CERTIFIED PATCHES')
    metric_limits(ax,xyz)
    ax.view_init(25,-70);fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)


def contact_plots(folder,out,g,obj,c,tool):
    from unloading_sim.grasp import suction_cup_layout_geometry
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    centers,_,_=suction_cup_layout_geometry(tool['layout']);rad=tool['layout']['cup_radius_m']
    rotation=np.asarray(c['candidate']['grasp_pose'])[:3,:3];point=np.asarray(c['candidate']['contact_point'])
    color=lambda reason: '#45b986' if reason is None else '#f49b42' if reason=='CUP_LIP_OUTSIDE_OBSERVED_PATCH' else '#b17cec' if reason=='LOCAL_DEPTH_PLANE_ERROR' else '#ee519b' if reason=='CUP_IMAGE_CROPPED' else '#de5049'
    rings=[]
    for center in centers:
        theta=np.linspace(0,2*np.pi,49);rings.append(point+rotation[:,:2]@center+np.cos(theta)[:,None]*rad*rotation[:,0]+np.sin(theta)[:,None]*rad*rotation[:,1])
    source=c['source']['source'];mod=next(m for m in g['modules'] if m['module_id']==source['module_id'])
    inst=next(i for i in mod['instances'] if i['id']==source['source_instance_id'])
    fig=plt.figure(figsize=(10,7));ax=fig.add_subplot(projection='3d');allpoints=[]
    for f in obj['surfaces']:
        q=np.asarray(f['corners_3d_m']);ax.plot(*np.vstack([q,q[0]]).T,color='steelblue');allpoints.extend(q)
    for r,cup in zip(rings,c['cups']):ax.plot(*r.T,color=color(cup['reason']));allpoints.extend(r)
    pre=np.asarray(c['candidate']['pregrasp_pose'])[:3,3];ax.quiver(*pre,*(point-pre),color='purple')
    for j,col in enumerate(('red','green','blue')):ax.quiver(*point,*(rotation[:,j]*.08),color=col)
    allpoints=np.asarray([*allpoints,pre]);mid=(allpoints.min(0)+allpoints.max(0))/2;span=max(np.ptp(allpoints,axis=0))*.65
    ax.set(xlim=(mid[0]-span,mid[0]+span),ylim=(mid[1]-span,mid[1]+span),zlim=(mid[2]-span,mid[2]+span),
        xlabel='world X (m)',ylabel='world Y (m)',zlabel='world Z (m)',title=f"BEST (not qualified): {c['candidate']['score']:.0f}/{c['required_cups']} cups")
    ax.set_box_aspect((1,1,1));ax.view_init(25,145);fig.tight_layout();fig.savefig(out/'contact-3d.png',dpi=140);plt.close(fig)
    image=Image.open(folder/mod['rgb']).convert('RGB');T=np.asarray(mod['camera']['T_W_C']);K=np.asarray(mod['camera']['K']).reshape(3,3)
    def project(p):
        camera=(p-T[:3,3])@T[:3,:3];xy=camera@K.T;return xy[:,:2]/xy[:,2,None]
    draw=ImageDraw.Draw(image)
    # The real valid-support overlay already exists as an unresampled pixel crop.
    crop=Image.open(folder/inst['images']['faces']).convert('RGB');image.paste(crop,tuple(inst['bounds'][:2]));draw=ImageDraw.Draw(image)
    pix=[project(r) for r in rings]
    for p,cup in zip(pix,c['cups']):draw.line([tuple(v) for v in p],fill=color(cup['reason']),width=3)
    allpix=np.concatenate(pix);b=inst['bounds'];lo=np.floor(np.minimum(allpix.min(0),b[:2])-25).astype(int);hi=np.ceil(np.maximum(allpix.max(0),b[2:])+25).astype(int)
    image.crop((max(0,lo[0]),max(0,lo[1]),min(image.width,hi[0]),min(image.height,hi[1]))).save(out/'contact-projection.png')


def fusion_plots(out,group,selected):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    allpoints=np.array([p for m in group['modules'] for i in m['instances'] for f in i['surfaces'] for p in f['corners_3d_m']])
    for after in (False,True):
        fig=plt.figure(figsize=(10,7));ax=fig.add_subplot(projection='3d');count=0
        if not after:
            surfaces=[(f,'steelblue' if j==0 else 'mediumpurple',1) for j,m in enumerate(group['modules']) for i in m['instances'] for f in i['surfaces']]
        else:
            surfaces=[]
            for obj in group['objects']:
                representatives=obj['diagnostics'].get('face_reduction',{}).get('representatives',[])
                keys={(r['module_id'],r['source_instance_id'],r['face_id']) for r in representatives}
                surfaces += [(f,'darkorange' if obj['id']==selected['id'] else 'seagreen',3 if obj['id']==selected['id'] else 1)
                             for f in obj['surfaces'] if (f['module_id'],f['source_instance_id'],f['face_id']) in keys]
        for f,color,width in surfaces:
            q=np.asarray(f['corners_3d_m']);ax.plot(*np.vstack([q,q[0]]).T,color=color,linewidth=width);count+=1
        ax.set(xlabel='world X (m)',ylabel='world Y (m)',zlabel='world Z (m)',
            title=('FUSED REPRESENTATIVES' if after else 'UPPER / LOWER OBSERVED FACES')+f' ({count})')
        metric_limits(ax,allpoints)
        ax.view_init(25,145);fig.tight_layout();fig.savefig(out/('fusion-after.png' if after else 'fusion-before.png'),dpi=140);plt.close(fig)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--viewer',type=Path,required=True);p.add_argument('--font',type=Path,required=True)
    a=p.parse_args(argv);started=perf_counter();data=json.loads((a.viewer/'data.json').read_text());out=a.viewer/'keyframes';out.mkdir(exist_ok=False)
    font=ImageFont.truetype(str(a.font),27);titlefont=ImageFont.truetype(str(a.font),38);small=ImageFont.truetype(str(a.font),22)
    frames=[]
    def text(draw,xy,value,width=29,fill='#dce8f3',f=font):
        y=xy[1]
        for paragraph in str(value).split('\n'):
            for pos in range(0,max(1,len(paragraph)),width):draw.text((xy[0],y),paragraph[pos:pos+width],font=f,fill=fill);y+=38
        return y
    def panel(canvas,path,box,label):
        draw=ImageDraw.Draw(canvas);draw.text((box[0],box[1]),label,font=small,fill='#dce8f3')
        if path and Path(path).exists():
            image=Image.open(path).convert('RGB');image.thumbnail((box[2],box[3]-38));canvas.paste(image,(box[0]+(box[2]-image.width)//2,box[1]+38))
        else:draw.text((box[0],box[1]+90),'NOT_RECORDED / 无结果',font=font,fill='#f5af4c')
    for g in data['groups']:
        obj=g['objects'][0];doc=contact_document(a.viewer,obj);c=doc['candidates'][0] if doc['candidates'] else None
        source=c['source']['source'] if c else dict(module_id=obj['members'][0][0],source_instance_id=obj['members'][0][1])
        mod=next(m for m in g['modules'] if m['module_id']==source['module_id']);inst=next(i for i in mod['instances'] if i['id']==source['source_instance_id'])
        gd=out/g['name'];gd.mkdir();spatial_plot(gd/'initial-3d.png',inst,True);spatial_plot(gd/'final-3d.png',inst,False)
        fusion_plots(gd,g,obj)
        if c:contact_plots(a.viewer,gd,g,obj,c,data['tool'])
        events=[e for e in (mod.get('trace') or {}).get('events',[]) if e['mask_id']==inst['mask_id']]
        for stage,title in enumerate(TITLES):
            canvas=Image.new('RGB',(1920,1080),'#101720');draw=ImageDraw.Draw(canvas)
            draw.text((36,25),f'{stage+1:02d}  {title}',font=titlefont,fill='white')
            draw.text((36,82),f"{g['name']} / {mod['module_id']} / SAM mask {inst['mask_id']} / {obj['id']}",font=small,fill='#7ed7ef')
            draw.text((36,123),'本次新算法结果回放 · 仿真采集 · ORACLE_PROPOSAL · READ ONLY · 非实时推理',font=small,fill='#ffcf7a')
            def pair(p1,p2,l1,l2):panel(canvas,p1,(35,180,640,740),l1);panel(canvas,p2,(700,180,640,740),l2)
            if stage==0:
                for j,m in enumerate(g['modules']):
                    panel(canvas,a.viewer/m['rgb'],(35+j*670,180,635,365),m['module_id']+' RGB')
                    panel(canvas,a.viewer/m['depth'],(35+j*670,570,635,365),'Depth：统一 0–5 m；粉色无效')
                note=f"源时间 {g['capture_time']:.6f} s\nros_sim_time\n原始 2592×1944\n使用采集时 K / T_W_C\n相机模型与哈希在查看器\nGT 未用于补全几何"
                elapsed='输入核验 '+str(mod['one_shot_stage_seconds'].get('payload_validation','未拆时'))+' s'
            elif stage==1:
                pair(a.viewer/mod['prompts'],a.viewer/mod['masks'],'实际送入 SAM 的提示框','实际 SAM 实例彩色 mask')
                note=f"本模组 {len(mod['instances'])} 个实例\n分数按原记录保留\n遮挡/粘连/重复：\n见独立 mask 评价页\n全部实例可检索\n原生裁剪见查看器"
                elapsed='SAM 阶段 '+str(mod['one_shot_stage_seconds'].get('sam_inference','未拆时'))+' s'
            elif stage==2:
                pair(a.viewer/inst['images']['raw'],a.viewer/inst['images'].get('filter',inst['images']['retained']),'原始有效深度支持','绿父支持 / 橙边缘 / 紫跳变（新结果）')
                note=f"原有效点 {inst['raw_cloud']['total']}\n过滤保留 {inst['filtered_cloud']['total']}\n无效深度、孔洞不补平\n点云显示 stride：\n{inst['raw_cloud']['stride']} / {inst['filtered_cloud']['stride']}\n算法保持全分辨率"
                elapsed='旁路点图 '+format(mod['timings']['pointmap_build_including_filter_audits'],'.3f')+' s；主支持计入面片阶段'
            elif stage==3:
                pair(a.viewer/inst['images'].get('labels',inst['images']['mask']),gd/'initial-3d.png','拟合前实际标签','初始平面和法向（不是最终平面）')
                note=f"真实初始标签：\n{len(inst['initial_points'])} 个区域\n冻结后进入拟合\n未分类像素保留\n空间单位 m\n显示范围不代表箱边界"
                elapsed='本实例提取 '+str(events[0]['seconds'] if events else 'NOT_RECORDED')+' s'
            elif stage==4:
                pair(gd/'initial-3d.png',gd/'final-3d.png','初始平面 / 相同相机坐标视角','最终独立验证面片')
                checks=inst['record'].get('final_face_validation',[])
                note=f"最终检查 {len(checks)} 项\n通过 {sum(r.get('status')=='PASS' for r in checks)} 项\n保留空间块留出检查\n平均残差门槛 3 mm\n详细支持域、残差、\n拒绝原因见原记录"
                elapsed='本实例拟合/校验 '+str(events[1]['seconds'] if len(events)>1 else 'NOT_RECORDED')+' s'
            elif stage==5:
                pair(a.viewer/inst['images']['raw'],a.viewer/inst['images']['faces'],'原始有效支持','内接四边形 / 未表达支持仍可见')
                note=f"认证面片 {len(inst['surfaces'])}\n完整箱体：\n{'ACCEPTED' if inst['record'].get('accepted') else 'UNKNOWN / 未通过'}\n四角不是物理箱角\n未观测体积未补齐\n不画虚构实心箱体"
                elapsed='含在米制拟合中；观测转换 '+str(mod['timings']['final_validation_and_observation'])+' s'
            elif stage==6:
                pair(gd/'fusion-before.png',gd/'fusion-after.png','融合前：上模组青 / 下模组紫','融合后代表面；橙色为当前对象')
                note=f"本组融合对象 {len(g['objects'])}\n当前对象来源 {len(obj['members'])}\n当前原观测面 {len(obj['surfaces'])}\nunknown {g['unknown_count']}\n逐面关联与代表选择\n见查看器 3D 与诊断\n不是跨帧稳定跟踪 ID"
                elapsed='融合/产物 '+str(g['run_stage_seconds'].get('algorithm_handoff','未拆时'))+' s'
            elif stage==7:
                text(draw,(110,280),'NOT_RUN\n显式尺寸先验实验未启用\n没有派生包络或执行通过结论',width=30,f=titlefont,fill='#a9bacb')
                note='独立离线分支\n不改变主链准入\n条件包络不是视觉完整体\n更不能作为吸附面'
                elapsed='NOT_RUN'
            elif stage==8:
                pair(gd/'contact-3d.png',gd/'contact-projection.png','世界坐标：杯盘 / TCP / 预接近','原 RGB 投影：支持域与全部杯盘')
                note=(f"最佳候选：\n{c['candidate']['score']:.0f}/{c['required_cups']} 杯\n分区 {c['candidate']['sealed_cups_per_zone']}\n绿：几何支持\n橙：超出面片\n红：孔洞/遮挡/无效\n紫：局部深度误差\n粉：图像裁切\n最佳 ≠ 合格" if c else
                      '当前对象无接触候选\n'+str(obj['summary'].get('face_errors') or obj['summary'].get('reason')))
                elapsed='两组联合适配 '+str(data['contacts_timing']['adapter_wall_seconds'])+' s（非单候选）'
            else:
                panel(canvas,a.viewer/('rviz-'+g['name']+'.png'),(35,180,1305,740),'本组真实 RViz 截图；完整组切换见 ros-rviz.mp4')
                note=f"{g['ros']['status']}\nplanning_admissible=false\n旧 Marker 删除：\n{g['ros'].get('previous_marker_deletes','未运行')}\n源时间和发布时间分列\n不授予抓取或执行资格"
                elapsed='ROS 交付 '+str(g['ros'].get('accepted_monotonic',0)-g['ros'].get('submitted_monotonic',0))+' s（同一单调时钟）'
            import re
            elapsed=re.sub(r'\d+\.\d+',lambda m:format(float(m.group()),'.3f'),elapsed)
            text(draw,(1380,195),'输入 → 处理 → 输出',width=21,fill='#7ed7ef')
            text(draw,(1380,255),note,width=21)
            text(draw,(36,966),'实际计算：'+elapsed,width=92,f=small,fill='#ffcf7a')
            draw.text((36,1020),'回看停留时间与算法耗时分开；IK / 刚体碰撞 / 密封 / 吸附力 / 动力学：NOT_EVALUATED',font=small,fill='#b1c0d0')
            target=gd/f'{stage+1:02d}.png';canvas.save(target);frames.append(target)
    concat=out/'slides.ffconcat';concat.write_text('ffconcat version 1.0\n'+''.join(f"file '{p.resolve().as_posix()}'\nduration 6\n" for p in frames)+f"file '{frames[-1].resolve().as_posix()}'\n")
    subprocess.run(['ffmpeg','-y','-f','concat','-safe','0','-i',str(concat),'-vf','fps=10,format=yuv420p',
                    '-c:v','libx264','-preset','fast','-crf','20','-movflags','+faststart',str(a.viewer/'algorithm-review.mp4')],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    (a.viewer/'video-render.json').write_text(json.dumps(dict(seconds=perf_counter()-started,resolution=[1920,1080],
        origin='CURRENT_RUN_RESULT_REVIEW_NOT_REALTIME',selection='first fused object in each fixed group, original best candidate',frames=len(frames)),indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
