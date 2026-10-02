"""Read-only before/after presentation of the fixed stage repair results."""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
from PIL import Image,ImageDraw,ImageFont


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('before','after','diagnosis','comparison','font'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();old=json.loads((a.before/'data.json').read_text());new=json.loads((a.after/'data.json').read_text())
    diagnosis=json.loads(a.diagnosis.read_text());comparison=json.loads(a.comparison.read_text())
    out=a.after/'repair-review';out.mkdir(exist_ok=False)
    font=ImageFont.truetype(str(a.font),24)
    def image_grid(name,rows):
        canvas=Image.new('RGB',(1440,500*len(rows)),'#101720');draw=ImageDraw.Draw(canvas)
        for row,items in enumerate(rows):
            for col,(path,title) in enumerate(items):
                im=Image.open(path).convert('RGB');im.thumbnail((1420//len(items),430))
                x=col*1440//len(items);y=row*500
                draw.text((x+12,y+10),title,font=font,fill='white');canvas.paste(im,(x+(1440//len(items)-im.width)//2,y+60))
        canvas.save(out/name)
    def module(data,frame,name):return next(m for g in data['groups'] if g['source_sequence']==frame for m in g['modules'] if m['module_id']==name)
    def inst(data):return next(i for i in module(data,602,'module_0_upper')['instances'] if i['mask_id']==1)
    x,y=inst(old),inst(new)
    image_grid('mask1-support-faces.png',[
        [(a.before/x['images'][k],label) for k,label in [('filter','旧03：旁路滤波'),('labels','旧04：独立支持 / 7 标签'),('faces','旧06：3 个认证面')]],
        [(a.after/y['images'][k],label) for k,label in [('filter','新03：实际测量父支持'),('labels','新04：只消费父支持'),('faces','新06：保留主面 / 窄带有记录')]]])
    from render_perception_stage_video import spatial_plot,fusion_plots
    spatial_plot(out/'mask1-old-3d.png',x);spatial_plot(out/'mask1-new-3d.png',y)
    image_grid('mask1-geometry.png',[[(out/'mask1-old-3d.png','旧：真实三面 / 等米制'),(out/'mask1-new-3d.png','新：主面 / 同视角、同坐标范围')]])
    lower=module(new,32,'module_1_lower');oldlower=module(old,32,'module_1_lower')
    baseline=a.diagnosis.parent/'frame32-module_1_lower-prompt-baseline.png'
    image_grid('frame32-lower-prompts.png',[[(baseline,'旧：原提示黄 / 渲染蓝 / SAM绿'),(a.after/lower['prompt_binding_overlay'],'新：原提示未移动 / 新 SAM 边界')]])
    # All source instances get a native-pixel comparison, not just a chosen success.
    prompt_crops=[]
    for i in lower['instances']:
        b=next(v for v in oldlower['instances'] if v['mask_id']==i['mask_id'])
        bounds=i['bounds'];prior=Image.open(baseline).crop(bounds)
        target=out/f"prompt-{i['mask_id']:02d}-before.png";prior.save(target)
        dest=out/f"prompt-{i['mask_id']:02d}-after.png";shutil.copyfile(a.after/i['images']['prompts'],dest)
        prompt_crops.append(dict(mask_id=i['mask_id'],before=target.name,after=dest.name,audit=i['prompt_audit']))
    rows=[]
    for ng in new['groups']:
        og=next(g for g in old['groups'] if g['name']==ng['name'])
        target=out/ng['name'];target.mkdir();fusion_plots(target,ng,ng['objects'][0])
        instances=[]
        for nm in ng['modules']:
            om=next(m for m in og['modules'] if m['module_id']==nm['module_id'])
            for ni in nm['instances']:
                oi=next(i for i in om['instances'] if i['mask_id']==ni['mask_id'])
                instances.append(dict(module=nm['module_id'],mask_id=ni['mask_id'],
                    old_faces=len(oi['surfaces']),new_faces=len(ni['surfaces']),
                    old_raw=oi['raw_cloud']['total'],new_raw=ni['raw_cloud']['total'],
                    old_bypass_retained=oi['filtered_cloud']['total'],new_consumed_parent=ni['filtered_cloud']['total'],
                    quality=ni['record'].get('face_quality',[]),
                    final_checks=ni['record'].get('final_face_validation',[]),
                    patch_methods=[f.get('patch_boundary_method') for f in ni['record'].get('camera_facing_faces',[])]))
        rows.append(dict(group=ng['name'],old_unknown=og['unknown_count'],new_unknown=ng['unknown_count'],
            old_objects=len(og['objects']),new_objects=len(ng['objects']),instances=instances,
            objects=[dict(id=o['id'],members=o['members'],**o['summary']) for o in ng['objects']]))
    report=dict(groups=rows,prompt_crops=prompt_crops,comparison=comparison,
        black_diagnosis=[dict(group=r['group'],module=r['module'],**r['black']) for r in diagnosis['groups']],
        mask1_quality=y['record']['face_quality'],source_changes='original images/bindings not modified',
        limitations=['No first-hit numeric image saved for historical continuous frames; exact black occluder unidentified',
                    'Future virtual rig transform consistency fixed and CPU tested, no new Isaac capture',
                    'No available IAB/Chrome automation; browser interaction NOT_VERIFIED',
                    'Local geometric contact support is not material seal, collision, IK or execution'])
    (out/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    pictures=[('frame32-lower-prompts.png','提示框逐项与绑定的渲染边界一致；透视轴对齐外接框可以重叠。'),
        ('mask1-support-faces.png','统一支持真实进入04。窄带保留为边缘证据，未成为自由空间。'),
        ('mask1-geometry.png','8–9 mm 窄带退出共享拟合；内接面片四角仍不是物理箱角。')]
    for r in rows:
        for mode in ('before','after'):pictures.append((r['group']+'/fusion-'+mode+'.png',r['group']+' / '+mode+' / 所有来源面或所有实际代表面'))
    html='<meta charset="utf-8"><title>本轮修复对照</title><style>body{background:#101720;color:#eef;font:18px sans-serif;margin:30px}img{max-width:100%;height:auto}a{color:#7df}section{margin:35px 0}</style><h1>真实产物修复对照 · READ ONLY</h1><a href="../index.html">打开全部对象、实例与十阶段查看器</a><p>原始输入未改变。全分辨率最终运行；半分辨率 mask 11 边界 P95=3 px，超过预设2 px，未采用。</p>'
    html+=''.join(f'<section><p>{caption}</p><a href="{path}"><img src="{path}"></a></section>' for path,caption in pictures)
    html+='<h2>全部下模组提示的原生像素放大</h2>'+''.join(f'<p>mask {r["mask_id"]}: <a href="{r["before"]}">原图输入核对</a> / <a href="{r["after"]}">实际五组扰动与新SAM</a></p>' for r in prompt_crops)
    html+='<p><a href="review.json">完整逐实例、逐对象结果与对照表</a></p><p>浏览器交互未核验：本环境无可用浏览器工具；图片与文件完整性另有实测记录。</p>'
    (out/'index.html').write_text(html,encoding='utf-8')
    print(json.dumps(dict(groups=len(rows),instances=sum(len(r['instances']) for r in rows),prompt_crops=len(prompt_crops))))


if __name__=='__main__':main()
