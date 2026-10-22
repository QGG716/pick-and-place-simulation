"""Package only the current fresh-world evidence and accepted native replay."""
from pathlib import Path
from collections import Counter
import hashlib,html,json,re,shutil,zipfile

root=Path(__file__).resolve().parents[2];art=Path(__file__).resolve().parent
dest=root/'docs/experiments/m710_population_x500_20261022'
dest.mkdir(parents=True,exist_ok=True)
(dest/'.gitattributes').write_text('* -text -whitespace\n',encoding='utf-8')
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
save=lambda p,v:p.write_text(json.dumps(v,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert read(art/'browser/browser-checks.json')['status']=='PASS'
assert sha(art/'formal-raw.tar.xz')==read(art/'formal-raw-receipt.json')['sha256']
names=['summary.json','protocol.json','verification.json','initial-state-check.json','source-check.json',
       'source-manifest.json','source-receipt.json','measured-source.tar.gz','formal-raw.tar.xz',
       'formal-raw-receipt.json','formal-v1.log','formal-v1.exit','focused-tests.log','formal-v1.sh',
       'all-targets.json','costs.json','failure-diagnostics.json','accepted-cycles.html',
       'accepted-cycles-checks.json','prepare.py','prepare_analysis.py','summarize-cycles.py',
       'failure-diagnostics.py','pack-evidence.py','analyze.sh','deploy-and-run.sh','build_delivery.py']
for n in names:shutil.copyfile(art/n,dest/n)
for n in ['browser-checks.json','initial.png','final.png']:shutil.copyfile(art/'browser'/n,dest/n)
for n in ['l03-c02.html','l03-c02-checks.json']:shutil.copyfile(art/'inspection'/n,dest/n)
for source,name in [('browser-checks.json','l03-c02-browser-checks.json'),('failed-ik.png','l03-c02.png'),('before-lowering.png','l03-before-lowering.png')]:
    shutil.copyfile(art/'inspection/browser'/source,dest/name)
shutil.copyfile(root/'tools/search_replay/vendor/LICENSE.three',dest/'LICENSE.three')
shutil.copyfile(root/'docs/experiments/m710_l05_x500_20261022/LICENSE.FANUC.txt',dest/'LICENSE.FANUC.txt')
s=read(dest/'summary.json');v=read(dest/'verification.json');rows=read(dest/'all-targets.json')['rows']
d={r['target_id']:r for r in read(dest/'failure-diagnostics.json')['rows']}
c=s['completed_count'];n=len(s['outcomes']);b=len(s['blocked']);source=s['source_commit']
assert len(rows)==40 and c==sum(r['complete'] for r in rows) and n+b==40
old=root/'docs/experiments/m710_population_x400_20261021'
old_s=read(old/'summary.json');old_rows=read(old/'all-targets.json')['rows']
before={r['target_id'] for r in old_rows if r['complete']};now={r['target_id'] for r in rows if r['complete']}
compare=dict(previous_source_commit=old_s['source_commit'],source_commit=source,
    previous_completed=len(before),current_completed=c,denominator=40,gained=sorted(now-before),lost=sorted(before-now),
    initial_cartons_equal=s['initial']['world_cartons']==old_s['initial']['world_cartons'],
    initial_q_equal=s['initial']['q_rad']==old_s['initial']['q_rad'],initial_height_equal=s['initial']['height_m']==old_s['initial']['height_m'],
    current_mount_world_m=[-1.825,.25,.6],previous_mount_world_m=[-1.725,.25,.6],
    scope='SEPARATE_MEASURED_NEW_WORLD_RUNS_NOT_NONREGRESSION_PROOF')
assert all(compare[k] for k in ['initial_cartons_equal','initial_q_equal','initial_height_equal'])
save(dest/'comparison.json',compare)

def detail(r):
    if r['complete']:
        a=r['accepted']
        return f"{a['face']}；delta={a['delta_m']:.5f} m；接收 {a['receive_height_m']:.2f} m；{a['synchronization']}；实际 {a['cups']['actual_contact_mask']}/72 杯；{r['wall_s']:.2f} s"
    if r.get('supported_cartons'):
        return '仍支撑 '+', '.join(x.replace('carton_','') for x in r['supported_cartons'])+'；未派发原生搜索。'
    attempts=d[r['target_id']]['attempts']
    velocities=[e for a in attempts for e in a['native_velocity_limit_events']]
    witnesses=[w for a in attempts for w in a['collision_witnesses']]
    counts=r['counts'];parts=[]
    if any(a['region_failure_counts'].get('TOOL_DEPARTURE') for a in attempts):
        parts.append('最远到工具分离检查，但完整释放后缀未接受')
    elif any(a['contact_complete'] for a in attempts):
        parts.append('接近和contact通过，后续抽离/接收未完整接受')
    elif counts.get('pregrasp'):
        parts.append('接近轨迹通过，contact未完成')
    elif counts.get('numerical_ik_success'):
        parts.append('有数值IK成功，未完成接近轨迹')
    else:parts.append('预算内未获得数值IK解，未到contact')
    if velocities:
        parts.append('原生速度超限记录：'+', '.join(sorted({e['joint'] for e in velocities}))+f'，共{len(velocities)}条')
    if witnesses:
        pairs=sorted({tuple(w['witness']['pair']) for w in witnesses})
        parts.append('本轮碰撞见证：'+'；'.join(' / '.join(p) for p in pairs))
    else:parts.append('无本轮碰撞拒绝见证')
    if r.get('unsearched_options'):parts.append(f"另有{r['unsearched_options']}个未搜索工艺选项")
    parts.append(f"实际派发{r['dispatched_candidates']}个工艺候选，耗时{r['wall_s']:.2f} s")
    return '；'.join(parts)+'。'

table=[]
for r in sorted(rows,key=lambda r:(-r['layer_index'],r['target_id'])):
    status='完整接受' if r['complete'] else ('承重未搜索' if r.get('supported_cartons') else '规划未完成')
    table.append('<tr><td>'+html.escape(r['target_id'].replace('carton_',''))+'</td><td>'+status+'</td><td>'+html.escape(detail(r))+'</td></tr>')
status='全部卸完' if c==40 else '尚未卸完'
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>−X500 mm · 全新40箱结果</title><style>
body{margin:0;background:#142132;color:#edf3fb;font:16px/1.65 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1160px;margin:auto;padding:30px}a{color:#8adbea}table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:9px;border:1px solid #41556b;text-align:left;overflow-wrap:anywhere}th{background:#23394e}.links a{display:inline-block;padding:12px 20px;background:#29465d;margin:4px;border-radius:8px}small{color:#b5c7db}</style><main>'''
page+=f'<h1>完整抓放 {c}/40 · {c/40:.1%} · {status}</h1>'
page+=f'<p>本轮从全新40箱、空历史台账开始，现场40→{40-c}箱。{n}箱实际搜索，{c}箱完整接受，{n-c}箱规划未完成，{b}箱仍承重而未搜索。</p>'
page+='<p>累计 −X500 mm、−Y100 mm；机器人基座 [−1.825, 0.250, 0.600] m。低层顶吸、横带固定最低0.60 m；中高层沿用原同步规则。全部关节位置、速度、加速度和减速度限制保留，本轮没有 l05_c03 速率豁免。</p>'
page+='<div class="links"><a href="accepted-cycles.html">本轮连续三维回放</a><a href="all-targets.json">逐箱记录</a><a href="failure-diagnostics.json">失败原始诊断摘要</a><a href="costs.json">全部尝试成本</a></div>'
page+='<div class="links"><a href="l03-c02.html">l03_c02 顶吸IK失败三维检查</a></div>'
page+='<p>回放由本轮原生q、实际父链和同源机构几何驱动。完整接受包含实际contact几何核验及附着、抽离、接收区域验收、同位姿解除附着、名义下落、名义首次触带、工具分离和 IDEAL_RECEIVER_DRAIN。失败路径不拼入已提交连续世界。</p>'
page+='<p>全部72个物理杯保留。选中且几何有效的杯采用 ideal_independent_cups 吸力足够假设。名义下落期间工具保持释放q；名义接收和理想清空不属于实测物理接收，本轮物理执行 NOT_RUN。</p>'
page+=f'<p>300秒单箱任务预算、90秒共享前缀预算、seed=71070。运行及归档墙钟 {s["planning_wall_s"]:.3f} s，箱任务调用累计 {s["native_task_wall_s"]:.3f} s，两者包含关系，不能相加。预算内未完成不证明全局不可达。</p>'
page+='<table><thead><tr><th>箱体</th><th>结果</th><th>实际记录</th></tr></thead><tbody>'+''.join(table)+'</tbody></table>'
page+='<p><a href="verification.json">原生父链与完整周期核验</a> · <a href="comparison.json">上一布局独立整批结果对照</a> · <a href="formal-raw.tar.xz">本轮全部原始记录</a> · <a href="measured-source.tar.gz">实测源码及模型</a> · <a href="README.txt">复现说明</a></p>'
page+=f'<small>实测源码 {source}<br>qualification_status=NOT_EVALUATED · continuous_swept_proof=false<br>physical_execution=NOT_RUN · execution_ready=false</small></main></html>'
(dest/'index.html').write_text(page,encoding='utf-8')
readme=f'''累计−X500 mm、−Y100 mm布局，全新40箱连续规划：{c}/40，{c/40:.1%}，{status}。
实际搜索{n}箱，规划未完成{n-c}箱，承重未搜索{b}箱；现场40→{40-c}箱。
实测源码：{source}
完整关节限制保留；历史单箱成功轨迹/附着不注入。
打开index.html查看报告，accepted-cycles.html查看本轮原生轨迹。
formal-raw.tar.xz保留全部原始证据字节与SHA256；measured-source.tar.gz包括本轮源码与同源模型。
在本次既有隔离环境中复现（输出目录必须尚不存在）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /bin/bash /population-x500-20261022/formal-v1.sh /reproduce-population-x500
入口与全部参数见formal-v1.sh；普通worker.sh和ROS环境沿用前一轮已恢复的固定环境，无依赖升级。
浏览器核验记录：browser-checks.json；运行源码/环境核验：source-check.json。
本轮只是规划。名义下落、名义接收与理想清空不属于物理执行。
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
最终交付提交SHA及推送状态见交付答复；此文件不自引用其提交SHA。
'''
(dest/'README.txt').write_text(readme,encoding='utf-8')
offline=['index.html','accepted-cycles.html','all-targets.json','failure-diagnostics.json','costs.json','verification.json','comparison.json','README.txt','browser-checks.json','accepted-cycles-checks.json','initial.png','final.png','LICENSE.three','LICENSE.FANUC.txt','l03-c02.html','l03-c02-checks.json','l03-c02-browser-checks.json','l03-c02.png','l03-before-lowering.png']
offline_page=re.sub(r'<a href="(?:formal-raw.tar.xz|measured-source.tar.gz)">.*?</a>','原始归档见仓库报告目录',page)
with zipfile.ZipFile(dest/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name in offline:
        if name=='index.html':z.writestr(name,offline_page.encode())
        else:z.write(dest/name,name)
with zipfile.ZipFile(dest/'offline-replay.zip') as z:
    assert z.testzip() is None
    for ref in re.findall(r'href="([^"]+)"',z.read('index.html').decode()):assert ref in z.namelist(),ref
save(dest/'manifest.json',dict(source_commit=source,sha256={p.name:sha(p) for p in sorted(dest.iterdir()) if p.is_file() and p.name!='manifest.json'}))
save(art/'delivery-summary.json',dict(completed=c,attempted=n,blocked=b,source_commit=source,details={r['target_id']:detail(r) for r in rows if not r['complete']}))
print(json.dumps(dict(completed=c,attempted=n,blocked=b,files=len(read(dest/'manifest.json')['sha256']))))
