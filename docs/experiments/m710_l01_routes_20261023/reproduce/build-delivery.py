from pathlib import Path
import hashlib,html,json,shutil,tarfile,zipfile

root=Path.cwd();base=root/'artifacts/m710_l01_routes_20261023';download=base/'downloaded'
out=root/'docs/experiments/m710_l01_routes_20261023'
for d in (out,out/'records',out/'reproduce',out/'qa',out/'preparation-not-run'):d.mkdir(parents=True,exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
write=lambda p,d:p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
raw=base/'formal-v1-raw.tar.xz';receipt=read(download/'raw-receipt.json')
assert hashlib.sha256(raw.read_bytes()).hexdigest()==receipt['sha256']
with tarfile.open(raw) as t:
    manifest=json.load(t.extractfile('file-sha256.json'))
    for n,sha in manifest.items():assert hashlib.sha256(t.extractfile(n).read()).hexdigest()==sha,n
assert len(manifest)==receipt['files']==40
for p in (download/'analysis-v1').iterdir():shutil.copy2(p,out/p.name)
for p in (download/'formal-v1').iterdir():shutil.copy2(p,out/'records'/p.name)
for p in download.iterdir():
    if p.is_file():shutil.copy2(p,out/'records'/p.name)
for n in ('source-manifest.json','source-receipt.json'):shutil.copy2(base/n,out/n)
shutil.copy2(raw,out/raw.name)
for n in ('prepare-source.py','formal-v1.sh','deploy-and-run.sh','analyze.py','analyze.sh','check-viewer.mjs','build-delivery.py'):
    shutil.copy2(base/n,out/'reproduce'/n)
for p in (base/'preparation-not-run').iterdir():shutil.copy2(p,out/'preparation-not-run'/p.name)
write(out/'preparation-not-run/status.json',dict(status='PREPARED_ONLY_NEVER_DEPLOYED_OR_RUN',reason='User corrected dispatch order before formal run',
    source_commit='8058c17ca721646ec5bed1cdd22805a2633eeeba',replaced_by='6558d70c2c068ad7e5223b13555efcc97e34e1cf'))
for n in ('browser-checks.json','l01_c01.png','l01_c00.png','l01_c04.png'):shutil.copy2(base/'viewer-qa'/n,out/'qa'/n)
report=read(out/'all-targets.json');verification=read(out/'records/verification.json')
viewer=json.JSONDecoder().raw_decode((out/'three-attempts.html').read_text(encoding='utf-8').split('const DATA=',1)[1])[0]
assert report['source_commit']==verification['source_commit']==viewer['raw']['source_commit']=='6558d70c2c068ad7e5223b13555efcc97e34e1cf'
assert read(out/'qa/browser-checks.json')['status']=='PASS'
reasons=[
    '+Y侧吸面被l01_c04遮挡，面可见性筛选拒绝；没有派发IK或轨迹规划。',
    '+Y侧吸面被l01_c01、l01_c04遮挡，面可见性筛选拒绝；没有派发IK或轨迹规划。',
    '顶吸接近阶段421次原生IK均未求解，90秒共享前缀预算耗尽；未到目标碰撞检查、接近轨迹、contact或+Z→−X抽离。']
rows=[]
for r,c,reason in zip(report['rows'],viewer['inspection_cases'],reasons):
    assert r['target_id']==c['target_id']
    rows.append(dict(target_id=r['target_id'],complete=False,reason=reason,task_wall_s=r['wall_s'],
        template=c['option']['template'],directions=c['option']['segment_directions_world'],
        delta_m=0.,height_m=c['height_m'],actual_height_changed=False,synchronization='NOT_REACHED_ALREADY_AT_HMAX',
        ik_requests=c['ik_calls'],numerical_ik_success=0,target_check_passed=0,approach_passed=0,contact_passed=0,complete_pick_place=0,
        predicted_cups=c.get('predicted_cups'),commanded_cups=c['commanded_cups'],actual_contact_cups=None,
        actual_start_q_rad=c['display_q'],start_q2_rad=c['q2_rad'],start_upper_arm=c['upper_arm'],selected_goal_q=None,
        blocking_cartons=c['blockers'],collision_witnesses=[]))
result=dict(source_commit=report['source_commit'],new_completed=0,new_targets=3,dispatch_order=[r['target_id'] for r in rows],
    historical_completed=32,cumulative_historical_completed=32,remaining=8,rows=rows,
    shared_prefix_budget_s=90,task_budget_s=300,seed=71070,seed_scope='OMPL_ONLY_KDL_INTERNAL_RNG_NOT_CONTROLLED',
    counts=dict(native_requests=434,ik_requests=421,numerical_ik_success=0,target_check_passed=0,approach_passed=0,contact_passed=0,complete_pick_place=0),
    loop_wall_s=report['loop_wall_s'],nested_task_wall_s=report['task_wall_s'],native_ik_solver_s=read(out/'three-attempts-checks.json')['native_solver_s'],
    business_targets_dispatched=3,actual_ik_or_motion_search_candidates=1,no_new_robot_motion=True,collision_witnesses=0,
    holding_assumption='ideal_independent_cups',reception_or_drain_executed=False,
    qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
write(out/'result.json',result)
arm=rows[0]['start_upper_arm'];lines='\n'.join(f"{r['target_id'].replace('carton_','')}：{r['reason']} 单箱用时{r['task_wall_s']:.9f} s。" for r in rows)
report_text=f'''本轮完整抓放：0/3；按c01 → c00 → c04实际派发。历史32箱单列，累计仍32/40，现场保留8箱。
指定路线：c01、c00为+Y侧吸，先+Y再−X；c04为顶吸，先+Z再−X。三箱横带均固定原始Hmax≈250 mm，delta=0。
实际没有到达contact或附着；c04预测有效/命令48杯，实际contact杯数未测；c01、c00未到吸杯计算。
承载保持ideal_independent_cups（所选有效杯吸力足够），没有新增承载杯数门槛。

{lines}

这轮改变抓面与路线后，三箱均未到抽离；不能引用上轮c04正吸抽离时的J2—左壁碰撞作为本轮原因。
c01、c00属于保留的面可见性筛选失败，不是数值IK失败，也不是已经验证的路径碰撞。
c01的+Y面到c04近面相距约860 mm，但默认射线遮挡规则仍将c04列为遮挡；未启用有限工具深度的侧面筛选。
c00的+Y面距c01近面约20 mm，且c01本轮未移除。未因失败而换序或在c04后重新派发c00/c01。
这不是三条路线都经过了IK搜索：三次业务任务/worker初始化，只有c04这一个业务候选进入IK。
c04的421次IK属于同一个固定上游接近目标的数值初值恢复；不是421个业务抓法，也不是421条RRT轨迹。
failure.attempts=422还包含预算耗尽前的最后一次循环检查；真正派发的IK请求数为421，以requests.json为准。
KDL返回NATIVE_IK_FAILED、error_code=-6；ik_collision_checked=false，因此没有证据将其归因于碰撞。
尚不能由预算内失败判定该目标全局不可达；未到达的阶段包括目标碰撞、接近轨迹、contact、两段抽离、释放与工具分离。
未探索其他抓面、偏移、高度、顺序、预算外构型和l00五箱；本轮没有扩大这些范围。

实测源码：{report['source_commit']}
初始条件：从e9dfef3215b55bb9905ed0671d43ca4b9716a71a已验收32箱末态恢复；前序summary/protocol/verification及任务证据均校验哈希。
本次8箱、历史台账、实际末态q和带高相同；不注入旧成功IK、运动轨迹或附着关系，每箱重新建立原生根。
三箱均未接受任何完整运动，终态与初态相同。实际初始q2={rows[0]['start_q2_rad']:.12f} rad，上臂仰角={arm['elevation_deg']:.9f}°；这不是选中的目标构型。
维持累计安装−X500 mm、−Y100 mm；纵带0.60 m、横带剪叉中心距0.26 m、low_profile_250_v1几何。
横带精确记录值为0.24999999999999994 m，来自冻结原始Hmax；没有新增临时降带、升带或抽离后同步。
c01仍保留冻结原始MIDDLE角色，只增加本次明确批准的侧吸路线；c04顶吸特许仅属于该目标。
保留仅l01工具—chassis及已有机器人—chassis豁免；其他碰撞、IK、位置/速度/加减速、接触与完整抓放验收规则不变。

成本及验证
任务循环{report['loop_wall_s']:.9f} s；嵌套单箱任务合计{report['task_wall_s']:.9f} s；原生IK求解内部用时{result['native_ik_solver_s']:.9f} s，三者不可相加。
434次原生请求：3次init、9次FK、1次初态检查、421次IK；0次运动规划，0次数值IK成功，0次目标检查通过，0次接近/contact/完整接受。
seed=71070；单箱300 s、共享前缀90 s、IPC100 s，未增加固定IK/RRT/高度/偏移次数上限。
原生记录明确seed仅控制OMPL，KDL内部随机性不完全受控；不宣称重跑次数和耗时逐位确定。
14项针对性回归在GPU通过；正式PinocchioHppFclBackend初态贯通检查、正式三目标运行及结果验证通过。
338份实测源码逐份核对未改变；47份native源文件与前序一致，复用已验worker和trace库并核对二进制哈希；未升级依赖。
没有本轮程序异常，formal-v1.exit=0。8058c17仅准备过源包，用户纠正顺序后被6558d70替代，未部署或规划；单列preparation-not-run。
旧AttributeError及其他历史集成失败仍属于之前各轮原记录，本轮没有将其混入三箱统计。

复现
GPU宿主rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
进入chroot后：bash /l01-routes-20261023/formal-v1.sh /l01-routes-20261023/reproduction-new
新输出目录必须不存在；完整参数见reproduce/formal-v1.sh。前序/l01-tool-chassis-v2-20261023/formal-v1必须存在且通过固定哈希检查。
正式source根/work-l01-routes-6558d70；源包保存于本地artifacts/m710_l01_routes_20261023/measured-source.tar.gz与宿主l01-routes-source.tar.gz。
源包重建脚本reproduce/prepare-source.py读取git实测提交与已核验的artifacts/m710_min250_20261023/prepared-source-v3.tar.gz资产包及其receipt；不会升级环境。
原始40份记录保存在formal-v1-raw.tar.xz，SHA256={receipt['sha256']}，内置逐文件哈希。
source-manifest/source-receipt绑定实测源文件及资产；records/native-build-reuse.json及source-check.json绑定实际worker。
只读导出：python tools/export_m710_selected_routes.py RUN OUT.html --source-root /work-l01-routes-6558d70 --overlay tools/search_replay/selected_routes.js
导出工具属于规划后的交付代码，未改写实测源码；exporter/overlay哈希见three-attempts-checks.json。
three-attempts.html显示本次实际原生起点q、8箱环境、面筛选遮挡和c04请求法兰；无新增求解、目标q、轨迹或碰撞点。
三箱浏览器检查通过：q与生产FK一致、环境8箱、0轨迹点、无附着，0脚本错误；qa保存截图。
offline-inspection.zip可离线解压打开index.html；这里没有成功运动可回放，也没有仅降带视频冒充抓放。
本轮没有名义下落、理想接收/清空或物理执行发生；所述路线的后缀保留在实现中，但未到达。
最终交付提交SHA及推送状态见交付消息和git历史，与实测源码SHA分别报告。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
'''
(out/'README.txt').write_text(report_text,encoding='utf-8')
table=''.join(f'<tr><td>{i+1}. {r["target_id"].replace("carton_","")}</td><td>{html.escape(r["reason"])}</td><td>{r["task_wall_s"]:.2f} s</td></tr>' for i,r in enumerate(rows))
page=f'''<!doctype html><meta charset="utf-8"><title>l01 三箱指定路线 · 0/3</title>
<style>body{{font:16px/1.7 system-ui,"Microsoft YaHei";max-width:1050px;margin:36px auto;padding:0 24px;background:#101a2b;color:#e0eaf5}}a{{color:#7be0e8}}table{{border-collapse:collapse;width:100%}}td,th{{padding:12px;text-align:left;border-bottom:1px solid #405065}}pre{{white-space:pre-wrap;font:14px/1.7 inherit}}h1{{font-size:28px}}</style>
<h1>本轮 0/3：C01 → C00 → C04</h1><p>Hmax固定250 mm，delta=0。c01/c00侧吸先+Y再−X；c04顶吸先+Z再−X。历史32箱单列，剩余8箱。</p>
<p><a href="three-attempts.html">打开本次三维失败记录</a> · <a href="README.txt">完整报告与复现命令</a> · <a href="result.json">结构化结果</a> · <a href="formal-v1-raw.tar.xz">全部原始记录</a></p>
<table><tr><th>实际顺序</th><th>本次失败点</th><th>单箱成本</th></tr>{table}</table>
<p>c04预测/命令48杯，未到实际contact。c01/c00未到吸杯计算。保持理想独立吸盘承载假设。</p>
<p>只有c04进入IK搜索。c01/c00是保留的射线遮挡规则拦截，不是实测路径碰撞。三箱无本次碰撞见证，无接受运动。</p>
<p>实测源码：<code>{report['source_commit']}</code>；14项GPU回归、生产后端及记录验证通过。</p>
<pre>{html.escape(report_text)}</pre>'''
(out/'index.html').write_text(page,encoding='utf-8')
(out/'.gitattributes').write_text('* -text whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol\n',encoding='utf-8')
with zipfile.ZipFile(out/'offline-inspection.zip','w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in sorted(out.rglob('*')):
        if p.is_file() and p.name not in ('offline-inspection.zip','.gitattributes'):z.write(p,p.relative_to(out))
write(out/'delivery-checks.json',dict(status='PASS',raw_files_verified=len(manifest),raw_sha256=receipt['sha256'],
    offline_zip_sha256=hashlib.sha256((out/'offline-inspection.zip').read_bytes()).hexdigest(),source_commit=report['source_commit']))
print(json.dumps(dict(folder=str(out),files=sum(p.is_file() for p in out.rglob('*')),raw_files_verified=len(manifest))))
