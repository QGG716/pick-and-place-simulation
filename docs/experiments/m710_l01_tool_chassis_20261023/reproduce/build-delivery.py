from pathlib import Path
import hashlib,html,json,shutil,tarfile,zipfile
root=Path.cwd();a=root/'artifacts/m710_l01_tool_chassis_20261023';d=root/'docs/experiments/m710_l01_tool_chassis_20261023'
d.mkdir(parents=True,exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def copy(src,dest):
    dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dest)
for f in (a/'download/formal-v1').iterdir():
    if f.is_file():copy(f,d/'records'/f.name)
for f in (a/'download/analysis-v1').iterdir():
    if f.is_file():copy(f,d/f.name)
for f in (a/'download').iterdir():
    if f.is_file():copy(f,d/'records'/f.name)
if (a/'qa').exists():
    for f in (a/'qa').iterdir():
        if f.is_file():copy(f,d/'qa'/f.name)
for name in ('source-manifest.json','source-receipt.json','formal-v1-raw.tar.xz'):
    copy(a/name,d/name)
copy(a/'c04-details.json',d/'c04-details.json')
for name in ('prepare-source.py','formal-v1.sh','deploy-and-run.sh','prepare-analysis.py','analyze.py','analyze.sh','build-delivery.py'):
    copy(a/name,d/'reproduce'/name)
for name in ('source-manifest.json','source-receipt.json','build.log','native-policy-tests.log','deploy-and-run.sh','receipt.json'):
    copy(a/'integration-v1'/name,d/'integration-v1'/name)
s=read(d/'records/summary.json');r=read(d/'all-targets.json');v=read(d/'records/verification.json');p=read(d/'records/protocol.json')
assert s['historical_completed_count']==30 and s['target_count']==5 and v['attempted_count']==5
assert v['status'].startswith('PASS_') and v['blocked_count']==0
assert p['scope']=='XY_TOP_LOWPROFILE250_L01_TOOL_CHASSIS'
assert len(p['out_of_scope_target_ids'])==5
rawreceipt=read(d/'records/raw-receipt.json')
assert hashlib.sha256((d/'formal-v1-raw.tar.xz').read_bytes()).hexdigest()==rawreceipt['sha256']
with tarfile.open(d/'formal-v1-raw.tar.xz') as t:
    hashes=json.loads(t.extractfile('file-sha256.json').read())
    for name,sha in hashes.items():assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==sha,name
descriptions=read(a/'descriptions.json')
rows=[];lines=[]
for x in r['rows']:
    name=x['target_id'];short=name.removeprefix('carton_');status='完整抓放接受' if x['complete'] else '未完成'
    detail=descriptions[name]
    rows.append(f'<tr><td>{short}</td><td>{status}</td><td>{html.escape(detail)}</td><td>{x["wall_s"]:.3f} s</td></tr>')
    lines.append(f'{short}：{status}；{detail}；单箱任务{x["wall_s"]:.9f} s。')
completed=s['completed_count'];remaining=len(s['remaining_target_ids']);cumulative=30+completed
accepted=[dict(target_id=x['target_id'],**x['accepted']) for x in r['rows'] if x['complete']]
result=dict(source_commit=s['source_commit'],new_completed=completed,new_targets=5,
    historical_completed=30,cumulative_completed=cumulative,total_original=40,remaining=remaining,
    out_of_scope=p['out_of_scope_target_ids'],accepted=accepted,counts=r['counts'],
    descriptions=descriptions,task_wall_s=r['task_wall_s'],loop_wall_s=r['loop_wall_s'],
    qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
(d/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
accepted_text='\n'.join(f"{x['target_id']}：delta={x['delta_m']*1000:.0f} mm；接收高度={x['height_m']*1000:.0f} mm；同步位置={x['receive_sync_location']}；预测/命令/实际均48杯；q2={x['q2_rad']:.6f} rad；上臂仰角={x['upper_arm']['elevation_deg']:.3f}°。" for x in accepted)
readme=f'''本轮 l01 完整抓放：{completed}/5；历史30箱单列，累计{cumulative}/40，现场保留{remaining}箱。
本轮采用原正吸/侧吸业务规则，增加用户明确批准的“仅l01工具与chassis接触”仿真规划豁免。
{accepted_text}
承载采用 ideal_independent_cups：所选有效杯吸力足够，不增加最低承载杯数门槛。

{chr(10).join(lines)}

实测源码：{s['source_commit']}
继承前序：f17d636ed547a86389150f268445d44bae6f58c4 的已验收30箱末态，输入 /lowprofile250-20261023/formal-v1。
初态10箱、250 mm带高、q、历史台账逐记录重建；不注入旧成功IK、轨迹或attachment；清空旧原生父链后建立本轮根。
这不是从40箱初态独立运行得到的当前策略成功率。l00五箱在场景中保留，本轮不派发，也不计入5箱分母。

权限和不变规则
工具权限由冻结原始40箱几何选定箱底0.30 m、箱顶0.60 m的一层；恰好5箱。未通过名字解析层号。
只允许当前目标属于这一层时，模型拥有的工具碰撞体与精确chassis对象接触；原生worker绑定工具所有权和策略指纹。
保留已有base_link及J1..J6对chassis豁免；不扩大到载荷、邻箱、墙、输送带、剪叉或其他自碰撞对象。
CPU固定环境缓存区分目标权限；原生豁免仅接受planning_only=true和cold_from_scratch。
全部IK、关节位置/速度/加速度/减速度、实际contact杯几何、父链和抓放工艺规则保留。
保持安装累计−X500 mm、−Y100 mm，纵带0.60 m、横带剪叉中心距0.26 m，low_profile_250_v1最低带高250 mm。
本轮没有继续使用上次顶吸专试；箱顶严格低于最低带高才自动采用顶吸，因此本轮l01仍走正吸/侧吸规则。
seed=71070，单箱300 s，共享前缀90 s，IPC100 s；无新增固定IK/RRT尝试上限。
原生记录注明seed作用于OMPL；KDL内部随机性并不由该seed完全控制，不宣称跨次搜索次数逐位确定。

证据与成本
全部结果区分数值IK、目标检查、接近、contact和完整抓放，详见all-targets.json及records/verification.json。
本轮138次原生请求，9次数值IK中6次成功；5次显式目标检查通过；3箱接近及contact通过；2箱完整接受。
真正进入IK/运动搜索的业务候选只有3个；另1个worker初始化后因侧面遮挡拒绝，不把它计作运动搜索候选。
c04抽离阶段91次请求均失败；728条J2—左壁见证来自19个不同q，不是728个独立候选。
c04接近时也有工具—横带的8条失败分支见证，但后来接近已通过；这些不是抽离失败的原因。
INTERSECTION_OR_CONTACT的负值是碰撞哨兵，不是穿透深度；未生成碰撞接触点，不把代理距离称作CAD穿透证明。
c02的PREGRASP外轴检查从0.25到0.24999999999999994 m，仅浮点端点差，没有实际升降；c03已在接收高度。
回放中的4秒机构演示时长不代表实测硬件升降时间。
完整接受要求同一箱经过实际contact末态附着、抽离、区域验收、同位姿解除附着、名义下落、名义首次触带、工具分离、IDEAL_RECEIVER_DRAIN。
名义下落时机器人保持释放q；理想清空是假设，不是物理传送、实测接收或Isaac执行。
任务循环{r['loop_wall_s']:.9f} s，嵌套单箱任务合计{r['task_wall_s']:.9f} s，不能相加。
全部请求成本、候选初值、目标构型、成功轨迹和失败见证在原始记录中；静态预览未计入规划候选。
未搜索部分包括本轮范围外的l00五箱及当前预算之外的构型/路线；预算内失败不等于全局不可达。

测试夹具错误单列
57e0957 的原生回归把测试对象对写成JSON对象，触发 PROCESS_ALLOWED_PAIR_INVALID，尚未开始正式规划。
e9dfef3 仅修正测试数组编码并重新构建；失败日志和源码清单保留在integration-v1/，不是5箱规划失败。
本地18项针对性回归通过；GPU针对性Python与原生reference/optimized检查见records/。
正式PinocchioHppFclBackend初态检查和原生链路已实际运行。source-check.json逐文件核对本次运行源码未变。

复现和交付
GPU rootfs：/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
进入chroot：bash /l01-tool-chassis-v2-20261023/formal-v1.sh /l01-tool-chassis-v2-20261023/reproduction-new
输出目录必须不存在；完整参数见reproduce/formal-v1.sh。需保留已验前序 /lowprofile250-20261023/formal-v1。
本轮worker和trace库独立构建在 /l01-tool-v2-build 与 /l01-tool-v2-install，未升级环境/依赖。
accepted-cycles.html为本轮接受的原生q和机构运动回放；未派发或失败目标不伪造成功轨迹。
原始记录formal-v1-raw.tar.xz：SHA256={rawreceipt['sha256']}，逐份校验{rawreceipt['files']}份文件。
source-manifest.json、source-receipt.json绑定所有实测src/tools/ros2/configs和模型资产。
最终证据提交SHA、推送状态见本轮交付消息和git历史；实测源码与证据提交明确区分。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
'''
(d/'README.txt').write_text(readme,encoding='utf-8')
page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>l01 工具—chassis豁免重跑</title>
<style>body{{margin:0;background:#101b2a;color:#e0eaf7;font:16px/1.7 system-ui,"Microsoft YaHei"}}main{{max-width:1100px;margin:auto;padding:30px 24px}}a{{color:#72dce5}}h1{{font-size:28px}}.result{{background:#20334b;padding:20px;border-radius:8px}}table{{width:100%;border-collapse:collapse}}th,td{{padding:10px;text-align:left;border-bottom:1px solid #39516b}}code{{overflow-wrap:anywhere}}small{{color:#b2c2d5}}</style>
<main><h1>l01 本轮完整抓放 {completed}/5；累计 {cumulative}/40</h1>
<div class="result">仅 l01 的工具—chassis 接触按用户授权放开；其他检查继续生效。<br>历史30箱单列，l00五箱保留且本轮不派发。现场剩余{remaining}箱。</div>
<p><a href="accepted-cycles.html">打开本轮完整抓放三维回放</a> · <a href="offline-replay.zip">离线包</a> · <a href="README.txt">完整说明和复现</a> · <a href="all-targets.json">逐箱原始统计</a></p>
<table><tr><th>箱体</th><th>结果</th><th>实际原因或完整路线</th><th>任务耗时</th></tr>{''.join(rows)}</table>
<p>{html.escape(accepted_text)}</p><p>吸盘承载采用理想独立杯假设。名义下落、名义触带与理想清空明确标注；物理执行 NOT_RUN。</p>
<p>任务循环 {r['loop_wall_s']:.3f} s，嵌套单箱合计 {r['task_wall_s']:.3f} s，不能相加。初版原生测试夹具错误已单独保留，正式规划绑定修复后源码。</p>
<p>实测源码：<code>{s['source_commit']}</code></p>
<small>qualification_status=NOT_EVALUATED · continuous_swept_proof=false · physical_execution=NOT_RUN · execution_ready=false</small></main></html>'''
(d/'index.html').write_text(page,encoding='utf-8')
with zipfile.ZipFile(d/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=7) as z:
    for path in sorted(d.rglob('*')):
        if path.is_file() and path.name not in ('offline-replay.zip','formal-v1-raw.tar.xz'):
            z.write(path,path.relative_to(d))
print(json.dumps(dict(completed=completed,targets=5,cumulative=cumulative,remaining=remaining,raw_files=rawreceipt['files'])))
