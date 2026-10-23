from pathlib import Path
import hashlib,html,json,shutil,tarfile,zipfile
root=Path.cwd();a=root/'artifacts/m710_bottom_two_top_20261023';d=root/'docs/experiments/m710_bottom_two_top_20261023'
d.mkdir(parents=True,exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def copy(src,dest):dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dest)
for f in (a/'download/formal-v1').iterdir():
 if f.is_file():copy(f,d/'records'/f.name)
for f in (a/'download/analysis-v1').iterdir():
 if f.is_file():copy(f,d/f.name)
for f in (a/'download').iterdir():
 if f.is_file():copy(f,d/'records'/f.name)
for f in (a/'qa').iterdir():
 if f.is_file():copy(f,d/'qa'/f.name)
for name in ('source-manifest.json','source-receipt.json','formal-v1-raw.tar.xz'):
 copy(a/name,d/name)
for name in ('prepare-source.py','formal-v1.sh','deploy-and-run.sh','prepare-analysis.py','analyze.py','analyze.sh','export-viewer.sh','build-delivery.py'):
 copy(a/name,d/'reproduce'/name)
copy(root/'tools/export_m710_top_ik_failures.py',d/'reproduce/export-top-failures.py')
s=read(d/'records/summary.json');r=read(d/'all-targets.json');v=read(d/'records/verification.json')
assert s['completed_count']==0 and s['historical_completed_count']==30 and s['target_count']==10
assert v['status'].startswith('PASS_') and v['attempted_count']==5 and v['blocked_count']==5
rawreceipt=read(d/'records/raw-receipt.json')
assert hashlib.sha256((d/'formal-v1-raw.tar.xz').read_bytes()).hexdigest()==rawreceipt['sha256']
with tarfile.open(d/'formal-v1-raw.tar.xz') as t:
 hashes=json.loads(t.extractfile('file-sha256.json').read())
 for name,sha in hashes.items():assert hashlib.sha256(t.extractfile(name).read()).hexdigest()==sha,name
rows=[];costtable=[]
for x in r['rows']:
 target=x['target_id'].removeprefix('carton_')
 if x.get('native_search_performed') is False:
  detail='未派发：仍支撑 '+', '.join(n.removeprefix('carton_') for n in x['supported_cartons'])
  rows.append(f'<tr><td>{target}</td><td>未搜索</td><td>0</td><td>{html.escape(detail)}</td></tr>')
 else:
  n=sum(z['counts']['ik_requests'] for z in x['attempts'])
  rows.append(f'<tr><td>{target}</td><td>{x["wall_s"]:.3f} s</td><td>{n}</td><td>顶吸接近IK未求解；90 s共享前缀预算耗尽。未到contact、+Z顶升或−X抽离。</td></tr>')
  costtable.append(f'{target}: {n}次原生IK请求，任务{x["wall_s"]:.9f} s；1个顶吸业务候选。')
result=dict(source_commit=s['source_commit'],new_completed=0,new_target_count=10,
 attempted_cartons=5,unsearched_support_blocked=5,historical_completed=30,cumulative_completed=30,total_original=40,
 top_route_world=[[0,0,1],[-1,0,0]],receiver_height_m=.25,delta_m=0.,predicted_and_commanded_cups=48,
 actual_contact_cups=None,actual_contact_q2_rad=None,actual_contact_upper_arm_angle=None,
 complete_chain=False,termination='PREGRASP_IK_SHARED_STAGE_BUDGET_EXHAUSTED',
 numerical_ik_success=0,target_collision_check_passed=0,approach_passed=0,contact_passed=0,
 full_pick_place_accepted=0,collision_witnesses=0,counts=r['counts'],
 task_wall_s=r['task_wall_s'],loop_wall_s=r['loop_wall_s'],costs_are_nested_not_additive=True,
 qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
(d/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
readme=f'''本轮完整抓放：0/10；实际派发5箱，另5箱因支撑关系未搜索。累计保持30/40。
只对冻结原层l01、l00开放顶吸；横带全程250 mm，delta=0。
5个已派发候选预测有效杯/命令杯均48；实际contact未到达，因此实际有效杯、contact q2及上臂仰角均未取得。
承载采用ideal_independent_cups：所选有效杯吸力足够，不添加最低杯数门槛。
最后阶段：顶吸pregrasp数值IK；未生成任何机器人运动，没有到达先+Z再−X路线。

实测源码：{s['source_commit']}
继承前序：f17d636ed547a86389150f268445d44bae6f58c4的已验收30箱末态；输入 /lowprofile250-20261023/formal-v1。
终态q、10箱位置、250 mm带高及历史30箱台账逐文件哈希和逐次提交重建验证；清除所有旧原生父链。
未注入历史成功IK、轨迹或attachment。本轮新建原生根；5箱失败均未改变世界。
前序记录与本轮分别存放，历史30箱不是本轮完成数，也不是当前策略从40箱初态测得的独立成功率。

实际结果
{chr(10).join(costtable)}
l00_c00..c04各自仍支撑同列l01箱；支撑关系验证通过，未强行移除或计作IK失败。
总计原生请求2111次，其中IK2086次：2082次NATIVE_IK_FAILED、4次NATIVE_IK_REQUEST_DEADLINE。
这属于5个业务候选的数值求解恢复尝试，不是2086个业务抓法，也不是固定尝试次数上限。
IK成功0；目标碰撞检查通过0（没有进入目标检查）；接近轨迹0；contact及同步0；完整抓放0。
无本轮碰撞见证。原生结果ik_collision_checked=false，不能把旧剪叉/底座对象对当作本轮失败原因。
当前预算内未求得解不等于证明全局不可达。未搜索范围包括l00五箱、接触/抽离/释放后缀，以及预算外的其他数值构型。
本轮用户指定两层顶吸，因此不再派发旧正吸/侧吸业务面。其他层未扩权。

配置和成本
保持机器人安装位置累计−X500 mm、−Y100 mm；纵带0.60 m、横带剪叉中心距0.26 m。
沿用low_profile_250_v1同源实体；只豁免base_link及J1..J6与chassis的7组对象对，工具/载荷/环境/自碰撞原规则保留。
顶吸权限由冻结原层箱顶几何<=0.60 m建立，恰好覆盖这10箱；不按名称解析层号授权。
顶吸路线：l01先+Z 50 mm，l00先+Z 300 mm，再−X；均为尚未到达的业务路线，不是生成轨迹。
seed=71070，单箱预算300 s、共享前缀90 s、IPC100 s；全部关节位置、速度、加速度和减速度规则保留。
原生记录注明seed作用于OMPL；KDL内部RNG不受该seed控制，不虚称跨次耗时/搜索次数逐位确定。
任务循环{r['loop_wall_s']:.9f} s；嵌套单箱任务合计{r['task_wall_s']:.9f} s，两者不能相加。
完整原始记录含各次初值、native响应和时间成本。没有程序异常，也没有失败后换seed重复搜索。

验证与查看
本地23项针对性回归通过；GPU同23项通过（records/focused-tests.log）。
正式PinocchioHppFclBackend初态检查通过，5个worker原生check_start通过。
47份原生源码与ab491d2构建版本一致，复用已校验worker/trace库；记录见native-build-reuse.json与source-check.json。
保存记录验证覆盖40箱身份、剩余支撑、父链、世界不变、统计与失败保留；见records/verification.json。
top-ik-failures.html复用原查看器与官方模型，显示5箱实际起点q和派发TCP，无虚构目标q/运动/碰撞点。
5个查看器用例均通过浏览器检查，渲染FK与生产FK一致，零运动点、零运行错误；见qa/。
没有成功周期，因此没有降带视频冒充抓放回放，也没有生成名义下落或理想清空事件。
名义下落、首次触带、工具分离和IDEAL_RECEIVER_DRAIN均未执行；物理执行NOT_RUN。
导出工具仅修复历史硬编码带高/历史计数并匹配当前底座权限，未修改实测冻结源码；导出器哈希见top-ik-failures-checks.json。

复现
现有GPU rootfs：/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
进入chroot后：bash /bottom-two-top-20261023/formal-v1.sh /bottom-two-top-20261023/reproduction-new
输出目录必须不存在；完整命令见reproduce/formal-v1.sh。需保留已验证前序/lowprofile250-20261023/formal-v1。
固定源码打包：python docs/experiments/m710_bottom_two_top_20261023/reproduce/prepare-source.py
打包脚本复用并哈希验证已有min250资产包，环境与模型不升级。部署脚本对已有目标目录拒绝覆盖。
原始记录：formal-v1-raw.tar.xz，SHA256={rawreceipt['sha256']}，{rawreceipt['files']}份文件逐份校验通过。
source-manifest.json绑定所有实测src/tools/ros2/configs；source-receipt.json记录资产和源码包。
最终交付提交SHA与推送状态见本轮交付消息及git记录；实测源码SHA与最终证据提交分开标记。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
'''
(d/'README.txt').write_text(readme,encoding='utf-8')
page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>l01 / l00 顶吸续跑结果</title>
<style>body{{margin:0;background:#0d1726;color:#dce8f7;font:16px/1.7 system-ui,"Microsoft YaHei"}}main{{max-width:1100px;margin:auto;padding:30px 24px}}h1{{font-size:28px}}a{{color:#75e0ec}}.result{{background:#1a2c42;padding:20px;border-radius:8px}}table{{width:100%;border-collapse:collapse;font-size:14px}}td,th{{padding:9px;text-align:left;border-bottom:1px solid #354960}}code{{overflow-wrap:anywhere}}small{{color:#a7bbd0}}</style>
<main><h1>本轮完整抓放 0/10；累计仍为30/40</h1><div class="result">l01的5箱顶吸接近IK均未求解；l00的5箱仍支撑上层箱，未派发。<br>横带250 mm，delta=0；预测/命令48杯，实际contact未到达。采用理想独立吸盘承载假设。</div>
<p><a href="top-ik-failures.html">打开5箱三维对照：真实起点与请求TCP</a> · <a href="offline-replay.zip">下载离线包</a> · <a href="README.txt">完整说明及复现命令</a></p>
<p>先+Z、再−X的顶吸路线尚未到达。5个业务候选共2086次原生IK调用，成功0；没有目标碰撞见证。保留10箱与原合法末态，不把预算内未求解称为全局不可达。</p>
<table><tr><th>箱体</th><th>任务耗时</th><th>IK请求数</th><th>结果</th></tr>{''.join(rows)}</table>
<p>保存记录一致性验证及浏览器检查通过。任务循环{r['loop_wall_s']:.3f} s，嵌套任务合计{r['task_wall_s']:.3f} s；不可相加。仅实际派发5箱，另5箱属于未搜索的支撑阻挡，不能称为10箱IK失败。</p>
<p>历史30箱单列。本轮数值IK、目标检查、接近、contact、完整抓放均无成功；没有生成机器人运动、名义下落或理想清空，更没有物理执行。</p>
<p>实测源码：<code>{s['source_commit']}</code></p>
<p><a href="all-targets.json">逐箱全部尝试成本</a> · <a href="records/verification.json">保存原生记录验收</a> · <a href="formal-v1-raw.tar.xz">全部原始记录</a> · <a href="source-check.json">源码与二进制校验</a></p>
<small>qualification_status=NOT_EVALUATED · continuous_swept_proof=false<br>physical_execution=NOT_RUN · execution_ready=false</small></main></html>'''
(d/'index.html').write_text(page,encoding='utf-8')
(d/'.gitattributes').write_text('* -text -whitespace\n')
with zipfile.ZipFile(d/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in d.rglob('*'):
  if p.is_file() and p.name not in ('offline-replay.zip','file-manifest.json'):z.write(p,p.relative_to(d))
manifest={str(p.relative_to(d)).replace('\\','/'):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in d.rglob('*') if p.is_file() and p.name!='file-manifest.json'}
(d/'file-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(new_completed=0,cumulative=30,files=len(manifest),raw_files_verified=len(hashes))))
