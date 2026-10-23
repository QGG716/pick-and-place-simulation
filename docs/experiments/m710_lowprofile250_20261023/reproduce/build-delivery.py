from pathlib import Path
import hashlib,html,json,shutil,zipfile
R=Path.cwd();A=R/'artifacts/m710_lowprofile250_20261023';D=R/'docs/experiments/m710_lowprofile250_20261023';D.mkdir(parents=True,exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
def copy(a,b):b.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(a,b)
for f in (A/'download/formal-v1').glob('*'):
 if f.is_file():copy(f,D/'records'/f.name)
for f in (A/'download/analysis-v2').glob('*'):
 if f.is_file():copy(f,D/f.name)
for f in (A/'download').glob('*'):
 if f.is_file():copy(f,D/'records'/f.name)
for name in ('prepare-source.py','formal-v1.sh','deploy-and-run.sh','deploy-repair.sh','analyze.py','analyze.sh','analyze-v2.py','analyze-v2.sh','build-delivery.py'):
 copy(A/name,D/'reproduce'/name)
for name in ('source-receipt.json','source-manifest.json','formal-v1-raw.tar.xz'):
 copy(A/name,D/name)
for name in ('geometry-sweep.json','local-tests.log','local-audit-fix-tests.log'):
 copy(A/name,D/'records'/name)
for name in ('source-manifest.json','source-receipt.json','formal-v1.sh','deploy-and-run.sh'):
 copy(A/'integration-v1'/name,D/'integration-v1'/name)
for f in (A/'qa').glob('*'):
 if f.is_file():copy(f,D/'qa'/f.name)
s=read(D/'records/summary.json');r=read(D/'all-targets.json');v=read(D/'records/verification.json')
assert v['status'].startswith('PASS_') and s['target_count']==12 and s['historical_completed_count']==28
n=s['completed_count'];cumulative=28+n;rows=[]
for target in r['rows']:
 item=dict(target_id=target['target_id'],complete=target['complete'])
 if target.get('native_search_performed') is False:
  item.update(stage='SUPPORT_BLOCKED_NOT_SEARCHED',supported_cartons=target['supported_cartons'],wall_s=0.)
 else:
  item.update(wall_s=target['wall_s'],counts={},attempts=target['attempts'])
  for a in target['attempts']:
   for k,num in a['counts'].items():item['counts'][k]=item['counts'].get(k,0)+num
  if target['complete']:item['accepted']=target['accepted']
 rows.append(item)
result=dict(source_commit=s['source_commit'],new_completed=n,new_target_count=12,
 historical_completed=28,cumulative_completed=cumulative,total_original=40,remaining=40-cumulative,
 rows=rows,counts=r['counts'],loop_wall_s=r['loop_wall_s'],task_wall_s=r['task_wall_s'],
 qualification_status='NOT_EVALUATED',continuous_swept_proof=False,physical_execution='NOT_RUN',execution_ready=False)
(D/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
table=[]
for x in rows:
 if x['complete']:
  a=x['accepted'];detail=f"完整抓放接受；{a['face']}，接收高度 {a['height_m']*1000:.0f} mm，同步 {a['receive_sync_location']}"
 elif 'supported_cartons' in x:detail='未搜索；仍支撑 '+', '.join(x['supported_cartons'])
 else:
  contacted=[a for a in x['attempts'] if a.get('contact_complete')]
  if contacted:
   pairs=sorted({pair for a in contacted for pair in a.get('collision_pairs',{})})
   samples=[a.get('failure',{}).get('samples_attempted') for a in contacted if isinstance(a.get('failure'),dict)]
   detail='接近和contact通过；抽离/接收区域搜索预算耗尽。实际尝试区域端点 '+str(samples)+'；本轮碰撞/净空见证：'+', '.join(pairs)
  else:
   if x['target_id']=='carton_l01_c01':detail='所需Hmin = -50 mm，低于最低250 mm；无当前合法接收路线，未派发IK。'
   elif x['target_id']=='carton_l01_c00':detail='正吸所需Hmin = -50 mm；右内侧吸Hmax = 250 mm，但该侧面被保留箱体遮挡，未派发IK。'
   else:detail='；'.join(dict.fromkeys(str(a.get('detail_failure') or a.get('failure') or a.get('screening')) for a in x['attempts']))
 table.append(f'<tr><td>{x["target_id"].removeprefix("carton_")}</td><td>{x["wall_s"]:.2f}</td><td>{html.escape(detail)}</td></tr>')
counts=r['counts'];source=s['source_commit']
readme=f'''250 mm薄型剪叉与安装底座碰撞权限：本轮完成 {n}/12；历史28箱单列，累计 {cumulative}/40，剩余 {40-cumulative} 箱。
成功箱l02_c01、l02_c00：均为正吸，delta=0，实际接收高度250 mm；接触前已在接收高度，ALREADY_AT_RECEIVE。预测/命令/实际contact均48杯，采用理想独立杯吸力足够假设。
两箱都走完降带/接近/contact/实际附着/-X抽离/完整接收区域/同姿态解附着/名义下落/首次触带/工具分离/IDEAL_RECEIVER_DRAIN。
实测源码：{source}。最终交付提交与推送状态见聊天/git记录。

策略与边界
从原先28箱已接受末态重新建立原生任务根，首箱l02_c01。历史末态q、实际550 mm带高和剩余12箱是合法输入；不注入历史成功候选、轨迹或attachment，不复用旧原生父链。
上一轮选层顶吸的0/12失败保留于 ../m710_layer_top_20261023/，没有覆盖；它没有改变上述已接受末态。
取消按l02层开放顶吸。只按冻结原始箱顶 z_bottom+height < 0.250 m 判定；等于250 mm也不允许。现有最低层箱顶300 mm，本轮没有任何箱体获得顶吸权限。
保留原有正面/侧面、Hmin/Hmax规则；允许顶吸时仍是先+Z再-X。未新增接触偏移权限，原始右端授权及中心delta=0保留。
机械臂累计安装偏移-X500 mm、-Y100 mm，世界基座[-1.825,.25,.6]；纵带宽600 mm，横带剪叉中心距260 mm。
新增且仅新增官方物理连杆base_link、J1_link..J6_link与精确世界对象chassis的碰撞接受权限。工具、杯、箱体、其他环境、剪叉及机器人自碰撞仍检查。Python精确校验和原生ACM使用同一策略；Isaac适配器按实际碰撞体归属接受此对象对并保留物理响应，本轮没有运行Isaac。
所有关节位置、速度、加速度、减速度检查保留；seed71070，单箱300 s，90 s共享前缀；未更换规划器或增加固定尝试次数上限。
理想独立杯吸力足够假设保留，无新增最低承载杯数门槛。实际contact重新检查命令杯全环封闭，保留真实偏心附着。

机构调整
新增独立配置 min250_lowprofile，旧min250配置不覆盖。横带两级1.25 m杆长、纵带四级0.60 m杆长保留。
横带：下铰点40 mm，杆截面垂向20 mm、导轨厚20 mm、上铰点距带底36 mm；250 mm带高时上铰点94 mm，单级升高27 mm，脚板实体高度10 mm，脚底仍在原地板。
纵带：下铰点40 mm，杆垂向20 mm、导轨20 mm；后端固定滑轨中心由400降到340 mm，滑座高度由80增至150 mm，使向下倾斜的纵带保持净空且连接连续。
没有删除任何实体、扩大装配接触豁免或降低5 mm规则；CPU、原生世界和回放使用同源几何。
第一次几何草案保留的20 mm横带杆配40 mm上端间距只有4.797 mm最小非配合销/杆间隙，纵带原25 mm杆和55 mm下铰点也不足5 mm；原后滑轨还与下倾纵带相交。随后调整为上述最终尺寸，保留全部检查。
250..2100 mm每10 mm共186个样本，机构与固定环境离散检查全部通过，耗时49.446573 s；这不证明连续扫掠，不替代正式运行中的机器人和实际箱体检查。

结果与成本
新增完整接受{n}/12（{n/12:.1%}）；历史28箱单列，累计{cumulative}/40（{cumulative/40:.1%}）。失败箱及其支撑依赖仍在世界中。
累计值包含以前布局下已接受的28箱，不是把新剪叉策略从40箱初态重新测得的独立成功率。
数值IK成功{counts.get('numerical_ik_success',0)}；显式目标检查通过{counts.get('explicit_target_check_passed',0)}。逐阶段、每个真实候选及全部原生尝试见all-targets.json和records/verification.json，数值IK次数不是业务候选数。
任务环路{r['loop_wall_s']:.6f} s，嵌套单箱任务合计{r['task_wall_s']:.6f} s，二者不相加；原生请求{counts.get('native_requests',0)}，IK请求{counts.get('ik_requests',0)}。
实际进入目标搜索的业务候选{r['actual_target_search_candidates']}，初始化worker的候选{r['worker_initialized_candidates']}；七箱得到终结记录，另五箱因上方支撑依赖未搜索。
l01_c02/c03/c04均已完成接近、实际contact几何48杯和attachment检查，分别尝试64/66/102个接收区域端点；抽离LIN/区域搜索在阶段预算内未完成，不是程序异常或全局不可达结论。
l01_c02/c03的本次见证为tool_rigid_13/11与chassis；l01_c04为tool_rigid_13与chassis以及J2_link与trailer_left_wall。这些对象对都没有获得新增豁免。
l01_c01的Hmin=-50 mm低于250 mm；l01_c00正吸同样越界，250 mm右内侧吸的业务面被保留箱体遮挡。l00_c00..c04分别仍支撑l01_c00..c04，不强行移除。
候选的delta、临时高度、同步、q2、上臂角、杯掩码及碰撞见证保留在verification/原始记录。不能把未搜索目标或旧静态预览算作本次规划结果。
仅完整接收区域、同姿态解除附着、名义下落、首次触带、工具分离、显式IDEAL_RECEIVER_DRAIN全链通过才计数；回放中名义下落和理想清空不是物理执行。

验证与复现
本地35项针对性测试通过，另38项原有剪叉/顶吸/续跑测试通过。GPU测试及新原生策略测试结果见records/日志。初次新测试把StartHeight写成dict导致单个测试失败，已修正测试数据类型，不影响正式运行。
首次生产贯通检查源码ab491d2314e5b08eff6a2f3f84b469bb25a215ae因新approval_basis没有加入布局数值审计映射而中断；未开始正式规划。保留production-check.log和integration-v1源码/清单，补入明确安装位置和600 mm纵带的既有数值后再次检查并完整运行。没有放宽审计阈值，新增了10 mm错误安装必须被拒绝的回归。
新worker在ab491d2编译；修复后的全部原生源码逐文件哈希与它一致，因此复用该二进制，见records/native-build-reuse.json。历史旧worker没有覆盖。
正式入口实际PinocchioHppFclBackend检查保留初态；源码及native worker/trace库哈希见source-check.json、source-receipt.json、records/native-binary-sha256.txt。
固定源码打包：python docs/experiments/m710_lowprofile250_20261023/reproduce/prepare-source.py（复用已保留且哈希核验的min250资产源包；不升级依赖）。
GPU现有rootfs：/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
chroot内复现：bash /lowprofile250-20261023/formal-after-audit-repair.sh /lowprofile250-20261023/reproduction-new
完整参数和新隔离编译过程见reproduce/formal-v1.sh与deploy-and-run.sh。输出目录必须不存在。
输入：/min250-20261023/formal-v2；逐份前序摘要、协议、验证和任务result固定哈希校验，仅恢复接受末态。
formal-v1-raw.tar.xz含所有失败与成功的原始请求、模型、关节轨迹、父链、检查、日志和文件哈希，不混入历史失败。
只读统计脚本初版对缺失的杯掩码求和报错，修为缺失仍标null；统计版本单列analyze-v2。原生验收在统计前已通过，未因此重跑规划或改变轨迹。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
'''
(D/'README.txt').write_text(readme,encoding='utf-8')
replay='<a href="accepted-cycles.html">打开本次真实原生轨迹回放</a> · ' if n else ''
page=f'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>250 mm剪叉与安装底座权限续跑</title><style>body{{background:#0d1726;color:#dce8f7;font:16px/1.7 system-ui,"Microsoft YaHei";margin:0}}main{{max-width:1120px;margin:auto;padding:36px 24px}}h1{{font-size:28px}}a{{color:#70dce9}}.result{{padding:20px;background:#1b2d44;border-radius:8px}}table{{width:100%;border-collapse:collapse;font-size:14px}}td,th{{padding:9px;text-align:left;border-bottom:1px solid #334760;overflow-wrap:anywhere}}td:first-child{{white-space:nowrap}}code{{overflow-wrap:anywhere}}small{{color:#9db2cc}}</style><main>
<h1>本轮完整抓放 {n}/12；累计 {cumulative}/40</h1><div class="result">250 mm薄型剪叉已启用，7组官方机械臂连杆—chassis碰撞权限已进入原生规划。<br>侧吸/正吸；仅原始箱顶严格低于250 mm才允许顶吸。本场景最底箱顶300 mm，因此没有顶吸候选。</div>
<p>成功：l02_c01、l02_c00。正吸delta=0；实际接收高度250 mm，接触前已同步。每箱48杯实际contact有效，采用理想独立杯吸力足够假设；完整释放后缀与理想清空均通过。</p>
<p>{replay}<a href="offline-replay.zip">下载离线包</a> · <a href="README.txt">详细说明与复现命令</a></p>
<table><tr><th>箱体</th><th>任务秒数</th><th>完整接受或本轮停止原因</th></tr>{''.join(table)}</table>
<p>历史28箱单列；本轮{n}/12（{n/12:.1%}），累计{cumulative}/40（{cumulative/40:.1%}）。仅完整释放后缀与理想清空通过计入完成，失败及支撑依赖保留。</p>
<p>机构186个离散高度样本通过；正式任务环路{r['loop_wall_s']:.2f} s。所有关节运动限制和其他碰撞检查保留；名义下落、理想清空与物理执行分开记录。</p>
<p>实测源码：<code>{source}</code>；最终交付提交/推送状态见聊天与git记录。</p>
<p><a href="all-targets.json">逐箱全部尝试成本</a> · <a href="records/verification.json">原生轨迹与父链验收</a> · <a href="formal-v1-raw.tar.xz">全部原始记录</a> · <a href="records/geometry-sweep.json">186个机构高度样本</a></p>
<small>qualification_status=NOT_EVALUATED · continuous_swept_proof=false<br>physical_execution=NOT_RUN · execution_ready=false</small></main></html>'''
(D/'index.html').write_text(page,encoding='utf-8')
(D/'.gitattributes').write_text('* -text -whitespace\n')
with zipfile.ZipFile(D/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in D.rglob('*'):
  if p.is_file() and p.name not in ('offline-replay.zip','file-manifest.json'):z.write(p,p.relative_to(D))
manifest={str(p.relative_to(D)).replace('\\','/'):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in D.rglob('*') if p.is_file() and p.name!='file-manifest.json'}
(D/'file-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(dict(completed=n,targets=12,cumulative=cumulative,files=len(manifest))))
