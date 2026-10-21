"""Package the separately counted six-joint rate diagnostic and its replay."""
from pathlib import Path
import hashlib,html,json,shutil,zipfile
art=Path(__file__).resolve().parent
root=art.parents[1]
dest=root/'docs/experiments/m710_l05_all_rates_20261021'
dest.mkdir(exist_ok=True,parents=True)
read=lambda n:json.loads((art/n).read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert sha(art/'formal-raw.tar.xz')==read('formal-raw-receipt.json')['sha256']
assert read('browser/browser-checks.json')['status']=='PASS'
names=['protocol.json','summary.json','verification.json','predecessor.json','source-check.json',
       'source-manifest.json','formal-run.log','focused-tests.log','formal-v3.sh','worker-rates-bounds.sh',
       'build-bounds.py','build-bounds.log','formal-raw.tar.xz','formal-raw-receipt.json',
       'all-targets.json','costs.json','failure-diagnostics.json','accepted-cycles.html',
       'accepted-cycles-checks.json','summarize-cycles.py','failure-diagnostics.py','pack-evidence.py',
       'build-delivery.py','state-bounds-source.tar.gz','all-rates-source-overlay.tar.gz','all-rates-fixed-source-overlay.tar.gz','run-attempts.json','replay-label-source.tar.gz',
       'rate-exception-evidence.py','rate-exception-evidence.json']
for name in names:shutil.copyfile(art/name,dest/name)
for name in ['browser-checks.json','initial.png','final.png']:shutil.copyfile(art/'browser'/name,dest/name)
shutil.copyfile(root/'tools/search_replay/vendor/LICENSE.three',dest/'LICENSE.three')
shutil.copyfile(root/'assets/robots/fanuc_m710id_70/official/LICENSES/Apache-2.0.txt',dest/'LICENSE.FANUC.txt')
s=read('summary.json');rows=read('all-targets.json')['rows'];diags={r['target_id']:r for r in read('failure-diagnostics.json')['rows']}
n=s['completed_count'];history=s['historical_completed_count'];attempted=len(s['outcomes']);blocked=len(s['blocked'])
assert len(rows)==29 and history==11 and n==sum(r['complete'] for r in rows)
table=[]
for r in sorted(rows,key=lambda r:(-r['layer_index'],r['target_id'])):
    if r['complete']:
        a=r['accepted'];status='完整抓放接受';detail=('?????/???????' if r['target_id']=='carton_l05_c03' else '')+f"{a['face']}，delta={a['delta_m']:.5f} m，接收{a['receive_height_m']:.2f} m，实际{a['cups']['actual_contact_mask']}/72杯"
    elif r.get('supported_cartons'):
        status='承重未搜索';detail='支撑 '+', '.join(r['supported_cartons'])
    else:
        status='已搜索未完成';d=diags[r['target_id']]
        if r['target_id']=='carton_l05_c03':
            detail='六关节速度/加减速度检查例外启用；其余检查仍然拒绝，具体原生见证见本箱诊断。'
        else:
            detail='本箱保持原速度检查；本次预算内未完成，详见逐箱诊断。'
    table.append('<tr><td>'+html.escape(r['target_id'].replace('carton_',''))+'</td><td>'+status+'</td><td>'+html.escape(detail)+'</td></tr>')
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>l05_c03 · 六关节运动限制例外实验</title><style>body{margin:0;background:#142132;color:#edf3fb;font:16px/1.65 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1120px;margin:auto;padding:30px}a{color:#8adbea}td,th{padding:9px;border:1px solid #41556b;text-align:left}table{border-collapse:collapse;width:100%}.links a{display:inline-block;padding:12px;background:#29465d;margin:5px;border-radius:8px}</style><main>'''
page+=f'<h1>单列实验：新增完整抓放 {n}/29</h1><p>继承已核验前序11箱；合计{history+n}/40。原规则整轮结果为18/40，不因本实验而改写。基座[−1.725,0.250,0.600] m。</p>'
page+=f'<p>29个剩余目标中，{attempted}箱实际搜索、{n}箱完整接受、{attempted-n}箱未完成、{blocked}箱承重未搜索。重建新的原生任务根，未输入历史求解轨迹或附着关系。</p>'
page+='<p>仅对l05_c03跳过六关节速度、加速度和减速度判定；IK、关节位置限位及所有碰撞/工艺检查保留。其他箱子使用原worker入口。本实验不代表满足原运动限制的执行方案。</p>'
page+='<p><a href="rate-exception-evidence.json">实际超限值、关节角变化和接受轨迹记录</a>；所有超限原始行保留在worker日志中。</p>'
page+='<div class="links"><a href="accepted-cycles.html">本实验新增周期三维回放</a><a href="failure-diagnostics.json">原生失败诊断</a><a href="run-attempts.json">启动失败与修复记录</a></div>'
page+='<p>回放从29箱和历史11箱状态开始，只展示本实验接受的新周期。名义下落、理想清空均为假设；物理执行NOT_RUN。</p>'
page+='<table><tr><th>目标</th><th>结果</th><th>记录</th></tr>'+''.join(table)+'</table>'
page+='<p><a href="all-targets.json">逐箱数据</a> · <a href="costs.json">搜索成本</a> · <a href="verification.json">原生链核验</a></p><p>实测源码655b49571f4422b2a86a4c2a02c0df830aedf0e8<br>qualification_status=NOT_EVALUATED · continuous_swept_proof=false · physical_execution=NOT_RUN · execution_ready=false</p></main></html>'
(dest/'index.html').write_text(page,encoding='utf-8')
(dest/'README.txt').write_text('打开index.html查看单列的l05_c03六关节运动限制例外实验。回放只包含本实验新增周期，继承11箱不重复计数。完整原始数据见formal-raw.tar.xz。运动限制例外不代表可物理执行；IK、位置限位和碰撞等检查保留。\n',encoding='utf-8')
offline=['index.html','README.txt','accepted-cycles.html','all-targets.json','costs.json','failure-diagnostics.json',
         'run-attempts.json','rate-exception-evidence.json','browser-checks.json','accepted-cycles-checks.json','initial.png','final.png','LICENSE.three','LICENSE.FANUC.txt']
with zipfile.ZipFile(dest/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for name in offline:z.write(dest/name,name)
with zipfile.ZipFile(dest/'offline-replay.zip') as z:assert z.testzip() is None
(dest/'manifest.json').write_text(json.dumps(dict(source_commit=s['source_commit'],sha256={p.name:sha(p) for p in sorted(dest.iterdir()) if p.is_file() and p.name!='manifest.json'}),indent=2)+'\n')
print(json.dumps(dict(new_completed=n,historical_completed=history,attempted=attempted,blocked=blocked)))
