"""Package the measured continuous population and its checked offline replay."""
from pathlib import Path
import hashlib, html, json, shutil, zipfile

root=Path(__file__).resolve().parents[2]
art=Path(__file__).resolve().parent
dest=root/'docs/experiments/m710_population_x300_20261021'
dest.mkdir(parents=True,exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
save=lambda p,v:p.write_text(json.dumps(v,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
receipt=read(art/'formal-raw-receipt.json')
assert sha(art/'formal-raw.tar.xz')==receipt['sha256']
assert read(art/'browser/browser-checks.json')['status']=='PASS'
for name in ('protocol.json','summary.json','verification.json','source-check.json',
             'source-manifest.json','formal-run.log','focused-tests.log','formal-v1.sh',
             'formal-raw.tar.xz','formal-raw-receipt.json','all-targets.json','costs.json',
             'accepted-cycles.html','accepted-cycles-checks.json','summarize-cycles.py',
             'pack-evidence.py','build-delivery.py','failure-diagnostics.py','failure-diagnostics.json'):
    shutil.copyfile(art/name,dest/name)
for name in ('browser-checks.json','initial.png','final.png'):
    shutil.copyfile(art/'browser'/name,dest/name)
shutil.copyfile(root/'artifacts/population-x300-source-cb6eb01.tar.gz',dest/'measured-source-overlay.tar.gz')
shutil.copyfile(root/'tools/search_replay/vendor/LICENSE.three',dest/'LICENSE.three')
shutil.copyfile(root/'assets/robots/fanuc_m710id_70/official/LICENSES/Apache-2.0.txt',dest/'LICENSE.FANUC.txt')
rows=read(dest/'all-targets.json')['rows'];summary=read(dest/'summary.json')
diagnostics={r['target_id']:r for r in read(dest/'failure-diagnostics.json')['rows']}
complete=sum(r['complete'] for r in rows);attempted=len(summary['outcomes']);blocked=len(summary['blocked'])
assert len(rows)==40 and complete==summary['completed_count']
table=[]
for r in sorted(rows,key=lambda r:(-r['layer_index'],r['target_id'])):
    if r['complete']:
        a=r['accepted'];status='完整抓放接受'
        detail=f"{a['face']} · δ={a['delta_m']:.5f} m · 接收 {a['receive_height_m']:.2f} m · {a['cups']['actual_contact_mask']}/72 杯 · {r['wall_s']:.3f} s"
    elif r.get('supported_cartons'):
        status='仍承重，未搜索';detail='支撑 '+', '.join(n.replace('carton_','') for n in r['supported_cartons'])
    else:
        status='规划未完成'
        d=diagnostics[r['target_id']]
        pairs=sorted({tuple(w['witness']['pair']) for a in d['attempts'] for w in a['collision_witnesses']})
        if any(a['region_failure_counts'].get('TOOL_DEPARTURE') for a in d['attempts']):
            detail='最远到释放后工具分离；该步骤原生 IK 失败，接收区域搜索最终耗尽预算。'
        elif pairs:
            detail='顶吸未完成；本轮拒绝见证：'+'；'.join(' / '.join(p) for p in pairs)
        elif not r['counts'].get('numerical_ik_success',0):
            detail='本次预算内未得到数值 IK 解；无本轮碰撞见证，不能据此证明全局不可达。'
        else:
            detail='当前预算内未找到合格构型；无本轮碰撞见证，不能据此证明全局不可达。'
    table.append('<tr><td>'+html.escape(r['target_id'].replace('carton_',''))+'</td><td>'+status+'</td><td>'+html.escape(detail)+'</td></tr>')
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>当前布局 · 40箱连续抓放结果</title><style>
body{margin:0;background:#142132;color:#edf3fb;font:16px/1.65 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1120px;margin:auto;padding:30px}a{color:#8adbea}table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:9px;border:1px solid #41556b;text-align:left;overflow-wrap:anywhere}th{background:#23394e}.links a{display:inline-block;padding:12px 20px;background:#29465d;margin:4px;border-radius:8px}small{color:#b5c7db}</style><main>'''
page+=f'<h1>完整抓放 {complete}/40 · {complete/40:.1%}</h1>'
page+='<p>当前机器人基座 [−1.625, 0.250, 0.600] m；相对原布局 −X300 mm、−Y100 mm。仅低层采用顶吸并固定横带于最低 0.60 m，中高层沿用既有同步规则。</p>'
page+=f'<p>一个新世界从40箱、空历史台账开始；{attempted}箱派发原生搜索，{complete}箱完整接受，{attempted-complete}箱规划未完成，{blocked}箱仍承重而未搜索。现场40→{40-complete}箱。</p>'
page+='<div class="links"><a href="accepted-cycles.html">本轮连续三维回放</a><a href="all-targets.json">逐箱记录</a><a href="costs.json">全部尝试成本</a></div>'
page+='<p>回放使用本轮原生关节轨迹及同源机构几何。每箱实际contact重验吸盘后附着，完整接收、同位姿解除附着、名义下落、工具分离和显式理想清空后才计为完成。所选有效吸盘假定吸力足够，全部72个物理杯保留。</p>'
page+='<p>名义下落和 IDEAL_RECEIVER_DRAIN 是理想假设；实测物理接收0，物理执行 NOT_RUN。失败候选不作为连续世界的已提交运动。</p>'
page+='<table><thead><tr><th>箱体</th><th>结果</th><th>实际记录</th></tr></thead><tbody>'+''.join(table)+'</tbody></table>'
page+='<p><a href="verification.json">原生父链、完整周期和逐箱计数核验</a> · <a href="failure-diagnostics.json">实际失败及未搜索范围</a> · <a href="formal-raw.tar.xz">原始记录</a></p>'
page+='<small>实测源码 cb6eb0199fb07f10bff632edf3aaa1234f679c00<br>qualification_status=NOT_EVALUATED · continuous_swept_proof=false<br>physical_execution=NOT_RUN · execution_ready=false</small></main></html>'
(dest/'index.html').write_text(page,encoding='utf-8')
(dest/'README.txt').write_text('打开 index.html 查看本轮40箱结果及连续离线三维回放。\nformal-raw.tar.xz 保留全部原始文件字节及SHA256；原逐任务gzip存储容器已展开。\nmeasured-source-overlay.tar.gz 与上一轮 measured-runtime.tar.gz 组合可重建本次实测源码和模型。\n本轮仅规划，名义接收与理想清空不代表物理执行。\n',encoding='utf-8')
names=['index.html','accepted-cycles.html','README.txt','all-targets.json','costs.json','failure-diagnostics.json',
       'browser-checks.json','accepted-cycles-checks.json','initial.png','final.png','LICENSE.three','LICENSE.FANUC.txt']
with zipfile.ZipFile(dest/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for n in names:z.write(dest/n,n)
with zipfile.ZipFile(dest/'offline-replay.zip') as z:
    assert z.testzip() is None
    assert all(z.read(n)==(dest/n).read_bytes() for n in names)
save(dest/'manifest.json',dict(source_commit='cb6eb0199fb07f10bff632edf3aaa1234f679c00',
    sha256={p.name:sha(p) for p in sorted(dest.iterdir()) if p.is_file() and p.name!='manifest.json'}))
print(json.dumps(dict(completed=complete,attempted=attempted,blocked=blocked,
    files=len(read(dest/'manifest.json')['sha256']),bytes=sum(p.stat().st_size for p in dest.iterdir() if p.is_file()))))
