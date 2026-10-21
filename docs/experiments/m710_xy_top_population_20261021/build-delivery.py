"""Package the measured run and checked viewers; no planning is performed."""
from pathlib import Path
import hashlib,html,json,shutil,tarfile,zipfile

root=Path(__file__).resolve().parents[2];art=Path(__file__).resolve().parent
dest=root/'docs/experiments/m710_xy_top_population_20261021';dest.mkdir(parents=True,exist_ok=True)
read=lambda p:json.loads(p.read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
save=lambda p,v:p.write_text(json.dumps(v,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
for name in ('formal-raw','measured-runtime'):
    receipt=read(art/f'xy-top-{name}-receipt.json')
    assert sha(art/receipt['file'])==receipt['sha256']
assert read(art/'formal-browser/browser-checks.json')['status']=='PASS'
assert read(art/'failed-goals-browser-final/browser-checks.json')['status']=='PASS'
files={
    'protocol.json':'protocol.json','summary.json':'summary.json','verification.json':'verification.json',
    'delivery-verification.json':'delivery-verification.json','source-manifest.json':'measured-source-manifest.json',
    'delivery-source-manifest.json':'delivery-source-manifest.json','source-check.json':'source-check.json',
    'xy-top-formal-v1.log':'formal-run.log','xy-top-formal-raw.tar.xz':'formal-raw.tar.xz',
    'xy-top-formal-raw-receipt.json':'formal-raw-receipt.json',
    'xy-top-measured-runtime.tar.gz':'measured-runtime.tar.gz',
    'xy-top-measured-runtime-receipt.json':'measured-runtime-receipt.json',
    'xy-top-accepted-cycles.html':'accepted-cycles.html','xy-top-accepted-cycles-checks.json':'replay-checks.json',
    'formal-browser/browser-checks.json':'browser-checks.json','formal-browser/initial.png':'initial.png',
    'formal-browser/final.png':'final.png','xy-top-failed-goals.html':'failed-goals.html',
    'xy-top-failed-goals-checks.json':'failed-goals-checks.json',
    'failed-goals-browser-final/browser-checks.json':'failed-goals-browser-checks.json',
    'failed-goals-browser-final/failed-goal-0.png':'failed-l03-c01.png',
    'failed-goals-browser-final/failed-goal-1.png':'failed-l03-c04.png',
    'xy-top-analysis/all-targets.json':'all-targets.json','xy-top-analysis/costs.json':'costs.json',
    'formal-v1.sh':'formal-v1.sh','summarize-cycles.py':'summarize-cycles.py',
    'export-failed-goals.py':'export-failed-goals.py','check-failed-goals.mjs':'check-failed-goals.mjs',
    'pack-evidence.py':'pack-evidence.py','pack-runtime.py':'pack-runtime.py',
    'build-delivery.py':'build-delivery.py'}
for source,name in files.items():shutil.copyfile(art/source,dest/name)
development=[p for p in art.glob('*.log') if p.name!='xy-top-formal-v1.log']
development += [art/n for n in ('development-source.tar.gz','development-v2-source.tar.gz',
    'development-v2-source-reconstruction.json','development-checks.sh')]
development += [p for p in (art/'xy-top-development-integration-v2').rglob('*') if p.is_file()]
development += [art/'development-browser/browser-checks.json']
with tarfile.open(dest/'development-records.tar.gz','w:gz') as archive:
    for p in development:archive.add(p,arcname=p.relative_to(art).as_posix())
save(dest/'development-history.json',dict(
    production_fixture='Synthetic exposed l03_c02, one complete cycle; excluded from formal counts',
    development_source_v2='Reconstructed from retained v1 export and the exact three uploaded changed files',
    formal_planning_integration_errors=0,
    retained_events=[
      'v1: directional_contact returned None for top normal; explicit in-plane axis added',
      'unit fixtures: missing submitted_request ownership_transition; corrected fixture provenance',
      'two lifted_reception assertions fail also on baseline cf40c1d; retained without geometry changes',
      'read-only verifier initially read terminal_check.passed instead of status and free_space_rules_restored',
      'delivery-only verification initially used host-absolute symlinks inaccessible inside chroot; relative links passed',
      'local Python had no pytest; focused tests used existing pinned production venv; dependencies not upgraded']))
for source,name in ((root/'tools/search_replay/vendor/LICENSE.three','LICENSE.three'),
    (root/'assets/robots/fanuc_m710id_70/official/LICENSES/Apache-2.0.txt','LICENSE.FANUC.txt')):
    shutil.copyfile(source,dest/name)
rows=read(dest/'all-targets.json')['rows'];summary=read(dest/'summary.json')
assert len(rows)==40 and sum(r['complete'] for r in rows)==17 and len(summary['blocked'])==18
reasons={
 'carton_l05_c03':'接触已通过；抽离区域 346 个端点后预算耗尽。无本次碰撞见证。',
 'carton_l04_c02':'接触已通过；抽离区域 346 个端点后预算耗尽。无本次碰撞见证。',
 'carton_l03_c01':'顶吸目标：刚性工具 / l04_c02 碰撞。contact 未到达。',
 'carton_l03_c04':'顶吸目标：刚性工具 / l04_c03 碰撞。contact 未到达。',
 'carton_l03_c00':'本次未得到数值 IK 解；没有本次碰撞见证。'}
table=[]
for r in sorted(rows,key=lambda r:(-r['layer_index'],r['target_id'])):
    if r['complete']:
        a=r['accepted'];status='完整成功'
        detail=f"{a['face']} · δ={a['delta_m']:.5f} m · 接收 {a['receive_height_m']:.2f} m · {a['cups']['actual_contact_mask']}/72 杯"
    elif r.get('supported_cartons'):
        status='支撑阻塞，未搜索';detail='仍支撑 '+', '.join(n.replace('carton_','') for n in r['supported_cartons'])
    else:status='原生规划未完成';detail=reasons[r['target_id']]
    table.append('<tr><td>'+html.escape(r['target_id'].replace('carton_',''))+'</td><td>'+status+'</td><td>'+html.escape(detail)+'</td></tr>')
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>XY −100 mm 与低层顶吸 · 全批结果</title><style>
body{margin:0;background:#142132;color:#edf3fb;font:16px/1.65 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1100px;margin:auto;padding:30px}a{color:#8adbea}table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:9px;border:1px solid #41556b;text-align:left}th{background:#23394e}.links a{display:inline-block;padding:12px 20px;background:#29465d;margin:4px;border-radius:8px}code{overflow-wrap:anywhere;font-size:12px}small{color:#b5c7db}</style>
<main><h1>完整抓放 17/40 · 42.5%</h1>
<p>机器人安装位置 −X、−Y 各 100 mm。中高层按原策略升降；仅低层顶吸固定横带于最低 0.60 m。</p>
<p>22 箱进入原生搜索：17 箱完成、5 箱未完成。18 箱仍承重，未派发搜索。低层派发 3 箱，完成 0 箱，不能把另外 17 个未搜索低层箱称为 IK 不可达。现场 40→23 箱。</p>
<p>旧轮同一组 36 个箱号：本轮 13/36（36.1%），旧轮 8/36（22.2%）。保留旧成功 7 箱、新增 6 箱；旧成功 l03_c04 改为顶吸后未完成。前序末态和策略不同，不是单因素对照。</p>
<div class="links"><a href="accepted-cycles.html">17 条接受周期三维回放</a><a href="failed-goals.html">两组顶吸碰撞目标</a></div>
<p>回放由本轮原生 q 驱动；失败目标查看器只显示实际被拒绝的 q，不生成合法运动。低层候选降带没有提交，因此连续回放最后已接受带高为 0.85 m。</p>
<p>名义下落与 IDEAL_RECEIVER_DRAIN 为理想假设，物理执行 NOT_RUN。所选有效吸盘假定吸力足够，全部 72 个物理杯保留。</p>
<table><thead><tr><th>箱体</th><th>结果</th><th>实际证据</th></tr></thead><tbody>'''+''.join(table)+'''</tbody></table>
<p><a href="all-targets.json">逐箱及实际接触构型 JSON</a> · <a href="costs.json">全部尝试成本</a></p>
<small>实测源码 f37448f998d0e82401c096a72c198b28e58fd2c2<br>qualification_status=NOT_EVALUATED · continuous_swept_proof=false<br>physical_execution=NOT_RUN · execution_ready=false</small></main></html>'''
(dest/'index.html').write_text(page,encoding='utf-8')
(dest/'README.txt').write_text('打开 index.html 查看本轮结果与两个离线三维查看器。\n'
    'formal-raw.tar.xz 保留原始文件字节及 SHA256，逐箱 gzip 存储容器已展开。\n'
    '使用交付版 tools/verify_m710_population.py 可直接核验展开的 formal-v1 目录。\n'
    'measured-runtime.tar.gz 为实测源码、配置、模型资产和原始 CAD 的实际运行快照。\n'
    'development-records.tar.gz 是开发失败及独立人工暴露低层场景，禁止混入正式 17/40。\n',encoding='utf-8')
viewer_names=['index.html','accepted-cycles.html','failed-goals.html','README.txt','all-targets.json','costs.json',
 'browser-checks.json','replay-checks.json','failed-goals-checks.json','failed-goals-browser-checks.json',
 'initial.png','final.png','failed-l03-c01.png','failed-l03-c04.png','LICENSE.three','LICENSE.FANUC.txt']
with zipfile.ZipFile(dest/'offline-replay.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for n in viewer_names:z.write(dest/n,n)
with zipfile.ZipFile(dest/'offline-replay.zip') as z:
    assert z.testzip() is None
    assert all(z.read(n)==(dest/n).read_bytes() for n in viewer_names)
save(dest/'manifest.json',dict(**read(dest/'source-check.json'),sha256={p.name:sha(p) for p in sorted(dest.iterdir()) if p.is_file() and p.name!='manifest.json'}))
print(json.dumps(dict(files=len(read(dest/'manifest.json')['sha256']),bytes=sum(p.stat().st_size for p in dest.iterdir() if p.is_file()),completed=17,targets=40)))
