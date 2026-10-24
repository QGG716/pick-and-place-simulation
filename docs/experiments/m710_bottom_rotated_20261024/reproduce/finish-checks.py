from pathlib import Path
import hashlib,json,re,zipfile,shutil
out=Path('docs/experiments/m710_bottom_rotated_20261024')
side=out/'comparison/analysis-side/accepted-cycle.html'
original=side.read_text(encoding='utf-8');marker='<!-- INDEPENDENT_COMPARISON_LABEL -->'
if marker not in original:
 changed=original+'''\n<!-- INDEPENDENT_COMPARISON_LABEL -->
<script>document.title='l01_c04 独立侧吸对比 · 成功';document.querySelector('h1').textContent='l01_c04 独立侧吸对比 · 完整抓放 1/1';</script>\n'''
 decoder=json.JSONDecoder()
 assert decoder.raw_decode(original.rsplit('const D=',1)[1])[0]==decoder.raw_decode(changed.rsplit('const D=',1)[1])[0]
 side.write_text(changed,encoding='utf-8',newline='\n')
 (out/'comparison/analysis-side/presentation-label-check.json').write_text(json.dumps(dict(status='PASS_DATA_UNCHANGED',
  before_sha256=hashlib.sha256(original.encode()).hexdigest(),after_sha256=hashlib.sha256(changed.encode()).hexdigest(),
  change='Independent comparison title only; native q, source and result unchanged'),indent=2)+'\n')
for path in (out/'index.html',out/'analysis-v1/accepted-cycles.html',side,out/'comparison/analysis-front/failure.html'):
 text=path.read_text(encoding='utf-8');assert '\ufffd' not in text
 assert '/*__DATA__*/' not in text and '/*__VENDOR__*/' not in text
 assert not re.search(r'<script[^>]*src=["\']https?://',text)
 for href in re.findall(r'href=["\']([^"\']+)',text):
  if href.endswith('offline-replay.zip') or href.startswith(('http','#')):continue
  assert (path.parent/href).exists(),href
(out/'browser-checks.json').write_text(json.dumps(dict(status='PASS_VISIBLE_UI_AND_SOURCE_CHECKS',
 main=dict(initial_history=36,initial_remaining=4,at_15_s=dict(target='carton_l00_c03',phase='top_lift',attached=True,commanded_cups=72),
  final=dict(completed=3,remaining=1,nominal_reception=3,ideal_drain=3,measured_reception=0)),
 front=dict(visible_numeric_ik=True,visible_approach=True,visible_contact=True,extraction_passed=False,actual_cups=48,
  displayed_witnesses=['4.660 mm J2/left wall gap','intersection/contact'],static_rejected_search_configuration=True),
 side=dict(initial_history=37,initial_remaining=3,independent=True),
 browser_runtime_fk_assertion=False,structural_exporter_native_q_checks=True,physical_execution='NOT_RUN'),indent=2)+'\n')
(out/'.gitattributes').write_text('*.json -text\n*.log -text\n*.html -text\n*.sh -text\n*.py -text\n*.exit -text\n*.txt -text\n')
shutil.copyfile(Path(__file__),out/'reproduce/finish-checks.py')
files={str(p.relative_to(out)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in out.rglob('*') if p.is_file() and p.name not in ('delivery-files.json','offline-replay.zip')}
(out/'delivery-files.json').write_text(json.dumps(files,indent=2)+'\n')
archive=out/'offline-replay.zip'
with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in out.rglob('*'):
  if p.is_file() and p!=archive:z.write(p,p.relative_to(out))
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
print(json.dumps(dict(offline_files=len(files),offline_bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())))
