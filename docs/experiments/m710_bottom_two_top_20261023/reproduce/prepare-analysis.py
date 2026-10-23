from pathlib import Path
out=Path('artifacts/m710_bottom_two_top_20261023')
old=Path('artifacts/m710_lowprofile250_20261023/analyze-v2.py').read_text(encoding='utf-8')
script=old.split('# Lossless expanded archive:')[0]
script=script.replace('/work-lowprofile-f17d636','/work-bottom-top-4337f91').replace('/lowprofile250-20261023','/bottom-two-top-20261023').replace("base/'analysis-v2'","base/'analysis-v1'")
script=script.replace('historical_completed=28,cumulative_completed=28+summary',"historical_completed=summary['historical_completed_count'],cumulative_completed=summary['historical_completed_count']+summary")
script+='''
# Preserve every source record, including all rejected numerical requests.
files={};rawout=base/'formal-v1-raw.tar.xz'
with tarfile.open(rawout,'w:xz',preset=3) as dest:
 def add(name,b):
  if name in files:raise ValueError('DUPLICATE_EVIDENCE_FILE')
  files[name]=hashlib.sha256(b).hexdigest();m=tarfile.TarInfo(name);m.size=len(b);m.mode=0o644;dest.addfile(m,io.BytesIO(b))
 for f in sorted(run.iterdir()):
  if f.is_file() and f.name.endswith('.tar.gz'):
   with tarfile.open(f) as source:
    for m in source:
     if m.isfile():add('formal-v1/'+m.name,source.extractfile(m).read())
  elif f.is_file():add('formal-v1/'+f.name,f.read_bytes())
 for name in ('formal-v1.log','formal-v1.exit','focused-tests.log','production-check.log','formal-v1.sh','native-build-reuse.json'):
  add(name,(base/name).read_bytes())
 add('production-check/initial-state-check.json',(base/'production-check/initial-state-check.json').read_bytes())
 raw=json.dumps(files,indent=2).encode();m=tarfile.TarInfo('file-sha256.json');m.size=len(raw);dest.addfile(m,io.BytesIO(raw))
save(base/'raw-receipt.json',dict(file=rawout.name,sha256=hashlib.sha256(rawout.read_bytes()).hexdigest(),bytes=rawout.stat().st_size,files=len(files)))
'''
(out/'analyze.py').write_text(script,encoding='utf-8',newline='\n')
(out/'analyze.sh').write_text('''#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /bin/bash -s <<'INSIDE'
set -euo pipefail
export PYTHONPATH=/work-bottom-top-4337f91/src:/work-bottom-top-4337f91/tools PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-bottom-top-4337f91 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cd /bottom-two-top-20261023
test "$(cat formal-v1.exit)" = 0
/opt/planning-venv/bin/python /work-bottom-top-4337f91/tools/verify_m710_population.py formal-v1 --prepare-replay replay-v1
/opt/planning-venv/bin/python analyze.py
INSIDE
tar czf /root/autodl-tmp/m710-l05-x500-20261022/bottom-top-analysis.tar.gz -C "$rootfs/bottom-two-top-20261023" analysis-v1 formal-v1/summary.json formal-v1/protocol.json formal-v1/verification.json formal-v1/initial-state-check.json formal-v1/predecessor.json focused-tests.log production-check.log formal-v1.log formal-v1.exit raw-receipt.json native-build-reuse.json
''',encoding='utf-8',newline='\n')
