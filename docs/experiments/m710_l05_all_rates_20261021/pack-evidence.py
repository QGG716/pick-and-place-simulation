"""Losslessly collect new-run evidence without expanding it on the GPU disk."""
from pathlib import Path
import hashlib,io,json,lzma,tarfile,time

base=Path('/root/autodl-tmp/m710-mount-20261021')
run=base/'rootfs/l05-all-rates-20261021/formal-v3'
summary=json.loads((run/'summary.json').read_text())
assert summary['status'] in ('COMPLETE_PLANNING_ONLY','INCOMPLETE_RETAINED')
assert (run/'verification.json').is_file()
started=time.monotonic();hashes={};out=base/'l05-all-rates-formal-raw.tar.xz'
assert not out.exists()
with lzma.open(out,'wb',format=lzma.FORMAT_XZ,
    filters=[dict(id=lzma.FILTER_LZMA2,preset=3,dict_size=32*1024*1024)]) as stream:
    with tarfile.open(fileobj=stream,mode='w|') as target:
        def add(name,data):
            info=tarfile.TarInfo(name);info.size=len(data);info.mode=0o644
            target.addfile(info,io.BytesIO(data))
            hashes[name]=hashlib.sha256(data).hexdigest()
        for p in sorted(run.glob('*.json')):add('formal-v3/'+p.name,p.read_bytes())
        for outcome in summary['outcomes']:
            receipt=outcome['evidence'];p=run/receipt['archive']
            assert hashlib.sha256(p.read_bytes()).hexdigest()==receipt['sha256']
            with tarfile.open(p) as source:
                for member in source:
                    if not member.isfile():continue
                    data=source.extractfile(member).read();relative=member.name.split('/',1)[1]
                    assert hashlib.sha256(data).hexdigest()==receipt['file_sha256'][relative]
                    add('formal-v3/'+member.name,data)
            print(json.dumps(dict(archived=p.name,elapsed_s=time.monotonic()-started)),flush=True)
        add('formal-run.log',(base/'rootfs/l05-all-rates-20261021/formal-v3.log').read_bytes())
        add('source-manifest.json',(base/'rootfs/work-l05-rates-bounds-655b495/source-manifest.json').read_bytes())
        add('formal-v3.sh',(base/'rootfs/l05-all-rates-20261021/formal-v3.sh').read_bytes())
        experiment=base/'rootfs/l05-all-rates-20261021'
        for n in ('formal-v3-startup-failure.json','formal-v3-startup-failure.log','formal-v3-startup-tests.log','formal-v3-startup-res-failure.json','formal-v3-startup-res-failure.log','formal-v3-startup-directory-failure.json','formal-v3-startup-directory-failure.log','worker-rates-bounds.sh','build-bounds.py','build-bounds.log','libl05_joint_rates_bounds.so',
                  'joint-rates-bounds-test','joint-rates-bounds-local-test',
                  'formal-v2.log','formal-v2.sh','formal-v2-integration-abort.json','source-check-v2-incomplete.json',
                  'worker-rates-fixed.sh','build-fixed.py','build-fixed.log','libl05_joint_rates_fixed.so',
                  'build.py','build.log','build-fixed-v1.log','libl05_joint_rates_only.so','worker-rates-only.sh',
                  'formal-v1.log','formal-v1.sh','formal-v1-integration-abort.json','source-check-v1-failed.json',
                  'joint-rates-fixed-test','joint-rates-local-test','run-attempts.json',
                  'joint-rates-test','focused-tests.log','source-check.json'):
            add('diagnostic-runtime/'+n,(experiment/n).read_bytes())
        for f in sorted((experiment/'formal-v1').rglob('*')):
            if f.is_file():add('integration-failure-v1/'+str(f.relative_to(experiment)),f.read_bytes())
        for f in sorted((experiment/'formal-v2').rglob('*')):
            if f.is_file():add('integration-incomplete-v2/'+str(f.relative_to(experiment)),f.read_bytes())
        manifest=json.dumps(hashes,sort_keys=True,indent=2).encode()
        info=tarfile.TarInfo('file-sha256.json');info.size=len(manifest)
        target.addfile(info,io.BytesIO(manifest))
checked=0
with tarfile.open(out) as archive:
    for m in archive:
        if m.isfile() and m.name in hashes:
            assert hashlib.sha256(archive.extractfile(m).read()).hexdigest()==hashes[m.name]
            checked+=1
assert checked==len(hashes)
receipt=dict(file=out.name,sha256=hashlib.sha256(out.read_bytes()).hexdigest(),
    bytes=out.stat().st_size,files=checked,wall_s=time.monotonic()-started,
    preservation='All original evidence file bytes preserved; gzip storage containers expanded',
    original_task_archive_receipts_preserved=True)
(base/'l05-all-rates-formal-raw-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt),flush=True)
