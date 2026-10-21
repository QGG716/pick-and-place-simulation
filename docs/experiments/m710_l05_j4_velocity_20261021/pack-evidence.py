"""Losslessly collect new-run evidence without expanding it on the GPU disk."""
from pathlib import Path
import hashlib,io,json,lzma,tarfile,time

base=Path('/root/autodl-tmp/m710-mount-20261021')
run=base/'rootfs/l05-j4-velocity-20261021/formal-v3'
summary=json.loads((run/'summary.json').read_text())
assert summary['status'] in ('COMPLETE_PLANNING_ONLY','INCOMPLETE_RETAINED')
assert (run/'verification.json').is_file()
started=time.monotonic();hashes={};out=base/'l05-j4-velocity-formal-raw.tar.xz'
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
        add('formal-run.log',(base/'rootfs/l05-j4-velocity-20261021/formal-v3.log').read_bytes())
        add('source-manifest.json',(base/'rootfs/work-j4-velocity-b6abb7a/source-manifest.json').read_bytes())
        add('formal-v3.sh',(base/'rootfs/l05-j4-velocity-20261021/formal-v3.sh').read_bytes())
        experiment=base/'rootfs/l05-j4-velocity-20261021'
        for failed_run in ('formal-v1','formal-v2'):
            for p in sorted((experiment/failed_run).rglob('*')):
                if p.is_file():add('startup-failures/'+str(p.relative_to(experiment)),p.read_bytes())
        for n in ('formal-v1.log','formal-v2.log','formal-v1.sh','formal-v2.sh',
                  'worker-j4-only-v1-failed.sh','worker-j4-only.sh','build.py','build.log',
                  'libl05_j4_velocity_only.so','velocity-only-test','focused-tests.log','source-check.json','run-attempts.json'):
            add('diagnostic-runtime/'+n,(experiment/n).read_bytes())
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
(base/'l05-j4-velocity-formal-raw-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt),flush=True)
