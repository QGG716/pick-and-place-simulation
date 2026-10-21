from pathlib import Path
import hashlib,json,tarfile
base=Path('/root/autodl-tmp/m710-mount-20261021')
source=base/'rootfs/work-xy-top-f37448f';out=base/'xy-top-measured-runtime.tar.gz'
assert not out.exists()
manifest={}
with tarfile.open(out,'w:gz',compresslevel=3) as archive:
    for p in sorted(source.rglob('*')):
        if not p.is_file() or '__pycache__' in p.parts or p.suffix=='.pyc':continue
        name=p.relative_to(source).as_posix()
        manifest[name]=hashlib.sha256(p.read_bytes()).hexdigest()
        archive.add(p,arcname=source.name+'/'+name)
with tarfile.open(out) as archive:
    for m in archive:
        if m.isfile():
            assert hashlib.sha256(archive.extractfile(m).read()).hexdigest()==manifest[m.name.split('/',1)[1]]
receipt=dict(file=out.name,sha256=hashlib.sha256(out.read_bytes()).hexdigest(),bytes=out.stat().st_size,
    source_commit='f37448f998d0e82401c096a72c198b28e58fd2c2',file_sha256=manifest)
(base/'xy-top-measured-runtime-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({k:v for k,v in receipt.items() if k!='file_sha256'}))
