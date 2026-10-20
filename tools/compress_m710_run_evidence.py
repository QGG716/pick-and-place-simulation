"""Lossless storage of large completed-run JSON, with byte-for-byte SHA proof.

Run verification/export before compression. To restore, gunzip the listed
files; the manifest records the SHA256 and byte count of every original.
"""
import argparse,gzip,hashlib,json,shutil
from pathlib import Path


def sha(path,compressed=False):
    h=hashlib.sha256();size=0
    with (gzip.open(path,"rb") if compressed else path.open("rb")) as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b""):
            h.update(chunk);size+=len(chunk)
    return h.hexdigest(),size


def compress(root):
    root=root.resolve()
    if not (root/"protocol.json").is_file() or not (root/"verification.json").is_file():
        raise ValueError("ONLY_VERIFIED_COMPLETED_RUN_STORAGE")
    manifest=root/"lossless-compression-manifest.json"
    rows=json.loads(manifest.read_text()) if manifest.exists() else []
    for path in sorted(root.rglob("*.json")):
        if not path.resolve().is_relative_to(root) or path==manifest or path.stat().st_size<1024*1024:continue
        target=path.with_suffix(".json.gz")
        if target.exists():raise FileExistsError(target)
        original=sha(path)
        with path.open("rb") as source,target.open("wb") as destination:
            with gzip.GzipFile(filename="",mode="wb",fileobj=destination,compresslevel=6,mtime=0) as z:
                shutil.copyfileobj(source,z,1024*1024)
        if sha(target,True)!=original:raise ValueError("LOSSLESS_VERIFY_FAILED")
        rows.append(dict(file=str(path.relative_to(root)),gzip=str(target.relative_to(root)),
            sha256=original[0],bytes=original[1],compressed_bytes=target.stat().st_size))
        manifest.write_text(json.dumps(rows,indent=2))
        path.unlink()  # only this hash-verified file inside the explicit run root
    print(json.dumps(dict(files=len(rows),original_bytes=sum(r["bytes"] for r in rows),
        compressed_bytes=sum(r["compressed_bytes"] for r in rows))))

if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("run",type=Path);compress(p.parse_args().run)
