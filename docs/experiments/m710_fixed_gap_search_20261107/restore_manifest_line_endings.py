"""Verify exact source bytes; optionally restore only hash-proven LF/CRLF changes."""
import argparse
import hashlib
import json
from pathlib import Path


def verify(root, manifest, restore=False):
    root=Path(root).resolve();repairs=[];errors=[]
    for name,expected in manifest['files'].items():
        path=(root/name).resolve()
        if not path.is_relative_to(root) or not path.is_file():
            errors.append(name+': MISSING_OR_OUTSIDE_ROOT');continue
        raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()==expected:continue
        lf=raw.replace(b'\r\n',b'\n')
        candidates=() if b'\0' in raw else (lf,lf.replace(b'\n',b'\r\n'))
        matched=next((value for value in candidates if hashlib.sha256(value).hexdigest()==expected),None)
        if matched is None:errors.append(name+': CONTENT_MISMATCH')
        else:repairs.append((name,path,raw,matched))
    if errors:raise ValueError('\n'.join(errors))
    if repairs and not restore:raise ValueError('LINE_ENDINGS_DIFFER: '+', '.join(n for n,*_ in repairs))
    # Validate the entire source first. Never "repair" semantic differences.
    for name,path,previous,matched in repairs:
        if path.read_bytes()!=previous:raise ValueError(name+': SOURCE_CHANGED_DURING_CHECK')
        path.write_bytes(matched)
    return dict(status='PASS_EXACT_MANIFEST_BYTES',files=len(manifest['files']),
        restored_line_endings=[n for n,*_ in repairs])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest',type=Path);p.add_argument('--root',type=Path,default=Path.cwd())
    p.add_argument('--restore-line-endings',action='store_true');a=p.parse_args()
    print(json.dumps(verify(a.root,json.loads(a.manifest.read_text()),a.restore_line_endings)))
