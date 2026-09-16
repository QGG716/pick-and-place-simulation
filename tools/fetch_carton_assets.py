"""Fetch pinned carton dependencies into a separate cache (run with Isaac Python)."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor
import urllib.request
import zipfile
from urllib.parse import urljoin, urlparse, unquote, quote


def fetch(url, destination, expected=None):
    url = quote(url, safe=':/?&=%')
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.is_file():
        partial = destination.with_name(destination.name + '.partial')
        # Bounded range requests avoid slow long-lived object-store connections.
        with urllib.request.urlopen(urllib.request.Request(url,headers={'Range':'bytes=0-0'}),timeout=30) as response:
            ranged=response.status==206
            length=int(response.headers['Content-Range'].split('/')[-1]) if ranged else 0
            download_url=response.geturl()
        if ranged and length > 1024*1024:
            chunk=256*1024
            def part(start):
                end=min(start+chunk,length)-1
                for attempt in range(3):
                    try:
                        with urllib.request.urlopen(urllib.request.Request(download_url,headers={'Range':f'bytes={start}-{end}'}),timeout=60) as response:
                            if response.status!=206 or response.headers.get('Content-Range')!=f'bytes {start}-{end}/{length}':
                                raise ValueError('Invalid range response')
                            data=response.read()
                        if len(data)!=end-start+1: raise ValueError('Incomplete asset range')
                        return data
                    except Exception:
                        if attempt==2: raise
            with ThreadPoolExecutor(max_workers=8) as executor, partial.open('wb') as stream:
                for data in executor.map(part,range(0,length,chunk)):
                    stream.write(data)
        else:
            subprocess.run(['curl', '--fail', '--location', '--connect-timeout', '20',
                            '--max-time', '180', '--retry', '2', '--silent', '--show-error',
                            '--output', str(partial), url], check=True)
        partial.replace(destination)
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    if expected and digest != expected:
        raise ValueError(f'Asset hash mismatch: {destination}')
    return digest


def mirror_usd(url, cache):
    """Preserve relative USD/texture/MDL paths; fail on missing explicit files."""
    from pxr import Sdf
    cache = cache.resolve()
    pending, records = [url], {}
    while pending:
        current = pending.pop()
        if current in records:
            continue
        parsed = urlparse(current)
        if parsed.scheme != 'https' or parsed.netloc != urlparse(url).netloc:
            raise ValueError(f'Unexpected dependency origin: {current}')
        relative = unquote(parsed.path).lstrip('/')
        path = (cache / relative).resolve()
        if not path.is_relative_to(cache):
            raise ValueError('Dependency escapes asset cache')
        digest = fetch(current, path)
        records[current] = {'url': current, 'path': relative, 'sha256': digest,
                            'bytes': path.stat().st_size}
        dependencies = []
        if path.suffix in ('.usd', '.usda', '.usdc'):
            layer = Sdf.Layer.FindOrOpen(str(path))
            if not layer:
                raise ValueError(f'Invalid USD: {path}')
            dependencies = [v for v in re.findall(r'@([^@\r\n]*)@', layer.ExportToString()) if v]
        elif path.suffix == '.mdl':
            source = path.read_text()
            dependencies = re.findall(r'"([^"\n]+\.(?:png|jpg|jpeg|exr|dds|mdl))"', source)
            # Standard ::df/tex/base/math and ::OmniPBR modules resolve from Isaac.
            dependencies += [m.replace('::', '/') + '.mdl' for m in
                             re.findall(r'import\s+\.::([\w:]+)::\*', source)]
            dependencies += [m.replace('::', '/') + '.mdl' for m in
                             re.findall(r'using\s+\.::([\w:]+)\s+import', source)]
        for dependency in dependencies:
            if dependency in ('OmniPBR.mdl','OmniGlass.mdl'):
                continue  # Runtime MDL modules, verified by Isaac's material resolver.
            if '<UDIM>' in dependency:
                raise ValueError('UDIM dependencies require an explicit pinned tile list')
            pending.append(urljoin(current, dependency))
    create_encoded_directory_aliases(cache)
    return sorted(records.values(), key=lambda x: x['path'])


def create_encoded_directory_aliases(cache):
    """USD file resolvers retain %20 in some upstream relative directory names.

    Keep original pinned source bytes; Linux cache aliases resolve both spellings.
    """
    for directory in list(Path(cache).rglob('*')):
        if not directory.is_dir() or directory.is_symlink() or ' ' not in directory.name:
            continue
        alias = directory.with_name(quote(directory.name))
        if not alias.exists():
            alias.symlink_to(directory.name, target_is_directory=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--url')
    parser.add_argument('--lock', type=Path)
    parser.add_argument('--config', type=Path, help='Reproduce the reviewed pinned cache without USD inspection')
    args = parser.parse_args()
    if args.config:
        config=json.loads(args.config.read_text())
        cache=args.cache.resolve()
        archives={}
        for archive in config.get('archives',[]):
            path=(cache/archive['path']).resolve()
            if not path.is_relative_to(cache): raise ValueError('Archive path escapes cache')
            fetch(archive['url'],path,archive['sha256'])
            archives[archive['id']]=path
        for dependency in config['dependencies']:
            target=(cache/dependency['path']).resolve()
            if not target.is_relative_to(cache): raise ValueError('Dependency path escapes cache')
            if 'archive' in dependency:
                with zipfile.ZipFile(archives[dependency['archive']]) as archive:
                    data=archive.read(dependency['member'])
                if hashlib.sha256(data).hexdigest()!=dependency['sha256']:
                    raise ValueError('Archive member hash mismatch')
                target.parent.mkdir(parents=True,exist_ok=True)
                if target.exists() and target.read_bytes()!=data:
                    raise ValueError('Existing asset differs; refusing to overwrite')
                target.write_bytes(data)
            else:
                fetch(dependency['url'],target,dependency['sha256'])
        create_encoded_directory_aliases(cache)
        print(f"Verified {len(config['dependencies'])} pinned dependencies")
        return
    if not args.url or not args.lock:
        parser.error('provide --config, or both --url and --lock')
    records = mirror_usd(args.url, args.cache)
    if args.lock.exists():
        expected = json.loads(args.lock.read_text())
        if records != expected['files']:
            raise ValueError('Source dependencies differ from pinned lock')
    else:
        args.lock.parent.mkdir(parents=True, exist_ok=True)
        args.lock.write_text(json.dumps({'entry_url': args.url, 'files': records}, indent=2)+'\n')
    print(f'Verified {len(records)} files')


if __name__ == '__main__':
    main()
