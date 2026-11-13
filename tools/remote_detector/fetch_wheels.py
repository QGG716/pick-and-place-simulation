"""Download pinned wheels with curl and verify each against its index SHA-256.

Runs on the GPU server with system python3. torch/torchvision come from the
Aliyun PyTorch mirror and are verified against download.pytorch.org's index
hashes; everything else from the Aliyun PyPI simple index (#sha256 fragments).
Unverified files are deleted. Usage: fetch_wheels.py closure.txt out_dir
"""
import hashlib, html, re, subprocess, sys, urllib.parse
from pathlib import Path
from pip._vendor.packaging.tags import sys_tags
from pip._vendor.packaging.utils import parse_wheel_filename, canonicalize_name

PYPI = 'http://mirrors.aliyun.com/pypi/simple/'
TORCH = 'https://mirrors.aliyun.com/pytorch-wheels/cu124/'
TORCH_INDEX = 'https://download.pytorch.org/whl/cu124/'
TAGS = list(sys_tags())


def fetch(url, timeout=60):
    return subprocess.run(['curl', '-sSfL', '-m', str(timeout), url], check=True, stdout=subprocess.PIPE).stdout.decode()


def links(index_url):
    page = fetch(index_url)
    for href, name in re.findall(r'<a[^>]*href="([^"]+)"[^>]*>([^<]+)</a>', page):
        href = html.unescape(href); url, _, frag = href.partition('#')
        yield name.strip(), urllib.parse.urljoin(index_url, url), frag[7:] if frag.startswith('sha256=') else None


def head_size(url):
    sizes = re.findall(r'(?im)^content-length:\s*(\d+)', subprocess.run(['curl', '-sSIL', '-m', '60', url], check=True,
                       stdout=subprocess.PIPE).stdout.decode())
    if not sizes: raise SystemExit('NO_CONTENT_LENGTH '+url)
    return int(sizes[-1])


def best(candidates):
    ranked = []
    for name, url, digest in candidates:
        if not name.endswith('.whl'): continue
        _, _, _, tags = parse_wheel_filename(name)
        rank = min((TAGS.index(t) for t in tags if t in TAGS), default=None)
        if rank is not None: ranked.append((rank, name, url, digest))
    if not ranked: raise SystemExit('NO_COMPATIBLE_WHEEL: '+str([c[0] for c in candidates]))
    return min(ranked)[1:]


def main():
    closure, out = Path(sys.argv[1]), Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
    for line in closure.read_text().split():
        name, version = line.split('==')
        if name in ('torch', 'torchvision'):
            official = [(n, u, d) for n, u, d in links(TORCH_INDEX+name+'/') if n.startswith(f'{name}-{version}-')]
            filename, _, digest = best(official)
            url = TORCH+urllib.parse.quote(filename)
        else:
            wanted = [(n, u, d) for n, u, d in links(PYPI+canonicalize_name(name)+'/')
                      if n.endswith('.whl') and parse_wheel_filename(n)[1] == parse_wheel_filename(n)[1].__class__(version)]
            filename, url, digest = best(wanted)
        if not digest: raise SystemExit('NO_INDEX_HASH: '+filename)
        target = out/filename
        if not (target.is_file() and hashlib.sha256(target.read_bytes()).hexdigest() == digest):
            # Parallel ranged parts: one HTTP stream is throttled on this host.
            size = head_size(url)
            parts = max(1, min(16, size//(8 << 20)))
            edges = [size*i//parts for i in range(parts+1)]
            procs = [subprocess.Popen(['curl', '-sSfL', '--retry', '5', '-m', '1800', '-r', f'{edges[i]}-{edges[i+1]-1}',
                                       '-o', f'{target}.part{i}', url]) for i in range(parts)]
            if any(p.wait() for p in procs): raise SystemExit('DOWNLOAD_FAILED '+filename)
            with open(target, 'wb') as joined:
                for i in range(parts):
                    piece = Path(f'{target}.part{i}'); joined.write(piece.read_bytes()); piece.unlink()
        actual = hashlib.sha256(target.read_bytes()).hexdigest()
        if actual != digest: target.unlink(); raise SystemExit(f'HASH_MISMATCH {filename} {actual} != {digest}')
        print(f'VERIFIED {filename} {digest} {target.stat().st_size}', flush=True)


if __name__ == '__main__':
    main()
