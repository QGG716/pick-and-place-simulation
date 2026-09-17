"""Archive official ECO65 preparation inputs; never execute vendor code.

Run explicitly with --discover to fetch official page/repository inventories,
then --download after inspecting the inventories. Existing verified files are reused.
"""
import argparse
import concurrent.futures
import datetime as dt
import hashlib
import json
from pathlib import Path
import re
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'assets/robots/realman_eco65/official'
BASE = 'https://develop.realman-robotics.com/'
PAGES = {
    'model': 'download/model/', 'manual': 'download/manual/',
    'parameters': 'robotParameter/ECO65OntologyParameters/',
    'interfaces': 'quickUseManual/interfaceDescriptionArm/',
    'description': 'ros2/description/', 'redevelopment': 'download/redevelopment/',
}
REPOS = {'rm_models': 'main', 'ros2_rm_robot': 'humble', 'Dev_Center': 'main', 'RM_API2': 'main'}


def fetch(url):
    req = urllib.request.Request(url, headers={'User-Agent': 'ECO65-bootstrap-archive'})
    with urllib.request.urlopen(req, timeout=90) as response:
        return response.read()


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def discover():
    if (DEST/'SOURCE_MANIFEST.json').exists():
        raise SystemExit('Pinned manifest already exists; use --restore, not moving branch discovery.')
    for repo, branch in REPOS.items():
        url = f'https://api.github.com/repos/RealManRobot/{repo}/git/trees/{branch}?recursive=1'
        data = fetch(url)
        write(DEST / 'web/inventory' / f'{repo}.json', data)
        tree = json.loads(data)
        assert not tree.get('truncated'), repo
        print(repo, tree['sha'], len(tree['tree']), flush=True)


def select(repo, path):
    low = path.lower()
    if '/' not in path and ('license' in low or 'readme' in low or 'copying' in low):
        return True
    if repo == 'rm_models':
        return path.startswith('ECO65/')
    if repo == 'Dev_Center':
        return '/download/manual/ECO/' in path
    if repo == 'ros2_rm_robot':
        return (path.startswith('rm_description/') and
                ('eco65' in low or path in ['rm_description/CMakeLists.txt', 'rm_description/package.xml']
                 or 'license' in low)) or path.startswith(('rm_driver/', 'rm_msgs/'))
    if repo == 'RM_API2':
        return low.endswith(('.py', '.h', '.md', '.txt', '.json')) and not low.startswith('demo/')
    return False


def local_path(repo, path):
    if repo == 'rm_models' and '/robot_model/' in path:
        return 'cad/rm_models/' + path
    if repo == 'rm_models' and '/dimension/' in path:
        return 'drawings/rm_models/' + path
    if repo == 'Dev_Center':
        return 'manuals/Dev_Center/' + path
    return 'description/' + repo + '/' + path


def inspect(path):
    data = path.read_bytes()
    assert data or path.name == '__init__.py', 'empty file'
    if not data:
        return {'nonempty': False, 'type_check': 'upstream_empty_python_package_marker', 'not_lfs_or_error_page': True}
    assert not data.startswith(b'version https://git-lfs.github.com/spec/v1'), 'Git LFS pointer'
    ext = path.suffix.lower()
    if ext not in ('.html', '.md', '.log'):
        assert b'<html' not in data[:500].lower() and b'<!doctype html' not in data[:500].lower(), 'HTML error page'
    if ext == '.pdf':
        assert data.startswith(b'%PDF-') and b'%%EOF' in data[-2048:], 'invalid PDF envelope'
    if ext in ('.step', '.stp'):
        assert b'ISO-10303-21;' in data[:1024] and b'END-ISO-10303-21;' in data[-1024:], 'invalid STEP envelope'
    if ext in ('.xml', '.urdf', '.xacro', '.launch'):
        ET.fromstring(data)
    if ext == '.stl':
        binary = len(data) >= 84 and 84 + int.from_bytes(data[80:84], 'little') * 50 == len(data)
        assert binary or (data.lstrip().startswith(b'solid') and b'endsolid' in data), 'invalid STL'
    if ext == '.zip':
        import zipfile
        with zipfile.ZipFile(path) as archive:
            assert archive.testzip() is None, 'ZIP CRC failure'
    return {'nonempty': True, 'type_check': 'passed', 'not_lfs_or_error_page': True}


def entry_download(item):
    item = dict(item)
    path = DEST / item['local_path']
    try:
        if not path.exists():
            write(path, fetch(item.get('restore_url', item['download_url'])))
        result = inspect(path)
        data = path.read_bytes()
        if item.get('sha256'):
            assert hashlib.sha256(data).hexdigest() == item['sha256'], 'pinned SHA-256 mismatch'
        if item.get('git_blob_sha'):
            git_sha = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
            assert git_sha == item['git_blob_sha'], 'upstream Git blob hash mismatch'
            result['git_blob_matches'] = True
        item.update(download_status='downloaded', integrity=result, size_bytes=len(data),
                    sha256=hashlib.sha256(data).hexdigest(),
                    downloaded_at=dt.datetime.fromtimestamp(path.stat().st_mtime, dt.timezone.utc).isoformat())
    except Exception as exc:
        item.update(download_status='failed', integrity={'error': str(exc)})
    print(item['download_status'], item['local_path'], flush=True)
    return item


def download():
    if (DEST/'SOURCE_MANIFEST.json').exists():
        raise SystemExit('Pinned manifest already exists; use --restore to preserve provenance.')
    tasks = []
    for gen in ['robot4th', 'robot']:
        for name, suffix in PAGES.items():
            url = BASE + gen + '/' + suffix
            tasks.append(dict(name=f'{gen} {name}', official_entry_url=url, download_url=url,
                local_path=f'web/{gen}/{name}/index.html', applicability=f'{gen} documentation; actual controller unknown',
                upstream_version=None, license='Official web material; local reference, redistribution not assumed'))
    for repo in REPOS:
        inventory = json.loads((DEST / f'web/inventory/{repo}.json').read_text('utf-8'))
        for blob in inventory['tree']:
            p = blob['path']
            if blob['type'] != 'blob' or not select(repo, p):
                continue
            category = 'manual' if repo == 'Dev_Center' else 'model' if repo == 'rm_models' else 'redevelopment'
            tasks.append(dict(name=f'{repo}/{p}', official_entry_url=BASE+('robot/' if p.startswith('RobotGen3/') else 'robot4th/')+PAGES[category],
                download_url=f'https://raw.githubusercontent.com/RealManRobot/{repo}/{inventory["sha"]}/'+urllib.parse.quote(p),
                local_path=local_path(repo, p), git_blob_sha=blob['sha'],
                applicability=f'ECO65 or shared documentation; preserve variant/path: {p}; actual hardware unconfirmed',
                upstream_version=inventory['sha'], license=f'See archived {repo} LICENSE and package declarations; no redistribution assumed'))
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        entries = list(pool.map(entry_download, tasks))
    # Preserve original image files used in official page bodies, including formulas.
    attachments = {}
    for item in entries:
        if item['download_status'] != 'downloaded' or not item['local_path'].endswith('.html'):
            continue
        html = (DEST/item['local_path']).read_text('utf-8')
        for src in re.findall(r'<img[^>]+src=["\']([^"\']+)', html):
            url = urllib.parse.urljoin(item['download_url'], src)
            if urllib.parse.urlparse(url).netloc != 'develop.realman-robotics.com':
                continue
            rel = 'web/attachments/' + urllib.parse.unquote(urllib.parse.urlparse(url).path).lstrip('/')
            attachments[url] = dict(name=src, official_entry_url=item['official_entry_url'], download_url=url,
                local_path=rel, applicability=item['applicability'], upstream_version=None, license='Official web image; local reference')
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        entries.extend(pool.map(entry_download, attachments.values()))
    manifest = dict(schema_version=1, phase='BOOTSTRAP', actual_controller_generation=None,
        actual_robot_variant='ECO65-B', robot_variant_evidence='user_confirmed_2026-09-17', integration_status='not_integrated',
        entries=entries, notes=['Original files, no CAD conversion or URDF changes.',
        'File integrity is not kinematic/dynamic validation. No hardware connection.',
        'Direct files downloaded with pinned Git blob verification; no ZIP extraction performed.',
        'HTML and images preserved; original URLs remain in HTML.'])
    write(DEST/'SOURCE_MANIFEST.json', (json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode())
    print('TOTAL', len(entries), 'FAILED', sum(e['download_status'] != 'downloaded' for e in entries))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--discover', action='store_true')
    p.add_argument('--download', action='store_true')
    p.add_argument('--restore', action='store_true', help='Restore exactly the URLs and hashes already in the manifest, without moving upstream versions')
    args = p.parse_args()
    if sum([args.discover, args.download, args.restore]) != 1:
        p.error('Choose exactly one explicit operation.')
    if args.discover:
        discover()
    if args.download:
        download()

    if args.restore:
        manifest_path = DEST/'SOURCE_MANIFEST.json'
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            manifest['entries'] = list(pool.map(entry_download, manifest['entries']))
        manifest['actual_robot_variant'] = 'ECO65-B'
        manifest['robot_variant_evidence'] = 'user_confirmed_2026-09-17'
        for entry in manifest['entries']:
            if 'RobotGen3/' in entry['local_path']:
                entry['official_entry_url'] = BASE + 'robot/download/manual/'
        failures = sum(e['download_status'] != 'downloaded' for e in manifest['entries'])
        manifest['download_summary']['failures'] = failures
        manifest['download_summary']['status'] = 'incomplete' if failures else 'all_selected_files_downloaded'
        write(manifest_path, (json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode())
        if failures:
            raise SystemExit(f'{failures} archive entries failed; see manifest.')
