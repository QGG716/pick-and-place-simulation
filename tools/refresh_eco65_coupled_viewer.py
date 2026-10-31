"""Refresh only the UI of a verified offline replay, preserving its packed records."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
from time import perf_counter
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def refresh(source, output, *, failures=False):
    start = perf_counter()
    source, output = Path(source).resolve(), Path(output).resolve()
    output.relative_to(ROOT / 'outputs')
    destinations = [output, output.with_suffix('.data.json.gz'),
                    output.with_suffix('.json'), output.with_suffix('.zip')]
    if any(p.exists() for p in destinations):
        raise FileExistsError('Use fresh output paths; preserve the original evidence')
    report = json.loads(source.with_suffix('.json').read_text(encoding='utf-8'))
    source_html = source.read_bytes()
    source_sha = hashlib.sha256(source_html).hexdigest()
    if source_sha != report['sha256']:
        raise ValueError('SOURCE_HTML_HASH_MISMATCH')
    packed = source.with_suffix('.data.json.gz').read_bytes()
    payload = gzip.decompress(packed).decode('utf-8')
    # Bind the records to the verified HTML, not merely to an adjacent gzip filename.
    escaped=payload.replace('</', '<\\/')
    formats=('const DATA='+escaped+';window.REPLAY_DATA=DATA;',
             '<script type="application/json" id="eco65-replay-data">'+escaped+'</script>')
    if not any(value.encode() in source_html for value in formats):
        raise ValueError('SOURCE_PACKED_DATA_NOT_IN_VERIFIED_HTML')
    data = json.loads(payload)
    if data['snapshot_fingerprint'] != report['snapshot_fingerprint']:
        raise ValueError('SOURCE_SNAPSHOT_MISMATCH')
    folder = ROOT / 'tools/search_replay'
    template = (folder/'eco65_coupled_viewer.html').read_text(encoding='utf-8')
    html = template.replace('/*__VENDOR__*/', (folder/'vendor/three.min.js').read_text()+'\n'+
                            (folder/'vendor/OrbitControls.js').read_text())
    html = html.replace('/*__DIAGNOSTICS__*/', (folder/'eco65_failure_diagnostics.js').read_text(encoding='utf-8'))
    html = html.replace('/*__DATA__*/', payload.replace('</', '<\\/'))
    if failures:
        # The presentation flag never enters the evidence payload.
        html = html.replace('<html lang="zh">', '<html lang="zh" data-start-in-failures="true">')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding='utf-8')
    shutil.copyfile(source.with_suffix('.data.json.gz'), output.with_suffix('.data.json.gz'))
    report['source_export_wall_s'] = report.pop('export_wall_s', None)
    report.update(html=output.name, sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                  bytes=output.stat().st_size, ui_refresh_only=True,
                  ui_refresh_wall_s=perf_counter()-start,
                  source_html_sha256=source_sha,
                  packed_records_sha256=hashlib.sha256(packed).hexdigest(),
                  source_html=str(source), start_in_failures=failures,
                  failure_requests=sum(a['response'].get('status')!='SUCCESS' for a in data['attempts']))
    output.with_suffix('.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with zipfile.ZipFile(output.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in destinations[:3]:
            archive.write(path, path.name)
        archive.write(folder/'vendor/LICENSE.three', 'LICENSE.three')
        archive.writestr('README.txt', 'Open '+output.name+' locally. All geometry, images, requests, responses and recorded traces are embedded.\nUI-only refresh; the packed evidence is byte-identical to the source. No new planning or hardware execution.\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--failures', action='store_true')
    args = parser.parse_args()
    print(json.dumps(refresh(args.source, args.output, failures=args.failures)))
