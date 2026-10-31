"""Re-render a recorded offline replay with one carton selected; no planning."""
import argparse
import hashlib
import html
import json
from pathlib import Path
import zipfile


def export(source, output, target):
    root=Path(__file__).resolve().parents[1]
    output=output.resolve();output.relative_to(root/'outputs')
    if output.exists():raise FileExistsError('Use a fresh output path')
    original=source.read_text(encoding='utf8')
    marker='<script type="application/json" id="eco65-replay-data">'
    payload=(original.split(marker,1)[1].split('</script>',1)[0] if marker in original
             else original.split('const DATA=',1)[1].split(';window.REPLAY_DATA=DATA;',1)[0])
    data=json.loads(payload)
    def resolve(value):
        while isinstance(value,dict) and 'shared_definition_ref' in value:
            value=data['shared_definitions'][value['shared_definition_ref']]
        return value
    indices=[i for i,a in enumerate(data['attempts']) if resolve(a['request'].get('process_policy',{})).get('target_id')==target]
    if not indices:raise ValueError('TARGET_HAS_NO_CAPTURED_REQUESTS')
    template=(root/'tools/search_replay/eco65_coupled_viewer.html').read_text(encoding='utf8')
    vendor=root/'tools/search_replay/vendor'
    page=template.replace('<html lang="zh">','<html lang="zh" data-focus-target="'+html.escape(target,quote=True)+'">')
    page=page.replace('/*__VENDOR__*/',(vendor/'three.min.js').read_text()+'\n'+(vendor/'OrbitControls.js').read_text())
    page=page.replace('/*__DIAGNOSTICS__*/',(root/'tools/search_replay/eco65_failure_diagnostics.js').read_text(encoding='utf8'))
    page=page.replace('/*__DATA__*/',payload)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(page,encoding='utf8')
    record=dict(schema='eco65_recorded_target_inspection_v1',target_id=target,request_indices=indices,
        source_html=str(source.resolve()),source_html_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        embedded_data_sha256=hashlib.sha256(payload.encode()).hexdigest(),native_data_changed=False,
        source_snapshot_fingerprint=data['snapshot_fingerprint'],
        html=output.name,html_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        planning_run='NOT_RUN_EXISTING_EVIDENCE_ONLY',physical_execution='NOT_RUN',execution_ready=False)
    output.with_suffix('.json').write_text(json.dumps(record,indent=2),encoding='utf8')
    with zipfile.ZipFile(output.with_suffix('.zip'),'w',zipfile.ZIP_DEFLATED) as z:
        z.write(output,output.name);z.write(output.with_suffix('.json'),'inspection.json')
        z.write(vendor/'LICENSE.three','LICENSE.three')
    print(json.dumps(dict(target_id=target,requests=len(indices),html=str(output),native_data_changed=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--target',required=True);a=p.parse_args();export(a.source,a.output,a.target)
