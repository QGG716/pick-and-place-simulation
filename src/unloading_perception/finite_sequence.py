"""Small finite capture manifest and provenance checks; no ROS or model runtime."""
import hashlib
import json
import os
from pathlib import Path
import tempfile

from unloading_contracts import canonical_fingerprint
from .algorithm_artifact import load_algorithm_artifact
from .fusion import ModuleFaceBatch, fuse_module_face_batches
from .isaac_payload import load_capture_payload
from .isaac_validation import IsaacSceneManifest


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def atomic_json(path, value):
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.'+path.name, delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def manifest_groups(path):
    path = Path(path).resolve()
    document = read_json(path)
    groups = document.get('groups')
    if (document.get('schema_version') != 'finite_capture_sequence_v1'
            or document.get('sequence_kind') != 'INDEPENDENT_CAPTURE_GROUPS'
            or not isinstance(groups, list) or not 1 <= len(groups) <= 16):
        raise ValueError('finite manifest requires 1..16 independent capture groups')
    ids = set()
    result = []
    for group in groups:
        identity = group['task_id']
        if (not isinstance(identity, str) or not identity or identity in ids
                or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in identity)):
            raise ValueError('task IDs must be unique safe directory names')
        ids.add(identity)
        if group['action'] not in ('RUN', 'REUSE_VERIFIED'):
            raise ValueError('action must explicitly select RUN or REUSE_VERIFIED')
        item = dict(group)
        for field in ('capture', 'reuse_summary'):
            if field in item:
                item[field] = str((path.parent/Path(item[field])).resolve())
        if not item.get('capture') or (item['action']=='REUSE_VERIFIED' and not item.get('reuse_summary')):
            raise ValueError('capture/reuse summary is required')
        result.append(item)
    return result


def verify_capture(capture):
    """Validate every consumed original buffer, releasing arrays module by module."""
    capture = Path(capture)
    manifest = IsaacSceneManifest.from_dict(read_json(capture/'manifest.json'))
    modules = [c['module_id'] for c in manifest.cameras]
    if len(modules)!=2 or len(set(modules))!=2:
        raise ValueError('finite sequence requires two distinct RGB-D modules')
    provenance, batches = {}, []
    for module in modules:
        payload = load_capture_payload(capture/'FULL_STACK_NOMINAL/modules'/module,
            manifest, expected_module_id=module, with_instance_masks=True)
        provenance[module] = payload.input_provenance()
        m = payload.metadata
        batches.append(ModuleFaceBatch(module,m.sensor_epoch,m.frame_sequence,m.capture_center_time,()))
        del payload
    fuse_module_face_batches(batches,expected_modules=modules)
    return provenance


def input_identity(provenance):
    return {key: provenance[key] for key in ('capture_id','module_id','sensor_epoch','frame_sequence','original_binding')} | {
        'consumed_files': {name: ref['sha256'] for name,ref in provenance['consumed_files'].items()}}


def verify_result(summary_path, inputs, models, config):
    """Reuse and new runs obey the same input/model/config/complete-result checks."""
    summary = read_json(summary_path)
    if not summary.get('finalized') or summary.get('exit_code') != 0 or summary.get('overall_status') != 'COMPLETED':
        raise ValueError('single-capture run did not complete technically')
    reference = summary['algorithm_artifact']  # Trusted expected value from original run, never repaired.
    observation,index = load_algorithm_artifact(reference['path'],reference['sha256'])
    if (index['run_id']!=summary['run_id'] or canonical_fingerprint(models)!=index['model_manifest_identity']
            or canonical_fingerprint(config)!=index['config_identity']
            or canonical_fingerprint(index['model_manifest'])!=canonical_fingerprint(models)):
        raise ValueError('algorithm run/model/config identity mismatch')
    coverage = observation.coverage['module_coverage']
    if set(coverage)!=set(inputs):
        raise ValueError('algorithm module roster differs from selected capture')
    for module, current in inputs.items():
        if canonical_fingerprint(input_identity(current)) != canonical_fingerprint(input_identity(coverage[module]['input_provenance'])):
            raise ValueError('algorithm input differs from selected original capture: '+module)
    counts = {'objects':len(observation.cargo),
        'raw_surfaces':sum(len(c.observed_surfaces) for c in observation.cargo),
        'representatives':sum(len(c.raw_result.get('fusion_diagnostics',{}).get('face_reduction',{}).get('representatives',())) for c in observation.cargo),
        'conflicts':sum(len(c.raw_result.get('fusion_diagnostics',{}).get('face_reduction',{}).get('conflicts',())) for c in observation.cargo),
        'unknown_regions':len(observation.unknown_regions)}
    return summary,reference,counts


def verify_sam_files(models):
    root = Path(models['sam']['snapshot_path']).resolve()
    files = models['sam']['files']
    if not {'model.safetensors','config.json','preprocessor_config.json'}.issubset(files):
        raise ValueError('incomplete pinned SAM model files')
    for name,expected in files.items():
        path = root/name
        # HF snapshot entries legitimately resolve to sibling content-addressed blobs.
        if Path(name).name != name:
            raise ValueError('model file name must be a basename')
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''): digest.update(block)
        if digest.hexdigest()!=expected:
            raise ValueError('SAM model file hash mismatch: '+name)
