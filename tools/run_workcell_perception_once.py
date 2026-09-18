"""One existing oracle-proposal SAM/RGB-D run per new module, without tuning."""
import argparse
import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'packages/unloading_contracts/src')]
COMPLETED = 'COMPLETED_WITH_ALGORITHM_RESULTS'


def _write_json(path, payload):
    """Publish a complete UTF-8 document, never a partially written success."""
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                         prefix='.' + path.name + '-', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError as exc:
                print(f'temporary report cleanup failed: {exc}', file=sys.stderr, flush=True)


class SummaryWriteError(RuntimeError):
    """A shared reporting failure must stop further inference."""


def _error(stage, exc):
    return {'stage': stage, 'error_type': type(exc).__name__, 'error': str(exc)}


def _save_summary(output, summary, *, final=False, interrupted_code=None):
    rows = summary['runs']
    summary['module_counts'] = {
        'expected': summary['expected_module_count'],
        'completed': sum(row['status'] == COMPLETED for row in rows),
        'failed': sum(row['status'] == 'TECHNICAL_FAILURE' for row in rows),
        'not_run': sum(row['status'] == 'NOT_RUN' for row in rows),
        'running': sum(row['status'] == 'RUNNING' for row in rows),
    }
    success = (final and bool(rows) and len(rows) == summary['expected_module_count']
               and all(row['status'] == COMPLETED for row in rows) and not summary['errors'])
    summary['finalized'] = final
    summary['overall_status'] = ('INTERRUPTED' if interrupted_code is not None else
                                 'COMPLETED' if success else 'TECHNICAL_FAILURE' if
                                 final or summary['errors'] or summary['module_counts']['failed'] else 'RUNNING')
    # Progress reports are conservatively nonzero until final publication succeeds.
    summary['exit_code'] = interrupted_code if interrupted_code is not None else 0 if success else 1
    try:
        _write_json(output / 'summary.json', summary)
    except Exception as exc:
        summary['errors'].append(_error('summary_write', exc))
        summary.update(overall_status='TECHNICAL_FAILURE', exit_code=interrupted_code or 1)
        print(f'summary_write: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        raise SummaryWriteError(str(exc)) from exc


def _read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def _check_technical_status(result, label):
    status = result.get('status') if isinstance(result, dict) else getattr(result, 'status', None)
    status = getattr(status, 'value', status)
    if status in {'FAILED', 'TECHNICAL_FAILURE', 'BACKEND_UNAVAILABLE', 'STALE', 'ERROR'}:
        raise RuntimeError(f'{label}: {status}: {result}')


def _owned_file(path, root):
    path = Path(path).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f'result is missing or outside this run: {path}')
    return path


def _mask_count(artifacts):
    import numpy as np
    with np.load(artifacts['cargo_masks.npz']['path'], allow_pickle=False) as archive:
        masks, ids, labels = archive['masks'], archive['mask_ids'], archive['labels']
        if masks.ndim != 3 or ids.shape != (len(masks),) or labels.shape != ids.shape:
            raise ValueError('invalid SAM mask/identity/label array structure')
        return len(masks)


def _algorithm_counts(result, geometry, mask_count):
    if not isinstance(result, dict) or not isinstance(geometry, dict):
        raise ValueError('geometry result must be a mapping')
    _check_technical_status(result, 'geometry')
    _check_technical_status(result.get('observation'), 'geometry observation')
    _check_technical_status(geometry, 'geometry file')
    sets = result['observed_face_sets']
    records = geometry['instances']
    if not isinstance(sets, (tuple, list)) or not isinstance(records, list) or len(records) != mask_count:
        raise ValueError('geometry result is missing per-mask records or observed face sets')
    for record in records:
        if (not isinstance(record, dict) or not isinstance(record.get('camera_facing_faces'), list)
                or not isinstance(record.get('accepted'), bool)):
            raise ValueError('geometry instance lacks camera_facing_faces/accepted structure')
    return {'observed_faces': sum(len(item.faces) for item in sets),
            'instances_without_accepted_face': sum(not record['camera_facing_faces'] for record in records),
            'complete_cuboids_accepted': sum(record['accepted'] for record in records)}


def evaluate_masks(folder, artifacts, output, *, payload):
    """Post-hoc rendered-mask evaluation; nominal box planes are not scan truth."""
    import cv2
    import numpy as np
    from evaluate_carton_appearance_ab import contour, boundary_metrics
    truth = {k: v for k, v in payload.instance_masks.items() if v.any()}
    with np.load(artifacts['cargo_masks.npz']['path'], allow_pickle=False) as prediction:
        masks, ids = prediction['masks'].astype(bool), prediction['mask_ids']
    gt_names = list(truth)
    areas = np.array([truth[k].sum() for k in gt_names])
    pred_areas = masks.sum(axis=(1,2))
    intersections = np.array([[np.count_nonzero(mask & truth[k]) for k in gt_names] for mask in masks])
    intersections = intersections.reshape((len(masks),len(truth)))
    iou = intersections / np.maximum(1, pred_areas[:,None]+areas[None,:]-intersections)
    relation = (intersections>=50)&(intersections>=.1*areas[None,:])&(intersections>=.1*pred_areas[:,None])
    rgb = payload.rgb[..., ::-1].copy()  # OpenCV drawing boundary is BGR.
    overlay=rgb.copy();gt_overlay=rgb.copy()
    pred_edge=np.zeros(rgb.shape[:2],bool);gt_edge=pred_edge.copy()
    for identity,mask in zip(ids,masks):
        edge=contour(mask);pred_edge|=edge
        color=tuple(int(v) for v in np.random.default_rng(int(identity)+17).integers(60,250,3))
        overlay[edge]=color
    for mask in truth.values(): gt_edge|=contour(mask)
    gt_overlay[gt_edge]=(255,255,0)
    for name, image in (('sam_boundaries.png', overlay), ('rendered_gt_boundaries_evaluation_only.png', gt_overlay)):
        if not cv2.imwrite(str(output/name), image):
            raise OSError(f'image write failed: {output/name}')
    summary={'visible_gt_objects':len(truth),'sam_count':len(masks),
        'merged_prediction_count':int((relation.sum(1)>1).sum()),
        'split_or_duplicate_gt_count':int((relation.sum(0)>1).sum()),
        'gt_below_best_iou_0_5':int((iou.max(0)<.5).sum()) if len(masks) else len(truth),
        'gt_without_significant_prediction':int((relation.sum(0)==0).sum()),
        'rendered_mask_boundary':boundary_metrics(pred_edge,gt_edge),
        'boundary_reference':'ACTUAL_RENDERED_OBJECT_MASK_CONTOURS_INCLUDES_OCCLUSION_AND_CLIP_EDGES',
        'exact_mesh_planes_corners':'NOT_EVALUATED',
        'policy':{'significant_pixels':50,'minimum_fraction_each_region':.1,'best_iou_threshold':.5}}
    _write_json(output/'mask_evaluation.json', {'summary':summary,'gt_objects':[
        {'id':k,'pixels':int(areas[j]),'best_iou':float(iou[:,j].max()) if len(masks) else 0.,
         'significant_sam_ids':[int(ids[i]) for i in np.flatnonzero(relation[:,j])]} for j,k in enumerate(gt_names)],
        'predictions':[{'sam_id':int(identity),'significant_gt_objects':[gt_names[j] for j in np.flatnonzero(relation[i])]} for i,identity in enumerate(ids)]})
    return summary


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--vision',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True);a=p.parse_args(argv)
    try:
        a.capture, a.vision = a.capture.resolve(), a.vision.resolve()
        output=a.capture/'perception-once'
        output.mkdir(exist_ok=False)
    except Exception as exc:
        print(f'output_directory: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        return 1
    summary = {'model_manifest': None, 'runs': [], 'run_id': None,
               'expected_module_count': None, 'errors': [], 'stage': 'initial_summary'}
    runtime = active = interrupted = None
    interrupted_code = None

    def checkpoint(stage, row=None):
        summary['stage'] = stage
        if row is not None:
            row['stage'] = stage
        _save_summary(output, summary)

    def fail_row(row, exc):
        row.update(status='TECHNICAL_FAILURE', failure_stage=row['stage'],
                   error_type=type(exc).__name__, error=str(exc))
        print(f"{row['module']}/{row['stage']}: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)

    try:
        checkpoint('run_identity')
        summary['run_id'] = uuid.uuid4().hex
        checkpoint('manifest')
        from unloading_perception.isaac_validation import IsaacSceneManifest
        manifest = IsaacSceneManifest.from_dict(_read_json(a.capture/'manifest.json'))
        summary['expected_module_count'] = len(manifest.cameras)
        summary['runs'] = [
            {'module': camera.get('module_id'), 'status': 'NOT_RUN', 'stage': 'pending',
             'sam_attempts': 0, 'metric_attempts': 0,
             'proposal_source': 'ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL',
             'raw_image_automatic': False, 'planning_admissible': False}
            for camera in manifest.cameras]
        modules = [row['module'] for row in summary['runs']]
        if (not modules or len(set(modules)) != len(modules) or
                any(not isinstance(module, str) or not module or module in {'.', '..', 'sam-runs'}
                    or '/' in module or '\\' in module for module in modules)):
            raise ValueError('manifest requires nonempty, unique, safe module IDs')
        from unloading_perception.isaac_payload import load_capture_payload, write_capture_snapshot
        prepared = {}
        for camera, row in zip(manifest.cameras, summary['runs']):
            active = row
            module = row['module']
            source = a.capture/'FULL_STACK_NOMINAL/modules'/module
            folder = output/module
            row.update(status='RUNNING', artifact_directory=str(folder))
            try:
                checkpoint('payload_validation', row)
                payload = load_capture_payload(source, manifest, expected_module_id=module, with_instance_masks=True)
                row['rgb_sha256'] = payload.binding.rgb_sha256
                checkpoint('module_directory', row)
                prepared[module] = write_capture_snapshot(payload, folder)
                row['input_provenance'] = prepared[module].input_provenance()
                row.update(status='NOT_RUN', stage='inputs_ready')
            except SummaryWriteError:
                raise
            except Exception as exc:
                fail_row(row, exc)
            checkpoint('inputs_checked')
            active = None
        if not prepared:
            raise RuntimeError('no valid capture payloads; model runtime was not initialized')
        checkpoint('models')
        models = summary['model_manifest'] = _read_json(a.models)
        checkpoint('config')
        import yaml
        config = yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text(encoding='utf-8'))
        checkpoint('dependencies')
        # Heavy optional imports happen only after a report and module roster exist.
        from run_metric_small_matrix import oracle_proposals, infer
        from run_isaac_rgbd_geometry import _worker_artifacts, _run_secondary_module
        from vision_resident_worker import ResidentRuntime, parser as worker_parser
        checkpoint('runtime_initialize')
        runtime=ResidentRuntime(worker_parser().parse_args(['--upstream-root',str(a.vision),
            '--output-root',str(output/'sam-runs'),'--input-root',str(output),'--sam-model',models['sam']['snapshot_path']]))
        for camera, row in zip(manifest.cameras, summary['runs']):
            if row['module'] not in prepared:
                continue
            active = row
            module = row['module']
            folder = output/module
            payload = prepared[module]
            row.update(status='RUNNING', artifact_directory=str(folder))
            try:
                checkpoint('oracle_proposals', row)
                binding, annotations, proposals = oracle_proposals(folder, camera, payload=payload)
                row['proposal_count'] = len(proposals['instances'])
                checkpoint('sam_inference', row)
                request_id = summary['run_id'] + '-' + module
                row['sam_attempts'] += 1
                response = infer(runtime, folder, binding, camera, request_id, payload=payload)
                _check_technical_status(response, 'SAM inference')
                checkpoint('worker_artifacts', row)
                response = _read_json(folder/'mode_b1_worker_response.json')
                if response['status'] != 'COMPLETE' or response['request_id'] != request_id:
                    raise ValueError('SAM response is incomplete or belongs to another request')
                metrics_path = _owned_file(response['metrics_reference']['path'], output/'sam-runs')
                safe_request = ''.join(c for c in request_id if c.isalnum() or c in '-_')
                if not metrics_path.parent.name.endswith(safe_request):
                    raise ValueError('SAM metrics belong to another request')
                artifacts = _worker_artifacts(folder, payload=payload)
                for name in ('cargo_masks.npz', 'cargo_instances.json', 'box_geometry_2d.json'):
                    _owned_file(artifacts[name]['path'], metrics_path.parent)
                count = _mask_count(artifacts)
                checkpoint('mask_evaluation', row)
                row['mask_evaluation'] = evaluate_masks(folder, artifacts, folder, payload=payload)
                if count == 0:
                    # The existing point-map builder requires at least one mask.
                    # A verified empty segmentation is a legal algorithm result.
                    checkpoint('empty_segmentation_result', row)
                    _write_json(folder/'rgbd_cuboids.json', {'instances': [], 'reason': 'EMPTY_SEGMENTATION',
                                                         'run_id': summary['run_id']})
                    result = {'observed_face_sets': ()}
                    row['metric_not_run_reason'] = 'EMPTY_SEGMENTATION'
                else:
                    checkpoint('metric_geometry', row)
                    row['metric_attempts'] += 1
                    result = _run_secondary_module(scene='FULL_STACK_NOMINAL', module_dir=folder, manifest=manifest,
                        artifacts=artifacts, config=config, vision_root=a.vision, upstream_python=Path(sys.executable), timeout=1200,
                        payload=payload)
                    _check_technical_status(result, 'geometry')
                checkpoint('geometry_result', row)
                geometry = _read_json(_owned_file(folder/'rgbd_cuboids.json', folder))
                row.update(_algorithm_counts(result, geometry, count))
                row.update(status=COMPLETED, stage='completed')
            except SummaryWriteError:
                raise
            except Exception as exc:
                fail_row(row, exc)
                try:
                    (folder/'failure.txt').write_text(traceback.format_exc(), encoding='utf-8')
                except Exception as report_exc:
                    summary['errors'].append(_error('failure_report', report_exc))
                    print(f'failure_report: {report_exc}', file=sys.stderr, flush=True)
            checkpoint('module_finished')
            print(json.dumps(row), flush=True)
            active = None
        checkpoint('runtime_guard')
        if runtime.moge_model is not None:
            raise RuntimeError('unexpected MoGe model in SAM-only runtime')
    except (KeyboardInterrupt, SystemExit) as exc:
        interrupted = exc
        interrupted_code = (130 if isinstance(exc, KeyboardInterrupt) else
                            exc.code if isinstance(exc.code, int) and 1 <= exc.code <= 255 else 1)
        summary['errors'].append(_error(summary['stage'], exc))
        if active is not None and active['status'] == 'RUNNING':
            fail_row(active, exc)
        print(f"{summary['stage']}: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
    except Exception as exc:
        if not isinstance(exc, SummaryWriteError):
            summary['errors'].append(_error(summary['stage'], exc))
        if active is not None and active['status'] == 'RUNNING':
            fail_row(active, exc)
        print(f"{summary['stage']}: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
    finally:
        # ResidentRuntime has no close/shutdown method; its CLI shutdown is a
        # protocol message, not an in-process API. Release this call's reference.
        runtime = None
    for row in summary['runs']:
        if row['status'] == 'NOT_RUN':
            row['not_run_reason'] = 'INTERRUPTED' if interrupted else 'RUN_FAILURE:' + summary['stage']
    try:
        _save_summary(output, summary, final=True, interrupted_code=interrupted_code)
    except SummaryWriteError:
        # Best effort to publish the write failure itself, never a success retry.
        try:
            _save_summary(output, summary, final=True, interrupted_code=interrupted_code)
        except SummaryWriteError:
            pass  # Both failures are already on stderr; earlier progress remains nonzero.
    if interrupted is not None:
        if isinstance(interrupted, KeyboardInterrupt):
            raise interrupted
        raise SystemExit(interrupted_code) from interrupted
    return summary['exit_code']


if __name__=='__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
