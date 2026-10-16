"""Shared proposal -> pinned SAM stage for one-shot and resident video entrypoints."""
import json
from pathlib import Path
from time import perf_counter
import uuid
import numpy as np
from unloading_perception.proposals import (DetectorConfig, DETECTOR_SOURCE, ORACLE_SOURCE,
    fingerprint, validate_detector_document)
from unloading_perception.rgb_input import load_rgb_input, snapshot_rgb_input


def write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')


def add_arguments(parser, *, include_sam_stop=True):
    parser.add_argument('--proposal-mode', choices=('grounding-dino', 'oracle'), default='grounding-dino')
    parser.add_argument('--detector-config', type=Path,
                        default=Path(__file__).resolve().parents[1]/'configs/perception/grounding_dino.json')
    if include_sam_stop:
        parser.add_argument('--stop-after', choices=('sam', 'geometry'))
    parser.add_argument('--capture-scene', default='FULL_STACK_NOMINAL')
    parser.add_argument('--evaluate-gt', action='store_true', help='load GT only after predictions are frozen')


def runtime_arguments(args, output, models):
    from vision_resident_worker import parser
    return parser().parse_args(['--upstream-root', str(args.vision), '--output-root', str(output/'sam-runs'),
        '--input-root', str(output), '--sam-model', models['sam']['snapshot_path'],
        '--detector-manifest', str(args.models.resolve()),
        '--detector-config', json.dumps(DetectorConfig(**json.loads(args.detector_config.read_text())).to_dict()),
        '--processing-divisor', str(args.processing_divisor)] + ([] if args.sam_image_reuse else ['--no-sam-image-reuse']))


def prepare_proposals(runtime, folder, *, mode, oracle_payload=None):
    from vision_resident_worker import sha256
    rgb = load_rgb_input(folder)
    started = perf_counter()
    if mode == 'grounding-dino':
        prediction = runtime.detect(rgb.rgb)
        identity = runtime.detector.identity
        namespace = dict(capture_id=rgb.metadata.capture_id, module_id=rgb.camera['module_id'],
                         sensor_epoch=rgb.metadata.sensor_epoch, frame_sequence=rgb.metadata.frame_sequence)
        document = dict(schema_version='detector_proposals_v1', coordinate_space='source_image',
            bbox_convention='continuous xyxy [0,W] x [0,H]', source=DETECTOR_SOURCE,
            raw_image_automatic=True, text_source='PRECONFIGURED_CATEGORY_TEXT',
            acquisition_source='ISAAC_RENDERED_RGB', rgb_sha256=rgb.rgb_sha256,
            source_size=[rgb.rgb.shape[1], rgb.rgb.shape[0]], detector=identity, **namespace,
            candidates=prediction['candidates'], model_input_shape=prediction['model_input_shape'],
            truncated=any(r.get('reason') == 'CANDIDATE_LIMIT' for r in prediction['candidates']),
            raw_output_coordinates='normalized_cxcywh; raw logits and token IDs in detector_raw.npz',
            instances=[dict(id=r['proposal_id'], instance_id=r['proposal_id'], proposal_id=r['proposal_id'],
                bbox=r['bbox_xyxy'], bbox_xyxy=r['bbox_xyxy'], raw_text_label=r['raw_text_label'],
                label='box', detector_score=r['detector_score'], proposal_source=DETECTOR_SOURCE,
                filter_candidate_id=r['proposal_id']) for r in prediction['final']])
        validate_detector_document(document, rgb_sha256=rgb.rgb_sha256, width=rgb.rgb.shape[1], height=rgb.rgb.shape[0], **namespace)
        np.savez_compressed(folder/'detector_raw.npz', **prediction['raw'])
        path = folder/'detector_proposals.json'
        timing = prediction['timings_seconds']
    elif mode == 'oracle':
        if oracle_payload is None:
            raise ValueError('EXPLICIT_ORACLE_REQUIRES_VERIFIED_EVALUATION_PAYLOAD')
        from run_metric_small_matrix import oracle_proposal_document
        document = oracle_proposal_document(oracle_payload)
        path = folder/'oracle_proposals.json'; timing = {}
    else:
        raise ValueError('unsupported proposal mode')
    write(path, document)
    write(folder/'proposal_binding.json', dict(filename=path.name, sha256=sha256(path), source=document['source']))
    timing['proposal_stage_including_io'] = perf_counter()-started
    return document, timing


def infer_sam(runtime, folder, request_id, *, mode, stop_after=None):
    from vision_resident_worker import sha256
    from unloading_perception.proposals import proposal_path
    rgb = load_rgb_input(folder)
    path = proposal_path(folder)
    request = dict(schema_version='1.1.0', op='infer', request_id=request_id, worker_epoch='workcell-perception',
        proposal_mode=mode, stop_after=stop_after, frame=rgb.frame(),
        proposal_reference=dict(uri=path.as_uri(), sha256=sha256(path)))
    response = runtime.infer(request)
    if response['status'] != 'COMPLETE' or response['request_id'] != request_id:
        raise RuntimeError('SAM_TECHNICAL_FAILURE:' + str(response))
    write(folder/'mode_b1_worker_response.json', response)
    return response


def run_sam_only(args, *, runtime_factory=None):
    """Same stages as full entry; no geometric results fabricated for the viewer."""
    from vision_resident_worker import ResidentRuntime, sha256
    from build_sam_review import build_review
    output = (args.output_directory or args.capture/'perception-sam').resolve()
    output.mkdir(parents=True, exist_ok=False)
    summary = dict(run_id=uuid.uuid4().hex, stop_after='sam', runs=[], errors=[], overall_status='RUNNING',
        input_validation_scope='RGB_ONLY_NO_GT_OR_DEPTH_READS',
        stages={s:'NOT_RUN' for s in ('geometry_2d', 'geometry', 'contacts', 'fusion', 'ros', 'planning', 'execution')},
        absence_means_free_space=False, planning_admissible=False)
    write(output/'summary.json', summary)
    try:
        models = json.loads(args.models.read_text(encoding='utf-8'))
        sources = sorted((args.capture/getattr(args, 'capture_scene', 'FULL_STACK_NOMINAL')/'modules').glob('*'))
        if not sources and (args.capture/'sensor_rgb.png').is_file():
            sources = [args.capture]
        if not sources:
            raise FileNotFoundError('no RGB capture modules')
        prepared = []
        for source in sources:
            row = dict(module=source.name, status='NOT_RUN', proposal_source=DETECTOR_SOURCE if args.proposal_mode=='grounding-dino' else ORACLE_SOURCE)
            summary['runs'].append(row)
            try:
                rgb = load_rgb_input(source)
                folder = output/rgb.camera['module_id']
                snapshot_rgb_input(rgb, folder)
                prepared.append((source, folder, row))
                row.update(module=rgb.camera['module_id'],rgb_sha256=rgb.rgb_sha256,
                           capture_id=rgb.metadata.capture_id, artifact_directory=str(folder))
            except Exception as exc:
                row.update(status='TECHNICAL_FAILURE', error=str(exc))
        # Freeze input hashes and configuration before initializing models.
        write(output/'frozen_inputs.json', dict(inputs=summary['runs'], models=models,
            config=DetectorConfig(**json.loads(args.detector_config.read_text())).to_dict(), proposal_mode=args.proposal_mode))
        from unloading_perception.finite_sequence import verify_sam_files
        verify_sam_files(models)
        started = perf_counter()
        previous_runtime = getattr(runtime_factory, 'runtime', None)
        runtime = (runtime_factory or ResidentRuntime)(runtime_arguments(args, output, models))
        summary['runtime_acquire_seconds'] = perf_counter()-started
        summary['sam_cold_start_seconds'] = summary['runtime_acquire_seconds'] if runtime is not previous_runtime else 0.
        summary['sam_model_load_seconds'] = getattr(runtime, 'model_load_seconds', None) if runtime is not previous_runtime else 0.
        for source, folder, row in prepared:
            started = perf_counter()
            try:
                oracle = None
                if args.proposal_mode == 'oracle':
                    from unloading_perception.isaac_payload import load_capture_payload
                    oracle = load_capture_payload(source, args.capture/'manifest.json', with_instance_masks=True)
                document, timings = prepare_proposals(runtime, folder, mode=args.proposal_mode, oracle_payload=oracle)
                response = infer_sam(runtime, folder, summary['run_id']+'-'+row['module'], mode=args.proposal_mode, stop_after='sam')
                metrics_path = Path(response['metrics_reference']['path']).resolve()
                if (not metrics_path.is_relative_to((output/'sam-runs').resolve())
                        or sha256(metrics_path) != response['metrics_reference']['sha256']):
                    raise ValueError('SAM_METRICS_BINDING_MISMATCH')
                metrics = json.loads(metrics_path.read_text())
                if (response['input_sha256'] != row['rgb_sha256']
                        or metrics['config']['proposal_sha256'] != sha256(folder/('detector_proposals.json' if args.proposal_mode=='grounding-dino' else 'oracle_proposals.json'))):
                    raise ValueError('SAM_RESULT_INPUT_MISMATCH')
                for ref in metrics['artifacts'].values():
                    if not Path(ref['path']).resolve().is_relative_to(metrics_path.parent) or sha256(Path(ref['path'])) != ref['sha256']:
                        raise ValueError('SAM_ARTIFACT_HASH_MISMATCH')
                mask_ready = perf_counter()
                row.update(proposal_count=len(document['instances']), timings_seconds={**timings, **response['stage_timings_seconds'],
                    'rgb_to_masks_available':mask_ready-started}, status='COMPLETED', artifacts=metrics['artifacts'])
                if args.evaluate_gt:
                    from evaluate_detector_sam import evaluate
                    row['evaluation'] = evaluate(source, folder, metrics['artifacts'])
                review_started = perf_counter()
                row['review'] = build_review(folder, metrics['artifacts'])
                row['timings_seconds']['evaluation_and_review_io'] = perf_counter()-review_started
                row['mask_count'] = row['review']['mask_count']
            except Exception as exc:
                import traceback
                (folder/'failure.txt').write_text(traceback.format_exc(), encoding='utf-8')
                row.update(status='TECHNICAL_FAILURE', error_type=type(exc).__name__, error=str(exc))
            write(output/'summary.json', summary)
        if getattr(runtime, 'detector', None) is not None:
            summary['detector_cold_start_seconds'] = runtime.detector.cold_start_seconds
            summary['detector_identity'] = runtime.detector.identity
    except Exception as exc:
        summary['errors'].append(dict(type=type(exc).__name__, error=str(exc)))
    summary['overall_status'] = 'COMPLETED' if summary['runs'] and all(r['status']=='COMPLETED' for r in summary['runs']) and not summary['errors'] else 'TECHNICAL_FAILURE'
    summary['exit_code'] = 0 if summary['overall_status']=='COMPLETED' else 1
    write(output/'summary.json', summary)
    return summary['exit_code']
