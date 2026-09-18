"""Explicit CPU-only model/worker/image substitutes; no real perception runs.

The production script, manifest validation, summary writer and CLI remain real.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import subprocess
from types import ModuleType, SimpleNamespace

import numpy as np

from unloading_perception.isaac_validation import IsaacSceneManifest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/run_workcell_perception_once.py"


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload), encoding="utf-8")


def capture_fixture(root, scenarios=("sam_error", "normal")):
    capture = root / "capture"
    capture.mkdir()
    from ros2_ws.src.unloading_ros_bridge.test.isaac_joint_fixture import capture_fixture as bound_fixture
    manifest, _, _ = bound_fixture(capture)
    modules = [camera['module_id'] for camera in manifest.cameras]
    assert len(modules) == len(scenarios) == 2
    write_json(capture / 'test-control.json', dict(zip(modules, scenarios)))
    models = root / 'models.json'
    write_json(models, {'sam': {'snapshot_path': 'CPU_TEST_SUBSTITUTE'}})
    for index, module in enumerate(modules):
        folder = capture / 'FULL_STACK_NOMINAL/modules' / module
        _, _, binding = bound_fixture(folder, camera_index=index)
        annotations = json.loads((folder/'gt_annotations.json').read_text(encoding='utf-8'))
        object_id = manifest.objects[0]['simulation_object_id']
        annotations['objects'] = [{'simulation_object_id': object_id, 'mask_key': object_id,
            'visible': True, 'occluded': False, 'bbox_xyxy': [0, 0, 3, 2]}]
        write_json(folder/'gt_annotations.json', annotations)
        np.savez(folder/'gt_instance_masks.npz', **{object_id: np.ones((2, 3), dtype=bool)})
        # Author a complete new synthetic acquisition before any fault injection.
        binding['gt_snapshot_sha256'] = hashlib.sha256((folder/'gt_annotations.json').read_bytes()).hexdigest()
        binding['instance_masks_sha256'] = hashlib.sha256((folder/'gt_instance_masks.npz').read_bytes()).hexdigest()
        write_json(folder/'capture_binding.json', binding)
    return capture, models, modules


def install(monkeypatch, capture):
    monkeypatch.syspath_prepend(str(ROOT/'tools'))
    from run_metric_small_matrix import oracle_proposals as production_proposals
    scenarios = json.loads((capture / "test-control.json").read_text(encoding="utf-8"))
    calls = []

    def mode(folder):
        return scenarios[folder.name]

    def record(stage, module):
        calls.append((stage, module))
        with (capture / "test-calls.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps([stage, module]) + "\n")
        progress = capture / 'perception-once/summary.json'
        if progress.exists():
            with (capture / 'test-progress.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(json.dumps({'call': stage, 'summary': json.loads(progress.read_text(encoding='utf-8'))}) + '\n')

    def digest(path):
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()

    def proposals(folder, camera, *, payload):
        record("proposals", folder.name)
        return production_proposals(folder, camera, payload=payload)

    class Runtime:
        moge_model = None

        def __init__(self, args):
            record("initialize", "shared")
            if 'init_error' in scenarios.values():
                raise RuntimeError('CPU_TEST_INITIALIZATION_FAILURE')
            self.output_root = args.output_root
            self.output_root.mkdir(parents=True)

    def infer(runtime, folder, binding, camera, request_id, *, payload):
        record("sam", folder.name)
        if mode(folder) == "sam_error":
            raise RuntimeError("CPU_TEST_SAM_FAILURE")
        if mode(folder) == 'sam_timeout':
            raise subprocess.TimeoutExpired('CPU_TEST_WORKER', .01)
        if mode(folder) == 'sam_structured':
            return {'status': 'FAILED', 'error_code': 'CPU_TEST_STRUCTURED_FAILURE'}
        if mode(folder) == 'sam_missing_response':
            return
        if mode(folder) == 'interrupt':
            raise KeyboardInterrupt('CPU_TEST_INTERRUPT')
        if mode(folder) == 'system_exit':
            raise SystemExit(7)
        if mode(folder) == 'system_exit_zero':
            raise SystemExit(0)
        run = runtime.output_root / request_id
        run.mkdir()
        count = 0 if mode(folder) == 'empty' else 1
        np.savez(run / "cargo_masks.npz", masks=np.ones((count, 2, 3), dtype=bool),
                 mask_ids=np.arange(1, count+1), labels=np.array(["box"] * count),
                 boxes=np.tile([0, 0, 3, 2], (count, 1)), scores=np.full(count, .8),
                 sources=np.full(count, 'CPU_TEST_SUBSTITUTE'))
        records = [dict(instance_id=1, proposal_id=1, bbox=[0, 0, 3, 2], label='box',
                        boundary_source='CPU_TEST_SUBSTITUTE', validation_score=.8, sam_iou_score=.9,
                        sam_prompt_stability=.99, mask_area=6)] if count else []
        write_json(run/'cargo_instances.json', {
            'source': str(folder/'sensor_rgb.png'), 'box_source': str(folder/'oracle_proposals.json'),
            'instances': records, 'proposal_audit': [{'status': 'accepted', 'proposal_id': 1, 'instance_id': 1}] if count else [],
        })
        write_json(run/'box_geometry_2d.json', {'instances': []})
        artifacts = {name: {"path": str(run / name), "sha256": digest(run / name)}
                     for name in ("cargo_masks.npz", "cargo_instances.json", "box_geometry_2d.json")}
        metrics = run / "metrics.json"
        write_json(metrics, {"artifacts": artifacts})
        write_json(folder / "mode_b1_worker_response.json", {
            "status": "FAILED" if mode(folder) == 'response_failed' else "COMPLETE",
            "request_id": 'old-request' if mode(folder) == 'old_request' else request_id,
            "metrics_reference": {"path": str(metrics), "sha256": digest(metrics)},
        })
        if mode(folder) == 'old_artifacts':
            write_json(folder / 'mode_b1_worker_response.json', {
                'status': 'COMPLETE', 'request_id': request_id,
                'metrics_reference': {'path': str(capture / 'old-metrics.json')},
            })
        if mode(folder) == 'corrupt_masks':
            (run / 'cargo_masks.npz').write_bytes(b'not an npz')

    def artifacts(folder, *, payload):
        record("artifacts", folder.name)
        response = json.loads((folder / "mode_b1_worker_response.json").read_text(encoding="utf-8"))
        result = json.loads(Path(response["metrics_reference"]["path"]).read_text(encoding="utf-8"))["artifacts"]
        # Even the synthetic normal/empty outputs must satisfy real lineage rules.
        from unloading_perception.lineage import load_instance_lineage
        load_instance_lineage(Path(result['cargo_masks.npz']['path']), Path(result['cargo_instances.json']['path']),
                              json.loads((folder/'oracle_proposals.json').read_text(encoding='utf-8')), {'instances': []},
                              sensor_epoch=payload.metadata.sensor_epoch, module_id=folder.name, capture_id=payload.metadata.capture_id,
                              source_path=folder/'sensor_rgb.png', proposal_path=folder/'oracle_proposals.json')
        return result

    def geometry(**kwargs):
        folder = kwargs["module_dir"]
        record("metric", folder.name)
        if mode(folder) == 'metric_error':
            raise RuntimeError('CPU_TEST_GEOMETRY_FAILURE')
        if mode(folder) == 'metric_timeout':
            raise subprocess.TimeoutExpired('CPU_TEST_GEOMETRY', .01)
        if mode(folder) == 'metric_structured':
            return {'status': 'TECHNICAL_FAILURE', 'error': 'CPU_TEST_GEOMETRY_STATUS'}
        write_json(folder / "rgbd_cuboids.json", {"instances": [{"camera_facing_faces": [], "accepted": False}]})
        if mode(folder) == 'geometry_invalid':
            (folder / 'rgbd_cuboids.json').write_text('{broken', encoding='utf-8')
        if mode(folder) == 'geometry_structure':
            write_json(folder / 'rgbd_cuboids.json', {'instances': [{}]})
        if mode(folder) == 'geometry_missing':
            (folder / 'rgbd_cuboids.json').unlink()
        if mode(folder) == 'missing_face_sets':
            return {}
        return {"observed_face_sets": (), "observation": SimpleNamespace(status="FAILED" if mode(folder) == 'observation_failed' else "PARTIAL")}

    matrix = ModuleType("run_metric_small_matrix")
    matrix.oracle_proposals, matrix.infer = proposals, infer
    secondary = ModuleType("run_isaac_rgbd_geometry")
    secondary._worker_artifacts, secondary._run_secondary_module = artifacts, geometry
    secondary.IsaacSceneManifest, secondary.sha256 = IsaacSceneManifest, digest
    worker = ModuleType("vision_resident_worker")
    worker.ResidentRuntime = Runtime

    def parser():
        result = argparse.ArgumentParser()
        for key in ("upstream-root", "output-root", "input-root", "sam-model"):
            result.add_argument("--" + key, type=Path)
        return result

    worker.parser = parser
    cv2 = ModuleType("cv2")
    cv2.imread = lambda path: np.zeros((2, 3, 3), dtype=np.uint8)

    def image_write(path, image):
        if mode(Path(path).parent) == 'image_write_failure':
            return False
        Path(path).write_bytes(b"CPU_IMAGE_OUTPUT_SUBSTITUTE")
        return True

    cv2.imwrite = image_write
    evaluation = ModuleType("evaluate_carton_appearance_ab")
    evaluation.contour = lambda mask: mask
    evaluation.boundary_metrics = lambda prediction, truth: {"test_substitute": True}
    for name, value in (("run_metric_small_matrix", matrix), ("run_isaac_rgbd_geometry", secondary),
                        ("vision_resident_worker", worker), ("cv2", cv2),
                        ("evaluate_carton_appearance_ab", evaluation)):
        monkeypatch.setitem(sys.modules, name, value)
    return calls
