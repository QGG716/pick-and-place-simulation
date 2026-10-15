"""Optional resident HF detector; semantic inputs are exclusively RGB + config."""
import importlib.metadata
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from unloading_perception.proposals import DetectorConfig, filter_candidates, fingerprint

MODEL_ID = 'IDEA-Research/grounding-dino-base'


def official_postprocess(processor, outputs, input_ids, *, height, width, text_threshold):
    return processor.post_process_grounded_object_detection(outputs, input_ids,
        threshold=0., text_threshold=text_threshold, target_sizes=[(height, width)])[0]


class GroundingDinoRuntime:
    def __init__(self, manifest, config):
        from vision_resident_worker import sha256
        self.config = config
        if 'grounding_dino' not in manifest:
            raise RuntimeError('DETECTOR_MODEL_MISSING: run prepare_vision_models.py --add-grounding-dino first')
        item = manifest['grounding_dino']
        if item['repository'] != MODEL_ID or len(item['revision']) != 40 or not item['files']:
            raise ValueError('DETECTOR_MODEL_IDENTITY_INVALID')
        path = Path(item['snapshot_path'])
        required = {'config.json', 'preprocessor_config.json', 'model.safetensors', 'tokenizer_config.json', 'tokenizer.json'}
        if not required.issubset(item['files']):
            raise ValueError('DETECTOR_MODEL_MANIFEST_INCOMPLETE')
        for name, digest in item['files'].items():
            if Path(name).name != name or sha256(path/name) != digest:
                raise ValueError('DETECTOR_MODEL_HASH_MISMATCH:' + name)
        import torch
        import transformers
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
        # This adapter is tested against the existing locked SAM environment.
        if transformers.__version__ != '4.53.2':
            raise RuntimeError('DETECTOR_REQUIRES_PINNED_TRANSFORMERS_4.53.2')
        if not torch.cuda.is_available():
            raise RuntimeError('DETECTOR_GPU_UNAVAILABLE')
        self.torch = torch
        torch.manual_seed(config.seed)
        torch.cuda.manual_seed_all(config.seed)
        torch.cuda.synchronize(); started = perf_counter()
        self.processor = AutoProcessor.from_pretrained(str(path), local_files_only=True)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(str(path), local_files_only=True).to('cuda').eval()
        torch.cuda.synchronize()
        self.cold_start_seconds = perf_counter() - started
        self.identity = dict(repository=MODEL_ID, revision=item['revision'], files=item['files'],
            weight_source=item.get('weight_source', 'https://huggingface.co/' + MODEL_ID),
            config=config.to_dict(), config_identity=fingerprint(config.to_dict()),
            dependencies={k: importlib.metadata.version(k) for k in ('torch', 'transformers', 'tokenizers', 'huggingface-hub', 'safetensors')})

    def detect(self, rgb):
        """No metadata, GT, depth, SAM output, or object-count argument is accepted."""
        from PIL import Image
        t = self.torch
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('DETECTOR_REQUIRES_RAW_RGB_UINT8')
        height, width = rgb.shape[:2]
        t.cuda.synchronize(); started = perf_counter()
        inputs = self.processor(images=Image.fromarray(rgb), text=self.config.text,
            size={'shortest_edge': self.config.shortest_edge, 'longest_edge': self.config.longest_edge},
            return_tensors='pt').to('cuda')
        t.cuda.synchronize(); preprocessed = perf_counter()
        with t.inference_mode():
            outputs = self.model(**inputs)
        t.cuda.synchronize(); inferred = perf_counter()
        # v4.53.2 uses threshold and text_labels. target_sizes is H,W, not W,H.
        result = official_postprocess(self.processor, outputs, inputs.input_ids, height=height, width=width,
                                      text_threshold=self.config.text_threshold)
        boxes = result['boxes'].detach().cpu().numpy()
        scores = result['scores'].detach().cpu().numpy()
        rows, final = filter_candidates(boxes, scores, result['text_labels'], width, height, self.config)
        raw = dict(logits=outputs.logits.detach().cpu().numpy(),
                   normalized_cxcywh=outputs.pred_boxes.detach().cpu().numpy(),
                   input_ids=inputs.input_ids.detach().cpu().numpy())
        t.cuda.synchronize(); finished = perf_counter()
        return dict(candidates=rows, final=final, raw=raw, model_input_shape=list(inputs.pixel_values.shape),
                    timings_seconds=dict(detector_preprocess=preprocessed-started,
                        detector_inference=inferred-preprocessed, detector_postprocess=finished-inferred))
