"""RGB-only capture reader. GT, depth and scene object lists are never opened."""
from dataclasses import dataclass
from io import BytesIO
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from .isaac_payload import camera_content_identity
from .rgbd import CaptureMetadata


@dataclass(frozen=True)
class RgbInput:
    directory: Path
    rgb: np.ndarray
    rgb_sha256: str
    metadata: CaptureMetadata
    camera: dict
    raw_files: dict

    def frame(self):
        m = self.metadata
        return dict(source='isaac-sim-6.0.1', stream='perception_validation', epoch=m.sensor_epoch,
                    sequence=m.frame_sequence, capture_time=m.capture_center_time, receive_time=m.capture_center_time,
                    clock_domain=m.clock_domain, frame_id=m.rgb_frame_id, module_id=self.camera['module_id'],
                    capture_id=m.capture_id, width=self.rgb.shape[1], height=self.rgb.shape[0], encoding='rgb8',
                    rgb_uri=(self.directory/'sensor_rgb.png').as_uri(), rgb_sha256=self.rgb_sha256)


def load_rgb_input(directory, *, expected_module_id=None, expected_frame=None):
    directory = Path(directory).resolve()
    raw = {}
    def read(name, expected=None):
        value = (directory/name).read_bytes()
        if expected is not None and (len(expected) != 64 or hashlib.sha256(value).hexdigest() != expected):
            raise ValueError('RGB_INPUT_HASH_MISMATCH:' + name)
        raw[name] = value
        return value
    binding = json.loads(read('capture_binding.json'))
    metadata = CaptureMetadata.from_dict(json.loads(read('capture_metadata.json', binding['capture_metadata_sha256'])))
    camera = json.loads(read('camera_info.json'))
    identity = camera_content_identity(camera)
    module = camera['module_id']
    if (binding['camera_calibration_identity'] != identity or metadata.calibration_identity != identity
            or binding['module_id'] != module or expected_module_id not in (None, module)
            or binding['capture_id'] != metadata.capture_id
            or binding['simulation_epoch'] != metadata.sensor_epoch
            or binding['frame_sequence'] != metadata.frame_sequence
            or expected_frame not in (None, metadata.frame_sequence)
            or binding['simulation_time'] != metadata.capture_center_time
            or binding.get('clock_domain', metadata.clock_domain) != metadata.clock_domain
            or metadata.rgb_frame_id != camera['frame_id']
            or metadata.sync_status != 'SYNCHRONIZED'
            or metadata.T_W_C_at_capture != tuple(tuple(r) for r in camera['T_W_C'])):
        raise ValueError('RGB_INPUT_IDENTITY_MISMATCH')
    with Image.open(BytesIO(read('sensor_rgb.png', binding['rgb_sha256']))) as image:
        if image.format != 'PNG' or image.mode != 'RGB' or image.size != (camera['width'], camera['height']):
            raise ValueError('RGB_INPUT_SIZE_OR_MODE_MISMATCH')
        rgb = np.array(image)
    rgb.setflags(write=False)
    return RgbInput(directory, rgb, binding['rgb_sha256'], metadata, camera, raw)


def snapshot_rgb_input(payload, directory):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    for name, raw in payload.raw_files.items():
        with (directory/name).open('xb') as stream:
            stream.write(raw)
    return load_rgb_input(directory, expected_module_id=payload.camera['module_id'],
                          expected_frame=payload.metadata.frame_sequence)
