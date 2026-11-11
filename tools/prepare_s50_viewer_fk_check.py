"""Write real URDF FK probes for an independent JavaScript viewer comparison."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from unloading_sim.s50_cad_model import ensure_preview_assets, load_definition, load_preview_robot


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/s50_cad_motion_20261110/viewer-fk-input.json')
    args = parser.parse_args(argv)
    ensure_preview_assets(ROOT)
    definition = load_definition(ROOT)
    robot = load_preview_robot(ROOT)
    q_values = [np.zeros(6)]
    for index in range(6):
        for sign in (-1, 1):
            q = np.zeros(6); q[index] = sign*.23; q_values.append(q)
    q_values.extend(np.random.default_rng(50).uniform(-1.7, 1.7, (12, 6)))
    probes = []
    mount = np.eye(4); mount[:3, 3] = [-1.85, .15, .9]
    world_robot = load_preview_robot(ROOT, world_from_mount=mount)
    for coordinate, instance, transform in [('sim_mount', robot, np.eye(4)), ('fixed_mount_translation', world_robot, mount)]:
        for q in q_values:
            probes.append(dict(q_rad=q.tolist(), coordinate_scope=coordinate, world_from_mount=transform.tolist(),
                expected_frames={k:v.tolist() for k,v in instance.named_link_frames(q).items()}))
    data = dict(schema='s50_viewer_independent_fk_input_v1', model=definition, probes=probes,
        source='Actual URDFRobot FK; independent viewer implementation must calculate its own matrices',
        display_ranges_are_hardware_limits=False, real_installation_qualified=False,
        javascript_sha256=hashlib.sha256((ROOT/'tools/s50_kinematics.js').read_bytes()).hexdigest())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'output':str(args.output),'probes':len(probes),'definition':definition['definition_sha256']}))


if __name__ == '__main__':
    main()
