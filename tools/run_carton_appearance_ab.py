"""Four fresh native-frame SAM/metric runs after checking appearance controls."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from run_isaac_rgbd_geometry import _worker_artifacts,_run_secondary_module,IsaacSceneManifest,sha256
from run_metric_small_matrix import infer


def folder(root,group,module):
    return root/group/'FULL_STACK_NOMINAL/modules'/module


def controls(root,historical):
    rows=[]
    a=json.loads((root/'A/manifest.json').read_text());b=json.loads((root/'B/manifest.json').read_text())
    same_scene=all(a[k]==b[k] for k in ('objects','robot','mechanisms','cameras','layout'))
    configs=[json.loads((root/g/'capture_configuration.json').read_text()) for g in ('A','B')]
    same_render=all(configs[0][k]==configs[1][k] for k in ('render_settings','stable_frames','robot_q_rad','environment_distant_intensity','environment_sphere_intensity','mechanical_entities_omitted'))
    for camera in a['cameras']:
        module=camera['module_id'];pa,pb=[folder(root,g,module) for g in ('A','B')]
        da,db=[np.load(p/'metric_depth_m.npy') for p in (pa,pb)]
        va,vb=[np.isfinite(d)&(d>0) for d in (da,db)]
        difference=np.abs(da[va&vb].astype(float)-db[va&vb])
        ga,gb=[np.load(p/'gt_instance_masks.npz') for p in (pa,pb)]
        gt_equal=set(ga.files)==set(gb.files) and all(np.array_equal(ga[k],gb[k]) for k in ga.files)
        old=historical/'FULL_STACK_NOMINAL'
        if module=='module_1_lower':old=old/'modules'/module
        d0=np.load(old/'metric_depth_m.npy');v0=np.isfinite(d0)&(d0>0)
        common=v0&va;old_diff=np.abs(d0[common].astype(float)-da[common])
        rows.append({'module':module,'same_scene_geometry_K_T':same_scene,'same_render_state':same_render,
            'validity_disagreement_pixels':int(np.count_nonzero(va!=vb)),
            'depth_mean_absolute_difference_m':float(difference.mean()),'depth_max_absolute_difference_m':float(difference.max()),
            'gt_masks_equal_by_physical_object':gt_equal,
            'rgb_sha256':{g:sha256(folder(root,g,module)/'sensor_rgb.png') for g in ('A','B')},
            'camera_info_equal':json.loads((pa/'camera_info.json').read_text())==json.loads((pb/'camera_info.json').read_text()),
            'A0_cube_vs_A_mesh':{'validity_disagreement_pixels':int(np.count_nonzero(va!=v0)),'common_depth_max_difference_m':float(old_diff.max()),
                                'note':'Historical reference; different capture, target highlight and warmup. Analytic mesh surfaces are exactly the original Cube boundary.'}})
    result={'scope':'APPEARANCE_ONLY_A_B_FULL_MECHANICAL_SCENE','modules':rows,
            'strict_equal':all(v['same_scene_geometry_K_T'] and v['same_render_state'] and v['camera_info_equal'] and v['validity_disagreement_pixels']==0 and v['depth_max_absolute_difference_m']==0 and v['gt_masks_equal_by_physical_object'] for v in rows),
            'proceed_numerical_tolerance_m':1e-6}
    (root/'ab_controls.json').write_text(json.dumps(result,indent=2))
    if not all(v['same_scene_geometry_K_T'] and v['same_render_state'] and v['camera_info_equal'] and v['validity_disagreement_pixels']==0 and v['depth_max_absolute_difference_m']<=1e-6 and v['gt_masks_equal_by_physical_object'] and len(set(v['rgb_sha256'].values()))==2 for v in rows):
        raise RuntimeError('APPEARANCE_CONTROLS_FAILED_SEE_AB_CONTROLS_JSON')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--historical',type=Path,required=True)
    p.add_argument('--vision',type=Path,required=True);p.add_argument('--models',type=Path,required=True)
    p.add_argument('--controls-only',action='store_true');args=p.parse_args()
    print(json.dumps(controls(args.capture,args.historical)),flush=True)
    if args.controls_only:return
    from vision_resident_worker import ResidentRuntime,parser as worker_parser
    import yaml
    models=json.loads(args.models.read_text());worker=worker_parser().parse_args(['--upstream-root',str(args.vision),'--output-root',str(args.capture/'sam-runs'),'--input-root',str(args.capture),'--sam-model',models['sam']['snapshot_path']])
    runtime=ResidentRuntime(worker);rows=[]
    config=yaml.safe_load((ROOT/'configs/isaac/perception_validation.yaml').read_text())
    for group in ('A','B'):
        manifest=IsaacSceneManifest.from_dict(json.loads((args.capture/group/'manifest.json').read_text()))
        for camera in manifest.cameras:
            module=camera['module_id'];target=folder(args.capture,group,module)
            source=args.historical/'FULL_STACK_NOMINAL'
            if module=='module_1_lower':source=source/'modules'/module
            proposals=json.loads((source/'oracle_proposals.json').read_text())
            binding=json.loads((target/'capture_metadata.json').read_text())
            proposals.update(simulation_epoch=binding['sensor_epoch'],frame_sequence=binding['frame_sequence'])
            (target/'oracle_proposals.json').write_text(json.dumps(proposals,indent=2))
            # No output reuse: ResidentRuntime creates each run directory exclusively,
            # hashes this RGB and sets a fresh image for SAM on every infer call.
            infer(runtime,target,binding,camera,'appearance-'+group+'-'+module)
            artifacts=_worker_artifacts(target)
            result=_run_secondary_module(scene='FULL_STACK_NOMINAL',module_dir=target,manifest=manifest,artifacts=artifacts,
                config=config,vision_root=args.vision,upstream_python=Path(sys.executable),timeout=1200)
            rows.append({'group':group,'module':module,'proposal_count':len(proposals['instances']),
                         'sam_count':len(result['observation'].cargo),'observed_faces':sum(len(f.faces) for f in result['observed_face_sets']),
                         'metric_seconds':result['elapsed_seconds'],'moge_imported':False,'planning_admissible':False,
                         'rgb_sha256':sha256(target/'sensor_rgb.png'),'worker_response':str(target/'mode_b1_worker_response.json')})
            (args.capture/'inference_summary.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows[-1]),flush=True)
    assert runtime.moge_model is None and not any(k=='moge' or k.startswith('moge.') for k in sys.modules)


if __name__=='__main__':main()
