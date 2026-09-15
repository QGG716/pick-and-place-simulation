"""Create isolated two-box fixtures without modifying shared mechanical layout."""
from copy import deepcopy
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.isaac_validation import canonical_digest, IsaacSceneManifest


def main():
    p=argparse.ArgumentParser(); p.add_argument('--source-bundle',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    index=json.loads((args.source_bundle/'index.json').read_text())
    base=json.loads((args.source_bundle/'RGBD_CALIBRATION_BOX.manifest.json').read_text())
    source_record=next(r for r in index['scenes'] if r['scene']=='RGBD_CALIBRATION_BOX')
    layout=json.loads((ROOT/'integration/isaac_scene_contract/m710id70_layout_v1/isaac_layout_contract.json').read_text())
    omissions=[v['name'] for v in layout['primitives'] if v['name']=='chassis' or 'conveyor' in v['name']]
    records=[]
    for sequence, name in enumerate(('GEOMETRY_ADJACENT_TWO','GEOMETRY_PARTIAL_TWO'),start=301):
        m=deepcopy(base); target=next(o for o in m['objects'] if o['visible']); second=next(o for o in m['objects'] if o['simulation_object_id']=='carton_l07_c01')
        T=np.asarray(target['T_W_object']); other=T.copy()
        if name=='GEOMETRY_ADJACENT_TWO':
            other[:3,3]+=T[:3,0]*.6
        else:
            other[:3,3]+=T[:3,:3]@np.array([-.38,.24,0.])
        second.update(T_W_object=other.tolist(),visible=True,occluded=False,state='ALGORITHM_DIAGNOSTIC_SECOND_BOX')
        m['run_id']=name.lower()+'-20260915'; m['timing'].update(simulation_epoch='metric-algorithm-only-20260915',simulation_frame=sequence,simulation_time=sequence/30)
        m['provenance'].update(validation_scope='GEOMETRY_ALGORITHM_ONLY',source_manifest_fingerprint=base['manifest_fingerprint'],
                               omitted_mechanical_entities=omissions,camera_geometry_unchanged=True,
                               full_mechanical_visibility_acceptance='NOT_CLAIMED',fixture_scene=name)
        m['dynamic_scene_fingerprint']=canonical_digest({'objects':m['objects'],'mechanisms':m['mechanisms'],
            'camera_calibration':[{k:c[k] for k in ('camera_id','frame_id','resolution','K','distortion_model','distortion','T_W_C','near_clip_m','far_clip_m')} for c in m['cameras']]})
        m['world_fingerprint']=canonical_digest({'layout_fingerprint':m['layout']['layout_fingerprint'],'dynamic_scene_fingerprint':m['dynamic_scene_fingerprint'],'robot':m['robot']})
        m.pop('manifest_fingerprint'); m['manifest_fingerprint']=canonical_digest(m)
        IsaacSceneManifest.from_dict(m)
        filename=name+'.manifest.json'; (args.output/filename).write_text(json.dumps(m,indent=2))
        records.append({**source_record,'scene':name,'path':filename,'manifest_fingerprint':m['manifest_fingerprint'],
                        'dynamic_scene_fingerprint':m['dynamic_scene_fingerprint'],'world_fingerprint':m['world_fingerprint'],
                        'simulation_epoch':m['timing']['simulation_epoch'],'simulation_frame':sequence})
    (args.output/'index.json').write_text(json.dumps({**index,'scenes':records,'validation_scope':'GEOMETRY_ALGORITHM_ONLY'},indent=2))


if __name__=='__main__': main()
