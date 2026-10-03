import sys,json,hashlib
from pathlib import Path
D=Path('/work-static-prior-20261003'); B=Path('/work-planning-opt-20261003-source'); A=Path('/tmp/static-prior-20261003')
sys.path.insert(0,str(D/'src'))
from unloading_sim.layout_single_carton import load_layout_motion_policy,build_verified_motion_input,_build_automatic_trajectory_connector
from unloading_sim.moveit2_backend import model_request
policy=load_layout_motion_policy(D/'configs/validation/m710id70_proof_of_concept.yaml')
scene=build_verified_motion_input(policy,D)
c=_build_automatic_trajectory_connector(scene,policy.layout_validation.layout.robot()).connector
c.planning_only=True
actual=model_request(c,scene)[0]['identity']['model_tool_fingerprint']
normalized=model_request(c,scene,asset_root=B)[0]['identity']['model_tool_fingerprint']
baseline=json.load(open(A/'baseline-carton_l07_c03/timing.json'))['model_identity']['model_tool_fingerprint']
files=[]
for prefix in ['assets/robots','assets/grippers','configs','config/robot']:
 for p in (D/prefix).rglob('*'):
  if p.is_file():
   rel=p.relative_to(D); old=B/rel
   files.append(dict(path=str(rel),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),baseline_sha256=hashlib.sha256(old.read_bytes()).hexdigest() if old.exists() else None))
result=dict(schema='planning_geometry_comparison_v1',new_model_fingerprint=actual,baseline_model_fingerprint=baseline,new_with_baseline_asset_uris=normalized,normalized_match=normalized==baseline,file_count=len(files),different_files=[x for x in files if x['sha256']!=x['baseline_sha256']],files=files,scope='Read-only construction; no IK or planning. Absolute URDF mesh URI prefixes account for different model digests.')
(A/'geometry-identity.json').write_text(json.dumps(result,indent=2)+chr(10))
print({k:v for k,v in result.items() if k!='files'})
