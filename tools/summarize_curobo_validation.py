"""Join immutable CPU/native evidence without claiming geometric equivalence."""
import argparse,json
from pathlib import Path


def main():
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--baseline-root',required=True);a=p.parse_args()
 root=Path(a.root);old=Path(a.baseline_root)
 cpu=json.loads((root/'cpu_pairs.json').read_text());gpu=json.loads((root/'native_audit.json').read_text());rows=[]
 for i,c in enumerate(cpu['endpoints']):
  fixture='business' if i<2 else 'direct';idx=i%2;original='final_business' if i<2 else 'final_direct_gpu'
  before=gpu['baseline'][original]['sphere_diagnostics']['states'][idx];after=gpu['endpoints'][fixture]['focus_pairs'][idx]
  for cp in c['pairs']:
   pair=cp['pair'];match=next(x for x in after if x['pair']==pair)
   if pair[0]=='base_link':category='EXISTING_ASSEMBLY_PERMISSION_NOT_MAPPED';oldgap=next(x['sphere_surface_gap_m'] for x in before['environment'] if x['link']=='base_link' and x['object']=='chassis')
   elif pair[0]=='tool_rigid_0':
    category='SPHERE_PROTRUSION_AND_ZERO_SELF_GAP_MAPPING';oldgap=next(x['sphere_surface_gap_m'] for x in before['self_collision'] if set(x['links'])=={'tool_part_0','held_carton'})
   else:
    category='SPHERE_PROTRUSION' if any(x['link']=='held_carton' and x['object']==pair[1] for x in before['environment']) else 'NO_MISMATCH_FOR_THIS_PAIR_AT_THIS_ENDPOINT'
    oldgap=next((x['sphere_surface_gap_m'] for x in before['environment'] if x['link']=='held_carton' and x['object']==pair[1]),None)
   rows.append(dict(endpoint=c['name'],pair=pair,authority=cp,before_gpu_gap_m=oldgap,
     before_gap_scope='CPU analysis of actual GPU transformed sphere coordinates; null means not in old <5mm violations',
     after_gpu=match,native_endpoint_before=gpu['baseline'][original]['native_feasible'][idx],
     native_endpoint_after=gpu['endpoints'][fixture]['feasible'][idx],classification=category))
 invariants={}
 for fixture,original in [('business','final_business'),('direct','final_direct_gpu')]:
  b=json.loads((root/fixture/'request.json').read_text());o=json.loads((old/original/'request.json').read_text())
  different=[k for k in b.keys()|o.keys() if b.get(k)!=o.get(k)]
  assert not different,(fixture,different)
  invariants[fixture]=dict(entire_request_identical=True,q_start=b['q_start'],q_goal=b['q_goal'],resources=b['resources'],policy=b['collision_policy_fingerprint'])
 record=dict(pairs=rows,fixture_invariants=invariants,category_catalog={
 '1_ACTUALLY_ILLEGAL':'All five labelled CPU/native negative cases rejected',
 '2_ASSEMBLY_PERMISSION_NOT_MAPPED':'base_link/chassis exact installation',
 '3_SPHERE_PROTRUSION':'rigid tool/payload, payload/neighbor, payload/conveyor',
 '4_UNDERCOVER_OR_FILTER':'Old whole flexible-cup/payload exemption omitted its bounded compression constraint; robot sample coverage remains finite',
 '5_FRAME_OR_IDENTITY':'Not observed in fixed endpoints; FK separately checked',
 '6_THRESHOLD_MAPPING':'tool/payload were wrongly treated as zero-gap native robot-self pairs',
 '7_UNKNOWN':'Global robot sphere coverage and collision equivalence not proven'},
 geometry_source_mismatch='uncompressed cups vs existing nominal 10mm compression; exported authority geometry now matches')
 (root/'pair_diagnosis.json').write_text(json.dumps(record,indent=2,allow_nan=False))
 print('FIXED_REQUESTS_IDENTICAL_AND_DIAGNOSIS_WRITTEN')
if __name__=='__main__':main()
