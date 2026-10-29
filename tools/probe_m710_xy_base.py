"""Check an explicit unloaded base translation from the frozen accepted prefix."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tools')]
import yaml
from plan_m710_joint_search import prepare_layout,restore_state,save
from unloading_sim.joint_search_layout import LayoutCandidate,BASELINE
from unloading_sim.xy_base_continuation import xy_candidate
from unloading_sim.xy_base_native import check_xy_relocation

def main():
    p=argparse.ArgumentParser();p.add_argument('--prefix',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--xy',type=float,nargs=2,required=True);p.add_argument('--seed',type=int,default=71070)
    p.add_argument('--initial-draws',type=int,default=64);p.add_argument('--worker-command',nargs='+',required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    db=json.loads((args.prefix/'checkpoint.json').read_text());record=db['layouts'][db['best']];e=record['candidate']
    source=LayoutCandidate(*e['base_xyz_a_m'],e['minimum_surface_height_m'],e['maximum_surface_height_m'],e['mechanism_profile'])
    context,evidence=prepare_layout(args,source);save(args.output/'source-context.json',evidence);assert context
    state=restore_state(record['nodes'][record['best_node']],context[2]);assert len(state.ledger)==31
    baseline=yaml.safe_load((ROOT/'configs/workcells'/BASELINE).read_text());destination=xy_candidate(source,args.xy,baseline)
    result=check_xy_relocation(args,context,state,source,destination,baseline,args.output/'motion')
    print(json.dumps(dict(passed=result['passed'],samples=len(result['samples']),failure=result['failure'],wall_s=result['wall_s'])),flush=True)
if __name__=='__main__':main()
