"""Verify ordered continuation, checked native edges and retained failures."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
from verify_m710_top_layer_batch import verify
from unloading_sim.collision_policy import SimulationCollisionPolicy

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
    summary=json.loads((args.run/'summary.json').read_text())
    predecessor=json.loads((args.run/'predecessor.json').read_text())
    order=['carton_l04_c02','carton_l04_c03'];root=predecessor['terminal'];reports=[];checked=0
    assert summary['initial']==root and summary['ordered_target_ids']==order and not summary['independent_trials']
    for index,outcome in enumerate(summary['outcomes']):
        assert outcome['target_id']==order[index]
        folder=args.run/outcome['folder'];protocol=json.loads((folder/'protocol.json').read_text())
        result=json.loads((folder/'seed-71070/result.json').read_text())
        assert protocol['initial_root']==result['initial']==root
        assert protocol['accepted_predecessor']==predecessor
        assert result['completed_count']==outcome['completed_count'] and result['planning_wall_s']==outcome['planning_wall_s']
        policy=SimulationCollisionPolicy.from_mapping(protocol['collision_policy'])
        for attempt in result['attempts']:
            if 'folder' not in attempt:continue
            evidence=folder/'seed-71070'/attempt['folder']
            trace=json.loads((evidence/'search-trace.json').read_text())
            urdf=ET.fromstring(trace['model']['parameters']['robot_description'])
            mount=next(j for j in urdf.findall('joint') if j.find('parent').get('link')=='world' and j.find('child').get('link')=='base_link')
            xyz=list(map(float,mount.find('origin').get('xyz').split()))
            assert max(abs(a-b) for a,b in zip(xyz,[-1.625,.15,.6]))<1e-12
            assert list(map(float,mount.find('origin').get('rpy').split()))==[0.,0.,0.]
            for item in trace['attempts']:
                req=item['request']
                if 'clearance_policy' not in req:continue
                cp=req['clearance_policy'];assert cp['source_policy']==protocol['collision_policy']
                assert req['planning_mode']=='cold_from_scratch' and req['planning_only'] and not req['pair_profile_enabled']
                expected=policy.tool_chassis_pairs(outcome['target_id'],cp['tool_links'])|policy.robot_mount_pairs()
                assert {tuple(p) for p in req['allowed_pairs'] if 'chassis' in p}==expected
                checked+=1
            calls=json.loads((evidence/'requests.json').read_text())
            assert not any('prior' in str(c['op']) or c.get('pipeline_id')=='static_prior' for c in calls)
        report=verify(folder);reports.append(dict(target_id=outcome['target_id'],**report))
        root=result['terminal']
        if not result['completed_count']:
            assert index==len(summary['outcomes'])-1 and root==result['initial']
            assert summary['unsearched']==[dict(target_id=n,reason='ORDERED_PREDECESSOR_NOT_COMPLETED') for n in order[index+1:]]
    assert root==summary['terminal']
    assert summary['completed_count']==sum(o['completed_count'] for o in summary['outcomes'])
    result=dict(status='PASS_ORDERED_NATIVE_CONTINUITY_AND_FAILURE_RETENTION',reports=reports,
        checked_policy_requests=checked,physical_execution='NOT_RUN',completed_count=summary['completed_count'])
    (args.run/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],completed=result['completed_count'],checked_policy_requests=checked)))

if __name__=='__main__':main()
