"""Verify each changed-mount native trial and retain every failed root."""
import argparse,json
from pathlib import Path
import xml.etree.ElementTree as ET
from verify_m710_top_layer_batch import verify
from unloading_sim.y_shift_retry import verify_mount_protocol,TARGETS
from unloading_sim.collision_policy import SimulationCollisionPolicy


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args()
    summary=json.loads((args.run/'summary.json').read_text());reports=[]
    assert summary['previous_population_count_unchanged']==24 and summary['independent_trials']
    checked=0
    for outcome in summary['outcomes']:
        folder=args.run/outcome['folder'];protocol=json.loads((folder/'protocol.json').read_text())
        verify_mount_protocol(protocol,Path(__file__).resolve().parents[1])
        assert protocol['authorized_mount_change']==summary['authorized_mount_change']
        assert outcome['target_id'] in TARGETS and outcome['case']==protocol['case']
        result=json.loads((folder/'seed-71070/result.json').read_text())
        assert result['completed_count']==outcome['completed_count'] and result['planning_wall_s']==outcome['planning_wall_s']
        policy=SimulationCollisionPolicy.from_mapping(protocol['collision_policy'])
        for attempt in result['attempts']:
            if 'folder' not in attempt:continue
            evidence=folder/'seed-71070'/attempt['folder']
            trace=json.loads((evidence/'search-trace.json').read_text())
            model=trace['model'];urdf=ET.fromstring(model['parameters']['robot_description'])
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
        report=verify(folder)
        (folder/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
        reports.append(dict(target_id=outcome['target_id'],case=outcome['case'],**report))
    for target in TARGETS:
        rows=[o for o in summary['outcomes'] if o['target_id']==target]
        assert rows and sum(o['completed_count'] for o in rows)<=1
        if any(o['completed_count'] for o in rows):assert rows[-1]['completed_count']==1
    result=dict(status='PASS_CHANGED_MOUNT_NATIVE_LINEAGE_AND_FAILURE_RETENTION',reports=reports,
        checked_policy_requests=checked,physical_execution='NOT_RUN',previous_population_count_unchanged=24)
    (args.run/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(status=result['status'],cases=len(reports),checked_policy_requests=checked)))


if __name__=='__main__':main()
