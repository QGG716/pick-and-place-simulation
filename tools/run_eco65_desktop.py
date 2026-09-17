"""Local ECO65-B commands; optional dependencies are loaded by the selected stage."""
from pathlib import Path
import argparse,ast,hashlib,importlib.util,json,subprocess,sys
from unloading_sim.eco65.model import ROOT,LOCAL,OUTPUT,URDF,World,load_json,save_json,asset_check,design_scene

def definitions(content):
    result={}
    for node in ast.parse(content).body:
        names=[node.name] if hasattr(node,"name") else [t.id for t in getattr(node,"targets",[]) if isinstance(t,ast.Name)]
        for name in names:result[name]=ast.dump(node,include_attributes=False)
    return result

def check():
    import yaml
    from unloading_sim.eco65.pipeline import numeric_checks
    lock=yaml.safe_load((ROOT/'configs/integration/eco65_desktop_sources.lock.yaml').read_text(encoding='utf-8'))
    refs={}
    for name,source in lock['sources'].items():
        actual=subprocess.check_output(['git','rev-parse',source['local_ref']],cwd=ROOT,text=True).strip()
        assert actual==source['sha'],(name,actual)
        refs[name]=actual
    ports=load_json(ROOT/'configs/integration/eco65_round1_ports.json')
    for port in ports:
        content=subprocess.check_output(['git','show',port['source_sha']+':'+port['source_path']],cwd=ROOT)
        assert hashlib.sha256(content).hexdigest()==port['source_sha256']
        original=definitions(content)
        local=definitions((ROOT/port['destination']).read_text(encoding='utf-8'))
        for name in port['definitions']:
            assert local[name]==original[name],(port['destination'],name)
    cad=asset_check();w=World(load_json(LOCAL/'tool.json'),load_json(LOCAL/'scene.json'))
    try:numeric=numeric_checks(w)
    finally:w.close()
    installed={name:importlib.util.find_spec(name) is not None for name in ('isaacsim','isaaclab')}
    result=dict(status='PASS',sources=refs,ported_definitions=sum(len(p['definitions']) for p in ports),
                cad_source_sha256=cad['source_sha256'],numeric=numeric,isaac_runtime_packages=installed)
    save_json(OUTPUT/'reports/check.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['check','prepare','plan','assembly','replay','export-isaac'])
    args=parser.parse_args()
    if args.stage=='prepare':
        from convert_desktop_suction import convert
        from build_desktop_suction import build
        convert();design_scene(build())
    elif args.stage=='check':check()
    elif args.stage=='plan':
        from unloading_sim.eco65.pipeline import plan_and_execute
        plan_and_execute()
    elif args.stage in ('assembly','replay'):
        from unloading_sim.eco65.visuals import assembly_views,replay_video
        snapshot=load_json(OUTPUT/'scene_snapshot.json');asset_check()
        assert snapshot['tool']==load_json(LOCAL/'tool.json'),'Tool changed since planning'
        assert snapshot['scene']==load_json(LOCAL/'scene.json'),'Scene changed since planning'
        w=World(snapshot['tool'],snapshot['scene'])
        try:
            if args.stage=='assembly':print('\n'.join(assembly_views(w,snapshot['home_q'])))
            else:
                validation=load_json(OUTPUT/'reports/validation.json')
                assert validation['status']=='VALID','No valid trajectory to replay'
                artifact=load_json(OUTPUT/'trajectory/plan.json');states=load_json(OUTPUT/'replay/states.json')
                from unloading_sim.eco65.observation_contracts import canonical_fingerprint
                expected=artifact.pop('trajectory_fingerprint');assert expected==canonical_fingerprint(artifact)
                artifact['trajectory_fingerprint']=expected
                assert validation['trajectory_fingerprint']==states['trajectory_fingerprint']==expected,'Stale result/trajectory'
                assert validation['world_fingerprint']==snapshot['world_fingerprint'],'Stale world validation'
                assert states['geometric_tasks_completed']==1
                print(replay_video(w,artifact,states))
        finally:w.close()
    else:
        from unloading_sim.eco65.usd_export import export_isaac
        export_isaac()

if __name__=='__main__':main()