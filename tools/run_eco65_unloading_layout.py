"""Build/check/render one private desktop layout candidate without running unloading."""
import argparse,hashlib,json,subprocess
from pathlib import Path
import yaml
from unloading_sim.eco65.model import ROOT,URDF,load_json,save_json,asset_check,canonical_fingerprint
from unloading_sim.eco65.unloading_layout import CONFIG,build_scene,audit_layout,UnloadingWorld,limited_reachability,source_mapping,find_initial_pose,audit_mount

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['build','home','poses','render','export','verify'])
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--config',type=Path,default=CONFIG)
    args=parser.parse_args()
    config_path=args.config.resolve()
    if not args.run_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in args.run_id):parser.error('run-id must be a simple directory name')
    out=ROOT/'outputs/eco65_desktop_layout'/args.run_id
    if args.stage=='build':
        if (out/'scene_snapshot.json').exists():raise FileExistsError('Use a new run-id; existing layout evidence is not overwritten')
        c=yaml.safe_load(config_path.read_text(encoding='utf-8'));cad=asset_check();tool=load_json(ROOT/c['robot']['private_tool_config']);scene=build_scene(c)
        assert tool['source_sha256']==cad['source_sha256']
        paths={URDF,ROOT/cad['source'],ROOT/c['robot']['private_tool_config']}
        paths.update(ROOT/p['visual'] for p in tool['visual_parts']);paths.update(ROOT/p['mesh'] for p in tool['collisions'])
        for file in URDF.parents[1].rglob('*.STL'):paths.add(file)
        for file in URDF.parents[1].rglob('*.stl'):paths.add(file)
        fingerprints={str(p.relative_to(ROOT)):digest(p) for p in sorted(paths)}
        snapshot=dict(scene=scene,tool=tool,config_sha256=digest(config_path),asset_sha256=fingerprints)
        snapshot['fingerprint']=canonical_fingerprint(snapshot)
        save_json(out/'scene_snapshot.json',snapshot)
        audit=audit_layout(scene,tool);audit['scene_fingerprint']=snapshot['fingerprint'];save_json(out/'reports/layout_audit.json',audit)
        save_json(out/'dimension_mapping.json',source_mapping(c,scene))
        save_json(out/'regions.json',dict(scene_fingerprint=snapshot['fingerprint'],regions=audit['functional_regions']))
        previous=ROOT/'outputs/eco65_desktop_round1/round1'
        save_json(out/'reports/previous_round_preserved.json',{str(p.relative_to(ROOT)):digest(p) for p in previous.rglob('*') if p.is_file()})
        historical=ROOT/'outputs/eco65_desktop_layout/layout_v1_20260917_candidate02'
        save_json(out/'reports/previous_nine_preserved.json',{str(p.relative_to(ROOT)):digest(p) for p in historical.rglob('*') if p.is_file()})
        save_json(out/'reports/build_provenance.json',dict(base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),run_id=args.run_id,private=True,source_sha=c['reference']['sha']))
        print(json.dumps(dict(output=str(out),audit=audit['status'],errors=audit['errors'],faces=audit['face_compatibility'],footprint=scene['footprint']),indent=2))
        if audit['status']!='PASS':raise RuntimeError('Static layout checks failed; evidence retained')
        return
    snapshot=load_json(out/'scene_snapshot.json');assert digest(config_path)==snapshot['config_sha256'],'Config differs from frozen run';claimed=snapshot.pop('fingerprint');assert claimed==canonical_fingerprint(snapshot);snapshot['fingerprint']=claimed
    for name,sha in snapshot['asset_sha256'].items():assert digest(ROOT/name)==sha,('Changed asset',name)
    if args.stage=='verify':
        for name,sha in load_json(out/'reports/previous_round_preserved.json').items():assert digest(ROOT/name)==sha,('Previous evidence modified',name)
        nine=out/'reports/previous_nine_preserved.json'
        if nine.exists():
            for name,sha in load_json(nine).items():assert digest(ROOT/name)==sha,('Nine-box evidence modified',name)
        audit=audit_layout(snapshot['scene'],snapshot['tool']);assert audit['status']=='PASS'
        print('Snapshot, private asset hashes, original round-1 outputs and static layout: PASS');return
    w=UnloadingWorld(snapshot['tool'],snapshot['scene'])
    try:
        if args.stage=='home':
            mount=audit_mount(w);save_json(out/'reports/base_footprint.json',mount)
            if not mount['covered']:raise ValueError('Configured mount does not cover official base mesh')
            path=out/'reports/poses.json'
            if path.exists():
                result=load_json(path);assert result['scene_fingerprint']==claimed and w.valid(result['home']['q'],'initial');print('Existing frozen HOME rechecked: PASS');return
            search=find_initial_pose(w);save_json(out/'reports/home_search.json',search)
            if search['home'] is None:raise RuntimeError('No HOME found within finite candidate search')
            save_json(path,dict(home=search['home'],candidates=[],scene_fingerprint=claimed,status='INITIAL_ONLY'));print('HOME PASS');return
        if args.stage=='poses':
            if (out/'reports/poses.json').exists():raise FileExistsError('Pose evidence exists; use another run-id for a revised design')
            result=limited_reachability(w);result['scene_fingerprint']=claimed;save_json(out/'reports/poses.json',result)
            print('RESULT',result['status'])
        else:
            result=load_json(out/'reports/poses.json');assert result['scene_fingerprint']==claimed
            from unloading_sim.eco65.layout_figures import render_figures,export_geometry
            if args.stage=='render':render_figures(w,snapshot,result,out)
            else:export_geometry(w,snapshot,result,out)
    finally:w.close()

if __name__=='__main__':main()