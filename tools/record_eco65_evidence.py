"""Record local-only hashes and numeric metrics after one successful ECO65 run."""
from pathlib import Path
import hashlib,json,subprocess,sys,platform,datetime,importlib.metadata,shutil
import numpy as np
from unloading_sim.eco65.model import ROOT,LOCAL,OUTPUT,OFFICIAL,load_json,save_json,asset_check
from unloading_sim.eco65.observation_contracts import canonical_fingerprint

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def record():
    validation=load_json(OUTPUT/'reports/validation.json');states=load_json(OUTPUT/'replay/states.json')
    assert validation['status']=='VALID' and states['geometric_tasks_completed']==1
    frozen=load_json(OUTPUT/'reports/frozen_run_inputs.json')
    for name,value in frozen.items():assert sha(ROOT/name)==value,('Run input changed',name)
    official=load_json(OFFICIAL/'SOURCE_MANIFEST.json')
    for entry in official['entries']:
        assert sha(OFFICIAL/entry['local_path'])==entry['sha256'],entry['local_path']
    cad=asset_check();plan=load_json(OUTPUT/'trajectory/plan.json');snap=load_json(OUTPUT/'scene_snapshot.json')
    sources=list((ROOT/'src/unloading_sim/eco65').glob('*.py'))
    sources += [ROOT/'src/unloading_sim'/n for n in ['robot.py','geometry.py','ik.py','planner.py','timing.py']]
    sources += list(ROOT.glob('tools/*desktop_suction.py'))+[Path(__file__).resolve(),ROOT/'tools/run_eco65_desktop.py',ROOT/'tests/test_eco65_round1.py']
    sources += list(ROOT.glob('requirements/eco65-*.txt'))+list(ROOT.glob('configs/integration/eco65*'))
    code={str(p.relative_to(ROOT)):sha(p) for p in sorted(set(sources))}
    geometry=dict(frozen)
    geometry[cad['source']]=cad['source_sha256']
    geometry['assets/robots/realman_eco65/official/SOURCE_MANIFEST.json']=sha(OFFICIAL/'SOURCE_MANIFEST.json')
    reports=OUTPUT/'reports'
    for name in ['round1-plan','round1-tests','final-task-test','check','assembly','usd-export','replay']:
        src=ROOT/('.tmp-eco65-'+name+'.log')
        if src.exists():shutil.copy2(src,reports/(name+'.log'))
    metrics=[]
    for seg in plan['segments']:
        q=np.array(seg['q']);assert seg['timing']['within_limits']
        metrics.append(dict(phase=seg['phase'],waypoints=len(q),duration_s=seg['duration_s'],joint_path_L2_rad=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),search=seg['search'],ik=seg['ik']))
    packages={n:importlib.metadata.version(n) for n in ['numpy','scipy','trimesh','mujoco','cadquery-ocp-novtk','imageio','imageio-ffmpeg','pillow','usd-core']}
    evidence=dict(recorded_local_time=datetime.datetime.now().astimezone().isoformat(),
        branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True).strip(),
        base_commit_before_delivery=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        code_file_sha256=code,code_tree_sha256=canonical_fingerprint(code),asset_file_sha256=geometry,asset_tree_sha256=canonical_fingerprint(geometry),
        official_files_verified=len(official['entries']),packages=packages,python=sys.version,platform=platform.platform(),
        seed=snap['scene']['seed'],plan_fingerprint=plan['trajectory_fingerprint'],world_fingerprint=snap['world_fingerprint'],
        simulation_duration_s=states['duration_s'],validated_states=validation['samples'],segments=metrics,
        post_run_source_changes=['Sampling comment corrected: uniform-time quintic spacing can reach 1.875x nominal joint subdivision.',
            'Execution deterministic-step capability corrected to false; no physics step API is implemented.',
            'Added matching trajectory/world fingerprints to output records after checking all frozen run inputs unchanged; no trajectory or collision algorithm change.',
            'Delivery CLI, exact-source checks, framing fix and USD export exercised separately.'],
        original_run_code_hashes=load_json(reports/'run_source_before_delivery.json'),
        execution='GEOMETRIC_REPLAY_ONLY',dynamics='NOT_EVALUATED',hardware_connected=False)
    save_json(reports/'evidence.json',evidence)
    print(json.dumps({k:evidence[k] for k in ['code_tree_sha256','asset_tree_sha256','official_files_verified','simulation_duration_s','validated_states']},indent=2))
    return evidence

if __name__=='__main__':record()