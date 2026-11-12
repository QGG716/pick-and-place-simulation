"""Measure component envelopes and belt distances on recorded cycle samples."""
from pathlib import Path
import sys,json,hashlib,copy,time,argparse
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from unloading_sim.s50_kinematic_sim import make_context,save
from unloading_sim.s50_sim_collision import SimCollision
from unloading_sim.s50_compactness import summarize_samples


def measure(report,output):
    report=Path(report);output=Path(output);source=report.read_bytes();data=json.loads(source)
    c=make_context(ROOT,output.parent/'measurement-model');checker=SimCollision(c)
    if data['source_model_sha256']!=c.definition['definition_sha256'] or data['environment_fingerprint']!=c.frozen['environment_fingerprint']:
        raise ValueError('MEASUREMENT_SCENE_CHANGED')
    start=time.perf_counter();cache={};rows=[]
    for i,s in enumerate(data['samples']):
        key=(*s['q_rad'],s['belt_height_m'])
        if key not in cache:cache[key]=checker.compactness(s['q_rad'],s['belt_height_m'],s['phase'])
        row=copy.deepcopy(cache[key]);row['pose_id']=f'sample_{i}';row['phase']=s['phase'];row['time_s']=s['time_s']
        rows.append(row)
        if i%200==0:print('measured',i,'/',len(data['samples']),flush=True)
    result=summarize_samples(rows)
    result.update(source_report_sha256=hashlib.sha256(source).hexdigest(),
        unique_configurations=len(cache),wall_seconds=time.perf_counter()-start,
        source='ORIGINAL_CAD_MESHES_AND_LOCAL_PATCHES_AGAINST_REAL_TRANSVERSE_CONVEX_SOLID')
    save(output,result)
    closest=min(range(len(rows)),key=lambda i:rows[i]['groups']['active_links']['minimum_unsigned_separation_m'])
    save(output.parent/'closest-belt-pose.json',dict(sample_index=closest,
        source_report_sha256=hashlib.sha256(source).hexdigest(),compactness=rows[closest]))
    print('saved',output,flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();measure(a.report,a.output)
