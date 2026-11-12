"""Offline full-state viewer for the explicitly authorized kinematic cycle."""
from pathlib import Path
import sys,argparse,json,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'tools'))
import numpy as np
from export_s50_cad_motion_viewer import build_data
from export_s50_model_viewer import encode_payload
from unloading_sim.robot_replacement_layout import frozen_baseline_layout,_box_record
from unloading_sim.conveyor_lift import coupled_conveyor_state

def write_html_payload(output, html, encoded):
    """Keep exact data in bounded local scripts; file:// needs no fetch/CORS."""
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    files=[];tags=[]
    for i,start in enumerate(range(0,len(encoded),4_000_000)):
        path=output.with_name(f'{output.stem}.data-{i:02d}.js')
        text='window.S50_KINEMATIC_PACK=window.S50_KINEMATIC_PACK||[];\n'
        text+=f'window.S50_KINEMATIC_PACK[{i}]='+json.dumps(encoded[start:start+4_000_000])+';\n'
        path.write_text(text,encoding='utf8')
        files.append(dict(path=path.name,bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        tags.append(f'<script src="{path.name}"></script>')
    html=html.replace('/*__DATA__*/',"window.S50_KINEMATIC_PACK.join('')")
    html=html.replace('<script>', ''.join(tags)+'<script>',1)
    output.write_text(html,encoding='utf8')
    return files

def export(report,output):
    report=Path(report);cycle=json.loads(report.read_text(encoding='utf8'))
    if cycle.get('schema')!='s50_full_kinematic_cycle_v1' or not cycle.get('samples'):
        raise ValueError('ACTUAL_COMPUTED_CYCLE_SAMPLES_REQUIRED')
    data=build_data(ROOT)
    if data['model_definition']['definition_sha256']!=cycle['source_model_sha256']:raise ValueError('SIM_VIEWER_MODEL_CHANGED')
    if data['environment']['environment_fingerprint']!=cycle['environment_fingerprint']:raise ValueError('SIM_VIEWER_ENVIRONMENT_CHANGED')
    heights=sorted({s['belt_height_m'] for s in cycle['samples']});layout=frozen_baseline_layout(ROOT)
    data['sim_environment']=[dict(height_m=h,entities=[_box_record(b) for b in coupled_conveyor_state(layout,h,()).boxes]) for h in heights]
    data['sim']=cycle;data['sim_report_sha256']=hashlib.sha256(report.read_bytes()).hexdigest()
    for filename,key in [('compactness-path-summary.json','sim_path_measurements'),('closest-belt-pose.json','sim_closest_belt_pose')]:
        path=report.parent/filename
        if path.is_file():
            value=json.loads(path.read_text(encoding='utf8'))
            if value['source_report_sha256']!=data['sim_report_sha256']:
                data.setdefault('omitted_stale_measurements',[]).append(filename)
                continue # Optional measurements from a previous run never enter this replay.
            data[key]=value
    data['model_status']='USER_AUTHORIZED_KINEMATIC_SIM_NOT_MANUFACTURER_QUALIFICATION'
    data['full_scene_fingerprint']=cycle.get('simulation_scene_fingerprint')
    data['trajectory']=cycle['samples'];data['trajectory_status']=cycle['status']
    data['joint_controls_enabled']=False # Recorded motion only; CAD sliders remain in the original preview.
    data['playback_scope']='FULL_ROBOT_TOOL_CARTON_AND_COUPLED_CONVEYOR_STATE'
    vendor=ROOT/'tools/search_replay/vendor'
    scripts='\n'.join((vendor/f).read_text(encoding='utf8') for f in ['three.min.js','OrbitControls.js'])
    scripts+='\n'+(ROOT/'tools/s50_kinematics.js').read_text(encoding='utf8')
    html=(ROOT/'tools/s50_kinematic_viewer.html').read_text(encoding='utf8').replace('/*__VENDOR__*/',scripts)
    output=Path(output);payload_files=write_html_payload(output,html,encode_payload(data))
    checks=dict(status=cycle['status'],sample_count=len(cycle['samples']),model_fk_source='SAME_CAD_DEFINITION',
        source_report_sha256=data['sim_report_sha256'],html_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        full_task_state=True,robot_joint_count=6,carton_initial_count=40,tool_connected=True,browser_acceptance='PENDING',
        offline_payload_files=payload_files)
    output.with_suffix('.json').write_text(json.dumps(checks,indent=2)+'\n',encoding='utf8');print(json.dumps(checks))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();export(a.report,a.output)
