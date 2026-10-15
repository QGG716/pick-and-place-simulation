"""Frozen multi-capture experiment using the actual one-shot and resident runtime."""
import argparse
from contextlib import contextmanager
import builtins
import io
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.rgb_input import load_rgb_input, snapshot_rgb_input
from unloading_perception.proposals import DetectorConfig
from perception_sam_pipeline import write


@contextmanager
def forbid_truth_reads():
    """Instrument Python file opens in addition to physically isolated RGB inputs."""
    originals=(builtins.open,io.open)
    forbidden=('gt_annotations','gt_instance_masks','oracle_proposals')
    def guard(original):
        def checked(file,*args,**kwargs):
            if not isinstance(file,int) and any(s in str(file).lower() for s in forbidden):
                raise AssertionError('INFERENCE_TRUTH_ACCESS:'+str(file))
            return original(file,*args,**kwargs)
        return checked
    builtins.open,io.open=map(guard,originals)
    try:yield
    finally:builtins.open,io.open=originals


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True,help='JSON with groups [{name,capture,frame}] fixed before runtime')
    p.add_argument('--models',type=Path,required=True);p.add_argument('--vision',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--detector-config',type=Path,default=ROOT/'configs/perception/grounding_dino.json')
    p.add_argument('--evaluate-gt',action='store_true')
    a=p.parse_args(argv);experiment_started=perf_counter();a.output.mkdir(parents=True,exist_ok=False)
    plan=json.loads(a.plan.read_text(encoding='utf-8'));plan=plan.get('plan',plan)
    groups=plan['groups']; frozen=[]
    for g in groups:
        capture=Path(g['capture']); name=g['name']
        if Path(name).name!=name or name in ('.','..'):raise ValueError('unsafe group name')
        inputs=[]
        for source in sorted((capture/'FULL_STACK_NOMINAL/modules').iterdir()):
            rgb=load_rgb_input(source,expected_module_id=source.name,expected_frame=g['frame'])
            expected=g.get('verified_inputs',{}).get(source.name,{}).get('original_binding',{}).get('rgb_sha256')
            if expected is not None and rgb.rgb_sha256!=expected:raise ValueError('FROZEN_PLAN_RGB_MISMATCH')
            isolated=a.output/'rgb-only'/name/'FULL_STACK_NOMINAL/modules'/source.name
            snapshot_rgb_input(rgb,isolated)
            inputs.append(dict(source=str(source),isolated=str(isolated),rgb_sha256=rgb.rgb_sha256,
                               module_id=source.name,capture_id=rgb.metadata.capture_id,source_size=list(rgb.rgb.shape[1::-1])))
        if not inputs:raise ValueError('no declared raw inputs')
        frozen.append(dict(name=name,frame=g['frame'],inputs=inputs))
    write(a.output/'frozen_plan.json',dict(groups=frozen,models=json.loads(a.models.read_text()),
        config=DetectorConfig(**json.loads(a.detector_config.read_text())).to_dict(),
        code_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                     for directory in ('tools','src/unloading_perception')
                     for p in sorted((ROOT/directory).glob('*.py'))},
        policy='all declared inputs, no tuning; original RGB hashes verified; no GT input files'))
    from workcell_video_worker import RuntimeCache
    from run_workcell_perception_once import main as run_capture
    cache=RuntimeCache();results=[]
    for group in frozen:
        output=a.output/group['name'];started=perf_counter()
        with forbid_truth_reads():
            code=run_capture(['--capture',str(a.output/'rgb-only'/group['name']), '--output-directory',str(output),
                '--vision',str(a.vision),'--models',str(a.models),'--detector-config',str(a.detector_config),'--stop-after','sam'],runtime_factory=cache)
        summary=json.loads((output/'summary.json').read_text())
        results.append(dict(name=group['name'],exit_code=code,wall_seconds=perf_counter()-started,
                            summary=summary,truth_access_guard='PASSED',resident_loads=cache.loads))
        write(a.output/'experiment.json',dict(results=results))
    # Only after all four predictions are frozen may evaluation open GT.
    if a.evaluate_gt:
        from evaluate_detector_sam import evaluate
        from build_sam_review import build_review
        for group,result in zip(frozen,results):
            for source,row in zip(group['inputs'],result['summary']['runs']):
                if row['status']!='COMPLETED':continue
                evaluation_started=perf_counter()
                folder=a.output/group['name']/source['module_id']
                row['evaluation']=evaluate(Path(source['source']),folder,row['artifacts'])
                row['review']=build_review(folder,row['artifacts'])
                row['postfreeze_evaluation_review_seconds']=perf_counter()-evaluation_started
    write(a.output/'experiment.json',dict(results=results,model_reuse_count=cache.loads,
        detector_object_id=id(getattr(cache.runtime,'detector',None)),sam_object_id=id(getattr(cache.runtime,'sam_model',None)),
        no_gt_test='isolated four-file module directories plus open interception; actual entrypoint',
        experiment_wall_seconds=perf_counter()-experiment_started,
        geometry='NOT_RUN',ros='NOT_RUN',planning='NOT_RUN'))
    links=[]
    for g in frozen:
        for i in g['inputs']:
            link=f"{g['name']}/{i['module_id']}/sam_review.html"
            if (a.output/link).exists():
                links.append(f'<li><a href="{link}">{g["name"]} / {i["module_id"]}</a></li>')
            else:
                links.append(f'<li>{g["name"]} / {i["module_id"]}: TECHNICAL_FAILURE / see experiment.json</li>')
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><h1>Grounding DINO + SAM 全部输入评审</h1><p>黄框检测；绿线 SAM；白框可选；蓝色 GT 默认关闭。后续阶段 NOT_RUN，执行 BLOCKED。</p><ul>'+''.join(links)+'</ul>',encoding='utf-8')
    return int(any(r['exit_code'] for r in results))


if __name__=='__main__':raise SystemExit(main())
