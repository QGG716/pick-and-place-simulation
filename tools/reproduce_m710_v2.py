"""Replay immutable V2 sources and trace effective calls, not current imports."""
from __future__ import annotations

import argparse
from dataclasses import asdict,is_dataclass
import functools
import hashlib
import importlib.util
import inspect
import io
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import zipfile

import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
REFERENCE='1a9e7be084903b3d4d2471659b645e52a367ef8e'


def serial(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    if is_dataclass(value):return serial(asdict(value))
    if isinstance(value,dict):return {str(k):serial(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [serial(v) for v in value]
    if isinstance(value,(str,int,float,bool)) or value is None:return value
    if isinstance(value,Path):return str(value)
    if hasattr(value,'base_transform'):
        return {'class':type(value).__name__,'base_transform':value.base_transform.tolist(),
                'tool_length':value.tool_length,'tool_collision_size':serial(value.tool_collision_size),
                'joint_limits':value.joint_limits.tolist(),'urdf_path':str(value.urdf_path)}
    return repr(value)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs/m710id70_v3/v2_instrumented')
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    snapshot=out/'source_snapshot'
    archive=subprocess.check_output(['git','archive','--format=zip',REFERENCE],cwd=ROOT)
    # Read-only git archive; extraction is restricted to a fresh named output.
    if not snapshot.exists():
        snapshot.mkdir()
        with zipfile.ZipFile(io.BytesIO(archive)) as zipped:
            for member in zipped.infolist():
                if not (snapshot/member.filename).resolve().is_relative_to(snapshot):
                    raise ValueError('unsafe archive member')
            zipped.extractall(snapshot)
    else:
        # Never trust an existing, potentially modified snapshot.
        with zipfile.ZipFile(io.BytesIO(archive)) as zipped:
            for member in zipped.infolist():
                if member.is_dir():continue
                if (snapshot/member.filename).read_bytes()!=zipped.read(member):
                    raise ValueError(f'snapshot modified: {member.filename}; use a new output directory')
    sys.path.insert(0,str(snapshot/'src'))
    os.chdir(snapshot)
    spec=importlib.util.spec_from_file_location('frozen_v2_acceptance',snapshot/'tools/run_m710id70_acceptance.py')
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    counter=0
    with (out/'effective_calls_and_task_results.jsonl').open('w',encoding='utf-8') as trace:
        def instrument(name):
            function=getattr(module,name);signature=inspect.signature(function)
            @functools.wraps(function)
            def wrapped(*args,**kwargs):
                nonlocal counter
                bound=signature.bind(*args,**kwargs);bound.apply_defaults()
                result=function(*args,**kwargs);counter+=1
                trace.write(json.dumps({'call':counter,'function':name,'effective_arguments':serial(bound.arguments),'result':serial(result)},ensure_ascii=False)+'\n')
                trace.flush()
                return result
            setattr(module,name,wrapped)
        for name in ('build_robot','evaluate_task_point','evaluate_complete_task','_solve_handoff'):
            instrument(name)
        module.main(['--output-dir',str(out),'--grid-step','0.30'])
    documents={p.relative_to(snapshot).as_posix():yaml.safe_load(p.read_text(encoding='utf-8'))
               for folder in ('config','configs') for p in (snapshot/folder).rglob('*.yaml')}
    manifest={'reference_commit':REFERENCE,'snapshot_archive_sha256':hashlib.sha256(archive).hexdigest(),
              'python':sys.version,'numpy':np.__version__,'platform':platform.platform(),
              'command':'python tools/reproduce_m710_v2.py --output-dir '+str(out),'instrumented_calls':counter,
              'configuration_documents':documents,'trace':'effective_calls_and_task_results.jsonl'}
    (out/'source_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False),encoding='utf-8')
    original=out.parent/'v2_baseline/acceptance_summary.json'
    if original.exists():
        a=json.loads(original.read_text(encoding='utf-8'));b=json.loads((out/'acceptance_summary.json').read_text(encoding='utf-8'))
        equal=a==b
        (out/'baseline_equality.json').write_text(json.dumps({'exact_summary_equality':equal,'original':str(original)},indent=2),encoding='utf-8')
        if not equal:raise RuntimeError('instrumented frozen baseline differs; inspect evidence before comparing V3')


if __name__=='__main__':main()
