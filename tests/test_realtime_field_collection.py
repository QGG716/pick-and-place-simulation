"""November 17 review tools: command guards and option forwarding, fake IO only."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import importlib.util

ROOT=Path(__file__).resolve().parents[1]


def test_collection_blocks_all_motion_and_common_valve_entry_points_without_io():
    program='''
import runpy
m=runpy.run_path('tools/collect_realtime_field_records.py')
for cls,method in [('CollectionService','submit'),('CollectionVacuum','_sequence'),
                   ('CollectionVacuum','establish'),('CollectionVacuum','release'),
                   ('CollectionTaskManager','execute'),('CollectionWorkbench','execute_cycle')]:
    try:getattr(m[cls],method)(object(),None)
    except RuntimeError as error:assert 'REVIEW_COLLECTION_ONLY' in str(error)
    else:raise AssertionError(method)
m['CollectionService'].request_stop(object())
'''
    result=subprocess.run([sys.executable,'-c',program],cwd=ROOT,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr


def test_all_on_tool_honors_native_in_both_resident_histories(monkeypatch,tmp_path):
    path=ROOT/'tools/check_step4_mechanisms.py'
    spec=importlib.util.spec_from_file_location('mechanism_review',path);tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
    calls=[]
    class Planner:
        resident=SimpleNamespace(wait_ready=lambda *a:None)
        def prewarm(self):return SimpleNamespace(join=lambda:None)
        def close(self):pass
    monkeypatch.setattr(tool,'planner_for',lambda *args:(calls.append(args) or (Planner(),None)))
    monkeypatch.setattr(tool,'run',lambda *a:None)
    monkeypatch.setattr(sys,'argv',[str(path),'--mode','on','--planning-module','NATIVE',
        '--collision-check-mode','TWO_LEVEL','--sent-path-audit','ADAPTIVE','--output',str(tmp_path/'results')])
    tool.main()
    assert len(calls)==2
    assert all(call[0] is True and call[2:] == ('TWO_LEVEL','ADAPTIVE','NATIVE') for call in calls)
