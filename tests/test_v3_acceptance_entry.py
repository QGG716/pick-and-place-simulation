import runpy
import sys

from tools.run_m710id70_acceptance import main


def test_public_acceptance_entry_forwards_config_and_phase_without_dropping_parameters(monkeypatch):
    calls=[];before=sys.argv
    def fake_run(path,run_name):
        calls.append((path,run_name,sys.argv.copy()))
    monkeypatch.setattr(runpy,'run_path',fake_run)
    args=['--config','custom.yaml','--phase','continuous','--workers','1']
    assert main(args)==0
    assert calls[0][0].endswith('run_m710id70_v3.py')
    assert calls[0][2][1:]==args
    assert sys.argv is before
