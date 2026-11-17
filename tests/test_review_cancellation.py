import pytest
from types import SimpleNamespace
from unloading_sim.workbench.single_carton.native_flow import NativeFlow
from unloading_sim.moveit2_backend import MoveItUnavailable

def test_screen_cancelled_before_template_work(tmp_path):
    p=SimpleNamespace(client=object(),clearance={},detach_supports=lambda *a:[],deadline=None)
    with pytest.raises(MoveItUnavailable,match='CANCELLED'):
        NativeFlow(p).screen([],[],{'id':'box'},[{}],[0],tmp_path,lambda:True)

def test_screen_passes_cancel_to_composite(tmp_path,monkeypatch):
    flag=lambda:False
    p=SimpleNamespace(client=object(),deadline=None)
    f=NativeFlow(p);seen=[]
    def templates(*args):
        assert args[-1] is flag
        return {'op':'sc_endpoint_screen'}
    def send(*args):
        seen.append(args[-1]);return {}
    monkeypatch.setattr(f,'screen_request',templates);monkeypatch.setattr(f,'send',send)
    f.screen([],[],{'id':'box'},[],[],tmp_path,flag)
    assert seen==[flag]
