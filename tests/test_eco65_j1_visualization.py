"""Directed optical-orbit and replay timing regressions only."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from eco65_j1_visualization import face_angle, schedule


def test_off_axis_optical_centre_changes_face_angle():
    class Robot:
        def named_link_frames(self,q):
            c,s=np.cos(q[0]),np.sin(q[0]);T=np.eye(4)
            T[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
            return {'link_1':T}
    X=np.eye(4);X[:3,:3]=[[0,0,1],[1,0,0],[0,1,0]];X[:3,3]=[0,.2,1]
    face,away,_=face_angle(Robot(),np.zeros(6),X,[1,0,0],[-3.1,3.1])
    assert abs(face+np.arcsin(.2))<1e-12
    assert abs(away-face-np.pi/2)<1e-12


def test_four_holds_return_to_same_endpoints_and_obey_slow_limit():
    rows,timing=schedule(-1.4,-1.4+np.pi/2,velocity=.2)
    assert timing['peak_velocity_rad_s']<=.2
    assert timing['duration_s']>26
    assert abs(len(rows)/5-timing['duration_s'])<1e-12
    holds={phase:{r['q1'] for r in rows if r['phase']==phase} for phase in ['face_1','face_2','away_1','away_2']}
    assert holds['face_1']==holds['face_2']=={-1.4}
    assert holds['away_1']==holds['away_2']=={-1.4+np.pi/2}
    assert all(abs(r['replay_time_s']-i/5)<1e-12 for i,r in enumerate(rows))
