"""Deterministic home validity diagnostic, never implicitly applied by planner."""
from pathlib import Path
import json
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
sys.path.insert(0,str(ROOT))
import numpy as np
from unloading_sim.validation_config import load_validation_config
from unloading_sim.validation_motion import Cell
from tools.run_m710id70_acceptance import _scene_regular


def main():
    cfg=load_validation_config();cell=Cell(cfg)
    original=np.array([-.17301878,-1.13655578,-.74874837,.63151726,-2.12393931,-2.83276516])
    obstacles=[*cell.fixtures(),*cell.decks((0,.2)),*_scene_regular()]
    rng=np.random.default_rng(71070);valid=[]
    for index in range(3000):
        q=rng.uniform(cell.robot.joint_limits[:,0]+.03,cell.robot.joint_limits[:,1]-.03)
        if cell.state_failure(q,obstacles) is None:
            valid.append({'index':index,'q':q.tolist(),'distance_from_invalid_v2_home_rad':float(np.linalg.norm(q-original))})
    valid.sort(key=lambda row:row['distance_from_invalid_v2_home_rad'])
    result={'seed':71070,'samples':3000,'original_home':original.tolist(),
            'original_failure':cell.state_failure(original,obstacles),'valid_count':len(valid),'nearest_valid':valid[:5],
            'scope':'offline initialization only; no motion from the colliding V2 pose is accepted'}
    output=ROOT/'outputs/m710id70_v3/home_probe.json';output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
