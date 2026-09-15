"""Analytic ray/box fixtures: GT generates/evaluates inputs, never fits them."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
from unloading_perception.metric_faces import extract_observation_labels, fit_metric_faces, MetricFitConfig
from unloading_perception.metric_support import backproject_pixels
from diagnose_metric_calibration import load_extractor,evaluate


def render(boxes,K,shape):
    yy,xx=np.indices(shape)
    rays=backproject_pixels(np.column_stack([xx.ravel(),yy.ravel()]),np.ones(xx.size),K)
    depth=np.full(len(rays),np.inf); identity=np.full(len(rays),-1,int)
    for i,box in enumerate(boxes):
        T=np.asarray(box['T_W_object']); dims=np.asarray(box['full_dimensions_m'])
        origin=-T[:3,3]@T[:3,:3]; direction=rays@T[:3,:3]
        with np.errstate(divide='ignore',invalid='ignore'):
            a=(-dims/2-origin)/direction; b=(dims/2-origin)/direction
        near=np.max(np.minimum(a,b),axis=1); far=np.min(np.maximum(a,b),axis=1)
        hit=(near>0)&(near<=far)&(near<depth)
        depth[hit]=near[hit]; identity[hit]=i
    return depth.reshape(shape),identity.reshape(shape)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--vision',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    from scipy.spatial.transform import Rotation
    R=Rotation.from_euler('xyz',[15,25,10],degrees=True).as_matrix()
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=[.1,.1,2.5]
    box={'T_W_object':T.tolist(),'full_dimensions_m':[.6,.4,.3]}
    K=np.array([[600.,0,239.5],[0,720.,199.5],[0,0,1.]])
    cfg=MetricFitConfig(positive_infinity_is_no_hit=True); extractor=load_extractor(args.vision)
    rows=[]
    for name,offset in [('SINGLE',None),('ADJACENT',[.6,0,0]),('PARTIAL',[-.38,.24,-.05])]:
        boxes=[box]
        if offset is not None:
            B=T.copy(); B[:3,3]+=R@offset; boxes.append({'T_W_object':B.tolist(),'full_dimensions_m':[.6,.4,.3]})
        depth,identities=render(boxes,K,(400,480))
        for i,g in enumerate(boxes):
            mask=identities==i
            labels,seeds,audit=extract_observation_labels(depth,mask,K,extractor,cfg)
            record=fit_metric_faces(depth,mask,K,labels,seeds,mask_id=i,config=cfg)
            metrics=evaluate(record,g,{'K':K.tolist(),'T_W_C':np.eye(4).tolist()})
            assert metrics['faces'], (name,i,record)
            assert all(f['plane_distance_m']<.005 and f['normal_error_degrees']<1 and f['boundary_outside_m']<.01 for f in metrics['faces']),metrics
            rows.append({'fixture':name,'instance':i,'evaluation':metrics})
    (args.output/'analytic_matrix.json').write_text(json.dumps(rows,indent=2))
    print(json.dumps(rows),flush=True)


if __name__=='__main__': main()
