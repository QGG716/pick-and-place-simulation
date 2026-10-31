"""Compose the complete configured enclosure; no unbound warehouse geometry."""
import argparse
import os
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdLux,Gf
from unloading_sim.eco65.model import load_json,save_json
from unloading_sim.eco65.native_io import file_hash


def compose(workstation,output):
    source=Path(workstation).resolve();out=Path(output).resolve();out.mkdir(parents=True,exist_ok=False)
    manifest=load_json(source.with_name('workstation_manifest.json'))
    if manifest['environment']['schema']!='eco65_parametric_enclosure_v1':raise ValueError('PROTOTYPE_ENVIRONMENT_REQUIRED')
    if file_hash(source)!=manifest['workstation_sha256']:raise ValueError('WORKSTATION_CHANGED')
    stage=Usd.Stage.CreateNew(str(out/'eco65_lab_prototype.usda'))
    UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
    root=UsdGeom.Xform.Define(stage,'/World').GetPrim();stage.SetDefaultPrim(root)
    station=UsdGeom.Xform.Define(stage,'/World/Workstation')
    station.GetPrim().GetReferences().AddReference(Path(os.path.relpath(source,out)).as_posix(),'/Workstation')
    station.AddTransformOp().Set(Gf.Matrix4d(np.asarray(manifest['T_W_D']).T.tolist()))
    light=UsdLux.DomeLight.Define(stage,'/World/Lighting');light.CreateIntensityAttr(800.)
    stage.GetRootLayer().Save()
    save_json(out/'composition.json',dict(status='STATIC_COMPOSITION_NOT_EXECUTION',
        workstation_sha256=file_hash(source),source_snapshot_fingerprint=manifest['source_snapshot_fingerprint'],
        enclosure='FLOOR_TWO_WALLS_ROOF_CLOSED_END_INCLUDED',physical_execution='NOT_RUN'))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--workstation',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();compose(a.workstation,a.output)
