"""Generate a coupled-scissor preview from one authoritative geometry model."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.workcell_layout import load_workcell_layout, sha256_file


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--source-commit",required=True)
    parser.add_argument("--usd-asset-cache",type=Path)
    args=parser.parse_args()
    if len(args.source_commit)!=40 or any(c not in "0123456789abcdef" for c in args.source_commit):
        raise ValueError("FULL_SOURCE_COMMIT_REQUIRED")
    args.output.mkdir(parents=True,exist_ok=False)
    layout=load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1.yaml")
    transverse=next(b for b in layout.fixed_components() if b.name=="conveyor_transverse")
    low=float(transverse.center[2]+transverse.half_extents[2])
    high=max(float(b.corners()[:,2].min()) for b in layout.cartons())-.05
    # Parameter samples, NOT seconds or claimed sweep collision verification.
    states=[coupled_conveyor_state(layout,h) for h in np.linspace(low,high,101)]
    evidence=[s.evidence() for s in states]
    result=dict(status=evidence[0]["status"],source_commit=args.source_commit,
        layout_fingerprint=layout.layout_fingerprint,command=sys.argv,
        qualification_status="NOT_EVALUATED",physical_execution="NOT_RUN",continuous_swept_proof=False,
        sample_parameter="HEIGHT_NOT_TIME",states=evidence,
        official_resources=dict(conveyor_manifest="configs/isaac/conveyor_assets.json",
          articulations="https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/rigid_bodies_articulations/articulations.html",
          conveyor="https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.gen.conveyor/docs/index.html"))
    if args.usd_asset_cache:
        sys.path.insert(0,str(ROOT/"scripts"))
        from coupled_conveyor_usd import create_stage, apply_state
        from pxr import Usd, UsdGeom
        config=json.loads((ROOT/"configs/isaac/conveyor_assets.json").read_text(encoding="utf-8"))
        stage,assets=create_stage(states[0],args.output/"coupled_conveyors.usda",
                                  asset_config=config,asset_cache=args.usd_asset_cache)
        # Real USD round trip over every pose. No SimulationApp / physics step.
        for state in states:
            apply_state(stage,state)
            cache=UsdGeom.XformCache()
            for box in state.boxes:
                actual=cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/CoupledConveyors/Bodies/"+box.name))
                if not np.allclose(np.asarray(actual).T,box.world_from_local,atol=1e-12,rtol=0):
                    raise ValueError("USD_GEOMETRY_DIVERGED")
        stage.GetRootLayer().Save()
        reopened=Usd.Stage.Open(str(args.output/"coupled_conveyors.usda"))
        if reopened.GetDefaultPrim().GetCustomDataByKey("scene_fingerprint")!=states[-1].scene_fingerprint:
            raise ValueError("USD_RELOAD_IDENTITY_MISMATCH")
        result["usd_validation"]=dict(status="PASS_GEOMETRY_ONLY",states=101,
                                      assets=assets,physics_steps=0)
    template=(ROOT/"tools/lifted_reception/viewer.html").read_text(encoding="utf-8")
    extension=(ROOT/"tools/lifted_reception/coupled_view.js").read_text(encoding="utf-8")
    vendor=ROOT/"tools/search_replay/vendor"
    scripts=(vendor/"three.min.js").read_text()+"\n"+(vendor/"OrbitControls.js").read_text()
    # Reuse the official robot geometry exporter at the declared configuration q.
    # No IK or collision acceptance is inferred from this static reference.
    import xml.etree.ElementTree as ET
    import yaml
    from export_m710_search_replay import export_model
    description=ET.parse(ROOT/"assets/robots/fanuc_m710id_70/official/fanuc_m710_description/urdf/m710id_70_official.urdf").getroot()
    for mesh in description.iter("mesh"):
        mesh.set("filename",str(ROOT/"assets/robots/fanuc_m710id_70/official"/mesh.get("filename").removeprefix("package://")))
    for joint in description.findall("joint"):
        if joint.find("parent").get("link")=="world" and joint.find("child").get("link")=="base_link":
            joint.find("origin").set("xyz"," ".join(map(str,layout.robot_base_transform()[:3,3])))
            joint.find("origin").set("rpy","0 0 0")
    q=yaml.safe_load((ROOT/"configs/validation/m710id70_layout_poc_pair_clearance.yaml").read_text(encoding="utf-8"))["initial_state"]["q_rad"]
    result["reference_q_rad"]=q
    result["robot_reference_status"]="STATIC_CONFIG_Q_NOT_CHECKED_WITH_NEW_MECHANISM"
    model=export_model(dict(parameters=dict(robot_description=ET.tostring(description,encoding="unicode")),
        tool_links=[],joint_names=[j.get("name") for j in description.findall("joint") if j.get("type")=="revolute"],
        identity=layout.assets,
        process_geometry=dict(flange_from_physical_contact=None)))
    data=dict(result=result,lift=True,model=model)
    (args.output/"geometry.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    html=template.replace("/*__VENDOR__*/",scripts).replace("/*__COUPLED__*/",extension).replace(
        "/*__DATA__*/",json.dumps(data,ensure_ascii=False,separators=(",",":")).replace("</","<\\/"))
    (args.output/"preview.html").write_text(html,encoding="utf-8")
    summary={k:v for k,v in result.items() if k!="states"}
    summary.update(low=states[0].mechanism,high=states[-1].mechanism,
        source_file_sha256={f:sha256_file(ROOT/f) for f in (
          "src/unloading_sim/conveyor_lift.py","scripts/coupled_conveyor_usd.py",
          "tools/preview_m710_conveyor_lift.py","tools/lifted_reception/coupled_view.js",
          "tools/lifted_reception/viewer.html")})
    (args.output/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(dict(states=101,low=low,high=high,tilt_degrees=float(np.rad2deg(states[-1].mechanism["tilt_rad"])),
                         near_tip_world=states[-1].mechanism["near_tip_world"],usd=result.get("usd_validation",{}).get("status"))))


if __name__=="__main__":
    main()
