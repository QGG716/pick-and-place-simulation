"""Write reproducible synchronized CPU/native geometry fixtures, no motion."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/"src"))
from unloading_sim.workcell_layout import load_workcell_layout,audit_layout_constraints
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.coupled_batch import coupled_binding
from unloading_sim.coupled_lift_checks import check_coupled_interval
from unloading_sim.moveit2_backend import box_message
from unloading_sim.top_layer_lifted import FrozenTopLayer,process_options


def main(out):
    out.mkdir(parents=True,exist_ok=False)
    layout=load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1.yaml")
    old=load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1_rear_fixed_20261019.yaml")
    cartons=layout.cartons();frozen=FrozenTopLayer.capture(cartons)
    belt=next(b for b in layout.fixed_components() if b.name=="conveyor_transverse")
    low=layout.data["conveyors"]["surface_z_a_m"]+layout.data["assembly"]["world_xyz_m"][2]
    heights=next(iter(frozen.evidence()["heights"].values()))
    middle,high=heights["Hmin_world_m"],heights["Hmax_world_m"]
    contacts=[["chassis","conveyor_transverse"],["chassis","conveyor_longitudinal"],["conveyor_transverse","conveyor_longitudinal"]]
    bindings=[];checks=[]
    for a,b in [(low,high),(high,low),(low,middle),(middle,low),(middle,high),(high,middle),(low,low),(middle,middle),(high,high)]:
        bindings.append(coupled_binding(layout,cartons,a,b,frozen.target_ids[0],contacts))
        check=check_coupled_interval(layout,a,b,designed_contacts=contacts)
        checks.append(check)
        print(json.dumps(dict(start=a,end=b,samples=check["sample_count"],status=check["status"])),flush=True)
    fixtures=dict(bindings=bindings,old_model=dict(height_m=low,world=[box_message(b) for b in coupled_conveyor_state(old,low,legacy_geometry=True).boxes]),releases=[])
    (out/"native-fixtures.json").write_text(json.dumps(fixtures,separators=(",",":")))
    report=dict(layout=audit_layout_constraints(layout),intervals=checks,process_options={b.name:process_options(b,belt,frozen,cartons) for b in cartons if b.name in frozen.target_ids},
        continuous_swept_proof=False,physical_execution="NOT_RUN",execution_ready=False,qualification_status="NOT_EVALUATED")
    (out/"checks.json").write_text(json.dumps(report,separators=(",",":")))
    assert all(all(s["passed"] for s in c["samples"]) for c in checks)

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--output",type=Path,required=True);main(p.parse_args().output)
