"""Cross-check measured CAD dimensions against official S50 mechanical drawings."""
import hashlib, json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
ASSET=ROOT/"assets/robots/huayan_s50_2000"
cad=json.loads((ASSET/"derived/cad_audit.json").read_text("utf8"))
geometry=json.loads((ASSET/"derived/interface_measurements.json").read_text("utf8"))
parts={p["id"]:p for p in cad["parts"]}
axes={a["joint_label"]:a for a in geometry["axes"]}
p=lambda joint: np.array(axes[joint]["point_CAD_m"])
j2_tube=max(parts["j2_s50"]["cylindrical_faces"],key=lambda f:f["area_m2"])
j3_tube=max(parts["j3_s50"]["cylindrical_faces"],key=lambda f:f["area_m2"])
flange=geometry["flange_distal_face"]
mount_y=geometry["support_plane"]["y_CAD_m"]
base_bolts=[]
for cylinder in parts["base_s50"]["cylindrical_faces"]:
    center=np.array(cylinder["origin_m"])
    if abs(cylinder["radius_m"]-.00675)>1e-7 or abs(cylinder["direction"][1])<.999999:continue
    if abs(np.linalg.norm(center[[0,2]])-.151)>1e-7:continue
    if not any(np.linalg.norm(center[[0,2]]-np.array(f["origin_m"])[[0,2]])<1e-7 for f in base_bolts):base_bolts.append(cylinder)
measured=[
 ("installation plane to J2 axis",25,(p("J2")[1]-mount_y)*1000,283.3),
 ("J2 to J3 parallel-axis distance",25,(p("J3")[1]-p("J2")[1])*1000,900),
 ("J3 to J4 parallel-axis distance",25,(p("J4")[1]-p("J3")[1])*1000,941.5),
 ("J4 to J6 parallel-axis distance",25,(p("J6")[1]-p("J4")[1])*1000,158.5),
 ("J3 to J6 parallel-axis distance",25,(p("J6")[1]-p("J3")[1])*1000,1100),
 ("J2 to J6 straight-reference distance",25,(p("J6")[1]-p("J2")[1])*1000,2000),
 ("base axis to J2 tube centre offset",25,abs(j2_tube["origin_m"][0])*1000,334.5),
 ("J2 to J3 tube centre offset",25,abs(j2_tube["origin_m"][0]-j3_tube["origin_m"][0])*1000,239),
 ("J3 tube to J5 axis offset",25,abs(j3_tube["origin_m"][0]-p("J5")[0])*1000,158.5),
 ("J5 axis to distal flange plane",25,abs(p("J5")[0]-flange["origin_m"][0])*1000,134.5),
 ("base mounting hole diameter",55,base_bolts[0]["radius_m"]*2000,13.5),
 ("base mounting bolt circle diameter",55,np.linalg.norm(np.array(base_bolts[0]["origin_m"])[[0,2]])*2000,302),
 ("flange bolt circle diameter",56,geometry["flange_distal_face"]["bolt_circle_radii_m"][0]*2000,92),
]
checks=[dict(measurement=name,pdf_page_1based=page,measured_mm=float(value),drawing_mm=expected,
             absolute_error_mm=abs(float(value)-expected),tolerance_mm=.01,passed=abs(float(value)-expected)<=.01)
        for name,page,value,expected in measured]
pdf=ASSET/"source/S-series-mechanical.pdf"
report=dict(schema="s50_manual_cad_crosscheck_v1",source_step_sha256=cad["source_sha256"],
            manual_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(),
            dimension_checks=checks,
            base_mounting_bolt_cylinders=base_bolts,base_mounting_bolt_count=len(base_bolts),
            flange_bolt_count=len(geometry["flange_distal_face"]["bolt_cylinders"]),
            threaded_flange_holes=dict(manual="8-M6 depth 10mm",cad_representation="8 cylindrical tap/core bores diameter 5mm",not_nominal_fastener_diameter=True),
            geometry_matches_drawings=all(c["passed"] for c in checks) and len(base_bolts)==6,
            controller_zero_and_signs_verified=False,mount_clocking_verified=False,
            physical_tool_adapter_verified=False,
            disclaimer="Dimensional agreement does not identify controller zero offsets, positive directions, or a permitted speed revision.")
(ASSET/"derived/manual_cad_crosscheck.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
print(json.dumps({k:report[k] for k in ["dimension_checks","base_mounting_bolt_count","flange_bolt_count","geometry_matches_drawings"]},indent=2))
assert report["geometry_matches_drawings"]
