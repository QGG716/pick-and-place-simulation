"""Measure unoriented interface axes from adjacent named STEP bodies.

These measurements are CAD evidence, not controller joint definitions.
"""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
FOLDER=ROOT/"assets/robots/huayan_s50_2000/derived"
audit=json.loads((FOLDER/"cad_audit.json").read_text("utf8"))
parts={part["id"]:part for part in audit["parts"]}
pairs=[("base_s50","j1_s50"),("j1_s50","j2_s50"),("j2_s50","j3_s50"),
       ("j3_s50","j4_s50"),("j4_s50","j5_s50"),("j5_s50","j6_s50")]
axes=[]
for number,(parent,child) in enumerate(pairs,1):
    threshold=.009 if number==3 else .02
    groups=[]
    for a in parts[parent]["cylindrical_faces"]:
        if a["radius_m"]<threshold:continue
        u=np.array(a["direction"]);p=np.array(a["origin_m"])
        for b in parts[child]["cylindrical_faces"]:
            if b["radius_m"]<threshold:continue
            v=np.array(b["direction"]);q=np.array(b["origin_m"])
            parallel_error=float(np.linalg.norm(np.cross(u,v)))
            offset=float(np.linalg.norm(np.cross(q-p,u)))
            if parallel_error>1e-5 or offset>1e-5:continue
            closest=p-u*np.dot(p,u)
            evidence=dict(parent_face_index=a["face_index"],child_face_index=b["face_index"],
                          parent_radius_m=a["radius_m"],child_radius_m=b["radius_m"],
                          axis_offset_m=offset,parallel_error=parallel_error)
            group=next((g for g in groups if np.linalg.norm(closest-np.array(g["point_CAD_m"]))<1e-5),None)
            if group is None:
                canonical=u.copy()
                if canonical[np.argmax(np.abs(canonical))]<0:canonical=-canonical
                group=dict(point_CAD_m=closest.tolist(),unoriented_axis_CAD=canonical.tolist(),evidence=[])
                groups.append(group)
            group["evidence"].append(evidence)
    if len(groups)!=1:raise ValueError(f"NONUNIQUE_INTERFACE_AXIS_J{number}:{len(groups)}")
    group=groups[0]
    axes.append(dict(joint_label=f"J{number}",parent_cad_body=parent,child_cad_body=child,
                     candidate_line_groups=len(groups),minimum_matching_radius_m=threshold,
                     controller_positive_sign=None,controller_zero_offset_rad=None,
                     **group))
base=parts["base_s50"]
base_planes=[f for f in base["planar_faces"] if f["normal"][1]<-.999999]
min_plane_y=min(f["origin_m"][1] for f in base_planes)
support_planes=[f for f in base_planes if abs(f["origin_m"][1]-min_plane_y)<1e-6]
flange=parts["j6_s50"]
flange_planes=[f for f in flange["planar_faces"] if f["normal"][0]<-.999999 and f["area_m2"]>.001]
distal=min(flange_planes,key=lambda f:f["origin_m"][0])
bolts=[f for f in flange["cylindrical_faces"] if abs(f["radius_m"]-.0025)<1e-7
       and abs(f["origin_m"][0]-distal["origin_m"][0])<1e-7]
radii=[float(np.linalg.norm(np.array(f["origin_m"])[1:]-np.array(distal["origin_m"])[1:])) for f in bolts]
report=dict(schema="s50_cad_interface_measurements_v1",source_sha256=audit["source_sha256"],
            coordinate_frame="Original STEP global CAD coordinates, metres; +Y is upright in the supplied reference",
            controller_semantics_verified=False,rigid_reference_assembly_is_not_controller_zero=True,
            axes=axes,
            support_plane=dict(y_CAD_m=min_plane_y,up_CAD=[0,1,0],axis_intersection_CAD_m=[0,min_plane_y,0],
                               support_faces=support_planes,
                               interpretation="Lowest parallel support surfaces of the base (not the recessed centre plane at Y=0). Mounting clocking and controller-base frame remain unverified."),
            flange_distal_face=dict(**distal,bolt_cylinders=bolts,bolt_circle_radii_m=radii,
                                    interpretation="Measured distal J6 plane and eight CAD cylinder bores of 5mm diameter on 92mm PCD. Official mechanical manual PDF page 56 specifies 8-M6 threaded holes, depth 10mm; 5mm is the modeled tap-drill/core geometry, NOT the nominal fastener diameter. Tool clocking/control flange axes still require documentation."),
            geometry_checks=dict(base_top_y_m=.1363,shoulder_axis_height_above_support_m=.2803-min_plane_y,
                                 J2_J3_axis_separation_m=.9,J3_J4_axis_separation_m=.9415,
                                 axis_line_tolerance_m=1e-5),
            not_inferred=["controller zero", "controller positive joint directions", "joint hard stops",
                          "mounting orientation about the installation axis", "controller flange clocking"])
(FOLDER/"interface_measurements.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf8")
print(json.dumps(dict(axes=[dict(label=a["joint_label"],point=a["point_CAD_m"],axis=a["unoriented_axis_CAD"],pairs=len(a["evidence"])) for a in axes],
                      support_y_m=min_plane_y,support_faces=len(support_planes),
                      flange_plane=distal,flange_holes=len(bolts),flange_pcd_m=[2*x for x in radii]),ensure_ascii=False,indent=2))
