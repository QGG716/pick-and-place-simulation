"""Generate private tool assembly configuration from measured CAD, never resize parts."""
from pathlib import Path
import json, hashlib
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1]
D=ROOT/"assets/tools/desktop_suction/derived"
def build():
    cad=json.loads((D/"manifests/cad.json").read_text(encoding="utf-8"))
    parts=cad["parts"]
    plate=max(parts,key=lambda p:np.prod(sorted(p["size_m"])[-2:]))
    # Repeated bellows/stem solids are identified by CAD geometry, not a hard-coded cup count.
    cups=[p for p in parts if p["size_m"][1]>.05 and .03<p["size_m"][0]<.05 and .03<p["size_m"][2]<.05]
    if not cups:raise ValueError("Cannot identify work-face components; inspect CAD, no demo fallback")
    lip_y=min(p["bounds_m"][0][1] for p in cups)
    if max(abs(p["bounds_m"][0][1]-lip_y) for p in cups)>.0002:raise ValueError("Work faces are not coplanar")
    R=np.array([[1,0,0],[0,0,1],[0,-1,0]],float)
    mount=np.eye(4);mount[:3,:3]=R
    # Named simulation standoff: keep gauge/fitting behind the tool clear of the mounting beam.
    back=max(p["bounds_m"][1][1] for p in parts)
    standoff=float(np.ceil((back+.040)/.005)*.005)
    mount[:3,3]=[0,0,standoff]
    tcp=np.eye(4);tcp[:3,:3]=R.T;tcp[:3,3]=[0,lip_y,0]
    chain=mount@tcp
    collisions=[];rings=[]
    for p in parts:
        m=trimesh.load(ROOT/p["visual"],process=False); v=np.asarray(m.vertices)
        if p in cups:
            b=np.array(p["bounds_m"]); center=b.mean(axis=0)
            band=v[v[:,1]<lip_y+.00015]
            radius=float(np.median(np.linalg.norm(band[:,[0,2]]-center[[0,2]],axis=1)))
            angles=np.linspace(0,2*np.pi,96,endpoint=False)
            ring=np.column_stack([center[0]+radius*np.cos(angles),np.full(len(angles),lip_y),center[2]+radius*np.sin(angles)])
            rings.append(dict(id=p["id"]+"_seal",component=p["id"],center_cad_m=[center[0],lip_y,center[2]],radius_m=radius,
                ring_cad_m=ring.tolist(),normal_cad=[0,-1,0],evidence="CAD terminal circular band, median radius; free-state tangent contact",real_compression_stroke_m=None))
            planes=[f for f in p["surface_features"] if f["type"]=="plane" and abs(f["axis"][1])>.999 and f["area_m2"]>.0001 and lip_y+.001<f["point_m"][1]<lip_y+.01]
            if not planes:raise ValueError("No CAD annular skirt termination plane")
            cut=min(f["point_m"][1] for f in planes)
            # Clip the actual triangle edges; convex hull of each clipped region covers all its triangles.
            tri=v[np.asarray(m.faces)];cross=[]
            for a,b in [(0,1),(1,2),(2,0)]:
                va,vb=tri[:,a],tri[:,b];mask=(va[:,1]-cut)*(vb[:,1]-cut)<0
                va,vb=va[mask],vb[mask]
                if len(va):cross.extend(va+(cut-va[:,1])[:,None]/(vb[:,1]-va[:,1])[:,None]*(vb-va))
            cross=np.asarray(cross)
            for suffix,mask,semantic in [("rigid",v[:,1]>=cut,"conservative_rigid"),("lip",v[:,1]<=cut,"terminal_contact_band")]:
                pts=np.concatenate([v[mask],cross]);hull=trimesh.convex.convex_hull(pts)
                target=D/"collision"/(p["id"]+"_"+suffix+".obj");hull.export(target)
                collisions.append(dict(id=p["id"]+"_"+suffix,source_part=p["id"],mesh=str(target.relative_to(ROOT)).replace("\\","/"),semantic=semantic,
                    coverage="convex hull of clipped triangle vertices and edge intersections",split_plane_cad_y_m=cut))
        else:
            collisions.append(dict(id=p["id"],source_part=p["id"],mesh=p["collision"],semantic="conservative_rigid"))
    # Explicit assumed adapter geometry, in flange coordinates. Bolt holes are not modeled.
    plate_y=max(p["bounds_m"][1][1] for p in parts if p["size_m"][0]>.15 and p["size_m"][1]<.02)
    leg_end=standoff-plate_y
    adapter=[dict(id="adapter_flange_disk",kind="cylinder",radius=.0285,half_length=.004,position=[0,0,.004]),
             dict(id="adapter_cross_plate",kind="box",size=[.18,.07,.008],position=[0,0,.012])]
    for x in [-.082,.082]:
        for y in [-.0225,.0225]:
            adapter.append(dict(id=f"adapter_leg_{len(adapter)}",kind="box",size=[.008,.008,leg_end-.016],position=[x,y,(leg_end+.016)/2]))
    result=dict(schema="eco65_tool_v1",source_sha256=cad["source_sha256"],transform_convention="T_A_B maps B to A",
        length_unit="m",T_flange_tool_cad=mount.tolist(),T_tool_cad_tcp=tcp.tolist(),T_flange_tcp=chain.tolist(),
        tcp_evidence="CAD-derived free-state mean seal-plane origin; +Z toward target, +X CAD X, +Y CAD Z",
        assembly_status="SIMULATION_MOUNT_ASSUMPTION",assembly_assumption="named_four_leg_standoff_v1",
        mounting_evidence=dict(official_flange="RMU01001 foundation version: 6x M4 on diameter49mm, outside57mm",
            cad_mounting_holes="four diameter5mm holes at CAD x=+/-82mm,z=+/-22.5mm on y=10mm plate; no matching 49mm bolt circle",
            ambiguity="0/180 degree yaw mechanically unresolved; symmetric plate/cup footprint; choose yaw0 in simulation"),
        adapter=adapter,collisions=collisions,seals=rings,visual_parts=parts,
        contact=dict(ideal_suction=True,geometric_position_tolerance_m=.0004,normal_tolerance_rad=.01,
            terminal_band_thickness_m=float(cut-lip_y),terminal_band_semantics="CAD lip-to-annular-plane skirt; contact-only allowance, material/compliance not measured",
            independent_control_verified=False,require_all_seals_fit=True),
        physics=dict(tool_mass_kg=1.0,adapter_mass_kg=.25,source="finite engineering simulation assumptions, not measured",
                     actual_mass_kg=None,actual_material=None,actual_tcp_calibration=None,vacuum_circuit=None))
    out=ROOT/"configs/local/eco65_desktop";out.mkdir(parents=True,exist_ok=True)
    (out/"tool.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    print("tool config",len(cups),"seals; TCP in flange",chain[:3,3],"adapter length",standoff,flush=True)
    return result
if __name__=="__main__":build()
