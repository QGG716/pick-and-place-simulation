"""Write a local interactive 3D view of the site layout roles and lift chain.

Private CAD-derived data: the page is written under the ignored outputs/
directory and is meant to be opened locally, not published.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from unloading_sim.eco65.site_mechanism import LiftMechanism, cad_robot_base, load_manifest, load_roles  # noqa: E402

PAGE = r"""<!doctype html><html lang="zh"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Site layout lift</title>
<style>
:root{--bg:#f6f7f9;--fg:#1d2430;--muted:#5b6575;--panel:#ffffffee;--line:#d5dae2}
@media (prefers-color-scheme:dark){:root{--bg:#14171c;--fg:#e6e9ef;--muted:#9aa4b2;--panel:#1d2128ee;--line:#333a45}}
html,body{margin:0;height:100%;background:var(--bg);color:var(--fg);font:13px/1.45 system-ui,sans-serif}
#view{position:fixed;inset:0}
#panel{position:fixed;left:12px;top:12px;max-width:340px;background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px 12px}
#panel h1{font-size:14px;margin:0 0 6px} .muted{color:var(--muted)} input[type=range]{width:100%}
.legend span{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px;vertical-align:-1px}
.legend div{white-space:nowrap} #tip{position:fixed;pointer-events:none;background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:3px 6px;display:none}
table{border-collapse:collapse} td{padding:0 8px 0 0}
</style></head><body><div id="view"></div>
<div id="panel"><h1>现场布局 · 升降机构</h1>
<div class="muted">base_link 坐标，+Z 向上；CAD 设计几何，非实测。</div>
<label>升降 h（相对 CAD 位置）：<b id="hv"></b> · 剪叉总高 <b id="ov"></b></label>
<input id="h" type="range" step="0.001">
<div class="muted">滑块范围 = 用户确认的升降行程（剪叉总高 160–300 mm）；CAD 位置为 346 mm，在行程之上。</div>
<table id="state"></table>
<div class="legend" id="legend"></div>
<label><input type="checkbox" id="ghost"> 显示被剔除的 CAD 零件（半透明）</label>
<div class="muted" id="robotinfo"></div>
</div><div id="tip"></div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>
const D = __DATA__;
const COLORS = {FIXED:0x8a94a6, LIFT_TOP:0x2f80ed, BELT_INCLINED:0xf2994a, BELT_SLIDER:0x9b51e0, LIFT_INTERIOR:0x27ae60,
  CARTON:0xc8a165, ROBOT_LINK:0x555555, ROBOT_CAMERA_MAST:0x777777, TOOL:0xeb5757};
const LABELS = {FIXED:'固定结构', LIFT_TOP:'升降顶板组', BELT_INCLINED:'斜输送带', BELT_SLIDER:'上铰点滑块组', LIFT_INTERIOR:'剪叉内部包络',
  CARTON:'纸箱（剔除）', ROBOT_LINK:'机械臂（剔除）', ROBOT_CAMERA_MAST:'相机支架（剔除）', TOOL:'吸盘工具（剔除）'};
const renderer = new THREE.WebGLRenderer({antialias:true}); renderer.setPixelRatio(devicePixelRatio);
document.getElementById('view').appendChild(renderer.domElement);
const scene = new THREE.Scene();
const dark = matchMedia('(prefers-color-scheme: dark)').matches; scene.background = new THREE.Color(dark?0x14171c:0xf6f7f9);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50); camera.up.set(0,0,1); camera.position.set(1.6,-2.4,1.4);
const controls = new THREE.OrbitControls(camera, renderer.domElement); controls.target.set(0.3,0.1,0);
scene.add(new THREE.HemisphereLight(0xffffff,0x445566,0.9)); const sun=new THREE.DirectionalLight(0xffffff,0.6); sun.position.set(2,-3,4); scene.add(sun);
scene.add(new THREE.AxesHelper(0.3));
const root = new THREE.Group(); const Tb = new THREE.Matrix4().fromArray(D.T_base_cad.flat()).transpose();
root.matrixAutoUpdate=false; root.matrix.copy(Tb); scene.add(root);
const ghosts=[]; const meshes=[];
function box(b, ghost){
  const g=new THREE.BoxGeometry(...b.size); const m=new THREE.MeshLambertMaterial({color:COLORS[b.role],transparent:ghost||b.role==='LIFT_INTERIOR',opacity:ghost?0.18:(b.role==='LIFT_INTERIOR'?0.35:1)});
  const mesh=new THREE.Mesh(g,m); mesh.matrixAutoUpdate=false; mesh.userData=b; root.add(mesh);
  if(!ghost){const e=new THREE.LineSegments(new THREE.EdgesGeometry(g),new THREE.LineBasicMaterial({color:0x000000,transparent:true,opacity:0.15})); mesh.add(e);}
  (ghost?ghosts:meshes).push(mesh); return mesh;
}
D.boxes.forEach(b=>box(b,false)); D.excluded.forEach(b=>box(b,true));
const robotMeshes=[];
if(D.robot){ const geo={};
  for(const [k,a] of Object.entries(D.robot.meshes)){ const g=new THREE.BufferGeometry();
    const pos=new Float32Array(Uint8Array.from(atob(a.positions_b64),c=>c.charCodeAt(0)).buffer);
    const idx=new Uint32Array(Uint8Array.from(atob(a.indices_b64),c=>c.charCodeAt(0)).buffer);
    g.setAttribute('position',new THREE.BufferAttribute(pos,3)); g.setIndex(new THREE.BufferAttribute(idx,1)); g.computeVertexNormals(); geo[k]=g; }
  for(const it of D.robot.items){ const m=new THREE.Mesh(geo[it.asset],new THREE.MeshLambertMaterial({color:it.tool?0xd64545:(it.link.startsWith('link')||it.link==='base_link'?0xe8eaee:0x6c7a89)}));
    m.matrixAutoUpdate=false; m.matrix.fromArray(it.pose.flat()).transpose(); m.userData={role:'ROBOT',name:it.link+(it.tool?' · 工具':''),id:it.id}; scene.add(m); robotMeshes.push(m);} }
const M=D.mech, dy0=M.upper[1]-M.lower[1], dz0=M.upper[2]-M.lower[2], L=Math.hypot(dy0,dz0), a0=Math.atan2(dy0,dz0);
function T(x,y,z){return new THREE.Matrix4().makeTranslation(x,y,z);}
function update(h){
  const dy=dy0-h, dz=Math.sqrt(L*L-dy*dy), a=Math.atan2(dy,dz), shift=dz-dz0, d=-(a-a0);
  const G={FIXED:new THREE.Matrix4(), LIFT_INTERIOR:new THREE.Matrix4(), LIFT_TOP:T(0,h,0), BELT_SLIDER:T(0,0,shift),
    BELT_INCLINED:T(M.lower[0],M.lower[1]+h,M.lower[2]).multiply(new THREE.Matrix4().makeRotationX(d)).multiply(T(-M.lower[0],-M.lower[1],-M.lower[2]))};
  for(const m of meshes.concat(ghosts)){ const b=m.userData; let c=b.centre.slice();
    if(b.role==='LIFT_INTERIOR'){ const top=M.top_under+h; m.scale.set(1,(top-M.base_top)/b.size[1],1); c[1]=(M.base_top+top)/2; m.matrix.compose(new THREE.Vector3(...c),new THREE.Quaternion(),m.scale); continue; }
    const g=G[b.role]||new THREE.Matrix4(); m.matrix.copy(g).multiply(T(...c)); }
  const over=Math.max(0,M.slider_z[1]+shift-M.rail_z[1]), gap=M.top_under+h-M.base_top;
  document.getElementById('hv').textContent=(h*1000).toFixed(0)+' mm'; document.getElementById('ov').textContent=((M.overall+h)*1000).toFixed(0)+' mm';
  document.getElementById('state').innerHTML=`<tr><td>斜输送带倾角</td><td>${(a*180/Math.PI).toFixed(2)}°</td></tr>
   <tr><td>上铰点滑块位移</td><td>${(shift*1000).toFixed(1)} mm</td></tr><tr><td>剪叉板间距</td><td>${(gap*1000).toFixed(0)} mm</td></tr>
   <tr><td>滑块超出导轨端</td><td>${(over*1000).toFixed(1)} mm</td></tr>`;
}
const leg=document.getElementById('legend'); for(const [k,v] of Object.entries(LABELS)){ const n=(D.counts[k]||0); leg.innerHTML+=`<div><span style="background:#${COLORS[k].toString(16).padStart(6,'0')}"></span>${v} · ${n}</div>`;}
const hs=document.getElementById('h'); hs.min=M.range[0]; hs.max=M.range[1]; hs.value=M.range[1]; hs.oninput=()=>update(+hs.value); update(M.range[1]);
document.getElementById('ghost').onchange=e=>ghosts.forEach(g=>g.visible=e.target.checked); ghosts.forEach(g=>g.visible=false);
document.getElementById('robotinfo').textContent=D.robot?`机械臂：官方 URDF + 吸盘工具 + 相机支架，姿态为现场记录 ${D.robot.source.record.slice(0,8)} 的冻结 q_start（仅显示，不是规划结果）`:'未绘制机械臂';
const ray=new THREE.Raycaster(), mouse=new THREE.Vector2(), tip=document.getElementById('tip');
renderer.domElement.addEventListener('pointermove',e=>{mouse.set(e.clientX/innerWidth*2-1,-e.clientY/innerHeight*2+1); ray.setFromCamera(mouse,camera);
  const hit=ray.intersectObjects(meshes.concat(robotMeshes,ghosts.filter(g=>g.visible)),false)[0];
  if(hit){const b=hit.object.userData; tip.style.display='block'; tip.style.left=e.clientX+12+'px'; tip.style.top=e.clientY+12+'px'; tip.textContent=`${LABELS[b.role]||'机械臂模型（官方 URDF / 工具 / 相机支架）'} · ${b.name} · ${b.id}`;} else tip.style.display='none';});
function resize(){renderer.setSize(innerWidth,innerHeight); camera.aspect=innerWidth/innerHeight; camera.updateProjectionMatrix();}
addEventListener('resize',resize); resize(); (function loop(){requestAnimationFrame(loop); controls.update(); renderer.render(scene,camera);})();
</script></body></html>"""


def robot_model(config_path, q):
    """Official URDF + tool + camera mast visual meshes at q (base_link = world).

    Same construction as the existing field viewers: model files only, no
    worker, device, plan or permit.
    """
    sys.path.insert(0, str(ROOT / 'tools'))
    from copy import deepcopy
    from export_eco65_search_replay import export_model
    from visualize_field_planning_failures import full_frames
    from unloading_sim.workbench.single_carton.config import load
    from unloading_sim.workbench.single_carton.planner import SinglePlanner
    from unloading_sim.workbench.controlled.trajectory import Authority
    from unloading_sim.workbench.coordinates import Installation, CoordinateModel
    from unloading_sim.workbench.ui import URDF, URDF_SHA, load_mapping
    config = load(config_path)
    installation = Installation.load(config['execution']['installation'])
    if not np.allclose(installation.world_from_base, np.eye(4)):
        raise ValueError('VIEW_REQUIRES_WORLD_EQUALS_BASE')
    coordinates = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    planner = SinglePlanner(config, load_mapping(config['execution']['mapping']), Authority('eco65_single_carton_segment_v1'), coordinates)
    planner.build()
    display = deepcopy(planner.source)
    display['native_model']['init'] = deepcopy(planner.init)
    display['native_model']['init']['process_geometry']['expected_contour_ids'] = config['topology']['connected_cup_ids']
    display['scene']['mesh_obstacles'] = []
    model = export_model(display)
    frames = full_frames(model, q)
    items, used = [], set()
    for item in model['links']:
        if item['layer'] != 'visual' or item.get('kind') != 'mesh':
            continue
        pose = frames[item['link']] @ np.asarray(item['origin'])
        items.append(dict(link=item['link'], id=item['id'], asset=item['asset'], tool=bool(item.get('tool')), pose=pose.tolist()))
        used.add(item['asset'])
    return dict(items=items, meshes={k: v for k, v in model['meshes'].items() if k in used}, q=list(q))


def render(manifest_path, roles_path, out, robot=None):
    manifest, roles = load_manifest(manifest_path), load_roles(roles_path)
    m = LiftMechanism(manifest, roles)
    boxes = []
    for b in m.boxes(0.):
        P = np.asarray(b['pose'])
        boxes.append(dict(id=b['id'], role=b['role'], name=b['name'], size=b['size'], centre=P[:3, 3].tolist()))
    excluded = []
    for p in manifest['parts']:
        role = m.classification[p['id']]['role']
        if role in roles['excluded_roles']:
            lo, hi = np.asarray(p['collision_bounds_m'], float)
            excluded.append(dict(id=p['id'], role=role, name=p.get('body_name', ''), size=(hi - lo).tolist(), centre=((lo + hi) / 2).tolist()))
    counts = {}
    for v in m.classification.values():
        counts[v['role']] = counts.get(v['role'], 0) + 1
    s = m.summary()
    data = dict(T_base_cad=cad_robot_base(roles).tolist(), boxes=boxes, excluded=excluded, counts=counts, robot=robot,
                mech=dict(lower=s['lower_hinge_m'], upper=s['upper_hinge_m'], base_top=s['base_plate_top_y_m'],
                          top_under=s['top_plate_underside_y_m'], rail_z=s['rail_z_m'], slider_z=s['slider_z_m'],
                          range=s['machine_range_h_m'], overall=s['overall_height_cad_m']))
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(PAGE.replace('__DATA__', json.dumps(data, ensure_ascii=False)), encoding='utf-8')
    return out


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', default=ROOT / 'assets/layout/derived/a4b6ed1298f7491b/site-layout.json')
    p.add_argument('--roles', default=ROOT / 'configs/workcells/eco65_site_layout_roles_v1.yaml')
    p.add_argument('--output', default=ROOT / 'outputs/site_layout/site-layout-view.html')
    p.add_argument('--robot-config', help='single-carton field config; draws the robot at --q-record q_start')
    p.add_argument('--q-record', help='saved single-carton task folder whose observation.json gives q_start')
    a = p.parse_args()
    robot = None
    if a.robot_config:
        obs = json.loads((Path(a.q_record) / 'observation.json').read_text(encoding='utf-8'))
        robot = robot_model(a.robot_config, obs['q_start'])
        robot['source'] = dict(record=Path(a.q_record).name, observation_id=obs['observation_id'])
    print(render(a.manifest, a.roles, a.output, robot))
