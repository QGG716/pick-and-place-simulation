"""Actual-model engineering figures and geometry-only backend export from one frozen scene."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from PIL import Image,ImageDraw
from .model import load_json,save_json,transform
from .unloading_layout import bounds

COLORS={'conveyor_transverse':'#278bab','conveyor_longitudinal':'#3b9d64','transfer':'#e2b442','outlet':'#8db590'}

def _label(rgb,title,subtitle=''):
    im=Image.fromarray(rgb);d=ImageDraw.Draw(im);d.rectangle((0,0,im.width,52),fill=(19,31,43));d.text((15,10),title,fill='white');d.text((15,30),subtitle,fill=(205,218,228));return im

def camera_image(w,q,title,subtitle,azimuth=320,elevation=-27,arrows=True):
    w.set_state(q,'pose')
    if not hasattr(w,'renderer'):w.renderer=w.mj.Renderer(w.model,height=720,width=1280)
    camera=w.mj.MjvCamera();camera.lookat[:]=[-.52,0,.35];camera.distance=3.0;camera.azimuth=azimuth;camera.elevation=elevation
    opt=w.mj.MjvOption();opt.geomgroup[3]=0
    w.renderer.update_scene(w.data,camera=camera,scene_option=opt)
    if arrows:
        c=w.scene['source_config'];z=c['conveyors']['surface_z_m']+.024
        a=c['conveyors'];tr=a['transverse']['bounds_xy_m'];lg=a['longitudinal']['bounds_xy_m'];x=(tr[0]+tr[1])/2;y=(lg[2]+lg[3])/2
        sections=[([x,tr[3]-.08,z],[x,y,z],[1.,.7,.1,1.]),([x-.10,y,z],[a['outlet']['final_center_xy_m'][0],y,z],[1.,.7,.1,1.])]
        for a,b,color in sections:
            g=w.renderer.scene.geoms[w.renderer.scene.ngeom]
            w.mj.mjv_initGeom(g,w.mj.mjtGeom.mjGEOM_ARROW,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color))
            w.mj.mjv_connector(g,w.mj.mjtGeom.mjGEOM_ARROW,.008,np.array(a),np.array(b));w.renderer.scene.ngeom+=1
    return _label(w.renderer.render().copy(),title,subtitle)

def project_actual(w,q,ax,axes=(0,1)):
    from matplotlib.patches import Polygon
    w.set_state(q,'pose')
    for e in w.entries:
        if e['kind'] not in ('robot','visual','adapter'):continue
        gid=e['gid'];m=w.model;typ=m.geom_type[gid];size=m.geom_size[gid]
        if typ==w.mj.mjtGeom.mjGEOM_MESH:
            mid=m.geom_dataid[gid];a=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid];pts=m.mesh_vert[a:a+n]
        elif typ==w.mj.mjtGeom.mjGEOM_BOX:pts=np.array([[x,y,z] for x in [-size[0],size[0]] for y in [-size[1],size[1]] for z in [-size[2],size[2]]])
        elif typ==w.mj.mjtGeom.mjGEOM_CYLINDER:
            angles=np.linspace(0,2*np.pi,40);pts=np.array([[size[0]*np.cos(a),size[0]*np.sin(a),z] for a in angles for z in [-size[1],size[1]]])
        else:continue
        points=(pts@w.data.geom_xmat[gid].reshape(3,3).T+w.data.geom_xpos[gid])[:,axes]
        if len(points)<3:continue
        try:hull=ConvexHull(points)
        except Exception:continue
        color='#b2bac4' if e['kind']=='robot' else ('#dba340' if e['kind']=='adapter' else '#bb4847')
        ax.add_patch(Polygon(points[hull.vertices],facecolor=color,edgecolor='#495563',linewidth=.35,alpha=.86,zorder=5))

def export_geometry(w,snapshot,results,out):
    q=results['home']['q'];w.select_target(w.scene['target_box_id']);w.set_state(q,'pose')
    tree=ET.fromstring(w.xml)
    for body in tree.find('worldbody').findall('body'):
        i=w.model.body(body.attrib['name']).mocapid[0]
        body.set('pos',' '.join(map(str,w.data.mocap_pos[i])));body.set('quat',' '.join(map(str,w.data.mocap_quat[i])))
    folder=out/'backend';folder.mkdir(exist_ok=True);path=folder/'scene.xml';ET.ElementTree(tree).write(path,encoding='utf-8',xml_declaration=True)
    model=w.mj.MjModel.from_xml_path(str(path));data=w.mj.MjData(model);w.mj.mj_forward(model,data)
    assert model.ngeom==w.model.ngeom
    error=float(np.max(np.abs(data.geom_xpos-w.data.geom_xpos)));assert error<1e-8
    assert all(model.body(name).id>=0 for name in w.boxes)
    save_json(folder/'manifest.json',dict(status='GEOMETRY_EXPORT_READBACK_PASS',scene_fingerprint=snapshot['fingerprint'],backend='MuJoCo MJCF geometric mocap scene',file=path.name,
        geom_count=int(model.ngeom),box_ids=list(w.boxes),max_geom_position_error_m=error,home_q=q,roof_and_both_side_walls_present=True,
        physics='NOT_EVALUATED',robot_motion='NOT_EXECUTED',private_assets='local absolute mesh paths; not a public upload package'))
    print('EXPORTED',path,flush=True)

def render_figures(w,snapshot,results,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
    folder=out/'figures';folder.mkdir(exist_ok=True);scene=w.scene;c=scene['source_config'];mapping=load_json(out/'dimension_mapping.json');q=results['home']['q']
    def rect(ax,xy,color,label=None,alpha=1,zorder=2):
        x0,x1,y0,y1=xy;ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=color,edgecolor='#465462',alpha=alpha,lw=.8,zorder=zorder))
        if label:ax.text((x0+x1)/2,(y0+y1)/2,label,ha='center',va='center',fontsize=9,zorder=8)
    def arrows(ax,trans,long,outx):
        x=(trans[0]+trans[1])/2;y=(long[2]+long[3])/2
        ax.annotate('',xy=(x,y),xytext=(x,trans[3]-.06),arrowprops=dict(arrowstyle='-|>',color='#ce6827',lw=2.5),zorder=8)
        ax.annotate('',xy=(outx,y),xytext=(x-.02,y),arrowprops=dict(arrowstyle='-|>',color='#ce6827',lw=2.5),zorder=8)
    def setup(ax,title):
        ax.set_aspect('equal');ax.set_xlabel('X into trailer (m)');ax.set_ylabel('Y left (m)');ax.set_title(title,loc='left',fontweight='bold');ax.grid(alpha=.16,zorder=0)
    def desktop(ax,robot=False):
        for o in scene['obstacles']:
            if o['role'] in ('leg','frame','roof'):continue
            lo,hi=bounds(o);xy=[lo[0],hi[0],lo[1],hi[1]]
            color={'table_envelope':'#ebe6dc','floor':'#e3e7ea','wall':'#9dabb6','mount':'#c0c7cc','bridge':'#e2b442','belt':COLORS.get(o['owner'],'#8db590')}.get(o['role'],'#ccc')
            rect(ax,xy,color,alpha=.9,zorder=1 if o['role'] in ('floor','table_envelope') else 2)
        for b in scene['boxes']:
            if b['layer']!=0:continue
            lo,hi=bounds(b);rect(ax,[lo[0],hi[0],lo[1],hi[1]],'#d6aa72',f"C{b['column']}\n3 layers")
        if robot:project_actual(w,q,ax)
        ax.axvline(scene['opening_x_m'],color='#bd5448',ls='--',lw=1.5);ax.text(scene['opening_x_m']-.015,.48,'OPENING',ha='right',color='#994b41',fontsize=9)
        a=c['conveyors'];arrows(ax,a['transverse']['bounds_xy_m'],a['longitudinal']['bounds_xy_m'],a['outlet']['final_center_xy_m'][0])
    def dimension(ax,p1,p2,text,rotation=0):
        ax.annotate('',xy=p1,xytext=p2,arrowprops=dict(arrowstyle='|-|',lw=1,color='#23415a'))
        mid=(np.array(p1)+p2)/2;ax.text(mid[0],mid[1],text,rotation=rotation,ha='center',va='center',bbox=dict(facecolor='white',edgecolor='none',pad=2),fontsize=9,color='#23415a')
    def save(fig,name):
        fig.savefig(folder/(name+'.png'),dpi=180,bbox_inches='tight');fig.savefig(folder/(name+'.svg'),bbox_inches='tight');plt.close(fig)
    # Authority configuration is read from the pinned commit, not copied historic diagram dimensions.
    fig,axs=plt.subplots(1,2,figsize=(16,6.5));old=mapping['original_layout'];t=old['trailer'];ax=axs[0]
    rect(ax,[t['opening_x_m'],t['closed_end_wall_x_m'],-t['inner_width_m']/2,t['inner_width_m']/2],'#e7edf2')
    oldbounds={r['item']:r['source_bounds_xy_m'] for r in mapping['entries'] if 'source_bounds_xy_m' in r}
    for k in ('transverse','longitudinal'):rect(ax,oldbounds[k],COLORS['conveyor_'+k])
    A=np.array(old['assembly']['world_xyz_m']);ch=old['chassis'];center=A+ch['center_a_m'];h=np.array(ch['size_xyz_m'])/2
    ax.add_patch(Rectangle(center[:2]-h[:2],2*h[0],2*h[1],fill=False,ls='--',lw=1.2,edgecolor='#777'))
    base=A[:2]+old['robot']['base_origin_xy_a_m'];lo=np.array(old['robot']['base_support_bbox_min_xyz_m'])[:2]+base;hi=np.array(old['robot']['base_support_bbox_max_xyz_m'])[:2]+base
    rect(ax,[lo[0],hi[0],lo[1],hi[1]],'#aab3bc','base')
    sz=old['carton_stack']['carton_size_xyz_m'];front=old['carton_stack']['front_face_x_m']
    for y in old['carton_stack']['center_y_m']:rect(ax,[front,front+sz[0],y-sz[1]/2,y+sz[1]/2],'#d6aa72')
    arrows(ax,oldbounds['transverse'],oldbounds['longitudinal'],-3.12);ax.axvline(-3.2,color='#bd5448',ls='--')
    ax.text(1.8,0,'Unused rear development space\nshortened in desktop design',ha='center',va='center',fontsize=9)
    setup(ax,'Pinned reference: official-base layout / 1 x 5 x 8');ax.set_xlim(-3.5,3.45);ax.set_ylim(-1.5,1.5)
    desktop(axs[1],robot=True);setup(axs[1],'Desktop DESIGN_CANDIDATE / 1 x 3 x 3');axs[1].set_xlim(-1.6,.5);axs[1].set_ylim(-.6,.6)
    fig.suptitle('Same L topology, separately designed dimensions | robot/tool remain 1:1 | no chassis',fontsize=14);fig.tight_layout(rect=(0,0,1,.94));save(fig,'01_reference_mapping')
    fig,ax=plt.subplots(figsize=(14,8));desktop(ax,robot=True);setup(ax,'Desktop unloading plan — DESIGN_CANDIDATE, all dimensions in metres')
    lo,hi,yl,yh=scene['footprint']['bounds_xy_m'];dimension(ax,(lo,-.57),(hi,-.57),f'{(hi-lo)*1000:.0f} mm static footprint')
    dimension(ax,(.40,yl),(.40,yh),f'{(yh-yl)*1000:.0f} mm outside',90);half=c['trailer']['inner_width_m']/2;dimension(ax,(.31,-half),(.31,half),f'{half*2000:.0f} mm inside',90)
    lg=c['conveyors']['longitudinal']['bounds_xy_m'];tr=c['conveyors']['transverse']['bounds_xy_m']
    dimension(ax,(lg[0],-.49),(lg[1],-.49),f'{(lg[1]-lg[0])*1000:.0f} mm longitudinal')
    dimension(ax,(-.045,tr[2]),(-.045,tr[3]),f'{(tr[3]-tr[2])*1000:.0f} mm transverse',90)
    ax.text(-1.44,.50,'Catch table\n'+' x '.join(f'{1000*d:.0f}' for d in np.diff(np.array(c['conveyors']['outlet']['catch_bounds_xy_m']).reshape(2,2),axis=1).ravel())+' mm',ha='left',fontsize=9);ax.text(-.85,.50,'Robot fixed plate '+' x '.join(f'{d*1000:.0f}' for d in c['robot']['mounting_plate_size_m'])+' mm\nNo chassis / no lift',fontsize=9)
    ax.text(.13,.55,f'{len(scene["boxes"])} cartons\n'+' x '.join(f'{d*1000:.0f}' for d in c['carton_stack']['carton_size_xyz_m'])+f' mm\n{c["carton_stack"]["column_gap_m"]*1000:.0f} mm column gap',ha='center',fontsize=9)
    ax.text(-.95,-.72,f'Effective widths: {(tr[1]-tr[0])*1000:.0f} / {(lg[3]-lg[2])*1000:.0f} mm | {(tr[2]-lg[3])*1000:.0f} mm transfer bridge | orange: -Y then -X\nDesk fit pending measurement; suggested edge reserve {c["installation"]["recommended_edge_reserve_m"]*1000:.0f} mm per side.',fontsize=10)
    ax.set_xlim(-1.59,.55);ax.set_ylim(-.78,.72);save(fig,'02_dimensioned_top')
    fig,ax=plt.subplots(figsize=(14,7))
    for o in scene['obstacles']:
        if o['id'] in ('trailer_left','trailer_right'):continue
        lo,hi=bounds(o);color={'wall':'#a8bbc8','roof':'#a8bbc8','belt':'#3a9e85','floor':'#d2d8dd','bridge':'#e2b442','table_envelope':'#e5dfd4','mount':'#bcc4ca'}.get(o['role'],'#aab1b7')
        rect(ax,[lo[0],hi[0],lo[2],hi[2]],color,alpha=.8)
    for b in scene['boxes']:
        if b['column']!=1:continue
        lo,hi=bounds(b);rect(ax,[lo[0],hi[0],lo[2],hi[2]],'#d6aa72',f"Layer {b['layer']}")
    project_actual(w,q,ax,(0,2));ax.axvline(scene['opening_x_m'],color='#bd5448',ls='--')
    dimension(ax,(.39,scene['floor_z_m']),(.39,scene['roof_inner_z_m']),f'{c["trailer"]["inner_height_m"]*1000:.0f} mm clear height',90)
    stackheight=c['carton_stack']['carton_size_xyz_m'][2]*c['carton_stack']['height_layers']
    dimension(ax,(.29,scene['floor_z_m']),(.29,scene['floor_z_m']+stackheight),f'{stackheight*1000:.0f} mm stack',90)
    ax.annotate(f'Belt surface Z = {c["conveyors"]["surface_z_m"]*1000:.0f} mm',xy=(-.78,c['conveyors']['surface_z_m']),xytext=(-1.02,.26),arrowprops=dict(arrowstyle='->'),fontsize=10)
    ax.annotate(f'Robot mounting Z = {scene["base_position_m"][2]*1000:.0f} mm\nFloor {scene["floor_z_m"]*1000:.0f} mm + plate {c["robot"]["mounting_plate_size_m"][2]*1000:.0f} mm',xy=(c['robot']['base_xy_m'][0],scene['base_position_m'][2]),xytext=(-.65,-.17),arrowprops=dict(arrowstyle='->'),fontsize=10)
    ax.text(-1.44,-.095,'Tabletop Z = 0; actual desk height unspecified',fontsize=10)
    ax.set_aspect('equal');ax.set_xlim(-1.58,.52);ax.set_ylim(-.22,.93);ax.set_xlabel('X into trailer (m)');ax.set_ylabel('Z above tabletop (m)');ax.grid(alpha=.15)
    ax.set_title('Side projection: complete roof, floor, finite walls, belt bodies and supports',loc='left',fontweight='bold');save(fig,'03_height_side')
    w.select_target(scene['target_box_id'])
    camera_image(w,q,'ECO65-B desktop unloading | DESIGN_CANDIDATE | 9 cartons', 'Actual robot + private CAD; assumed adapter | transparent enclosure remains in collision | no motion executed').save(folder/'04_assembly_overview.png')
    camera_image(w,q,'L-shaped conveyors and supported outlet','Transverse -Y / longitudinal -X | gold bridges | static support only, transfer physics NOT_EVALUATED',azimuth=300,elevation=-52).save(folder/'05_conveyors_outlet.png')
    fig,ax=plt.subplots(figsize=(14,8));desktop(ax,robot=False);setup(ax,'Representative pose checks — pose success is not a complete motion path')
    lines=[]
    for i,r in enumerate(results['candidates'],1):
        p=np.array(r['target_tcp'])[:3,3];valid=r['status']=='POSE_VALID';color='#26965c' if valid else '#c74c46'
        ax.scatter(*p[:2],s=90,facecolors=color,zorder=10,edgecolors='white');ax.annotate(str(i),p[:2],xytext=(6,6+8*(i%2)),textcoords='offset points',color=color,weight='bold')
        status=r['status'].replace('IK_NOT_FOUND_WITHIN_BUDGET','NO IK FOUND (bounded)')
        if r['removal_status']=='SUPPORT_DEPENDENCY_BLOCKED':status+=' / supports upper box'
        lines.append(f'{i}. {r["id"]}: {status}')
        candidate_q=r.get('q') or (r.get('collision_solutions') or [{}])[0].get('q')
        if candidate_q is not None:
            w.select_target(r['box_id'],np.array(r['box_pose']),r['face'])
            camera_image(w,candidate_q,f'{i}. {r["id"]} | {r["status"]}',f'All 9 box IDs retained | {r["removal_status"]} | full path NOT_EVALUATED',arrows=False).save(folder/f'pose_{i:02d}.png')
    ax.text(0,-.13,'\n'.join(lines),transform=ax.transAxes,va='top',fontsize=9,family='monospace')
    ax.set_xlim(-1.57,.45);ax.set_ylim(-.56,.60);fig.subplots_adjust(bottom=.29);save(fig,'06_representative_poses')
    w.select_target(scene['target_box_id']);w.set_state(q,'pose')
    save_json(folder/'manifest.json',dict(scene_fingerprint=snapshot['fingerprint'],figures=[p.name for p in folder.glob('*.png')],
        actual_private_geometry=True,display_only_transparency=True,all_panels_collide=True,full_path='NOT_EVALUATED'))
    print('FIGURES',folder,flush=True)