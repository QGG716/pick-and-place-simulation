"""Offline static initial-pose comparison, with exact FK and no motion credit."""
import argparse,copy,hashlib,json
from pathlib import Path
import numpy as np
from export_m710_top_layer_replay import write_viewer
from plan_m710_gap_search import frozen_layout


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--home',type=Path,required=True)
    p.add_argument('--geometry-viewer',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();home=json.loads(args.home.read_text(encoding='utf-8'))
    assert home['selected'] and home['selected']['initial_state_audit']['status']=='PASS'
    prior=json.JSONDecoder().raw_decode(args.geometry_viewer.read_text(encoding='utf-8').rsplit('const D=',1)[1])[0]
    layout=frozen_layout(home['gap_m'],home['mount_height_m']);robot=layout.robot()
    assert layout.layout_fingerprint==home['layout_fingerprint']
    assert prior['initialization']['layout_fingerprint']==layout.layout_fingerprint
    cartons=prior['segments'][0]['cartons'];assert len(cartons)==len({b['id'] for b in cartons})==40
    root=Path(__file__).resolve().parents[1]
    for digest,asset in prior['model']['meshes'].items():
        relative='assets/'+asset['source'].split('/assets/',1)[1]
        assert hashlib.sha256((root/relative).read_bytes()).hexdigest()==digest
    data=copy.deepcopy(prior);q=home['selected']['q_rad']
    old_q=prior['segments'][0]['q'];data['segments'][0].update(q=q,phase='新初态 · 静态预览',source='STATIC_INITIAL_POSE_NOT_A_TRAJECTORY')
    data.update(initialization=home,seed=home['seed'],source_commit='LOCAL_FRONT_WORKING_INITIAL_20261110',
        verification=dict(scope='STATIC_INITIAL_STATE_ONLY',initial_geometry_passed=True,
            exact_endpoint_validation=home['exact_endpoint_validation'],
            initial_state_accepted=home['accepted_initial_state'],execution_ready=False),
        execution_ready=False,physical_execution='NOT_RUN')
    fk=robot.named_link_frames(np.asarray(q))['flange'].T.reshape(-1).tolist()
    old_fk=robot.named_link_frames(np.asarray(old_q))['flange'].T.reshape(-1).tolist()
    data['fk_witnesses']=[fk]
    write_viewer(data,args.output)
    labels=dict(new_q=q,old_q=old_q,new_fk=fk,old_fk=old_fk,
        qualified=home['accepted_initial_state'],joints_deg=np.degrees(q).tolist())
    script=r'''
<style>aside h3{margin:15px 0 7px}#play,#reset,#seek,#clock,#timing,#identity{display:none}
.posebar{display:flex;gap:4px;margin-bottom:10px}.posebar button{flex:1;margin:0}.posebar button.active{background:#167e87;border-color:#88eced}
#quality{line-height:1.8}#detail{font-size:12px}.muted{line-height:1.6}#source{max-width:90%}
</style><script>
const P=/*POSE_DATA*/;
document.title='FANUC M710 · 正面工作初态';
document.querySelector('h1').textContent='FANUC M-710 · 正面工作初态';
document.querySelector('header small').textContent='g = 700 mm · 安装面 900 mm · J1 自然偏转 · 肩肘向前 · 静态对比';
const bar=document.createElement('div');bar.className='posebar';bar.innerHTML='<button id="newPose">新初态</button><button id="oldPose">调整前</button>';
document.querySelector('aside').prepend(bar);
const headings=document.querySelectorAll('aside h3');headings[0].textContent='场景';headings[1].textContent='关节角度';headings[2].textContent='构型说明';headings[3].style.display='none';headings[4].textContent='检查状态';
function staticPose(which){
 playing=false;D.segments[0].q=(which==='new'?P.new_q:P.old_q).slice();D.segments[0].phase=which==='new'?'新初态 · 静态预览':'调整前 · 静态对照';draw(0);
 $('newPose').classList.toggle('active',which==='new');$('oldPose').classList.toggle('active',which==='old');
 $('counts').textContent='完整40箱 · 底盘及输送带位置保持';
 $('detail').textContent=D.segments[0].q.map((x,i)=>'J'+(i+1)+'   '+(x*180/Math.PI).toFixed(2)+'°').join('\n');
 $('quality').textContent=which==='new'?'J1 接近0°，未固定\n大臂向前上方\n小臂向前下方\n吸具朝向箱堆 +X\n接近/接触端点无臂型硬门槛':'旧初态：J1 已接近0°\n但上臂向后倾斜\n用于对照，不是过渡动作';
 document.querySelector('.muted').textContent=which==='new'?(P.qualified?'初态几何及精确端点检查通过。静态展示，未执行抓放。':'本地关节、FK、初态几何及奇异位检查通过。服务器精确网格端点检查待完成；本方案暂不用于执行。'):'此前已检查的静态初态。切换按钮仅切换展示姿态，不代表机器人运动。';
 $('source').textContent=which==='new'?'新初态 · 静态预览 · 非物理执行':'调整前 · 静态对照 · 非物理执行';
 $('hud').innerHTML='目标：未选择 · 静态初态<br>附着：否 · 吸盘：关闭<br>名义接收0 · 理想清空0 · 实测接收0';
 window.previewState.pose_variant=which;window.previewState.continuous=false;window.previewState.execution_ready=false;
}
$('newPose').onclick=()=>staticPose('new');$('oldPose').onclick=()=>staticPose('old');
$('side').onclick=()=>{camera.position.set(-1.05,-5.6,1.8);controls.target.set(-1.05,0,1.4);controls.update();};
$('iso').onclick=()=>{camera.position.set(-4.7,-4.5,3.2);controls.target.set(-.9,0,1.35);controls.update();};
window.selectInitialPose=staticPose;staticPose('new');$('side').click();
</script>'''.replace('/*POSE_DATA*/',json.dumps(labels,separators=(',',':')))
    script=script.replace(chr(92)*2+'n',chr(92)+'n')
    args.output.write_text(args.output.read_text(encoding='utf-8').replace('</html>',script+'</html>'),encoding='utf-8')
    checks=dict(q_rad=q,old_q_rad=old_q,flange_matrix_column_major=fk,old_flange_matrix_column_major=old_fk,
        carton_ids=sorted(b['id'] for b in cartons),carton_count=40,layout_fingerprint=layout.layout_fingerprint,
        home_sha256=hashlib.sha256(args.home.read_bytes()).hexdigest(),
        geometry_source_sha256=hashlib.sha256(args.geometry_viewer.read_bytes()).hexdigest(),
        html_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
        initial_geometry_passed=True,exact_endpoint_validation=home['exact_endpoint_validation'],
        execution_ready=False,physical_execution='NOT_RUN',scope='STATIC_COMPARISON_NO_TRAJECTORY')
    args.output.with_suffix('.checks.json').write_text(json.dumps(checks,indent=2),encoding='utf-8')
    print(json.dumps(dict(output=str(args.output),bytes=args.output.stat().st_size,static_comparison=True)))


if __name__=='__main__':main()
