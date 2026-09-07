"""Build the V3 technical report and portable, checksummed evidence bundle."""
from __future__ import annotations

from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT))
import numpy as np
from tools.run_m710id70_v3 import write_json,write_csv


def read_json(path):return json.loads(path.read_text(encoding='utf-8'))
def read_csv(path):return list(csv.DictReader(path.open(encoding='utf-8-sig')))
def truth(value):return value is True or value=='True'


def main():
    root=ROOT/'outputs/m710id70_v3';out=root/'v3';baseline=root/'v2_instrumented'
    required=['task_reachability.csv','continuous_summary.json','lift_coverage_union.json','conveyor_ab.csv',
              'independent_fk_jacobian_audit.json','loaded_unit_scene_dynamics_audit.json',
              'bottom_support_release_alternative.json','determinism_audit.json','pytest.xml']
    missing=[name for name in required if not (out/name).exists()]
    if missing:raise RuntimeError(f'Incomplete V3 evidence, will not generate a completed report: {missing}')
    if not read_json(baseline/'baseline_equality.json')['exact_summary_equality']:
        raise RuntimeError('Frozen V2 baseline mismatch')
    v2=read_json(baseline/'acceptance_summary.json');manifest=read_json(out/'source_manifest.json')
    valid=[r for r in read_csv(out/'task_reachability.csv') if truth(r['inside_cross_section'])]
    continuous=read_json(out/'continuous_summary.json');ab=read_csv(out/'conveyor_ab.csv');lift=read_json(out/'lift_coverage_union.json')
    v3_counts={key:sum(truth(row[key]) for row in valid) for key in ['GRASP_REACHABLE','EXTRACTION_FEASIBLE','GEOMETRICALLY_REACHABLE','PAYLOAD_QUALIFIED','DYNAMICS_VERIFIED']}
    v2n=v2['coverage']['valid_orientation_samples'];n=len(valid)
    if n!=v2n or sum(row['total_boxes'] for row in continuous)!=sum(row['total_boxes'] for row in v2['continuous']):
        raise RuntimeError('V2/V3 populations differ')
    counts=Counter((row['failure_stage'],row['failure_reason']) for row in valid)
    failure_rows=[{'stage':stage,'reason':reason,'count':count} for (stage,reason),count in counts.most_common()]
    write_csv(out/'failure_stage_summary.csv',failure_rows)
    attempts=[];distances=[];sides=[];candidate_reasons=Counter();stage_rows=[]
    for path in sorted((out/'tasks').glob('grid_*_dynamic.json')):
        record=read_json(path)['result']
        options=[option for attempt in record['attempts'] for option in attempt['conveyor_attempts']]
        stage_rows.append({'task_id':path.stem,
            'grasp_status':'PASS' if record['grasp_reachable'] else 'FAIL_WITHIN_RECORDED_CANDIDATES',
            'extraction_status':'PASS' if record['extraction_feasible'] else ('FAIL_INITIAL_ATTACHED_STATE' if any(a['stage']=='attachment_clearance' for a in record['attempts']) else ('FAIL' if any('extraction' in option['paths'] for option in options) else 'NOT_EVALUATED_UPSTREAM_STAGE')),
            'handoff_status':'PASS' if record['geometric_feasible'] else ('FAIL' if any(option['stage'] in ['handoff','carry','place','withdrawal'] for option in options) else 'NOT_EVALUATED_UPSTREAM_STAGE'),
            'payload_status':record['load_status'],'full_robot_dynamics_status':'NOT_EVALUATED'})
        for a in record['attempts']:
            item={'task':path.stem,'face':a['face'],'roll_deg':a['roll_deg'],'stage':a['stage'],'reason':a['reason'],
                  'sealed_cups':a['coverage']['sealed_cups'],'actual_sealed_cups':a.get('actual_coverage',{}).get('sealed_cups'),
                  'distance_m':a['extraction']['distance_m'],'constraints':a['extraction']['constraint_names']}
            attempts.append(item);candidate_reasons[(a['face'],a['roll_deg'],a['stage'],a['reason'])]+=1
            if a['face']=='front' and a['roll_deg']==0:
                distances.append({**item,'both_sides_constrained':all(name in a['extraction']['constraint_names'] for name in ['left_neighbor','right_neighbor']),
                                  'constraint_geometry':a['extraction']['constraint_boxes'],'box_size_m':a['extraction']['box_size_xyz_m']})
            if a['face'] in ('left','right'):sides.append(item)
    write_csv(out/'candidate_attempt_summary.csv',attempts);write_csv(out/'extraction_constraint_audit.csv',distances)
    write_csv(out/'validation_stage_status.csv',stage_rows)
    extraction_statuses=dict(Counter(row['extraction_status'] for row in stage_rows))
    bilateral=[row['distance_m'] for row in distances if row['both_sides_constrained'] and row['distance_m'] is not None]
    histogram=dict(sorted(Counter(round(float(d),6) for d in bilateral).items()))
    quantiles=None if not bilateral else np.percentile(bilateral,[50,95]).tolist()
    geom_rows=read_csv(out/'extraction_independent_cases.csv')
    jac=read_json(out/'independent_fk_jacobian_audit.json');timing=read_json(out/'trajectory_dynamics_audit.json')
    loaded=read_json(out/'loaded_unit_scene_dynamics_audit.json');bottom=read_json(out/'bottom_support_release_alternative.json')
    testroot=ET.parse(out/'pytest.xml').getroot();suites=list(testroot.iter('testsuite'))
    tests=sum(int(s.get('tests',0)) for s in suites);failures=sum(int(s.get('failures',0))+int(s.get('errors',0)) for s in suites)
    if failures:raise RuntimeError('Cannot report completed regression verification with failing tests')
    demo=ET.parse(out/'demo_pytest.xml').getroot()
    demo_suites=list(demo.iter('testsuite'))
    demo_tests=sum(int(s.get('tests',0)) for s in demo_suites)
    demo_errors=sum(int(s.get('failures',0))+int(s.get('errors',0)) for s in demo_suites)
    if demo_errors:raise RuntimeError('Default CPU demo regression failed')
    new=sum(truth(r['new_feasible']) for r in ab);lost=sum(truth(r['lost_feasible']) for r in ab);common=[r for r in ab if truth(r['common_feasible'])]
    action_mean=None if not common else float(np.mean([float(r['cycle_delta_s']) for r in common]))
    load_faces=read_json(out/'load_face_roll_audit.json')
    loaded_max_m=np.max([r['load']['moment_utilization'] for r in loaded.get('samples',[])],axis=0).tolist() if loaded.get('samples') else None
    summary={'reference_commit':v2.get('reference_commit','1a9e7be084903b3d4d2471659b645e52a367ef8e'),
             'implementation_commit':manifest['commit'],'grid_denominator':n,'v3_grid':v3_counts,'v2_coverage':v2['coverage'],
             'continuous':continuous,'ab':{'new_feasible':new,'lost_feasible':lost,'common_feasible':len(common),'mean_cycle_delta_s':action_mean},
             'lift':lift,'bilateral_extraction_histogram_m':histogram,'bilateral_p50_p95_m':quantiles,
             'extraction_stage_status_counts':extraction_statuses,
             'independent_fk_jacobian_pass':jac['pass'],'tests_passed':tests,
             'bottom_support_release_alternative':{'geometric_feasible':bottom['result']['geometric_feasible'],'load_status':bottom['result']['load_status']},
             'full_robot_dynamics_status':'NOT_EVALUATED','task_success_does_not_equal_hardware_qualification':True}
    write_json(out/'acceptance_summary_v3.json',summary)
    compare='\n'.join(f"| {label} | {old}/{v2n} | {v3_counts[key]}/{n} |" for label,old,key in [
        ('抓取（两版判据不同）',round(v2['coverage']['grasp_coverage']*v2n),'GRASP_REACHABLE'),
        ('脱垛',round(v2['coverage']['extraction_coverage']*v2n),'EXTRACTION_FEASIBLE'),
        ('完整几何任务',round(v2['coverage']['full_task_coverage']*v2n),'GEOMETRICALLY_REACHABLE'),
        ('负载资格通过',round(v2['coverage']['payload_qualified_coverage']*v2n),'PAYLOAD_QUALIFIED')])
    old_cont={r['scenario']:r for r in v2['continuous']}
    cont_table='\n'.join(f"| {r['scenario']} | {r['total_boxes']} | {old_cont[r['scenario']]['geometrically_unloaded_boxes']} | {r['geometrically_unloaded_boxes']} | {r['remaining_boxes']} |" for r in continuous)
    failure_table='\n'.join(f"| {r['stage']} | {r['reason']} | {r['count']} |" for r in failure_rows)
    face_table='\n'.join(f"| {r['face']} | {r['load']['combined_com_flange_m'][0]:.9f} | {r['load']['statuses']['rated_payload']} | {r['load']['statuses']['payload_cg']} |" for r in load_faces if r['roll_deg']==90)
    bottom_best=bottom['result']['selected'] or {}
    bottom_line=(f"完整几何动作成功；吸取面 {bottom_best.get('face')}、滚转 {bottom_best.get('roll_deg')}°，带载 TCP 路径 {bottom_best.get('loaded_tcp_path_m',0):.6f} m，"
                 f"含配置动作开销的周期 {bottom_best.get('cycle_s',0):.6f} s，负载资格 {bottom['result']['load_status']}。"
                 f"实际 FK 放置穿透数值 {bottom_best.get('support',{}).get('penetration_m',0):.9f} m（必须不超过显式接触容差 0.0002 m）。") if bottom['result']['geometric_feasible'] else f"失败：{bottom['result']['failure_stage']} / {bottom['result']['failure_reason']}。"
    report=f'''# FANUC M-710iD/70 V3 技术验证报告

## 结论与验证范围

已实施参数、坐标、SO(3)、IK、刚体附着、碰撞路径、负载及时间参数化修复。
冻结 V2 两次复跑汇总完全一致。V3 在原始 {n} 个有效网格任务及
{sum(r['total_boxes'] for r in continuous)} 箱连续场景上完成复跑。
当前完整几何成功 {v3_counts['GEOMETRICALLY_REACHABLE']}/{n}，连续几何卸货
{sum(r['geometrically_unloaded_boxes'] for r in continuous)}/{sum(r['total_boxes'] for r in continuous)}。
这些数字是当前配置、代理几何、离散精度和确定性搜索预算下找到的解数，
不能外推为所有可能路径均不存在。新增约束排除了 V2 的若干乐观结果。

全机器人动力学、驱动资格和吸附承载证据仍为 **NOT_EVALUATED**。
原 20 kg 夹具 + 42.5 kg 箱体 + 250 mm TCP + 600 mm 深箱正面吸取的
质心问题保留 **FAIL**。另有底层箱替代动作完整几何验证成功，独立列示，
不加入原始场景分子或分母。

## 1. 版本、种子和基线

- 审计参考提交：`1a9e7be084903b3d4d2471659b645e52a367ef8e`。
- 主验证实现提交：`{manifest['commit']}`，分支 `fix/m710id70-v3-validation`。
- Python / NumPy：`{manifest['python'].splitlines()[0]}` / `{manifest['numpy']}`。
- 网格规划种子：71070 + 原始行号；随机场景：71071、71072、71073。
- 连续候选种子：74000 + 场景序号×1000 + 已完成序号×50 + 候选序号。
- V2 冻结源码由 `git archive` 导出并逐文件校验；包装器记录实际调用参数，
  不改变算法。`v2_instrumented/effective_calls_and_task_results.jsonl` 包含
  工具/机器人构造、候选和交接调用，补齐 V2 原日志缺少失败候选的问题。
- 最终 V3 全量复跑使用 12 个独立 CPU 进程，内容指纹缓存仅复用一致输入。
  `determinism_audit.json` 中 3 个代表任务单进程重算与原结果逐字段相同。

| 指标 | 冻结 V2 | V3 |
| --- | ---: | ---: |
{compare}
| 完整机器人动力学已验证 | NOT_EVALUATED | NOT_EVALUATED |

V3 的脱垛计数表示完整有序管线中已验证通过的任务数。逐阶段状态为
`{extraction_statuses}`；被上游覆盖/IK/传送带阶段阻止而未执行脱垛的
任务标为 NOT_EVALUATED_UPSTREAM_STAGE，不当作已证明脱垛碰撞失败。
完整分母仍为原始 {n}，没有通过缩小场景或重定义分母提高成功率。
见 `validation_stage_status.csv`。

| 连续场景 | 原箱数 | V2 几何卸出 | V3 几何卸出 | V3 保留箱数 |
| --- | ---: | ---: | ---: | ---: |
{cont_table}

## 2. 参数和坐标修复

V2 脚本绕过继承后的场景：实际车厢 2.3×2.7 m，而 common YAML 是
2.35×3.0 m；实际碰撞 margin=0.001 m，而配置是 0.01 m。
V3 用独立、严格校验且支持带来源继承的 `configs/validation/m710id70_v3.yaml`。
完整配置、物理工具质量/质心/惯量、URDF、运动限制和每个叶参数来源都保存在
`effective_config.json`。机器人安装位置、底盘世界位姿和升降高度分开组合。
升降机器人不会移动底盘或独立传送带。

机械工具坐标采用法兰系中配置的六维 TCP 变换；吸取任务系 +Z 为法向，
通过固定 Ry(π/2) 与机械 +X 法向关联。`tool0_from_task_tcp` 明确输出。
夹具外形由存储的 [长、短、深] 转为任务系 [短、长、法向]，
与 6×12、48 mm 间距、21.5 mm 半径的离散吸盘布局一致。
完整圆面落在箱面内才计为密封杯；配置要求至少 60 杯。
实际吸附力、分区阀路、箱面密封质量和剪切摩擦仍未验证。

吸附时保存 `T_TCP_box = inverse(T_world_TCP_actual) @ T_world_box_initial`；
后续 OBB、箱体质心及惯量均由 `T_world_TCP_actual @ T_TCP_box` 更新。
四个面×四个滚转角的回归确保初始箱体不会因滚转而凭空转动。

原 home 在新增工具自碰撞检查下命中 tool_envelope/J4_link。
V3 显式采用 seed=71070、3000 次离线初始化搜索中第 1507 个有效姿态；
`home_probe.json` 保存候选和原 home 失败。没有接受从碰撞 home 开始的运动。
质量、TCP 长度和箱体尺寸没有为提高通过率而改写。

碰撞 margin 保留配置 0.01 m 的原始 SAT 语义：两个 OBB 各膨胀
0.01 m，所以共轴表面需要约 0.02 m 间隙。原场景的 0.01 m 邻箱缝隙
可能直接不满足该工程余量。仅指定的吸取/支撑平面允许接触；超过
0.0002 m 的数值穿透失败。这些容差均显式记录，不代表真机允许穿透。

## 3. 运动学、路径及统计变化原因

SO(3) 在 π 附近由对称矩阵最大对角元恢复轴的相对符号，atan2 恢复角度。
IK 关节居中项进入真正数值零空间，最终按实际 FK 残差判定。
V3 位置/姿态 IK 容差为 0.0001 m / 0.0002 rad；V2 抓取是
0.012 m / 0.09 rad，交接是 0.015 m / 0.10 rad。

独立中心差分在非零三维 TCP 偏置、TCP 旋转和基座旋转下验证 20 个
固定种子姿态：线速度 Jacobian 最大误差
{max(r['translation_max_error'] for r in jac['samples']):.3e}，角速度 Jacobian 最大误差
{max(r['rotation_max_error'] for r in jac['samples']):.3e}，检查通过。
角速度数值差分使用 Rdot·Rᵀ，不复用待测 SO(3) log。

当前状态→接近→接触→脱垛→搬运→实际放置→空载撤离均有路径检查。
关节插值边内部间隔最大 0.01 rad；直线段 IK 采样步长 0.025 m。
保留自碰撞检查，并补上 URDF 全部碰撞原语、夹具—机械臂、
箱体—机械臂以及车厢/底盘/升降柱/传送带检查。
最终支撑由实际 FK 箱体角点验证平面接触、悬空、穿透和边缘余量。

主网格失败汇总（一个任务只记一个代表原因，全部候选仍在 JSON 中）：

| 阶段 | 原因 | 任务数 |
| --- | --- | ---: |
{failure_table}

成功率变化同时包含：完整吸盘覆盖、夹具轴向修正、工具自碰撞和 URDF
角点代理补齐、恢复配置碰撞余量、提高实际 FK 精度、验证当前状态与
传送带扫掠。未做逐项单因素消融，因此不把总差值归因于某一个改动。

碰撞仍是工程原语近似：圆柱/球使用包围 OBB，可能保守误拒；现有胶囊
自碰撞排除对仍需 CAD 复核。离散中间点检查不构成连续碰撞检测证书，
缺少厂家碰撞网格与机构包络时不宣称整机绝对安全。

## 4. 负载、动力学和时间

同一刚体附着用于碰撞和外载计算。惯量采用 R·I·Rᵀ 和张量平行轴定理，
通过均匀长方体八个 Gauss 积分点独立验证。按实际关节轴投影静态重力矩、
动态 Newton–Euler 外力矩和轴向惯量，不把腕部允许外载力矩当作电机峰值转矩。

| 吸取面（参考姿态、90°滚转） | 组合轴向质心 m | 额定质量 | 质心判据 |
| --- | ---: | --- | --- |
{face_table}

正面组合质心约 0.421939 m，甚至超过更低 60 kg 曲线的 0.352 m
轴向截距，因此在给定质量及图表数据假设下仍是 FAIL。
曲线截距矩形内的点不能据此判为 PASS；完整曲线、参考定义及厂家负载
设置复核尚缺。较小裕量的侧吸结论尤其需要核对图表取点精度。

速度、加速度和 jerk 使用同一条 C2、逐段静止到静止五次轨迹的解析导数。
s′、s″、s‴ 的绝对最大值分别为 15/8、10/√3、60，直接决定各段时长。
一个 1 rad、加速度上限 1 rad/s² 的静止到静止运动，独立时间下界 2 s，
实际算法给出 {timing['single_joint_rest_to_rest']['actual_duration_s']:.9f} s。
时间缩放后仍使用相同曲线和导数，消除了仅靠离散差分通过的虚假快速运动。
这是一种保守 CPU 执行模型，逐节点停车的周期不能冒充控制器优化节拍。

另有非零空载/带载数值单元场景，沿同一解析轨迹保存 q/qd/qdd/jerk
及外载。带载单元场景状态 `{loaded['status']}`，峰值外载力矩利用率
J4/J5/J6 为 `{loaded_max_m}`。它是附加单元验证，不替代原始货垛任务。
底层替代动作也保存整条轨迹的负载时序。其余未完成任务保留接触负载
或部分路径状态，不伪造完整动力学结果。

额定质量、质心、腕部力矩、惯量、驱动与吸附分开记录。缺项返回
NOT_EVALUATED，合并时 FAIL 优先。连杆惯量、驱动转矩/功率/热限制、
厂家动态外载定义、吸附承载证据缺失，完整机器人动力学通过数为零且
状态为 NOT_EVALUATED，不能解读为已证明所有动力学任务失败。

## 5. 四项能力实验

**脱垛距离。** 双侧受限 front 候选的逐例几何距离分布为
`{histogram}`，P50/P95=`{quantiles}` m。
600 mm 深箱在固定姿态、侧面间隙 10 mm 小于所需 20 mm 时，要沿 -X
脱开邻箱 X 投影 600 mm，再留 20 mm，得到 620 mm；400 mm 深箱对应
420 mm。不同前后错位或无邻箱会改变约束，不能由 P50=P95 推断全部相同。
无邻接约束时此函数可返回 0；后续搬运仍须独立保证支撑释放和碰撞间隙。
`extraction_constraint_audit.csv` 记录每例邻箱几何，`extraction_independent_cases.csv`
记录独立算例；距离本身不证明路径可行。

**侧面吸取。** 同一 600×400×300 mm 箱，侧吸 0° 时仅 36 个完整杯，
90° 时可达 72 杯。增加滚转已修复这一覆盖候选缺失，但当前受控左右
侧吸尚未建立完整成功案例；实际 IK/工具干涉失败点保存在
`tasks/controlled_left.json`、`tasks/controlled_right.json`。不将覆盖通过
或单点 IK 当作完整侧吸成功。

**传送带 A/B。** 固定与动态使用相同 {n} 个任务、初始状态、种子、
规划器预算及检查精度。动态新增可行 {new}，失去可行 {lost}，共同可行
{len(common)}。共同集合为空时，带载路径/周期差值为 NOT_EVALUATED，
不填 0 或使用直线下界冒充真实规划路径。`conveyor_ab.csv` 分开记录
伸缩和升降动作时间成本。这些外部轴成本按配置匀速估算，缺少加减速、
驱动和同步证据，不属于已验证动态周期。当前证据尚不能证明动态带缩短周期。

**升降轴。** 相对固定安装位的高度分别为 0/0.15/0.30/0.45/0.60 m。
每个高度都保留相同 {n} 个任务；沿整个升降过程检查机构和障碍，
不可通过升降路径的任务直接失败并保存具体高度与碰撞对。
各高度完整任务覆盖数 `{lift['coverage_by_height']}`，覆盖并集
{lift['union_count']}/{lift['denominator']}。这不能推导“无需升降轴”或零行程；
制造行程、立柱刚度、制动和稳定性仍待机械与厂家证据。

## 6. 连续卸货和底层替代动作

选箱遍历全部当前暴露且不支撑上层的箱体，保留每个面/滚转/传送带
候选及原因。调度器回归覆盖“前两箱失败、第三箱可行”，保证不会
因仅尝试前两箱而停止。未尝试的支撑箱标记其上层依赖，不伪装为 IK 失败。
放置箱体在实际支撑成功、真空释放和空载撤离后进入接料状态；不会删除。
出料尝试验证支撑面并集和扫掠，L 转角支撑缺口或碰撞则保留箱体并阻塞接料。

原规则的动态传送带上限是当前最高箱底减 15 mm，最低接料面是 200 mm；
对唯一底层箱，上限低于下限，明确记录机械/布局约束。
固定带替代尝试暴露“脱垛最小距离为零，但带载搬运从地板接触出发”
的问题。主规划器 `Cell.transit` 已集成支撑释放动作，独立复现入口
`tools/validate_m710_bottom_alternative.py` 使用同一实现，根据实际箱底
推导一个抬升动作：达到地板顶面 + 两倍 OBB margin + 两倍支撑容差，
随后执行原检查和原规划器。没有降低余量或删除地板。

{bottom_line}

该结果只证明此独立底层动作的几何可行，完整资格仍为
{bottom['result']['load_status']}，未改写原始连续场景统计，也没有宣称已连续
清空 40 箱。原动态带高度冲突和上层任务干涉仍阻止原场景连续完成。

## 7. 分机构结论与数据缺口

| 对象 | 已有验证证据 | 尚缺证据 / 当前结论 |
| --- | --- | --- |
| M-710iD/70 | FK/Jacobian 数值一致性、限位、确定性 IK、原语路径检查 | 完整连杆惯量、驱动限制、厂家碰撞/运动包络与控制器插补；未获整机动力学资格 |
| 20 kg 夹具 | 6D TCP、长短轴、杯覆盖、刚体载荷一致性；正面 CoM FAIL | 实测含阀/线缆质量分布、真空分区/压差/泄漏/剪切与惯性吸附试验 |
| 独立 L 传送带 | 预定位扫掠、实际放置支撑、接料保留状态；底层固定带替代动作成功 | 全流程 A/B 暂无共同成功任务；L 转角支撑、带速/摩擦/接料动态和驱动证据 |
| 升降轴及底盘 | 世界位姿与安装位解耦、全任务高度矩阵、升降扫掠 | 不能从零覆盖/离散最佳高度推出零行程；需安装连接、立柱刚度、载荷稳定性及驱动/制动数据 |

原始 regular/random 行间存在 10 mm 几何间隔，属于原始第一层测试场景，
本轮保留以保证人口与口径可比；货垛在真实重力下的支撑/沉降稳定性
尚未验证。厂家数据缺失未阻止 CPU 几何与数值部分实施和复跑。

## 8. 测试、复现与交付证据

默认完整回归：{tests} 项通过，0 失败（仓库默认排除的 simulation 标记测试
仍按 AGENTS.md 的 `pytest -q` 配置处理）。测试结果见 `pytest.xml`。
另显式运行默认 CPU demo 的 simulation 回归：{demo_tests} 项通过，
见 `demo_pytest.xml`；该次 pytest 执行约 2.08 s，满足五秒级默认演示目标。

```powershell
.venv\\Scripts\\python.exe tools/reproduce_m710_v2.py
.venv\\Scripts\\python.exe tools/run_m710id70_v3.py --phase all --workers 4
# 可对未完成的升降部分使用更多独立 CPU 进程；缓存必须匹配代码/参数指纹
.venv\\Scripts\\python.exe tools/run_m710id70_v3.py --phase lift --workers 12
.venv\\Scripts\\python.exe tools/audit_m710_v3.py
.venv\\Scripts\\python.exe tools/validate_m710_bottom_alternative.py
.venv\\Scripts\\python.exe tools/check_m710_v3_reproducibility.py
.venv\\Scripts\\python.exe -m pytest -q --junitxml outputs/m710id70_v3/v3/pytest.xml
.venv\\Scripts\\python.exe -m pytest -q -o addopts= tests/test_demo.py::test_default_demo_succeeds --junitxml outputs/m710id70_v3/v3/demo_pytest.xml
.venv\\Scripts\\python.exe tools/report_m710_v3.py
```

完整原始证据在 `outputs/m710id70_v3/`。便携副本为本报告同目录
`evidence/m710id70_v3_evidence.zip`，包含生效配置、源文件指纹、V2
实际调用追踪、全部 V3 任务 JSON、CSV、轨迹负载时序和独立回归结果；
不包含复制出的 V2 源码树，源码由参考提交独立恢复。ZIP 内的
`evidence_manifest.json` 给出逐文件 SHA-256。分阶段提交与说明见
`m710id70_v3_stages.md`。
'''
    docs=ROOT/'docs/validation';docs.mkdir(parents=True,exist_ok=True)
    name='technical_qualification_report_m710id70_v3.md'
    (docs/name).write_text(report,encoding='utf-8');(out/name).write_text(report,encoding='utf-8')
    # Package generated evidence, excluding snapshots/caches and prior bundles.
    paths=[p for p in root.rglob('*') if p.is_file() and 'source_snapshot' not in p.parts
           and '__pycache__' not in p.parts and p.suffix in {'.json','.jsonl','.csv','.xml','.md','.png'}]
    evidence={'files':{p.relative_to(root).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}}
    bundle=docs/'evidence/m710id70_v3_evidence.zip';bundle.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(bundle,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
        for path in sorted(paths):archive.write(path,path.relative_to(root).as_posix())
        archive.writestr('evidence_manifest.json',json.dumps(evidence,indent=2))
    write_json(docs/'evidence/m710id70_v3_evidence.sha256.json',{'file':bundle.name,'sha256':hashlib.sha256(bundle.read_bytes()).hexdigest(),
               'bytes':bundle.stat().st_size,'evidence_files':len(paths)})
    print(json.dumps(summary,ensure_ascii=False));print('Evidence bundle:',bundle,bundle.stat().st_size,'bytes')


if __name__=='__main__':main()
