# 合法释放后的理想接管与最高排连续验证

## 修复范围

基线为 `cced10c9ea2901f4110f28f7713bb77925664172`，运行源码提交为
`1d9f3fed5513197e4d66b116e224957ff06b6355`。本轮没有修改规划几何、运行碰撞净空、
释放高度政策、动力学、控制、插值或物理频率。上一轮第五箱的报告和原始证据保持不变。

旧代码在实际解除约束前后两次调用 `audit_runtime_short_drop`，随后
`begin_ideal_transport` 又把正在下落的当前箱体当作释放前箱体验收。
服务器旧实现复现：21.5 → 19.5 mm 被 `IDEAL_RELEASE_HEIGHT_OUT_OF_BOUNDS`
拒绝；计划 25 mm、实际释放 24 mm → 20.8 mm 被
`ACTUAL_RELEASE_REGION_MISMATCH` 拒绝。两例释放前验收均通过。

现在生产运行使用单任务 `IdealReleaseHandoff`：

- 解除约束前仍执行原实际状态验收：20–50 mm、计划位姿绑定、目标、接收区域与避碰。
  保存测量、实际障碍状态和完整验收结果；失败不生成释放凭据。
- 约束已移除且原独立性阈值通过后，凭据才可用于首次接管。凭据绑定世界、任务、
  目标、接收机、完整任务元数据和策略，不能由序列化的 `true` 恢复。
- 接管检查实际释放到当前测量的有限过渡，复用释放策略的 0.25 s 物理飞行窗口、
  重力、位置/速度不确定度、5 ms 预测步长、角速度及落地速度上限。
  这不是规划墙钟预算。检查当前完整投影、姿态、非穿透、释放至当前再至接收面的
  保守包络，以及非接收结构和当前活动箱体障碍；不再检查当前高度是否 ≥20 mm，
  也不再要求下落后位置距名义释放位置 ≤3 mm。
- `begin_ideal_transport` 的理想接收分支必须消费这一当前状态验收，并核对接收机几何
  和释放政策；物理接收分支继续要求真实接触和支撑。重复接管返回同一记录。
  运行器仅在第一次接管时切换同一刚体，碰撞关闭仍在独立性确认之后。

`actual_release_prediction` 始终表示解除约束前测量；`ideal_takeover_audit` 单独保存
接管测量。事件同时记录 `actual_release_height_m` 和 `first_takeover_height_m`；
兼容字段 `release_height_m` 明确表示实际解除约束前最低角点高度，不再表示接管高度。

## 定向验证

服务器沿用 `/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python`，
未安装依赖。最终修复测试：**128 passed in 18.10s**。
覆盖两种合法下降、释放边界拒绝、缺少释放/独立性证据、错误世界/任务/目标/接收机/
策略、过期和异常运动、当前障碍及穿透、重复接管、接管后下降；另复查物理接收、
接收顶面身份、step 1.1 续箱事件和原 4.958109 mm transit 拒绝。
测试调用生产释放、独立性、过渡和接管函数，未替换核心验收函数。

实际运行的定向模块：`test_ideal_release_handoff.py`、`test_poc_release_reserves.py`、
`test_poc_continuation_state.py`、`test_post_landing_transport.py`、`test_proof_of_concept.py`、
`test_adaptive_release_motion.py`、`test_m710_runtime_contact_policy.py`、
`test_obb_query_identity.py`、`test_same_world_continuation_runner.py`，均位于 `tests/`。
新增交接模块 18 个用例；未跑全量 pytest、104/129、40 箱清空或性能扫描。

运行根目录：`/root/autodl-tmp/m710-poc-handoff-20260918/`。
`evidence/old_transition_reproduction.txt` 保留旧行为；`evidence/targeted_final.txt`
保留最终测试结果。106 个源码/配置文件逐一匹配 Git SHA-256，见
`evidence/source_verification.json`。脚本编译检查通过。

## 本轮运行约束

从固定初始 40 箱新建一个世界，不导入历史完成事件。正常选箱决定顺序；历史 motion
仅提供经过当前实际起点和政策重验的候选。初始化静置后，使用既有实际高度聚类得到
最高排身份集合，写入 `initial_highest_row.json`；运行器禁止本轮任务越出该集合。
每箱结束后通过既有暂停世界/实际状态/离线规划/续箱请求链路继续，不重置机器人和箱体。

首箱使用现有首箱配额。续箱使用已支持的有限配额：连接 54,000 次、局部笛卡尔
采样 720 次；其余数值保持当前政策。没有总规划墙钟截止，也没有 shell timeout。
每份完整结果先完成 preflight、export 和 bundle readback，保存 `first_feasible`；
不追加节拍或腕部优化。录像维持 640×360、5 fps、正常物理时间。

## 同一世界结果：1/5，第二箱失败后停止

唯一新世界：`1789707683.8917766`。初始化实际 40 箱，静置 0.904167 s；最高排集合由
实际高度聚类得到 `carton_l07_c00`、`carton_l07_c01`、`carton_l07_c02`、
`carton_l07_c03`、`carton_l07_c04`。未导入历史完成事件。
正常选箱首先选择 c02，随后从该世界实际 q、39 个活动箱体及事件选择 c01。
第二段接线验证 `scene_restored_or_teleported=false`，没有任务间重置。
初始实际状态文件及 SHA 见 [initial_world_scope.json](evidence/m710_poc_legal_release_handoff/initial_world_scope.json)。

| 本轮目标 | 实际抓取 | 实际移除约束 | 理想接收 | 撤离完成 | 理想送出 | 名义 / 实际释放 / 首次接管高度 mm |
| --- | --- | --- | --- | --- | --- | --- |
| c02（第 1 个任务） | 1 | 1 | 1 | 是 | 1 | 25 / 24.575833 / 22.546142 |
| c01（第 2 个任务） | 1 | 0 | 0 | 否 | 0 | 25 / 未释放 / 未接管 |
| c00、c03、c04 | 未执行 | 未执行 | 未执行 | 未执行 | 未执行 | 无实际高度 |

合计：实际抓取 **2**、实际释放 **1**、新增实际物理接收 **0**、理想接收 **1**、
理想送出 **1**，完整流程 **1/5**。c01 仍附着，不计完成，也不进入下一任务。
停机状态保留 c02 的理想接收、已处理、已送出及非活动身份，实际接收集合仍为空。
这些数量全部来自本世界；上一轮 c04 的成功不计入本轮。

名义高度来自现有离散候选 25/35/45 mm；motion 保存的 FK 最低角点高度分别为
24.896611 / 24.941669 mm。精简摘要以“距复验 FK 高度 1 mm 内唯一离散候选”明确标注
名义高度的提取方式，没有把 FK、实际释放和接管高度混为一谈。
c02 在物理 t=89.204167 s 合法释放，t=89.225000 s 首次接管，间隔 20.833 ms；
实测独立性位移 2.109621 mm，约束已不存在。接管位置残差 0.423353 mm，小于
本次有限过渡包络 2.719271 mm；t=99.279167 s 整箱越过 X=-3.2 m，随后完成撤离。
该物理案例没有跨过 20 mm 下限；跨下限与超过名义 3 mm 的合法下落由生产交接定向测试覆盖，
不为再造这些工况重开世界。

## CPU 路径、暂停与耗时

两箱均先获得完整接近、抓取、脱垛、搬运、放置、释放、撤离路径，完成正常
preflight、export、bundle readback 后才交给 Isaac。首个完整结果保存到只读 `first_feasible`。

| 目标 | 完整路径节点 | 规划墙钟 s | preflight / export / readback s | 实际物理时间 s | 回放墙钟 s |
| --- | --- | --- | --- | --- | --- |
| c02 | 174 | 1273.247396 | 0.282796 / 0.286584 / 0.110029 | 102.375000 | 1409.384931 |
| c01 | 171 | 1405.267452 | 0.283636 / 0.327934 / 0.128375 | 40.379167（失败停止） | 451.744012 |

规划合计 **2678.514848 s**，分段物理时间合计 **142.754167 s**；初始化静置另计。
回放墙钟合计 1861.128943 s，不含离线规划、人工核查和交接配置修正的等待。
规划期间世界明确暂停，不能据此声称传送带不停或实时在线规划完成。

c02 历史提示经过全部阶段重新验证，共 35,527 次状态/边采样查询，174 节点。
c01 的 4 个历史提示因 `HISTORY_START_ADAPTATION_BOUND` 被拒绝，没有放宽起点适配。
随后正常候选池生成 72 个候选，实际搜索 1 个普通候选即得到完整解；剩余 71 个未搜索。
共 44,468 次轨迹状态验证、43,554 个边采样、263 次 RRT 迭代、160 个笛卡尔采样；
不把生成数称为搜索数。两箱均未追加腕部或节拍优化。

续箱首次 preflight 因工具默认执行配置仍引用原 motion policy，触发
`actual motion input policy mismatch`。当时没有发布续箱请求，世界继续暂停。
保留原失败记录，显式配对执行配置与本轮有限搜索配额配置后，在另一个 delivery 目录
重做正常 preflight/export/readback，全部通过；没有改写 motion、重新搜索或重开世界。
原 motion SHA 为 `76a934d8f2ec3aefbb364bdc4d4ac12374517b0f2a6fd552156ef843c74ea351`。
原 motion 的 `execution_ready=false` 是导出前字段；独立 delivery 的
`simulation_execution_ready=true` 与 readback PASS 才是交付状态，未给旧包重新贴标签。
两个提交的 `m710id70_handoff_continuation.yaml` 保存实际配对配置；除连接 54,000 和
局部笛卡尔 720 的有限配额外，其余规划和执行数值保持不变。

## 第二箱的可复现阻塞

c01 在 t=16.058333 s 实际抓取，t=40.379167 s 的 **extraction、仍附着** 状态停止。
失败世界累计时间为 142.754167 s，停止后实际 q（rad）为：

```text
[-1.171761393547058, -0.3911304175853729, -0.30697116255760193,
  1.6064982414245605, 1.1731631755828857, -4.804100036621094]
```

首个拒绝对象对为 `/Validation/Scene/carton_l07_c01` 与
`/Validation/Scene/carton_l06_c01`。PhysX 有限接触点 separation 为
**0.06643666 mm**，要求 **5 mm**，分类 `CLEARANCE_INSUFFICIENT`，没有接触许可。
外层历史停止标签为 `UNEXPECTED_ROBOT_OR_RIGID_TOOL_PROXIMITY`，但本次实际对象对是
箱体/箱体，不能据该标签称为机器人或刚性工具碰撞。

脱垛监测器在 t=40.375000 s 刚锁存 `free_space_reached=true`，下一物理步
（4.166667 ms 后）接触回调恢复正常净空规则并拒绝该报告。规划的 free-transit 边界门尚未通过。
固定停机状态离线复查显示：相同两箱实际姿态的 OBB 最短表面距离为 **5.388677 mm**，
也是全部剩余箱体中最近的一对。生产 `ActualStackContactMonitor` 的单次构型检查通过；
把原接触 separation 和原上下文交给生产 `classify_poc_runtime_pair`，仍重现
`CLEARANCE_INSUFFICIENT`。两条结论与原始数据均保留，未覆盖成成功。

已证实的是**自由空间转换后，接触点距离与停机几何最短距离不一致导致拒绝**。
未保存该步接触点坐标、法向及 manifold 特征身份；停机姿态也不是一份独立同步的
PhysX manifold 快照。因此不能据此确认接触缓存陈旧、CPU 代理误判、真实实体穿透或物理不可达。
没有降低 5 mm，没有恢复已结束的世界，也没有自动补跑剩余三箱。
下一步唯一优先问题是核清此转换处 **PhysX 接触点距离与当前物体最短距离的对应关系**，
先补齐同步接触特征证据，再决定是否属于测量解释/接线错误。

可独立离线复查的 [fixture](evidence/m710_poc_legal_release_handoff/second_carton_failure_fixture.json)
保存 q、两箱实际姿态/速度/尺寸、原上下文、策略和原始文件 SHA；
[复查结果](evidence/m710_poc_legal_release_handoff/second_carton_failure_review.json)
及 [重放脚本](evidence/m710_poc_legal_release_handoff/replay_second_carton_failure.py) 一并提交。
脚本只调用 CPU 生产查询与分类，不启动 Isaac、不伪造完整动力学重放。
服务器执行该脚本 PASS。真实驱动力矩遥测缺失仍单列为未评估，没有阻止本轮流程。

## 证据与录像

精简证据见 [证据索引](evidence/m710_poc_legal_release_handoff/README.md)。106 个源码/配置文件
在运行前后均与提交 `1d9f3fe` 的 Git 字节一致；实际使用的新增配置单独保存。
最终文档/配置/证据提交是该源码提交的后续提交，具体 SHA 以本报告所在提交及交付消息为准，
不宣称它重新执行了物理试验。

服务器原始根目录：`/root/autodl-tmp/m710-poc-handoff-20260918/`。

- 第一段录像：`repo/outputs/isaac_highest_row/replay.mp4`。
- 第二段失败录像：`repo/outputs/isaac_highest_row/segment_002/replay.mp4`。
- 两段的 `result.json`、`execution_events.json`、`actual_remaining_state.json`、
  `actual_frame_states.json` 和 `stack_contact_monitor.json` 位于各自录像目录。
- 完整 CPU 交付：`repo/outputs/initial_plan/first_feasible/`、
  `repo/outputs/plan_002_delivery/first_feasible/`；原续箱失败保留在 `repo/outputs/plan_002/`。
- 大日志、视频、原始状态和运行控制脚本只留服务器，不进 Git。

两段均 640×360、5 fps、1x 物理时间，逐帧解码通过：511 / 201 帧，视频时长
102.2 / 40.2 s，与对应物理时间相差不到一个视频帧。物理频率维持 240 Hz，
控制和接触监测频率未因录制降低。原 HUD 高度显示问题未作为重新试验理由；
本报告释放/接管高度取自实际事件与分别保存的验收证据。
失败后确认本轮 Isaac、规划器与续箱控制进程均已结束。
