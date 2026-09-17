# POC step 2：成对净空与有限墙体

## 身份和范围

基线：`1cdbee11a6652637b48acf7c30f9624ce25983d1`，操作前 fetch 后本地与远端一致，工作区干净。
分支仅为 `feat/v0.5-feasibility-core`。最终提交身份见本报告所在提交及交付回复。
服务器隔离目录：`/root/autodl-tmp/m710-poc-collision-step2-20260917/`。
操作前未发现相关规划/Isaac 活动进程；历史目录、视频和状态未覆盖。
沿用既有 CPU/Isaac 环境，无依赖安装。

新策略仅由 POC motion 配置引用独立的
`configs/validation/m710id70_layout_poc_pair_clearance.yaml`，旧显式配置不变。
未改变布局、初始关节、工具 CAD、质量/惯量、IK/FK、关节限制、速度、PD、力矩、插值、物理步长或监测频率。
step 1 的理想吸附/接收/送出及 step 1.1 的验证处理状态、事件单调性继续保留。

## 旧/新语义

| 对象对 | 旧规则 | 本轮 POC 原始碰撞表面总间隙 |
|---|---|---|
| 非豁免机器人自碰 | mesh 距离 ≤ 2×10 mm 拒绝 | 0 mm 附加净空；相交或接触仍拒绝 |
| 机器人—非豁免自有工具 | 2×10 mm | 5 mm |
| 机器人—外部环境 | 2×10 mm | 5 mm |
| 刚性工具—外部环境/箱体 | 两个 OBB 各膨胀 10 mm | 5 mm 欧氏表面间距 |
| 自由载荷—非预期环境 | 两个 OBB 各膨胀 10 mm | 5 mm 欧氏表面间距 |
| 脱垛进入自由空间 | 20.2 mm | 5.2 mm；失去自由净空阈值 5 mm |
| 释放杯—目标许可结束 | 全部 CONTACT_LOST | 全部 LOST 或当步有效实测间距达到 5.2 mm |

5 mm 不是每侧 5 mm。POC mesh 接口传 `margin=0` 和显式策略，同时传非零旧 margin 会拒绝。
旧 `margin_m=0.010` 保留作旧模式及保守粗筛参数，POC 最终窄相位不再叠加它。
正间距阈值比较有独立的 1 nm 浮点容差；零间距/相交不使用该容差放行。
新 collision policy 指纹：`91e8bebb851170e3620b5e2fc1bda49b5a72d1e62c8b3a3cd0dbed3c05d928e5`。
旧显式策略指纹仍为 `a4e1d53c75298fc0c460c4569f20e1cb30c05f43beeb1ac1e9f6c51574d5bb7a`。

独立储备保持：3 mm 实际脱垛储备、IK 残差储备、接触容差及原释放扫掠最小 10 mm。
接触/支持许可的既有压缩与阶段阈值未扩大；本轮没有新增允许刚性穿透量。

## 正式调用链核对

| 入口/阶段 | 几何、距离与阈值 | 许可与拒绝证据 |
|---|---|---|
| 初始/续箱准备 | 冻结场景 OBB 粗检，再由官方 mesh exact validator 验证实际 q | 底座安装/具名所有权许可保留；代理拒绝标注 proxy |
| 抓取 IK 终点 | 官方原尺寸 collision mesh 用 Coal collide/distance；工具保持原代理盒 | 完整杯密封、目标身份/压缩保留；`ROBOT_MESH_COLLISION` 等附分类与查询来源 |
| RRT/完整边 | 同一个 exact validator；最终全边新增更密插值复查 | 既有节点不删；4 m 保守杆臂乘六关节增量，单点位移采样上界 1.25 mm；不是通用连续碰撞证明 |
| 支持释放/脱垛 | stack tracker、运行时 actual stack monitor 使用原尺寸盒欧氏距离 | 既有箱箱阶段许可和扰动/进度监测保留；达到自由空间后恢复 5 mm |
| 带载转移/放置/撤离 | 载荷、刚性工具、机器人按各对象对检查；释放预测/支持仍独立校验 | 支持接触只对声明 receiver 和放置阶段；理想接收仍要求同箱、真实移除约束和交接区域 |
| 历史适配 | 历史路线仅作提示，当前附件、场景、每条边、释放与驻留重新验证 | 不复用旧 PASS 或旧间隙证明 |
| preflight/export/readback | motion policy、collision policy、冻结场景及源码身份绑定 | POC motion 与预检策略不符拒绝；bundle metadata 即使重算外层 hash 也不能替换绑定策略 |
| Isaac runtime | 现有 PhysX collider 的有效 contact separation，按所有权区分 self/external | 正间隙 header 按 5 mm 判断；无点/无效点沿既有 pending 机制，无法判定为 UNKNOWN；不清零失败计数 |

明确分类：`GEOMETRY_CONTACT_OR_INTERSECTION`、`CLEARANCE_INSUFFICIENT`、
`ALLOWED_STAGE_CONTACT`、`PROXY_GEOMETRY_INTERSECTION`、`UNKNOWN`、`CLEAR`。
关键拒绝保留 stage、物体/collider、geometry_scope、query_method、表面距离、要求间距、许可来源和策略指纹。
SAT 只用于相交判断和原有粗穿透保护，不把 SAT 轴间隙当欧氏距离、不把重叠量冒充可靠穿透深度。
OBB 最终距离由顶点—面与边—边特征最小值求得，并与 Coal 原尺寸 Box 的旋转样本对照。
当前工具 OBB 相交仍保守拒绝，但不宣称原始 CAD 已相撞。

J5/J6—自有工具的具名所有权豁免、柔性杯—具名非目标邻箱例外、SRDF 机器人对、安装面许可均保留。
未知对象没有新增豁免。PhysX 原 `contactOffset=0.010 m`、`restOffset=0` 保持，
运行时读回并验证其接触生成包络覆盖 5 mm；offset 不叠加进净空。

有限墙体直接来自冻结布局的 `trailer_left_wall`、`trailer_right_wall`、
`trailer_closed_end_wall`、`trailer_ceiling`，CPU 与导出使用同一组尺寸/姿态。
POC 跳过重复无限 Y 平面，检查完整连杆/工具/箱体对有限墙盒，中心越过开口不构成豁免。
真实地面、安装面、端墙、顶棚保留；没有移动墙体或创造通道。
策略/边界/几何身份进入状态和边上下文、motion 指纹、preflight、bundle 及源码摘要。

## 同构型离线复查

读取第五箱保存的实际场景与失败 motion（路径和 SHA 在 `docs/evidence/m710_poc_collision_step2/`）。
固定相同 q、障碍姿态、工具和 pregrasp 接触上下文：

| 保存构型 | 旧/新状态结论 | 最近刚性工具代理—环境对表面间距 |
|---|---|---|
| 第五箱实际起点 | PASS / PASS | `tool_rigid_0 / trailer_right_wall`，22.141173 mm |
| 保存的 pregrasp 终点 | PASS / PASS | `tool_rigid_0 / trailer_left_wall`，22.106400 mm |

上述是该工具代理集合的最近对，不冒称全机器人全对象的全局最近距离。
原失败记录为 `STAGE_CONNECTION_DEADLINE`，pregrasp，127 iterations；日志没有拒绝的 RRT 内部 q。
因此本轮没有证据把第五箱失败归因于旧净空/无限墙，也没有证明第五箱恢复。
没有伪造一个阻塞姿态替代原始数据；本次两个实际存档姿态没有发现仍相交的代理零件对。
可确认的纠偏来自定向几何用例：15 mm 旧拒绝/新接受、厢外不再被无限 Y 平面拒绝、旋转/对角盒使用真实欧氏间距。
4 mm、真实接触及代理相交仍拒绝，UNKNOWN 仍不放行。

## 验证结果

服务器 CPU 定向命令：

```bash
$CPU -m pytest -q tests/test_poc_pair_clearance.py tests/test_poc_continuation_state.py \
  tests/test_proof_of_concept.py tests/test_contact_simulation_policy.py \
  tests/test_m710_runtime_contact_policy.py tests/test_compliant_cup_contact_policy.py \
  tests/test_zero_point_contact_resolver.py tests/test_m710_drive_reference.py \
  tests/test_m710_replay_contract.py
```

最终结果：**156 passed in 4.96s**。包括真实 runtime 回调的 POC 分支、同 actor 内
柔性杯许可不能掩盖刚性插片，以及同配置路径下有效碰撞策略被替换时的预检拒绝。
执行脚本 py_compile 通过。
包括 4/5/6/15 mm、接触/交叠、自碰零附加净空、40 个确定性旋转盒 Coal 对照、有限墙完整包络、
旧策略隔离、cache 身份、runtime 分类、5.2 mm 释放 latch、策略不匹配拒绝和 step 1/1.1 回归。
几何单测不构成官方机器人整流程成功证明。

正式普通 CLI：历史首箱路线仅为提示，当前 POC 全路径重验，无默认业务墙钟截止。
预备 CPU 任务因冻结最终接线修复而正常取消，保留日志；没有先启动物理运行。
正式普通入口：

```bash
$CPU tools/run_m710_contact_unloading.py \
  --history-source /root/autodl-tmp/m710-ideal-outfeed-20260915-retry02/repo/outputs/ideal_plan01_final \
  --output outputs/poc_pair_canonical
```

规划 **PASS，1163.545573 s**；一个历史候选，29,693 次状态验证、29,798 个边内采样、0 个 RRT 迭代。
目标为 `carton_l07_c02`。这是一条原本成功的首箱路线的更密回归，不证明新搜索能力或第五箱恢复。
完整 motion 在预检前保存。随后预检拒绝执行配置仍引用旧布局文件；修正 POC 执行配置后，
还修复了合法 JSON 读回 tuple/list 表示差异，用规范摘要比较有效策略。
CPU 规划源码及保存的 motion 字节未改变，没有把旧策略证明重标成新策略。
最终通过真实 `build_m710_execution_preflight`、官方 `scripts/export_isaac_fanuc_replay.py`
和 `verify_m710_replay_bundle(..., project_root=...)` 继续处理已完整验证的 motion：
预检 **READY，0.283355 s**；导出 **PASS，0.283908 s**；读回 **PASS，0.083515 s**。
最终产物：`outputs/poc_pair_delivery_final/`，三件套另存 `first_feasible/`。
原 CLI 日志和配置不匹配失败保留在服务器，未覆写原 motion。
没有删除模式、场景、来源、内容摘要或策略一致性校验。

规划完成后才修改预检/执行配置并重新导出；物理运行使用最终代码和最终包。
21 个本轮源码/配置/测试文件与待提交 Git 索引逐字节一致，详见精简证据的 `source_scope`。
这一区分不改变 CPU 规划实现身份，最终 preflight/bundle 源码绑定覆盖实际物理运行源码。

## 唯一一次新世界 Isaac

使用最终 `outputs/poc_pair_delivery_final/replay_bundle.json`，已有官方模型 USD 仅作经校验的模型复用。
未恢复旧存活世界、未读入第五箱状态运行；本轮只启动一次、只运行首箱。
世界 ID：`1789633559.8247085`。初始化/静置通过，40 个箱体保留实际刚体身份；实际抓取接触杯数 40。
实际抓取、带载转移、真实约束移除和机器人撤离完成，`workflow_cycle_completed=true`。
`runtime_stop_reason=null`，`first_unexpected_runtime_robot_contact=null`。
实际自由空间 latch 在 29.841667 s 达到门槛；自由转移门禁在 30.808333 s 通过，等待 0 s。
89.200000 s 发出释放，89.220833 s 同箱体通过理想接收交接，99.275000 s 完整包络越过 X=-3.2 m。

| 本世界独立事件计数 | 数量 |
|---|---:|
| 实际接收 `completed_carton_ids` | 0 |
| 理想接收 `ideal_received_ids` | 1 |
| 工作流处理 `processed_carton_ids` | 1 |
| 理想送出 `handed_off_ids` | 1 |

三种理想/处理事件均对应 `carton_l07_c02`，没有写入实际接收集合。
因此 `physical_cycle_completed=false`：本轮没有完成实际接收及下游物理资格，不能将 POC 工作流成功改称完整物理接收成功。
独立 qualification 仅有 `joint_efforts_within_limit` 未通过，其详情为
`NOT_EVALUATED: joint effort ratios are unavailable`，不是测得了力矩超限。
这项既有遥测缺口没有被伪造为通过，也没有作为本轮执行前置条件。

运行时保留所有分类计数：robot 记录 1,254,909 次已批准阶段接触；payload 记录
1,260,928 次已批准阶段接触及 309 次 `CLEAR` 正间隙近接。两组有共同 robot/target 对，不可相加。
没有将近接 header 统一计为穿透，也没有清零失败计数。
259 个 authored collider、209 个 articulation shape、40 个 carton shape 的 offset 读回均 PASS，
仍为 contactOffset 10 mm、restOffset 0；5 mm 是验收间隙，不改变求解器。

录像经既有 OpenCV 离线读回（未启动 SimulationApp）：**640×360，5 fps，496 帧，99.2 s**，末帧可解码。
物理运行墙钟 1370.078583 s；录像按正常仿真时间，未加速物理时钟。
服务器原始证据目录：
`/root/autodl-tmp/m710-poc-collision-step2-20260917/outputs/poc_pair_single_world/isaac/`，
包含 `result.json`、`actual_remaining_state.json`、`replay.mp4`、offset 读回和初始化证据。
原始文件大小及 SHA-256 见 [精简摘要](evidence/m710_poc_collision_step2/final_step2_summary.json)。
录像及原始结果另下载到本地被 Git 忽略的 `.tmp/poc-step2/`，未提交大文件。

完成后仅进行离线状态载入：真实 `apply_actual_motion_state` 和生产连接器构造通过。
实际接收 0、理想处理 1 时采用 **1800** 次续箱连接预算；候选为 c00/c01/c03/c04，
已处理 c02 未重新进入候选。没有继续搜索或启动第二个世界。

本轮不执行全量 pytest、104/129、整排、40 箱清空或性能扫描。
首箱只用于回归，不作为第五箱搜索恢复或真机安全资格证明。

## 未闭合范围

工具仍使用既有代理 OBB，没有完成原 CAD 贴合重建；代理相交仍保守拒绝。
本次第五箱存档没有内部 RRT 拒绝姿态，无法指定一个经复现确认的反复阻塞零件对，
下一轮应保留这类 q/接触上下文，再决定是否需要细化相应代理。
本次密集检查耗时约 19.4 分钟，仅是单次实际耗时记录，没有进行性能扫描；查询成本可后续优化，
但不能以减少必要边内检查作为通过条件。力矩输出遥测仍是既有独立资格缺口，本轮未处理。
