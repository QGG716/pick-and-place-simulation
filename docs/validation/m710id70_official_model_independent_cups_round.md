# M-710iD/70 官方模型、独立吸盘与动力学验证轮次

日期：2026-09-10
分支：`feat/v0.5-feasibility-core`
冻结布局：`m710id70_unloading_layout_v1`

## 结论

本轮把固定提交
`FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`
中的完整官方 M-710iD/70 描述接入了 CPU 规划和 Isaac 适配链，并把皖泰吸具改为
`ideal_independent_cups`：72 杯具有稳定独立 ID，几何可用、指令激活和实际接触是
三个不同的 72-bit mask。完整的、有界的八阶段运动连接器以及实际接触后附着、
接收面支撑后释放、撤离后输送的执行门均已实现。

但是，动态取放 preflight 仍有三个独立的机器可读 blocker：当前初始位姿在严格
20 mm 成对净空下触发 `INITIAL_STATE_INVALID`；58 个 CAD-derived 工具 OBB 尚无
逐刚体向外包含证明；当前 J6—工具安装接触例外仍覆盖全部 58 个工具刚体，尚未收窄
为有尺寸依据的安装界面。规划器在任何 IK 或路径搜索前失败关闭。五个顶层任务均未
开始搜索，完整轨迹为 **0/5**，没有可执行的目标箱、抓取面、吸盘集合或回放 bundle。

Isaac 6.0.1 实际完成的是一个明确隔离的**初始化诊断**：官方机器人、20 kg 工具、
全部 40 个 42.5 kg 动态箱在重力下前向运行 6 s，记录 180 个正常时间倍率状态和
1920×1080 连续录像。最终 render-synchronized 运行状态为
`INITIALIZATION_ONLY_NOT_PICK_SUCCESS`，诊断分类为
`RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY`，`physical_pick_success=false`、
`attachment_count=0`。40 箱在 6 s 诊断中保持初始构型并在末窗静止，首帧、第二帧和
末帧目视一致；但 penetration gate 未执行，且该合同禁止运动执行和附着，所以它仍
不是取箱、带载、放置或完整动力学资格。

| 验证层 | 实际状态 | 可声称内容 |
| --- | --- | --- |
| 官方机器人资产 | `PASS` | 固定源、7 visual、7 collision、完整关节/帧/惯量和有限速度/力矩已核验 |
| 严格 CPU 初始状态 | `FAIL` | `J5_link`—`tool_rigid_13` 未达到 20 mm 成对净空 |
| 顶层 5 箱规划 | `NOT_RUN` | 初始状态失败后没有执行 IK、连接或候选筛选 |
| 动态取放 preflight | `BLOCKED` | 3 个 blocker：初始状态、工具 OBB 覆盖证明、J6 安装接触范围 |
| Isaac 初始化诊断 | `RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY` | 真实加载、有限驱动和重力前向运行；不包含抓取 |
| 物理取放 | **未完成** | 无附着、无带载、无支撑释放、无输送 |

旧 104-task 和 129-carton 集合没有在本轮运行，也不是该冻结布局的工程验收分母。
其任务数和场景定义均未修改。

## 官方 FANUC 模型与安装变换

官方资产归档在 `assets/robots/fanuc_m710id_70/official/`，来源和逐文件哈希记录在
[`provenance.yaml`](../../assets/robots/fanuc_m710id_70/official/provenance.yaml)。固定身份为：

- 上游仓库：`https://github.com/FANUC-CORPORATION/fanuc_description.git`；
- commit：`fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`；
- tag/package：`v2.3.0` / `fanuc_m710_description 2.3.0`；
- license：Apache-2.0，许可证文本随资产归档；
- 18 个固定源文件，共 8,657,343 bytes，逐文件 SHA-256 均由审计器核验；
- 7 个 DAE visual 和 7 个 STL collision mesh，碰撞网格合计 17,634 三角形；
- 确定性展开 URDF SHA-256：
  `2a813af47694819c44c888fe54e0041b045355ffb5273b012e3ff995b4999bcf`；
- SRDF SHA-256：
  `5991658df1a6035019b2f9bf3e0e8eccef3c4da201d472b25783301b13a5812a`；
- provenance manifest SHA-256：
  `adf419aed06c1b1ed8874dea98246ca25795e268fdd71515ff441d400dc72af6`。

关节链保持独立 `J1..J6`，没有把 J3 设为 mimic；`wbase`、`flange`、
`fanuc_flange`、`ee_link` 和项目 `tool0` 固定帧均保留。三组 Pinocchio/Coal 与
轻量链的一致性探针中，flange/TCP/Jacobian 最大绝对误差不超过
`4.44e-16`。

官方逐连杆质量之和为 580.347 kg；所有 COM 和完整惯量张量（包括非对角项）原样
进入工程动力学输入。六轴有限 effort 为
`[8000, 10000, 5000, 2000, 1000, 900] N·m`，有限速度为
`[π, π, π, 4.537856055, 4.537856055, 6.457718232] rad/s`。
这些官方公开字段已被选择为工程仿真输入；`simulation_input_accepted` 与
`machine_qualified` 分开记录，缺少机器认证不会把工程仿真永久锁死。

官方 base collision mesh 的局部 X 范围为 `[-0.3385, 0.225] m`。冻结布局采用
`world_from_mount.xyz = [-1.325, 0.35, 0.6] m`，因此真实 base 前缘保持在
世界 X = `-1.1 m`，没有用代理半径或修改布局包络替代重算。

## 皖泰工具与 72 杯独立选择

工具几何继续使用仓库已有皖泰 STEP 派生资产，并绑定 visual/collision/mass-property
哈希。当前规划表示使用启发式 58/144 实体分类后得到的 58 个复合 OBB；它尚无逐实体
物理语义和向外包含证书，因此只允许用于诊断与保守拒绝，**不能作为执行路径的
no-false-negative 接受证明**。完整 STL 用于本轮 J5 最近距离局部复核，但该 49 点 J6
扫描不是全路径、全碰撞对或刚体分类覆盖证明。柔性杯实体可视化，但不能因为柔性
外观而删除未经确认可变形的 insert、刚性安装板或绕过机器人—工具安全 margin。
此外，当前精确验证器将 J6 父连杆与全部 58 个工具盒排除在相互碰撞检查之外；这比
“只允许有尺寸依据的安装界面接触”更宽，因此明确记录为
`ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED`，在收窄并回归前不得取得执行资格。

`ideal_independent_cups` 的几何合同是 6×12、共 72 杯，pitch 为
`[0.048, 0.048] m`、杯半径 0.0215 m。每个杯具有稳定 ID、位置、分区和完整圆形
seal ring。对一个指定目标箱指定面，只有整圈均落在表面边界内、接近间隙与法向
均合法的杯才进入 `geometrically_eligible_mask`。默认命令全部几何可用杯，也允许
按稳定 ID 显式选择其非空子集。

完整轨迹和执行层保留三张相互独立的 72-bit mask：

1. `geometrically_eligible_mask`：目标面和请求/实际位姿下的完整环几何资格；
2. `commanded_active_mask`：本分支真正发出的独立杯命令，只能是 eligible 的非空子集；
3. `actual_contact_mask`：附着前根据**实际机器人 FK 和实际箱体位姿**重新计算，
   等于 commanded 与当前完整环接触的交集。

任一 commanded 杯若没有保留完整环实际接触，轨迹合同拒绝附着。该模式明确不使用
最小承载杯数、真空力、剪切、剥离、泄漏或 break-envelope 门；这只是本轮指定的
持有能力假设，不允许隔空附着、抓错箱、穿透、减轻 42.5 kg 箱体、使用无限驱动力
或瞬移箱体。

由于规划被初始状态门截断，本轮**没有成功目标箱、抓取面、杯 ID 或非空 mask**。
任何把 72 杯定义或测试夹具中的 mask 当成本轮实际抓取选择的表述都是错误的。

## 严格完整运动与执行管线

完整连接器已实现官方每连杆 triangle mesh（Pinocchio/Coal）及 58-box 工具工程表示，
全部 40 箱和冻结底盘/传送带几何。它要求从真实当前路径终点开始，生成并连续验证
八个阶段：

```text
HOME -> PREGRASP -> CONTACT -> SUPPORT_RELEASE
     -> EXTRACTION -> TRANSIT -> PLACE -> WITHDRAWAL
```

轨迹必须包含连续 6-DOF 状态、严格 FK、关节余量、全边碰撞、工具净空、目标身份、
实际物理接触和实际放置支撑审计。事件固定为 contact 终点 `ATTACH`、place 终点
`RELEASE`、withdrawal 终点 `RELEASE_RETREAT_COMPLETE`。相同 TCP 位姿的不同关节解
不能无路径切换；接触终点变化会重新计算实际接触和该分支自己的刚体相对变换。

搜索是有界、确定且惰性的。当前 seed 为 71070；单任务跨 face/roll/task-set
共享 12 次 pose-connection 尝试，每 pose 最多 4 个 grasp 分支；单阶段最多保留
3 个 IK 候选，并在该阶段共享 3 次连接、每次 600 次扩展；Cartesian 阶段最多
80 个样本，extraction 最多 5 个方向、0.8 m。先验证已有解，连接失败后才继续同一
有界种子流，不为每个候选复制整份昂贵规划预算。

执行适配的门控也已接通：

- 只有实际完整环接触通过后，才允许为同一目标箱建立基于实际相对位姿的 fixed joint；
- payload 不会在附着时 teleport，目标箱也不会从碰撞场景整体删除；
- 带载阶段继承实际刚体附着和接收面的合法支撑接触语义；
- 只有实际箱底与指定 receiver 的 gap、重叠、倾斜和穿透门均通过，才允许释放；
- 释放后先完成空载 withdrawal，才启动传送带；转角处同一时刻最多激活一个带面。

该管线“已实现、已由单元/合同测试覆盖”不等于工具碰撞表示或本轮轨迹已取得执行
资格。即使初始 home 问题解除，在逐刚体语义和向外包含证书归档前，工具资格门仍须
保持关闭。本轮 motion
evidence 中 `tasks_searched=0`，IK 调用、seed、迭代、候选、路径连接、状态/边验证和
RRT 扩展全部为 0；五个任务均为 `NOT_RUN_INITIAL_STATE_INVALID`。

## 初始状态阻塞证据

生产碰撞策略仍是每个独立碰撞体 10 mm margin，即不同体之间要求 20 mm 成对净空。
没有降低 margin，没有增加 J5—工具 SRDF 例外，没有修改工具偏移，也没有用简化代理
代替官方网格。

冻结初始关节位姿为：

```text
[3.113590129, 0.785456673, 0.649621871,
 0.067223496, -1.211750309, -1.941518523] rad
```

宽相和精确状态检查均首先报告
`ROBOT_RIGID_TOOL_COLLISION: J5_link vs tool_rigid_13`。只读诊断进一步执行：

- J6 从 `-π` 到 `+π` 的 49 点扫描；
- 完整、哈希绑定的工具 STL 与官方 J5 mesh 最近距离计算；
- seed 71070 的 256 个确定性关节随机样本；
- 保留相同 10 mm 每体 margin、完整 40 箱及固定布局。

完整 STL 扫描最小/最大距离分别为
`0.019126613227165135 / 0.019126613227165412 m`，均小于 0.020 m；最大净空缺口
为 `0.0008733867728345883 m`。简化安装板探针的距离约为 18.821092 mm。
256 个随机样本没有找到满足所有严格条件的 witness；其中 144 次仍由同一
J5—tool_rigid_13 对阻断。该结果是有界离散证据，`continuous_exhaustiveness=false`，
不能表述成全构型空间的数学不可行证明。但这组安装内净空与 J6 旋转无关的扫描结果
为当前 `J5_link`—`tool_rigid_13` 初始状态 blocker 提供了局部几何佐证，足以说明
当前冻结初始位姿不能被悄悄接受。它既不是连续全局证明，也不证明 58 个 OBB 对全部
工具刚体无 false negative，更不能替代 J6 安装接触范围的尺寸资格。

最终 dynamic preflight 明确保留三项独立 blocker，而不是把它们合并成一个：

1. `INITIAL_STATE_INVALID`：当前初始位姿下，官方 `J5_link` 与
   `tool_rigid_13` 未满足不变的每体 10 mm、成对 20 mm 工程净空；
2. `TOOL_RIGID_COLLISION_COVERAGE_NOT_PROVEN`：58/144 刚体/柔性实体分类尚无物理
   语义证书，58 个序列化 OBB 也没有逐实体向外包含证书；当前表示可以保守拒绝，
   不能据此接受执行路径；
3. `ROBOT_TOOL_MOUNT_CONTACT_SCOPE_NOT_QUALIFIED`：精确验证器目前忽略
   `J6_link` 对全部 58 个工具刚体的碰撞，没有可溯源尺寸将例外限定到真实安装界面。

因此正式结果是：

- motion：`BLOCKED`；
- complete trajectory：`FAIL_CLOSED / INITIAL_STATE_INVALID`；
- 五个顶层箱：`carton_l07_c02, c01, c03, c00, c04`，全部未搜索；
- dynamic preflight：`BLOCKED`，`simulation_execution_ready=false`，包含上述 3 个 blocker；
- replay trajectory：不存在，不能导出或运行物理取放 bundle。

解决这些 blocker 分别需要：可溯源的真实装配尺寸或经批准的安装适配件/垫片定义；
58 个工具刚体的语义分类和逐实体保守包含证书；以及只覆盖真实安装界面的、带尺寸的
J6—工具合法接触规则。猜测 spacer、修改 CAD 相对位姿、缩小 margin、扩大 SRDF、
忽略工具刚体或从碰撞场景删掉 J5 都不属于合格修复。

## Isaac 初始化诊断实跑

为区分“后端/资产能否实际初始化”与“取放轨迹是否合格”，本轮建立了独立
`m710id70_initialization_diagnostic_contract_v1`。该合同保留初始审计的 `FAIL`，并
硬编码 `motion_execution_permitted=false`、`attachment_permitted=false`、
`robot_pose_role=diagnostic_pose_not_motion_qualified_home`；重新哈希也不能把这些字段
改成执行授权。

在隔离目录和 Isaac Sim 6.0.1.0 中的最终实跑结果为：

| 字段 | 实测值 |
| --- | --- |
| status/scope | `INITIALIZATION_ONLY_NOT_PICK_SUCCESS` |
| initialization diagnostic result | `RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY` |
| backend initialization | `true` |
| physical pick / attachment count | `false / 0` |
| official robot + fixed tool mass | 600.347 kg |
| dynamic cartons | 40 × 42.5 kg |
| physics duration / step | 6.0 s / 1/240 s |
| state records / video frames | 180 / 180 |
| video | 1920×1080, 30 FPS, physical time scale 1.0 |
| reset joint-position error | 0 rad |
| reset carton-position error | `5.571041818e-08 m` |
| run-wide peak carton linear/angular speed | 0.0184131 m/s / 0.0353579 rad/s |
| 最大 authored-position 位移 | `9.03746846e-05 m`，`carton_l07_c03`，t=0.133333 s |
| last 0.533 s peak speed / drift | 0.0 / 0.0，`final_window_motion_stable=true` |
| initial configuration retained | `true` |
| full-run / final max joint tracking error | 0.02712375 / 0.02455294 rad |
| capture-induced state change | joint/carton 均为 0 |
| penetration qualification | `NOT_EVALUATED_DIAGNOSTIC_ONLY` |
| generated USD aggregate SHA-256 | `c7001b60ad6e2208a57a2c4d1244c52003702fe531263636542c95984ded3a7f` |
| MP4 SHA-256 | `4b3b74431d298179ca272de39a4d1b77957890472286a413929ae3c1f6c99cca` |

Isaac 使用官方 visual/collision USD、官方逐连杆惯量及有限 force-limited PD drive。
20 kg 工具惯量只合并进 J6 一次，机器人加工具质量为 600.347 kg；40 箱均作为独立
刚体。机器人初始关节位置只在初始化时写入一次，后续每物理步不再
`set_dof_positions`，也没有为箱体调用姿态设置 API。SRDF 只过滤官方相邻机器人
collider 对；7 个 imported collision instance 被显式 materialize 后逐 collider
过滤，随后增加的工具 collision 不继承该例外。

关节初值通过 USD PhysX joint state 在 `world.reset()` 前写入；reset 后六轴实测值与
命令值逐项一致，最大误差为 0 rad。40 个箱体 reset 后相对 authored pose 的最大位置
误差为 `5.571041818e-08 m`。渲染采用 zero-delta Replicator 预热和“物理步—Fabric
同步—zero-delta capture”顺序；正式采帧前后最大关节变化和箱体位置变化均为 0，
因此录像采集本身没有推进物理状态。

最终 180 行状态中，箱体相对 authored pose 的全程最大位移为
`9.03746846e-05 m`，全程线/角速度峰值为 0.0184131 m/s 和 0.0353579 rad/s；
最后约 0.533 s 的峰值速度和窗口漂移为 0，且
`initial_configuration_retained_throughout=true`。关节目标的全程最大跟踪误差为
0.02712375 rad，末帧为 0.02455294 rad。首帧、第二帧与末帧目视保持一致；三个
关键帧均来自与状态记录一致的正式采集路径。生成的 9 文件 USD 树也以
`c7001b60...ded3a7f` 聚合 SHA-256 绑定。

这些结果支持“官方模型、有限驱动、40 箱重力初始化在该 6 s 诊断中保持并落稳”，
但不支持更宽的执行结论：诊断没有执行 1 mm penetration gate，没有选择目标/抓取面/
吸盘，没有建立附着，没有带载、放置、释放或输送。其 scope 和状态仍明确是
`INITIALIZATION_ONLY_NOT_PICK_SUCCESS`。

## 动力学边界

本轮将官方公开逐连杆 inertial、joint velocity/effort 作为选定工程仿真输入，工具
质量/惯量采用项目工程模型，箱体保持 42.5 kg。材料、阻尼、PD gain、加速度、jerk、
纸箱摩擦和输送带参数仍是显式工程假设。机器认证状态单独为
`machine_qualified=false`，不与 `simulation_input_accepted=true` 合并。

由于没有合格轨迹，本轮没有测得或验证：带载关节跟踪误差、drive saturation、
关节力矩/反力、工具—箱接触力、搬运中滑移、放置冲击、实际 receiver 支撑、释放后
撤离或输送进度。初始化诊断也没有运行 attachment、support-release 或 conveyor
控制器。

## 复现

CPU 计算在隔离服务器目录
`/root/autodl-tmp/m710-official-dynamics-20260910` 中执行；环境为 Python 3.12.3、
NumPy 2.3.2、Pinocchio 4.1.0、Coal 3.0.3、pytest 9.1.1。Isaac 为 6.0.1.0，GPU
为 NVIDIA RTX PRO 6000 Blackwell Server Edition。以下命令从仓库根目录执行；
路径按本地输出目录调整即可。

```bash
RUN_ROOT=/root/autodl-tmp/m710-official-dynamics-20260910
cd "$RUN_ROOT/repo"
RUN_OUTPUT=outputs/m710_official_dynamics_20260910_final_v2
export PYTHONPATH=src

"$RUN_ROOT/cpu-venv/bin/python" tools/run_m710id70_layout_single_carton.py \
  --config configs/validation/m710id70_layout_v1_single_carton.yaml \
  --output "$RUN_OUTPUT/single_carton_motion_audit.json"

"$RUN_ROOT/cpu-venv/bin/python" tools/probe_m710_layout_initial_state.py \
  --config configs/validation/m710id70_layout_v1_single_carton.yaml \
  --output "$RUN_OUTPUT/initial_clearance_probe.json" \
  --wrist-samples 49 --random-draws 256 --full-tool-mesh

"$RUN_ROOT/cpu-venv/bin/python" tools/prepare_m710id70_dynamic_execution.py \
  --config configs/simulation/m710id70_dynamic_execution_v1.yaml \
  --motion-result "$RUN_OUTPUT/single_carton_motion_audit.json" \
  --output "$RUN_OUTPUT/dynamic_execution_preflight.json"

"$RUN_ROOT/cpu-venv/bin/python" tools/prepare_m710_initialization_smoke.py \
  --validation-config configs/validation/m710id70_layout_v1.yaml \
  --dynamics-config configs/simulation/m710id70_official_dynamics_v2.yaml \
  --output "$RUN_OUTPUT/initialization_contract.json"

"$RUN_ROOT/cpu-venv/bin/python" tools/archive_m710id70_dynamic_execution_evidence.py \
  --motion-result "$RUN_OUTPUT/single_carton_motion_audit.json" \
  --preflight "$RUN_OUTPUT/dynamic_execution_preflight.json" \
  --output "$RUN_OUTPUT/run_summary.json" \
  --source-base-head a372c7f117e61509229b2c15eef344cd7908d499 \
  --source-branch feat/v0.5-feasibility-core
```

Isaac 初始化诊断在已明确接受 NVIDIA EULA 的隔离环境中执行；下面给出本轮服务器上
使用的隔离 runtime/cache/output 路径：

```bash
RUN_ROOT=/root/autodl-tmp/m710-official-dynamics-20260910
cd "$RUN_ROOT/repo"
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libgcc_s.so.1 \
OMNI_KIT_ACCEPT_EULA=YES \
XDG_CACHE_HOME="$RUN_ROOT/runtime/cache" \
XDG_CONFIG_HOME="$RUN_ROOT/runtime/config" \
XDG_DATA_HOME="$RUN_ROOT/runtime/data" \
/root/autodl-tmp/envs/isaacsim-clean/bin/python \
  scripts/isaacsim_m710_initialization_smoke.py \
  --contract outputs/m710_official_dynamics_20260910_final_v2/initialization_contract.json \
  --project-root "$RUN_ROOT/repo" \
  --usd-directory "$RUN_ROOT/isaac_usd" \
  --output "$RUN_ROOT/isaac_outputs/initialization_render_sync_logged_final" \
  --seconds 6
```

在用户服务器的现有 Isaac Python 发行版中，还以进程级 `LD_PRELOAD` 指向系统
`libstdc++.so.6` 和 `libgcc_s.so.1`，只修正其宿主 Conda runtime 的 ABI 搜索顺序；
没有修改、覆盖或复用服务器现有环境。完整 CPU 回归命令为：

```bash
PYTHONPATH=src "$RUN_ROOT/cpu-venv/bin/python" -m pytest -q
```

最终服务器全量回归为
**468 passed, 1 skipped, 4 deselected in 37.62 s**；覆盖官方模型、工具门控、初始化
与渲染同步的定向集合为 **160 passed, 1 skipped in 10.90 s**。日志均随证据目录归档。
唯一 skip 是纯 CPU venv 中不可用的 `pxr` runtime；同一 collision-policy helper 已在
实际 Isaac 导入的官方 USD 上核验。4 个 deselected 来自仓库既有显式选择策略，
不是本轮隐藏失败。

## 证据与身份

本轮轻量证据位于
[`evidence/m710id70_official_dynamics_20260910/`](evidence/m710id70_official_dynamics_20260910/)。
核心身份为：

| 对象 | 身份 |
| --- | --- |
| layout fingerprint | `82ec898fef66cde35ee88108828ef80ce45047d290eef19d77b84bfb38ae60ec` |
| motion policy fingerprint | `400aaf60839b48d76d7aef3fd128c56b15b829df536a6a338dd52bd387a11e0b` |
| motion evidence fingerprint | `26001900fdc637943b276566e069e639ee0183bad60a818b45c2b69b99fb947c` |
| dynamic preflight fingerprint | `f5071f67ca9895acbf2b965ce4e77850bbf7acbfe741e84c1595c21f2f996810` |
| initialization contract fingerprint | `ab0465e967ffee0d2b2a648e7fe077c8f37285a7be65f0bc1202b00906695933` |
| generated USD aggregate SHA-256 | `c7001b60ad6e2208a57a2c4d1244c52003702fe531263636542c95984ded3a7f` |
| run summary fingerprint | `fb4bbb7126f944c1d541f9cc5bab10f1b3b6a032130dce00b79db90e461f086d` |

`run_summary.json` 记录了实际计算基础 HEAD
`a372c7f117e61509229b2c15eef344cd7908d499`、分支、运行环境、参与源码哈希和生成命令。
服务器是内容校验的 source copy，因此 `metadata_source=declared_source_copy`、
`dirty=true`，不冒充 clean checkout；motion/preflight 的互相身份检查为 `PASS`。
上述 base HEAD 是参与计算的代码基础身份，不是本轮最终提交 SHA。

## 明确未完成项与下一步

本轮未完成且没有声称完成：

- 严格可执行的单箱完整轨迹；
- 成功的 target/face/cup IDs 和三张实际非空 mask；
- Isaac 中实际接触后附着、42.5 kg 带载搬运、接收面支撑、释放和撤离；
- 释放后的横带/纵带输送；
- 完整 pick-and-place 正常时间录像、动作关键帧和带载 actual-state/force 日志；
- 完整 penetration、带载跟踪误差、驱动饱和和接触力资格。
- 58/144 工具实体分类的物理语义确认，以及所有刚体实体的向外 OBB 包含证书。
- J6—工具碰撞语义从整连杆忽略收窄为有尺寸依据的安装界面合法接触。

进入完整取放前必须分别解除三项 preflight blocker：用可溯源安装尺寸解释并解除
约 0.8734 mm 的严格净空缺口；归档 58 个刚体实体的语义分类及逐实体向外 OBB 包含
证明；把 J6—工具碰撞例外收窄为有尺寸依据的真实安装界面。完成后应使用同一冻结
布局、官方完整模型、经逐实体证明的保守工具碰撞、72 杯独立实际接触重检和现有有界
连接器，先在五个顶层箱中恢复一条完整轨迹，再启动 Isaac 实际取放。不得以 margin、
SRDF、工具偏移、代理几何、箱体删除、瞬移或无限约束制造成功。
