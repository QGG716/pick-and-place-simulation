# 空载接近直连分层修复与有限 A/B（2026-10-15）

**分层功能已接入真实 worker；本轮没有证明 c04 性能改善。** 四次独立单箱诊断均完整生成，但冻结版本的 c04 从 10.087147 s 退化到 30.474779 s；c02 为 1.552238→1.522763 s。

**版本限制：A/B 使用 `f34443996148062afccbbe7d1f02154b39698b09`。** 汇总时发现新分支把 `INVALID_GOAL_CLEARANCE` 错标为 REQUEST 中断，影响原有候选继续语义。已在 **`3ec35410d98a821b0f722e24c124e3764ee882a7`** 恢复 CANDIDATE 局部拒绝，并用先失败、后通过的回归覆盖。四次完整任务限额已用完，**最终补修版未再做任务级 A/B**；不能把下面的时间当作最终代码性能，也不能把 c04 全部退化归因于正确分层本身。

结果全部为 `PLANNING_ONLY_NOT_EXECUTABLE`、`qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`。

## 错位与最小修改

基线/远端均为 `b4e02a2f08310168eb683ce97d16f973b0199108`，Windows 和原 Linux 工作区均未覆盖；新目录为 `/work-approach-tier-20261015`。上层 `_connect_pose()` 已有惰性 IK 的直连轮和 deferred 轮，但适配层 `_native_plan(allow_ompl=False)` 仍无条件先插入完整 SPARSE_PRM；它会执行 Dijkstra、当前场景边检查及最多 6 s / 128 边的搜索。旧代码回归明确捕获该请求。

限定 planning-only/static_prior_fast/空载 pregrasp/FREE_APPROACH：
- 上层显式传递 DIRECT_ONLY / GLOBAL_ALLOWED。前者提交 CHECKED_JOINT_DIRECT，再保留 Pilz PTP；worker 复用原 `StaticPriorGraph::edge()`、原采样、Clearance/ProcessPolicy 和 IPTP/MTC 状态传播，直接分支不读取图节点/边、不建邻接表、不进入 Dijkstra 或 OMPL。
- deferred 轮才允许 SPARSE_PRM 和原有合法回退。6 s / 128 边、阶段/任务预算不变；先验超时保留原状态，仍允许原有 OMPL 回退。cold、持箱 transit、工艺 LIN 的求解顺序不变。
- 已拒绝的关节直连/PTP只在当前连接调用的租约内记录；键绑定完整请求语义，包括 q、模型、世界、目标/附着、策略、Task、parent receipt。全局请求引用实际失败 stage，worker 再核对原状态和上下文，避免重复同一直接边。超时/取消不成为几何失败缓存。没有跨任务或跨连接调用扩展缓存。
- selected_method 按实际来源区分 JOINT_DIRECT、PILZ_PTP、STATIC_PRIOR_REUSE、RRT_CONNECT；先验不记为 OMPL，检查直连的 native_solver_calls 仍为空。最小旧行为开关为 `--legacy-approach-layering`，只允许 diagnostic-input 使用。
- 前几轮全部优化、成功/失败聚合证据和 pair_profile_enabled=false 保留。没有修改几何、5 mm 净空、权限、采样、杯 mask 生成、候选数量或 IK 种子。

历史两次接近端点是同一目标姿态的不同腕部 IK 构型：FK 位置差 3.323e-7 m、姿态差 8.395e-7 rad，J4/J6 相差约 ±π。A/B 的前两个实际起点、目标 q 和连接 seed 也完全相同。未把不同抓取/工艺上下文混成一个候选池；后续门位和局部 LIN 仍由原 `_approach()` 组织。

## 两对实测

运行顺序事先固定 c04-A、c04-B、c02-B、c02-A；每项仅一次、新进程、新 Task。A 为同一二进制的旧分层开关，B 为新分层。c04 恢复已移除 c02/c01/c03/c00 的实际起点及 SLOW_COMPLETION；c02 在看到结果前选定，从当前配置初态、完整40箱和 FAST 开始。

| 任务/模式 | 完整轨迹 | 纯规划(s) | 在线交付(s) | 初始化(s) | 场景准备(s) | IK/PTP/LIN/OMPL |
|---|---|---:|---:|---:|---:|---|
| c04 A | 是 | 10.087147 | 10.167506 | 1.016039 | 0.042353 | 5/1/4/0 |
| c04 B（补修前） | 是 | 30.474779 | 30.541732 | 1.000983 | 0.034430 | 48/27/9/2 |
| c02 A | 是 | 1.552238 | 1.595607 | 1.006543 | 0.030897 | 3/0/4/0 |
| c02 B（补修前） | 是 | 1.522763 | 1.563552 | 0.998215 | 0.028120 | 3/0/4/0 |

初始化/场景准备不计入纯规划；候选、IK、失败和回退均计入。c04 多耗 20.387632 s；c02 少 0.029475 s（约1.90%），单次小样本不证明稳定收益。没有新的五箱成绩。

c04 A：首候选直接进入完整先验请求→超时 6.042227 s→PTP失败 0.104151 s→第二 IK 端点先验成功 1.526582 s。完整查图请求前没有其他端点的独立直连机会。

c04 B：首次完整查图请求前实际尝试 **23次端点直连（15组不同数值 q）**，其中同 q 可处于不同连接调用；不按 q 跨调用缓存。23次关节直连均拒绝，往返累计 **1.212703 s**；23次PTP均失败，累计 **2.947302 s**。另有1次目标端点拒绝 **0.046401 s** 和原有4次局部LIN失败 **0.149074 s**。这些端点通过当前 native 入口检查后才进入直连；目标端点拒绝那次不计入23次。

之后全局先验仍超时 **6.043058 s**：85次边检查、21,997个插补状态、58条几何拒绝、1条因截止时间中断。见证涉及 tool_rigid_58/carton_l04_c01、柔性波纹管/右墙；它们是搜索边见证，**不是把 TIMEOUT 改成目标不可达**。同上下文直接边已通过失败 stage receipt 排除，`direct_rejection_reused=true`、direct_connector 新拒绝次数为0。接近 OMPL 成功 **3.493575 s**。随后所选构型的持箱搬运尝试了5次失败先验、4次失败PTP，最终 OMPL **7.198714 s**；transit 总请求往返 **10.785136 s**，A 仅 **0.761287 s**。持箱算法未变，输入构型和实际搜索工作不同。

c02 B 首个 checked joint direct 成功 **0.082075 s**，没有接近查图/PTP/OMPL；A 的完整先验入口也以当前关节直连成功，往返 **0.083293 s**。两侧完整任务几何计数完全一致。

以上 request ID 均由本次 worker 产生，未写死历史编号；完整顺序、状态、耗时和计数见 [requests.csv](experiments/m710_approach_tiers_20261015/requests.csv)。

## 实际检查量与验证

| 项目 | c04 A | c04 B | c02 A/B（相同） |
|---|---:|---:|---:|
| Clearance 状态检查数 | 29,861 | 59,895 | 819 |
| Clearance 累计(s) | 7.813070 | 13.249098 | 0.150680 / 0.149271 |
| 工作区准备(s) | 0.027654 | 0.214270 | 0.022292 / 0.025320 |
| AABB检查 / 跳过 | 228,874,860 / 225,124,487 | 390,764,087 / 385,378,067 | 6,658,405 / 6,607,407 |
| 实际FCL查询（含补充） | 3,750,743 | 5,387,610 | 51,368 |
| 一次性补充查询 | 370 | 1,590 | 370 |
| AABB资格 / 复用 | 150 / 32,994 | 230 / 172,725 | 150 / 23,322 |
| 窄阶段资格 / 复用 | 370 / 198,277 | 1,590 / 2,336,463 | 370 / 50,778 |

统计按实际 request_id 去重，不累加父记录；四项均无重复冲突或缺失 Clearance。Clearance/solver 时间包含在请求往返内，不与任务墙钟再次相加。两侧 profiling 均关闭、对象对细项均为null，ProcessSealInput/输入预解析/两类固定关系复用均开启。完整路径末态、父链、目标和附着/释放边界的轻量核查通过；没有追加密集验证。

冻结前 **126项 Python 定向回归、3个原生测试程序通过**。覆盖两种直连能力、所有端点先直连再全局、内部碰撞、超时/取消、上下文失效、来源计数与正常模式隔离。原生 FCL 测试用“禁止读取的图节点/边”验证 direct-only 不进入图遍历；端点合法而中段有障碍仍拒绝。开发单边探针在官方模型真实 worker 中通过 MTC、IPTP，返回 NATIVE_DIRECT_CONNECTION；它不是完整任务。两次探针初始化失败（字段名、缺少杯几何）均保留且未产生轨迹；补齐探针输入后成功，未开展完整单箱任务，也未使用历史成功轨迹。

事后补修新增的“无效第一端点、合法第二端点继续”测试先失败后通过；最终 **127项定向 Python 测试通过**。原生源码/二进制未再改变。汇总器也允许在原生阶段尚未登记时保留端点失败，未伪造 stage_id。**没有第五次完整任务。**

## 身份、复现与结论边界

CPU Xeon Gold 6459C，实际 Python/worker 亲和性0–21，OMP/OpenBLAS=1；既有 Ubuntu22.04/ROS Humble、FCL0.7.0、MoveIt/Pilz2.5.10、MTC0.1.3、g++11.4，Release -O3 -DNDEBUG。依赖未升级，模型资产/数据库/输入在两侧相同。

worker SHA256：`46ac78b7f2367f205ae3c4886767b87de8a5c4e707b38b1a4af4e3d26eb5e98a`。数据库 SHA256：`df5b0c02c0566542b98607162d2c3bd61871fb0792950200f5ccb61b54754311`。正式源码与最终补修源码明确分开；源文件和中文报告均采用UTF-8，提交不同是版本身份不同。

[实际四次命令](experiments/m710_approach_tiers_20261015/run-bounded-ab.sh) · [冻结身份/预算](experiments/m710_approach_tiers_20261015/freeze.json) · [机器汇总](experiments/m710_approach_tiers_20261015/summary.json) · [逐任务CSV](experiments/m710_approach_tiers_20261015/tasks.csv) · [候选语义核对](experiments/m710_approach_tiers_20261015/candidate-semantics.json)。同目录四个子目录保留本次 timing、requests、完整实验轨迹和日志。复现需在独立目录检出正式源码并使用新的输出目录；本轮没有再次执行复现命令。只重新汇总：`python docs/experiments/m710_approach_tiers_20261015/analyze.py`。

**分层错位已修复，但当前数据不支持“c04更快”的结论。** 原有限候选/连接组织仍可能选择导致昂贵全局回退与后续搬运搜索的构型；还没有实现多候选时间片或抓取—出口—放置联合选择。最终端点语义补修后的完整任务成本尚未测量，不能强行归因或外推。没有变更宽阶段索引、几何算法、采样、执行合同；未运行新五箱/40箱、Isaac、真机、执行导出、preflight、重型资格检查或全量pytest。此处停止。
