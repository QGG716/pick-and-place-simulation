# POC step 3.1：全链路规划余量与 20–50 mm 理想释放

本轮目标为原第五箱 `carton_l07_c04`。初始 fetch 后本地/远端均为 `2387448458d0e9298140f55a7c99315386373b24`，工作区干净，服务器无遗留搜索或 Isaac 进程。沿用已有 CPU/Isaac 环境，没有安装依赖。

## 输入与证据边界

使用原始未附着实际存档 `m710-ideal-outfeed-20260915-retry02/delivery/segment_004_actual_remaining_state.json`，SHA-256 为 `a7063b72dccf396e6a5ccaa24ffd2e1f12f599f7b7773153e081e95a7988f72e`。实际 q、37 个活动箱体、历史实际接收 4/理想接收 0/已送出 3 保持原来源；仍在接收区的 c00 保留。未将上轮仍附着的失败末态变成 between-task 起点。

历史候选来自上一轮只读 `first_feasible/motion.json`，SHA-256 为 `52ac4db8a2987bcc1f9d4898dffe8c3e6643f6baea68eda750e8d9257ba302f1`。旧 motion、bundle、失败日志和录像不改写。此次历史来源只提供关节路径、目标接触、接收 XY/姿态意图；所有采用的前缀和新后缀均通过当前生产验证器。

CPU 搜索隔离目录：`/root/autodl-tmp/m710-poc-step31-20260918`。后续执行绑定校验在 `m710-poc-step31-v2-20260918` 补齐后，执行证据版本固定于 `m710-poc-step31-v3-20260918`；运行中的源码不覆盖。CPU 源码提交 `d3905098ec537de88fee4cad8ea4e7b99cc82c52`，执行源码提交 `53537b783ba55e3e3054e91aab692e656a33767d`，中间提交 `4ff0726` 增加显式执行策略绑定。最终交付提交见 Git 记录。

执行目录的 35 个 CPU 实现文件及 22 个执行实现文件与 Git 提交逐字节一致，CPU 文件也与搜索目录一致。额外检查的 150 个源码/配置文件中，仅不在这两组实现身份中的 `scripts/run_demo.sh` 有 CRLF/LF 差异，换行标准化后相同；没有以换行标准化绕过生产源码指纹。

## 生效余量清单

下表数值均为本次 POC 配置。`p=0.1 mm` 是既有 IK 位置容差，`a=0.0002 rad` 是既有 IK 姿态容差，`c=0.2 mm` 是接触数值容差；这些数值不作为真实执行误差储备。

正式入口为 `tools/run_m710_contact_unloading.py`，经 `src/unloading_sim/layout_single_carton.py` 构造连接器。下表未另标模块的私有函数均位于 `src/unloading_sim/layout_trajectory.py`；历史适配为同目录 `history_candidates.py` / `history_adaptation.py`，接收预测为 `release_motion.py`。执行绑定由 `m710_execution.py` → `isaac_bridge.py` → `m710_replay_contract.py` 消费；实际状态门禁位于 `m710_replay_physics.py`，PhysX 事件接线位于 `scripts/isaacsim_fanuc_replay.py` / `isaac_collision_policy.py`。没有新增另一套规划器或执行器。

| 入口与函数 | 修改前 | 修改后与检查范围 | 分类、保留的许可 |
|---|---|---|---|
| 正常 CLI → `_build_automatic_trajectory_connector` | POC 普通/续箱预算与原接触策略 | 同入口消费高度政策及 3 mm 定向储备；实际接收/理想处理后的续箱判断不变 | 无业务墙钟截止；本次显式有限续箱迭代 54000、局部 Cartesian 720，不改分支默认预算 |
| `_approach` → `_connect_pose` → `_cartesian` | 终端距离下限 `5+c+2p=5.4 mm`，另与工具投影深度/4及历史 standoff 比较 | 下限 `5+3+c+2p=8.4 mm`；仍取原 `max`，因此几何深度主导时不会机械增加 3 mm；实际 FK 自由端点的工具/目标至少 8 mm，工具 provider 包含刚性件及杯碰撞体 | 3 mm 是接近自由端点工程储备；自由前缀整边仍走原门槛，随后的终端弧保留合法杯接触 |
| `_support_release` | 严格分支 lift 使用 `5+2c=5.4 mm` | 当前 POC 允许阶段性脱垛接触，因此该分支返回无强制 lift，储备由脱垛出口承担；旧显式物理模式保留 | 不给合法初始支撑/滑动强套自由空间余量，不把未生效公式当本轮修复 |
| `_extraction_options` | `max(5+c,5.2)+3=8.2 mm`；生成 guard 为 `2p+2‖half_extents‖a+c≈0.5562 mm` | 保留同一 3 mm 储备与生成 guard；新增转向/IK 后实际箱体出口至少 8.2 mm 的复核，历史出口亦检查 | 储备只针对箱体/邻箱；工具、机器人维持原合法性与身份检查，不宣称工具自动多出 3 mm，也不再次叠加储备 |
| 自由及带载搬运 `_checked_state_failure` / `_path_failure` | 外部物体对 5 mm、自碰零附加净空 | 运行门槛不变；`transit` 中载荷/接收机的规划要求为 **8 mm**，进入真实状态和全边检查，不只检查端点 | 仅定向接收机对加 3 mm；墙、其他物体及允许接触规则不全局膨胀 |
| 接收区 `_finish_place_branch` | `receiver_clearance=5+c+2p=5.4 mm`；零释放高度会先降至这一层 | 接收构造下限 **8.4 mm**；`preplace += max(0,8.4 mm−release_height)`；默认 25/35/45 mm 均高于该值，因此不会先降到 5.4 mm 再抬高 | 接收区规划储备与实际释放范围是不同约束；搬运中间位置可高于 50 mm |
| POC 释放候选 | 顺序约 0/25/49.6 mm，首解可能优先选零高度 | 实际范围 **20–50 mm**；独立 5 mm 高度储备，名义候选 **25/35/45 mm** | 0/10/15 mm 不作为 POC 候选或回退；20/50 mm 几何边界允许，数值比较 epsilon 仅 1e−12 m |
| `history_adaptation.adapt_branch/loaded_suffix` | 高度只查上限，旧低高度末段可进入新结果 | 原落点 XY/姿态投影重验；高度改由正常候选生成。载荷历史前缀保留到接收顶面至少 **53 mm** 的提前位置，再经当前 IK/Cartesian/全边验证重建后缀 | 53 mm 是后缀重建的提前位置，不是实际释放高度上限；旧 RELEASE/PASS 不继承 |
| `ideal_reception_region` / `predict_release` | 最低点 −1…50 mm；同时将 50 mm 传给支撑共面容差 | 最低角点 20…50 mm；独立复用原多边形覆盖、5° 底面姿态、水平且共面的指定接收面检查；连续垂直包络排除非接收结构相交 | 不生成实际支撑/着带证据；轻微倾斜较高底角超过 50 mm 不再误判高度不合规；严格物理接收算法不改 |
| `_departure_search` / residence | 投影逃离距离使用 `5+c=5.2 mm`，最终通过运行阈值即可 | POC 目标用 `5+3+c+2p=8.4 mm`；最终自由驻留端点的全部工具碰撞体/释放包络至少 8 mm；所有弧、完整包络和驻留重验 | 撤离过渡中的合法杯接触仍保留，8 mm 用于脱离后的自由端点；理想接管后的整带运输不重新变成物理资格要求 |
| `release_flight_envelope` | POC `flight_time=0` 只返回释放箱体 | 保留从释放到理想接收姿态的整个连续垂直包络，供撤离检查 | 这是规划包络，不是箱体放大或运行碰撞体改变 |
| preflight / export / readback | 已有 motion、场景、源码、配置指纹和释放几何复核 | 保留全部绑定；preflight 显式比较配置与 motion 的高度政策/储备；bundle 继续绑定该策略证据 | 旧包不得重新贴标签；实际释放与预测仍区分 |
| runtime 释放前门禁 | 当前实际位姿/接收区域检查，但高度下限允许贴带 | 解除约束之前调用当前政策，以最新实际箱体全部角点最低 Z 检查 20…50 mm，保留既有 3 mm 位姿绑定和姿态/覆盖/占用检查 | 指令合格不代替实际合格；高度不合格保持约束；没有测量夹紧或事件补造 |
| runtime 接收顶面身份 | 根 actor 与 `TopCollision` actor 可能匹配不一致，物理接触 helper 有宽泛子路径匹配 | 从创建的顶面建立明确 `TopCollision → receiver root` 所有权映射；载荷身份、声明接收面、双方顺序及 stage 共同决定既有许可 | 侧框/支腿/未知子物体、机器人/工具不继承；transit 不提前变成 place。这不是上次 transit 停止的已证实直接原因 |

`preplace_standoff_m`、`withdrawal_distance_m`、`post_release_vertical_lift_m` 的历史字段没有构造本次强制中间站；不把只出现在预算/证据中的值当成执行距离。状态缓存键、同请求上下文及完整任务证据加入实际消费的储备，避免换策略命中旧缓存。源文件指纹继续自动更新，没有移除任何绑定检查。

## 储备依据与局限

按上一轮每个已保存帧的同一时刻对齐 q 指令、实测 q 和实际箱体姿态，用完整对应角点位移纳入旋转影响：接收区 transit 样本最大“指令名义箱体 → 实际箱体”差为 **1.213572 mm**；全部已附着保存样本为 **1.506681 mm**。同帧实测 q 名义箱体 → 实际箱体最大约 **0.336151 mm**。3 mm 定向储备和 5 mm 高度储备是本次工程假设，不是将不相关最大值相加，也不是只补最后 0.041891 mm。

这些帧不是每个物理步，观测最大值不代表全局误差界。旧停止时 PhysX 4.958109 mm 与保存姿态 OBB 4.905988 mm 使用不同表示；指令 q 名义附着 5.804531 mm、实测 q 名义附着 5.155768 mm 也不能直接当精确可加的因果分解。原 4.958109 mm 记录仍被当前 5 mm 分类器拒绝。

旧录像帧可重建每帧有效附着变换，但不能精确还原抓取瞬间的测量变换；实测 q 名义箱体与实际箱体的差不能全部叫作“保持误差”。本轮新增 `actual_attachment_capture` 事件，记录建约束同一状态的实际接触平面、箱体及相对变换，无中间物理步。

实际释放前检查、实际约束移除、首次理想接管分别记录时刻及高度；移除时高度来自同一预移除状态，两者间没有物理步。理想接管后按原连续路由下降和送出，不再拿 20 mm 下限阻止合法下降。

## 定向验证与交付结果

服务器 CPU 定向测试 **156 passed, 20 skipped（19.94 s）**。20 项跳过来自旧历史 fixture 不可用，不能计为通过；真实 c04 历史入口另行实际重建。最后执行证据记录增补后，相关 47 项测试通过（14.63 s），Isaac 适配脚本编译检查通过。测试覆盖实际构造的接近/脱垛/搬运/撤离储备、普通/历史/导出/runtime 高度门禁、零位移 place 时序、精确顶面身份、缓存、冻结几何及 step1.1 状态语义。

新增回归位于 `tests/test_poc_release_reserves.py`；所选既有测试为 `test_adaptive_release_motion.py`、`test_default_history_adaptation.py`、`test_proof_of_concept.py`、`test_m710_runtime_contact_policy.py`、`test_m710_archive_initialization.py`、`test_m710_replay_contract.py`、`test_poc_continuation_state.py`、`test_obb_query_identity.py`、`test_poc_pair_clearance.py`。小型构造用例调用实际 IK、Cartesian、状态/边验证及阶段生成，未把所验证的核心函数替换成恒真。

未跑全量 pytest、104/129、40 箱或性能扫描。固定场景、CAD/质量惯量、IK/FK/Jacobian/关节范围、PD/力矩、速度插值、PhysX offsets 和监测频率保持原值；真实驱动输出力矩缺失独立记录。

## 完整 CPU 任务与新后缀

正常 CLI 的第一个历史候选实际完成搜索及全链路重验，未访问后续候选、未运行新的 RRT 轮次，也未借用旧 PASS。候选 ID 为 `0afb65f4664749590790574589cc6b82f852c7c244908596a9cddc2f09f714d6`，实际下发 seed 为 184247796。有限续箱连接额度 54000、局部 Cartesian 720 已配置，但没有把配置额度写成实际访问量。CPU 墙钟为 **2903.313 s（48.39 min）**，无业务性墙钟截止，首次完整可行即停止搜索。

结果共 **251 个节点**：pregrasp `[0,48]`、contact `[48,51]`、extraction `[51,92]`、transit `[92,224]`、place `[224,225]`、withdrawal `[225,250]`。历史带载 133 个节点中的前 126 个保留并逐边重验，从最低角点高于顶面 **55.157528 mm** 的位置重建其后 7 个节点；撤离按当前释放姿态重新生成。接触及附着重新计算，原实际起点仍是第一个节点，未回到首箱 home。

实际完整边采样：接近 58328、接触 718、脱垛 6949、带载 31904、place 3、撤离 5774，总计 **103676**；另有任务起点、接触端点、支撑释放和驻留状态检查。此次成功候选没有拒绝构型，不能拿其他历史失败当本轮阻塞。

选中第一层名义候选 25 mm，实际 CPU FK 最低角点高度 **24.999959 mm**。旧历史释放高度约 0.004344 mm，没有被继承。preplace 与 release 同位姿；独立阶段、释放保持和事件仍存在，未把低位 transit 改名 place。

正式 preflight、export、bundle readback 均 PASS。新只读文件位于服务器执行目录 `outputs/recovery_delivery/first_feasible/`，文件权限 0444、目录 0555：

| 文件 | SHA-256 |
|---|---|
| motion.json | `4e0a82a4866126c3f4d4f53429c6604ae3f59e16dd5da06c62580de05b2839a4` |
| preflight.json | `f8bbd1636c82c443b81c76ea919e9e596569661168286b9bf6e72389ff87c8327` |
| replay_bundle.json | `34fb623deb4b1527029a056502434c1627620a674491afaf12bc338027a380f42` |

新后缀敏感性复查使用上一轮接收区同帧 153.0 s 和 163.8 s 样本，分别施加该帧 `q_actual−q_command` 与同帧有效附着相对名义附着的变换。每组使用与生产 `_path_failure` 相同的密集边采样公式，检查接收区前导段、新后缀及 place，共 3060 个样本；没有把两个样本的最大值相加。

| 条件 | 横向接收机最小净空 mm | 纵向接收机最小净空 mm | 释放高度 mm | 接收区域 |
|---|---:|---:|---:|---|
| 名义 | 26.907337 | 24.999959 | 24.999959 | PASS |
| 同帧最大综合偏差样本 | 25.835331 | 23.760819 | 23.760819 | PASS |
| 同帧最大有效附着差样本 | 26.693782 | 24.859981 | 24.859981 | PASS |

新后缀对应箱角最大位移分别为 1.327168 mm、0.445308 mm，包含旋转效应。这是对两组已观测偏差的局部复查，不是所有碰撞对或未来工况的完备鲁棒性证明。真实运行仍执行原碰撞、关节、附着及释放门禁。

## 唯一一次新世界物理验证

**本次 c04 工作流完整完成，运行停止原因为空。** 只执行了一次对应新任务的 Isaac 循环，未追加试验或运行其他待抓箱体。输出目录为服务器执行目录下的 `outputs/isaac_step31_once`。

新世界身份 `1789701695.800743`，原世界身份 `1789448839.7527404`。按原存档初始化后静置 PASS，37 个活动箱体的实际初态绑定通过，无运行中关节重置或箱体瞬移修正。历史实际接收 4、理想接收 0、已处理 4、已送出 3 仅作初始化上下文；仍在接收区的历史 c00 本次完成剩余送出，也不计为新目标 c04 的成果。

| 本轮 c04 新增事件 | 数量 |
|---|---:|
| 实际抓取 | 1 |
| 实际释放 | 1 |
| 实际物理接收 | 0 |
| 理想接收 | 1 |
| 理想送出 | 1 |

`workflow_cycle_completed=true`、当前 POC 的 `simulation_execution_qualified=true` / `execution_qualified=true`。`physical_cycle_completed=false` 保留，因为本轮没有声称实际物理着带及下游物理接收资格。

以下均使用本次循环的局部仿真时间；运输事件保存的是会话时间，减去初始化偏移 452.7625 s 后列在表中：

| 事件 | 局部仿真时间 s | 最低角点高度 mm / 结果 |
|---|---:|---|
| 实际接触通过并建抓取约束 | 47.408333 | 同一目标及实际接触 mask 已核对 |
| 实际自由搬运门禁 | 73.020833 | PASS，等待 0 s |
| 释放前最新实际状态检查 | 165.004167 | **24.549831 mm**，PASS |
| 实际移除约束 | 165.004167 | **24.549831 mm**，同一预移除状态，无中间物理步 |
| 首次理想接管 | 165.025000 | **22.497002 mm**，实际独立性确认后接管 |
| 理想送出整包络越过 X=−3.2 m | 175.079167 | 实测 max X=−3.201000006 m |
| 完成撤离并结束循环 | 178.025000 | 完整指令计划已执行 |

20 mm 下限仅在释放/首次接管门禁使用；获准接管后，同一箱体连续下降到接收面再沿 −X 送出，没有用下限阻止下降或补造实际接收事件。

459 个保存的已附着 `transit` 帧中，c04 / 横向接收机最小 OBB 表面距离为 **26.193723 mm**，c04 / 纵向接收机为 **24.287181 mm**，均出现在局部仿真时间 164.8 s。这是 **5 fps 保存帧的实际位姿观察最小值**，不是所有物理步或所有物体对的全局最小值；完整物理步仍由既有 PhysX/运行门禁监测。`first_unexpected_runtime_robot_contact=null`。运行 `poc_pair_queries` 仅保存前 200 条分类及所有拒绝，不能将其误当完整距离扫描。

本次实测抓取变换相对名义附着：平移差 0.268293 mm、旋转差 0.000098830 rad、对应箱角最大差 0.301136 mm。之后的最大相对保持误差为 **0.016825 mm / 0.000042296 rad**；两者含义分开，未将抓取偏差叫作保持漂移。关节位置均在官方范围内。

真实驱动力矩输出仍缺失：`drive_effort_output_qualified=false`，旧聚合字段 `qualification_failures` 中保留 `joint_efforts_within_limit`，其详细状态为 **NOT_EVALUATED：joint effort ratios are unavailable**。没有将其宣称为力矩通过，也没有将它当成此次规划或 POC 流程的阻塞。

实际创建并记录了两个精确映射：`/Validation/Scene/conveyor_transverse/TopCollision → /Validation/Scene/conveyor_transverse` 与对应的 `conveyor_longitudinal` 映射。根 actor / 顶面 actor 的双向许可、错误阶段及侧框/未知物体拒绝由定向测试覆盖。本次在高于顶面的位置理想接管，未声称本循环实测了所有接收顶面接触返回形式。

结束后以 **同一新世界**的 `initialized_actual_state.json → actual_remaining_state.json` 调用真实 `apply_actual_motion_state` 链路，再重复提交最终状态：两次均通过且幂等。原四箱实际接收来源保留，c04 只进入理想接收/已处理/已送出集合，未重新进入待抓候选；剩余 35 个箱体保留。没有跨世界累计成功数。

## 证据位置与结论范围

精简证据见 [evidence/m710_poc_step31_release_reserves/README.md](evidence/m710_poc_step31_release_reserves/README.md)，约 95 KB 的 JSON、日志摘要及离线复查脚本。大日志、原始状态、完整包与视频保留在服务器。

视频：`/root/autodl-tmp/m710-poc-step31-v3-20260918/outputs/isaac_step31_once/replay.mp4`。**640×360、5 fps、178.0 s，890/890 帧解码通过**；物理时间 178.025 s、视频物理时间比例 1.0，PhysX 240 Hz。实际执行墙钟为 1840.403 s。视频 SHA-256：`c6c3e4c80a8b017597627ad5bf41afafffcbd5397d43b60fa3314336e683ec0a`。

本轮证实当前原第五箱存档工况可通过新高度/定向储备完成完整 CPU 任务及一次 POC 实跑。旧 4.958109 mm 仍被 5 mm 规则拒绝；成功来自正常生成的新末段，不是分类器放宽。未证明储备覆盖所有未来工况、真实下游接收/送出资格或真实驱动力矩输出。这些边界不回写为旧录像的新版本实跑结论。
