# Tesseract + OMPL 独立后端 v0.1 实验记录

日期：2026-09-22。分支：`feat/v0.5-backend-tesseract-ompl`。

## 结论与证据边界

真实原生后端已接通，两个 FANUC 自由直连段通过项目权威复检；本轮**没有观察到端到端加速**。空载绕障工程段在 100,000 次原生状态检查内没有找到精确解，而 legacy 在 600 次迭代预算内找到并验证了路径。有限预算耗尽不证明路径不存在。

原生高频循环迁移已经实现：OMPL RRTConnect、FK、Jacobian/SVD、FCL 状态碰撞和关节插值边检查均在 C++ 中。Python 仍负责冻结场景导出、文件哈希、IPC、结果转换、既有 IK/工艺流程和最后的权威复检。后者在长路径上仍是显著瓶颈。

单候选完整任务的 12 次自由接近连接也全部耗尽预算，未生成完整轨迹，未启动 Isaac。

本轮支持继续进行针对性修复和剖析，尚不能将该实现列为经过工程验证的 legacy 替代品。没有在线连续卸货、硬实时、吞吐量或整机动力学合格结论。不同任务的三个样本不合并计算 p50/p95。

## 基线、范围与文件

开始时工作目录为空，检查远端后独立克隆指定分支；实际本地/远端基线均为 `4f6e3037aceb47a525711e58360d98c8751fec6b`，没有未提交用户改动，没有 reset。AGENTS.md 仅修正历史提交分支说明。其他实验分支没有修改、合并或推送。

新增薄合同 `planning_contract.py`、可选适配器 `tesseract_ompl_backend.py`、同源场景导出 `tesseract_scene.py`、`native/tesseract_ompl/` 原生 worker、复现入口及环境/构建工具。没有复制业务状态机，也没有引入 ROS、MoveIt、GPU 规划或 Python 原生依赖到核心包。

所有检查和规划运行于已授权 GPU 服务器的独立目录 `/root/autodl-tmp/tesseract-ompl-20260922`。仓库内证据目录为 [validation/evidence/backend_tesseract_ompl_v0_1](validation/evidence/backend_tesseract_ompl_v0_1/)。凭据不纳入代码与证据。

## 实际接线与执行门槛

实际调用链如下：

1. `tools/run_m710id70_layout_single_carton.py` → `run_layout_single_carton_audit`，保留既有候选、IK、行/任务选择与阶段预算。
2. `LayoutTrajectoryConnector.plan` → `_plan_branch_search` → `_approach` / 既有放置连接 → `_connect_pose` → `_transit`。
3. 仅显式选择原生后端时，`_transit` 的 `pregrasp`（空载、接触之前）和 `transit`（已附着、脱离之后）进入薄适配器。默认 `legacy` 仍走原代码。比较工具使用 `LegacyBackend` 包装同一个 `RRTConnectPlanner`。
4. 原生候选回到 `_path_failure`，复用现有逐边细分、官方机器人/工具/附着物碰撞、关节裕量、奇异性和阶段规则。只有权威复核接受才返回 `B_STRICT_LOCAL_CONNECTION` 路径。
5. 完整段仍须经过 `_finalize_task_checked`。它和阶段 VERIFIED 不等同于执行许可。`m710_execution.build_m710_execution_preflight` 检查 motion/实现/场景/模型指纹、完整轨迹与资产门槛，决定 `simulation_execution_ready`。
6. `isaac_bridge` 在导出执行轨迹时调用现有 `time_parameterize_joint_path`，保留事件点、官方速度、载荷阶段缩放、轨迹审计及有限驱动。原生 worker 不做时间参数化。

接触、吸附、支撑解除、箱堆抽离、放置、释放、接收、撤离、垂直驻留继续使用既有实现。`withdrawal` 本轮没有能够准确表达为自由关节空间的阶段合同，因此明确不支持；不能凭阶段名称替换。非空 support/target-contact 许可、ATTACH/RELEASE 等事件和未知约束均拒绝进入原生自由段。

结果分开记录 `candidate_found`、`exact_solution`、`native_validated`、`authority_validated`、`deliverable_stage_path`；公共结果的 `complete_task_executable` 始终为 false。取消、过期场景、预算、非法端点、依赖不可用和模型不一致有独立状态，显式选择原生后端时不静默回退。

## 环境与实际 API

服务器为 Ubuntu 22.04.5 x86_64，Intel Xeon Platinum 8470Q，22 CPU 配额、110 GiB 内存；GPU 为 RTX PRO 6000 Blackwell，驱动 580.119.02。本实验规划不用 GPU。worker 为单实例、单线程搜索，Python 进程设置 `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1`。共享服务器不是独占性能台，未评估调度噪声或尾延迟。

| 部件 | 实际版本/构建 |
|---|---|
| Tesseract | conda `tesseract-robotics 0.35.0 h4510aa5_0` |
| Tesseract Planning | 安装 `0.35.0 h535d943_1`；未链接、未调用其 pipeline |
| OMPL | 1.7.0，conda `py314hc49c504_2`，实际使用 C++ API |
| 碰撞 | Tesseract `FCLDiscreteBVHManager`，FCL 0.7.0 `hf5102b8_9` |
| 编译 | 系统 GCC 11.4，CMake，C++17，Release；隔离前缀 libstdc++ 16.2 |
| Eigen | 5.0.1 |
| Python 权威环境 | Python 3.12.3，原有 cpu-venv；Pinocchio 4.1.0、Coal 3.0.3、NumPy 2.3.2 |
| Python Tesseract 绑定 | 未用于最终实现；独立 JSON-lines worker |

优先检查了官方 nanobind 路线。服务器默认 PyPI 镜像找不到包，切换 PyPI 后开始下载，但本轮未确认绑定能够将项目自定义 Jacobian、距离规则、细分、有限状态预算及取消检查全部留在原生调用链内。因此只维护小型 C++ worker 一条实现路线；这不是声称官方绑定完全不能规划。尝试下载的 wheel 为 nanobind 0.35.0.8 / CPython 3.10；下载被停止，没有据此报安装成功。

API 依据为固定 0.35.0 官方头文件和源码，tag 对应 `b96e876a54a4d6c8bfe80c8695f694f69dfbe6bf`：

- [Tesseract Environment](https://github.com/tesseract-robotics/tesseract/blob/0.35.0/environment/include/tesseract/environment/environment.h)：URDF/SRDF 字符串初始化、`getState`、`getDiscreteContactManager`。
- [DiscreteContactManager](https://github.com/tesseract-robotics/tesseract/blob/0.35.0/collision/core/include/tesseract/collision/discrete_contact_manager.h)：对象覆盖、active objects、transform、margin 和 `contactTest`。
- [CollisionMarginData](https://github.com/tesseract-robotics/tesseract/blob/0.35.0/common/include/tesseract/common/collision_margin_data.h)：单对总间隙。
- [OMPL 1.7.0 RRTConnect](https://github.com/ompl/ompl/blob/1.7.0/src/ompl/geometric/planners/rrt/RRTConnect.h) 与 [MotionValidator](https://github.com/ompl/ompl/blob/1.7.0/src/ompl/base/MotionValidator.h)。

worker 显式注册 `FCLDiscreteBVHManagerFactory` 插件。流程是直接 OMPL，不是通过名字推测的 `plan_freespace`：没有 TaskComposer、TrajOpt、平滑、shortcut 或隐式最优规划器。首次链接碰到系统 libstdc++ 符号不匹配，构建脚本通过隔离 prefix 的链接/RPATH 修正；URDF 增加 Tesseract 0.35 必需的 `make_convex=false` 属性。源码 git clone 曾因网络 EOF 失败，最终依据已安装头文件编译。安装、源码获取和最初测试失败日志保留，未升级既有 Isaac 环境。

## 同一个世界与碰撞语义

模型来自仓库固定官方 FANUC 模型 `FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2` 的既有 URDF/SRDF。每个原始 visual/collision mesh 均解析并哈希验证，不做凸包、胶囊或单盒替换。每次请求检查文件哈希；最终 worker 检查预期 collision object 存在、启用且具有几何，否则失败。

保留 J1…J6 的顺序、J3 独立语义、官方限位、安装矩阵、flange/fanuc_flange/tool0 和现有虚拟 TCP 变换。+X 入厢、+Y 左、+Z 上，米/弧度/秒。没有 clamp、基座移动或目标调整。3 个确定性状态的 10 个关键机器人 link 逐矩阵最大绝对误差 **4.44e-16**，容差 **1e-10**；TCP 也单独验证通过。原始逐 link 数据见 `final-native-tests.json`。

工具来源是现有权威校验器已审计的全部刚性 OBB 与压缩柔性杯 OBB，每个独立 collider 成为固定 link，完整保留覆盖与对象所有权。它是既有 CAD 审计后的保守碰撞表示，不把 OBB 接触说成原 CAD 确定穿透。最终空载工程场景中原生 collision object 数为 255。

所有冻结箱体、侧壁、传送带、地板和底座支撑障碍物保留。权威无限地板额外以覆盖机器人/附着物可达域的 1 km 封闭盒表达；导出检查臂长、安装位置、附着几何边界，超出证明域即拒绝。该转换适用于当前固定布局，不是任意场景的无限平面等价承诺。

附着箱体作为 `backend_tcp` 下的真实碰撞 link；使用既有 `PhysicalContactAttachment.box_at` 得到同一个刚体变换。世界障碍物里的相同目标必须移除一次，发现重复则拒绝。杯与目标的固定压缩关系在导出时用现有 SAT 谓词验证后才生成固定许可。自由段不包含释放，回到世界由原流程负责。

碰撞策略直接导出：官方相邻自碰撞例外、同装配工具对、按物理所有权生成的 J5/J6—工具对、明确命名的柔性杯—非目标堆叠箱对。没有名称子串过滤。测试名为 `J5_link_tool_environment_obstacle` 的环境障碍仍被拒绝。机器人自碰撞间隙为 0；外部对象未膨胀表面总间隙为 POC 的 5 mm；载荷—传送带加既有 3 mm runtime reserve。没有双重膨胀或归零外部间隙。

20 kg 工具、42.5 kg 箱体、官方惯性、驱动和 PhysX 配置没有修改。几何成功不代表动力学合格。

## 搜索、复检与预算

原生起终点检查后，直连与 RRTConnect 扩展使用同一个 `DenseMotion`：关节线性插值，细分数

`max(1, ceil(sum(abs(dq))/0.0003125), 2*ceil(max(abs(dq))/edge_resolution_rad))`。

其中 0.0003125 rad 来自现有最终检查的 4 m 杠杆 / 1.25 mm 位移界。它是密集离散检查，不是任意关节曲线的连续无碰撞数学证明。没有用一次 link swept 查询冒充该证明。自定义状态校验执行官方 joint margin、TCP 几何 Jacobian/SVD 条件数、径向限制和 FCL 查询；没有 Python 采样回调。

seed=71070；单个 RRTConnect，range=0.18 rad；比较用 legacy 核心 planner 的默认 step=0.18 rad、goal_bias=0.12，普通任务入口原有 step=0.20 / goal_bias=0.20 不变（本表不是该普通入口的整任务基准）；每次请求新建 state space、problem、planner、tree，没有路径缓存。几何环境按完整导出 fingerprint 复用；附着物、约束、阶段或障碍变化会重建。绑定场景 revision 在求解与权威复核前后检查，旧结果不交付。取消标记由 Python 每 50 ms 轮询写入，C++ 每次有效性/终止检查读取。已有 authority callable 内部没有新增取消轮询，取消会在它返回后再次确认；因此该适配器没有硬实时取消时延保证。

native 每个自由连接最多 100,000 次状态检查，修复最多 2 次且共享该预算；legacy 比较使用 600 iterations。二者不是等价工作量。外层接线将既有分配的 iteration slice 记为调度预算消耗，明确不是测得的 OMPL iteration。原任务 statistics 中 rrt_iterations_consumed 仍是既有 legacy 计数，不能用其 0 值解释 native 工作量；native iterations 不可获得，应读取 free_motion_records 的状态/边/细分计数。POC 不启用业务墙钟截止时间。可选显式 wall budget 在复核后再次检查；依赖启动的 60 秒握手保护仅保护进程启动。

若权威复核拒绝，保留失败边/状态/对象对/距离等已有诊断和指纹。拒绝状态也被 native 拒绝时，认定可能是细分网格漏检，将网格减半并在剩余预算内新建 planner 重搜；若 native 仍放行该点，标为模型/策略不一致并停止。没有只换种子赌通过，旧树不复用。真实工程两条候选均无权威拒绝；修复/重复路径/场景过期分支由单列的 fake transport 合同测试覆盖，不能当作已观测真实故障的证据。

## 三个冻结工程样例与原始比较

来源为服务器历史已完成候选 `m710-auto-resume-20260920/repo/outputs/plan_002/first_feasible/motion.json`（SHA256 `4514c342791455f18e735d141f3146fb99db9de43a964e514e63b115c841f0e9`）和 `delivery/segments/001_carton_l07_c03/actual_remaining_state.json`（SHA256 `c2478fbd880806d8499cdc26a4f5613548709e7e76aab4af7731c67d430b2b1a`）。仓库冻结副本在 `tests/fixtures/tesseract_ompl/`。其中 historical_segment.json 是从原 motion 提取的 selected segment，文件 SHA256 为 `cf306dd532c1d20a2a5132dabe0af7ae7bf1ba1de2e2b14fc9a1d3bda1ee81dd`；state 为原始文件副本。通过 .gitattributes 保留原始证据字节与原生源文件 LF，避免 Windows/Linux 换行改变指纹。

场景有 37 个实际剩余箱体（原 40 箱，历史 c01/c02/c03 已移出），不是为某个后端删障碍。目标 `carton_l07_c04`。实际场景快照 fingerprint 为 `5f9e06b478ed009dba1255b5f1a9fb1a1b86af8e367732fbc1bd3a23029078e0`。这是历史状态构建的 CPU 场景，未宣称是本轮实时 Isaac 世界。

固定原路径索引：空载直连 78→79；空载绕障 78→208；附着搬运 78→79。两后端同一 q、scene、工具、附着矩阵、seed、约束与权威 checker，无新 IK。空载绕障端点均合法，直线被 `tool_rigid_0 / carton_l05_c03` 阻挡；载荷差异定向见证为原 78→208 线段 u=0.003，空载可行、附着后碰邻箱。

| 工程段 | 后端 | 结果 | 搜索/直连(s) | 权威(s) | 请求总计(s) | 路径点/关节L2长度(rad) |
|---|---|---|---:|---:|---:|---|
| 空载直连 | native | VERIFIED | 直连 0.480，OMPL 0 | 2.193 | 3.078 | 2 / 0.03230 |
| 空载直连 | legacy | VERIFIED | 搜索含状态检查 0.037 | 2.183 | 2.228 | 2 / 0.03230 |
| 空载绕障 | native | BUDGET_EXHAUSTED | 直连 20.613，OMPL 293.030 | 未产生候选 | 313.781 | 不可交付 |
| 空载绕障 | legacy | VERIFIED | 搜索含状态检查 14.628 | 372.144 | 386.772 | 29 / 4.98765 |
| 附着搬运 | native | VERIFIED | 直连 0.495，OMPL 0 | 3.055 | 4.011 | 2 / 0.03230 |
| 附着搬运 | legacy | VERIFIED | 搜索含状态检查 0.048 | 2.920 | 2.968 | 2 / 0.03230 |

| 计数 | 空载直连 | 空载绕障 | 附着搬运 |
|---|---:|---:|---:|
| native 状态检查 | 201 | 100000 | 201 |
| native 边检查 | 1 | 129 | 1 |
| native 细分样本 | 199 | 99893 | 199 |
| legacy 搜索状态检查 | 4 | 1192 | 4 |
| legacy 搜索边检查 | 1 | 287 | 1 |
| legacy 搜索边样本 | 2 | 1190 | 2 |
| legacy 实际 iterations | 0 | 215 | 0 |

legacy 搜索的采样比最终权威细分稀疏；native 把密集规则放进所有树扩展，因此计数与工作量显著不同。native 单次总耗时较短的失败段不能和 legacy 成功交付时延比较为“加速”。没有证据证明更换几何库本身带来收益，也没有做纯 FCL/Coal 微基准。

时间包含关系：首次公共场景/权威初始化另计 0.505 s。三个场景导出分别 0.103/0.113/0.105 s，未包含在上表 `request_total_s` 中；若从场景打包开始，分别给对应两后端加该值。native 第一次 worker 冷启动 0.051 s，已计入请求总计；Environment 初始化 0.211 s（空载首请求）、0（空载同场景复用）、0.290 s（附着变化后重建）。没有原地更新路径，场景更新计入重建时间。将首次公共初始化、导出和原生请求相加，冷态到已验证直连结果约 3.687 s（明确为非重叠字段的派生值）；同空载 Environment 复用后的绕障请求含导出约 313.894 s，以预算耗尽终止。

worker roundtrip 分别 0.810/313.753/0.927 s，**包含**原生初始化、检查、搜索和 IPC；native_total 分别 0.696/313.650/0.791 s，**包含**初始化、端点、直连、OMPL 和转换。不可把这些嵌套时钟再次相加。原始批次未单独测 JSON/结果转换，标记不可获得；没有几何后处理。最终实现增加转换计时，其功能复验见 `final-functional-smoke.json`，不是性能批次追加样本。Python 权威 cache 在每个后端前清空，底层 mesh 几何对象保持加载。

转换计时字段的真实功能复验（独立批次，不参与上表比较）：输入/资产验证 0.050754 s，worker 冷启动 0.040702 s，Environment 0.228878 s，直连 0.497112 s，Python 结果转换 0.000255 s，权威复核 2.568737 s，请求总计 3.508885 s。未执行几何后处理；直接路径没有 OMPL PathGeometric 转换。原始 JSON 序列化与 IPC 的各子项未分别插桩，保留包含它们的 worker_roundtrip_s_inclusive，不填估计的纯 IPC 时间。

原始比较在最终诊断增强前运行；`benchmark-build.sha256` 与 `worker-benchmark-source.cpp` 固定其构建来源。最终改动增加几何覆盖失败检查、真实 FCL 版本/碰撞查询计数、转换计时、近似解候选标记、取消/预算保护和独立 prismatic 单元夹具支持，没有修改 FANUC 路径、seed 或碰撞阈值。原始数据中的 planning 字段代表安装包，不代表调用了 planning pipeline；最终版本明确报告其未链接。最终 worker SHA 见 `environment.json`，真实适配器功能复验同样 VERIFIED；该复验发生在近似解记录字段修正之前，构建身份见 before-approximate-flag-environment.json。修正后的真实原生回归也通过。原始批次的 approximate 解在 ompl_status 中保留，旧 candidate_found 表示未生成可交付候选；最终代码改为记录 approximate 候选存在，但 exact_solution/native_validated 均为 false，路径不交付。

## 定向回归

服务器命令：

```bash
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export UNLOADING_TESSERACT_WORKER=/root/autodl-tmp/tesseract-ompl-20260922/build/unloading_tesseract_ompl
export UNLOADING_NATIVE_TEST_EVIDENCE=/root/autodl-tmp/tesseract-ompl-20260922/final-native-tests.json
$CPU -m pytest -q tests/test_tesseract_ompl_contract.py tests/test_tesseract_ompl_native.py tests/test_layout_trajectory.py tests/test_planner.py tests/test_poc_pair_clearance.py
```

该相关模块批次结果 **55 passed in 9.90s**，无 skipped。最后补齐 legacy 适配器的场景/策略身份校验后，只重跑受影响的合同文件，结果 **14 passed in 0.32s**（含 1 项新增回归）；本轮最终覆盖 56 个不同测试，未追加全量 pytest。最后一次命令为 `$CPU -m pytest -q tests/test_tesseract_ompl_contract.py`，原始日志为 `final-contract-tests.log`。其中 6 项为真实 Tesseract/FCL/OMPL 测试；合同 fake 测试与真实原生测试文件分离。覆盖依赖隔离/无 fallback、FK、越限/NaN/碰撞端点、完整附着物、环境碰撞豁免不扩大、场景缓存失效、unsupported 阶段、预算/取消/过期结果、权威拒绝和修复，以及旧 planner/trajectory/POC 回归。

真实 RRTConnect 绕障单元夹具是独立 XY 双移动关节小场景，**不是 FANUC 工程成功**：精确解，native 27,835 状态、53 边、OMPL 0.0736 s，独立项目 OBB 表面距离复检 9,996 个样本通过（上述计时来自 before-approximate-flag-final-native-tests.json，语义修正后的回归另见 final-native-tests.json）。同一单元测试随后复用 Environment、保持相同起终点/seed，将原生状态预算限制为 10,000：真实 OMPL 返回 approximate，worker 记录 candidate_found=true、exact_solution=false、native_validated=false、空路径和 BUDGET_EXHAUSTED。此前已求出的精确路径没有被缓存返回。这组补核为 `pytest -q tests/test_tesseract_ompl_native.py`，6 passed in 8.67s，未增加不同的工程场景或 seed。它验证真实搜索调用与边检查，不改善工程绕障失败统计。

最初相关回归发现基线 `test_grasp_branch_search_is_lazy_bounded_and_backtracks_to_alternative` 的 scripted fixture 缺少 `position_tolerance_m`。独立原始基线复现相同 KeyError 后，仅补齐测试夹具配置；没有放宽运行时代码。初始失败和基线复现日志保留。一次测试命令误写现有 POC 文件名，零测试运行，已改正并保留日志。

## 完整任务与 Isaac

真实任务调用为 `tools/run_tesseract_ompl_task.py` → 生产代码 `LayoutTrajectoryConnector.plan`，只使用同一冻结世界、目标 `carton_l07_c04` 和一个历史接触候选。普通审计入口的工厂接线已实现；本轮没有再从该普通入口展开另一轮全候选搜索。既有接触候选的 FK、吸盘几何与工艺逻辑仍执行，预接近 IK 仍由原实现产生。

该调用于服务器 2026-09-22 19:31 左右开始，约 21:08 结束，实际耗时 **5844.065 s（97.40 min）**，`success=false`。最终失败为 **pregrasp / BUDGET_EXHAUSTED**；外层只在所提供的一个候选范围内报告 GRASP_CANDIDATES_EXHAUSTED，不能解释为全部抓取候选或任务无解。

原有候选定义为 direct 接近，加 0.0350009965 / 0.10 / 0.140003986 m 三个 adaptive 接近距离；每个最多 3 个自由连接。实际 **12 次** native 请求全部用尽各自 100,000 次状态检查预算，共 **1,200,000 次状态检查、1,866 次边检查**。12 次 RRTConnect 均只产生 approximate solution，全部拒绝；没有精确候选可送入路径权威复核，authority_rejections=0 不代表通过了复核。种子依次为 71081、72090、73099、71181、72190、73199、71281、72290、73299、71381、72390、73399，均来自原有分支/连接 seed 规则，没有额外扫描。

累计非重叠核心项：原生直连检查 241.749 s，OMPL 搜索 5598.368 s；Environment 首次初始化 0.206 s，其余 11 次复用；场景导出 1.302 s；输入/资产验证 0.297 s；既有轨迹 IK 0.560 s。原生总计 5840.494 s，包含其内部检查/搜索；worker roundtrip 5841.769 s 和各请求总计 5842.127 s 又包含前述 native 时间，不得相加。整次任务 5844.065 s 包含外层 IK、场景转换和请求。这里主要耗时在 native 密集边检查下的搜索，不能把任务阻塞归因于接触、抽离或接收阶段。

**没有完整轨迹包，因此执行前检查未运行，Isaac 未启动，没有新视频，也没有物理执行成功。** 20 kg 工具、42.5 kg 箱体与原有物理配置不变。没有追加第二次任务调用、整排/40 箱运行、104/129 人口测试或物理重试。

原始输出：`task-result.json`、`task.log`、`task-native.log`；由原始 JSON 派生的精简统计为 `task-summary.json`，候选上限的只读展开为 `task-candidate-definition.json`。任务使用与最初比较相同的早期 native 构建，其 source/binary SHA 在 `benchmark-build.sha256`；最终诊断增强后的构建通过独立原生回归，未用它重新跑长任务以覆盖原始失败。

## 复现环境与运行命令

以下在服务器仓库根执行。`CPU` 为原有权威 Python，`EXP` 为独立实验目录；不要对 Isaac Python 执行安装。

```bash
EXP=/root/autodl-tmp/tesseract-ompl-20260922
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
# 本轮实际采用 conda 的隔离前缀；完整解析包/构建/下载来源见 environment.json
conda create -y -p "$EXP/native" -c conda-forge -c tesseract-robotics \
  tesseract-robotics=0.35.0 tesseract-robotics-planning=0.35.0 ompl=1.7.0 fcl=0.7.0 nlohmann_json
bash tools/build_tesseract_ompl.sh "$EXP/native" "$EXP/build"
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export UNLOADING_TESSERACT_WORKER="$EXP/build/unloading_tesseract_ompl"
$CPU tools/check_tesseract_ompl_environment.py --prefix "$EXP/native" \
  --worker "$UNLOADING_TESSERACT_WORKER" --output "$EXP/environment.json"
$CPU tools/run_tesseract_ompl_comparison.py \
  --state tests/fixtures/tesseract_ompl/historical_state.json \
  --segment tests/fixtures/tesseract_ompl/historical_segment.json \
  --worker "$UNLOADING_TESSERACT_WORKER" --output "$EXP/comparison-reproduction.json"
$CPU tools/run_tesseract_ompl_task.py \
  --state tests/fixtures/tesseract_ompl/historical_state.json \
  --segment tests/fixtures/tesseract_ompl/historical_segment.json \
  --worker "$UNLOADING_TESSERACT_WORKER" --output "$EXP/task-reproduction.json"
```

普通任务入口显式选择示例（本轮冻结单候选工具直接调用其实际 connector，避免另一次全候选任务扫描）：

```bash
$CPU tools/run_m710id70_layout_single_carton.py \
  --config configs/validation/m710id70_proof_of_concept.yaml \
  --free-motion-backend tesseract_ompl --output "$EXP/motion.json"
```

资产哈希记录包含解析后的绝对路径，所以搬迁 checkout 后应重新导出 scene fingerprint；不要将旧 worker 消息的路径机械替换后复用签名。历史输入文件本身的 SHA 保持可核对。

## 未解决问题与后续边界

优先定位真实 FANUC 密集 native 边检查的成本和有限预算绕障失败，之后再做少量同输入复评；不得先减小安全间隙、删几何或加大边步长。端到端还需解决长路径权威复核、场景序列化/哈希与完整工艺连接成本。本轮没有实现统一批处理 authority、在线 session 或整个 legacy 的重搜机制。

FCL 与 Coal 在碰撞边界的数值一致性目前仅有少量定向见证；不能保证所有空间状态等价。连续安全仍受现有离散细分模型的范围约束。后端选择、二进制版本和文件/策略指纹必须随证据保留。可复用合同与未来公共核心对齐的位置是 `FreeMotionRequest/Result` 和 `connect_free_motion`；本轮没有假定其他分支接口已合入。
