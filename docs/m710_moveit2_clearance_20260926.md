# 原生自由空间净空与能力提前检查 — 2026-09-26

本轮完成原生搜索净空接线、历史失配回归和提前能力检查。三处历史失败状态及对应失败边在新原生规则中均被拒绝，与原权威检查一致；固定起终点两侧均合法。**真实带载段尚未找到路径**：原有三次、每次 12 秒 OMPL 预算耗尽，不能解释为不可达，也没有完整带载轨迹通过的声明。

旋转任务 TCP 的限制现已在固定候选的接触 IK 后、重型接触/前缀检查前识别，实际候选判断约 **0.00388 s**，含构建与冷启动约 **0.936 s**，重型状态/边检查和原生规划调用均为零。该结论描述当前适配器的恒姿态限制，不是 Pilz 本身的能力限制。

本轮不做完整卸货搜索，不执行 Isaac，不改放置姿态或接触策略，不增加 seed、求解时间、CIRC 或连续轨迹混合。

## 基线、环境与身份

- 本地及远端实际起点：`575ad763e04f500f5413e9b103b0835687024403`，开始时工作区干净；没有回滚、合并或切换其他后端分支。
- 分支：`feat/v0.5-backend-moveit2-mtc-pilz`。
- 实验目录：`/root/autodl-tmp/m710-moveit2-clearance-20260926`。
- 复用上一轮隔离 Ubuntu 22.04 / Humble rootfs，工作副本位于 `/work-round2`，构建目录 `/work-round2/build-round2`。旧 `/work`、旧二进制及 20260922 证据未覆盖。
- MoveIt/Pilz 2.5.10、MTC 0.1.3、OMPL 1.7.0。完整 Debian 版本在 `packages.lock`。
- CPU Xeon Platinum 8470Q；GPU RTX PRO 6000 Blackwell，实际驱动 595.71.05。上述规划与检查在 CPU 上运行，未启动 GPU/Isaac 任务。
- 模型、202 个工具碰撞体、全部 40 箱、20 kg 工具、42.5 kg 箱体、安装与 TCP 变换不变。
- 场景：`e2af5af3746cd1ce311f4ad93473469d6b377377f7509def59a6d894971d0302`。
- 完整配置策略：`741b7d3b5243fbd21a4e1ac8d95e690ea6e819bd6aa3a0b54989e357f00bcc1a`。
- 碰撞子策略：`91e8bebb851170e3620b5e2fc1bda49b5a72d1e62c8b3a3cd0dbed3c05d928e5`。
- 本轮模型/工具参数身份：`8de710f3b4618944ea234a5e6fc84ea2b5483719015399bd8119d651c0ffeb35`；此身份包含资产 URI，`/work` 改为 `/work-round2` 会改变参数散列，资产字节另由 `input-manifest.json` 绑定。
- worker SHA256：`8e3adc99c6bb3c3c8236d9ef3395bf6195f89907e7774980cb90e9f8692e9b51`。

`source-manifest.json` 记录实际执行源码；已逐字节核对本轮所有修改的运行代码、测试、fixture 与本地待提交版本一致。`final-build.sha256` 同时绑定 worker、原生测试程序和源码清单。最终测量来自同一构建，命令串行执行，时间窗为 **04:58:57–04:59:58 UTC**。准备期及格式整理前的结果在服务器另存，没有混入最终数字。

## 原生搜索实际执行的规则

新增 `ros2/m710_moveit_backend/src/clearance.h`，复用 MoveIt 的未膨胀 FCL 几何与距离 API。Python 从现有有效策略生成可执行的 `m710_native_free_clearance_v1`：传入阶段、工具所有权、柔性杯 ID、命名垛箱、载荷、输送机、例外、净空及已有接近预留量。worker 将策略和工具身份与初始化绑定，不接受另一套策略或缺失碰撞几何。

| 对象对 | 原生规则 |
| --- | --- |
| 普通机器人自身 | 相交/接触拒绝，额外净空为 0；保留原 SRDF 例外 |
| 机器人—工具 | 5 mm 总表面净空，即使工具在 MoveIt 中属于 self link |
| 工具内部装配、J5/J6 与自有工具 | 保留准确对象对许可 |
| 机器人/工具—环境与非豁免箱体 | 5 mm 总表面净空 |
| 附着载荷—环境、机器人、刚性工具 | 5 mm 总表面净空 |
| transit 载荷—输送机 | 原有 5 mm 净空加 3 mm 接收运行预留量；没有新增阈值 |
| 柔性杯—命名非目标垛箱 | 保留指定柔性杯对的许可，不扩展至刚性工具 |
| 附着载荷—柔性杯 | 保留原窄 touch links；固定附着下杯压缩仍由原权威检查负责 |

没有将机器人自碰撞全部加上 5 mm，也没有给物体两侧各膨胀 5 mm。距离请求的 **query range** 为最大适用阈值再加 1 mm，仅是查询范围：本带载上下文为 9 mm，接受阈值仍分别是 5/8 mm。数值容差仍为 **1e-9 m**。

距离查询使用 `SINGLE` 模式取得范围内各对象对的最小表面距离。成功的空结果只说明没有非豁免对象对进入有限查询范围，不报告“最小距离无限远”。异常、非有限返回、缺失形状、空网格、策略不一致均失败关闭；输入场景仅接受现有 box 对象与审计后的官方网格。

每个候选创建自己的距离上下文：持有该候选的不可变 world/FCL 环境、ACM 副本和策略，回调消费求解器传来的当前 RobotState，包括其附着载荷。不存在跨候选状态有效性缓存，也不捕获固定 `q_start`。world、ACM、附着切换分别有真实回归。

通过 `PlanningScene::setStateFeasibilityPredicate` 接入，Humble OMPL 的 StateValidityChecker 确实调用 `isStateFeasible`，边验证也经过这个状态检查。证据中 `search` 为 MTC/pipeline 求解期间的谓词调用；它也可能包括 pipeline 自身的解验证，不把它冒称为纯 OMPL 扩展节点数。实际无解的三轮 OMPL 均在此阶段产生大量净空拒绝，并保留全模型原相交规则仍允许的状态见证。

原生输出路径在 IPTP 后再次检查，Pilz PTP/LIN 同样适用。采用与 Python `_path_failure` 相同的双倍采样计数，以及 `4 m × 六关节增量和 / 0.0025 m` 的现有保守点运动采样规则；没有粗化原 OMPL 分辨率。该结果是离散边验证，**不是严格连续扫掠证明**。原 Python 完整边检查、最终合同和时间下界导出全部保留。

## 原始失配回归

`tests/fixtures/moveit2/clearance_rejections.json` 从旧 `native-loaded-tight-goal.json` 直接提取 double 精度 q、原边端点、边号、fraction、对象对、阈值和身份，没有从毫米汇总重建。旧文件与完整原路径仍保留。

原始 raw 请求未留存，因此附着矩阵按原 fixture 抓取 q、原 FK 和同一 `RigidAttachment.capture` 重建；明确记录了这一来源，不声称有不存在的原始矩阵日志。当前完整请求（含 world、附着矩阵、TCP、可执行策略）保存在 `differential-final.json`。三处对象对均为目标 `carton_l07_c02` 与邻箱 `carton_l07_c01`。

| 原记录边号 | 权威失败点距离 m | 新原生同点距离 m | 原生减权威 m |
| --- | ---: | ---: | ---: |
| 31 | 0.004984457298504093 | 0.004984457298504187 | 9.37e-17 |
| 14 | 0.004994185483438035 | 0.004994185483437884 | -1.51e-16 |
| 23 | 0.004998934563793401 | 0.004998934563793597 | 1.96e-16 |

三点都满足旧相交规则、违反 5 mm 净空。新原生与原权威均拒绝这三点及三条对应边。边内首次拒绝位置一致到浮点取样表示精度，例如 `0.38666666666666666` 与 `0.3866666666666667`。表中是**失败点距离，不是整条路径的最小净空**。

固定 q_start/q_goal 均原样通过两侧检查。另将原始失败 q 作为端点，真实 worker 返回 `INVALID_START_CLEARANCE` / `INVALID_GOAL_CLEARANCE`，pipeline 调用数为 0；Python 入口对应返回 `INVALID_GOAL_AUTHORITY`，没有转交 RRT 或修改端点。

原生人工几何测试覆盖低于/高于阈值、阈值两侧纳米邻域、合法端点间的非法边、普通 self 与 robot/tool 的不同规则、精确杯对许可、附着载荷和独立 diff。它们仅是实现验证，不计入真实场景成功率。

## 真实调用、耗时与结果

23 项相关 Python 测试通过，耗时 **0.29 s**；包含原有续搜、取消/过期结果处理、轨迹出口时间下界及新增能力检查。真实 FCL 人工测试和历史差异检查均通过，后者端到端 **16.789 s**。

小段与带载段分别启动一个常驻进程；小段进程按 PTP→LIN 顺序复用模型。带载进程的顺序为 PTP→OMPL→OMPL→OMPL。resident RNG seed 为 71070；带载业务 seed 仍为 71072；三次 OMPL、每次 12 s，range=.2、longest_valid_segment_fraction=.001 均未增加。有限墙钟求解不承诺逐位可复现。

| 请求 | 原生求解 s | 原生输出边检查 s | Python 完整边检查 s | 段端到端 s | 结果 |
| --- | ---: | ---: | ---: | ---: | --- |
| 小范围 PTP | 0.086174 | 0.464357 | 0.411587 | 1.071866 | 两侧 PASS |
| 既有恒姿态 LIN | 0.054653 | 0.406336 | 0.400397 | 0.924629 | 两侧 PASS |
| 真实带载段 | PTP 0.857947；OMPL 12.009690 / 12.012249 / 12.016777 | NOT_RUN，无返回候选 | NOT_RUN，无返回候选 | 37.159244 | SEARCH_EXHAUSTED |

小段 PTP/LIN 原生输出分别检查 47/42 个状态。Python 端点检查分别 0.021610/0.000145 s；带载端点检查 0.028350 s，仅执行一次，重复出现的字段是同一共享计时，不能相加。带载完整程序含准备/冷启动 **38.156103 s**。差异检查进程的模型/插件冷启动为 0.127688 s，含进程/IPC 0.279944 s。

| 带载调用 | 求解期间净空状态查询 | 全部拒绝 | 其中正距离净空不足 | 距离检查累计 s（嵌套） |
| --- | ---: | ---: | ---: | ---: |
| PTP/pipeline | 75 | 42 | 6 | 0.781935 |
| OMPL 1 | 1136 | 690 | 64 | 11.562220 |
| OMPL 2 | 1192 | 826 | 90 | 11.626493 |
| OMPL 3 | 1102 | 682 | 55 | 11.588744 |

每次 OMPL 都记录了 `legacy_intersection_valid=true` 的正距离拒绝见证。例如三次首次见证的间隙分别为 0.004757396377684421、0.00013672968121439255、0.0018162584908237938 m。这些不是 mock，也不是仅输出后的检查。`queries` 计状态调用，`distance_api_calls` 计 self/world 距离 API 次数，二者不混用。

距离检查计时包含在原生求解或输出检查里；IPC 包含 worker 工作；这些嵌套值不能相加成总时间。用于见证的旧相交诊断调用也单独计数。新增原生完整输出检查可能长于原先 IPC 等待预算，因此另设可配置 **600 s IPC 看门狗**；它不增加 OMPL 的 12 s 搜索时间，也不是新的业务总超时。超时仍销毁该进程并拒绝迟到结果。

本次没有得到新的“原生通过、权威拒绝”路径。若以后出现，会返回 `NATIVE_AUTHORITY_MISMATCH` 并保存具体路径、对象对、状态/边及剩余次数，先定位差异；正常原生拒绝和搜索失败仍按既有预算续搜。当前剩余失败是**该预算内搜索没有找到**，不能由少量样例断言所有几何/策略已完全等价。

## 能力预检

纯 `linear_capability` 只比较起终姿态与当前适配器能力，记录任务 TCP 变换、姿态、阶段、位置及调用数，不做 IK、碰撞或可达性判断。其约束仍是当前适配器只支持恒姿态，不论 TCP 偏移是否为零；没有删除姿态检查或偷换 flange 直线。

固定历史候选先用已有 `solve_local` 确定实际接触解，再用同一几何切点、接收意图和释放高度检查后缀。仅当所有既有释放高度都需要不支持的姿态变化时，才提前拒绝该候选。信息不足则保留正常流程；另在实际后缀入口、其重型前缀复检前再次检查。普通核心没有该 hook，仍沿原有流程运行。

最终 `--suite capability` 使用真实 fixture 和普通候选调用链，并安装“一旦调用重型状态或路径检查即报错”的守卫。结果：候选判断 0.003878 s；含全部准备 0.936183 s；状态验证 0、边调用 0、原生规划调用 0。没有再次启动半小时的完整任务验证，也没有把未支持写成不可达。

## 仍依赖权威检查及具体阻塞

- 原生本轮覆盖当前 POC 自由段；历史/接触/支撑释放的有状态语义、杯压缩、工艺预留条件、完整事件合同、动力学与物理执行限制仍由既有核心负责。未覆盖的策略明确拒绝，不自动套用 5 mm。
- 三处历史状态/边失配已消除；不推断任意场景均等价。
- 真实带载段无返回解，尚未实现“该段原生通过→权威完整边通过”。查询成本占用了大部分原生求解时间。最小后续工作是分析 FCL 对偶查询与 broad phase 成本、保持相同判定的前提下减少重复几何工作，而不是扩大 seed 数量或放宽阈值。
- 普通自碰撞的 0 额外净空、许可分离及当前载荷变换已验证；连续扫掠没有严格证明。
- 通用旋转任务 TCP LIN 仍未实现。完整 MTC 卸货、Isaac 和物理接收：**NOT_RUN（本轮明确范围之外）**。

不使用上一轮并发完整任务的时间计算提速，也不报告 P95、吞吐率或速度倍数。

## 复现与证据

证据目录：`docs/validation/evidence/m710_moveit2_20260926/`。主要文件：

- `differential-final.json`：原始失败点/边、同一完整请求、两侧距离、端点和上下文切换。
- `synthetic-final.json`：原生 FCL 人工几何结果。
- `loaded-final.json`：带载真实调用、拒绝计数、见证、预算与失败。
- `small-final.json`：PTP/LIN 原生输出与权威 PASS。
- `capability-final.json`：早期退出位置、三种释放高度姿态与零重型调用。
- `python-final.log`、`build-final.log`、`final-validation.log`：最终检查与串行时间窗。
- `worker-final-window.log`：最终时间窗内原始 worker 日志；`worker-final.log` 还保留了格式整理前的准备期调用，不能重复累计。
- `source-manifest.json`、`input-manifest.json`、`final-build.sha256`、`packages.lock`：源码、输入、二进制与依赖身份。
- `final-validation.sh`、`worker-launcher.sh`：服务器实际命令，未包含 SSH 凭据。

兼容 Humble 安装上的复现命令：

```bash
bash ros2/m710_moveit_backend/build.sh
export M710_MOVEIT_COMMAND="$PWD/ros2/m710_moveit_backend/worker.sh"
export M710_MOVEIT_SEED=71070 M710_MOVEIT_STAGE_SECONDS=12
export M710_MOVEIT_REQUEST_TIMEOUT=600
export PYTHONPATH=src
./build/moveit/m710_clearance_test
python tools/check_m710_moveit_clearance.py --output outputs/differential.json
python tools/run_m710_moveit_integration.py --suite capability --output outputs/capability.json
python tools/run_m710_moveit_integration.py --case empty_short --case linear_fixed_orientation --output outputs/small.json
python tools/run_m710_moveit_integration.py --case loaded_fixed_transit --output outputs/loaded.json
python -m pytest tests/test_moveit2_backend.py tests/test_layout_trajectory.py \
  tests/test_isaac_bridge.py::test_native_duration_floor_reaches_existing_replay_exporter -q --tb=short
```

服务器用旧 CPU venv 的 Python、`M710_MOVEIT_ASSET_ROOT=/work-round2` 和本轮 `worker.sh`；隔离安装方式沿用上一轮报告。常规入口继续支持 `--backend moveit2`，ROS 仍为可选依赖。

核查的官方调用链：[Humble OMPL StateValidityChecker](https://github.com/moveit/moveit2/blob/humble/moveit_planners/ompl/ompl_interface/src/detail/state_validity_checker.cpp)、[MoveIt FCL 距离回调](https://github.com/moveit/moveit2/blob/humble/moveit_core/collision_detection_fcl/src/collision_common.cpp)、[FCL CollisionEnv](https://github.com/moveit/moveit2/blob/humble/moveit_core/collision_detection_fcl/src/collision_env_fcl.cpp)。实际编译使用服务器锁定的 Humble 头文件和共享库。
