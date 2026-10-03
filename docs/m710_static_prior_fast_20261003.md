# 静态先验辅助顶排五箱 planning-only 实测（2026-10-03 至 10-04）

GPU 服务器上唯一一次新的正式批次完成 **5/5**。**T_plan_5 = 29.687083974 秒，未达到 ≤1 秒目标；T_online_5 = 30.040590232 秒。** 共享 1 秒快阶段内完成 0/5，随后在同一批次、同一总预算内补全。c03 为 9.234296285 秒，c04 为 12.036047772 秒；整批 **OMPL 调用 0 次**，自由转移由当前短连接或静态门户路线完成。

所有结果均为 **PLANNING_ONLY_NOT_EXECUTABLE**，`qualification_status = NOT_EVALUATED`；跳过项保持 `SKIPPED_PLANNING_ONLY`。这是先验辅助候选生成性能，不是从零规划、执行资格或实际卸货节拍。

证据入口：[汇总 JSON](experiments/m710_static_prior_fast_20261003/summary.json)、[逐箱 CSV](experiments/m710_static_prior_fast_20261003/five-box-summary.csv)、[同机对照](experiments/m710_static_prior_fast_20261003/comparison-summary.json)、[完整轨迹](experiments/m710_static_prior_fast_20261003/formal/trajectories.json.gz)。

## 正式五箱结果

完整 40 箱初始场景，官方 FANUC M-710iD/70、完整工具、20 kg 工具和 42.5 kg 箱体配置。实际顺序 c02 → c01 → c03 → c00 → c04，均选择 front 抓取。第一箱使用配置初始 q；后续沿用上一箱实际撤离末态，不回 home，不读取旧路径或局部诊断的成功末态。

| 箱体 | 完整 | 规划秒 | 使用的空载/持物通道 | IK / PTP / LIN / OMPL | 先验命中 / 直连 / 局部修复 / 慢 OMPL | 失败尝试 |
|---|---|---:|---|---|---|---|
| c02 | 是 | 1.817321455 | 两段当前直接连接 | 3 / 0 / 4 / 0 | 0 / 2 / 0 / 0 | 无运动请求失败 |
| c01 | 是 | 2.277386935 | 空载门户、持物直连 | 3 / 0 / 4 / 0 | 1 / 1 / 0 / 0 | 无运动请求失败 |
| c03 | 是 | 9.234296285 | 空载门户、持物门户图 | 3 / 0 / 4 / 0 | 2 / 0 / 0 / 0 | 无运动请求失败 |
| c00 | 是 | 4.322031528 | 空载门户、持物门户图 | 7 / 0 / 4 / 0 | 2 / 0 / 0 / 0 | 部分 IK seed 未成功 |
| c04 | 是 | 12.036047772 | 空载门户、持物门户图 | 5 / 1 / 4 / 0 | 2 / 0 / 0 / 0 | 1 次查图超时、1 次 PTP 失败 |
| 合计 | 5/5 | **29.687083974** |  | **21 / 1 / 20 / 0** | **7 / 3 / 0 / 0** | 全部计入墙钟 |

7 次命中共复用 13 个门户/图节点和 6 条存储图边；仅复用一个门户也如实计为节点复用，不伪称使用了存储边。3 次纯起终点直连单列为 `NATIVE_DIRECT_CONNECTION`。5 次 place 生成无运动语义事件，真实原生求解次数为 0。20 次成功运动来自 Pilz LIN；唯一 PTP 调用失败。普通入口的求解来源要求不变。

| 时间口径 | 秒 |
|---|---:|
| 五箱纯规划，包含排序、所有失败、IK、查图、连接、IPC、拼接 | 29.687083974 |
| 当前批次场景更新 | 0.201989586 |
| 批次记账、组装和逐箱进度输出 | 0.000931889 |
| 结果文件写出 | 0.150584783 |
| T_online_5（以上之和） | **30.040590232** |
| 初始化（包含静态模型、worker、先验加载） | 0.941833938 |
| 其中先验文件加载及原生注册 | 0.044792442 |
| 全新 Python 进程启动至退出，含导入和关闭 | 31.902409082 |

最后的计时记账文件及最终 stdout 不计入自身写出时间，见 `write_timing.json`。初始化中的加载分项不能再次加到初始化总数上。

共享快阶段预算为 1 秒，实际落在纯规划区间的快阶段时间为 0.968567375 秒，慢阶段规划为 28.718516599 秒；不是每箱重新获得 1 秒。正式 FAST 请求未出现返回越过快截止时间的记录。慢阶段仍使用先验，不等于调用 OMPL。阶段/单箱/整批交付/IPC 上限为 **90 / 300 / 900 / 100 秒**，单次先验查图上限 **6 秒**，始终取共享剩余预算与自身上限的较小值。全部结果返回后才宣布完成，没有后台补算。

## 改动及作用

保留显式 `cold_from_scratch`，新增仅供 planning-only 的 `static_prior_fast`。MoveIt 场景、MTC Task/前驱、Pilz 工艺直线和 OMPL 后备均保留；未增加 GPU 规划后端。

- **释放事件化**：当前真实末态满足原有 MoveIt 目标约束（位置/姿态 1e-6）、目标/附着/Task/前驱一致时，生成 `PLACE_TARGET_REACHED`，points 为空、terminal_q 保留实际值。未到位仍求短 LIN。旧 `native_zero_motion` 单点兼容路径保留，没有重新修复或改写历史 place 根因。
- **静态门户图**：空载和名义持物各 160 节点，离线静态完整几何检查；查询当前实际起终点、剩余箱堆和实际附着，复用选中的构型与图边。没有输入旧成功 q 或五箱完整答案。
- **受控出口**：先完成原 extraction，到已脱垛的实际末态，再优先追加一段沿原 outward 方向、保持 TCP 姿态的 LIN，朝配置推导的 front 通道前进。新增段是独立 transit / FREE_LOADED_TRANSFER，恢复正常 payload/stack 规则，不携带 extraction 的 initial_proximity 或支撑豁免。总前移受原 0.80 m 上限约束；当前约只能到 x=-0.7675 m，不能宣称已到 x=-0.9 m 门户平面。最终关节兼容性由随后真实查图确认。失败时复用原 extraction 路径和 receipt，后续从对应实际末态继续。
- **有界查询与批量 IK**：每次先验查询最多 6 秒、128 条边检查；超时、未连通和检查数上限有不同状态。IK seeds 每批最多 4 个，原生上限 8，保持既有尝试顺序和首个成功语义。查询和候选边检查留在常驻 worker 内。成功来源、无运动事件、纯直连与先验复用分开计数，不伪造 native_solver_calls。
- **计时修正**：connection_elapsed_s 是连接累计墙钟，request_round_trip_s 是当前请求独占往返，native_solver_s 是实际内核分项；T_online 使用整个 batch_wall 加交付写出，分项不重复相加。

当前仍按阶段请求，**尚未实现一箱一次或整批一次复合请求、入口/出口 IK 联合选择、局部路径修复或静态几何包络复用**。出口策略是一个有界启发，不是完整关节构型族联合优化。

主要代码：[worker.cpp](../ros2/m710_moveit_backend/src/worker.cpp)、[static_prior.h](../ros2/m710_moveit_backend/src/static_prior.h)、[native_semantic_event.h](../ros2/m710_moveit_backend/src/native_semantic_event.h)、[moveit2_backend.py](../src/unloading_sim/moveit2_backend.py)、[layout_trajectory.py](../src/unloading_sim/layout_trajectory.py)、[static_prior.py](../src/unloading_sim/static_prior.py)、[planning_only.py](../src/unloading_sim/planning_only.py)。入口：[建库](../tools/build_m710_static_prior.py)、[固定连接对照](../tools/benchmark_m710_prior_connection.py)、[连续批次](../tools/plan_m710_top_row_only.py)。

## 同服务器局部对照

A 固定自由连接使用相同起终 q、实际附着/场景、种子、60 秒请求预算和 90 秒 IPC。每侧为新 worker / 新诊断 Task，输入几何哈希成对一致；不读取历史路径作为答案。B 固定相同卸货任务（c03 当前 38 箱、c04 当前 36 箱），允许选择不同合法中间分支。两类比较不混用。

| A：固定连接 | 从零秒 | v3 先验秒 | 结果 |
|---|---:|---:|---|
| c03 空载接近 | 9.671366 | 4.929139 | 成功，约 1.96 倍 |
| c03 持物搬运 | 21.587375 | 3.221221 | 成功，约 6.70 倍 |
| c04 持物搬运 | 16.516251 | 0.731753 | 先验未连通，**不算成功加速比** |
| 未建库 c03，起始 J1 +0.005 rad | 30.744060 | 3.399702 | 成功，两侧输入一致 |
| 未建库 c04，起始 J1 +0.005 rad | 17.243762 | 0.989384 | 先验未连通，保留失败 |

c03 固定持物连接复用了 front-clearance 和 transition-top 两个门户、一条图边以及两条当前连接。c04 固定请求的 29 次边检查全部拒绝（28 次起点连接、1 次直连），少量见证涉及箱体与下层箱间隙、车顶接触、刚性工具与侧墙间隙。不能统称“碰撞”，更不能推断放置永久不可达。相同请求从零 OMPL 成功说明这里主要是图及入口覆盖不足。

| B：固定卸货任务 | 原基线秒 | 冻结版可比秒 | 冻结版全计规划秒 | 原→新 IK / PTP / LIN / OMPL | 可比改善 |
|---|---:|---:|---:|---|---:|
| c03 | 44.843897 | 9.283187 | 9.330556 | 37/20/9/2 → 3/0/4/0 | 79.30%，4.83 倍 |
| c04 | 31.077815 | 16.229850 | 16.269672 | 39/20/6/2 → 8/2/5/0 | 47.78%，1.91 倍 |

原基线将目标排序计入场景更新，新版将其计入纯规划。B 可比列仅为对齐旧口径减去新版排序（c03 0.047369 秒、c04 0.039822 秒）；全计列与正式五箱始终保留排序。B 预算固定 90/300/300/100 秒、seed 71070；开发快阶段预算为 300 秒，不作为 1 秒正式成绩。

保留所有研发结果：未加出口及查询上限的 v3，B c03 全计为 9.229671 秒，但 c04 **退化至 78.661968 秒**。c04 前两个接近分支查图分别约 24.486、26.111 秒，最终持物 OMPL 又耗 22.455 秒。冻结版将无效查图限制到各 6 秒，并通过正常权限出口 LIN 使持物先验查询在约 1.044 秒成功，OMPL 降为 0。一次开发启动因隔离环境没有 git 命令而在规划前退出；已改为由操作端传入核实的提交身份，失败启动日志保留，未安装 git、未形成额外正式批次。

## 建库、绑定与剩余瓶颈

v3 通用建库 **251.121629896 秒**，初始化另计 0.793277601 秒，数据库 **238781 字节**。离线 30 个布局推导 TCP 姿态覆盖箱前、转向和接收区，每姿态最多 3 个 IK 分支；510 次原生 IK、90 批 IPC，得到 89 个分支。每个负载类别最终接受 73 个门户（25/30 姿态），其余 87 节点由有界 Halton 关节采样补充。空载图 545 边、持物图 386 边，最多 1600 次采样、10 个近邻，seed 71071。持物图仍有 18 个连通分量，最大 101 节点；不代表全空间覆盖。

名义持物使用配置的完整 0.6×0.4×0.3 m 箱体和前面中央物理接触变换。绑定机器人/关节限制/安装、完整工具/TCP、静态布局、碰撞和接收策略、空载/持物类别、尺寸与附着。实际附着检索范围为平移 ≤0.025 m、旋转 ≤0.05 rad；这只是候选检索范围，当前实际几何仍重新检查，未放宽目标或碰撞阈值。

在线每条选中图边和新连接使用现有完整 Clearance + ProcessPolicy，以 L1 关节插补步长 min(edge_resolution, 0.01 rad) 检查当前场景，随后原生 IPTP 正常时间参数化。该策略是离散规划检查，不是连续扫掠证明；没有只检查端点或 TCP，也没有把静态安全结论无条件转用于新附着物。

研发建库成本全部保留：v1 104.722443 秒后遇到 Python tuple / JSON list 表示误拒绝；2 节点 probe 0.248093 秒复现（原生实际成功 0.220486492 秒）；修复采用 canonical JSON 严格比较。v2 纯 Halton 图建库 207.203463 秒，固定 c03 请求仍未连通；随后才增加布局门户生成 v3。四次生成累计约 **563.296 秒**，各自初始化另列，不能仅展示成功建库成本。

正式批次查图内核累计 **21.852096 秒**，占 T_plan 约 73.6%；141 次边检查、37019 次状态检查、88 次边拒绝。运动请求往返累计 25.435970 秒，IK 请求往返 0.414116 秒，均是总墙钟的内含分项。独立 c03 固定连接中，仅当前几何检查就约 2.810 秒/3.221 秒。正式 c04 的一个不适合接近分支仍消耗 6 秒查图预算。因此 ≤1 秒尚有明确差距：几何检查、分支匹配及阶段组织仍占主导，单靠删去 OMPL 不足。此次未继续添加 GPU 后端或扩大参数矩阵，也未重复正式五箱挑最好成绩。

## 环境、身份、复现及验证

复用同一 Ubuntu 22.04.5 隔离环境：Python 3.10.12、ROS Humble、MoveIt/Pilz 2.5.10、MTC 0.1.3、OMPL 1.7.0、GCC 11.4、CMake 3.22。Release（-O3 -DNDEBUG），构建 -j2；固定 taskset 0-21、OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1。容器 22 CPU / 110 GiB，主机 Xeon Platinum 8470Q。正式进程 CPU user 31.084 秒、system 0.876 秒，最大子进程 RSS 1878600 KiB（约 1.79 GiB）；采样 Python 2 线程、worker 11 线程，包含 ROS 后台线程，不等于 11 路并行规划。

GPU 为 RTX PRO 6000 Blackwell Server Edition、驱动 595.71.05、97887 MiB。本实现实际使用 CPU MoveIt/Pilz/OMPL/FCL。设备级显示的 100% 利用率、0 MiB 显存并不证明本规划器使用 GPU。没有终止其他项目进程、安装系统依赖、升级环境或启动 Isaac。正式开始 loadavg 约 1.01/2.85/4.19，资源采样随证据交付。

初次及发布前远端 HEAD 核对为 f7846bee960d290de695312b0e2ded0ef4975034；基线源码 tree 为 6c8f059f49baa4483aed22193752ac2f83ca045e。新源码唯一编辑位置为服务器 /work-static-prior-20261003，冻结提交 **00352b6ff2ab3114b5248b266a0f3e955dd30f08**，其后仅追加报告/证据。基线 worker SHA-256 为 5126aa3275288827554ab8e14db98d1e3eca8b5191645cb3430892a35e049d09；冻结 worker 为 **23432aac551a63dbf366a02eb8ecc9c6aee1c7566e468893bcf52a19d76605f6**；v3 数据库为 **df5b0c02c0566542b98607162d2c3bd61871fb0792950200f5ccb61b54754311**。84 个模型/工具/配置文件逐一同内容；模型摘要差异仅源于 URDF 网格绝对路径，使用相同路径前缀后摘要相同。见 formal-freeze.json、geometry-identity.json、包版本和构建身份记录。

构建与入口在服务器执行，Windows 只控制、收取和发布受控快照。实际命令见 [commands](experiments/m710_static_prior_fast_20261003/commands/) 及各 build/timing JSON；形式如下（已有隔离环境内）：

```sh
source /opt/ros/humble/setup.bash
cd /work-static-prior-20261003
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1

# 通用建库，未输入正式任务端点
/opt/planning-opt-20261003-venv/bin/python tools/build_m710_static_prior.py \
  --layout-portals --output /tmp/static-prior-20261003/static-prior-v3.json \
  --worker-command /tmp/static-prior-20261003/worker-current.sh \
  --worker-binary /tmp/static-prior-20261003/build/m710_moveit_worker \
  --source-commit 30c6f3bf058563fcd2ffe23c2523ef71db265809

# 唯一正式批次；外层 taskset -c 0-21，完整参数保存在脚本中
/bin/bash /tmp/static-prior-20261003/run-formal.sh
```

新的输出目录拒绝覆盖已有实验。换工作目录或模型身份后应重新构建匹配数据库，不能编辑绑定摘要绕过检查。

最终 121 项 focused Python 测试通过；原生 semantic-event、static-prior、clearance、zero-motion 与 process-geometry（15 用例）、process-policy（11 用例）测试通过。涵盖目标已到/未到、模型/TCP/尺寸/附着及空载/持物不匹配、新障碍阻断路径中段、连杆/负载几何、超时/取消、实际 parent/末态、释放后箱体及冷启动/执行隔离。合成协议/几何测试仅为代码回归，不作为实测性能或执行资格。正式连续性与来源共 47 项轻量核对未发现不一致，另存 verification.json；未重新运行被隔离的重型路径验收。撤离时目标存在由当前阶段上下文、原生日志及冻结代码联合支持，未归档该时刻独立完整场景快照；此证据边界已明示。

## 范围限制及参考

保持完整几何、邻箱、阶段身份、名义吸附/释放及理想送出；已释放箱体参与撤离后才移除。没有传感器实测、真空建立/抓持力确认、物理碰撞力、落带稳定性或动力学/力矩/jerk 资格证据。未执行真机、Isaac 或导出可执行包；小样本不能宣称 p99 或硬实时。

借鉴 [OMPL Lightning 检索修复源码](https://raw.githubusercontent.com/ompl/ompl/1.7.0/src/ompl/geometric/planners/experience/src/LightningRetrieveRepair.cpp)、[Thunder 源码](https://raw.githubusercontent.com/ompl/ompl/1.7.0/src/ompl/geometric/planners/experience/src/ThunderRetrieveRepair.cpp)及[稀疏经验路线论文](https://arxiv.org/abs/1410.1950)中的检索、真实起终点连接与当前场景适配思路；未集成 Thunder/Lightning 或宣称其理论保证。已阅读 [CSDecomp](https://github.com/wernerpe/csdecomp)及[论文](https://arxiv.org/abs/2504.10783)、[VAMP 论文](https://arxiv.org/abs/2309.14545)、[cuRoboV2 论文](https://arxiv.org/abs/2603.05493)；本轮未增加这些依赖，其发表时间指标不作为当前五箱的性能保证。

原 f7846be 及更早实验记录保持原样。代码和证据以同一集成分支的快进提交发布；原服务器代码历史通过增量 Git bundle 保留，最终发布 tree 与服务器交付 tree 核对一致。
