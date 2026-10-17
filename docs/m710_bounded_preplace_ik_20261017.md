# 固定预放置目标的有界原生 IK（2026-10-17）

本轮两个全新案例都在 **FRONT / x-，一次选面**完成 pregrasp、contact、extraction、transit，到达仍附着的 preplace。c04 正面 IK 已恢复，实际进入并完成一次持箱 RRTConnect；没有重试案例、换 seed、移动目标或扩大预算。

结果是 **PLANNING_ONLY_NOT_EXECUTABLE**；qualification_status=NOT_EVALUATED，continuous_swept_proof=false，physical_execution=NOT_RUN。不是放置、释放、真实吸附或卸箱完成。

## 接口原因与最小改动

实际环境是 MoveIt/KDL 插件 **2.5.10**、Orocos KDL **1.5.1**。旧接口 getPositionIK 内部以 timeout=0 调用 searchPositionIK，只做初始数值尝试；失败可能给出 TIMED_OUT（−6）。旧 ik_timeout_s=0.25 没有成为该 API 的搜索期限，不能解释为用满 0.25 秒、RRT 失败、负载碰撞或目标不可达。[官方 2.5.10 源码](https://github.com/moveit/moveit2/blob/2.5.10/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp)

新增显式 bounded_preplace 模式，只由 through-preplace / planning-only / cold_from_scratch / transit / FREE_LOADED_TRANSFER / 正确附着上下文启用。启动时绑定能力，worker 检查当前 Task 的实际 extraction 父阶段、末 q、剩余世界、目标、附着变换及 mask。普通入口仍使用 getPositionIK，contact/extraction 内部运动学不变。

新模式以本次 extraction 实际末 q 为首次初值，**一次 searchPositionIK API 调用**，配置 0.10 s，取任务/阶段剩余预算较小值，并在原生调用前再次扣除请求内已用时间。Python 在构造前、派发前、返回后及 FK 后检查取消/截止；原生在求解前后检查阶段期限，外层 IPC watchdog 保留。KDL 在局部求解之间检查超时，不能声称严格 100 ms 上界。保留完整六维目标、原 FK 阈值、有限数/维度/关节限位；不启用近似解、position-only 或碰撞绕过。

返回只含一个严格运动学解；碰撞不在这次 IK 内检查，后续原生 plan 仍检查目标及完整持箱路径。每面最多一次 transit RRTConnect，起点/attachment 不随数值初值改变；support_names=[]、initial_proximity=null。

## 目标核对与固定输入接口对照

[归档目标差异](experiments/m710_bounded_preplace_ik_20261017/target-comparison.json)：c02/c04 模型、工具、基座、关节范围、物理接触/TCP 约定一致，仅剩余世界指纹不同。两例预放置箱体平移差 4.88e-8 m，物理接触和虚拟 TCP 平移差均为 1.20e-7 m；捕获 attachment 矩阵最大逐项差 2.28e-7。目标并非逐值相同，差异来自保留的实际接触捕获。抽离末初值六关节差约为 [1.8113, 0.1197, 0.0469, −3.5245, 0.1414, 3.7577] rad。没有发现需要用搜索掩盖的变换或身份错误。

正式运行前，使用同份源码的非安装 IK 专用程序，各模式每例只调用一次。它只允许 init/FK/IK，不能规划运动；固定输入没有现场 Task，明确不冒充实时父链。读取归档实际输入 q/目标/attachment，未使用成功 IK q 作种子。诊断输出不注入正式案例。

| 固定输入 | API | 结果 / 原码 | API 实耗(s) |
|---|---|---|---:|
| c02 | getPositionIK | SUCCESS / 1 | 0.000354 |
| c02 | searchPositionIK，0.10 s | SUCCESS / 1 | 0.000324 |
| c04 | getPositionIK | NATIVE_IK_FAILED / −6 | 0.000567 |
| c04 | searchPositionIK，0.10 s | SUCCESS / 1 | 0.000931 |

每行都是一次上层请求、一次 API 调用。bounded 模式的内部初始化/重启次数为 **null / NOT_OBSERVABLE**，不是 0 或 1；native_ik_calls 保持“显式 op=ik 的 API 调用数”兼容口径，不包含 Pilz 内部 IK。API、请求/有效预算、actual_ik_s、原错误码和 collision_checked=false 都在原始响应中。

现有 seed=71070 控制 OMPL；KDL 私有 RobotState 的随机初始化没有公开插件种子设置接口，本轮没有改系统插件或虚构种子可控性，不宣称内部全过程逐位可复现。

## 冻结源码上的新四段结果

| 案例 | 完整到 preplace | 面 / 回退 | 总规划(s) | 预放置 IK API(s) | transit(s) |
|---|---|---|---:|---:|---:|
| c02（完整40箱配置初态） | 是 | FRONT / 无 | **13.055027** | 0.000245 | 8.994054 |
| c04（合法归档起点、剩余36箱） | 是 | FRONT / 无 | **86.430988** | 0.000643 | 73.153627 |

每案均为 **2 次显式 IK API、2 次 OMPL、2 次 LIN、0 PTP、0 先验**。一次预放置 IK 后只有一次持箱 RRTConnect。所有前缀重新生成；Task、父阶段、实际首末点连续，未导入旧轨迹。终端仍是同一附着箱体，无 RELEASE。实际箱体终点误差：c02 1.50e-6 m / 5.64e-7 rad，c04 3.19e-9 m / 7.93e-9 rad；原生终点检查通过，正常自由空间规则已恢复。

| 分项(s) | c02 | c04 |
|---|---:|---:|
| 目标集合生成 | 0.013744 | 0.015970 |
| 面目标几何 / 显式接近 IK 包装 | 0.013621 / 0.061518 | 0.013418 / 0.057338 |
| pregrasp | 1.154522 | 10.421075 |
| contact / 接触终端几何 | 0.223458 / 0.008771 | 0.241364 / 0.008915 |
| 附着上下文 | 0.007294 | 0.007163 |
| 抽离目标 / extraction | 0.349248 / 0.472065 | 0.349344 / 0.455580 |
| 已返回抽离点核查 / 分离终点 | 1.206572 / 0.081648 | 1.167008 / 0.078486 |
| 固定预放置目标 | 0.141597 | 0.134943 |
| 预放置 IK（含构造、往返及返回处理） | 0.050041 | 0.047665 |
| transit / preplace 终点核查 | 8.994054 / 0.093668 | 73.153627 / 0.080416 |
| 初始化（不计规划） | 1.711744 | 1.702307 |
| 其中场景准备（包含于初始化） | 0.013117 | 0.012900 |

总规划包含所有阶段、候选处理及失败尝试；本次没有换面失败。API 时间包含在 IK 包装时间内，原生检查与求解包含于运动请求/阶段时间，不能再次相加。记录未覆盖的组织余额仍是未归因时间。

c02 保持成功；固定输入对照没有显示可辨识的额外 IK 成本（两者均约 0.3 ms）。本次 c02 总时间比旧轮 8.586939 s 更长，主要对应 transit 8.994054 s 对旧 4.615810 s，不能把非成对任务波动归因为 IK 开销或宣称性能提升。c04 新增完整持箱路径的主要成本为 transit：其请求约 73.042245 s，包含 101035 次 Clearance 状态检查、累计 26.921953 s Clearance 检查。IK 恢复已验证，搬运成本没有解决，本轮不继续优化。

两例请求级去重后的 Clearance 状态数为 10547 / 112454，实际 FCL 查询 543211 / 3850024。ProcessSealInput、预解析、AABB/窄阶段固定关系复用仍开启；实际响应 pair_profile_enabled=false，未测量逐对象对时间为 null。未追加密集复检。

## 版本、验证与复现

实测源码：**9a2856be05ee45560b8e3c719e1964824089d2c2**；基线 959bbffa，无远端增量、起始工作区干净。冻结后没有修改规划代码。最终报告提交建立在该源码提交之上，两个 SHA 表示不同提交，源码与报告均为 UTF-8。

worker SHA256：d91ee76fb2a00643eb19596b595b7479ecec55e383f617bea1d85c6a6e0e89f9。Release / -O3 -DNDEBUG / 构建 -j2；Xeon Platinum 8352V，CPU 0–21，OMP/OpenBLAS=1；Ubuntu 22.04.5、ROS Humble、MoveIt/Pilz 2.5.10、MTC 0.1.3、FCL 0.7.0。99 项资产摘要与上一轮一致；依赖未升级。官方模型、20 kg 工具、42.5 kg 箱体和 5 mm 正常净空未改。

定向 Python **130 passed / 1.73 s**；m710_bounded_ik_test、m710_native_semantic_event_test、m710_request_evidence_test 均通过。测试覆盖真实 API 分派与预算传值、旧入口隔离、附着作用域、取消/截止、非法返回、保持实际起点、最多一次 transit、失败不能返回成功前缀。初次 C++ 编译的 JSON→set 插入类型错误已在运行前修正，原构建日志保留；没有因此重跑正式案例。

[机器可读汇总](experiments/m710_bounded_preplace_ik_20261017/summary.json)、
[逐面计时](experiments/m710_bounded_preplace_ik_20261017/timing.csv)、
[冻结身份](experiments/m710_bounded_preplace_ik_20261017/freeze.json)、
[实际完整命令](experiments/m710_bounded_preplace_ik_20261017/run-cases.sh)、
[固定 IK 对照](experiments/m710_bounded_preplace_ik_20261017/ik-comparison.json)。
原始请求/响应、轨迹、日志分别保存在同目录 c02、c04 和 ik-probe 子目录，旧失败与旧报告未覆盖。

隔离环境中的复现命令（另用不存在的输出目录）：

~~~bash
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
O=/tmp/preplace-ik-20261017
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
  --through-preplace --target carton_l07_c02 --output "$O/c02" \
  --worker-command "$O/worker.sh" --worker-binary "$O/build/m710_moveit_worker" \
  --source-commit 9a2856be05ee45560b8e3c719e1964824089d2c2 \
  --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100
# c04 改目标/输出为 c04，并增加 --initial-state "$O/c04-input.json"。
# 新预放置 IK 独立预算固定 0.10 s，不按失败调整。
~~~

本轮未接 place/release/withdrawal，旧完整取放入口未迁移；未运行 Isaac、真机、真实吸附、preflight、执行包、五箱/40箱任务、全量 pytest 或重型资格验收。没有新的五箱成绩。此次有限结果不证明稳定成功率、p99、硬实时或所有目标可达；在此停止。
