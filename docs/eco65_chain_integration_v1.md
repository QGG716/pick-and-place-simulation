# ECO65 轻量原生规划与运动学回放入口（2026-10-19）

本步增加显式 `evaluation_profile=chain_integration_v1`。默认仍为 `strict`；
旧计划缺少该字段时按 strict 处理。新配置只适用于 ECO65-B、CAD 密封轮廓、
`planning_only=True`、`planning_mode=cold_from_scratch`。它不是关闭碰撞的模式。
输入仍为当前冻结场景指定的单箱目标，尚未接入视觉、标定观测、J1 有向往返或连续两箱任务。

## 路径检查与身份

初始化绑定 evaluation profile，每个 worker 请求及响应都携带该字段；同一 worker/任务不能改变配置。
NativeTask、阶段、原生请求/结果、计划、导出源引用、回放报告继续传递配置，计划指纹覆盖这些字段。
模型、CAD、分块凸体、12 个物理杯及未选杯碰撞体、场景和碰撞容差完全复用。

保留原生 IK/FK、有限数值、关节限位、IK 后状态检查、Clearance/ProcessPolicy、
搜索状态谓词、起点检查、q_goal 检查、实际输出末点和原生 MTC 来源。
计划验收保留关节顺序、时间戳、阶段连续性、TCP 身份、抓放对象/变换、附着生命周期及冻结世界身份。
simulation independent cups 授权不改变硬件 `independent_control_verified` 证据。

新配置只遍历一次输出路径。逐边使用
`N=max(1, ceil(max(abs(q_next-q_prev))/edge_resolution_rad))`，本次仍为 **0.002 rad**。
检查全部原始节点和必要的插值状态，共享节点只检查一次；同一 RobotState 依次进入净空、
接触过程及普通场景碰撞检查。最后一点额外执行 `process.check(state, true)` 的端点语义，
以及关节目标或 TCP FK 目标检查（原生目标容差 1e-6 m / 1e-6 rad）。
每段记录规则、采样数、三类实际调用数、端点结果和耗时；不写大型逐采样状态报告。

回放沿用现有 `q=a+s(u)*(b-a)`、`s(u)=10u³-15u⁴+6u⁵` 的时间参数化。
所有关节共享单调进度，因此采样和回放经过完全相同的关节线段；时间参数化不更改节点。
这只是有限采样检查，**不宣称连续无碰撞证明**。
超时、异常和中途拒绝保留 `completed=false` 及原因，不转为通过；客户端 RPC 超时仍终止 worker。

| 项目 | chain_integration_v1 | strict |
|---|---|---|
| 搜索内状态、起终状态检查 | 保留，并记录真实调用数 | 保留 |
| 输出路径 | 联合 `BASIC_SAMPLED_PASS` | 原双重密集 `PASS` |
| Clearance / ProcessPolicy 密集 checkPath | `dense_path_calls=0` | 保留原算法 |
| Python 每段密集 LIN audit | `NOT_RUN_PROFILE` | 保留 |
| 主入口自动 MuJoCo review_plan | `NOT_RUN_PROFILE`，不因此重试 | 保留 |
| 回放 review.json | 不需要，不生成占位文件 | 仍要求同计划 PASS |
| 动力学/力矩/物理接收资格 | `NOT_EVALUATED` | 本次不修改原入口 |

`planner_internal_process_checks=true` 只表示谓词已启用；真实次数另见
`clearance.counts.search.queries`、`process_policy.planner_state_checks`。
输出次数见 `basic_path_check`，两者不相互冒充。求解时间为 `native_solver_s`，
输出检查、RPC 和任务总时间分别保存。

## 使用命令

以下在现有服务器工作副本执行；需先部署本次源码并编译 worker，不升级依赖。
目录 `chain_integration_example` 必须是新的；复用 run61 当前布局和同源 HOME。

```bash
cd /root/autodl-tmp/eco65-native-20261015/repo
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=2
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
S=outputs/eco65_native/small_20261019_run61/build
O=outputs/eco65_native/chain_integration_example
mkdir "$O"

chroot "$R" /bin/bash -c 'source /opt/ros/humble/setup.bash; cd /work-eco65-20261015; cmake -S ros2/m710_moveit_backend -B build-chain-20261019 -DCMAKE_BUILD_TYPE=Release; cmake --build build-chain-20261019 --target m710_moveit_worker -j2'
sed 's@/build/m710_moveit_worker@/build-chain-20261019/m710_moveit_worker@' ../worker.sh > "$O/worker.sh"
chmod 700 "$O/worker.sh"

"$P" tools/run_eco65_moveit2_single_carton.py plan \
  --snapshot "$S/snapshot.json" --home-candidate "$S/home_candidate.json" \
  --worker-command "$PWD/$O/worker.sh" --evaluation-profile chain_integration_v1 \
  --preferred-grasp-face top --task-seconds 3600 --stage-seconds 120 \
  --rpc-seconds 600 --attempts 1 --output "$O/plan"

# 只有完整新计划通过基本检查，才执行以下两步；不导入历史成功轨迹。
"$P" tools/run_eco65_moveit2_single_carton.py export \
  --snapshot "$S/snapshot.json" --plan "$O/plan/plan.json" \
  --evaluation-profile chain_integration_v1 --output "$O/export"

# 复用此前验证的进程局部系统运行库配置；不安装或替换库。
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y OMNI_KIT_ALLOW_ROOT=1
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libgcc_s.so.1 \
"$P" scripts/isaacsim_replay_eco65_native.py \
  --snapshot "$S/snapshot.json" --plan "$O/plan/plan.json" \
  --evaluation-profile chain_integration_v1 \
  --assets outputs/eco65_native/warehouse_assets \
  --workstation "$O/export/workstation_D.usdc" --preview --output "$O/replay"
```

`--preview` 只显示各阶段中点和边界等少量代表帧，每个姿态累积 4 个真实 RGB 样本；
它不是完整实时视频。省略此选项则保持原 640×360、5 fps 录制设置。
回放始终 `KINEMATIC_REPLAY`，计划始终 `PLANNING_ONLY_NOT_EXECUTABLE`；时间线停止、物理步数为零。

真实原生定向检查入口：

```bash
"$P" tools/check_eco65_chain_integration.py \
  --snapshot "$S/snapshot.json" --home-candidate "$S/home_candidate.json" \
  --worker-command "$PWD/$O/worker.sh" --output "$O/directed"
```

其 `check_path_offline` 调用复用生产路径检查器和同一个完整场景，仅接受 chain profile，
输出标记为 `OFFLINE_DIAGNOSTIC_NOT_A_PLAN`。诊断路径不会成为 native-cold 规划输入。

## 本次定向验证

代码从 `da3b9233d84f915a7f50e1fd8e426fed3f54b831` 开始；开始时已 fetch，远端无后续提交且工作区干净。
复用原 ROS Humble / MoveIt 2.5.10、MTC/Pilz/OMPL、Isaac 隔离环境。
未重建 CAD、移动场景、删除障碍、升级依赖或执行物理循环。

- 现有 ECO65 定向测试：51 passed、3 skipped（可选本地 C++/USD 条件缺失）；新增配置/入口测试：23 passed。
- 保留 FANUC 入口/回放合同：19 passed。
- 实际 worker 编译成功；原生 strict Clearance 与 ProcessPolicy 定向回归通过。
- `chain_20261019_run03/directed/report.json`：PASS，41.441 s。
  碰撞起点与目标分别返回 `INVALID_START_CLEARANCE` / `INVALID_GOAL_CLEARANCE`，
  实际碰撞对为 `part_000 / eco65_mast_mast_part_019`。
  原生 FCL 的 -1 哨兵仅表示碰撞，不解读为 1 m 穿透深度。
- 中间路径以合法 HOME 起终，包含已由实际 FCL 判为碰撞的内部节点。
  0.002 rad 检查在第 967 个状态、第一条边进度 0.5564516 提前拒绝：
  `link_3 / part_039` 净空 2.320375 mm，小于原 2.4 mm 要求。
  这是实际有限采样拒绝，不是连续碰撞证明。
- 合法短诊断路径检查 3 个状态，末点过程检查 1 次；两种旧密集遍历均 0 次。
  混用 strict 和未知初始配置均由真实 worker 拒绝。
- 最初两份诊断报告保留为失败：测试脚本先未捕获预期协议拒绝，随后错误复用已被客户端关闭的连接。
  已修正诊断脚本；不把这两份历史报告改为 PASS。

完整单箱任务本轮只启动一次：`chain_20261019_run01/plan01`，场景目标 `carton_r00_l03_c01`。
十二箱快照指纹 `ad20a4a45f4a9461a2b9a458d82360fe37374e3dd8bf01c4cfae1052dbd89f51`，
新编译 worker SHA-256 `c1f9e3f878e421d81474abf920abced325fa3e5fece97d3d0f679951f56c13a9`。
该次运行绑定 run01 的源码部署清单；运行期间交付代码只继续补充 Python TCP/导出引用验收、
摘要字段和诊断脚本的预期错误处理，原生 C++ 检查器没有变化。最终合同测试使用交付代码，
没有为这些补充重新启动完整规划。
本次最终结果为 **NO_BASIC_CHECKED_COMPLETE_PLAN**，终止原因为
`NATIVE_TASK_BUDGET_EXHAUSTED`。配置任务预算 3600 s，报告记录总耗时 **3664.109613 s**
（包括初始化及第一份失败记录写入，最终汇总文件自身的写入不在此计时内）。
最后活动抓取为索引 **699 / 1488**（零起始）的正面候选
`78ee2223797ea1d65bd4204adf080513dbea3cff5abd7b356182f28d8d0b546e`，
选中杯 `part_020_seal`。该分支通过接触和抽离，但未完成转运；
最近的 `transit-22754`、`transit-22755`、`transit-22756` 均返回 `NATIVE_PLANNING_FAILED`，
原生求解原因均为 `TIMED_OUT`，原生求解耗时分别为 120.083996、120.454264、120.318596 s，
归档 RPC 耗时分别为 121.311121、121.744796、121.593010 s。
最后一次转运搜索记录了带载目标与邻箱 `carton_r00_l03_c00` 的接触拒绝，
以及与 `carton_r00_l02_c01` 的 2.665686 mm 间距低于原 3.4 mm 要求；
这些是搜索状态被拒的证据，不是对所有路径不可行的证明。
随后 `extraction-22757` 成功，下一转运目标生成/状态检查期间任务预算耗尽。
搜索记录为 `INTERRUPTED`：开始尝试 700 个候选（索引 0–699，最后一个未完成），
尚有 788 个候选（索引 700–1487）未搜索。
120 次原生运动请求中有 17 次局部成功、103 次失败；这表示本次有界搜索未得到完整计划，
不能据此声称所有候选均不可达。

本次没有 `plan.json`，导出与代表帧回放均为 **NOT_RUN_NO_COMPLETE_PLAN**；
没有生成空 review，也没有执行 MuJoCo 独立复核或动力学。
未改变布局或障碍、未关闭碰撞、未启动第二次完整规划。
精简结果和完整原始文件哈希保存在忽略目录
`outputs/eco65_native/chain_20261019_delivery01/`，服务器保留 run01 完整记录。

以下是早期已通过、随后因后续转运失败而回溯的阶段证据，**不是完整计划验收**：

| 阶段 | 原生求解 s | 一次路径检查 s | 状态数 | 搜索内过程调用 | 旧密集调用 |
|---|---:|---:|---:|---:|---:|
| pregrasp-39 | 1.703482 | 21.558997 | 1808 | 136 | 0 / 0 |
| contact-41 | 0.239089 | 1.961387 | 162 | 18 | 0 / 0 |
| extraction-48 | 0.115232 | 0.610495 | 49 | 8 | 0 / 0 |

两段 LIN 末点 `endpoint_process_checks=1`；接触 TCP 位置/姿态误差为
2.681237e-7 m / 4.973062e-7 rad，抽离为 1.696423e-8 m / 3.101260e-16 rad。
Python 密集 LIN 审计与自动独立复核均由控制流回归验证未调用。

## 文件与后续入口

| 文件 | 目的 |
|---|---|
| `ros2/m710_moveit_backend/src/basic_sampled_path.h` | 单遍共同进度采样、实际 FCL/过程检查、失败/超时证据 |
| `ros2/m710_moveit_backend/src/worker.cpp` | 配置绑定、响应传播、原生输出分流、真实末点 FK、离线定向诊断 |
| `clearance.h` / `process_policy.h` | 增加实际调用计数；保留旧 strict 算法 |
| `src/unloading_sim/eco65/evaluation.py` | 配置与采样证据合同、strict 回放 review 门槛 |
| `src/unloading_sim/eco65/native_task.py` | 配置贯穿、跳过额外 LIN 审计、结构与事件验收 |
| `tools/run_eco65_moveit2_single_carton.py` | CLI、直接保存基本检查计划、分项运行报告 |
| `src/unloading_sim/eco65/native_replay.py` | 新配置无 review 回放，保留计划/事件验证 |
| `scripts/isaacsim_replay_eco65_native.py` | 可选 review、配置一致性、代表帧预览 |
| `tools/export_eco65_warehouse_workstation.py` | 导出引用记录 evaluation profile |
| `tools/check_eco65_chain_integration.py` | 当前真实模型/场景的有限定向原生诊断 |
| `tests/test_eco65_evaluation.py` | 混配、缺失/伪通过证据、strict 兼容与跳过复核的控制流回归 |

下一步可在现有 `verify_snapshot` / `scene.native_task` 目标身份入口接入视觉输出和标定变换，
随后仍交给 `NativeTask` 原生生成。当前没有将场景配置伪装为视觉识别结果。
本步不改变 J1 方向、搜索布局或连续任务定义。私有模型、原始调用、快照、USD、图片和视频均留在忽略目录。
