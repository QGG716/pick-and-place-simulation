# M-710 原理验证 step 1（2026-09-17）

本轮基于 `feat/v0.5-feasibility-core` 的 `9db77cb8a51e9bf1821c6e84e90a7632fbce6b26`。
操作前 `git fetch origin` 后本地与远端一致、工作区干净。服务器没有运行中的规划/Isaac；
旧 motion-quality 工作区有修改，未触碰。本轮独立目录：
`/root/autodl-tmp/m710-poc-step1-20260917`。交付提交身份以本报告所在 Git 提交及最终回复为准。

## 入口与模式

```bash
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
PYTHONPATH=src "$CPU" tools/run_m710_contact_unloading.py \
  --config configs/validation/m710id70_proof_of_concept.yaml \
  --output outputs/poc_new_run
```

普通规划及 continuation CLI 默认选择此 profile，并自动配对
`configs/simulation/m710id70_proof_of_concept.yaml`。显式旧 `--config` 保留原行为；
`--execution-config` 优先于自动配对；CLI approach/history/有限墙钟覆盖配置。
profile、三项理想假设、物理范围和非真机资格声明进入 motion、preflight、bundle 和 runtime。
执行配置与 motion 不同模式会拒绝。旧历史只作为几何提示；内容摘要、官方模型/工具身份、
关节语义、当前初态、完整路径和释放几何都重新验证，旧通过结论不继承。

## 计算限制与保留边界

| 调用层 | 原理验证行为 |
|---|---|
| 总请求、抓取 IK | `planning_wall_time_s: null`；不再附加 3 秒 IK 截止 |
| 候选、姿态/分支、阶段 RRT | 不附加 20/50/100、30、12 秒墙钟截止；显式请求截止仍有效 |
| Cartesian 连接 | 沿用请求截止（默认无）；保留样本、迭代、分支跳变限制 |
| 脱垛/放置 | 遍历已有展开路线；每个放置候选恢复有限迭代/采样配额；跳过零向量和重复方向 |
| 候选队列 | 取消固定 36/48 截断，完整有限池及两轮新种子重试交替执行 |
| 历史读取/适配 | 2/45/20 秒计算截止可空；保留文件数、字节数、结构与有限适配范围；记录文件筛选遗漏 |
| shortcut、下一箱前瞻、腕部与完整任务比较 | 默认交付前跳过可选优化；显式调用 POC 比较时无旧短墙钟窗口 |
| 最终验证、预检、导出 | 完整路径校验仍必需；不因返回/序列化延迟丢弃已完成结果 |
| CLI / 续箱等待 | 子进程无规划 timeout；POC 等待离线续箱默认无墙钟截止，物理时间暂停 |

`None` 表示无截止，0 表示无计算配额，正数为有限截止；不使用巨数或 JSON Infinity。
有限求解失败不构成物理不可达证明。SIGTERM/KeyboardInterrupt 写入取消事件，进度含候选身份、
seed/轮次、阶段及失败原因；不实现跨进程树恢复，也不继承旧 monotonic 时钟或完成证明。

未改变碰撞 margin、墙体、工具几何、自碰规则、IK/FK 容差、关节/径向/Jacobian 阈值、
质量/惯量/重心、驱动上限、PD、速度比例或轨迹插值。保留实际抓取、全环密封、机器人/刚性工具
环境碰撞、有限关节限制、分离进展/扰动、释放确认、丢箱/无进展与状态反馈停止条件。
J5/J6 自有工具及柔性杯邻箱验收例外范围不变。无新后端或重型依赖。

## 理想接收

新释放类型 `IDEAL_RECEPTION_RELEASE` 使用完整支撑多边形覆盖、既有放置族/法向、
现有 50 mm 最大交接高度、原 5 度支撑面姿态界及 1 mm 运行时穿透界。
接收区域检查与瞬时共面/接触报告分开；不要求所有底角同时落在 0.2 mm 带内。
保留当前箱体和非接收面的连续交接包络检查。CPU 预测的 `actual_support`、
`actual_landing_state` 不伪造成物理观察。

运行时仍须实际抓取当前目标、到达已规划接收姿态并真实移除约束。
接管后只该箱体禁用碰撞并转运动学，从实际释放姿态按仿真时间连续下降（无水平/旋转纠正），
再接已有 -Y/-X 路径。记录释放姿态、接收姿态、有限修正量。
事件为 `IDEAL_RECEPTION_ACCEPTED`，来源为 `RECEPTION_ASSUMED`；
真实模式仍要求 `ACTUAL_RELEASE_AND_RECEIVER_TOP_CONTACT`。
未观察到接触不写 `actual_top_contact_observed=true`。

`completed_carton_ids` 继续只表示真实接收；新 `ideal_received_ids` 与 `processed_carton_ids`
驱动 POC 续箱，不重复抓取已处理箱体。`OUTFED_ASSUMED` 仅在同一箱体完整包络越过 X=-3.2 m
后记录；ID 集合计数幂等，不提前删除未送出箱体，不声明后道物理资格。

## 完整结果保存

可行路径完成校验后直接返回；默认不先做完整任务腕部/下一箱优化。
CLI 保存 motion、独立 preflight、经读回校验的 bundle 到 `first_feasible/`，拒绝覆盖已有
输出目录的完整结果。基线与后续可选优化分开，partial path 不能升级为完整任务。
记录首次完整结果和首次可导出结果时间，默认后续优化耗时为 0。

## 验证与实际执行

所有数值/规划/Isaac 均使用原服务器环境，不重装依赖。
定向 pytest：profile、候选调度、预算结果保留、接收/释放、serial 和 continuation。
额外的历史兼容检查保留其需要外部旧证据而跳过的测试，不把 skip 写成通过。
未运行全量 pytest、104/129、40 箱清空或性能扫描。

真实规划经普通入口使用 `--history-source
/root/autodl-tmp/m710-ideal-outfeed-20260915-retry02/repo/outputs/ideal_plan01_final`。
`outputs/poc_final/` 的首箱为 `carton_l07_c02`：6 次轨迹 IK、9 次迭代、423 次状态校验、
528 个边采样；规划 11.038 s，preflight READY，无 blocker，导出及 bundle readback PASS。
4670 条命令，计划物理时长 90.386 s；首次可导出结果 11.948 s。
历史首轮接线失败的 `plan01.log` 及后续修复验证日志全部保留。

Isaac 只启动一次：`outputs/poc_single_world/isaac/`，新世界，与旧录像/完成数不合并。
使用 640×360、5 fps、1×物理时间，原 physics/control/contact 监测率。
实际结果、计数、阻塞、视频与源码快照身份在附属 evidence summary 中记录。

实跑 `workflow_cycle_completed=true`，包含实际抓取、搬运、解除约束、理想接收、机器人撤离和理想送出。
真实抓取 1、真实释放 1、实际物理接收 0、理想接收 1、理想送出 1、工作流处理 1。
`physical_cycle_completed=false`；不把理想接收声明为物理接收完成。
接管发生于 89.220833 s，局部连续下移 22.443 mm；99.275 s 时全包络最大 X=-3.200686 m。
只禁用 `/Validation/Scene/carton_l07_c02` 的碰撞。39 箱仍在场，1 箱保留 inactive 身份，共 40 箱。
本次 world_session_id 为 `1789622627.8082964`；世界已结束，不可作存活世界继续运行。

没有非预期机器人接触，也没有运行停止原因。运行时 `qualification_passed=false`，唯一未通过项为
`joint_efforts_within_limit: NOT_EVALUATED`，原因是实际驱动关节力矩比值不可用，**不是观测到力矩超限**。
模型逆动力学比值最大约 0.146，不能替代实际驱动力矩测量。
结果中的旧 `execution_qualified` / `simulation_execution_qualified` 是导入的预检元数据，
不能用其 true 覆盖运行时资格结论。本轮没有改动驱动力矩、PD、动力学或验收阈值来消除该问题。

实跑共 23826 物理步、99.275 s 仿真时间、约 815.098 s 回放墙钟时间；物理 240 Hz。
录像为 496 帧、640×360、5 fps、正常物理时间，末帧可解码。
**原始录像 HUD 存在旧显示错误**：把理想接收标成 `Actual received 1`，且沿用“落带后理想输送”说明。
JSON 来源与计数正确（实际接收 0）。原始录像保留、不改写；最终源码 HUD 已复用按来源分类的计数器，
分别显示 actual / ideal reception / ideal outfeed，并声明理想接收。未为显示修复追加物理试验。
末帧在 99.2 s，早于 99.275 s 的最终越界事件，故末帧 outfed=0 不代表最终未送出。

精简证据见 [evidence](validation/evidence/m710_poc_step1/)。大文件保留服务器：
`/root/autodl-tmp/m710-poc-step1-20260917/outputs/poc_single_world/isaac/`，
包括 `result.json`、`replay.mp4`、`actual_remaining_state.json`、`execution_events.json`、
`ideal_transport_events.json` 和 `evidence_manifest.json`。失败和取消日志保留在本轮服务器根目录。

### 实际命令及检查

以下均在本轮服务器工作区执行，未重新安装依赖：

```bash
export PYTHONPATH=src
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
$CPU -m pytest -q tests/test_proof_of_concept.py tests/test_serial_unloading.py \
  tests/test_same_world_continuation_runner.py tests/test_contact_candidate_scheduler.py \
  tests/test_feasible_result_budget.py tests/test_post_landing_transport.py \
  tests/test_adaptive_release_motion.py tests/test_default_history_adaptation.py \
  tests/test_wrist_transfer_quality.py
$CPU tools/run_m710_contact_unloading.py \
  --history-source /root/autodl-tmp/m710-ideal-outfeed-20260915-retry02/repo/outputs/ideal_plan01_final \
  --output outputs/poc_final
```

定向检查：**132 passed, 22 skipped**。跳过项需要外部 `M710_HISTORY_FIXTURE_ROOT`，不算通过。
HUD 最后修复后复查 `tests/test_proof_of_concept.py`：20 passed；脚本编译及从实际源码提取的 HUD 块
用本轮真实记录执行，显示 actual 0 / ideal reception 1 / ideal outfed 1。
65 候选 / 195 次不同种子调度和 6 条脱垛路线的注入测试只证明生产接线，不能称为机器人规划成功。
正式数值规划另由上面的普通入口完成。显式零预算输出 `FAIL_CLOSED`，初始检查标记
`NOT_EVALUATED / PLANNING_WALL_CLOCK_DEADLINE`，432 个候选未搜索，0 次 IK/连接；不误判初态非法。
真实 SIGTERM 取消保留候选 ID、seed、轮次、阶段及 `CANCELLED` 原因，不声称支持跨进程恢复。
旧 execution config 配新 motion 被 `motion simulation profile mismatch` 拒绝。
不可变基线三份文件与首次导出文件逐字节一致。

唯一 Isaac 命令（在实跑快照根目录；现有源 USD 及导入证据只用于已校验模型复用）：

```bash
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root
export XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg
export XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg
export XDG_DATA_HOME=/root/autodl-tmp/data/xdg
/root/autodl-tmp/envs/isaacsim-clean/bin/python scripts/isaacsim_fanuc_replay.py \
  --bundle outputs/poc_final/replay_bundle.json --project-root "$PWD" \
  --usd-directory outputs/poc_single_world/usd \
  --reuse-usd-entrypoint /root/autodl-tmp/m710-official-dynamics-20260910/isaac_usd/m710id_70_official_8/m710id_70_official.usda \
  --reuse-usd-run-evidence /root/autodl-tmp/m710-official-dynamics-20260910/isaac_outputs/initialization_render_sync_logged_final/run_status.json \
  --reuse-usd-source-contract /root/autodl-tmp/m710-official-dynamics-20260910/repo/outputs/m710_official_dynamics_20260910_final/initialization_contract.json \
  --output outputs/poc_single_world/isaac --record-video --video-preview-speed 1 --maximum-segments 1
```

### 源码身份与下一步

实跑开始后，独立 `final-source/` 内修正了零预算初态错误归因、调度预算证据的 null 表示及 HUD。
没有修改正在运行的源码或世界。另有 Git 规范换行产生的字节差异；因此实跑快照和最终提交不是同一源码身份。
两套 motion、preflight、bundle 与指纹均保留，不能交换 bundle 绕过源码绑定检查。
最终 Git-index 源码重新执行正式 CPU 规划/预检/导出/读回；关节数值路径与释放预测和实跑完全一致。
最终产物位于 `final-source/outputs/canonical_hud_plan/`：规划 11.210 s，首次可导出结果 12.147 s，
preflight READY、bundle readback PASS、后续可选优化 0 s；该包没有另行进行物理运行。
最后 HUD 修复只更改显示计数和说明，不改变物理循环、控制、接收交接或监测。
具体树身份、motion 指纹和最终 CPU 耗时见 `source_scope.json` 与 `canonical_delivery.json`。
本报告所在提交包含最终源码和精简证据，提交 SHA 由最终交付回复及 Git 历史给出。

本轮目标流程完成；没有碰撞/运动学阻塞该单箱。下一步需独立处理驱动力矩遥测缺失，
才能判断运行时动力学资格。真实接收、后道物理资格和多箱连续执行尚未由本轮证明。
未追加整排、第五箱复现或第二次物理运行。
