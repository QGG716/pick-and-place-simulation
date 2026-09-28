# M-710 旋转任务 TCP LIN 与固定单箱验证（2026-09-28）

本轮评审基线为 `85e968fd5b3e12a05938e9ff1a00f1807e532904`。开始时 HEAD 与 origin 均为该提交，工作区干净；只修改 `feat/v0.5-backend-moveit2-mtc-pilz`，未合并其他分支。

本轮规划目标完成：真实偏移旋转 TCP 后缀、固定完整单箱几何计划、独立执行前检查和最终执行包 TCP 审计均通过。只执行了一次 Isaac：抓取、释放、撤离及理想接收/出料完成，但执行资格未完全通过，原因是实际位置驱动输出力矩不可用（NOT_EVALUATED），不是已测得力矩超限；未证明物理接收。

## 实现与语义

选择单一路线：在导出的 MoveIt 模型中增加无碰撞几何的固定 `m710_virtual_task_tcp` link，固定变换直接来自原核心 `flange_from_virtual_task_tcp`。操纵器 SRDF 链的 IK tip 指向该辅助 frame；MTC MoveTo 的 IK frame 是此 link 的单位偏移。官方实体 link、关节轴、安装位置、202 个工具碰撞体和附着箱体的 flange 坐标不变。

实际核查锁定源码：

- [MTC 0.1.3 发布源码 PipelinePlanner](https://raw.githubusercontent.com/ros2-gbp/moveit_task_constructor-release/release/humble/moveit_task_constructor_core/0.1.3-1/src/solvers/pipeline_planner.cpp)：目标会乘 `offset.inverse()`。
- [MoveIt 2.5.10 TrajectoryGeneratorLIN](https://raw.githubusercontent.com/moveit/moveit2/2.5.10/moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_lin.cpp)：从请求选定 link，计算该 link 起点 FK；KDL Path_Line 与 SingleAxis 旋转插值生成整段，再对同一 link 求 IK。
- [2.5.10 trajectory_functions](https://raw.githubusercontent.com/moveit/moveit2/2.5.10/moveit_planners/pilz_industrial_motion_planner/src/trajectory_functions.cpp)：`getConstraintPose` 只减去目标姿态旋转后的 target_point_offset，不能使旧法兰插值变成任务 TCP 插值。
- 服务器实际安装头文件 `TrajectoryGenerator::generate(..., sampling_time=0.1)` 和 `PlanningContextBase::solve()` 确认默认原生采样 0.1 s。本轮全部通过用例在此采样下满足原工艺容差，没有手工插值伪装 Pilz，也没有调宽容差。

因此整段满足 `T_WF(s) = T_WT(s) * inverse(T_FT)`，不是只变换法兰终点。任务 TCP 原有 0.25 m 平移及旋转偏移都保留。实际 FK 比较覆盖三个关节状态；第二个零平移偏移 TCP 上下文也进行原生 LIN 调用，再恢复第一个上下文核对 FK。

常驻进程最多保留四个独立模型上下文，每个绑定完整模型/工具/策略/TCP 身份并保有自己的 pipeline、场景和可变查询工作区。相同 init 内容复用模型；同身份不同内容失败关闭。RNG 只在进程首次初始化，后续上下文不能偷偷重置。此上限是适配器资源范围，不是 Pilz 的运动限制。

新增 `moveit2_tcp.audit_linear_tcp` 对执行器实际遍历的关节直线边进行密集检查：每边采样间隔数量为 `2*max(1, ceil(max(abs(dq))/.04), ceil(4*sum(abs(dq))/.0025))`，与已有保守边网格相同。检查起终点、线段偏差、最短旋转插值误差、位置/姿态共同进度、回退及越界；纯旋转使用相对旋转轴上的有符号进度。保留位置 0.0001 m、姿态 0.0002 rad 容差；不是连续扫掠证明。原生净空和 Python 权威检查均保留，原生输出通过而权威拒绝仍返回 `NATIVE_AUTHORITY_MISMATCH`。

能力判断只做结构/上下文检查，不做规划、碰撞或可达性判断。合法固定 TCP 的旋转请求现在受支持；错配 TCP、无效变换及未覆盖阶段仍明确退出。历史前置判断和独立后缀提取共享同一个纯几何入口，避免两套工艺公式漂移。

## 定向真实结果

证据根目录：`docs/validation/evidence/m710_moveit2_tcp_20260928/`。

| 用例 | 原生规划 s | 原生输出边检查 s | TCP 审计 s | Python 权威 s | 段端到端 s |
|---|---:|---:|---:|---:|---:|
| 恒姿态平移 PASS | 0.008195 | 0.017703 | 0.002178 | 0.370540 | 0.481836 |
| 非零偏移纯旋转 PASS | 0.008675 | 0.086708 | 0.011489 | 2.062295 | 2.226176 |
| 非零偏移平移＋旋转 PASS | 0.008533 | 0.097644 | 0.014279 | 2.341016 | 2.517987 |
| 真实带载旋转后缀 PASS | 0.027403 | 0.324847 | 0.034193 | 8.177127 | 8.650449 |
| 小范围 PTP PASS | 0.010142 | 0.020202 | 不适用 | 0.403354 | 0.526791 |

零平移偏移旋转使用同一 resident 的第二模型上下文，实际 LIN 调用一次，原生/密集 TCP/权威均通过，总耗时 2.054714 s（含新上下文加载）。四项语义测试合计四次实际 Pilz LIN；恢复旧模型的 FK 不算规划调用。完整数据、模型身份及每个上下文成本见 `semantics.json`。这些构造运动只证明实现，不计入真实任务成功率。

真实后缀从冻结候选和当前工艺纯计算提取，提取时用禁止调用的检查替身证明重型前缀检查次数为零。`suffix.json` 保存原始精度的起点 q、任务 TCP 起终位姿、world 参考系、实体接触 frame、接触 q、附着变换、接收意图和全部既有释放高度；只运行第一个原有 0.025 m 高度。实际 Pilz 返回 18 节点，净空输出检查 635 次、搜索/管线可行性检查 18 次，均有效。TCP 审计 619 个去重采样，最大线段偏差 `5.822868602313926e-6 m`，最大姿态误差 `8.846831179575685e-6 rad`。查询次数不等于树节点；此处 LIN 没有 OMPL 树。

真正不支持的错配 TCP 上下文负例，在 `after_contact_ik_before_contact_or_prefix_collision` 退出，0 次原生规划、0 次重型状态/边检查；候选判断 0.004528 s，含初始化脚本 1.015249 s。19 项定向 pytest 通过（1.41 s）；包括端点正确但任务 TCP 中段错误的法兰直线负例、节点和中点均正确而四分之一边错误的负例、姿态进度与平移不同步、回退和无效变换。

## 完整任务与执行层级

| 验收层级 | 结果 |
|---|---|
| 真实原生调用 | PASS，实际 Pilz LIN |
| 旋转任务 TCP 工艺段 | PASS，独立后缀及完整任务中的该段均通过 |
| 固定完整几何任务 | PASS，189 节点，原完整合同通过 |
| 独立执行前检查 | READY，0 个 simulation_readiness_blockers |
| 现有执行入口及最终 C2 路径 TCP 审计 | PASS |
| 单次 Isaac 实际执行/接收 | 流程完成；资格未通过：力矩检查 NOT_EVALUATED。理想接收 1、理想出料 1、物理接收 0 |

完整任务使用 `carton_l07_c02`、冻结 40 箱、原始实际 home 状态及固定历史候选；首解 `first-accepted-full-task.json` 独立保存。stage_ranges：pregrasp 0–4、contact 4–7、extraction 7–49、transit 49–160、place 160–161、withdrawal 161–188。事件为 ATTACH@7、RELEASE@161、RELEASE_RETREAT_COMPLETE@188；支撑释放检查没有额外运动。历史自由前缀/脱垛提示均重新通过当前权威全边检查；接触、放置、撤离由原受控工艺模块生成；transit 中 143–160 是真实 Pilz LIN，未拼用上一轮带载首解。MTC 组织 6 个阶段，完成世界对象→附着→释放转换，末态附着集合为空；这些组织步骤不计作运动规划调用。

完整规划端到端 740.794261 s，模型/初始化 0.929307 s，固定候选 739.823009 s。原核心累计碰撞验证 734.131513 s 为嵌套计时。以下是路径检查入口的按阶段累计实测，不与总耗时相加：

| 阶段 | 权威路径检查调用 | 累计 s |
|---|---:|---:|
| pregrasp | 1 | 71.262651 |
| contact | 4 | 6.509189 |
| support-release | 1 | 0.015164 |
| extraction | 1 | 175.941302 |
| transit | 2 | 448.316684 |
| place | 1 | 0.004522 |
| withdrawal | 28 | 36.893589 |

完整任务中的唯一原生 LIN：规划 0.028921 s（其中净空可行性 18 次、0.010434 s）、原生输出边检查 0.333288 s（635 次全部有效）、TCP 审计 0.034628 s、Python 权威 8.893833 s；本段端到端 9.363746 s。MTC 组织计时 0.032608 s，原最终合同复核 0.072741 s。没有 OMPL 调用，没有 seed 扫描。

独立 preflight 耗时约 0.82 s，原执行包导出约 0.45 s（shell wall clock，精度 0.01 s）；包含 5,498 条采样命令，规划执行时间 106.163639 s。`executor-tcp.json` 对最终 `C2_piecewise_quintic_rest_to_rest` 关节参考路径再次密集审计，0.609123 s，通过；原生段边界在实际参考中保留；原有共线节点合并使该段由 18 个原生节点变为 17 个执行参考节点，已对实际 17 节点的全部关节边重新审计。所有必要输入身份、完整事件、有限关节时间限制与原生时间下界仍由原入口校验。

`preflight.json` 的 machine_qualified=false 是原有设备认证字段，与本次 simulation_execution_ready=true 分开，不据此改写物理结论。

固定候选通过现有标准单箱入口运行。仓库精简 fixture 不包含完整历史报告的内容指纹及旧策略/实现元数据，因此生成独立 `.hint.json`：保存原文件 SHA256，明确 HINT_ONLY、NOT_VALIDATED、validation_inherited=false；缺失旧元数据为 null，不用当前身份冒充历史身份。原 `HistorySource` 内容校验和 `compatible_hint` 资产、FK、尺寸、场景约定检查仍执行。命令明确限制一候选、零新候选扫描，保留全部当前工艺和验收门。

已有 `m710_moveit2_perf_20260928/first-accepted-loaded.json` 未修改、未拼入完整任务，其 SHA256 仍为 `16e6cb3d8d79d8c2b87c56c0b45a86dcea0fe3b02d68e47c8335a457b70f59d5`。

## 单次 Isaac 结果与具体阻塞

运行 ID 为 `m710-tcp-lin-20260928-single-box`，新 world/session ID 为 `1790570161.5443099`，未继承历史完成数。原监督入口从真实初始状态执行，同一 `carton_l07_c02` 原始动态刚体、42.5 kg 箱体与 20 kg 工具保留；没有从已附着状态开始。现场 40 个动态箱体完成初始稳定性检查。

- 实际抓取 1、实际释放 1；附着时实际接触杯 40/72。抓取闭合 4.125 s，垛箱支撑脱离 29.850 s，自由搬运门通过 30.8125 s，释放命令 93.100 s，约束移除确认 93.120833 s，理想出料 103.175 s，最后撤离完成。
- `workflow_cycle_completed=true`，`physical_cycle_completed=false`；`ideal_reception=1`、`ideal_outfeed=1`、`actual_reception=0`。原 POC 在真实释放约束移除后进行理想接收，实际释放高度约 0.024789 m，理想接管下移约 0.022671 m；不能将该假设写成真实物理落料接收通过。
- `qualification_passed=false`，唯一失败项 `joint_efforts_within_limit` 的详细状态是 **NOT_EVALUATED**，原因 `joint effort ratios are unavailable`。原脚本的 `drive_effort_output_qualified=false`，因此 `effort_limit_ratio=[]`。显式 effort 输入通道不是位置驱动实际输出；PhysX 约束反力也不是已校准执行器力矩。模型逆动力学比值虽有值，但明确不含外部箱体，不能替代实际带载执行器力矩。没有修改既有判定，也没有第二次物理运行。
- 其余适用资格检查通过；意外机器人接触与提前接触输送机均为 0，`runtime_stop_reason=null`，51,010 条运行反馈无首个失败。关节跟踪峰值 0.000977099 rad；附着误差峰值约 0.0000129025 m / 0.0000275230 rad。
- 全流程实际 TCP 跟踪峰值为 0.002027454 m / 0.001717493 rad，这是物理执行反馈相对参考的误差，**不是**已通过的几何 LIN 审计误差。本轮没有证明实际机器人保持 0.0001 m / 0.0002 rad 的工艺精度，不能从规划 PASS 推导该物理精度结论。

| 物理运行计时 | 实测 |
|---|---:|
| App 启动墙钟 | 13.568059 s |
| 已核验官方 USD 复用 | 0.020011 s |
| 物理重放墙钟 | 1226.922028 s |
| run_status 起止墙钟（包含初始化/收尾） | 1255.425336 s |
| 实际重放物理时间 | 106.270833 s |
| 物理步数 / 频率 | 25,505 / 240 Hz |
| 原配置录制 | 640×360，5 fps，531 帧，物理时间倍率 1.0 |

这些是不同或嵌套口径，不相加成新总耗时。没有单独测量监测与渲染开销。视频已只读检查元数据及抓取/末尾画面。结果和事件位于 `isaac/result.json`、`isaac/execution_events.json`、`isaac/runtime_feedback.json`；视频为 `isaac/replay.mp4`。`isaac/raw-evidence.tar.gz` 保留全部原始运行文件和 driver.log，包括逐步关节遥测与实际 frame 状态；`isaac/archive-manifest.json` 保存哈希。最终 result SHA256：`f4a8a2c777898515e7e4a518adfb01d2b567473cef52c2a53e2fb29bda8d77d9`。

剩余具体阻塞是实际驱动输出力矩的可信测量/资格判定，以及尚未验证的物理接收与上述物理 TCP 精度。它们不归因于 LIN 不支持或几何不可达；本轮按一次物理结果停止，没有扩大试验。

## 身份、计时口径和复现

最终 worker SHA256：`a0ff29a86f4e9183f5706a3e6d84843c04e1a78d21adfd950c079bdd581953e4`。Release `-O3 -DNDEBUG`，没有 fast-math。锁定依赖 MoveIt/Pilz 2.5.10、MTC 0.1.3、FCL 0.7.0、OMPL 1.7.0。完整包版本、环境、编译选项和源码哈希见证据中的 lock/manifest 文件；native identity 的旧 `baseline_sha=baeb2a7...` 是继承的协议元数据，本轮源码由 review_baseline、文件哈希及最终提交身份标识。

最终版本的语义集、真实后缀、完整任务串行运行。小范围 PTP/提前退出回归与正在进行的 Python 完整任务复检有短暂重叠，不用这些数据计算性能提升。净空计时嵌套于规划或输出检查，不能相加成端到端时间。不报告 P95、吞吐提升、总体成功率或速度倍数。

Windows 上传换行与 Git LF 的映射单独记录（未修改的 frozen_candidate.json 原始上传字节可能为 CRLF，归一化 LF 后身份一致）；仅转换 CRLF，LF 重建后二进制逐字节不变。开发期通过后缀、输入格式/元数据拒绝和最终身份补齐前中止的试跑与最终证据分开保存，不冒充最终版本验收。

服务器实验目录 `/root/autodl-tmp/m710-tcp-lin-20260928`；Humble rootfs 构建 `/work-round4/build`。在实验 repo 和原 CPU venv 中：

```bash
export M710_MOVEIT_COMMAND=/root/autodl-tmp/m710-tcp-lin-20260928/worker.sh
export M710_MOVEIT_ASSET_ROOT=/work-round4
python tools/check_m710_rotating_tcp.py --suite semantics --output semantics.json
python tools/check_m710_rotating_tcp.py --suite suffix --output suffix.json
python -c "import json; d=json.load(open('suffix.json')); assert d['status']=='COMPLETED' and d['results'][0]['failure'] is None"
python tools/run_m710id70_layout_single_carton.py --backend moveit2 --target carton_l07_c02 --fixed-history-fixture tests/fixtures/moveit2/frozen_candidate.json --output full-task.json
python tools/run_m710_moveit_integration.py --suite capability --output capability.json
python tools/run_m710_moveit_integration.py --case empty_short --output ptp.json
PYTHONPATH=src python -m pytest tests/test_moveit2_tcp.py tests/test_moveit2_backend.py -q
```

完整计划通过后才允许准备原 POC preflight、导出 bundle、运行 `check_m710_moveit_execution_tcp.py`，再考虑一次监督 Isaac。没有新建控制器、blend、CIRC、在线框架或完整 MTC 候选搜索。

独立执行前检查复现（仅对完整 PASS 计划）：

```bash
python tools/prepare_m710id70_dynamic_execution.py --config configs/simulation/m710id70_proof_of_concept.yaml --motion-result full-task.json --output preflight.json
PYTHONPATH=src python scripts/export_isaac_fanuc_replay.py --preflight preflight.json --output replay-bundle.json
python tools/check_m710_moveit_execution_tcp.py --bundle replay-bundle.json --requests task-requests.jsonl --output executor-tcp.json
```

原始 `final-validation.sh` 的冗余 suffix 断言曾误指开发期 `final/suffix.json`，该原始脚本作为执行记录保留不改；最终 `final2/suffix.json` 实际也通过并独立核对，完整任务保留了自身的全部验收门。上面的复现命令明确断言本次输出，不能直接复用旧文件作为门控。

完整规划前设置 `M710_MOVEIT_REQUEST_LOG=task-requests.jsonl`、`M710_AUTHORITY_TRACE=authority-paths.jsonl` 可保留逐调用输入和路径计时。物理启动命令及不可重复启动的门控保存在 `isaac-once.sh`；它使用现有监督执行脚本，`--maximum-segments 1`，不设置 continuation，不调用 task.execute()。
