# Tesseract + OMPL v0.6：单候选配置和交付出口接线

本轮没有生成完整任务轨迹，没有启动 Isaac。唯一一次真实
`LayoutTrajectoryConnector.plan` 在新增的首请求身份门禁处误拦截：Python tuple
与历史 JSON list 被直接比较，实际关节数值完全相同。问题已修复并通过定向回归，
但遵守一次任务调用上限，没有进行第二次工程验证。不能将最终代码的单元测试
写成本轮工程路径成功。

## 本轮结果

| 问题 | 实际结果 |
|---|---|
| 是否使用 endpoint_mixture | 正式入口配置和准备好的请求均为冻结配置；**未到达 native，实际 sampler 未确认** |
| 正式任务首段是否通过 | 否；home→pregrasp 在 host 身份门禁停止，native 搜索调用为 0 |
| 接触、附着、支撑解除、抽离、持物 transit | 工艺路径未到达；不能借用 v0.5 空载结果 |
| place、release、reception、withdrawal | 未到达，没有物理事件 |
| 完整几何轨迹 | NOT_AVAILABLE，没有完整轨迹包 |
| 时间参数化、实际执行轨迹验收 | NOT_RUN |
| execution preflight | NOT_RUN，simulation_execution_ready=false |
| Isaac / 视频 | 未启动 / 未生成 |
| 首个真实任务阻塞 | 本轮新增身份门禁的容器类型比较错误，已修复；不是碰撞、采样预算或不可达 |

原始记录在 [证据目录](validation/evidence/backend_tesseract_ompl_v0_6/)，
准确计数及原始字段解释见 [summary.json](validation/evidence/backend_tesseract_ompl_v0_6/summary.json)。

## 基线、输入和执行次数

开始时本地和远端 HEAD 均为 `e16a759d857d55003ed50b7627a1fa012274c19a`，
工作树干净。仅修改本实验分支，没有 reset、force push 或合并其他分支。
所有检查使用原服务器 CPU 环境，无安装、升级或 worker 重建。

固定历史 state/segment、37 箱场景、`carton_l07_c04`、front 面、home、接触候选、
支撑关系和接收区。任务 seed=71070，生产流程派生首请求 seed=71081。
没有注入历史路径、路网、缓存解，没有重新求另一组抓取姿态。

冻结配置完整保存为 [planner-config.json](validation/evidence/backend_tesseract_ompl_v0_6/planner-config.json)：
非 star LazyPRM、k=5，endpoint_mixture 的 local_probability=0.5、
joint_span_half_width_fraction=0.05。每个 native 请求 max_state_checks=100000、
max_samples=10000、max_roadmap_vertices=10002、max_roadmap_edges=50010、
max_attempts=1，stop_on_native_block=true，没有业务墙钟截止时间。

有两次 CLI 启动，但只有 **1 次 connector.plan，0 次 native transport/search**：

1. `engineering/` 是初始化失败。启动脚本多加的 `LD_LIBRARY_PATH=native/lib...`
   使 CPU Pinocchio 误加载 native 的 assimp，出现 `CXXABI_1.3.15 not found`。
   当时尚未创建连接器。只读导入检查确认原环境 Pinocchio 4.1.0 正常，移除临时覆盖，
   未安装依赖。原始日志和启动命令保留。
2. `task/` 是唯一真实 connector.plan。完成请求准备后，门禁把相同 q 的 tuple/list
   判为不同，尚未调用 `NativeWorker.call` 就停止。后续没有任务、候选、seed 或预算重试。

原始 `task-result.json` 的 `native_call_count=1` 是旧 RecordingWorker 包装器进入次数，
**不是 native 调用数**；原始 `native-01.result.json` 是 host 门禁产物，**不是二进制输出**。
原始 `BACKEND_UNAVAILABLE` 是 RuntimeError 通用捕获造成的分类，不能据此认为 worker
或依赖不可用。这些原文件均未更改，解释在 postmortem/summary 中追加。

## 具体修改和门禁修复

- `--planner-config` 严格 JSON → 现有 `OMPLPlannerConfig` / `EndpointSamplingConfig`
  → backend → 请求。拒绝未知字段、重复 JSON key、非法参数及 CLI/JSON 冲突；
  不指定时仍为原 RRTConnect，普通预算/调度默认不变。
- 保留 worker 的 planner_config、effective_planner、effective_sampler 确认要求。
  未确认不进入 authority，不把旧 worker 或其他策略结果当成功。
- 首请求比较 q_start/q_goal、seed、完整 scene 和 planner_config。完整 scene 包含模型、
  工具、TCP、附着、阶段、几何、策略、细分和路径资产身份。
  最终实现按 JSON 传输语义比较，未修改关节值或指纹；真实差异仍在发送前拒绝。
- 显式 `NativePlanningBlocked` 向外传播，避免被通用异常包装成依赖不可用。
  分开记录准备请求数与进入 transport 的调用数。
- 可选只读阶段回调即时写出 free-motion 原始结果、authority 结果、已验证路径和工艺阶段。
  支撑解除、抽离、附着变换、放置释放合同、撤离使用原生产检查；记录区分规划几何与
  实际附着/释放事件。不改变 candidate 排序、碰撞判据或已有自由段选择。

[离线 postmortem](validation/evidence/backend_tesseract_ompl_v0_6/postmortem.json)
比较的是两份已经序列化的请求，没有运行 native、authority、IK 或 connector：
起终点、seed、完整 scene、planner_config **全部相等**。不存在已证实的模型/策略差异。
最终补上了生产 backend → RecordingWorker 的 tuple/list 回归，而不是只测手写 list。

## 成功出口和执行轨迹合同

新增 `tesseract_task_delivery.py`，在工程调用前已经接线和测试。沿用生产依赖顺序：

1. `validate_trajectory_segment`、`validate_layout_trajectory_stage_contract` 和当前连接器
   `_task_completed` 检查完整几何、阶段事件以及本请求的完成证明；障碍物身份顺序与生产
   finalizer 相同。不能由 outcome.success 推导其他 PASS。
2. 将已通过的 segment 薄封装为生产 motion result，明确合法目标集合与实际调用次数的区别。
   不编造资产资格、接触记录或事件。调用 `build_m710_execution_preflight`，显式传入
   motion_result 和 motion_input，避免其默认入口重新规划。
3. 使用 preflight 的绑定输入调用 `build_fanuc_isaac_replay_bundle`。该生产函数实际调用
   `time_parameterize_joint_path`，进行解析速度/加速度/jerk 审计并保留工艺停留。
4. 使用真实执行入口的 `replay_command_arrays` 和 `sample_joint_reference` 验收控制参考。
   再核对有序折线、生产共线删点、被保护边界、ATTACH/RELEASE/RETREAT 的 q 和路径累计位置。
   仅允许同一路径上的定时与静止停留；曲线或角度 wrap 改路会拒绝。
5. 对落盘包调用 `verify_m710_replay_bundle(project_root=...)` 核查实现、资产及绑定内容。
   全部通过才发布本工具的 simulation_execution_ready。每个失败保留 FAIL；下游未运行
   保持 NOT_RUN。不会在这层启动另一套执行器。

生产 bridge 要求先有 verified preflight，故不能机械地将首次 preflight 移到 timing 后；
导出后还有实际控制参考验收与 workspace/preflight/bundle readback。

大于 π 的 3.2545 rad 关节变化用定向合成 fixture 测试：原角度折线完整保留，
不走周期最短角；控制流是五次插值，稀疏点数不等于控制步数。新增了“事件处 q 相同，
但移到路径较早一次经过”的拒绝测试。现有模型/碰撞检查没有改变，未重跑旧路径审计。

**这些是接线测试，不是本次真实任务的定时、preflight 或物理执行证据。**
本轮没有完整路径可供验收，也没有执行包或视频位置可交付。

## 定向测试

- 工程调用前：127 passed、5 skipped，见 `tests-directed.log`。
- 门禁和异常分类修复后：**129 passed、5 skipped**，见 `tests-corrected.log`。
- 覆盖配置传递/拒绝、默认行为、实际 backend 请求的容器类型、真实身份差异停止、
  阶段交接、原生阻塞传播、取消/过期候选门禁、完整完成证明、事件/关节路径保持、
  生产 timing/export/控制参考/bundle 合同及逐门失败传播。
- 5 个 skipped 为未启用 worker 环境变量的真实 native 单元用例，不能计为真实 native PASS。
  成功出口 fixture 的场景/preflight 构造使用测试替身；生产 export、定时和合同验证函数
  实际运行，不能将 fixture 的 READY 当成本工程 READY。
- 一次较宽的直接相关回归得到 133 passed、5 skipped、1 failed，保留于 `tests-final.log`。
  失败是既有 `test_reuse_tool_rejects_changed_non_target_obstacle_and_current_release_policy`：
  其 SimpleNamespace budget 缺少现有 `release_policy()`。测试、history_adaptation 和
  reuse 工具均未修改；没有为本轮顺手修历史复用逻辑。最终清单仅选择相关定时/控制参考用例。

## 耗时、预算和身份

| 实测项 | 秒 | 范围 |
|---|---:|---|
| fixture 初始化及身份写出 | 0.586263 | 独立于后续 task elapsed |
| 单候选 task elapsed | 0.228089 | 含下面的连接、准备和检查，不应重复相加 |
| path connection inclusive | 0.201192 | 含场景导出、backend 与记录开销 |
| scene export | 0.092202 | 嵌套于连接 |
| backend request total | 0.106450 | 嵌套于连接；含 host 门禁，不是 native solve |
| request input validation | 0.046149 | 嵌套于 backend total |
| trajectory IK/preparation 计时字段 | 0.014736 | 生产统计字段；IK solver 调用数为 0 |
| collision validation nested | 0.011708 | 1 次状态检查，不是路径 authority 审计 |
| native / authority 路径复检 / 完整检查 / timing / preflight / physics | 未运行 | 耗时为 null，不写性能 PASS |

没有实际 native 状态计算、采样或边检查。原始 outer legacy 配额扣记 600 只是调度分配，
不能写成 OMPL 迭代数或消耗预算。不存在本轮搜索性能结论。

复用 v0.5 worker SHA256：
`742607df1eca48881f48f463c9bf021be5144183e2959addcdca70d1a572d53f`。
`worker.cpp`、`endpoint_sampler.h`、`roadmap_diagnostics.h` 与 v0.5 编译身份逐项一致；
没有将 v0.5 成功写成这轮新结果。`final-code-identity.json` 记录全部最终源码 hash、
编译输入身份和原构建证据 hash。

`task/task-result.identity.json` 和 `executed-source.tar.gz` 对应**失败时版本**，
`final-code-identity.json` 对应**修复后、仅经定向测试的版本**，二者明确分开。
`authority-before.py.gz` 为审阅提交的原始 authority 源码。身份回归去除明确列举的只读
记录回调后比较整个模块 AST；native Context/DenseMotion 和细分合同保持原内容身份。
新增回归确认真正修改状态检查会使该比较失败。

## 尚缺证据与下一步

当前可交付的是正式配置、记录和成功出口接线，以及真实 host 阻塞及其修复。
尚不能证明最终入口在工程任务里得到 native 首段结果，更不能证明工艺交接、持物 transit、
完整几何、执行包或物理执行通过。

下一步应在新的任务授权轮次使用已修复入口验证单候选流程；当前证据不支持再改采样参数、
增预算或换规划器。本轮严格没有第二次 connector.plan 或 Isaac。一次工程结果即使以后成功，
也不能推导为在线能力、普遍成功率或机器资格。
