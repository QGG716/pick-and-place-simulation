# Tesseract + OMPL v0.4：固定 LazyPRM 的真实单候选任务

本轮唯一一次 `LayoutTrajectoryConnector.plan` 已执行。首个真实空载 pregrasp 请求使用了指定的非 star LazyPRM，但耗尽累计采样预算，未交付自由段或完整任务。按约定立即停止，没有第二次任务，没有启动 Isaac。有限预算失败不表示物理不可达。

## 1. 实际单候选入口是否用了 LazyPRM

**是。** 修复 `tools/run_tesseract_ompl_task.py` 的正式参数链：

`--ompl-planner lazy_prm` → `OMPLPlannerConfig` → `TesseractOMPLBackend` → worker 请求 → `planner_config` 确认及 `effective_planner` 校验。

真实返回为 `ompl::geometric::LazyPRM`、`star=false`、默认 `KBoundedStrategy`、k=5，setup 计算连接距离 **4.431260361343388 rad（关节空间 L2）**，空间最大尺度 22.15630180671694。目标为默认 PathLengthOptimizationObjective，cost threshold 为正无穷，首条完整原生有效路径即停止。本轮没有改 worker、换规划器、调参、平滑、shortcut 或注入路网。

普通单候选入口仍默认 `rrt_connect`、两次修复上限；普通主 CLI 仍默认 legacy。本轮明确传入 `--native-max-attempts 1 --stop-on-native-block`。未知 CLI 参数拒绝；LazyPRM 配置不被确认、回显不同 planner、star 模式或不同 k 时不进入 authority，也不 fallback。

详见 [实际请求](validation/evidence/backend_tesseract_ompl_v0_4/task-result-native/native-01.request.json)、[原生结果](validation/evidence/backend_tesseract_ompl_v0_4/task-result-native/native-01.result.json)、[实际命令](validation/evidence/backend_tesseract_ompl_v0_4/run-task.sh)。

## 2. 哪些真实自由段通过，哪些未通过

| 阶段 | 本轮状态 |
| --- | --- |
| home → 抓前自由接近（pregrasp） | 已调用 native；`BUDGET_EXHAUSTED / CUMULATIVE_SAMPLE_LIMIT`，无可交付路径 |
| contact / ATTACH / 支撑解除 / extraction | 未完成接近，未进入这些工艺阶段 |
| 附着后的 transit | 未调用 |
| place / RELEASE / reception / withdrawal | 未进入 |

两个 native 端点通过初始状态检查；直连边被拒绝。最早网格碰撞见原生结果 `first_failed_sample`：边 0、位置 9070/18141（约中点），`carton_l06_c02` 与 `tool_rigid_104`，有符号距离 -0.08558153294285857 m，要求 0.005 m；完整 q、TCP、条件数和裕量均保留。这是搜索中一个碰撞见证，**不是最终资源终止原因，也不是本轮已做 authority 同点确认的证据**。

共检查 19 条边：1 有效、18 无效、0 未完成。那条有效内部边不是一条可交付阶段路径。未返回 exact 或 approximate 候选，没有启动 authority 路径复检。

自由段仍仅由生产连接器 `_transit` 的 pregrasp/transit 分派进入 native。contact、支撑解除、抽离、放置、释放及撤离代码和判据均未修改。附着场景导出仍要求移除世界中的同名目标，并保留工具、payload 和全部环境碰撞对象；真实小型附着回归通过，但**本任务尚未到达附着阶段**，不把该回归写成持物任务成功。

## 3. 是否生成完整几何轨迹

**没有。** `selected_trajectory_segment=null`、`complete_geometry_status=NOT_AVAILABLE`。

使用原 `fixture_context`、37 箱冻结快照、目标 `carton_l07_c04`、front 面、原固定接触候选和 `historical_segment.path[0]` 的 home。历史文件只提供这些任务输入；未使用历史完整路径，也未接入 v0.3 的 78→208 路径。起终点明确为：

```text
q_start = [-0.5284209847450256, 0.09451229125261307, -0.21063973009586334,
           -3.1416585445404053, 1.2654187679290771, 0.5286964774131775]
q_goal  = [0.5000137206581776, -0.18736779957354116, -0.15311780513086534,
           -1.6334118791635104, 0.5008636650157714, -1.4994301903402696]
```

任务 seed 仍为 71070；生产 `_approach`/`_connect_pose` 固定偏移产生首个 native seed **71081 = 71070 + 10 + 1**，没有换 seed 重试。请求 attachment=null、events=[]。模型、TCP、策略、剩余箱体名单、输入 SHA、有限工艺预算在 [任务身份](validation/evidence/backend_tesseract_ompl_v0_4/task-result.identity.json) 中，实际完整 URDF/SRDF、网格文件 SHA 和成对规则在请求中。

返回路径点数为 0；路径长度、各关节总变化、执行时长均 **null / 不适用**，不能用空路径的零长度表示运动质量。没有通过补写 success、事件或接触证据生成轨迹包。

## 4. 时间参数化和执行 preflight

两者均 **NOT_RUN**，原因是没有完整几何轨迹。`simulation_execution_ready=false`。没有可供执行的稀疏点序列，也没有跳过时间参数化直接发给控制器。

本轮运行了生产阶段顺序/事件、阶段关节衔接、完整轨迹导出、preflight 防篡改及时间参数化的定向合同回归。它们证明相关合同没有被此改动绕过，**不构成本任务的 preflight PASS**。完整几何、时间参数化、仿真就绪、实际物理执行与机器资格继续分别记录；机器资格未建立。

## 5. Isaac 是否启动，实际完成到哪个事件

**未启动。** 没有新 Isaac 世界、物理执行、ATTACH/RELEASE 事件、物理状态日志或视频。新世界完成箱数为 0，历史快照此前移除的箱数不计入本轮。环境复用没有涉及依赖升级、重装或物理参数变更。

## 6. 首个阻塞点

首个真实 native pregrasp 请求在 **累计采样 10000** 时停止：

| 指标 | 实测 / 上限 |
| --- | --- |
| 状态检查请求 | 8753 |
| 精确状态缓存命中 | 33 |
| 实际状态计算（缓存未命中） | 8720 / 100000 |
| FCL 查询 | 8710 |
| 累计采样器调用 | 10000 / 10000 |
| OMPL LazyPRM 迭代 | 9900 |
| 终止时路网节点 | 9619 / 10002；48 已知有效，9571 UNKNOWN |
| 终止时无向边 | 47446 / 50010；1 已知有效，47445 UNKNOWN |
| 必需网格样本请求 | 8420 |
| approximate 目标残差 | null，未取得 approximate 解 |

10000 次采样包含 setup 投影估计的 100 次采样和 9900 次 LazyPRM 迭代，不能把它们算成 10000 次 FCL 检查。边资源还有每次增长预留保护，本轮实际首先触发的是采样限制。OMPL 文本 `Timeout` 来自终止条件；本请求 `wall_time_s=null`（worker 为 0），**不是新增短墙钟截止时间**。

实际计算拒绝分类：碰撞 290、关节裕量 3、径向限制 4、奇异性 3。包括缓存的请求级碰撞拒绝为 291；两种口径分开。预算终止没有记为碰撞或无效边。

任务调用 **1 次**，任务 native 请求 **1 次**，修复/探针 **0 次**，authority 路径复检 **0 次**，物理执行 **0 次**。显式实验选项在追加阶段证据后抛出专用 `NativePlanningBlocked`，逃出外层接近距离/IK 分支调度，避免预算失败后继续展开三次 seed × 多个距离。默认调度不变。中断发生前尚未封装外层 branch trace，因此 `attempts=[]`；实际请求与失败在 `free_motion_records[0]` 和独立原生文件中，并不表示没有尝试。

本輪辅助小型回归另有 6 次 configure 操作、3 次附着端点检查，均不属于本轮工程任务调用，没有额外工程绕障搜索。原始任务退出码为 1，见 [日志](validation/evidence/backend_tesseract_ompl_v0_4/task.log) 与 [任务结果](validation/evidence/backend_tesseract_ompl_v0_4/task-result.json)。

## 7. 下一步应修什么

本轮修正了确定的**入口配置接线**和**实验停止传播**问题。当前明确阻塞是首个自由段的采样资源用尽；下一步需要诊断该固定请求的搜索进展/有效连接瓶颈，而非先做执行适配或物理试验。本轮没有证据表明应修改模型、放宽约束或改变工艺输入，也不能从一次失败推出所有候选不可行。

FCL 仍是实际运行时间的主要部分，但更快碰撞检查本身不证明能在相同 10000 次采样内得到路径；这里同时存在“检查成本”和“有限采样内未找到完整有效连接”两件事。本轮没有继续性能重构或调大任何预算。持物自由段、全部工艺交接、完整任务时间参数化、执行适配与物理表现均仍无本任务证据。

## 分项耗时及口径

| 时钟 | 秒 |
| --- | ---: |
| fixture 初始化、轻量身份导出/记录 | 0.556960 |
| 唯一 connector.plan 调用（含下列搜索工作） | 12.865175 |
| 连接器 IK/工艺候选准备计时（含端点检查） | 0.014946 |
| 实际自由段场景导出 | 0.087345 |
| adapter 输入校验 | 0.044186 |
| worker 冷启动 | 0.040764 |
| worker 往返（不含冷启动） | 12.621601 |
| native 总计 | 12.515146 |
| 其中环境初始化 | 0.188111 |
| 其中端点检查 / 直连检查 | 0.009824 / 0.002367 |
| 其中 OMPL solve | 12.277921 |
| authority / 完整轨迹检查 / 时间参数化 / preflight / Isaac 初始化及执行 | 未运行，null |

原生聚合 profile：

| 分项 | 秒 |
| --- | ---: |
| FK / 场景状态 | 0.456389 |
| Jacobian / SVD | 0.043565 |
| 碰撞对象变换 | 1.083602 |
| FCL contactTest | 10.319205 |
| 预算/取消轮询 | 0.024438 |
| 边调度自身 | 0.003240 |
| 状态检查 inclusive | 12.091000 |
| 边检查 inclusive | 11.697055 |

FCL 约占 native 总时间 82.5%。native 总时间包含环境初始化和 solve 等；solve 包含状态/边检查；状态 inclusive 包含 FK/SVD/变换/FCL 等；边 inclusive 又包含边内状态检查，两者重叠，**不可重复相加**。边调度是扣除边内状态检查后的计时。connector 的碰撞计时也嵌套在 IK/候选准备中，非额外阶段。`profile=true` 使用现有聚合时钟；未另跑无插桩请求估计开销，不能将上述数字称为无插桩性能或与 v0.3 不同起终点做加速比较。

## 基线、源码和测试身份

本地、远端和审阅基线均为 `e7c73a8c40ad07d964339560ff455aa29e59b531`，开始时工作树干净。服务器是既有源码副本而非 Git checkout；变更前 adapter/tool 文件与该提交字节 SHA 一致，已备份。只改本分支，未 reset/force push 或改动其他分支；v0.1/v0.2/v0.3 原证据保持不变。

复用原 worker：SHA256 **`12e2e94a5ea530d389e5d097fca4bd0daa117c30016b00049827e1b33860e288`**；C++ 源码 SHA256 `c7510871593bf7646ea6092de278c30037094737b5cff29a9249f8db54dafcf6`。Tesseract 0.35.0 / OMPL 1.7.0 / FCL 0.7.0。没有重建后端、重新安装环境或重新执行几百秒旧路径审计；既有 checker 身份回归读取先前证明。

唯一工程任务执行 adapter SHA 是 `9fd4885b509a092d0339e8210eac8e5f6841c265e58af4bd27790dfc2b61e3a5`，见 [实际执行源码归档](validation/evidence/backend_tesseract_ompl_v0_4/task-executed-sources.tar.gz)。任务结束后仅增加了 `effective_planner` 必须为 dict 的防御检查；最终 adapter SHA 为 `62f1958082dc6478126aaaecce183788c43a33781c70db37996dc768ab71761f`。真实任务返回的是正常 dict，这条防御检查不改变该结果；**没有为最终修正重跑任务，原任务身份也没有改写成最终源码身份**。工具源码和 worker 全程相同，最终源码已另跑合同回归。

定向测试（服务器原有 CPU 环境）：

| 组 | 结果 | 证据范围 |
| --- | --- | --- |
| backend 合同 + 新单候选工具 | 66 passed / 0.33 s | fake transport、默认/显式配置、错误 planner、停止传播、取消/过期、approximate 不交付等；非物理证据 |
| 阶段/导出/preflight/时间参数化合同 | 12 passed / 0.24 s | 合成数据合同回归，非本任务合格轨迹 |
| 真实 worker 配置确认 | 1 passed / 0.14 s | configure 操作确认默认 RRT、显式 LazyPRM 及未知配置拒绝 |
| 真实 FANUC 附着端点检查 | 1 passed / 1.59 s | 附着改变查询、重复世界 payload 拒绝；非持物完整段 |

最终合计 **80 项通过**；较早 64 项测试日志也保留，但不重复计入总数。精确命令见 [定向测试脚本](validation/evidence/backend_tesseract_ompl_v0_4/run-directed-tests.sh)，实际日志与源码 SHA 在同目录。没有完整测试集、104/129 任务集、整排或 40 箱试验。

证据入口：[summary.json](validation/evidence/backend_tesseract_ompl_v0_4/summary.json)、[baseline.json](validation/evidence/backend_tesseract_ompl_v0_4/baseline.json)。原始请求、结果、日志、输入/实现/模型身份、真实失败及实际执行源码一并保留；本轮没有可提供的完整执行包或视频。
