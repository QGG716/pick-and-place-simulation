# Tesseract + OMPL v0.7：真实单候选任务

本轮真正运行了一次单候选抓放任务，首个 home→pregrasp 得到 exact 原生路径并通过 authority。任务随后在附着 transit 的第二个原生请求耗尽实际状态计算预算，按 stop_on_native_block 停止。**没有完整轨迹、执行包或视频，没有启动 Isaac。** 没有重启任务、调参或追加试验。

| 项目 | 本轮实际状态 |
|---|---|
| 轻量启动检查 | PASS；真实握手/configure 1 次，搜索 0 次 |
| 正式完整任务调用 | 1 次，确实进入 native；此前本轮启动失败 0 次 |
| 正式原生请求 / 搜索 | 2 / 2；另有直连检查 2 次，不能另计为任务 |
| home→pregrasp | native exact + authority PASS，9 点 |
| 接近、接触、附着几何、支撑解除、抽离 | 已有通过记录；均为规划阶段，非物理事件 |
| 持物 transit | 一个 131 点笛卡尔工艺连接 PASS；后续自由连接 native-02 BUDGET_EXHAUSTED |
| place / release / reception / withdrawal | 无通过证据；中间 place 分支失败详情未持久化，见局限 |
| 完整几何 | NOT_AVAILABLE；没有完整任务完成证明 |
| 时间参数化 / 实际控制参考验收 / execution preflight | 全部 NOT_RUN |
| simulation_execution_ready | false |
| Isaac / 物理事件 / 机器资格 | NOT_RUN / 无 / false |
| 首个导致停止的阻塞 | `native-02`，transit，seed=71140，ACTUAL_STATE_COMPUTATION_LIMIT |

原始结果见 [task-result.json](validation/evidence/backend_tesseract_ompl_v0_7/task-01/task-result.json)，聚合计数见 [summary.json](validation/evidence/backend_tesseract_ompl_v0_7/summary.json)。所有历史 v0.1–v0.6 证据保留。

## 输入、配置与身份

开始时本地和远端 HEAD 均为 `b4c840003ac52f61d3a059d235e012cde439f320`，工作树干净。只在 `feat/v0.5-backend-tesseract-ompl` 开发，没有 reset、合并、force push 或修改历史证据。

固定 historical_state / historical_segment、37 箱场景、`carton_l07_c04`、front 面、原 home、接触候选、支撑与接收配置。任务 seed=71070；首请求由现有规则派生为 71081。首请求的完整 scene、关节端点、seed 和 planner_config 与 v0.5 请求按 JSON 传输语义精确相等；参考文件只提供请求身份，没有读取历史结果路径、节点或路网。

正式 `--planner-config` 配置为非 star LazyPRM，k=5，endpoint_mixture 的 local_probability=0.5、joint_span_half_width_fraction=0.05。每请求 max_state_checks=100000、max_samples=10000、max_roadmap_vertices=10002、max_roadmap_edges=50010；max_attempts=1，stop_on_native_block=true。没有新增业务墙钟截止、修改 DenseMotion、调整工艺/净空或扩大候选集合。10 万次是每请求上限，不是整任务上限。

复用 CPU Python 3.12.3 / Pinocchio 4.1.0 环境及已有 worker；父进程未设置 LD_LIBRARY_PATH，worker 使用现有 RPATH。没有安装、升级依赖或重建未改动的二进制。worker SHA256：`742607df1eca48881f48f463c9bf021be5144183e2959addcdca70d1a572d53f`。源码和编译头文件校对见 [source-identity.json](validation/evidence/backend_tesseract_ompl_v0_7/source-identity.json)，其编译身份仍属于 v0.5 的原始构建，并非本轮新编译。

## 轻量启动检查与本轮修改

新增 `tools/check_tesseract_ompl_task_startup.py`，复用正式参数解析、场景导出、RecordingWorker 和现有身份门禁。它检查 CPU 依赖导入、显式 execution_config 对应的场景/策略、20 kg 工具和 42.5 kg 箱体配置，并执行一次真实握手/configure。实际返回 CONFIGURED，确认 LazyPRM、k=5 和冻结 sampler；连接距离由 setup 得到 4.431260361343388 rad。启动检查用时 1.179 s，没有状态检查、工程搜索或路径验收。配置兼容性不是 execution preflight。

`RecordingWorker` 增加小型聚合 activity 记录，区分准备请求、通信尝试、返回结果、configure、状态计算、直连运动检查和 OMPL 搜索。通信计数在调用前增加，只能证明通信尝试；真实计算与搜索计数来自返回的 worker 证据。未返回的通信不推断为零工作。异常单独落盘为 host transport exception，不能冒充 native 结果。沿用已有阶段回调即时保存通过与失败记录。

`src/unloading_sim`、native 源码及既有成功交付模块均未改动；普通入口默认配置不变。没有新增缓存、恢复、调度、工艺算法或执行器。

## 首段真实原生结果与耗时

首段实际返回 9 点 exact 路径，native_validated=true，随后 authority 通过。状态请求 97,539 次，其中精确缓存命中 45 次，实际计算 97,494 次；累计采样 1,389 次包含 setup 100 次和建图 1,289 次。建图采样全域 644、起点邻域 314、终点邻域 331。边检查 39 次，其中有效 18、无效 21、未完成 0。上游迭代 1,289；终止路网 751 节点、2,835 边，没有复用上一请求路网。

| 时钟 | 秒 | 包含关系 |
|---|---:|---|
| native total | 462.867 | 包括环境、端点、直连与求解 |
| OMPL solve | 462.667 | native total 内 |
| FCL contactTest | 441.246 | 状态检查内 |
| 对象变换更新 | 11.770 | 状态检查内 |
| FK / 场景状态 | 6.515 | 状态检查内 |
| Jacobian / SVD | 0.556 | 状态检查内 |
| 预算/取消轮询 | 0.280 | 多层调用内 |
| 边调度 | 0.044 | 边检查内，不包括其状态计算 |
| worker roundtrip | 462.973 | 包含 native total |
| authority | 637.325 | native 返回后独立复检 |
| backend request total | 1100.476 | 包含 roundtrip、authority、验证与转换 |

状态检查 inclusive=462.604 s、边检查 inclusive=461.876 s，二者互相嵌套，不能与上表重复相加。启用了既有 profile，未测量其插桩开销，因此不能将此表称为无插桩性能。主要原生耗时仍在 FCL；authority 本次耗时大于 native。本轮只记录，不进行性能优化。

## 工艺阶段证据的含义

阶段 001/002 记录首段自由连接通过和 9 点路径；003/004 记录包括接近末端的 12 点接触路径。005 为当前任务的刚体附着几何，世界障碍物中移除了同名目标；physical_attachment=NOT_RUN。006 为现有 POC 支撑解除判据（允许抽离中的支撑滑动/抬升），没有把它解释为实际物理释放。007 为首个直线抽离方案通过，距离 0.6077140408754352 m、42 个关节点，没有重新挑选抓取候选。

008 记录一个现有笛卡尔 transit 通过。其直接连接首先因 payload/邻箱间隙 0.0049944919791913844 m 小于既定 0.005 m 被拒绝，随后既有算法做 0.12 m 向外缓冲并重新完整检查。共享 240 样本预算剩余 104，没有放宽间隙。该工艺路径不是 LazyPRM 返回路径，局部通过也不是放置/完整任务完成证明。

后续原生 transit 请求 seed=71140，从本任务抽离末点出发，使用自己的目标关节和端点采样中心。附着变换来自本任务，不能复用空载 pregrasp 场景或首段端点。

该请求实际使用相同 LazyPRM / sampler，独立空路网，附着目标不再重复保留于世界障碍物中，场景身份与空载请求不同。它最终返回 **BUDGET_EXHAUSTED / ACTUAL_STATE_COMPUTATION_LIMIT**，exact=false、native_validated=false、path=[]。没有候选可送 authority，不是 authority 拒绝，也不是 approximate 被交付。

第二请求状态请求 100,056、精确缓存命中 55、实际计算 100,000；剩余一次调用为预算中断。累计采样 5,240，其中 setup 100、建图 5,140（全域 2,597、起点邻域 1,266、终点邻域 1,277）。终止路网 3,529 节点、14,780 边，明确 VALID 为 186 节点、22 边。46 次边检查包含有效 22、无效 23、未完成 1；中断边没有记为碰撞或已验证通过。实际计算拒绝分类为碰撞 1,578、关节裕量 21、径向限制 35、奇异性 1。缓存命中的拒绝与实际计算拒绝单列于原始结果。

`first_failed_sample` 是直连中点的 payload/箱体碰撞，仅能解释该直连失败；最终终止原因是预算。UNKNOWN 路网边多不能单独证明实现错误或场景不可达。本轮没有诊断配置扫描或进一步搜索。

## 整轮时间与交付边界

正式 connector 调用 2676.423 s（约 44.61 分钟），前置场景与身份准备 0.580 s。两请求实际状态计算累计 **197,494**，不是整任务仅使用 10 万次；configure 没有消耗这部分预算。authority 实际运行 1 次、通过 1 次；第二请求没有 authority 调用。

| 时钟 | pregrasp / 秒 | transit native-02 / 秒 |
|---|---:|---:|
| 场景导出 | 0.0915 | 0.0971 |
| native total | 462.867 | 353.837 |
| OMPL solve（native 内） | 462.667 | 353.600 |
| FCL contactTest（native 内） | 441.246 | 330.735 |
| FK / 场景状态（native 内） | 6.515 | 6.477 |
| Jacobian / SVD（native 内） | 0.556 | 0.585 |
| 碰撞变换（native 内） | 11.770 | 12.746 |
| 预算/取消轮询（native 内） | 0.280 | 0.277 |
| 边调度（native 内） | 0.044 | 0.047 |
| authority | 637.325 | NOT_RUN |
| backend request total | 1100.476 | 354.035 |

生产统计另记录：IK 182 次/558 迭代、trajectory_ik_wall_seconds=0.0312；既有接收位置生成 12 个，用时 0.0349 s；笛卡尔工艺采样 181。这里的 12 是当前目标的生产接收位置，不是 12 个抓取候选或 12 次 native 搜索。path_connection_wall_seconds_inclusive=1454.710 s 包含原生与 authority；collision_validation_wall_seconds_nested=1842.807 s 横跨 authority 和工艺检查，不能与连接时钟相加。运动学/关节检查聚合为 28.338 s，完整任务 final_recheck 为 0（未到达）。现有插桩没有将全部工艺准备时间切成互斥分项，因此不能用总耗时减法虚构某个阶段的精确耗时。

完整轨迹检查、生产时间参数化、控制参考验收、preflight、bundle 读回均未到达，不能用数值 0 或单测 PASS 冒充实际执行成功。没有执行时长、完整任务关节长度或物理性能结论。局部路径的点数、L2 长度和各关节总变化保存在 summary，不能把它们拼成未验收的完整路径。Isaac 环境只做文件可用性检查，没有导入/启动物理应用。

## 尚缺证据与下一步

当前阻塞已从 home→pregrasp 移到附着 transit：先定位该请求的检查成本与连接障碍，不能继续宣称完整任务/执行适配已通过。没有证据表明模型或策略不一致，也没有物理不可达证明；本轮不调整算法、预算或执行链。

有一项明确的证据局限：既有工艺分支曾返回一个通过的笛卡尔 transit，然后进入下一放置高度的自由连接；异常停止时，外层局部 `release_attempts` 尚未汇入返回结果，所以最终 attempts=[]。从生产控制流和不同目标关节可判断有后续分支，但中间 place 的具体失败原因没有即时持久化，不能补写猜测，更不能把 attempts=[] 解读为未进行工艺试探。后续宜先补齐这个失败回调，再研究冻结的附着请求；本轮没有为补证据重跑任务。

成功出口仍复用 `tesseract_task_delivery.py`，本轮源码未变。只有未来真实完整任务通过几何→绑定 preflight→定时/导出→实际控制参考→bundle 读回后，才能开展新的物理执行验证。当前没有执行包/视频位置可以提供，机器认证也未进行。

## 定向回归

审阅基线父版本 `e16a759d857d55003ed50b7627a1fa012274c19a` 与审阅基线分别复现指定历史复用单测，均因 SimpleNamespace budget 缺少 release_policy() 失败。仅将测试 fixture 改成实际 LayoutTrajectoryBudget，并使用 dataclasses.replace 修改冻结参数；生产历史复用代码未变。

修正后包含该文件全部用例的定向回归为 **142 passed、5 skipped**。覆盖生产 backend 生成的 tuple 与 JSON list 身份相等、真实 q/seed/模型/策略/sampler 差异在发送前拒绝、通信与真实工作计数、配置传递、失败传播、成功交付、控制参考、定时及执行合同。5 项 skipped 为未开启 worker 环境变量的真实 native 单元用例，不能写成通过。真实 configure 和本轮工程 native 另有原始证据；mock 合同结果不能替代它们。

命令与日志见 [证据目录](validation/evidence/backend_tesseract_ompl_v0_7/)。未重复旧路径审计、全量测试或完整历史任务集。
