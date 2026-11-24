# 任务：ECO65 六段循环规划从约 12 s 降到 2 s 以内

## 目标和口径

分支 `feat/v0.6-eco65-ubuntu-real-integration`，HEAD `e918a2d`。六段循环（`cycle_structure.mode=START_RELEASE_SEPARATE`：approach、contact、detach、transfer、withdraw、return_empty）的离线回放规划现在约 12 s，见 `docs/eco65_real_site.md` 末尾"2026-11-22 C++ 侧提速"一节和 `docs/evidence/20261122_cpp_latency/report.json`。

目标是 `AGENTS.md` 11-16 §1 那一条：`prepare_plan` 入口到 `PLAN_READY`，p95 ≤ 2.0 s，成功率不低于基线。连续作业入口要求曝光后 `plan_validity_s`（缺省 5 s）内起动，所以要的是 p95，失败分支的耗时和成功分支一样重要。

注意两个口径不是一回事。12 s 是 `tools/replay_single_carton_record.py` 的 `planning_s`，它从 `acquire_workers` 之后开始计，到 `search_box` 返回为止。绑定、六个包的注册摘要、`complete-plan.json`、证据 flush 都不在里面。目标口径包含这些。

达不到时按环节如实报告差多少，不要靠削弱或跳过任何检查来达标。

## 现状

cda4e336、起点 S、cpp2 配置，`planning_s` 12.57 s：

| 环节 | 现在 |
|---|---:|
| 候选生成（纯 Python） | 1.6 s |
| 终态预检（含场景上传） | 1.9 s |
| 六段车道规划 | 5.2 s |
| 六段审计 | 3.8 s |
| search_box 之后到 PLAN_READY | 文档未报 |

## 代码审阅得到的事实

来自一次外部审阅。审阅者没有运行任何东西，所有耗时判断都要用剖析核实；数据不支持的按数据走，并在报告里写明哪条被否定。

全文读过：`worker.cpp`、`clearance.h`、`clearance_workspace.h`、`sc_composite.h`、`native_prepare.h`、`eco65_single_carton.h`；`single_carton/` 下的 `planner.py`、`cycle_structure.py`、`native_flow.py`、`resident.py`、`lifecycle.py`、`holding_capacity.py`、`geometry.py`、`evidence.py`；`controlled/trajectory.py`、`binary_frames.py`、`timing.py`、`tools/replay_single_carton_record.py`。

没读：C++ 的 `binary_frames.h`、`native_robot.h`、`native_deadline.h`、`scene_geometry.h`、`recorded_ompl_manager.cpp`；Python 的 `controlled/planner.py`、`coordinates.py`、`config.py`、`site_scene.py`、`runtime.py`、`cycle.py`；全部测试；`eco65_real_site.md` 的前 1900 行。

### 从代码直接确认的

1. **四个阶段严格串行。** `search_box` 里候选生成、预检、车道规划、审计依次进行。`parallel_candidates` 要等一个候选的六段全部规划完（`PLANNED`）才调用 `verify_candidate`，审计期间用 `idle.clear()` 暂停车道。
2. **候选 2–4 和候选 1 同时规划。** 4 条车道 × 每车道 3 个 worker，占满 12 个进程。三帧的失败记录只有持力不足和终态受阻，被接受的都是排序第一的候选，其余三条车道的工作没有用上。
3. **每个 plan/validate 请求都付一次固定开销。** 路径是 `Worker::dispatch` → `expandSceneHandle` → `Worker::run`：
   - 把请求整体拷贝一次，再把场景句柄里的整份 world JSON 拷回请求；
   - `base->diff()` 加 `decoupleParent()` 复制整个场景，设置 ACM，挂载附着物；
   - 重新构造 `SingleCartonContact`、`Clearance`、`ClearanceWorkspace`。后者对全部机器人/吸具实体与全部物体逐对枚举，每对按名字查 ACM 和间隙要求；世界物体的凸体每次重建，机器人凸体只在 TWO_LEVEL 下有进程级缓存；然后建层次树，再做 `prepareTwoLevel` 和 `prepareAdaptive`；
   - 再跑一次 MoveIt 网格碰撞检查（`scene->checkCollision`）；
   - 回复里带整份策略 JSON（`Clearance::evidence`）。

   `planner.py` 自己记录的常数是每个审计请求 0.3 s（`AUDIT_REQUEST_S`）；文档记录失败的 LIN 每次 0.19 s。
4. **LIN 降速阶梯每一档是一个独立请求。** `plan_segment` 按 1.0、0.5、0.25、0.1、0.05 逐档发请求。三帧里接触用到 0.25 或 0.1，脱垛 0.25 或 0.1，退离都是 0.05。失败的档只付固定开销，不产生任何结果。
5. **退离依赖实际接触位姿。** `plan_empty_return` 用 `withdraw_goal(tcp(q_release), actual, 面法向, 预接触距离)` 算目标，用 `released_task` 把箱子、吸附面和实测箱刚体移到释放位姿。所以它排在接触段之后，每轮的轨迹只是近似相同，不是逐位相同。三帧点数相同（5001 / 7958）是因为时长由速度比例和 J1 行程决定。
6. **规划内的稠密输出检查已经推迟给审计**（`defer_output_check_to_sent_path`）。规划请求里剩下的是对规划器返回的每个路点跑一次间隙判据（`solution_path_check=CLEARANCE_PREDICATE`），文档记录长 PTP 约 0.35 s。
7. **审计。** `Clearance::checkPath` 对探测路径的每条边至少再对半细分一次，检查状态数约为探测状态的 2 倍。自适应跳步的约束按刚体组建立（`adaptiveConstraints`）：间隙取组内所有物体对的最小值，力臂取组内所有实体的最大值，间隙上限 50 mm。`planner.py` 记录的单 worker 成本是 ADAPTIVE 每个探测状态 0.25 ms、ADAPTIVE_CONTACT 0.6 ms、SAMPLED 4.5 ms。ADAPTIVE_CONTACT 每个样本要做一次整机 FK 和单箱规则检查。
8. **`planning_module=NATIVE` 落后于 PYTHON 流程。** `sc_composite.h` 只有四段流程；`search_box` 里六段循环加 NATIVE 直接报错。`sentCheck` 的分块在单个 worker 内串行。复合请求返回后，`native_flow.plan_one` 让 Python 把 `plan_candidate` 对着记录重放一遍，每个请求再做一次规范 JSON 比对。GEOMETRIC 探测网格只有 Python 实现（`trajectory.prepare`），`native_prepare.h` 里没有。
9. **Python 侧的工作量。**
   - 车道阶段有 12 个 Python 线程（4 车道 × 3 条链）加 1 个证据线程，共用一个 GIL。
   - `EvidenceWriter` 是单线程，逐条 `json.dumps` 后写盘。报告里 `evidence_json_and_disk` 累计 15.35 s，整段是 12.57 s。
   - 每段的 `prepare` 用 scipy 分段多项式，再把发送点和探测路径转成 Python 列表。
   - `holding_capacity.sent_poses` 对脱垛和转运的每个发送点逐点调用 Python FK（cda4e336 约 9600 点），在审计之后串行执行。
   - `binary_frames.split` 编码时对每个浮点做 Python 级类型检查；规划回复里的 `points` 是字典列表，不走二进制块。
   - 每个请求发送前同步写一份 PENDING 记录。
10. **search_box 之后的部分。** `Authority.register` 对每个包做一次规范 JSON 加 SHA-256。包里有 `prepared`（发送点、探测路径、原始路点）和 `evidence`；`evidence['context']` 带着整份 world，六段各一份；`binding` 里还有 `site_scene` 和 `release_world`。然后写 `complete-plan.json`，再 `evidence.flush` 等队列清空。回放工具里有 `measure_plan_tail` 专门量这一段。
11. **候选生成。** `contact_candidates` 对偏移网格和抬升高度逐位姿做 Python 循环，每个位姿调用 `valid_cups`、`seal_check`、`candidate_contact_check`。`contact_search` 分支里每个基候选重建一次 KD 树，而面是同一个。
12. **场景上传是懒加载。** 每个 worker 第一次收到请求时才上传，Python 端每个 worker 各编码一次 world。

### 推断，未经验证

- 按文档给的单次请求耗时估，车道关键路径（接触 → 退离阶梯 → 空载回程）上的原生请求合计约 2 s，而车道阶段实测 5.2 s。差的约 3 s 判断在 Python 侧：GIL 争用、`prepare`、记录写盘。
- 固定开销的主要成分是逐对枚举加按名字查 ACM。机器人自身的实体对与场景无关，可以每进程只算一次。
- 审计里精确点多，是因为"组内最小间隙 × 组内最大力臂"过于保守：靠近轴线的近距离对力臂很小，力臂大的远端实体间隙很大。
- search_box 之后到 PLAN_READY 的部分在秒级。

## 建议的顺序

每一步单独提交、单独测量。顺序可以按剖析结果调整，调整了就说明原因。

0. **测量。**
   - 用 `measure_plan_tail` 和正式入口各量一次 search_box 之后的部分，分项报出。
   - 逐请求的关键路径：每个请求的固定开销（解析、world 拷贝、场景复制、工作区构造、网格检查、回复）和可变开销。
   - 车道阶段各 Python 线程的时间去向，证据线程的队列长度随时间的变化。
   - 审计每段的样本数、精确点数、每个精确点的耗时、`adaptiveConstraints` 耗时。
   - 列出全部能按六段循环回放的保存记录，数量如实报告。至少要有首选候选在路径或审计阶段失败的样本；文档提到 dev2 回放里有 9 次空载回程被否决、18 次 OMPL 预算耗尽。
1. **调度，只改 Python，判定不变。**
   - 审计流水化：某段的 `prepared` 一出来就开始审计，不等六段规划完。审计请求不调用规划器，不消耗 OMPL 随机数。接受规则不变，仍是优先级最高且规划和审计都通过的候选。
   - 给候选 1 留出审计用的 worker。车道数和每车道 worker 数怎么分，用测量决定。
   - 场景上传提前到候选生成期间，world 只编码一次。
2. **worker 固定开销，C++。**
   - LIN 阶梯并入一个请求：worker 内复用同一个场景和 `Clearance`，按原顺序逐档尝试，每档结果都写进回复。判定应当相同；`mtc_attempt_index`、`pipeline_calls`、间隙计数这类证据字段会变，在等价报告里列出。
   - 工作区改成"模板加增量"：机器人自身实体对、分组、包围球、力臂每进程算一次；静态世界部分每个场景句柄算一次；每个请求只处理附着物和本阶段的允许对。键要覆盖模型与代理来源哈希、场景句柄、附着、阶段接触许可、物体对策略、间隙与检查模式；任一变化或任务重置即失效。
   - 请求内不再拷贝 world JSON。
3. **进程内的原生候选循环。** 从预检到六段审计在一个进程内用线程完成，共享只读几何；Python 发一个请求、收一个结果。包含六段循环、J1 方向检查、腕部与姿态规则、带 GEOMETRIC 探测网格的原生 `prepare`、持力后检。轨迹和探测路径用二进制，证据同理（`compact_requests` 已经对探测路径这样做了）。包摘要用 `canonical_digest`（11-16 §3g 已允许）。Python 实现保留为参考，现有的录制重放机制留作离线等价核对。
4. **审计。** 先看第 0 步的数据。候选方向：约束从按组改成按实体（每个实体用自己的间隙和自己的力臂）、静态场景的保守距离场（11-16 §3a 允许）、锚点处 `adaptiveConstraints` 的成本。任何新的下界都要给出证明，并用 SAMPLED 在保存记录上逐例对照判定、失败原因和首个失败点。
5. **候选生成。** 向量化或移到 C++，输出逐位相同。KD 树每个面只建一次。
6. **尾部。** 让注定失败的候选在毫秒级被否决，给出失败样本上的耗时分布。

审阅者的粗略估计：第 1 步后 8–9 s，第 2 步后 5 s 左右，第 3 步后 3 s 左右，第 4、5 步后进 2 s。这些不是验收门槛。

## 待用户决定，不要自行实施

走到这些地方时，先用离线诊断量出"做了能省多少、等价核对结果如何"，带着数据停下来问。诊断不得进入任何配置。

- **A. 退离方向改为吸具自身法向（工具系常量）。** 改了之后退离和空载回程才可能提前生成或跨轮次复用；后者还涉及 11-16 §6"路径不跨轮次"。第 2 步做完后这一项的收益不大，优先级低。
- **B. 规划内的逐路点间隙判据并入审计。** 每个 PTP 省约 0.3 s。但现在 PTP 在这一步失败会转 OMPL，并入后失败要到审计才发现，回退行为会变。与 11-16 §2、§3b 相关。
- **C. `prepare` 的等价验收方式。** 11-18 的核对有 33/193 例加速度差 1.8e-8，过不了 1e-12，来源是两套实现的舍入。备选做法是 Python 也调用同一份 C++ `prepare`。不解决这一项，第 3 步不能设为缺省。
- **D. 直线段用沿直线的时间参数化代替降速阶梯。** 轨迹会变，限值不变。现在速度和加速度比例一起降，退离约 60 mm 要 25 s。
- **E. 2 s 截止时间下 OMPL 回退怎么处理。** 现场 283 次请求成功 0 次，5 s 预算放不进 2 s。
- **F. PLAN_READY 之前必须落盘的证据范围。** 11-16 §3e 的原文是计划证据记录必须写完，回复和可读证据可以异步；现在的实现是把队列里的全部证据都 flush 完才返回。是否收窄到原文的范围，由用户定。

## 约束

`AGENTS.md` 全部规则照旧，尤其是 11-16 §2 的不变项。这里只强调最容易在提速时碰到的几条：

- 每个新机制都是显式开关，缺省关闭；在保存记录上有等价报告之后才可以提议改缺省；失败报错，不静默回退。
- 不改在用现场配置。开关只开在 `outputs/workbench/single_carton.start_release_cycle.json` 这类变体里，改前备份。
- 原生改动用开发版前缀（`build/moveit-dev`、`install/moveit-dev`），不覆盖正式安装的 worker，除非用户明确要求。
- 每个 worker 的请求历史决定它的 OMPL 结果。改调度时不能破坏这一点的可复现性。
- 不连接设备，不运动，不写阀门，执行许可为 0，物理验收保持 NOT_RUN。
- 只跑定向检查；每步之后 `m710_clearance_test optimized/reference`、单箱原生组件检查和文档规定的定向 pytest 集合都要通过。首轮失败如实保留，不用复测覆盖。

## 测量口径

沿用 11-22 的做法：worker 预热另计，回放前等 1 分钟负载降到 2 以下，`ECO65_TIMING=1`，不设截止时间。目标是 2 s 时单次测量的噪声已经不能忽略，所以每个配置每帧至少跑 5 次，报中位数和最大值。`planning_s`、search_box 之后的部分、正式的 `prepare_plan → PLAN_READY` 三个数分开报。

## 每一步交回什么

- 三帧各环节的前后耗时（中位数、最大值），以及失败样本上的耗时。
- 等价核对：选中候选、各段规划器、速度比例、发送点数、失败统计是否相同，发送轨迹逐点最大差值，六段审计结果。
- 哪些是实测，哪些是推断；上面"推断"里哪几条被证实、哪几条被否定。
- 没做的和没验证的。
- 证据写入 `docs/evidence/`，并在 `docs/eco65_real_site.md` 追加带日期的一节。

需要停下来问的只有三种情况：走到 A–F 任一项；要覆盖正式 worker 或改在用现场配置；剖析结果说明这份顺序整体不对。其余自行推进。
