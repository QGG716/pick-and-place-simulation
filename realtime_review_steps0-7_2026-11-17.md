# ECO65 实时化改造第 0–7 步审查记录

| 项目 | 内容 |
|---|---|
| 审查日期 | 2026-11-17 |
| 仓库 / 分支 | `QGG716/pick-and-place-simulation`，`feat/v0.6-eco65-ubuntu-real-integration` |
| 审查到的提交 | `abe36ae01abdeb9010f4012eaa76689380a437c8`（第 7.3 步） |
| 改造起点 | `0783e86abce45e3e1d79827cd5faef1ec02d6834`（2026-11-13，现场最后使用的版本） |
| 审查方式 | 只读 GitHub 上的报告、文档和部分源码。没有运行任何代码，没有接触硬件。 |
| 目标 | `prepare_plan` 入口到 `PLAN_READY`，p95 ≤ 2.0 s（AGENTS.md 2026-11-16 §1） |

## 1. 结论

1. 第 0–7 步都已提交。新增开关缺省全部关闭，示例配置 `configs/workbench/single_carton.example.json` 没有改动。
2. 没有一项验收是在现场数据上做的。现场任务记录已删除，所有等价性和计时都来自两个合成场景，运行在共享 GPU 服务器上。
3. 规划时延离目标还很远。目前最好的一次是 47.6 s，约为 2 s 目标的 24 倍。按现状第 8 步的检查点过不了。
4. 剩余时间主要是每个原生请求的固定开销（每次重建碰撞工作区），不是碰撞检查本身。第 8 步之前需要先处理这一项。
5. 我读过的两级检查和自适应审计代码里，没有发现会放过不合格状态的路径。

## 2. 在现场机上使用本分支前要知道的事

- **实机验收全部是 NOT_RUN。** 第 1–7 步的任何机制都没有在真实机械臂、相机、串口上运行过。
- **实机执行时不要打开任何新开关。** 这些开关是：`planning_deadline_s`、`resident_workers`、`scene_handles`、`async_evidence`、`path_endpoint_reuse`、`binding_hash_cache`、`collision_check_mode`、`sent_path_audit`、`planning_module`。
- **开着 `ECO65_TIMING=1` 不要点"执行"。** 第 1 步报告写明：计时与实机执行同时运行的安全性没有确立。计时只用于观测和规划，执行前重启并关闭计时。
- **缺省配置下的 HEAD 也没有实机运行过**，并且没有与 `0783e86` 做过完整规划的对比（见问题 R2）。建议先完成第 8 节第 4 项，再用 HEAD 做实机抓放。
- **拉取后要重新编译安装 worker。** `worker.cpp`、`clearance.h`、`clearance_workspace.h` 等原生文件都有改动。
- **`resident_workers` 没有在完整工作台界面下跑过**，只测过 `plan_cycle` 和 `TaskManager` 的启动钩子（第 4 步报告）。
- 第 1 步报告记载，现场机上的任务记录和现场本地配置文件都已删除。采集前先确认 `outputs/workbench/` 下的本地配置是否还在。

## 3. 审查范围

**读了全文的：**

- 七份步骤报告（`docs/evidence/realtime_20261116_*/report.json`）和 16 个提交的说明。
- `AGENTS.md` 的提交差异、`docs/native_boundary.md`、`docs/sent_path_adaptive_audit.md`。
- 原生代码：`clearance_workspace.h`、`clearance.h`、`eco65_single_carton.h`。
- Python：`single_carton/runtime.py`、`controlled/trajectory.py`、`single_carton/evidence.py`、`single_carton/native_flow.py`。
- 工具和配置：`tools/check_step4_mechanisms.py`、`tools/check_native_planning.py`、`configs/workbench/planning_budget.json`、`configs/workbench/single_carton.example.json`。

**没读的（相关结论只来自报告）：**

- `single_carton/planner.py`、`resident.py`、`config.py` 的现行版本。
- `worker.cpp`、`sc_composite.h`、`native_prepare.h`、`binary_frames.h`、`clearance_test.cpp`。
- 新增的测试文件和其余检查工具（量具、碰撞等价、自适应审计、prepare、二进制协议）。
- `docs/eco65_real_site.md` 里各步新追加的小节。

## 4. 提交与报告

| 步 | 提交 | 报告目录（`docs/evidence/` 下） | 定向回归 |
|---|---|---|---|
| 0 修订 AGENTS.md | `de8cf04` | — | — |
| 1 基线测量 | `062ad93`、`abf6835` | `realtime_20261116_baseline` | 435 通过，1 失败 |
| 2 量具与截止时间 | `4dd9c20` | `realtime_20261116_deadline` | 465 通过，1 失败 |
| 3 接口冻结 | `2bf97af` | `realtime_20261116_boundary` | 未运行（只改文档） |
| 4 低风险五项 | `aa360ec`、`ffeac9f`、`256e4c9`、`c5696c5`、`40c2af1`、`f1a2a1c` | `realtime_20261116_low_risk` | 501 通过，1 失败 |
| 5 两级碰撞检查 | `0cf6714` | `realtime_20261116_two_level` | 510 通过，1 失败 |
| 6 自适应发送路径审计 | `c70e3bd` | `realtime_20261116_adaptive_audit` | 526 通过，1 失败 |
| 7 复合请求、二进制帧、prepare 下沉 | `521d2db`、`8acd14e`、`abe36ae` | `realtime_20261116_native_planning` | 540 通过，1 失败 |

每一步失败的都是同一个测试，见问题 R15。原生检查 `m710_clearance_test` 的 `optimized`、`reference` 每步都是 PASS，第 5 步起加 `two_level`，第 6 步起加 `adaptive`。

## 5. 逐步审查

### 第 0 步　修订 AGENTS.md

- 修订稿原样插入最顶部，共 +171 行、−1 行。唯一改动的原文是 11 月 12 日一节的标题后缀。
- 我要求列出的"仍然矛盾的句子"清单不在仓库里，我没有看到。

### 第 1 步　基线测量

- 计时埋点完成，缺省关闭（`ECO65_TIMING=1`、`M710_NATIVE_TIMING=1`）。
- 调度基线有（fake IO，5 次）：点击执行到首个点发出 3.25–3.88 s。
- 规划和视觉基线都没有：现场记录已删除，服务器上没有相机和冻结帧。
- 证实了文件重哈希的开销：每次绑定重算 807 个文件要 1.86 s，其中两个模型目录（1.31 GB）占 1.67 s。一个周期在执行路径上算 5 次，每段开始都要等约 1.9 s。
- 修正我之前的一个假设：单状态检查的时间不在 FCL 精确距离上。合成场景里 1489 个状态共 3.07 s，粗筛（遍历包围树）占 2.81 s，精确距离只占 0.10 s。
- 发现一个既有风险，见问题 R14。

### 第 2 步　量具与统一截止时间

- `tools/bench_planning_latency.py`、预算表 `configs/workbench/planning_budget.json`、可选配置 `planning_deadline_s` 都已完成。
- 不设截止时间时，合成搜索的 24 个记录文件与改前逐个一致。
- 到期归类为 `PlanningBudgetExhausted`，与不可行和 IPC 失败分开，并给出已搜、未搜候选数。
- 缺口：量具对规划和视觉没有样本；16 次真实 worker 到期测试中 1 次晚了 122 ms（要求 50 ms 内）。
- 报告里记下的数字：worker 冷启动 3.8 s；worker 关闭每个 5 s（忽略 SIGTERM）；`contact_candidates` 约 0.6 s 纯 Python。

### 第 3 步　接口冻结

- `docs/native_boundary.md` 写了 A–E 五个契约，每个字段组都标了代码来源。
- 通道选的是"现有管道上的二进制帧加共享内存"。共享内存到第 7 步也没有实现，文档已注明。

### 第 4 步　低风险五项

- 五个开关各自一个提交，缺省关闭。
- 合成场景上三项对比都逐项一致：开关关对开关开；同一场景连跑两次；先 A 后 B 对单独跑 B。
- 调度收益明确：打开 `binding_hash_cache` 后，点击执行到首点发出从 3.3–3.9 s 降到 0.12–0.15 s，每段交接从约 1.9 s 降到 0.06–0.17 s。
- 请求仍在发出前同步落盘；`PLAN_READY` 和执行前都会等证据写完，写失败则拒绝。这一点我在 `runtime.py` 和 `evidence.py` 里核对过。
- 缺口：场景句柄只省了传输和哈希，worker 仍然每个请求重建碰撞工作区（问题 R3）。这是我写 prompt 时的遗漏。

### 第 5 步　两级碰撞检查

- 代理用的是包围球（由实体自身顶点算出）、刚体组包围球和静态包围树。代理只能判通过，不能判拒绝。
- 其余物体对按原顺序走原来的 AABB 测试和 `fcl::distance`，所以首个失败的物体对与 EXACT 相同。这一点我在 `clearance_workspace.h` 里核对过。
- 等价性：316 条合成记录、520,340 次检查，零反例、零多余拒绝、零首个失败差异。
- 单状态中位数：接触段 1.573 → 0.464 ms，脱离段 1.588 → 0.485 ms。0.5 ms 的硬指标在服务器上达标，余量很小，p90 约 0.8 ms。
- 缺口见问题 R16。

### 第 6 步　自适应发送路径审计

- 实现比 AGENTS.md §3b 授权的更保守：采样点序列与 SAMPLED 完全相同，只跳过能从上一个精确检查点证明通过的点。
- 推导写在 `docs/sent_path_adaptive_audit.md`。我对照 `clearance.h` 和 `clearance_workspace.h` 读过，力臂界和两物体同时运动的处理成立。
- 等价性：EXACT 下 228 个请求、TWO_LEVEL 下 152 个请求，判定和首个失败全部相同。构造的擦碰（间隙 2.398 mm）和穿入，两种方法都拒绝。
- 接近段精确检查次数 91,236 → 131。
- 只适用于"没有接触许可、没有支撑物"的段，其余段 worker 会拒绝 ADAPTIVE。对现场的影响见问题 R6。

### 第 7 步　复合请求、二进制帧、prepare 下沉

- `planning_module: PYTHON | NATIVE`，缺省 PYTHON。
- NATIVE 的做法：worker 执行复合请求并记录每个子请求和结果；Python 随后用回放客户端把原来的 `screen_candidate`、`plan_candidate` 原样重跑一遍，任何不一致都报 `NATIVE_FLOW_DIVERGED`，不回退。这一点我在 `native_flow.py` 里核对过。
- 合成场景上 NATIVE 与 PYTHON 的结果在 1e-12 内一致。往返次数 2535 → 23（场景 A）。
- prepare 的 C++ 版：424 个用例里 292 个拒绝情形与 Python 完全相同；位置和检查路径相差不超过 4e-14。
- 缺口较多，见问题 R4、R5、R7、R9、R10、R11。

## 6. 规划时延现状

以下都是合成场景上 `plan_cycle` 的墙钟时间，各跑一次，运行在共享服务器上，不是现场电脑。单位秒。

| 配置 | 场景 A | 场景 B | 来源 |
|---|---|---|---|
| 全部缺省 | 129.84 | 166.84 | 第 4 步 |
| 第 4 步五个开关全开 | 116.58 | 136.85 | 第 4 步 |
| TWO_LEVEL | 66.56 | 81.00 | 第 5 步 |
| TWO_LEVEL + ADAPTIVE | 62.47 | 65.75 | 第 6 步 |
| PYTHON + TWO_LEVEL + ADAPTIVE（第 7 步重跑） | 51.8 | 75.5 | 第 7 步 |
| NATIVE + TWO_LEVEL + ADAPTIVE | 47.6 | 66.1 | 第 7 步 |
| NATIVE + EXACT + SAMPLED | 238.0 | 386.2 | 第 7 步 |

- 上表各行第 4 步的开关都是关的，墙钟里含 worker 的启动和逐个关闭。
- 五个开关全开加 TWO_LEVEL、ADAPTIVE（PYTHON 流程，4 次规划）的量具结果是 p50 76.7 s、p95 105.2 s。其中连续任务之间等 worker 重置占 25–28 s。

这 4 次规划的分环节结果（第 6 步报告）：

| 环节 | p50 | p95 | 预算 |
|---|---|---|---|
| 原位检查与候选生成 | 1.06 | 1.61 | 0.02 |
| 终态预检 | 40.5 | 40.9 | 0.45 |
| 候选规划 | 10.1 | 21.6 | 0.55 |
| 发送路径检查 | 6.0 | 10.6 | 0.55 |
| 收尾（绑定、签发、写计划） | 0.96 | 1.38 | 0.15 |

## 7. 问题清单

### 阻塞第 8 步检查点的

**R1　没有现场数据。**
- 所有等价性和计时都来自两个合成场景：同一个 150×100×100 mm 箱子、两个起点。
- 合成场景的世界里只有目标箱一个物体。`check_step4_mechanisms.py` 去掉了现场配置，所以没有障碍、没有托盘支撑、没有工作范围边界。第 6 步的审计对比另外加了带杂物和带边界的变体，其他步骤没有。
- 现场场景文档记录的是 40 个观测障碍加 6 面边界，箱子放在托盘上。
- AGENTS.md §2 要求在保存记录上有等价报告之后，NATIVE 才能成为缺省。这个报告目前不存在。

**R2　缺省配置下的 HEAD 没有与 `0783e86` 做过完整规划对比。**
- 现有证据是定向回归、原生组件检查，以及第 2 步一个无解合成搜索的逐文件对比。
- 第 4 步重构了 `plan_cycle` 的 worker 生命周期。完整四段规划的新旧对比没有做过，因为可行的合成场景是第 4 步才建的。

**R3　每个原生请求都重建碰撞工作区。**
- 重建内容：拷贝世界、重建 715 个机器人碰撞体的 FCL 对象、逐对过滤。
- 第 6 步报告：每个校验请求约 0.3–0.4 s 花在导入场景和建工作区上。
- 第 7 步的分项（NATIVE、TWO_LEVEL、ADAPTIVE，场景 A）：153 个校验请求共 16.5 s，平均每个 83–173 ms；而 2305 次 IK 一共只有 1.09 s。
- 终态预检 14.7 s 基本就是这项开销。

**R4　最终组合没有跑过。**
- "五个开关全开 + TWO_LEVEL + ADAPTIVE + NATIVE"既没有等价性结果，也没有计时。
- `check_step4_mechanisms.py` 的 `on` 模式没有把 `planning_module` 传给规划器，现有工具跑不了这个组合。

### 需要处理的

**R5　NATIVE 配 EXACT、SAMPLED 时比 PYTHON 慢一倍以上。**
- 126 → 238 s，151 → 386 s。
- 发送路径检查在车道自己的 worker 里串行做，不再分给检查池；每条车道都把自己的候选完整验证一遍。

**R6　现场的接触段和脱离段用不上自适应审计。**
- 接触段有接触许可；箱子放在托盘上，脱离段有支撑物。两段都按原密度检查。
- 合成场景里脱离段没有支撑，所以报告里脱离段的收益（131,082 → 89 次检查）在现场不会出现。
- 估算：每个探测状态约 3 次原生检查，按 TWO_LEVEL 中位数 0.46–0.49 ms 计，合成场景的接触段（2125 个探测状态）单进程约 2–3 s。现场这两段的实际耗时要用现场记录来量。

**R7　纯 Python 开销没有被任何步骤覆盖。**
- `contact_candidates` 约 0.6 s；原位检查与候选生成合计 1.06–1.61 s。
- 计划收尾 0.96–1.38 s。
- NATIVE 流程在复合请求返回后还要把 Python 原流程完整重放一遍，这部分耗时没有单独计量。

**R8　worker 的启动、关闭、重置都很慢。**
- 冷启动 3.8 s；关闭每个 5 s；常驻模式下连续任务之间等重置 25–28 s。
- 连续多箱作业会直接受重置时间影响。

**R9　OMPL 没有被等价性测试覆盖到。**
- 场景 A、B 的四段全是 Pilz（PTP、LIN）。"任务间随机数状态不串"这一条没有被一次 OMPL 成功规划检验过。
- 单车道时，NATIVE 把接近和带载返回放在同一个 worker 上依次规划，PYTHON 分到两个 worker。需要 OMPL 时两者的随机数历史可能不同（第 7 步报告自述）。

**R10　二进制帧目前没有传输收益，共享内存没有实现。**
- 同一个请求：JSON 10.32 MB、4.96 s；二进制 10.07 MB、5.87 s。请求体的大头是世界描述，不是浮点数组。
- NATIVE 下每次规划仍发送约 94–102 MB，接收量从约 2.5 MB 涨到 77–109 MB（worker 回传子请求记录和 prepare 结果）。这些数字是第 4 步开关关闭时测的，场景句柄打开后的数字没有。
- `docs/native_boundary.md` §0 仍把共享内存写在通道选择里，建议删掉。

**R11　prepare 的速度、加速度没有达到 1e-12。**
- 速度最大相差 5.4e-12，加速度最大相差 8.4e-10；位置达标。
- 报告把原因归为 numpy 在该服务器 CPU 上的数组 `pow` 与 libm 相差 1 ulp，并指出 Python 自己的结果也随 CPU 变化。
- 现场电脑的 CPU 不同，这项对比要在现场机上重跑。是否把速度、加速度的容差放宽，需要用户决定。

**R12　截止时间到期后的返回偶尔超过 50 ms。**
- 30 s 截止时间那次晚了 122 ms。原因是 Python 线程在持有 GIL 的 C 调用里无法被抢占。

**R13　worker 的内部尝试日志缺省写在 `/tmp`。**
- 路径由 `M710_WORKER_JOURNAL_DIR` 决定，缺省 `/tmp/m710_worker_journal_<pid>.jsonl`。
- 应写到任务记录目录，否则重启后证据丢失。

### 记录在案的

**R14　界面进程的垃圾回收停顿可能触发停止（既有风险，不是本次改造引入的）。**
- 持有规划快照的进程里，Python 第 2 代回收停顿 0.04–0.135 s，关闭计时时也存在。
- `Cycle.tick` 要求关节样本不超过 0.1 s。停顿正好落在检查之前，周期会以 `LIVE_ROBOT_STALE_DISCONNECTED_OR_FAULT` 停止。
- 目前只在 fake IO 加第一版埋点时出现过。停止本身是安全方向，但带箱时停止后需要现场处置。

**R15　一个测试在服务器上一直失败。**
- `tests/test_single_carton_review_fixes.py::test_exact_prior_face_keeps_a_bounded_envelope_that_covers_the_prior`。
- 在改造前的代码上同样失败，报告归因于服务器的库版本与现场 `.venv` 不同。
- 需要在现场环境确认它是否通过。

**R16　两级检查在多静态障碍场景下的覆盖没有确认。**
- 52 万次等价性检查用的是只有一个世界物体的场景，静态包围树里几乎只有机器人自己的固定部件。
- 单元测试 `m710_clearance_test two_level`（4500 个状态）用的是什么场景，我没有读。

## 8. 现场机待办

按顺序做。前四项都不产生运动和阀门命令。

1. **拉取并重建。**

   ```bash
   git fetch && git checkout abe36ae01abdeb9010f4012eaa76689380a437c8
   source /opt/ros/humble/setup.bash
   cmake --build build/moveit --target m710_moveit_worker m710_clearance_test -j2
   cmake --install build/moveit --prefix "$PWD/install/moveit"
   for mode in optimized reference two_level adaptive; do build/moveit/m710_clearance_test $mode; done
   .venv/bin/python tools/check_eco65_single_carton_native.py
   ```

2. **在现场环境跑定向回归**（命令见 `docs/eco65_real_site.md` 的"软件验证与实机范围"一节）。记录通过数，并确认 R15 那个测试在这里是否通过。

3. **采集现场记录。**
   - 以 `ECO65_TIMING=1` 启动单箱工作台，配置保持缺省（不开任何新开关）。
   - 每次只做"准备观测"和"规划"，不点"执行"。
   - 换不同的箱子位置和遮挡方式多采几次，包括规划失败的情形。建议不少于 20 条，p95 才有意义。
   - 保留每个任务目录：`observation.json/.npz`、`segmentation.npz`、`geometry.json`、`site-scene.json`、`timing-observe.json`、`timing-prepare_plan.json`。
   - 采集结束后重启工作台并关闭计时。

4. **离线对比（不连设备）。**
   - 用 `tools/replay_single_carton_record.py` 在同一批记录上分别回放 `0783e86` 和 HEAD（缺省配置），比较选中的候选、四段规划器、发送点数、时长、失败统计。这一项对应 R2。
   - 用 `tools/bench_planning_latency.py --discover` 得到规划和视觉的现场基线，并据此修订预算表。
   - 在同一批记录上重跑第 5、6、7 步的等价性工具，对应 R1、R11、R16。

5. **把记录交回开发环境。** `outputs/` 不进仓库，传输方式由你决定。第 9 步（场景构建下沉）也需要这些帧。

## 9. 第 8 步之前建议插入的工作

1. 碰撞工作区按场景构建一次，后续请求只更新附着、许可和阶段（R3）。
2. 让检查工具支持"全部开关 + NATIVE"，补上等价性和分环节计时（R4）。
3. 合成场景加入障碍、托盘支撑和工作范围边界，直到现场记录可用（R1、R6、R16）。
4. 计量并处理 Python 侧开销：候选生成、计划收尾、NATIVE 的重放（R7）。
5. worker 的启动、关闭和重置时间（R8）。
6. NATIVE 下发送路径检查的分配方式（R5）。
7. 日志路径改到任务目录；契约文档删去共享内存（R13、R10）。

第 8 步原定的 IK 改造收益有限：2305 次 IK 一共约 1 s，瓶颈不在 IK 求解本身。

## 10. 未验证事项

- 报告里的全部数字我没有复现，只核对了它们之间是否自洽。
- 第 3 节"没读的"文件里如果有问题，本记录发现不了。
- 现场电脑（i5-12500H）上的任何耗时都是未知的。
- 真实 SDK、UDP 反馈、串口压力心跳、控制器负载查询的延迟没有测过，报告里用的都是假设备。
