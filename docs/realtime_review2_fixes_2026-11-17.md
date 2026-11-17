# ECO65 2bd36a9 复审正确性修复（2026-11-17）

本轮实现 R2-01～R2-06 及附件的三个集成补充项。执行锁、40 mm 工具—机器人间隙、原显式豁免、腕部分支限制和默认 PYTHON/优化开关保持原值。没有连接机械臂、相机或真空设备，没有采集新现场样本，没有运动或阀命令。物理验收仍为 **NOT_RUN**，前 7 步整体验收仍为 **false**。

## 提交与证据归属

- 复审基准：`2bd36a93063eeae77c05ad263abfaa9f62bf7f2a`；实际开始 HEAD：`9ba28a6080b3b248028772b6a08c870e5107c231`。读取两者差异后从实际 HEAD 开发，没有 reset、覆盖旧工作区或改写历史。
- 最终代码验证提交：`48043b4e84a6c2e4f52b850590e2ccabbee29670`。最后的报告提交仅追加文档及证据；其 SHA 由 Git 历史和交付回复提供。
- 并行会话提交 `d3359038a25ca19e409283e8132f145189008fcb`（保存失败记录的可视化）完整保留。原有未跟踪的 `realtime_review_steps0-7_2026-11-17.md` 未修改、未加入提交。
- 各修复提交及复现见下表；结构化汇总见 [report.json](evidence/realtime_20261117_review2/report.json)。逐步证据中的 HEAD 是当时已提交的父提交，检查包含其后的工作树修改；最终固定 SHA 检查单独记录，不把父提交误标为受测代码。
- `4ce654b8d12a1d8862370fbc250c5aa0432bd41a` 上的保存记录验证明确单列。到最终代码 SHA 只修正离线回放工具的胜出候选计数及测试/文档；`src/` 和 `ros2/` 字节未变，证据记录相应源码及 worker SHA-256。

## 六项修复

| 项目 / 提交 | 原因与最终修改 | 修复前 → 修复后复现 |
|---|---|---|
| R2-01 / `123033f` | [runtime.py](../src/unloading_sim/workbench/single_carton/runtime.py) 最终证据等待后，重新检查取消、观测代次及每段许可/会话。校验与 cycle.start 置于 [backend.py](../src/unloading_sim/workbench/backend.py) 的观测锁边界；设备失效的代次递增本身也原子化。实际执行仍原子 claim 许可，不跨设备 I/O 持有许可锁。 | 实际 EvidenceWriter 的 NPZ 写盘阻塞期间失效：3 failed、1 passed → 通过；补启动互斥及并发递增用例。取消、会话、许可失效均拒绝启动。 |
| R2-02 / `a532154` | [native_flow.py](../src/unloading_sim/workbench/single_carton/native_flow.py) 传相同腕部配置与冻结 q_start；[sc_composite.h](../ros2/m710_moveit_backend/src/sc_composite.h) 在与 Python 相同的 IK 排序后、分段 prepare 后过滤。拒绝分支仍检查未消费调用，保留 NATIVE_FLOW_DIVERGED。 | 实际 C++ composite/nativePrepare 输出驱动实际 Python 回放：14 failed、4 passed → 18 passed。覆盖 J5 符号/最小角、J4/J6、全拒绝/部分保留、四段中间越界、正常链路，全部 unused=0。 |
| R2-03 / `a426c02`，补充 `48043b4` | [lifecycle.py](../src/unloading_sim/workbench/single_carton/lifecycle.py) 将 rename/回滚/记录锁等待放入后台；调用线程的任务锁与许可锁使用同一绝对截止时间。仅发布当前、未过期且已经持久化的结果。记录替换与回滚串行，迟到旧任务不能覆盖新任务记录。 | 250 ms rename（成功/失败）及两种锁等待原 4 failed → 通过，保留 100 ms + 50 ms。联合生命周期/预算定向组 63 passed，新增清理细节组 12 passed。应用层通过不表示操作系统硬实时保证。 |
| R2-04 / `1d65f9a` | [planner.py](../src/unloading_sim/workbench/single_carton/planner.py) 在同一锁下更新 closed/代次；预热、pool 启动及 [resident.py](../src/unloading_sim/workbench/single_carton/resident.py) 补 worker 发布前重新检查。拒收资源在锁外关闭，重复 close 不重复接收资源；清理异常和 wait_closed 结果可查询。 | 5 个确定性竞争窗口均修复前失败；修复后 resident 相关 41 passed，扩展关闭/清理组 7 passed。 |
| R2-05 / `5b99ddc` | [timing.py](../src/unloading_sim/workbench/timing.py) 将无时间戳的溢出汇总放在 since 过滤之后，注明 RECORDER_DRAIN_NOT_PHASE_TIMESTAMP；仍保留 8192 上限、GC 默认行为及写错误。 | 实际 8193 次 gc_pause + write_later(since) 原同步 TypeError → 3 passed（溢出、重入/并发、写失败可见性）。 |
| R2-06 / `4ce654b` | [bench_planning_latency.py](../tools/bench_planning_latency.py) 用全部输入计算完整/部分/缺失/无计时覆盖率，保留失败/取消/超时；单列完整成功子集耗时。覆盖不完整或没有成功样本时不宣称全量 PLAN_READY 达标。 | 完整 1 s 成功 + 无计时部分失败、完整无计时失败两种错误达标均复现；原有及新增统计组 13 passed，后续发现目录/成功子集用例纳入固定 SHA 检查。 |

三个集成补充项也已完成：

1. plan_one/detach_request 的所有 scene handle 上传传同一个 cancelled；修复前 CONTACT 的两种入口失败，修复后的跨模块测试逐次断言回调。生命周期证据分开记录 `caller_returned_s`、`planning_thread_finished_s`、`worker_requests_finished_s`、`cleanup_completed_s` 与清理错误，调用者返回不等于 worker 停止。
2. 搜索胜出立即记录 accepted/SEARCH_SUCCEEDED；[budget.py](../src/unloading_sim/workbench/single_carton/budget.py) 在收尾超时时计入已搜索并标记 SEARCH_SUCCEEDED_FINALIZATION_BUDGET_EXHAUSTED，保留其余未搜索数量。离线 replay 的独立入口也同步修复，10 项相关检查通过。
3. discover 默认覆盖 `outputs/single_carton` 和 `outputs/realtime-field-*`，支持重复 `--discover-root` 并去重；完整入口基线明确使用 `--timing`，部分离线 replay 另记。

## 固定代码验证

最终本地检查：**656 passed**（185.89 s），`m710_clearance_test optimized/reference` 与 `tools/check_eco65_single_carton_native.py` PASS。实际 C++ limits/journal/path 契约和 two_level/adaptive 也通过。journal 使用已有 `/dev/full` 的符号链接注入故障，不要求容器允许 mknod；没有跳过故障断言。

GPU 同一代码提交的四种碰撞模式、实际 C++ limits/journal/path 契约及单箱原生组件全部 PASS；组件为 491 发送点、2.45 s 合成轨迹，实际最终路径审计通过，没有许可或硬件执行。首轮完整 Python 集为 636 passed / 2 failed / 18 errors：18 个错误是主机缺 nlohmann 头文件；两项失败分别为既有 prior 几何用例，以及 `test_tail_stages_are_transactional_and_bounded[True-plan_record]` 约 167.733 ms > 原 150 ms。没有证据将后一项完全归因于 OS 调度，稳定返回门槛仍未过。

该超时的后台记录为 INVALIDATED、SEARCH_STEP_RUNNING_AT_DEADLINE，说明这一失败运行尚未到达测试所注入的最终记录写盘点；记录的 caller_returned_s 对应约 167.674 ms，之后 worker 请求及清理均结束且 cleanup_errors 为空。它证明本轮最终 rename 的确定性阻塞已消除之外，整个调用返回的稳定性仍有未过样本，不能把清理成功当成返回时限通过。

GPU 使用已有 `/root/v05-gpu-venv` 和 Ubuntu 22.04 Humble/MoveIt chroot，没有升级依赖或改系统配置。独立 checkout 为 `/root/m710-gap-search-20261107/rootfs/work-eco65-review2-20261117`；旧脏工作区未覆盖。新复制资产导致根盘 ENOSPC 后，仅将本轮新副本改为 2378 个哈希一致的只读用途硬链接，所有必需模型资产可用；另外 126 个非必需旧副本与对应 LFS 指针不符，未采用，也未覆盖原字节。日志放在数据盘 `/root/autodl-tmp/eco65-review2-20261117`。本地 NumPy/SciPy 为 2.2.6/1.15.3，GPU 为 1.26.4/1.14.1；两端 Python 均 3.10.12。

GPU 补上已有 chroot 的 JSON 头文件搜索路径后，对同一完整定向集复测：**655 passed / 1 failed / 0 errors / 0 skipped**（475.85 s），唯一失败仍为既有 prior 几何用例；本轮新增跨语言/竞争/统计回归通过，生命周期组此次也通过。保留首轮超时，仍不宣称稳定返回已验收。原生运行仅为该子进程设置已有 chroot 的库及 ament 路径，所有具体命令/环境变量及两端二进制哈希见 [verification.json](evidence/realtime_20261117_review2/verification.json)。

必需定向集及本轮/上轮回归的命令：

```bash
xvfb-run -a .venv/bin/pytest -q tests/test_single_carton_*.py \
  tests/test_controlled_status_ui.py tests/test_controlled_read_ui.py \
  tests/test_controlled_execution.py tests/test_controlled_payload_setting.py \
  tests/test_controlled_goal_planning.py tests/test_workbench_readonly.py \
  tests/test_workbench_coordinates.py tests/test_workbench_installation.py \
  tests/test_review2_*.py tests/test_review_*.py tests/test_realtime_field_collection.py
source /opt/ros/humble/setup.bash
build/moveit/m710_clearance_test optimized
build/moveit/m710_clearance_test reference
build/moveit/m710_clearance_test two_level
build/moveit/m710_clearance_test adaptive
.venv/bin/python tools/check_eco65_single_carton_native.py --output NEW_OUTPUT_DIR
g++ -std=c++17 -O1 -Iros2/m710_moveit_backend/src -I/usr/include/eigen3 \
  tests/native_review_contracts.cpp -o /tmp/eco65-review2-contracts
/tmp/eco65-review2-contracts limits
/tmp/eco65-review2-contracts journal
/tmp/eco65-review2-contracts path
```

逐步必需集依次为 573、591、609 项通过，均附 optimized/reference/native component PASS；R2-03～R2-06 的 609 项为共同工作树检查，不能重复计作四组独立样本。最终 656 项包含全部相关回归。

保留的中间失败包括：R2-01 首次必需集 571 passed / 2 个缺少新锁的 fake fixture 失败（补 fixture 初始化后 573 passed）；R2-03 首次联合组 47 passed / 1 个 BEFORE_DISPATCH 标签不兼容（恢复原标签后通过）；GPU 环境准备的 ENOSPC、主机缺 MTC 开发包，以及后台编译同时运行时中断的测试结果。没有放宽时间/数值阈值或删除断言来消除失败。

## 保存记录与指标的适用范围

旧完整现场记录仍在现场机，不宣称全部丢失。本轮只读使用已有输入，原始字节/哈希保持一致。

| 当前实现上的检查 | 输入与结果 | 可证明的范围 |
|---|---|---|
| 终态预检 PYTHON / NATIVE | 旧任务 `4cde3b86824046f99bc8085d419deb83`：32 候选，CLEAR 11 / REJECTED 13 / NOT_CONCLUSIVE 8；2413 次调用全部消费 | 实际当前 worker、完整 36 对象场景；两侧结果一致，无许可 |
| 终态预检 PYTHON / NATIVE | 已保存任务 `313f32ad66a240798df17905025edc89`：32 候选，CLEAR 10 / REJECTED 13 / NOT_CONCLUSIVE 9；2379 次调用全部消费 | 实际当前 worker、完整 40 对象场景；两侧结果一致，无许可 |
| 本地 prepare 数值 | 任务 `d99b1a8f5d71445395c158b3c353ca42`，27 条唯一保存轨迹，193 案例；193 一致，33 prepared / 160 rejected | 原 1e-12；q/v/a/t/碰撞路径最大差 0，解析速度界差 1.3878e-17。不同环境/输入的通过不能覆盖服务器旧失败 |
| GPU 原历史数值样例 | 同一旧服务器组件轨迹，11 案例中 8 一致，3 个 prepared 数值不合格；最大加速度差 9.3158283e-12 | 在本轮代码/worker 上重现历史失败；原 1e-12 未改 |
| GPU 同一保存轨迹集 | 上述 27 条轨迹原字节传输校验一致；193 案例中 160 一致，33 个 prepared 数值不合格；最大加速度差 1.8383593e-8 | 拒绝判定一致，数值等价不通过。两环境结果并列保留，尚未隔离数值差的根因 |
| 已保存完整入口计时汇总 | 两次失败返回为 118.800721 / 119.269865 s；成功 PLAN_READY 样本 0；meets_total_p95=null | 这是既有采集的计时，用当前统计工具重新汇总，不是本轮新运行的现场 p95 |
| 视觉及调度 | 视觉 3 次（含 1 次仅观测），p95 21.013707 s；完整调度 NO_SAMPLES | 不把仅观测追加为新的完整箱位，不混入规划/调度统计 |

终态预检没有运行完整四段路径，不能替代完整 plan_cycle 等价或物理验收。保存轨迹 prepare 也不是重新生成并验收一个实机可执行计划。数值工具进程正常结束并不代表通过：必须读取 `acceptance.all_identical`；两项 GPU 数值报告该字段均为 false。

```bash
.venv/bin/python tools/check_native_prepare.py \
  --records outputs/single_carton/d99b1a8f5d71445395c158b3c353ca42 \
  --output outputs/review2_20261117/local-saved-prepare.json
.venv/bin/python tools/bench_planning_latency.py \
  --timing outputs/realtime-field-20261117 --output outputs/review2_20261117/field-timing
# 使用尚不存在的输出目录；仅重建保存帧并比较终态预检，不规划/签发。
.venv/bin/python docs/evidence/realtime_20261117_review2/check_field_endpoints.py \
  outputs/single_carton/4cde3b86824046f99bc8085d419deb83 NEW_ENDPOINT_OUTPUT_DIR
```

## 保留门槛与下一阶段

- **steps_1_to_7_accepted=false**。六项代码修复完成与整体验收是两个结论；当前检查的完整范围、环境失败和剩余证据分别保留。
- **deadline_bound=NOT_STABLY_PASSED**：明确的同步 rename/锁阻塞路径已修复，但 GPU 仍观测到 167.733 ms 的失败，不用单次重跑通过代替稳定性结论。该时间断言之后的同用例断言在失败运行中未执行，不能据此声称该次完整事务检查通过。
- **prepare_numeric=FAILED**：1e-12 数值门槛保持原值；服务器历史及同一保存轨迹集的失败不能由本地 193 案例通过替代。
- GPU 既有 `test_exact_prior_face_keeps_a_bounded_envelope_that_covers_the_prior` 仍失败，尺寸为 `[0.14007188, 0.18569301, 0.108]`；与上轮服务器报告一致，未修改该断言或几何实现来取得通过。
- 完整调度仍缺成功样本；至少 20 条新现场输入未完成；prepare_plan→PLAN_READY p95≤2 s 与不降低基线成功率尚未证明。
- 执行锁保持 locked；本轮无普通设备采集或实机动作，PHYSICAL_CYCLE_COMPLETE / PHYSICAL_JOB_COMPLETE 均未获得。
- 跨请求碰撞工作区复用 **NOT_IMPLEMENTED**，本轮未启动。下一阶段单独提交：先确定每 worker 的完整上下文键（模型、场景版本、附件、接触许可、pair policy、裕量、清除模式），确保任务重置不携带前任务状态；再独立比较构造次数/耗时与 EXACT 判定等价，失败可追踪，不能跳过检查或静默回退。完成后再安排授权范围内的新现场基线及独立实机验收。
