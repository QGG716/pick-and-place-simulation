# ECO65 复审续修与 GPU 验证（2026-11-18）

本轮从目标分支实际 HEAD `2d5b5b5195dc0b165184e1439e89aa9a1d60a8cc` 开始。R2-01～R2-06 和三个集成补充项已由此前提交实现，本轮保留这些修改及现场姿态诊断，继续补齐返回路径和验收工具。源代码验证提交为 `8041a3c687f5e0b72d7e5c02cd0668d647c2d09b`；随后仅追加报告。执行锁、40 mm 间隙、腕部限制、默认 PYTHON 和所有优化开关保持不变。

原 Windows 工作区及既有 GPU checkout 均未覆盖。本轮使用独立 worktree 和服务器已有 Python 3.10 / NumPy 1.26.4 / SciPy 1.14.1、Ubuntu 22.04 Humble/MoveIt 环境，没有升级依赖。仅执行 fake IO、保存输入和原生测试进程；没有连接机械臂、相机或真空，没有运动、阀命令或新现场采集。

## 本轮代码修改

`b64ed90` 修复 R2-03 的另一条返回阻塞路径。此前主调用者已经超时，仍调用 `Thread.start()` 启动截止报告线程；`TaskManager.prepare_plan` 的拒绝记录和 finally 计时也会启动新线程。线程启动本身等待底层线程及 GIL 调度，并不等于无等待。

实际 `run_task` 注入证据线程启动延迟 250 ms、预算 100 ms，两个用例均超过原 150 ms 门槛；实际 `TaskManager.prepare_plan` 同时开启计时，约 603 ms 才返回。现在截止报告复用任务线程，在回滚和资源清理后写入；拒绝记录和计时复用任务已经启动的 journal 线程。记录分别保留 `PENDING_TASK_CLEANUP` / `WRITTEN` / `FAILED`、写入完成时间和错误。journal 在接收写入后失效时，排队回执也结束并暴露错误。没有先发布未落盘计划，没有改预算或许可事务。

新增 `tests/test_review2_deadline_record.py` 覆盖慢线程启动、拒绝与计时的完整返回、写盘失败、journal 关闭和接收后死亡。定向启动门/计时组 **22 passed**，其中本轮新增 5 项。此前的 167.733 ms 返回失败依然保留；这次补上一个确定性阻塞来源，不能据此断言旧失败已被完全解释或系统已获得硬实时保证。

`f1d2a98` 修复 `tools/check_native_prepare.py` 的验收报告和清理：数值或判定不一致时退出码为 1；Python prepared 而 native rejected 时，报告保留判定分歧，不再因缺少 `max_abs` 崩溃；没有可比较数组时数值统计为 null；worker 由 finally 清理。每个案例记录实际 time_scale、period 和 reach，报告记录 Python/NumPy/SciPy 版本。原 `1e-12`、Python 参考插值和 C++ 数值算法均未修改。

`8041a3c` 补充只读系数诊断和验收工具回归。工具回归 4 项通过，包括数值失败、判定分歧、正常控制和 worker 异常清理。保留中间失败：首次修复后 3 个测试错误地要求所有构造案例的 scale≥1，随后改为明确断言故意非法的 timing 案例仍保存 0.5；未删除案例、修改业务判据或比较阈值。

## 验证与数值归因

最终结果及所有命令、退出码、耗时、日志哈希、输入和构建身份见 [report.json](evidence/realtime_20261118_review_fixes/report.json) 与 [verification.json](evidence/realtime_20261118_review_fixes/verification.json)。

固定 SHA 上的完整定向集：**663 passed / 2 failed / 0 errors / 0 skipped**，499.34 s。本轮新增 9 项回归全部通过。两项失败均保留：

- `test_exact_prior_face_keeps_a_bounded_envelope_that_covers_the_prior`：既有 GPU 几何失败，尺寸仍为 `[0.14007188, 0.18569301, 0.108]`；未修改几何或断言。
- `test_unanswered_request_is_abandoned_at_the_deadline_and_never_reused`：该次到期仍处于 `SCREENING`，13 份请求记录只有 IK/validate，没有进入测试注入的阻塞 plan。调用者在截止后约 0.228 ms 返回，清理完成且无错误；失败为 abandoned 为空，不能据此断言已经发生无响应 worker 的隔离遗漏。相同 SHA、原用例及原门槛单独复测 **1 passed**，落盘记录确认进入 contact plan 的等待并留下 REQUEST_FAILED。复测不替换完整组的失败。

原生 `optimized/reference/two_level/adaptive`、真实头文件 `limits/journal/path`、四种实际取消消息格式（并验证错误/重复/迟到 ID 和随后 reset）、单箱组件及 output-check deferral 范围均通过。单箱组件为合成 491 点、2.45 s 轨迹，未签发实机许可；它不代表四段循环或物理验收。worker 从本轮原生源码重新编译，其文件与 `8041a3c` 的相关源码一致。

两份保存现场帧各 32 个候选的端点预检，在当前源码与本轮新建 worker 上完成 PYTHON/NATIVE 比较：

| 保存记录 | 场景物体数 | 两侧相同的 REJECTED / CLEAR / NOT_CONCLUSIVE 数 | 回放已消费 / 记录调用数 |
|---|---:|---:|---:|
| 313f32ad | 40 | 13 / 12 / 7 | 2412 / 2412 |
| 6966ce4b | 38 | 11 / 11 / 10 | 2405 / 2405 |

两份均 `all_identical=true`、`unused=0`、`source_record_unchanged=true`、`permits_issued=0`。这是当前环境中同一输入的端点判定等价，不是跨环境调用数一致，也不是完整四段路径、20 条新现场样本或物理验收。命令、耗时、输入哈希和原始报告哈希见 [field-verification.json](evidence/realtime_20261118_review_fixes/field-verification.json)。首次运行将输出放在 chroot 外部，原生进程不可见该目录，因 `ATTEMPT_JOURNAL_OPEN_FAILED` 拒绝；这两次失败保留在 [field-first-attempt.json](evidence/realtime_20261118_review_fixes/field-first-attempt.json)。改用共享 checkout 内的新输出目录后完成上述比较，未绕过 journal 门槛。

数值比较继续使用原保存输入：27 条唯一轨迹 / 193 案例，160 一致，33 个 prepared 数值未过，最大加速度差 `1.8383592925719228e-8`；旧服务器组件 11 案例为 8 一致、3 未过，最大加速度差 `9.315828276854907e-12`。两次 CLI 均保留报告并正确返回 1，没有以进程正常结束宣称验收通过。

本轮在相同保存轨迹上比较实际 native 系数和原 SciPy Bernstein→power 系数。额外的诊断副本仅替换数组幂为标量幂，检查它能否重现 C++ 系数；生产 Python 参考保持原字节。数组幂与标量幂的舍入差经过短末段和 Bernstein 转换中的相消，可放大为加速度差。诊断不算数值等价通过，也不把某个环境的通过覆盖另一环境的失败。

最大差对应 scale=1.5、末区间约 1.58429 ms；幂系数最大差 0.0006103515625。诊断副本与实际 C++ 系数逐值差为 0；仅将这些系数代入同一 SciPy 求值器，所得 q/v/a 差即完整重现 worker 报告的最大差。证据见 [rounding.json](evidence/realtime_20261118_review_fixes/rounding.json)。这定位了该样例的来源，没有将更换 Python 参考实现当作修复。

```bash
.venv/bin/python tools/check_native_prepare.py \
  --records SAVED_TRAJECTORIES --output NEW_COMPARISON.json
# 比较失败仍写报告，并返回 1。后续诊断需单独运行，不能用 && 隐去失败。
.venv/bin/python docs/evidence/realtime_20261118_review_fixes/diagnose_rounding.py \
  --comparison NEW_COMPARISON.json --output NEW_DIAGNOSTIC_DIRECTORY
```

## 保留的验收边界

前 7 步整体验收仍为 false；数值门槛、稳定返回时限、完整调度、新至少 20 条现场输入、完整 prepare_plan→PLAN_READY p95≤2 s 和物理验收分别列示。旧现场记录可用与新现场样本未完成同时成立。没有执行完整实机循环，没有把原生组件或保存端点预检当作物理验收。

本轮未开始碰撞工作区缓存。下一阶段仍应单独提交：每 worker 的完整上下文键包含模型及代理来源哈希、场景版本、附件、阶段接触权限、pair policy、裕量和清除模式；任一身份变化或 reset 均使相关状态失效。以相同保存输入逐请求比较 EXACT 决策、碰撞对及失败原因，并测量构造次数 N→K、构造耗时、内存和取消清理。预计减少 N−K 次重复构造，实际 N/K 必须测量，不将历史 132 次直接当成本轮收益。
