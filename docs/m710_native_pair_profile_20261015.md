# 逐对象对性能计时移出正常路径（2026-10-15）

已完成代码、Release 构建、定向回归、真实 worker 选项检查及唯一一次三组交替 A/B。在相同完整固定片段上，关闭逐对象对性能计时后，transit 检查中位数 **0.155963 → 0.051669 s（减少 66.87%）**；pregrasp **0.011099 → 0.003009 s（减少 72.89%）**。这是**减少性能观测开销**，不是减少碰撞检查数量或提升 FCL 算法速度；没有新五箱结果。

本轮起点及核对的远端均为 `6410d87799fd2ff6c4a18cacd77866f230766e21`，包含上一轮 `8df907e` 的完整优化。实测源码 **`33dd0025fa4b19b73bc801ae6c8fff75ae87fb7e`**。在授权服务器独立目录 `/work-native-pair-profile-20261015` 修改/构建/运行，旧工作区的未提交日志、源码、输入和报告均保留。依赖未升级，未终止其他项目进程；Windows 仅收取及发布。

## 路径与证据语义

`pair_profile_enabled` 在建立 Clearance/工作区时解析，默认 **false**，循环内不读 JSON/环境变量。正常 worker 接收该请求选项；基准使用 `pair-profile-on/off` 两个模式。只有两处逐对象对计时改为同一个可关闭的轻量计时器，几何算法没有复制。打开时仍将一次性补充资格查询包含在同一 narrow 时间区间中。

| 时钟类别 | 本轮处理 |
|---|---|
| 每对 AABB broad、FCL narrow 性能时钟 | 显式开启才读取；关闭时不读取 |
| 每状态变换更新、Clearance 检查/记账、候选准备 | 保留 |
| 基准 setup/check/total 及进程外层墙钟 | 保留，两侧相同 |
| worker 预算、超时、取消、截止时间 | 完全保留；worker 源码仅增加构造器选项转发 |

现有 `broad_phase_s/narrow_s/self_narrow_s/world_narrow_s` 字段保留；关闭时为 **null**，工作区声明 `pair_profile_enabled=false, pair_profile_status=NOT_MEASURED`；开启时为数字及 `MEASURED`。未测量不记成零。代码检索未发现其他生产代码消费这些细项作数值运算；基准消费者新增状态/null 校验。narrow 与 self/world 为包含关系，不相加；任何细项也不加回总时间。旧窄阶段资格代码区块逐字保持一致，几何、顺序、阈值、采样、锁、状态复制和变换更新不变。

## 验证

- 复用 `m710_narrow_reuse_test`、`m710_fixed_reuse_test`、`m710_clearance_test optimized reuse`。测试专用双检查器让每个原有场景同时运行计时开/关，逐次比较判定、首个失败见证及全部工作区几何计数；包括 24+32 个边界差分、AABB/窄阶段两类资格、活动工具关节/连杆、邻箱/环境、附着变化、释放、模型及策略/ACM 失效，全部通过。
- `m710_pair_profile_test` 注入计数时钟，确认关闭 **0 次**读时钟、开启一次计时 **2 次**。正式 worker/基准没有这个计数器。源码路径测试确认对象对循环无直接 `Clock::now()`，只用此封装。
- 两项 Python 证据/路径测试通过，验证 null 与实际零的区别，并确认几何计数改变会被比较器识别。
- [真实 worker 探针](experiments/m710_native_pair_profile_20261015/worker-probe-summary.json)：仅 validate 两次相同的归档 transit 状态，显式 true 与省略选项的默认关闭均使用真实 FCL，判定和计数一致。没有规划或执行请求。首次开发探针用了冻结模型初始化时的旧 scene/policy_scope 标签，worker 正确返回 UNKNOWN_MODEL_TCP_CONTEXT；同时探针把 READY 写成了 SUCCESS 断言。仅修正探针初始化标签绑定及响应断言后通过，几何未改。首次脚本/响应/日志以 worker-probe-initial 保存，不计入正式对照。

[原生测试命令](experiments/m710_native_pair_profile_20261015/test-command.sh)、[Python 测试命令](experiments/m710_native_pair_profile_20261015/python-test-command.sh)及同目录日志保留。测试 PASS 只描述上述范围，不是执行资格通过。

## 唯一一次有限对照

同一二进制，A=详细计时开启，B=关闭。两侧 ProcessSealInput、输入预解析、AABB 和窄阶段固定复用全部开启。使用上一轮引用的完整冻结模型和前三条边，**359/27 状态**，输入字节、14 个官方 mesh 及依赖锁一致。Xeon Gold 6459C，Ubuntu 22.04 / ROS 2 Humble / FCL 0.7.0-3，Release -O3 -DNDEBUG；CPU 0，OMP/OPENBLAS=1；独立进程/候选上下文，A/B、B/A、A/B，无判定预热。模型加载单列；准备包含场景/检查器建立；首次资格计算仍包含在检查内。内层剖析选项是唯一控制变量，外层计时、日志、输出结构和基准记账相同。

| 组 / 实际顺序 | 模式 | transit 准备 | 检查 | 合计 | pregrasp 准备 | 检查 | 合计 |
|---|---|---:|---:|---:|---:|---:|---:|
| 1 / 1 | A | 0.062508 | 0.156063 | 0.218571 | 0.013686 | 0.011097 | 0.024783 |
| 1 / 2 | B | 0.060513 | 0.050501 | 0.111014 | 0.013541 | 0.003018 | 0.016558 |
| 2 / 3 | B | 0.060921 | 0.052638 | 0.113558 | 0.013484 | 0.002970 | 0.016454 |
| 2 / 4 | A | 0.061777 | 0.155963 | 0.217740 | 0.013600 | 0.011099 | 0.024699 |
| 3 / 5 | A | 0.061898 | 0.155456 | 0.217353 | 0.013524 | 0.012946 | 0.026470 |
| 3 / 6 | B | 0.060665 | 0.051669 | 0.112334 | 0.014118 | 0.003009 | 0.017128 |

单位为秒。每次合计=准备+检查；原始精度、全部计数见 [CSV](experiments/m710_native_pair_profile_20261015/timing.csv)，全部响应保留为六份 gzip。[机器结果及中位数](experiments/m710_native_pair_profile_20261015/formal/comparison.json)。

| 片段 | A 检查中位数 | B 检查中位数 | A 合计中位数 | B 合计中位数 |
|---|---:|---:|---:|---:|
| transit | 0.155963 | 0.051669 | 0.217740 | 0.112334 |
| pregrasp | 0.011099 | 0.003009 | 0.024783 | 0.016558 |

合计中位数分别减少 48.41% / 33.19%。三组检查均缩短，但不筛掉波动：第 3 组 B 空载准备 **0.014118 s**，比 A 的 **0.013524 s** 增加约 **0.594 ms（4.40%）**；工艺检查 transit 分项中位数也从 0.005767 s 略增至 0.005819 s。没有因波动追加试验。三组小样本不是稳定实时或 p99 保证。

下列每一项在所有三组 A/B 中均完全相同，比较器对所有非计时工作区计数进行精确校验：

| 项目 | transit：A = B | pregrasp：A = B |
|---|---:|---:|
| 实际状态数 | 359 | 27 |
| 遍历对象对数（含固定复用） | 2,952,057 | 222,696 |
| 实际 AABB 检查／对象对计算 | 2,905,517 | 222,696 |
| AABB 跳过 | 2,896,518 | 222,210 |
| 实际 FCL 窄查询（含补充） | 9,121 | 486 |
| 一次性补充查询 | 122 | 0 |
| AABB 来源资格／复用 | 8 / 2,864 | 0 / 0 |
| 窄阶段来源资格／复用 | 122 / 43,676 | 0 / 0 |
| 动态对象对检查 | 2,905,387 | 222,696 |
| self / world 窄查询 | 5,890 / 3,231 | 216 / 270 |

两侧全部采样状态有效，完整判定、失败见证及最终 seal 一致。密封几何求值仍为 359，压缩 SAT 仍为 25,848。计时关闭时细项为 null，报告没有用其计算所谓“宽/窄阶段 100% 加速”。

## 复现与范围

[源码、二进制和输入哈希](experiments/m710_native_pair_profile_20261015/freeze.json)、[依赖/资产身份](experiments/m710_native_pair_profile_20261015/environment.json)、[构建](experiments/m710_native_pair_profile_20261015/build-command.sh)、[实际基准命令](experiments/m710_native_pair_profile_20261015/benchmark-command.sh)。在既有 rootfs 内运行；将 `docs/experiments/m710_native_reuse_20261015/{model,fixed-input}.json.gz` 原字节解压至新证据目录对应 JSON，保持既有 mesh URI。基准 `--pair-profile-only --pairs 3 --cpu 0`，每个子进程上限 120 s，无自动重试；复现须另选输出目录，不能覆盖本轮 formal。

结果为 `CHECKER_BENCHMARK_NOT_EXECUTABLE`，`qualification_status=NOT_EVALUATED`，`continuous_swept_proof=false`。未运行新五箱、40箱、历史大任务集、全量 pytest、Isaac、真机或重型执行资格验收。未改空间索引、候选/IK/先验/规划策略、Python 独立验证、时间参数化或执行合同。本轮停止，不推算新五箱耗时。
