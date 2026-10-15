# 固定关系窄阶段复用补修（2026-10-15）

已修复并在授权 Linux 服务器完成 Release 构建、定向测试和唯一一次三组交替 A/B。固定 transit 片段检查中位数 **0.176564 → 0.153827 s（减少 12.88%）**；空载 pregrasp 检查中位数 **增加 0.004611 ms（0.042%）**。保留第 3 组 transit 退化，不追加实验，不外推五箱成绩。

审查及实际起点均为 `d33eabdab489a504a5d548abe79e7355aa4b79eb`，本地目标分支干净、远端无增量。实测源码：`8df907ec68ccb7929d8e91e422bf02f99ba66930`。开发、编译、测试在独立目录 `/work-native-narrow-20261015` 完成；旧工作区、输入及证据保留。Windows 仅收取并发布。

## 实际 FCL 复现与修复

服务器安装 `libfcl-dev/libfcl0.7 0.7.0-3`，worker 和基准实际链接 `libfcl.so.0.7`。已核对安装头文件：`DistanceResult::update()` 只向更小值更新，box–box 的 `ShapeShapeDistance` 返回 `result.min_distance`。见[源码摘录](experiments/m710_native_narrow_reuse_20261015/fcl-source.txt)、[版本/哈希及复现值](experiments/m710_native_narrow_reuse_20261015/reproduction.json)。

真实 FCL 查询使用共同旋转 45° 的两个 1×0.01×0.01 m box，局部横向中心间距 0.05 m：

| 查询 | 结果初值(m) | 返回值(m) | 查询后 min_distance(m) | 旧资格条件 |
|---|---:|---:|---:|---|
| 原接受查询 | 0.006 | 0.006 | 0.006 | false |
| 不截断的补充查询 | 1.7976931348623157e308 | 0.040000000000000015 | 0.040000000000000015 | 不适用 |

AABB 距离下界为 **0**。补修保留原接受/拒绝阈值、原失败见证和 AABB 资格；只对既有“同法兰固定工具 box–附着 box”资格集合，在首次成功窄查询后尝试一次未截断补充查询。距离返回值、结果、几何身份和截断关系必须一致，且严格大于原阈值 + 1 mm，才建立窄阶段来源资格。边界内直接记录 `WITHIN_REUSE_GUARD`；未知值、不一致或补充异常各记原因、不缓存成功，后续继续原检查。每对最多一次资格尝试，补充调用及首次成本计入当前 check / narrow_calls。模型、附着身份、活动关节、变换约束和候选局部生命周期不变。

## 独立回归与判定

新增 `m710_narrow_reuse_test` 的工作区只有 **1 个 eligible pair**，工具和附着箱真实绑定同一法兰。第一次明确断言 AABB skips=0，确实做 FCL；接着断言 narrow 资格=1、AABB 资格=0、补充查询=1，三个合法后续 q 均从 narrow 来源复用。四次判定参考/优化均有效：旧实现及诊断基线共 **4 次窄查询、0 次复用**；修复后共 **2 次窄查询、3 次窄来源复用**。首次额外查询完整计入，不能将 3 次命中说成净省 3 次 FCL。

先编译旧 d33eabd 头文件和[兼容旧接口的回归源](experiments/m710_native_narrow_reuse_20261015/old-code-reproducer.cpp)，实际在 `NARROW_QUALIFICATION_NOT_REUSED` 断言失败，见[旧代码日志](experiments/m710_native_narrow_reuse_20261015/old-test.log)。修复后同一核心断言通过；新增测试还覆盖 24 个窄阶段边界差分、环境/邻箱碰撞、活动工具关节、重新附着/变换变化、释放、模型及策略/ACM 变化。旧固定关系测试的 32 个边界差分继续通过，并明确断言 AABB 来源资格/复用保留、无补充查询。

通过的原生测试：`m710_narrow_reuse_test`、`m710_fixed_reuse_test`、`m710_clearance_test optimized reuse`；另有 3 项父身份/上下文/工艺协议回归通过，46 项未选。见[测试命令](experiments/m710_native_narrow_reuse_20261015/test-command.sh)及同目录原始日志。此处 PASS 只覆盖这些定向检查。

## 同输入有限对照

复用[上一轮冻结输入](experiments/m710_native_reuse_20261015/)，transit/pregrasp 都为前三条边，分别 359/27 状态。两份输入原字节 SHA256、14 个官方 mesh、依赖锁全部一致；几何、样本、访问顺序、日志和编译选项相同。CPU 为 Xeon Gold 6459C，Ubuntu 22.04 / ROS 2 Humble，Release `-O3 -DNDEBUG`，固定 CPU 0，OMP/OPENBLAS=1。

采用同一二进制的最小诊断模式：A=`narrow-baseline` 仅令 `qualify_fixed_narrow=false`，B=`optimized` 开启补修。两侧 `native_check_reuse=true`，上一轮 ProcessSealInput、预解析和 AABB 优化全开；计数/计时代码一致。A 重现 d33eabd 的窄阶段无资格行为，不把上轮 5.75% 再计入。本开关仅由 C++ 检查基准使用，worker 协议和正常默认配置未改。每次独立进程/冷候选，无资格预热，首次建立成本包含在 check，setup 为场景和检查器准备；模型加载另记。嵌套分项不再次相加。

| 组/实际顺序 | 模式 | transit 准备 | 检查 | 合计 | pregrasp 准备 | 检查 | 合计 |
|---|---|---:|---:|---:|---:|---:|---:|
| 1 / 1 | A | 0.062374 | 0.176749 | 0.239123 | 0.013585 | 0.010963 | 0.024548 |
| 1 / 2 | B | 0.061776 | 0.153785 | 0.215561 | 0.015521 | 0.011373 | 0.026894 |
| 2 / 3 | B | 0.061535 | 0.153827 | 0.215361 | 0.013523 | 0.010962 | 0.024485 |
| 2 / 4 | A | 0.060946 | 0.174676 | 0.235622 | 0.013729 | 0.012401 | 0.026129 |
| 3 / 5 | A | 0.060257 | 0.176564 | 0.236820 | 0.013706 | 0.010978 | 0.024684 |
| 3 / 6 | B | 0.073060 | 0.179044 | 0.252104 | 0.013659 | 0.010983 | 0.024641 |

每次合计=准备+检查；表中为秒，原精度见 [CSV](experiments/m710_native_narrow_reuse_20261015/timing.csv)。三次中位数：

| 片段 | A 准备 | B 准备 | A 检查 | B 检查 | A 合计 | B 合计 |
|---|---:|---:|---:|---:|---:|---:|
| transit | 0.060946 | 0.061776 | 0.176564 | 0.153827 | 0.236820 | 0.215561 |
| pregrasp | 0.013706 | 0.013659 | 0.010978 | 0.010983 | 0.024684 | 0.024641 |

transit 合计中位数减少 8.98%；第 3 组 B 检查反而增加约 1.40%、合计增加 6.45%。空载没有新增命中，检查中位数略退化；首组空载准备/检查也增加。共享服务器上的三组小样本不证明稳定实时收益。

每次 transit 的来源计数均如下（空载两侧资格/复用/补充均为 0，窄查询均为 486）：

| 计数 | A | B |
|---|---:|---:|
| AABB 来源资格建立 | 9 | 8 |
| narrow 来源资格建立 | 0 | 122 |
| 一次性补充查询 | 0 | 122 |
| AABB 来源复用 | 3,033 | 2,864 |
| narrow 来源复用 | 0 | 43,676 |
| 实际 FCL 窄查询总数（含补充） | 52,505 | 9,121 |
| 其中 world 窄查询 | 3,231 | 3,231 |

B 提前从窄阶段建立一个原来稍后才由 AABB 建立的资格，因此来源分布改变；AABB 分支保留。实际窄查询净减少 **43,384**，不以命中数代替。动态对象对检查均为 2,905,387，密封几何求值均为 359，压缩 SAT 均为 25,848。两侧全部 386 个状态的判定、首个失败见证及最终 seal 完全一致，原始逐状态结果保留在六份 gzip 中。[机器结果](experiments/m710_native_narrow_reuse_20261015/formal/comparison.json)。

[构建命令](experiments/m710_native_narrow_reuse_20261015/build-command.sh)、[实际基准命令](experiments/m710_native_narrow_reuse_20261015/benchmark-command.sh)、[源码/二进制/输入身份](experiments/m710_native_narrow_reuse_20261015/freeze.json)、[实际环境](experiments/m710_native_narrow_reuse_20261015/environment.json)。复现时将旧目录 `model.json.gz` 和 `fixed-input.json.gz` 用 `gzip -dc` 解压到新目录对应文件；既有 mesh URI 不改。基准子进程上限 120 s，最多三组，无自动重试。旧缺陷复现是在 d33eabd 独立检出中将上述 reproducer 置为测试源、添加同样的 MoveIt CMake target 后运行；无需运行正式基准。

结果状态为 `CHECKER_BENCHMARK_NOT_EXECUTABLE`，执行资格 `NOT_EVALUATED`，`continuous_swept_proof=false`。未运行新五箱、40箱、历史大任务集、全量 pytest、Isaac、真机或重型执行验收；没有新的五箱耗时。未改宽阶段索引、候选/IK/先验/求解器、采样、碰撞权限、Python 独立验证、时间参数化或执行合同。本轮到此停止。
