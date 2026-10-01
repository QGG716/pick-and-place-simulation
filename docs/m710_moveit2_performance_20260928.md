# M-710 原生净空查询优化，2026-09-28

评审基线 `baeb2a7056e8af0397855e9f54c0d9d421472e6d`；仅开发和推送 `feat/v0.5-backend-moveit2-mtc-pilz`。开始时本地和远端均为该提交，工作区干净。9 月 26 日连接被拒绝，9 月 28 日同端口恢复后继续；没有回滚，也没有覆盖两轮历史证据。

本轮结果及逐项数据见下方最终测量表。全部最终验证来自同一 Release worker，reference 和 optimized 在同一二进制中可选，串行运行。人工几何只用于实现验证，不进入真实任务成功率。

## 规则与实现

新增 `clearance_workspace.h`，是现有 MoveIt FCL 环境的小型适配器，不是新的碰撞库或规划器。它通过 `CollisionEnvFCL` 复制构造复用未膨胀几何/BVH，调用 MoveIt 的 `constructFCLObjectRobot` 一次建立候选私有 FCL 对象，并保留独立世界快照。每次检查更新当前状态的碰撞体变换和 AABB，不在 self/world 两个距离入口分别重建机器人对象。窄相仍使用 FCL 默认距离请求及求解器。

对象对只预处理一次：same-object、ACM ALWAYS、精确 attached touch-links；普通机器人 self 为零额外净空，robot/tool 仍为 5mm，transit payload/conveyor 使用原有接收预留量。工具固定 link 不会因此被当成普通 self 豁免；柔性杯许可不扩展到刚性件。不同候选、世界、ACM、阶段、策略建立新工作区；每次查询还核对模型、附着体形状、所属 link、touch-links 和精确局部变换。不缓存近似关节状态或有效性结果。候选检查由 mutex 串行保护，不共享可变状态/管理器。

广相遍历所有未豁免对象对，用 FCL AABB 表面距离下界排除远距离组合。每对查询带为其实际阈值加 1mm，只有下界严格超出查询带才跳过；这只是查询范围，不是几何膨胀、接收阈值或数值容差。近距离对全部做实际 FCL 窄相。接触仍拒绝，净空比较仍为 `distance + 1e-9 < required`。缺失几何、无效变换/AABB、未知距离、上下文变化均失败关闭。

`Clearance` 的 predicate 继续接入 OMPL 搜索；MoveIt 原始相交检查保持开启，shortcut/简化后仍按原分辨率检查最终返回路径，Python `_path_failure` 和协议/轨迹时间合同保持不变。没有改动 202 个工具碰撞体、40 箱、20kg 工具、42.5kg 载荷、固定端点、TCP、附着变换、range 或 longest_valid_segment_fraction。没有声称离散保守采样等于严格连续扫掠证明。

保留 `M710_CLEARANCE_MODE=reference`，执行原来的 MoveIt `distanceSelf`/`distanceRobot` SINGLE 查询和原判定；默认 `optimized`。reference 用于同输入差异及性能对照，不是另一套生产规划器。计数改为原生结构，结束时生成 JSON；保留真实失败次数、首个非相交净空失败搜索见证、最多八条含原始 q 的失败样例以及诊断模式逐状态失败信息。

## 剖析口径与可观测性

先用原实现做小规模实测，再实施优化。固定真实集合共 166 次检查、164 个唯一状态：两个原端点、三个原始失败 q、各历史失败边的九个固定点、冻结 transit 路径点、起点各轴固定偏移。另有双模式人工阈值邻域、机器人 self/tool 分类、柔性杯/刚性区别、端点合格而边内失败、payload/conveyor 预留量和阶段切换等测试。历史失败边另用完整原采样复检，不以集合中的九点替代。

初次 reference 剖析：self API 0.887074s，world API 0.747898s，RobotState 复制/变换 0.003253s；166 次布尔判定与权威一致。因此优化距离查询的重复准备和远距离对象对，而不是改变有效空间或扩大搜索预算。

最终数据区分：状态检查次数、固定集合唯一状态数、广相对象形状对测试数、跳过数、实际 FCL 窄相调用数、有效/拒绝状态数。查询次数不是 OMPL 树节点数；没有取得 OMPL 树节点/运动边验证器调用计数，不作推算。同一逻辑对象可能有多个形状，窄相计数是实际形状对调用。

reference 的 self/world API 内部构造、管理器、过滤和窄相仍为锁定共享库中的整体耗时，未独立插桩，内部调用数标记为不可得，不用 optimized 数量代替。优化版本分别计量候选对象准备、关系过滤、逐状态变换/AABB、广相、self/world 窄相。固定集合另测保留的相交检查；OMPL 内部相交耗时未拆出。统计/见证维护及证据对象生成分别记录；JSON wire 编码与 IPC 没有独立分离，只有 `transport_and_worker_s`，不将其差值冒充纯序列化时间。

计时存在嵌套：窄相 self/world 包含于窄相总时间，原生检查包含于规划/输出边检查，见证包含于统计维护，所有这些又包含于端到端。不得相加为新的总耗时。固定集合平均值/吞吐仅描述本次固定集合，不是多次规划成功率、P95 或系统吞吐保证。

## 最终测量

结论：同规则下原生查询开销实际降低，最终版本的真实 `loaded_fixed_transit` 在第一次 OMPL 调用中完整通过，未运行 120s diagnostic。原始 reference 三次 12s 均无解；该结果不是不可达证明。没有 `NATIVE_AUTHORITY_MISMATCH`。

| 固定真实集合（166 次，164 个唯一状态） | reference | optimized |
|---|---:|---:|
| 原生检查总耗时 s | 1.669698 | 0.103712 |
| 平均 ms/次 | 10.058423 | 0.624770 |
| 本集合状态吞吐 次/s | 99.419 | 1600.588 |
| 有效 / 拒绝 | 145 / 21 | 145 / 21 |
| 另测原相交检查 s | 0.135149 | 0.130274 |

reference 分项：RobotState 复制/更新 0.003315s；self API 0.906080s；world API 0.759889s。
optimized 分项：候选工作区整体建立 0.003390s，其中对象准备段 0.000110s、关系过滤 0.003180s；一次建立 257 个 FCL 对象，8223 个未豁免形状对，23592 个豁免形状对。逐状态复制/更新共 0.003069s；变换/AABB 更新 0.001109s；广相 0.037622s；实际 self 窄相 0.029412s、world 窄相 0.001718s。
广相实际测试 1,364,780 次，安全跳过 1,331,205 次；实际窄相 33,575 次（self 32,307，world 1,268）。统计/见证维护：reference 0.000057948s，optimized 0.000046791s；证据 JSON 对象生成分别 0.000690434s / 0.000499061s。
固定集合的 reference / optimized / Python 三方布尔判定完全一致；两种原生模式的失败对象对和阈值一致，正距离失败见证的最大原生距离差为 1.7780915628762273e-17m。三个历史状态及完整失败边在两模式均拒绝，两个原端点仍合法。人工阈值和隔离回归全部通过。

| 真实带载调用（最终版本，串行） | 规划 s | 原生搜索查询 / 拒绝 | 原生输出检查 s | Python 完整边 s |
|---|---:|---:|---:|---:|
| reference 1: PTP / SEARCH_EXHAUSTED | 0.838169 | 75 / 42 | 未返回路径 | 未运行 |
| reference 2: RRTConnectkConfigDefault / SEARCH_EXHAUSTED | 12.020550 | 1157 / 601 | 未返回路径 | 未运行 |
| reference 3: RRTConnectkConfigDefault / SEARCH_EXHAUSTED | 12.007554 | 1201 / 785 | 未返回路径 | 未运行 |
| reference 4: RRTConnectkConfigDefault / SEARCH_EXHAUSTED | 12.055323 | 1195 / 699 | 未返回路径 | 未运行 |
| optimized 1: PTP / SEARCH_EXHAUSTED | 0.109371 | 75 / 42 | 未返回路径 | 未运行 |
| optimized 2: RRTConnectkConfigDefault / SUCCESS | 7.702997 | 9476 / 6271 | 10.012526 | 768.150517 |

最终带载段 114 个路径点；成功 OMPL 的净空检查累计 4.821271s（嵌套于规划），有效 3205、拒绝 6271，其中正距离净空拒绝 863。搜索中广相 62,384,055、跳过 60,911,591、实际窄相 1,472,464；`search_gap_witness` 保存了原相交规则允许而净空不足的真实搜索 q。输出密集检查 17,091 次全部有效。
reference 带载段端到端 37.213993s（含初始化的脚本总时长 38.195965s）；optimized 段端到端 786.123147s（脚本总时长 787.059515s）。两者输出成果不同，不能把这些端到端时间相除声称规划速度倍数。成功版本的大部分验收耗时转移到了原 Python 全边检查，未对其做优化或裁减。
真实路径 CPU 时间合同另测 0.292520s，复用现有执行配置的有限 velocity/acceleration/jerk、原 C2 时间律和 native edge-duration floor；`within_limits=true`，关节路径未改变，原生时间下界保留。此项不是物理执行，也不是完整卸货执行 bundle 验收。审计精确核对累加前的下界操作与累加后的相同时间戳，不增加容差；累计时间戳相减得到的最小下界差为 -1.7763568394002505e-15s，原始浮点舍入值保留在证据中。

| 防回归 | 原生规划 s | 原生输出边 s | Python 权威 s | 段端到端 s |
|---|---:|---:|---:|---:|
| empty_short PASS | 0.010083 | 0.018861 | 0.388684 | 0.509126 |
| linear_fixed_orientation PASS | 0.007765 | 0.017091 | 0.376581 | 0.458218 |

TCP 能力退出：候选 0.004126s，脚本总时长 0.930472s；原生规划调用 0，重型状态/边检查 0。定向 pytest：13 passed（0.23s），见 `pytest.log`。

剩余限制：只证明这一固定集合与一个真实带载段，不声称总体成功率提升；Python 完整权威复检仍慢；reference 共享库内部构造/过滤/窄相没有独立插桩计数，OMPL 树节点和边验证器计数也未取得。未实现旋转 TCP、完整 MTC 卸货、CIRC、轨迹混合、在线框架或 Isaac。

首条完整通过的开发期 90 点路径保留在 `pre-identity-normalization/first-accepted-loaded.json`，对应旧 SRDF 集合序列化顺序，绝不冒充最终版本复跑。最终数据来自当前根目录证据（服务器 `final2`）；只排序许可输出以稳定身份，未修改许可内容。全部修改运行源码与构建输入逐字节一致；未改动 fixture 的历史 CRLF/LF 映射列于源码清单，原始数字文本和 JSON 值一致。

最终 worker SHA256：`130a07733cac90a6191ede0f349016d995b775b35be188395a92bf1ec971c46a`。

最终固定集合中，16 个同对象对正距离失败状态的优化原生/权威距离最大差为 3.7816971776294395e-16m。Python 实际完整路径检查调用及采样计数见 `loaded-optimized.json` 的 `authority_statistics`，没有用它冒充 OMPL 边验证器计数。

## 复现与证据

证据目录：`docs/validation/evidence/m710_moveit2_perf_20260928/`。

- `source-manifest.json`、`binaries.sha256`、`flags.make`、`packages.lock`、`environment.txt`：实际源码、最终二进制、Release 编译选项和依赖/运行环境。
- `fixed-{reference,optimized}.json`：原样请求、全部固定 q、标签、唯一状态数、每状态布尔/失败/耗时、Python 权威结果。
- `historical-{reference,optimized}.json`：三个历史失败状态、完整失败边、原端点、端点直接拒绝与场景/附着切换。
- `synthetic-{reference,optimized}.json`：人工几何双模式规则验证。
- `loaded-{reference,optimized}.json`、对应 requests JSONL 和 worker 日志：同预算真实带载段及常驻调用顺序；每次独立 worker seed=71070，loaded request seed=71072。
- `first-accepted-loaded.json`：以 exclusive-create 保存第一条完整通过路径，不能被后续失败覆盖。
- `small.json`、`capability.json`、`pytest.log`：小范围 PTP、固定姿态 LIN、提前能力退出和定向模块测试。

完整实际串行命令保存在 `final-validation.sh`。服务器保留实验目录 `/root/autodl-tmp/m710-clearance-perf-20260928`，复用 Humble rootfs 的 `/work-round3/build-perf`，未修改以前的 `/work` 或 `/work-round2`。

```bash
export M710_MOVEIT_COMMAND=/root/autodl-tmp/m710-clearance-perf-20260928/worker.sh
export M710_MOVEIT_ASSET_ROOT=/work-round3
# 在实验 repo 与原 CPU venv 中运行；两个模式串行，各自启动 resident worker。
M710_CLEARANCE_MODE=reference python tools/profile_m710_clearance.py --output fixed-reference.json
M710_CLEARANCE_MODE=optimized python tools/profile_m710_clearance.py --output fixed-optimized.json
M710_CLEARANCE_MODE=reference python tools/run_m710_moveit_integration.py --case loaded_fixed_transit --output loaded-reference.json
M710_CLEARANCE_MODE=optimized python tools/run_m710_moveit_integration.py --case loaded_fixed_transit --output loaded-optimized.json --preserve-first-path first-accepted-loaded.json
# 仅在优化版 3×12s 无返回路径时单独执行；不是同预算性能对比：
M710_CLEARANCE_MODE=optimized python tools/run_m710_moveit_integration.py --case loaded_fixed_transit --diagnostic-only --output diagnostic-only.json --preserve-first-path first-accepted-loaded.json
```

诊断入口明确限制为同一 loaded 段，PTP 后至多一次 120s OMPL；不扫描 seed。旧能力前置检查保持：当前适配器拒绝姿态变化的任务 TCP LIN，不把这一范围表述为 Pilz 本身不支持旋转。历史/接触工艺、杯压缩等仍依赖原权威检查，不用本轮自由空间模块扩大声明。没有全量 pytest、历史任务人口扫描、完整卸货或 Isaac 执行。

附加 CPU 时间合同复现：`python tools/check_m710_loaded_timing.py --accepted first-accepted-loaded.json --output timing-contract.json`。该脚本显式使用现有 POC 执行配置；两次审计脚本开发失败日志（历史默认配置、对累计时间戳直接相减作严格比较）单独保留，均未改动原执行器。
