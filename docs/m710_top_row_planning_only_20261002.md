# 顶排五箱连续 planning-only 实验（2026-10-02）

**5/5 完整轨迹已生成，纯轨迹规划合计 846.982063 秒。** 这是一次离线规划测量，结果标记为 `PLANNING_ONLY_NOT_EXECUTABLE`；没有执行卸货，也没有完整执行资格结论。

| 顺序 | 箱体 | 完整轨迹 | 本箱规划秒数 | 累计秒数 | 实际调用 IK / PTP / LIN / OMPL（含失败尝试） |
|---|---|---|---:|---:|---|
| 1 | carton_l07_c02 | 已生成 | 12.218554 | 12.218554 | 11 / 5 / 5 / 1 |
| 2 | carton_l07_c01 | 已生成 | 37.679780 | 49.898333 | 69 / 20 / 11 / 4 |
| 3 | carton_l07_c03 | 已生成 | 60.808557 | 110.706890 | 43 / 20 / 7 / 2 |
| 4 | carton_l07_c00 | 已生成 | 539.178130 | 649.885020 | 263 / 77 / 39 / 19 |
| 5 | carton_l07_c04 | 已生成 | 197.097043 | 846.982063 | 90 / 42 / 18 / 8 |

计时从每箱候选生成前开始，到完整抓放及撤离轨迹返回内存为止，包含 IK、失败候选、MTC、Pilz、OMPL、必要内部检查、IPC 和拼接。初始化/模型及静态场景预加载 **0.851408 秒**；第一箱任务场景准备 **0.077174 秒**，四次箱间场景更新 **0.331702 秒**，均不计入表中。批次墙钟 **847.392963 秒**，在写结果文件前结束。原生求解分项 **806.487657 秒**，原生 IK 内核 **0.300526 秒**、含 IPC 的 IK **14.064612 秒**，均为嵌套分项，不能再加到合计上。第四箱用了 19 次 OMPL，并发生 18 次 place 求解失败后回退；没有删除其耗时或另跑一批挑选数字。

使用 `m710id70_unloading_layout_v1`、完整官方 FANUC M-710iD/70 模型、完整工具几何、20 kg 工具和 42.5 kg 箱体配置。支撑关系及几何排序确认顶排五箱中心 Z 均为 **2.25 m**，其他箱中心最高 **1.95 m**。沿用最高行、固定行中心和当前接近代价排序。**离线规划基准，初始机器人静止，场景采用配置几何，非本次物理实测状态。** 首箱使用配置 q，后续四个起点逐一等于上一箱实际规划末点；每箱使用新 Task，共用一个常驻 worker。各箱开始时剩余箱数为 40、39、38、37、36；接触后附着，释放后在放置位置保留目标参与撤离，撤离完成后才按理想送出假设从下一任务移除。无落带、输送等待或邻箱物理位移仿真。

独立入口 [plan_m710_top_row_only.py](../tools/plan_m710_top_row_only.py) 复用现有候选和原生链。实验子类把“原生已生成”记录与正常模式的 `native_verified` 分开，保留显式父阶段、Task、目标和实际末态。worker 在初始化时固定模式，实验模式禁止 `task_audit`、`compose` 和独立 `validate`。正常入口默认值和执行门槛保持原状。

跳过 Python 独立状态/密集边复检、worker 求解后的额外密集输出复检、完整路径复检、独立精细封口/压缩/接触生命周期审查、逐采样 TCP/LIN 审计、全路径来源覆盖扫描、preflight、执行导出/加载、动力学/力矩/jerk 资格及执行时间律转换。保留候选和杯 mask 几何生成、规划器内部关节限位/碰撞/既有阶段接触规则、目标所有权转换、返回值及连接检查、原生时间参数化。实际记录 **1,037,193 次原生 clearance 查询**；额外密集输出查询、ordered process 输出查询、Python 独立路径检查及已验证阶段记录均为 **0**。跳过项为 `SKIPPED_PLANNING_ONLY`，执行资格为 `NOT_EVALUATED`。历史输入、旧生成器调用、Isaac 启动及执行请求均为 **0**。未生成可执行 bundle。

实际基线与远端均为 `f65bc5e40a585d93e669c48a1c6a6dcc07bb2bde`，开始时工作区干净；没有合并、reset 或覆盖旧证据。正式源码 **`b1635ed3a47358f3480b320450a95d966db8d62b`**；运行 ID **`planning-only-65987c3380ca44908aea4a47eb9e2d88`**，种子 **71070**。正式前只有 **53 项定向测试**和不调用 IK/规划的预加载检查；开发预加载曾因新目录缺少只读 CAD/模型生成脚本而失败，补齐静态文件后通过。只运行了这一个正式五箱批次。

环境为既有 Ubuntu 22.04.5 / ROS 2 Humble 隔离环境，MoveIt/Pilz **2.5.10**、MTC **0.1.3**，CPU Python **3.12.3**；Intel Xeon Platinum 8470Q，`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`，未升级依赖或改动 Isaac 环境。预算：阶段 **300 s**、单箱 **1800 s**、整批共享 **7200 s**、IPC **360 s**，外层 watchdog **7500 s**；候选不另设 12 秒截断。worker SHA-256：`24062898e71b4968e36a0afb9b27c07c721dfdc964c7afbfddc04edf517cdfc4`。正式运行后 338 个源码文件核对一致。

实际命令见 [actual-command.sh](experiments/m710_top_row_planning_only_20261002/actual-command.sh)，编译及环境路径见同目录 [build-command.sh](experiments/m710_top_row_planning_only_20261002/build-command.sh) 和依赖锁。入口复现时需指定新的 `--output`，已有目录会拒绝覆盖。

交付：[简短计时 JSON](experiments/m710_top_row_planning_only_20261002/summary.json)、[CSV](experiments/m710_top_row_planning_only_20261002/timing.csv)、[五箱实验轨迹](experiments/m710_top_row_planning_only_20261002/trajectories.json)、[调用和计时明细](experiments/m710_top_row_planning_only_20261002/timing.json)、[逐箱进度](experiments/m710_top_row_planning_only_20261002/progress.jsonl)、[压缩原生日志](experiments/m710_top_row_planning_only_20261002/worker.log.gz)。交付明细仅省略每个响应重复的 `clearance.policy`，原始计时文件保留在服务器并记录哈希；实际测量值不变。运行后仅精简了入口的结果序列化，并单列第一箱准备/箱间更新时间，未再规划。此前 Isaac 接触监测失败及完整验收记录保持原样。
