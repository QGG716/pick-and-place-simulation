# 单目标规划附着与受控抽离（2026-10-16）

最终源码 `65035e7ca871645b65f9b5581c3e61d042e248a8` 的 c02、c04 均重新生成到 extraction，均首个正面（x−）成功，无面级回退。**只证明本轮规划候选到抽离；不可执行，未卸箱。** `qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`。

[机器可读汇总](experiments/m710_single_extraction_20261016/summary.json)保存分项、真实调用、父链、初始状态及失败记录；[原始文件摘要](experiments/m710_single_extraction_20261016/raw-manifest.json)对应压缩后的完整结果、三段轨迹、轻量请求/响应和 worker 日志。解压结果只用于复核，不作为规划输入。

## 改动与边界

在原入口增加显式 `--through-extraction`，由 `single_extraction.py` 扩展成功 contact 的完成钩子；不传该参数仍止于 contact。保留正面→顶面→侧面、每面单目标、单接近 IK、单 RRTConnect、单 contact LIN。未调用旧完整 plan / _approach / _connect_pose，未增加候选、PTP、先验或抽离 OMPL。

附着捕获本次 contact 实际末 q 的物理接触 FK，以及当前目标箱位姿；保留实际法兰变换和 parent receipt。终端几何检查两案各得到 **48 个命令杯接触成立**，据此更新附着上下文；预期 actual mask 仍为 null，未把占位值当成证据。这些都是规划几何，不是物理测量。world→attached 使用既有 extraction 请求的 `ownership_transition=ATTACH`；没有新增 ATTACH worker 操作。附着前后箱体世界矩阵最大误差分别为 **4.44e−16 / 1.11e−16**，contact→extraction 的 q 逐值完全相同，Task/父链正确，目标身份唯一。

唯一抽离目标沿实际接触面的向外法向 −X，保持实际末态工具朝向。复用 `minimum_clearance_extraction_distance`，针对完整箱体与全部剩余世界 OBB（c02 为 47 个、c04 为 43 个，包含相应 l06 支撑箱、邻箱和静态环境）计算；沿用 0.01 m 扫描与 0.8 m 上界。最终距离均为 **0.608756205 m**，来自箱体分离需求及原有净空/误差保护，不等于 0.1 m 接近退距。终端要求为原 5 mm 外部净空 + 0.2 mm 接触容差 + 3 mm 既有抽离储备 = **8.2 mm**，目标生成另保留既有 FK 边界保护 **0.556205 mm**。

只发一次 extraction Pilz LIN。返回的 **159 个原生轨迹点**按序核查物理 FK、负载位姿和原有接触历史，新增插值采样数为 0。末态最小箱体/世界间隙分别为 **8.756212 / 8.756205 mm**，tracker 已完全分离。新 `check_extraction_end` 只检查实际末态一个状态，移除初始接触/支撑许可，保留正常机器人、完整工具、负载的净空规则；两案均通过。此检查绑定已有 extraction 父记录，没有新运动求解，也不改返回末样本。

抽离内部沿用批准的初始接触机制；本轮仅补返回点有序核查，原生 `ordered_checks=0`。不将这些状态检查或返回点核查称为独立密集工艺审计、连续扫掠证明。面级失败恢复原始未附着候选上下文；取消、过期、起点或身份错误停止任务。

## 实测时间与调用

单位秒；同机顺序 c02→c04，各最终案例运行一次。

| 项目 | c02 | c04 |
|---|---:|---:|
| 三段完整生成 / 选面 | 是 / 正面 x− | 是 / 正面 x− |
| 目标列表生成 | 0.013634 | 0.015844 |
| 当前面目标/杯几何 | 0.013560 | 0.013409 |
| 显式接近 IK | 0.061110 | 0.057624 |
| pregrasp | 1.116334 | 11.558831 |
| contact | 0.247186 | 0.239129 |
| 终端接触几何核查 | 0.008805 | 0.009014 |
| 附着上下文 | 0.007390 | 0.007331 |
| 唯一抽离目标计算 | 0.330595 | 0.336865 |
| extraction 请求及返回处理 | 0.463652 | 0.451117 |
| 已返回抽离点 FK/负载/历史核查 | 1.199097 | 1.134530 |
| 原生终点自由空间检查（含构造/往返） | 0.080120 | 0.075471 |
| **完整三段规划总时间** | **3.682375** | **14.047261** |
| 初始化（不计规划） | 1.666976 | 1.668419 |
| 其中场景准备（包含于初始化） | 0.013260 | 0.013021 |

各阶段是调用侧墙钟，包含请求构造、通信和返回处理；原生 solver/检查时间包含在其中，不再相加。总规划还包含起点检查、receipt/场景组织等工作，未全部归因。未单独测量物理运动或在线交付。c04 主要耗时仍在 pregrasp，本轮不做搜索优化；此处不是前轮速度对照，也不是五箱成绩。

每案真实求解调用均为 **IK=1、OMPL/RRTConnect=1、LIN=2、PTP=0**，其中 extraction LIN 仅 1 次；先验 build/load/query=0，旧生成器/历史路径输入=0，禁止入口尝试=0。请求序列为初始化/模型核对→check_start→IK→pregrasp→contact→extraction（ATTACH）→check_extraction_end。不是“求解一次仅检查一个状态”。

| 实际 Clearance 工作量（各请求独立统计后相加） | c02 | c04 |
|---|---:|---:|
| 状态查询 | 927 | 11,445 |
| AABB 测试 | 7,612,657 | 65,628,842 |
| AABB 跳过 | 7,600,976 | 65,278,897 |
| 实际 FCL 窄查询（含补充） | 11,807 | 350,071 |
| Clearance 累计内部检查秒（嵌套于上表） | 0.223188 | 3.048527 |

每案 extraction 的新工作区首次建立 **126 个 AABB 来源、4 个窄阶段来源**固定关系资格；随后各复用 **20,034 / 636 次**。终点检查另建工作区，重新建立 8 / 122 个资格，未继承旧缓存。复用次数不等于省掉的 FCL 次数。ProcessSealInput 每请求编译一次，持箱 160 次工艺检查仍实际计算 seal/compression，最终密封判定复用为 0。所有检查响应 `pair_profile_enabled=false`，逐对象对计时细项为 null；工艺细粒度计时也关闭。

## 版本、测试及保留失败

起始本地/远端均为 `28dfa777827dd44ec7e9147430b1f803ba23010e`，工作区干净；无重置、覆盖未提交文件或依赖升级。

首版源码 `52a50ebcc846c80b12efccb9bbb57107a41fb8c8` 的两次尝试均因我遗漏新终点操作的 worker 白名单登记而失败：c02 **8.676886 s**、c04 **19.894602 s**。三段原生子解虽返回，终点验收未完成，均保留 complete=false。约 5 秒错误清理等待包含在其失败时间中，不是几何不可达。完整失败留在 [v1](experiments/m710_single_extraction_20261016/v1/freeze.json)，未用最终结果覆盖。

只补操作登记并增加共享调度白名单回归，冻结为 `65035e7` 后按授权重跑受影响的两案；seed、初始状态、几何距离规则、预算和资源不变。[v2 冻结身份](experiments/m710_single_extraction_20261016/v2/freeze.json)。实测之后没有再改规划源码。

- Python：**138 passed / 1.82 s**，含 25 项新增抽离测试；最终补修只改 C++，Python 文件散列未变。
- 原生 Release 重建后：`m710_native_semantic_event_test`、`m710_request_evidence_test`、`m710_process_policy_test`、`m710_fixed_reuse_test`、`m710_clearance_test` 全部通过。
- 覆盖实际末态/机器精度、捕获位姿、终端 mask、父链、唯一 LIN、距离不足、返回中段/终点拒绝、接触历史重入、失败后不交成功前缀、面级恢复、旧两段协议及禁止调用。真实 worker 已实际完成新终点操作。见 [测试记录](experiments/m710_single_extraction_20261016/tests/python-tests.log)。

## 环境与复现

Ubuntu 22.04.5 / ROS 2 Humble 隔离 rootfs，Xeon Platinum 8352V，CPU 0–21，OMP/OpenBLAS=1，Release `-O3 -DNDEBUG`，g++ 11.4，MoveIt/Pilz 2.5.10、MTC 0.1.3、FCL 0.7.0。使用既有 Python 环境，未升级。正式 FANUC M-710iD/70、完整工具及 20 kg / 42.5 kg 配置保持原样，99 个资产散列与已有冻结身份一致，见 [资产清单](experiments/m710_single_extraction_20261016/asset-identities.json)。

最终 worker SHA256：
`5626f8b47186d3de72714e2780dbe90a8832592bb898270ae99cdb2f2bccec0c`。
两案种子 71070，任务共享各面截止 300 s，阶段 90 s，IPC 100 s。c02 从配置完整 40 箱静止状态开始；c04 从冻结的实际规划撤离初始 q 和已理想移除四箱列表恢复，剩余 36 箱。两者均非本次物理实测。输入仅恢复起始条件，不含成功终点/路径/mask；不加载先验数据库。

实际命令完整保存于 [run-cases.sh](experiments/m710_single_extraction_20261016/v2/run-cases.sh)；在该隔离环境中：

```bash
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
O=/tmp/single-extraction-20261016-v2
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
  --through-extraction --target carton_l07_c02 --output "$O/c02" \
  --worker-command "$O/worker.sh" --worker-binary "$O/build/m710_moveit_worker" \
  --source-commit 65035e7ca871645b65f9b5581c3e61d042e248a8 \
  --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100
# c04 使用相同参数、独立输出 c04，并增加 --initial-state "$O/c04-input.json"。
# 复现须另选不存在的输出目录，入口拒绝覆盖已有结果。
```

运行 ID：c02 `single-contact-06486fdfd4364688839e51e1f8dadce2`；c04 `single-contact-18ccc3e412b9405ba79b15add5656916`。

旧完整取放入口、五箱入口及其 transit/place/release/withdrawal 仍未迁移到本次新链路。本轮没有 Isaac、真机、真实吸附/承载、视觉、preflight、执行包、动力学、密集资格验收、五箱/40 箱或历史大任务集，也没有全量 pytest。完成规划附着与受控抽离后停止。
