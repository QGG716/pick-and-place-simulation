# 固定站位间距搜索：实际结果（2026-11-07）

本轮最好结果为 **同一固定站位、同一完整状态父链连续 3/40 箱**，g=200 mm。尚未找到完整 40/40；剩余 37 箱保留在场景中。独立复核状态为 `PASS_PARTIAL_SINGLE_BRANCH_ONLY`。历史移动基座结果、其他 g、独立试验和失败周期均未拼入该计数。

`physical_execution=NOT_RUN`。这是原生 MoveIt 规划、名义重力下落与理想接收/出料模型下的结果；不是 Isaac、实体运行或物理接收资格。

## 实际计算覆盖

共完成 146 次正式尝试记录，另保留 1 次中断/导出错误记录。累计各搜索进程计入的初始化和规划墙钟耗时之和为 **13806.19 s（3.84 h）**；并行运行，因此不是日历历时。准入几何检查另计 51.67 s。失败和修订前成本未清零。

共对 52 个不同 g 做几何/支撑准入检查，其中 9 个 g 真正进入原生规划。下表逐距离取一个连续分支的最大计数，禁止把各行相加。

| g (mm) | 原生尝试数 | 最好连续箱数 | 尝试墙钟累计 (s) |
|---:|---:|---:|---:|
| 5 | 9 | 1/40 | 549.57 |
| 50 | 5 | 1/40 | 317.70 |
| 100 | 5 | 1/40 | 376.93 |
| 150 | 5 | 1/40 | 313.62 |
| 190 | 18 | 1/40 | 1802.17 |
| 195 | 18 | 1/40 | 1842.35 |
| 200 | 50 | 3/40 | 4832.80 |
| 205 | 18 | 1/40 | 1775.63 |
| 210 | 18 | 1/40 | 1857.68 |

原生未访问但已通过离散几何/支撑准入的 g (mm)：10, 15, 25, 30, 40, 45, 55, 60, 70, 80, 90, 95, 105, 110, 120, 130, 140, 145, 155, 160, 170, 180, 220。

现有有限地板之外的支撑尚不明确，故未进入正式原生规划的 g (mm)：250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 1050, 1100, 1150, 1200。它们标为 `UNKNOWN_EXTERNAL_SUPPORT`，没有扩大地板、延长机构或移动出料平面，也没有据此判为全局无解。

主搜索先完成粗样本，再聚焦 g=200 mm；四个独立细化世界 g=190、195、205、210 mm 各完成 18 次有界尝试。v6 主调用共分配新增 40 次尝试、单次 240 s、前缀 90 s、阶段 120 s；主搜索在累计 59 条记录后保留最佳三箱分支，显式回访其两箱父状态，把剩余 15 次分配用于其他顺序。所有节点、游标、累计 4984.799 s 成本和已接受状态保持不变，驱动与备份哈希见 `branch-revisit-event.json`。第二数值轮次允许更高的单次预算。各次真实预算、种子、未访问候选数和源修订详见 `result.json`。没有整体业务截止时间，也没有把一次 IK/RRT 超时当成不可达。

失败/完成记录分类：`COMPLETE_CYCLE`=12, `FINITE_NATIVE_SEARCH_INCOMPLETE`=3, `NUMERICAL_SEARCH_DEADLINE_NOT_UNREACHABLE`=130, `SOFTWARE_ERROR`=1。`COMPLETE_CYCLE` 是跨距离、跨分支的成功尝试记录数，不是连续卸箱数；正式结果只取上述单分支计数。

## 最好单分支与固定尺寸

总成 A 原点世界坐标为 [-1.950, 0.350, 0.000] m，机器人 base_link/J1 原点为 [-1.350, 0.150, 1.000] m；底盘实体中心为 [-1.950, 0.350, 0.300] m。横带中心世界 X=-0.550 m、Y=0.000 m，+X 前沿为 −g=-0.200 m；厚度 0.120 m，因此带体中心 Z=带面高度−0.060 m。

独立验证保持 base_A=[0.600, −0.200, 1.000] m、1.050−0.600=0.450 m、底盘高 0.600 m、实心台座高 0.400 m。安装轴线没有误用不对称基座网格的包围盒中心。初始带面 0.600 m，候选和所有机构过渡均限定在 [0.550, 1.500] m；已接受周期的接收面如下。

| 顺序 | 箱子 | 抓取面 / 模式 | 带面 (m) | 释放箱底 (m) | 实际名义落差 (m) | 完整周期规划耗时 (s) |
|---:|---|---|---:|---:|---:|---:|
| 1 | `carton_l07_c04` | FRONT / `FRONT_DIRECT_HIGH_DROP` | 1.500 | 2.100000 | 0.600000 | 64.61 |
| 2 | `carton_l07_c01` | FRONT / `FRONT_DIRECT_HIGH_DROP` | 1.500 | 2.100000 | 0.600000 | 113.79 |
| 3 | `carton_l07_c00` | FRONT / `FRONT_DIRECT_HIGH_DROP` | 1.500 | 2.100000 | 0.600000 | 34.70 |

上表规划耗时包含寻找及检查完整周期，不能当作机器人机械节拍。机械臂原生时间参数轨迹、名义落箱、理想出料的模型时间分别为：

| 箱子 | 机械臂轨迹 (s) | 名义落箱 (s) | 理想出料 (s) | 实测完整机械周期 |
|---|---:|---:|---:|---|
| `carton_l07_c04` | 32.716 | 0.349749 | 23.799 | NOT_RUN |
| `carton_l07_c01` | 67.049 | 0.349749 | 17.499 | NOT_RUN |
| `carton_l07_c00` | 29.575 | 0.349749 | 15.599 | NOT_RUN |

横带升降没有经过硬件速度标定；回放中的升降显示时间不代表实测执行时间。每个已接受周期均验证真实原生父链、当前 72 位吸盘掩码、同箱附着/解除、全投影落带、完整名义下落路径、工具分离及整个箱体越过 X=−3.200 m。失败箱不会被移除。

## 当前卡点与未搜索范围

以下针对最终选中的最佳三箱父状态。同样达到三箱的另一分支没有覆盖它，也没有叠加计数；后续其他节点的尝试、种子及归档见 `result.json`。

- `carton_l07_c02`：该最佳父状态下保留 11 次失败；最近 trial 59，接触完成=True，结果 `RECEIVER_REGION_SEARCH_INCOMPLETE` / `LOCAL_NUMERICAL_VISIT_DEADLINE_NOT_UNREACHABLE`。原生失败细项：extraction/plan: NATIVE_PLANNING_FAILED (NO_IK_SOLUTION) ×121；pregrasp/plan: NATIVE_PLANNING_FAILED (INVALID_MOTION_PLAN) ×1。
- `carton_l07_c03`：该最佳父状态下保留 10 次失败；最近 trial 58，接触完成=False，结果 `SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED`。原生失败细项：pregrasp/plan: NATIVE_PLANNING_FAILED (INVALID_MOTION_PLAN) ×14；pregrasp/plan: NATIVE_PLANNING_FAILED (PLANNING_FAILED) ×13；pregrasp/plan: NATIVE_PLANNING_FAILED (TIMED_OUT) ×36；pregrasp/ik: NATIVE_IK_FAILED ×12；pregrasp/plan: NATIVE_PLANNING_FAILED (GOAL_STATE_INVALID) ×1。

当前最高可卸层尚未完成，因此更低层未进入正式连续原生搜索。中下层 TOP→FRONT→SIDE、公平轮次、真实 +Z 脱垛/抬升和最低 0.600 m 释放箱底已实现并有针对性边界测试；不能把这些测试说成已卸完中下层。

补充纯运动学诊断：g=200 mm 下，`l07_c02` 的固定 Y 接收采样点以及一个完整法向脱垛采样点，各用 49 个数值初值仍未找到满足严格 FK 误差的解；横带两侧接收采样点找到了纯运动学解，但没有验证碰撞、完整脱垛或接触到接收点的连接路径。这不是新增成功，也不是不可达证明。`kinematic-diagnostics-summary.json` 另保留全部 15.444 s CPU 诊断成本；其中首版未按 g 过滤的诊断明确作废并重做，未进入正式规划输入或完成数。两个诊断脚本一并提供。

上层保持一个明确的正吸接触姿态，当前区域连接沿法向进入横带；本次有限候选未覆盖所有可能的 SE(3) 搬运路径。未访问的 g、数值轮次、顺序分支和中下层仍保留为未搜索。**没有证明不存在可行固定距离。**

## 碰撞、假设与验证

- 本地与服务器交付目录各 47 项针对性测试通过；运行时 v6 为 46 项通过，差异是终态保存回归，非碰撞放宽。原生 C++ gap contract/height/transition/outfeed/安装反例测试通过。
- 固定 base_link/台座安装面和台座/底盘接触有几何依据。整臂/底盘、默认 base_link/底盘和指定箱工具/底盘豁免均关闭；J5/J6 自有工具及命名柔性吸盘邻箱规则没有扩展到环境。
- 真实 CPU/原生反例均拒绝：J4/底盘、J4/台座和工具代理/底盘等非法姿态。两端第一碰撞证据可能不同；工具代理相交不冒充原 CAD 穿透。详见 `counterexamples-summary.json` 的姿态、碰撞对、来源和原始证据哈希。
- 独立复核整条最佳分支，通过原始 40 个身份、固定安装变换、同一父链、机构重建、5 mm 普通外部间隙、官方有限关节限制、当前接触掩码、释放与出料事件。接触掩码由当前 FK 几何重算，不是传感器实测。原生逐样本检查不是连续扫掠证明。
- 保留 `ideal_independent_cups`、固定姿态名义重力下落、`RECEPTION_ASSUMED`、理想出料。有限纵带必须有真实表面接触重叠，允许显式理想悬伸；未验证真空承载、冲击、反弹、倾覆、机构强度或下游物理稳定性。

运行修订与失败都保留：首次 NumPy 布尔值序列化问题；gap schema 出料客户端门控；释放后 payload 身份在独立验证中的分类问题；原生 IK 分支的有限区域访问时间片。一次原生周期虽生成完成但导出失败、计划未保存的 trial 4 没有计入完成数，其 70.166 s 成本和日志保留。

## 交付与复现

代码、冻结配置、命令、运行时及交付源码 manifest、恢复换行符工具和版本差异见本目录 `README.md`。Git 内容重建验证已核对交付 568 文件和运行时 v6 的 567 文件。原生二进制 SHA-256 为 `4bf8a96c3c7f6b84ec498629a67d06541a71eae3e7da64697e426a362fcd3fe2`。`native-runtime-identity.json` 另记录跟踪共享库与启动脚本哈希，以及首次接受任务返回的 MoveIt/Pilz 2.5.10、MTC 0.1.3 版本。

三维回放 `best-replay.html` 已浏览器核验：初始 40 箱，末态 3 箱完成、37 箱保留，无 JavaScript 错误。显示箱堆、厢体、底盘、400 mm 台座、机械臂、横纵带及升降机构，并标注 1000 mm、450 mm、550–1500 mm 和最终 g。回放总显示时长 194.284 s，仅为规划展示。

服务器宿主根目录 `/root/m710-gap-search-20261107/rootfs/gap-search-20261107`；chroot 内为 `/gap-search-20261107`。原始压缩证据包未下载到本地。各检查点与归档均留在下列服务器目录：

- `/gap-search-20261107/run-v1/checkpoint.json`，SHA-256 `166454f028526602cb9e5ff7b6e944dbdc3b9a927dfcefb8f9c24d5c7949dcb5`，终态 `INVOCATION_RESOURCE_LIMIT_CHECKPOINTED`。
- `/gap-search-20261107/refine-g0190/checkpoint.json`，SHA-256 `6f14e787f726b7b08cdba4502ab548803d9e795eb3777e8fc3d24f8a3697350f`，终态 `INVOCATION_RESOURCE_LIMIT_CHECKPOINTED`。
- `/gap-search-20261107/refine-g0195/checkpoint.json`，SHA-256 `064a8807c0112bf00e8d712c6be42d7366033046918e26d635df45e226559968`，终态 `INVOCATION_RESOURCE_LIMIT_CHECKPOINTED`。
- `/gap-search-20261107/refine-g0205/checkpoint.json`，SHA-256 `fa32de40d2dd70e878e7c00782baf62fccb7880243ae79a3a36c222056519ab8`，终态 `INVOCATION_RESOURCE_LIMIT_CHECKPOINTED`。
- `/gap-search-20261107/refine-g0210/checkpoint.json`，SHA-256 `52594f36494d171b93660b5bf66f8ebaea702d3816c222cd329f564ebac13945`，终态 `INVOCATION_RESOURCE_LIMIT_CHECKPOINTED`。

关键产物：

| 文件 | SHA-256 |
|---|---|
| `best-branch-verification.json` | `b199509e6507aa5c25b4086d9302b5309ef0dc3218811c6edcf95bf6e6c4fe4e` |
| `best-replay.html` | `733b27214c979d3ce9faf47d1f661eeca55f90a1abb702f686724545165ac0f8` |
| `best-replay.checks.json` | `3b2c1e916b21a431df3d909d84c550ccd308d2fd9f31ee32f22c43c8abebb000` |
| `result.json` | `4eda378fc4805537b82658257487ad336ab45d63e7818c8157a9354b400218a1` |
| `source-manifest-v6.json` | `fdff0ac49b2b45bde9544687f4416def4284ce15b2ddf47319fd6117470f595e` |
| `refine-source-manifest-v6.json` | `fdff0ac49b2b45bde9544687f4416def4284ce15b2ddf47319fd6117470f595e` |
| `delivery-source-manifest.json` | `73b00613f6e12b0dd0f9c7305d849eb028403e56b8d1d50488b233d5d83ceb8f` |

本轮是可恢复的部分结果；所有服务器检查点、原生失败请求和累计成本均保留。
