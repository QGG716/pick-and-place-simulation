# FANUC M-710iD/70 V3 几何可行性恢复优先级

> **当前门控（2026-09-10）**：固定提交
> `FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`
> 的完整官方关节链、7 个 visual、7 个 collision mesh、逐连杆惯量和有限关节
> speed/effort 已作为工程仿真输入接入；真机资格继续独立记录。72 杯独立几何选择、
> 官方机器人 triangle-mesh 严格规划、58 个 CAD 派生实体包围盒的工程复合表示、完整
> 阶段连接及 Isaac 实际接触/支撑门控执行入口也已实现。
> 但当前 58/144 实体分类和包围盒尚无逐实体语义及向外包含证书，因此该工具表示只能
> 用于诊断和保守拒绝，不能接受执行路径；当前精确验证器对 J6 与全部工具盒的父级
> 忽略也尚未收窄为有尺寸依据的安装界面接触。这两项都是独立于 home 冲突的
> fail-closed 门。
> 当前 `m710id70_unloading_layout_v1` 的生产 home 在规划前严格失败：每体 10 mm margin
> 对 `J5_link`—`tool_rigid_13` 要求 20 mm 成对净空，完整工具 STL 的 49 点 J6 扫描最大
> 仅约 19.1266 mm。因而本轮 5 个顶层任务尚未进入 IK/路径搜索，动态取放 preflight
> 正确保持 `BLOCKED`；不得通过 SRDF、margin、工具偏移或代理碰撞绕过。模型初始化
> 诊断与完整取放执行分级：前者可以加载真实资产和 40 个动态箱，但必须显式标记
> `INITIALIZATION_ONLY_NOT_PICK_SUCCESS`。本轮详情见
> [官方模型与独立吸盘验证报告](validation/m710id70_official_model_independent_cups_round.md)。
> 本文件下面的 104-task 与 129-carton 结果仍是旧 V3 布局的历史算法证据，不是新布局
> 工程验收分母；其接触感知、task-set IK、escape 与 support-release 实现继续复用。

## 当前工程闭环实施顺序

1. 取得可溯源的实际装配尺寸或经批准的安装适配件定义，解释并解除官方 J5 与皖泰
   安装板当前约 0.8734 mm 的工程净空缺口；不能猜测垫片、改变 margin 或扩大 SRDF。
2. 为 58 个工具刚体补齐逐实体物理语义和保守向外包含证书，并把 J6—工具合法接触
   例外收窄到有可溯源尺寸依据的真实安装界面；在这两个门关闭前，规划只能作诊断搜索，
   不能接受为执行轨迹。
3. 三个 fail-closed 门全部通过后，保留全部 40 箱，在 5 个顶层可移除箱中运行已接通的
   task-set IK、独立杯选择和完整阶段连接，恢复至少一条完整轨迹。
4. 只有完整轨迹通过，才允许实际接触后建立同一箱体的理想固定约束，并按真实接收面
   支撑门控完成带载搬运、释放、撤离和输送。
5. 保存实际状态日志、正常时间倍率连续录像及接触/附着/释放关键帧；初始化诊断录像
   只证明模型加载与正向运行并记录落稳结果，不能替代完整取放。当前规范实跑记录为
   `RETAINED_AND_SETTLED_DIAGNOSTIC_ONLY`，但未执行 penetration gate，且严格合同禁止
   进入取放，因此不能称为完整取放成功或执行资格。

状态：上面的工程闭环顺序是当前开发分支的权威实施顺序。冻结的
[V3 技术验证报告](validation/technical_qualification_report_m710id70_v3.md)
仍是已完成验证结果；下面的 104/129 章节仅保留旧 V3 算法恢复目标、实现合同与
历史回归门，不回写历史结果，也不替代新布局工程验收。

## 历史 V3 基线与恢复目标

V3 已修正坐标、SO(3)、严格 IK/FK 残差、刚体附着、完整碰撞路径、负载和
时间参数化。原始 104 个有效网格任务的完整几何成功为 **0/104**，四组原始
连续场景为 **0/129**。当前代表失败分区为：

| 阶段 | 原因 | 任务数 |
| --- | --- | ---: |
| grasp IK | `GRASP_CONSTRAINT_FAILED` | 44 |
| grasp IK | `NO_IK` | 36 |
| attachment clearance | `PAYLOAD_INITIAL_CLEARANCE_FAILED` | 24 |

冻结 V2 的 52/104（50%）完整覆盖率不再是有效 baseline，也不得作为路径长度、
周期或优化收益的比较分母。

旧 V3 几何恢复目标是：**在不放宽物理约束、不关闭碰撞、不降低数值正确性的前提下，
恢复物理合理且可解释的单箱完整几何可行性。**

## 已完成的旧 V3 算法恢复进展（不替换冻结报告）

- P0-1 已接入带状态的 initial-proximity contract。原始 104 任务探针中，冻结 V3
  的 24 个 `PAYLOAD_INITIAL_CLEARANCE_FAILED` 均合理通过初始门；16 个在开始
  分离前被下游阶段阻止，8 个进入分离但未恢复完整 margin，完整成功仍为 0。
- P0-2 已加入面内 task set、小姿态自由度、多确定性种子、合法 wrist-flip seeds、
  严格无碰撞诊断及细分失败输出。全 104 grasp-only 正式复跑可确定性复现：固定
  TCP 的严格有效 grasp 为 24，task set 后为 52，净恢复 28。其中冻结 V3 的
  44 个 `GRASP_CONSTRAINT_FAILED` 恢复 27 个，36 个 `NO_IK` 恢复 1 个；剩余
  17 个约束失败和 35 个搜索预算耗尽。该结果只证明严格 grasp，不推断完整路径。
- P0-3 已加入逐步几何 escape 检测和严格 SE(3) 执行，四项距离分别进入候选证据。
  原始 `grid_022` 固定传送带 top/90° witness：pure straight 0.320 m，面内偏移
  25 mm，约 0.010002 m 侧向 escape 后恢复普通 margin，并完成完整几何链路。
- P0-4 已把 `Cell.transit` 的 floor 特例替换为由 `SupportRelationGraph`/floor
  关系驱动的一等 `SUPPORT_RELEASE` 阶段。底层替代 witness 的抬升为 0.0204 m，
  完整几何仍通过，负载资格仍为 `NOT_EVALUATED`。
- 固定传送带、原始 104 有效任务的 P0 完整几何复跑已恢复 **10/104**：严格 grasp
  52、完成 extraction 26、完整链路 10；成功项均为 top grasp，其中 5 项执行
  正式 `SUPPORT_RELEASE`。冻结 V3 的 24 个初始净空失败全部通过初始门，19 个
  进入分离、11 个恢复完整 margin、10 个完成全链路。P1 动态传送带与 base Z
  未参与该结果，几何成功也不代表负载或动力学资格通过。

以上为开发回归，不回写冻结 V3 的正式 0/104、0/129 历史报告。连续 129 场景
尚未按新主链路全量复跑，P1 性能优化仍不启动。

## 不可破坏的约束

- 保持世界坐标 `+X` 入车厢、`+Y` 向左、`+Z` 向上以及全部 SI 单位。
- 保持 V3 严格最终 FK 残差、关节限位、完整路径碰撞、工具净空和吸盘覆盖检查。
- 不全局降低 collision margin；不关闭自碰撞或环境碰撞。
- initial-proximity 例外只能作用于已登记的“目标箱—静止邻箱”对，不能传播到
  机器人—车厢、机器人—工具、机器人—底盘、工具—环境或其他安全对。
- 不缩小原始场景，不删除困难任务，不修改 104 和 40/27/32/30 的分母。
- 负载资格与几何搜索独立记录。`PAYLOAD_CG_FAIL` 不得终止后续几何候选搜索，
  也不得修改 FANUC 曲线或证据映射来制造 `PASS`。
- 所有新增搜索必须由显式 deterministic seed 控制，并可串行复现。

## P0-1：初始接触/近接感知的脱垛语义

当前每个 OBB 按 10 mm 膨胀，两个箱体需要约 20 mm 工程净空，而原始货垛
邻箱间隙约 10 mm。目标箱刚附着、尚未运动时因此可能直接触发
`PAYLOAD_INITIAL_CLEARANCE_FAILED`。这不是降低全局 margin 的理由。

实现一个仅限 payload-neighbor 的 initial-proximity contract：

1. 在刚体附着前记录目标箱与每个静止邻箱的真实几何 signed distance、接触/穿透
   状态、法向或最小分离方向，并冻结为该次候选的 allowed initial proximity 集合。
2. 初始真实几何没有穿透时，允许其从小于工程 margin 的已有间隙开始分离；初始
   已穿透不能因 proximity 语义变成合法状态。
3. 每个离散边和边内采样点都不得产生新的真实几何穿透，也不得让登记对的
   signed distance 低于其初始值（数值容差须显式配置和记录）。局部分离阶段优先
   要求 signed distance 单调增大；若允许短平台，必须保持不恶化且可解释。
4. 某一登记对首次达到正常工程净空后立即退出例外；不得再次进入 margin 内。
   所有登记对都退出后，后续 extraction/transit 完全恢复普通碰撞判据。
5. 失败证据至少保存碰撞对、初始/当前 signed distance、最差变化量、路径分数、
   是否已退出例外和具体失败原因。

专门回归测试必须覆盖：

- 10 mm 邻箱间隙下，目标箱允许开始分离；
- 朝邻箱方向的第一条运动边立即失败；
- 朝远离邻箱方向的运动允许；
- 达到正常安全间隙后恢复常规碰撞判据，重新靠近会失败；
- 非 payload-neighbor 碰撞对的 margin 完全不变；
- 初始真实穿透以及移动中新增/恶化穿透始终失败。

P0-1 的统计输出要单独说明 24 个
`PAYLOAD_INITIAL_CLEARANCE_FAILED` 中：合理解除、真实穿透、分离方向错误、
其他安全碰撞及下游失败各有多少，不能把“通过初始门”计为完整任务成功。

## P0-2：由精确单点 IK 升级为 task-set grasp IK

保持 V3 的 `0.0001 m / 0.0002 rad` 严格最终 FK 残差判据，不恢复 V2 宽松
tolerance。变化发生在目标集合与搜索覆盖，而不是最终正确性。

对每一个已通过暴露面和吸盘几何预筛的箱面，确定性搜索：

- 面内吸取点位置；
- 绕面法向的 roll；
- 显式配置的小范围姿态自由度；
- 多个 deterministic seeds；
- 机械臂模型与限位允许时的 wrist-flip / mirrored configuration。

每个最终候选仍必须独立通过严格 FK 残差、joint limit、奇异性策略、机器人/工具/
环境碰撞、工具净空、实际 FK 下的吸盘覆盖和刚体附着验证。候选预算、枚举次序、
种子派生规则和去重规则必须写入证据，避免并行度改变结果。

把现有 44 个 `GRASP_CONSTRAINT_FAILED` 至少细分为：工具—机械臂碰撞、
工具—环境/邻箱碰撞、机器人—环境碰撞、实际吸盘覆盖失败、关节限位、
奇异性策略以及其他明确约束；不得继续用一个聚合标签掩盖原因。

把现有 36 个 `NO_IK` 至少细分为：严格残差未收敛、位置不可达、姿态不可达、
可达但所有分支超限、可达但所有最终候选碰撞，以及搜索预算耗尽/尚未证明。
同时报告其中多少由 task-set IK 恢复为严格有效 grasp、多少进入后续阶段、多少完成
完整任务。

## P0-3：基于 escape path 的 extraction

保留 620 mm 作为“600 mm 深箱、当前邻箱间隙、固定姿态下 pure straight
extraction”的正确基线，不把它标为 bug。主规划器改为：

```text
INITIAL_CONSTRAINED_SEPARATION
  -> 每个小步更新 initial-proximity 状态并检测局部 escape trajectory
  -> 一旦存在安全的斜移/抬升/转向 SE(3) 路径，结束纯直线约束
  -> ESCAPE
  -> FREE_MOTION
```

局部 escape 路径必须连续继承当前严格附着、FK、限位、碰撞和工具净空判据，
不得用端点无碰撞代替完整边检查。搜索及插值分辨率、姿态自由度和 RNG seed
必须显式、确定且进入缓存键。

每个候选分别记录，失败时也尽量保留：

- `pure_straight_clearance_distance`
- `actual_constrained_extraction_distance`
- `distance_until_first_escape_path`
- `total_stack_release_distance`

这些指标不得互相代替；找到 escape path 也不等于后续完整任务成功。

## P0-4：正式的 support-release motion primitive

把 V3 bottom alternative 已验证的先抬升机制从 `Cell.transit` 内部特例提升为主规划器
的一等阶段。由 `SupportRelationGraph` 和实际支撑审计决定是否执行：

```text
GRASP -> [SUPPORT_RELEASE] -> EXTRACTION -> TRANSIT -> PLACE
```

`SUPPORT_RELEASE` 必须有明确的进入条件、目标支撑关系、最小释放位移、完整路径、
退出证据和失败原因。不得在箱体仍接触地板或其他支撑面时直接进入普通 loaded
transit；无支撑或已释放的箱体不得被无条件额外抬升。该阶段与 P0-1 的邻箱近接
状态同时生效时，必须同时满足“支撑距离增大”和“邻箱 signed distance 不恶化”。

## P1：暂缓性能与安装位优化

在原始任务中出现稳定、可重复的非零完整几何成功集合之前，暂停：

- dynamic conveyor A/B 优化；
- base Z / lift range 优化；
- 以路径长度、周期或共同可行集合为目标的比较。

相关代码和历史证据可以保留，但不得作为当前 P0 工作的成功判据。共同可行集合为
零时，性能差继续记为 `NOT_EVALUATED`。

## 旧 V3 恢复验收门（历史）

按以下顺序验收，不允许以后项掩盖前项：

1. 原始完整单箱任务由 0 恢复为非零，且串行/并行确定性复现一致；
2. front、side 或 top 至少一种抓取建立可重复的完整成功 case；
3. 对 24 个 `PAYLOAD_INITIAL_CLEARANCE_FAILED` 给出 P0-1 后的完整分解；
4. 对 44 个 `GRASP_CONSTRAINT_FAILED` 给出可操作的细分原因；
5. 对 36 个 `NO_IK` 报告 task-set IK 的恢复数与后续阶段去向；
6. 主规划器形成标准 `SUPPORT_RELEASE` primitive 及回归证据；
7. 以上稳定后，才重新开启传送带与升降轴优化。

每个非零成功 witness 必须保存有效配置、输入场景、种子、候选 task-set 参数、
实际 FK 残差、逐阶段路径、碰撞/近接审计、支撑释放、放置支撑和独立负载资格。
几何成功不得被表述为负载、动力学或真机安全资格通过。
