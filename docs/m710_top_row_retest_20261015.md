# 顶排五箱完整 planning-only 复测（2026-10-15）

唯一一次正式批次 **5/5 完整生成，纯规划 21.237330 s，在线交付 21.576754 s**。这是离线配置几何中的连续规划；初始机器人静止，非本次物理实测状态。结果为 `PLANNING_ONLY_NOT_EXECUTABLE`，`qualification_status=NOT_EVALUATED`，`continuous_swept_proof=false`。

基线和核对的远端为 `cb95afadcb078160c08671a6ca87dc1cadcec329`。正式源码为 **`8d55861839702128afb457af42ca7ac0be0d62e5`**，仅在现有入口保留已有请求 ID、指纹和已有时间字段，未增加计时点、改规划或几何。后续提交新增只读汇总工具和本报告；代码提交与交付提交表示两个不同快照，均使用 UTF-8。运行 ID：`planning-only-443d386e0e1b4274b530a871e39a222a`。

## 实测结果

| 顺序 | 箱体 | 完整 | 纯规划(s) | 累计(s) | 场景准备(s) | IK/PTP/LIN/OMPL | 先验命中/直连/失败查询 |
|---|---|---|---:|---:|---:|---|---|
| 1 | carton_l07_c02 | 是 | 1.680845 | 1.680845 | 0.036031 | 3/0/4/0 | 0/2/0 |
| 2 | carton_l07_c01 | 是 | 1.778008 | 3.458853 | 0.037473 | 3/0/4/0 | 1/1/0 |
| 3 | carton_l07_c03 | 是 | 4.949768 | 8.408621 | 0.038389 | 3/0/4/0 | 2/0/0 |
| 4 | carton_l07_c00 | 是 | 2.963806 | 11.372427 | 0.037108 | 7/0/4/0 | 2/0/0 |
| 5 | carton_l07_c04 | 是 | 9.864904 | 21.237330 | 0.039149 | 5/1/4/0 | 2/0/1 |

既有几何顶排策略决定此顺序；没有手工指定顺序或使用 diagnostic-input。起始完整 40 箱，后续箱从上一箱实际撤离末态继续。各箱完整候选失败计数均为 0，但第 4 箱有 4 个失败 IK seed；第 5 箱 pregrasp 包含一次先验超时及 PTP INVALID_MOTION_PLAN，后续局部回退成功，全部耗时已计入。5 个 place 为合法零运动语义事件，不伪造 LIN 调用。

初始化 **0.983464 s** 单列；首箱场景准备 0.036031 s，箱间场景更新 **0.152119 s**，合计 0.188150 s；批次墙钟 21.426390 s，记账 0.000909 s，结果写出 0.150364 s。在线时间=批次墙钟+写出，不含初始化、write_timing.json 自身写出、最终记账和 stdout，沿用入口定义。进程启动至退出 23.576757 s，不是机器人节拍。

## 当前主要成本及证据边界

| 已测项目 | 秒 | 关系 |
|---|---:|---|
| 37 次运动/语义请求往返 | 16.383266 | 纯规划中的互斥请求区间 |
| 17 次显式 IK 批请求往返（21 次 IK） | 0.387175 | 每批 IPC 只计一次 |
| 未归因剩余时间 | 4.466890 | 纯规划减上述两项，不能称 Python 或 IPC 耗时 |
| 11 次先验查询 | 13.385182 | 包含于运动请求，不再相加 |
| 原生 Pilz 求解 | 1.251635 | 包含于运动请求，含失败 PTP |
| 原生 IK 求解 | 0.005409 | 包含于 IK 请求 |
| 35 份可用响应的 Clearance 累计 | 6.741633 | 不完整、嵌套于相应请求/求解 |
| 同 35 份工作区准备 | 0.111332 | 仅工作区准备，非全部上下文成本 |

按阶段汇总运动请求：**pregrasp 10.882557 s**、transit 3.690898 s、extraction 1.032483 s、withdrawal 0.301780 s、contact 0.265747 s、place 0.209800 s。最大单请求是 c04 pregrasp 请求 54，约 **6.035 s**，状态 PRIOR_QUERY_TIMEOUT；该查询检查 87 条边、拒绝 60 条边，再进入既有回退。全批先验查询检查 172 条边、48,693 个状态，拒绝 108 条边，包含首次资格成本和失败消耗。

请求 54（先验超时）和 55（PTP 失败）没有返回 Clearance 明细；不能把缺失项记零或把 6.741633 s 称为全批检查总时间。已有源码显示每个请求新建独立 Clearance，阶段 phase 为互斥计数；MTC 重用 ready 父子解不重算该父记录。按 request_id / IK 的 request_id+batch_seed_index 检查，本次无重复记录；不累计 Task 全局 pipeline_calls，不用最大值/最后值代替去重。IK 批次 seed 0 承担请求往返，其余为 0，核对 consumed 数一致。未新增逐对象对剖析。

**下一步可明确定位到 pregrasp 的先验连接搜索及其失败尝试；尚不能据此决定优先改几何遍历还是候选选择。** 大量碰撞采样和一个 6 秒失败连接同时存在，既缺两个失败请求的 Clearance 聚合，也缺剩余 4.466890 s 的边界归因；没有证据将整个查询当 FCL 或直接指定宽阶段改造。本轮不实施后续优化。

## 优化和连续状态核查

35 份含明细响应均为 optimized、native_check_reuse=true、fixed_narrow_qualification=true；ProcessSealInput 各构造一次、输入预解析启用。pair_profile_enabled=false、pair_profile_status=NOT_MEASURED，所有 broad/narrow/self/world 细项均为 null；逐吸盘 profile_enabled=false。两个缺明细的失败请求使用相同已核对构造器默认值，但不虚构其响应字段。

可用响应累计：AABB 检查 **216,772,881**、跳过 **213,735,595**；实际 FCL 窄查询 **3,039,136**（含补充资格查询 1,850）；AABB 来源资格/复用 **750 / 162,674**，窄阶段来源资格/复用 **1,850 / 956,370**。这些是可用响应小计；缓存命中不是省掉的 FCL 调用数。

仅阅读本次记录：四次箱间 q 精确相等；每箱独立 Task；选中父链、阶段端点、目标身份及 attach/release 顺序一致；撤离末态中目标仍在规划放置位置，其余未卸箱原位，随后才理想移除。该核查未做逐采样来源扫描或密集碰撞复检。历史输入、旧生成器、执行请求均为 0。实际旧优化已接入完整规划链，未以开关声明代替响应证据。

## 环境、复现及范围

Xeon Gold 6459C，固定亲和性 0–21，OMP/OpenBLAS=1；实测 Python 2 线程、worker 11 线程。既有 Ubuntu 22.04 chroot / ROS Humble / FCL 0.7.0-3 / MoveIt/Pilz 2.5.10 / MTC 0.1.3，Release -O3 -DNDEBUG。未升级依赖；本轮未改 C++，复用已构建 worker，当前所有原生源码与其构建源码逐字匹配。

worker SHA256：`520821740534541ef4d779cbe305397cef35fcd7b26a8469f43c55019d03bbc0`；构建源码 `33dd0025fa4b19b73bc801ae6c8fff75ae87fb7e`。v3 先验 SHA256：`df5b0c02c0566542b98607162d2c3bd61871fb0792950200f5ccb61b54754311`。官方 FANUC、完整工具、20/42.5 kg、5 mm 规则及配置均保留。只恢复已有资产原字节；以现有 M710_MOVEIT_ASSET_ROOT 指向原资产根，未修改数据库身份。prepare-only 的三个缺资源失败和成功准备记录均说明保留，没有试规划预热。

[实际命令](experiments/m710_top_row_retest_20261015/run-formal.sh)固定 seed=71070，static_prior_fast、slow-completion，共享 FAST=1 s，stage/box/batch=90/300/900 s，IPC=100 s，先验单查询原有上限 6 s/128 边。执行方式为既有 rootfs 内 `bash /tmp/top-row-retest-20261015/run-formal.sh`，复现需使用新输出目录，本轮未重跑。

[机器汇总](experiments/m710_top_row_retest_20261015/summary.json)、[逐箱 CSV（含实际 q）](experiments/m710_top_row_retest_20261015/boxes.csv)、[冻结身份](experiments/m710_top_row_retest_20261015/freeze.json)、[运行后身份核对](experiments/m710_top_row_retest_20261015/post-run-identity.json)、[原始轨迹](experiments/m710_top_row_retest_20261015/formal/trajectories.json.gz)；同目录保留原始 timing、请求诊断、worker 日志、依赖和失败记录。汇总可运行：

```sh
python tools/summarize_m710_top_row_retest.py docs/experiments/m710_top_row_retest_20261015/formal --output /tmp/retest-summary
```

6 项相关计时/去重定向测试通过（0.25 s），加本次只读边界核查；未运行全量测试。历史 29.687 s 使用 Xeon Platinum 8470Q，与本轮硬件/环境不同，并非本轮同条件 A/B，不计算三轮优化端到端因果倍数，不外推固定片段 3.02 倍。

跳过 Python 独立密集复检、原生重复输出复检、全路径/TCP/来源重型审计、执行包、preflight、动力学/力矩/jerk、Isaac 和真实接收；规划器必需的碰撞/限位、工艺权限、附着对象、正常采样和时间参数化保留。未运行真机、40 箱、104/129 历史任务或新的五箱批次；未评价执行资格或真实卸货节拍。本轮完成后停止。
