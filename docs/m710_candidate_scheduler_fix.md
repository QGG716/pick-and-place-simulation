# 接触候选调度修复（2026-09-16）

## 基线与范围

本地、远端均从 `a082c8a9a0b77cc2682a1c16b704f575ae8e8430` 开始，工作区干净。
未回退；本轮只改枚举、队列、重试身份/seed 和相应统计。
前瞻接触/mask 代码、碰撞策略、路径简化、计时、释放策略均未改动。
活动配置的 `ignore` 值保持不变；前后枚举使用同一实际场景、官方模型与策略。

## 根因及对照

直接提取并执行基线 `run_layout_single_carton_audit()` 的原始 AST 语句
（`families` 到 `rounds`），没有另写一个模拟旧循环。
12 个 face/roll 家族各含 4 个不同位置的姿态，和新调度回归使用同一 fixture：

| 调度 | 生成 | 实际访问次数 | 不同候选 | 重复 | 未访问 |
|---|---:|---:|---:|---:|---:|
| 原三轮重复 `breadth[:12]` | 48 | 36 | 12 | 24 | 36 |
| 新队列，上限 36 | 48 | 36 | 36 | 0 | 12 |
| 新队列，上限 48 | 48 | 48 | 48 | 0 | 0 |

旧循环的 seed 不含轮次，确实原样重建随机搜索。
此外 `_scheduled_contact_poses()` 在分家族前已用 `grasp_poses_per_task=48` 截断。
本次实际输入按现有面、roll、稀疏偏移/倾斜配置可生成 **108** 个姿态，
旧生成入口只保留 48 个；没有新增任何偏移值、倾斜值或法向微调。

## 实现

- 新增小型 `contact_scheduler.py`，由生产 audit 主循环直接消费。
  家族按原优先级首次出现的顺序轮转，内部游标持续推进；耗尽家族跳过，空池正常结束。
  完整池保留，未访问队列耗尽后才处理有限重试队列，每个候选最多重试两次。
- `family_id` 标识目标/face/roll；`candidate_id` 用现有 `canonical_digest`
  绑定目标姿态/尺寸、完整接触位置/朝向、变体工艺信息、场景和策略指纹；
  `attempt_id` 再区分重试次数。精确重复才去重，不取整，小到 `1e-12 m` 的变动仍可区分。
- 首次 IK/path seed 分别由 request seed、candidate ID 和用途稳定派生；重试增加确定性偏移。
  IK seed 传入 `_audit_pose()` 的真实 IK RNG，path seed 传入 `plan()`，继续传向连接规划。
  不复用 IK 解或 RRT 树；每次仍重新经过原覆盖与精确终点检查。
- 固定覆盖/接触几何失败不原样重试。`NO_IK`、IK 超时保留有限搜索失败语义；
  路径失败可以有界重试，并记录新 IK/path seed，不能当成物理不可达。
- `task_pose_connection_attempts` 保留为兼容的批次大小，默认 12；
  显式属性 `task_pose_batch_size` 与 `task_complete_connection_attempt_limit` 分开表达，后者仍为 **36**。
  `grasp_poses_per_task=48` 现在约束评估次数而非生成前缀；实际总尝试上限取它与 36 的较小值。
  完整连接计数指 `plan()` 调用数，另保留内部 RRT/分支连接计数。
- 沿用 20/50/100 s 批次切片、3 s IK 上限和既有总 deadline/最终预留；默认 900 s 不变。
  IK 返回后再检查总 deadline，截止后不发起新的完整连接，也不重置请求时钟。
- 每次记录三种身份、变体、双 seed、切片预算、耗时、失败阶段和重试原因。
  按家族输出生成/访问/剩余数，区分池耗尽、尝试上限、总截止和完整轨迹成功。
  未枚举的后续箱单独统计，不冒充未搜索候选数为零。
  严格 IK 解按已有关节等价规则在任务内去重，原始跨尝试记录另列，避免重复记成新增独立解。
- 新模块加入 `MOTION_IMPLEMENTATION_FILES`；补充现有配置字段校验和预算证据。
  配置仅增加语义注释，没有更改数值或碰撞配置。历史报告未改写。

## 最小验证

所有检查在 GPU 服务器原 CPU venv 执行，Pinocchio 4.1.0 / Coal 3.0.3；
官方模型固定提交 `fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`。

```bash
export FIX=/root/autodl-tmp/m710-candidate-scheduler-fix-20260916
cd "$FIX"
export PYTHONPATH=$PWD/src
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
$CPU -m pytest -q --basetemp="$FIX/test-evidence" tests/test_contact_candidate_scheduler.py tests/test_contact_pose_action_prior.py tests/test_lookahead_contact.py
```

最终 **30 passed in 2.57s**。覆盖 48/36 次预算、长度不等/空家族、精确去重、稳定双 seed、
有限重试、新姿态优先、覆盖失败不重试、deadline/连接次数上限、成功即停止、统计与调用一致。
上一轮真实前瞻入口、mask 绑定、上下文恢复和缓存隔离回归全部保留并通过。

生产接线测试调用真实 `run_layout_single_carton_audit()`：前 12 个姿态失败，
第 **13** 个变体的实际 pose 和 candidate ID 进入评估与 `plan()`，成功后立即停止。
昂贵求解边界使用可控结果，**这不是机器人规划成功证据**。
重试测试还捕获真实 `_transit()` 构造的 RRT RNG 状态，确认三次下发 seed 不同且对应日志；
该测试使用零距离连接隔离 seed 接线，不宣称完成取放。

## 唯一一次真实场景检查

运行 `$CPU check_actual_candidates.py`，脚本及完整日志保留在上述服务器目录。
输入原 `m710-ideal-outfeed-20260915-retry02/isaac/segment_004/actual_remaining_state.json`，
目标为原第五箱 `carton_l07_c04`，未使用替代状态，未移动任何箱体或修改关节限制。
旧/新枚举比较共享同一个 scene、policy 和官方模型；只执行一次新代码规划请求，
没有进行新旧机器人搜索性能对比。

请求上限 **60 s**，沿用最终预留后约 **57.025 s** 结束：

- 生成 108 个候选，实际评估 **7** 个不同候选，重试 **0**，完整连接调用 **3**。
- IK 实际尝试 72 个 seed、11127 次迭代，12 条任务内不同严格 IK；
  精确终点检查共 **24** 次，通过 24 次，原杯覆盖、FK、关节与碰撞检查保留。
- **101** 个候选未访问，终止为 `PLANNING_WALL_CLOCK_DEADLINE`。
  前两个耗时连接各用约 20 s，第三个消耗剩余预算。
- 实际访问仍在旧首批前缀内，**没有在这次短检查中实测到后续变体**；
  不能把已排入队列当成已评估，也不宣称第五箱恢复成功。未追加第二次实测。

精简输入、访问对照、生产调用、seed、统计、源码/状态指纹见
[机器证据](evidence/m710_candidate_scheduler_fix.json)。完整候选池、日志和测试接线 JSON 留在服务器。
未运行全量 pytest、104/129、整排、Isaac 或录像；未处理连接搜索耗时、J6、路径平滑、释放微调等问题。
