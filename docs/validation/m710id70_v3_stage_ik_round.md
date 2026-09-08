# M-710iD/70 V3 阶段 IK 多解连接验证

日期：2026-09-08

分支：`feat/v0.5-feasibility-core`

审查基线：`839dc2dc5cd63ce4311f07dde9cd057fe8de41ce`。该提交相对计算代码 `66676207225533b869357a7d033be151c71f52ac` 只增加报告；基线归档中的代码、配置、模型资产、运行时和任务集合身份可复核，因此没有因文档提交重复生成同一基线。

本轮按三个独立提交推进：

1. `9b1d040`：只补证据归档与全尝试搜索统计，不改变搜索行为；
2. `a52760e`：修正预抓取和交接阶段有效性，不接入多解；
3. `47a12d8`：接入有界、惰性的阶段 IK 多解连接与局部回溯。

本轮没有引入新规划器，没有改动 ROS、视觉、动态传送带、升降轴、连续 129 箱或时间参数化。原始场景、104 分母、碰撞余量、接触容差、IK/FK 残差、关节限制、吸盘覆盖和负载曲线均未放宽。

## 1. 可复核基线和统计口径

上一轮固定传送带 104 项的轻量证据保存在 `docs/validation/evidence/m710id70_v3_hardening_baseline/`，包含修改前后 104 行、逐任务比较、10 个成功任务、有效配置、代码/资产/运行时身份、命令和 `grid_022` 代表轨迹。没有使用或覆盖未更新的历史 `m710id70_v3_evidence.zip`，也没有提交 `.tmp` 中的大体积任务缓存。

本轮归档保存在 `docs/validation/evidence/m710id70_v3_stage_ik_round/`。归档脚本验证：前后各 104 行、104 个唯一任务 ID、任务集合完全相等。任务成功只按唯一任务 ID 计数；同一任务的多个候选只进入候选级统计。

旧结果中的 35 个 `NO_IK` 是抓取求解路径的最终失败，不是本轮预抓取或交接搜索的预设恢复对象。基线任务分层保持为：

| 任务层级 | 数量 |
| --- | ---: |
| 抓取有效但未完成脱垛 | 26 |
| 已完成脱垛但未完成周期 | 16 |

本轮结果中的阶段失败统计如下。任务数表示至少一个候选在该阶段失败的唯一任务数；候选数允许同一任务出现多次。

| 阶段 | 失败候选数 | 涉及任务数 |
| --- | ---: | ---: |
| approach | 48 | 19 |
| contact | 0 | 0 |
| handoff | 58 | 15 |
| carry | 9 | 5 |
| place | 0 | 0 |
| withdrawal | 0 | 0 |

全 104 项记录的实际搜索量为：13,249 次 IK 调用、37,783 个实际尝试种子、4,200,877 次 IK 迭代、16,634 个收敛姿态结果、7,172 个有效解、7,149 个去重候选、232 次路径连接、12,868 次 RRT 迭代、20,570 次扩展、197,373 次状态验证、20,802 次边验证，以及 573 次全部候选范围的 escape 机器人验证。旧 `compact()` 中的 `escape_robot_validations` 继续明确标注为“仅 selected 方案”；新增的 `escape_robot_validations_task_total` 才是任务全部成功和失败尝试的总数。

## 2. 阶段有效性

预抓取候选使用真实未附着状态，目标箱仍作为普通障碍；没有提前建立 `RigidAttachment`，也没有使用目标接触例外。交接候选使用当前分支路径产生的真实附着关系，并向状态/路径检查传入接收台面名称；只有既定支撑容差内的台面接触可通过，台面和目标箱没有从场景删除，超过容差的穿透仍以 `PAYLOAD_COLLISION` 拒绝。

回归证明同一交接关节状态在候选筛选和路径终点检查中结论一致。合法支撑接触通过，抬高台面制造的超容差穿透被拒绝。最终放置仍执行实际箱体位姿的 `support_audit`，撤离仍为空载并保留已放置箱体。

## 3. 惰性多解和连接回溯

`IKCandidateStream` 一次性建立确定性显式种子和随机种子流，并在下游连接失败后从未消费的种子继续。它不会重新调用旧接口而反复返回首解。候选必须先通过严格 FK 残差、关节限制和阶段真实碰撞语义；去重区分有界旋转关节、连续关节和移动关节的弧度/米单位。

每次阶段连接都从该分支的真实路径终点开始。两个相同 TCP 的关节解不会直接切换。实际路径终点必须与所选候选一致，否则以 `CONNECTION_ENDPOINT_MISMATCH` 拒绝。接触路径结束后仍重新执行最终接触闭合验证，并只为该分支创建附着关系。

默认限制为每个预抓取或交接子问题最多 3 个去重候选、3 次连接尝试，全部候选共同分享 800 次 RRT 迭代。分配按剩余预算和剩余槽位确定性计算，未给每个 IK 候选复制一份 800 次预算。原有 `escape_path_attempt_limit=4` 仍是每个下游候选调用的范围，不被改成任务级总预算。

真实 M-710 回归固定了两个严格有效的 handoff 关节分支：第一分支在实际碰撞路径搜索中耗尽 400 次迭代，第二分支从同一个 extraction 终点连接成功；总消耗 406/800，连接首尾、附着身份、障碍身份、支撑接触和最终路径均重新验证。

原始 104 项中观察到 4 次非首候选连接成功：`grid_020` 的 3 个预抓取候选尝试，以及 `grid_041` 的 handoff。`grid_041` 的第一候选连接在路径内部遇到 `SINGULARITY`，第二个去重构型从同一起点成功，阶段共消耗 330/800；该任务最终仍为完整几何成功。

## 4. A/B/C 最小消融

消融使用同一代码路径、原始 `grid_022` 和 `grid_045`、相同任务种子与全部严格判据，仅切换：A `legacy_single`、B `filtered_single`、C `multi_solution`。

| 任务 | A | B | C |
| --- | --- | --- | --- |
| `grid_022` | 完整成功 | 完整成功 | 完整成功 |
| `grid_045` | carry 路径耗尽 | carry 路径耗尽 | carry 路径耗尽 |

这两个代表任务没有任务级结果变化，因此不能把它们解释为成功率提升。阶段筛选的作用由专门一致性/穿透回归隔离，多解连接能力由真实双分支回归和全 104 项中的非首候选事件隔离。

计算量不能只按配置数字宣称“等预算”。实际消融数据如下：

| 任务 | 指标 | A | B | C |
| --- | --- | ---: | ---: | ---: |
| `grid_022` | RRT 迭代 | 363 | 363 | 363 |
| `grid_022` | IK 种子 | 415 | 415 | 415 |
| `grid_022` | 墙钟秒 | 104.65 | 90.99 | 90.17 |
| `grid_045` | RRT 迭代 | 2,400 | 2,400 | 801 |
| `grid_045` | IK 种子 | 359 | 359 | 371 |
| `grid_045` | 墙钟秒 | 85.39 | 84.42 | 44.82 |

墙钟仅是同机实测，不作为稳定性能基准。`grid_045` 显示 C 增加了 12 个 IK 种子，但共享连接预算避免把三个失败连接扩大为 2,400 次 RRT 迭代。

## 5. 原始固定传送带 104 项对比

命令：

```powershell
.venv\Scripts\python.exe tools\run_m710id70_v3.py `
  --phase grid-fixed --workers 12 `
  --output-dir .tmp\review_stage_ik_post
```

| 指标 | 上一轮 | 本轮 | 变化 |
| --- | ---: | ---: | ---: |
| 原始任务分母 | 104 | 104 | 0 |
| grasp reachable | 52 | 52 | 0 |
| extraction feasible | 26 | 26 | 0 |
| 完整几何成功 | 10 | 10 | 0 |
| 新增成功 | - | 0 | - |
| 丢失成功 | - | 0 | - |

成功集合完全一致：`grid_022, grid_023, grid_025, grid_029, grid_031, grid_040, grid_041, grid_043, grid_047, grid_049`。全部为可重复的 top 抓取完整链路，其中 5 项执行标准 support-release primitive。

21 个任务的最终分类发生变化：

| 上一轮分类 | 本轮分类 | 数量 |
| --- | --- | ---: |
| approach / `SINGULARITY` | extraction / `TOOL_SELF_COLLISION` | 1 |
| approach / `TOOL_SELF_COLLISION` | approach / `PREGRASP_NO_IK` | 5 |
| carry / `ROBOT_COLLISION` | handoff / `HANDOFF_NO_IK` | 15 |

后两类是阶段有效性筛选把非法端点在连接前拒绝后的更准确归类，不是几何判据放宽。`grid_020` 则由替代预抓取构型越过旧的 approach 分支，但随后仍被严格 extraction 工具自碰撞拒绝。

当前最终失败原因是：35 `NO_IK`、17 `GRASP_CONSTRAINT_FAILED`、15 `HANDOFF_NO_IK`、8 `PAYLOAD_PROXIMITY_NOT_RELEASED`、8 `PATH_SEARCH_EXHAUSTED`、5 `PREGRASP_NO_IK`、4 `TOOL_SELF_COLLISION`、1 `PAYLOAD_PROXIMITY_WORSENED`、1 `ROBOT_COLLISION`。

上一阶段的 P0 验收信息保持可复核：原 24 个 `PAYLOAD_INITIAL_CLEARANCE_FAILED` 均通过新的初始近接登记，其中 20 项实际开始分离、11 项恢复正常工程余量、10 项完整成功；通过初始门禁不被计作完整成功。原 44 个 `GRASP_CONSTRAINT_FAILED` 中 27 项通过严格 task-set grasp 获得抓取，剩余 17 项仍失败。原 36 个抓取 `NO_IK` 中只有 `grid_038` 获得严格抓取，但随后在预抓取阶段失败；本轮没有把这 35 个当前抓取 `NO_IK` 预先计作可恢复对象。

负载资格继续独立记录，几何探索不会因 `PAYLOAD_CG_FAIL` 短路，也没有修改 FANUC 曲线使其通过。动态传送带与 base Z/lift 优化继续保持 P1 暂缓。

## 6. 回归与证据

```powershell
.venv\Scripts\python.exe -m pytest -q --basetemp .tmp\pytest-all-final
# 299 passed, 4 deselected in 60.13s

.venv\Scripts\python.exe -m pytest -q -m slow `
  --basetemp .tmp\pytest-stage3-real `
  tests\test_v3_motion_contract.py `
  -k "actual_contact_endpoint_outside_coverage or real_handoff_uses_second"
# 2 passed, 22 deselected in 50.51s

.venv\Scripts\python.exe -m pytest -q -m slow `
  --basetemp .tmp\pytest-stage3-witness `
  tests\test_v3_motion_contract.py -k original_grid_022
# 1 passed, 23 deselected in 56.69s
```

新增的任务链集成回归让真实 contact Cartesian 路径终点越过吸盘覆盖边界，`evaluate_task()` 返回 `FINAL_CONTACT_COVERAGE_FAILED`，且没有创建 `contact_state` 或 `tcp_from_box`。该测试消费真实 FK/覆盖判定，不是辅助函数布尔 mock。

轻量归档约 0.46 MB，包括前后 104 行、逐任务比较、A/B/C 消融、任务级与候选级搜索统计、有效配置、模型指纹、验证策略、种子、运行环境、代码内容摘要、命令和代表成功轨迹。`.tmp` 中的大体积逐任务缓存不提交。

## 7. 尚未完成

- 本轮没有提高完整成功数；结果仍为 10/104，但严格阶段语义、替代构型连接和成本归因已建立。
- front/side 尚无完整成功，已有可重复 top 成功集合满足当前“front、side 或 top 至少一种”验收。
- 15 个 `HANDOFF_NO_IK`、5 个 `PREGRASP_NO_IK`、8 个 `PATH_SEARCH_EXHAUSTED` 仍是有限搜索下未确认项，不是不可行性证明。
- 连续 129 箱、动态传送带、base Z/lift range 和时间优化没有重启。
