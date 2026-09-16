# 已验证结果与预算衔接修复（2026-09-16）

基线为 `90cfbaac8f82c11a2b680cf6d84e3a1e844f5f98`。开始时本地与 GitHub
的 `feat/v0.5-feasibility-core` 一致，工作区干净。服务器旧 runtime 工作区有未提交
修改，未覆盖；本轮使用 `/root/autodl-tmp/m710-feasible-budget-fix-20260916` 独立目录。
模型、配置、接触策略、候选队列/seed/上限、J6、释放运动、插值与录像均未更改。

## 基线复现

生产函数使用可控单调时钟，无 sleep。输入及精简结果见
[机器证据](evidence/m710_feasible_result_budget_fix.json)；原探针和日志保留在服务器。

| 调用链/输入 | 原行为 | 修复后 |
|---|---|---|
| `_transit`，当前截止 102，可选简化消耗到截止 | RRT 成功，调用方复查在首样本超时，路径不可用 | 先做原严格复查；短预算跳过改善，保留 B |
| `_transit`，改善耗尽专属窗口且改写工作副本 | 原始路径可被改写，尚未建立已验证回退对象 | 原路径独立保存，未完成复查/评分的替代路径不发布 |
| `_connect_pose` → `_approach`，第二个 IK 消耗 5 秒 | 首条连接成功后继续取 IK，必要接触失败 | 取第二个 IK 前保护必要接触窗口，实际执行末端检查 |
| 完整任务最终检查于 118.9 完成，返回于 119.1，搜索截止 119 | 外层返回 request 超时，覆盖成功 | 核对完成证据及相同内容/上下文，保留 D |
| 请求硬截止 120、搜索截止 119、最终预留 1 秒 | 119.2 的验证仍受搜索截止拒绝 | 仅完整候选的最终检查可使用硬截止前窗口 |
| `_remaining_wall_time() == 0.0` | `or inf` 给 RRT 下发 12 秒 | 不启动 RRT；None 与零明确分开 |
| 单次必要验证从 100 执行到 103，截止 101 | 最后一次验证返回通过 | 返回原阶段超时，不登记完成 |

## 实现与边界

- 请求硬截止只在原入口设定一次，默认 900 秒与既有 `min(15, 5%)` 总预留不变。
  候选/分支的搜索和最终处理均受原父级窗口限制；不会按嵌套次数追加预留。
  完整轨迹构造后才进入最终窗口，那里只运行原最终阶段/事件/杯证据合同检查和
  内容/上下文绑定。搜索截止后不启动新的 IK、RRT 或改善。
- 普通预算作用域只能缩短 deadline，所有临时切换均在 `finally` 恢复。
  前瞻仍受单次 4 秒、请求累计 12 秒及父级窗口约束，无权借最终预留。
- 必要后续工作的保留量为 `min(3 s, stage_wall_time_s)`，从当前父级剩余时间中扣出。
  这是有界调度余量，不保证复杂接触一定完成。简化和局部连接比较另受原
  `postprocess_wall_time_s` 上限约束；完整任务后的放置/撤离比较使用父级剩余窗口，
  保留旧少量候选比较与原 30 秒 release 比较条件。
- `_transit` 先运行原 `_path_failure` 全边采样，保存不可变路径数值，再在独立副本上
  简化。RRT/shortcut 与调用方的两套不同采样网格全部保留。新方案必须完成原复查、
  保持端点并取得完整且更好的原 soft score 才替换。回退不重复检查原路径。
- FK、SO(3)、Jacobian 评分也受可选 deadline 约束；未完成用 `None` 和明确状态表示，
  不视作零成本。仅捕获专用 `QualityDeadline`；验证异常继续向上传播，不能伪装成功。
- 保留记录绑定本请求、候选、目标、场景实体、模型/工具、策略、mask、附着、阶段权限、
  起终点和阶段范围。上下文改变则拒绝复用。完整任务另绑定内容摘要和私有完成记录；
  单有 segment 或一个 passed 布尔值不构成完成证据。
- 截止边界统一为 **完成时间严格小于有效 deadline**。必要验证到达/超过截止失败；
  已合法完成后的统计或返回延迟单独记录，不改写为无解。保留原失败阶段和原因，
  候选/分支预算终止作为附加信息。实际 RRT/边检查计数沿用原计数器。

验证层级为 A 未完成复查、B 严格局部连接、C 杯覆盖与接触检查完成的完整接近、
D 全部必要规划验证完成的任务。E 仍由独立执行预检与导出决定。
`plan()` 和 motion audit 均明确 `execution_ready=false / INDEPENDENT_PREFLIGHT_NOT_RUN`。
旧字段 `final_export_reserve_s` 保留兼容，但明确其实际范围只含本次完整轨迹验证和
进程内绑定，**不宣称覆盖独立 CLI 的预检、序列化、导出或 Isaac**。
现有 `prepare_m710id70_dynamic_execution.py --motion-result ...` 和导出器读取同一绑定结果，
继续执行原预检/来源/配置/场景/轨迹校验，无须重新搜索；不跨进程传递绝对单调时钟。
不传 `motion_result` 的程序化预检接口仍会独立规划，属于新请求，不计作旧请求的交付。

## 验证

所有数值/规划测试在 GPU 服务器原 CPU venv 执行：

```bash
cd /root/autodl-tmp/m710-feasible-budget-fix-20260916
export PYTHONPATH=$PWD/src
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
$CPU -m pytest -q --basetemp="$PWD/delivery-verification" --junitxml="$PWD/delivery-tests.xml" \
  tests/test_feasible_result_budget.py tests/test_lookahead_contact.py \
  tests/test_contact_candidate_scheduler.py tests/test_contact_pose_action_prior.py \
  tests/test_motion_quality.py tests/test_layout_trajectory.py \
  tests/test_m710_replay_contract.py::test_random_hash_and_self_reported_ready_cannot_forge_preflight \
  tests/test_m710_replay_contract.py::test_blocked_preflight_has_no_exportable_trajectory \
  tests/test_m710_replay_contract.py::test_ready_preflight_contract_and_bundle_round_trip
```

最终结果：**88 passed in 3.24s**。定向测试覆盖回退独立副本、捷径内部障碍、成功改善仍被采用、第二连接择优、
必要接触失败、完整任务的缺证据/迟到/恰到截止、最终预留、上下文变化、异常恢复、
前瞻上限，以及完整放置方案的进一步择优和超时回退。两个历史合成成功桩补齐
新完成记录；原候选顺序、数量、seed、惰性访问断言不变，不视为实际规划成功。

官方模型检查复用前轮保留的近距离 START_Q/CONTACT_Q、`carton_l07_c02`，
只省去该已知 seed 的随机 IK 重启。官方固定提交
`fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`，Pinocchio 4.1.0 / Coal 3.0.3，
活动 `ignore` 策略不变。真实生产 `_next_contact_cost` → `_approach` → `_connect_pose`
→ `_transit` → 末端接触与最终杯检查全部运行，FK/IK/Coal/完整密封环未替换为恒真桩。
仅把可选 shortcut 的时钟推进到自身截止，验证回退后仍完成 C：**40 杯**，
关节路径长 **0.052298760333683504 rad**，接近完成时刻为注入时钟 **101**，
单次前瞻截止 **104**。没有生成完整取放或 E 层级就绪结论。

没有运行全量 pytest、104/129、第五箱随机搜索、整排、Isaac 或录像；不声称物理执行成功。
未新增生产模块或改变配置合同；四个改动的生产文件均已在原实现指纹清单内，
当前输入会重算源码哈希，历史证据不回写。
