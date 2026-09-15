# 下一箱前瞻接触修复（2026-09-15）

## 基线与根因

操作前本地 HEAD、远端 `feat/v0.5-feasibility-core` 均为
`e9b4d2f0234fdeed454ce6270bd00780836eb822`，工作区干净；未回退。
GPU 服务器隔离目录为 `/root/autodl-tmp/m710-lookahead-contact-fix-20260915`。
原生产 `layout_trajectory.py` 的 SHA256 为
`a5cc534001dabbc349906a6e47b783ba6a0b9c50027ef306aabc9a5e0e19274c`。

两项根因均确认：`next_contact` 被拼接成未获接触许可的
`next_contact_ik_endpoint`；候选杯选择在 IK 过滤成功以后才执行，过滤阶段使用已有 validator mask。
原缓存已经包含阶段、mask、目标姿态与邻箱集合；问题不能归结为“缓存完全没有 mask 键”。

使用保留的首箱 `carton_l07_c02/front` 接触构型及其近距离、无碰撞自由接近起点，
完整官方 FANUC 模型和 Coal 检查确认正常 `contact_endpoint` 合法。
即使预先绑定正确的 40 杯 mask，原 IK 阶段仍首先被
`tool_compliant_bellows_0 / carton_l07_c03` 拒绝。

| 同一最小输入 | 修复前 | 仅阶段/mask 修复后 |
|---|---|---|
| `_next_contact_cost()` → 真实 IK 终点语义 | `next_contact_ik_endpoint` | `contact_endpoint` |
| IK 终点杯来源 | 已有 validator 上下文 | 候选 q 的 FK、当前目标姿态、面、suction |
| 终点检查 | `RIGID_TOOL_COLLISION` | 通过，40 杯 |
| 前瞻结果 | `NO_NEXT_CONTACT_IK`，代价 `None` | `CHECKED_NEXT_CONTACT_CONNECTION` |
| 实际计算的关节路径长度 | 未知 | `0.052298760333683504 rad` |

这里的最小 fixture 只使用已知近邻 IK seed，`random_restarts=0`；
生产默认 IK 参数和前瞻单次 4 s、请求累计 12 s 均未修改。
另保留默认 12 次随机重启的结果：阶段/mask 修复后终点已通过，但后续
pregrasp 分支搜索在约 4.012 s 截止。该独立失败为 `STAGE_IK_DEADLINE/pregrasp`，
代价仍为 `None`，不证明物理不可达，本轮不修调度或预算。

## 实现与隔离

- `_ik_stream()` 仅把明确的 `next_contact` 映射到已有 `contact_endpoint`。
  未知阶段、其它 `*_ik_endpoint`、自由 pregrasp 不获得目标接触权限。
  回调先复用 `_contact_selection()` 计算完整密封环和命令 mask，再调用原 `_state_failure()`。
  覆盖失败与精确碰撞原因、候选 q、两种规划 mask 写入 `endpoint_checks`。
- `_contact_selection()` 绑定独立不可变命令 mask，并绑定目标身份与邻箱集合。
  `_next_contact_cost()` 用相同候选上下文检查接近路径，再重算最终接触并检查终点。
  若最终 mask 改变，按原 free/contact 边界重新检查整条接近路径。
- 新增小型 `_contact_context()`：每个候选及每次 IK 回调分别深拷贝保存
  validator mask、邻箱集合、目标身份和 connector 邻箱集合，均在 `finally` 恢复并清状态缓存。
  成功、拒绝、路径失败、超时、异常退出使用相同作用域；外层 `finally` 恢复原 deadline。
- `_state_failure()` 保留完整缓存键，增加绑定目标身份，并支持 numpy mask，
  避免数组布尔求值。Exact validator 的静态缓存只覆盖与杯许可无关的固定环境检查，
  动态箱体许可仍逐次检查。
- 不写 Isaac 的 `actual_contact_mask`，不生成附着事件。
  未计算代价仍为 `None`；前瞻仅供评分，执行仍须从同一世界实测状态重新规划。

## 本轮追加的用户策略

用户随后明确指示：**“本轮和以后都不考虑柔性杯—邻箱碰撞”**。
因此新增 `compliant_cup_neighbor_contact_mode`，并在活动布局配置设为 `ignore`、写入 `AGENTS.md`。
`ExactM710LayoutStateValidator.__call__()` 和 Isaac 的
`classify_compliant_cup_contact()` 对已知柔性杯与具名非目标堆垛箱不再因
命令状态、阶段或压缩深度拒绝。此项是追加授权，不是让原阶段修复用例通过的前提；
回归分别在原 `check` 与新 `ignore` 策略下得到相同有限代价。

当前目标身份单独绑定，避免 pregrasp 没有 `target_contact` 参数时把当前目标误认为邻箱。
目标密封/压缩、刚性插芯、刚性工具、机器人及墙/地面/输送机/未知物体规则不变。
Isaac 保留物理碰撞响应，只调整接触验收分类；未宣称邻箱接触已物理合格。
旧策略仍可显式回归，历史运行报告与录像不改写；策略指纹变化使旧计划不能冒充新策略证据。

## 验证与未验证范围

全部数值和规划检查在 GPU 服务器原 CPU 环境执行：
`/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python`，
Pinocchio **4.1.0**、Coal **3.0.3**，官方模型固定提交
`fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`，构建时执行现有模型/工具审计。

```bash
export PYTHONPATH=$PWD/src
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
$CPU -m pytest -q -s tests/test_lookahead_contact.py tests/test_compliant_cup_contact_policy.py tests/test_contact_simulation_policy.py tests/test_layout_trajectory.py
```

结果：**73 passed in 1.46s**。先行的三文件定向检查为 61 passed。
覆盖真实生产入口、40/32 杯不同候选、错误面/过压、刚性碰撞、未知阶段/pregrasp 拒绝、
旧邻箱有限接触规则、新邻箱豁免边界、numpy mask/阶段/邻箱/目标缓存隔离，
以及成功/失败/超时/异常后的可变上下文恢复。
成功用例没有替换 IK、杯选择、碰撞或接近路径为成功桩；仅隔离退出测试注入失败或异常。

输入、前后结果、独立超时及源码哈希见
[机器证据](evidence/m710_lookahead_contact_fix.json)。服务器保留原探针脚本和日志。
未运行全量 pytest、104/129 任务集、整排、Isaac 动态试验或录像；
未处理候选调度、第五箱恢复、路径简化、J6、释放策略或连续计时。
