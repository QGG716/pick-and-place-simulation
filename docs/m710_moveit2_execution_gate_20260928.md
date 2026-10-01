# MoveIt 最终执行参考 TCP 正式门控（2026-09-28）

本轮基线 `153f769de53d24233c24e068170b8553d934e4d5`；开始时 HEAD 与基线一致、工作区干净。仅开发 `feat/v0.5-backend-moveit2-mtc-pilz`，未合并其他分支。目标已完成：原始 LIN 合同进入计划与执行包，正式导出和实际加载入口强制检查最终参考；本轮原生规划调用 0、真实 Isaac 启动 0。

## 实际调用链

1. `MoveItLayoutConnector._native_plan` 从原始请求的起始 q/FK、目标 pose、完整 flange→TCP、模型/工具/策略/TCP 身份以及当前容差生成 `m710_task_tcp_lin_v1`。每个 LIN 成功记录带稳定 `stage_id`；完整计划保留 native record 的 `path_range`。`native_timing_floor` 对整段原始路径存在多个相同匹配时明确拒绝，不再拿首次命中冒充唯一阶段。
2. `build_fanuc_isaac_replay_bundle` 保护 LIN 阶段边界，执行原共线节点合并、时间下界、停到停 C2 时间处理及吸附/释放停留。新增 `source_path_indices` 随导出及每次 hold 插入传递，因此最终参考中的重复 q/重复 hold 可以按源索引区分。`lin_reference_bindings` 把 stage ID、规划范围、最终参考范围和合同哈希绑定，不搜索最后一个相同 q。
3. 最终 `joint_reference` 确定后，builder 调用 `m710_execution_tcp.audit_bundle_tcp`，审计实际执行器遍历的关节边。正式 `scripts/export_isaac_fanuc_replay.py` 在写出前再调用公共 `verify_m710_replay_bundle`，检查当前工作区身份、既有合同与最终 TCP 语义。
4. `scripts/isaacsim_fanuc_replay.py` 原有启动前公共调用现在包含同一审计，因此在创建 SimulationApp、记录物理 run started 之前可以拒绝无效包。其后资产校验和 continuation 现有调用也仍经过公共入口。没有修改 Isaac 控制器、物理模型或监督脚本，也没有调用 task.execute()。
5. `tools/check_m710_moveit_execution_tcp.py` 只是同一公共入口的离线 CLI，不再实现第二套审计。正式执行无需人工先运行它，更不读取它生成的 PASS 文件。

Isaac 原入口在 Kit 启动前以独立模块方式加载标准库门控。为避免在 Kit 主进程预先导入 NumPy/FK 依赖，该路径用当前 Python 的隔离子进程读取同一个内存包、进行同一审计，再返回结果；子进程不导入 Isaac，不加载 ROS，不规划，不接触物理世界。优化标志传递给子进程。普通包内调用直接复用函数。异常、非零退出、超时或无法解析结果均停止入口。

## 合同、身份与边界

合同保存 world 参考系、原始任务 TCP 起终位姿、原始 q_start、完整 TCP 固定变换、原生模型/工具/场景/策略/TCP 身份、官方 URDF 哈希及安装变换、原始位置/姿态容差、边分辨率与共同进度语义。元数据同时保存 `stage_id`、`path_range`、`reference_range`。起点 pose 从请求 q 的官方 FK 得到；目标和容差不从输出路径反推。

审计保留既有 0.0001 m / 0.0002 rad 容差和原保守关节边网格；平移使用线段投影进度，纯旋转使用最短旋转轴进度。检查端点、整段线偏差、姿态/位置共同进度、单调性及范围；没有改成只看节点/中点，也不宣称严格连续扫掠证明。全部映射节点必须仍对应被预检绑定的源路径节点；hold 是显式源索引重复。

`m710_final_reference_tcp_v1` 的 binding SHA 覆盖最终参考（含时间与映射）、LIN 合同和记录、阶段范围、执行输入身份以及 TCP/FK 审计源码哈希。既有 bundle payload SHA、preflight 输入绑定、轨迹/事件、模型资产、时间限制和运行监测保留。每次加载实际包都重新执行语义审计；内嵌报告存在时还核对 binding SHA。外部 PASS 文件根本不是执行输入；更换包或合同不能凭旧报告放行。哈希仅表示内容一致性，不是机器认证、签名或授权。

缺合同、错 frame/TCP、缺边界、错规划范围、歧义、未知上下文、语义失败和过期报告均显式失败。正式检查无 assert 放行条件；正常 Python、`-O`、`PYTHONOPTIMIZE=1` 同样生效。真正无 native LIN 的 core/PTP 计划返回 NOT_APPLICABLE，保留原有验证语义。

当前 FK 上下文明确限定本项目固定 POC 模型、安装和 TCP。其他上下文不会以当前模型猜测通过；需要单独适配和证据。没有新增通用工具框架。

## 原始证据与显式派生

上一轮 `m710_moveit2_tcp_20260928/` 全部文件、`isaac-once.sh` 与 `executor-tcp.json` 不改写。旧包未自带 LIN 原始请求，因此正常新入口会报 `LIN_CONTRACT_MISSING_REQUIRES_MIGRATION_OR_REAUDIT`。

一次性 `--legacy-requests ... --derive-bundle 新路径` 仅用于显式补齐旧包：先核验原包完整性，精确匹配原请求指纹与 native 上下文，容差取原规划审计记录，q_start/目标/TCP 取原请求；生成带原文件 SHA、原包指纹、原 preflight 指纹、请求来源 SHA、原执行源码身份的派生副本。源运动/权威证据保留，不宣称新规划或新碰撞复检。只允许本轮执行参考接线涉及的源码身份刷新；其他碰撞、附着、监测源码变化明确拒绝迁移。输出采用新文件排他写入，不覆盖历史文件。

本轮 `derived-bundle.json` 的最终关节参考位置与旧包逐项相同，LIN 规划范围 `[143,160]`，最终参考范围 `[144,160]`（17 节点）；审计 617 个样本，最大线偏差 `5.822868602313926e-6 m`、姿态误差 `8.84683117979168e-6 rad`，均通过。差异是新增合同、索引映射、来源说明与当前执行检查身份，不是新求解出的完整计划。

## 定向离线结果

证据目录：`docs/validation/evidence/m710_execution_gate_20260928/`。服务器：`/root/autodl-tmp/m710-execution-gate-20260928`，使用上一轮 CPU venv；未重编原生 worker。

| 验证 | 结果 |
|---|---|
| 定向 gate、TCP、backend、既有 replay contract 与少量 core 导出/hold/时间下界 | 64 passed，16.07 s |
| `python -O` 关键语义检查 | 14 passed，4.77 s |
| `PYTHONOPTIMIZE=1` 关键语义检查 | 14 passed，4.61 s |
| 无请求日志的标准自检 | PASS，0.433123 s |
| 无请求日志的 `-O` 自检 | PASS，0.435127 s |
| 正式导出后的公共自检 | PASS，0.370661 s |
| 显式旧包派生（初次/限定迁移复核） | PASS，1.174201 / 1.184871 s |

优化模式的 pytest 提示“非测试模块 assert 被禁用”原样保留；生产门控使用明确条件/异常，不依赖这些 assert。上述耗时各有独立口径，不相加为整体性能指标。

负例覆盖：删除合同、改变 TCP/frame、删除/错配规划边界、删除映射边界、重复经过相同 q、审计后修改合同或替换参考、preflight 未准备好。中间节点偏移负例保留起终点，重新生成外层内容指纹并明确通过完整性入口，然后被 `LIN_TASK_TCP_CONSTRAINT` 拒绝；不是仅凭 SHA 不匹配拒绝。正确重算外层哈希也不能让坏 TCP 语义通过。正式 builder 对错误目标合同同样拒绝。

15 个真实监督入口子进程测试覆盖三种 Python 模式 ×（合法包、缺合同、中段错误、未 ready、旧 PASS）。测试通过替换 `isaacsim` 模块，把 SimulationApp 位置换成只写计数哨兵并立即退出的函数：合法包到达哨兵；12 个负例到达计数均为 0，且没有生成物理 run_status。实际 Isaac SDK/SimulationApp/World 都没有启动。这是入口接线验证，不是物理试验。

## 标准复现

在配置好的服务器 repo/CPU venv 中，正常新计划已包含 LIN 合同：

```bash
export PYTHONPATH=src
python scripts/export_isaac_fanuc_replay.py --preflight preflight.json --output replay-bundle.json
python tools/check_m710_moveit_execution_tcp.py --bundle replay-bundle.json --output bundle-check.json
```

第一条即包含强制自检，第二条仅供独立查看结果。正式监督入口依旧为 `scripts/isaacsim_fanuc_replay.py --project-root ... --bundle ... --usd-directory ... --output ...`，它自行验证实际加载包，不能用外部报告跳过。本轮不要执行该物理命令。

复核上一轮旧包时先生成新副本（该命令会拒绝覆盖已有输出）：

```bash
python tools/check_m710_moveit_execution_tcp.py \
  --bundle docs/validation/evidence/m710_moveit2_tcp_20260928/replay-bundle.json \
  --legacy-requests docs/validation/evidence/m710_moveit2_tcp_20260928/task-requests.jsonl \
  --derive-bundle outputs/tcp-gate/derived-bundle.json \
  --output outputs/tcp-gate/migration.json
python tools/check_m710_moveit_execution_tcp.py \
  --bundle outputs/tcp-gate/derived-bundle.json --output outputs/tcp-gate/self-check.json
M710_GATE_TEST_BUNDLE=outputs/tcp-gate/derived-bundle.json python -m pytest tests/test_m710_execution_tcp.py -q
M710_GATE_TEST_BUNDLE=outputs/tcp-gate/derived-bundle.json python -O -m pytest tests/test_m710_execution_tcp.py -k 'not actual_supervised' -q
```

最终源码/环境身份见 `source-manifest.json`、`packages.lock`；完整执行命令保留为 `validation-command.sh`。新派生包仍含来源身份，绝不冒充上一轮已执行过这个新包。

## 保留的结论与未覆盖项

上一轮固定历史候选完整几何任务 PASS；一次 Isaac workflow 完成；理想接收 1、理想出料 1；物理接收未验证；驱动力矩 NOT_EVALUATED，既不是超限也不是通过。固定历史候选不能代表无提示新任务成功率。本轮没有新增物理证据，没有实际力矩测量、控制器调整、物理接收改动或新规划性能结论。完整边碰撞权威结果沿用已保存计划的原证据，本轮只对最终执行参考及其绑定做新的离线检查。
