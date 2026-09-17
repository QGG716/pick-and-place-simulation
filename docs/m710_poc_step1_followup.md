# POC step 1.1：续箱状态语义补齐（2026-09-17）

基于 `db1b2997b2516b8a3631f9c941a1b983d143458d`。
开始前 fetch 确认本地/远端一致、工作区干净；服务器无运行中的规划、pytest 或 Isaac。
使用独立目录 `/root/autodl-tmp/m710-poc-step1-1-20260917` 和既有 CPU venv，未重装依赖。
交付提交身份见本报告所在 Git 提交；测试源码的 Git blob 身份保存在精简证据中。

## 两项修复

1. `_build_automatic_trajectory_connector()` 原来只检查实际接收集合，因此 POC 已理想处理一箱时
   仍选首箱 600 次连接迭代。现在通过共享的状态校验读取工作流处理集合：验证快照指纹、状态来源、
   原始身份、接收记录来源、显式集合一致性和剩余箱体集合，然后选择既有续箱 1800 次参数。
   不直接信任 `processed_carton_ids`，不将理想接收写入 `completed_carton_ids`。
   真正首箱仍为 600；旧物理接收续箱仍为 1800。没有改变任何迭代参数数值或增加墙钟截止。
2. `_apply_motion_state()` 原来仅用合并集合检查旧实际完成事件，理想事件可以消失，实际/理想来源可以互换。
   现在分别保持实际接收、理想接收、已处理集合单调，并明确拒绝实际与理想来源改写。
   已送出事件及活动身份仍按既有规则防止回退或复活。错误输入直接报错，不补造事件。

缺少新增字段的旧物理接收状态仍兼容；存在理想接收时必须显式提供一致的理想/已处理集合。
已有理想事件不能借“旧格式兼容”消失。重复合法输入保持事件计数、活动身份、待抓集合幂等；合法新增事件允许。
理想接收但尚未送出的箱体保留活动身份并排除在待抓集合之外。
本轮没有引入来源转换规则，因此实际与理想事件之间的改写均拒绝。

## 定向验证

所有数值检查在服务器执行：

```bash
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
PYTHONPATH=src "$CPU" -m pytest -q \
  tests/test_poc_continuation_state.py \
  tests/test_serial_unloading.py \
  tests/test_proof_of_concept.py
```

结果：**41 passed in 3.38s**，无跳过。
新增 12 个参数化用例，覆盖首箱、POC 理想续箱、旧物理续箱、未经验证的处理集合、
实际/理想事件回退、双向来源改写、处理字段缺失/错误类型、送出回退/复活、幂等及合法新增。
预算用例构建真实官方场景并调用生产连接器，不替换构造器、状态校验或官方机器人。
事件用例调用真实 `apply_actual_motion_state`；沿用小型几何 fixture，仅替代机器人几何生成。
其中 4 个事件遗漏/来源改写用例先在未修改基线上运行，得到预期的 4 个失败（旧代码未拒绝），
修复后全部通过。这不是轨迹搜索或物理执行成功声明。

## 上一轮存档的离线检查

读取且未修改：
`/root/autodl-tmp/m710-poc-step1-20260917/outputs/poc_single_world/isaac/actual_remaining_state.json`。

| 检查 | 修改前 | 修改后 |
|---|---:|---:|
| 实际接收 | 0 | 0 |
| 理想接收 / 已处理 | 1 / 1 | 1 / 1 |
| 生产连接器阶段连接迭代 | 600 | **1800** |

修复后连接器 `AVAILABLE`；总请求、候选和阶段墙钟仍为 `null`。
`carton_l07_c02` 未进入剩余待抓或可抓候选；保留 40 个原始身份，39 个活动箱体。
同一存档重复载入保持事件和待抓集合幂等。
这仅是离线存档载入与连接器构造，**没有恢复存活世界、没有路径搜索、没有启动 Isaac**。

精简证据：[test_summary.json](validation/evidence/m710_poc_step1_1/test_summary.json)、
[archive_before.json](validation/evidence/m710_poc_step1_1/archive_before.json)、
[archive_after.json](validation/evidence/m710_poc_step1_1/archive_after.json)。
详细测试日志保留在本轮服务器隔离目录。

上一轮报告、视频与物理成果原样保留，不能当作本版重新实跑。**本轮新增物理完成数为 0**。
POC 模式、理想接收/送出、完整结果优先保存、实际/理想分开计数不变；
碰撞 margin、墙体/工具几何、接触许可、IK/FK、关节限制、质量/惯量、PD、力矩上限和插值均未修改。
