# 世界快照心跳与源采样新鲜度

基线 `18fb30f742cd712bd930a2d6849555ec15483bab`，分支 `feat/v0.5-perception-ros2`。
开始时读取 AGENTS.md、fetch，确认本地/远端一致且工作区干净。保留第一步回放隔离与在线时间准入。

## 原始反例

合法 JointState/MechanismState 持续更新内部采样时间；原代码只有内容变化或此前过期时才因状态回调发布世界。
watchdog 又仅在过期状态发生变化时发布。静止状态因此一直保持内部新鲜，反而不能把新采样时间传给执行桥。

生产修改前，在真实 WorldBridgeNode、ExecutionBridgeNode、mock 控制器和 DDS 上运行新回归。
先完成端点发现，再发布一条明确合成的在线感知，按 50 Hz 持续发布同内容、新时间的机器人/机构状态和合法上下文。
原阈值未改；纯心跳窗口为 1.2 s，超过机器人 0.5 s 和世界接收 1.0 s，仍小于感知 2.0 s。

旧代码实际失败：`1 failed in 1.44s`。
世界节点最新机器人时间 `1789700930.0267653`，执行桥仍是 `1789700928.866746`，
约落后 1.17 s；实际 `_authoritative_state_error()` 为 `WORLD_PUBLISHER_STALE_OR_TIME_JUMP`。
不是持续感知造成的刷新，也没有直接修改节点时间或替换门控。日志见隔离验证目录 `red-heartbeat.log`。

## 实现与时间、指纹语义

唯一生产修改在 `world_bridge_node.py::check_freshness()`：
复用现有 0.1 s 稳态时钟 watchdog，每次都调用 `_commit_snapshot()`，保留内容变化事件触发。
同内容高频采样合并到约 10 Hz 的周期发布，不新增参数、线程、总线或组装路径。
0.1 s 小于现有机器人 0.5 s、机构 2.0 s、世界接收 1.0 s 的限值；不是调高有效期。

心跳重新计算时间准入并从已确认状态组装领域快照，不复制旧消息后改发布时间：

- published_time：本次世界消息的 ROS 时间。
- robot_sample_time / mechanism_sample_time：真正接收且通过现有校验的源采样时间；定时器不更新它们。
- source_capture_time：最后接纳感知的原始时间；显式历史回放仍保持第一步的占位/原始来源元数据语义。

调度用稳态时钟，年龄用 ROS 时间。/clock 停住时可继续发送心跳，但不会仅因墙钟流逝令源过期；
published_time 在暂停时也保持该 ROS 时间，publisher_sequence 仍可递增。

不修改共享契约或执行桥：RobotStateRevision 现有指纹仅覆盖关节内容，sample_time 单独序列化；
机构内容及身份与 sample_time 传输字段原本分离。世界指纹仍由实际领域状态计算。
机器人样本时间、感知源时间与 payload 对应字段按原 ROS Time 精度一致；机构源 epoch/模拟身份保持一致，
其采样时间沿用现有 ROS 显式字段，未向机构内容字典塞入会改变指纹的时间值。

纯心跳保持 scene/robot/mechanism 内容版本及其指纹、world_fingerprint、publisher_epoch 不变，
仅推进发布序号/时间及真正更新的源采样时间。publisher_restart 只在首条世界消息为 true。
真实关节或工具内容变化仍推进对应版本和世界指纹；准入变为过期阻断也改变世界指纹并撤销旧授权。

## 失联、恢复及授权验证

新增 12 项真实 ROS 测试使用隔离 ROS_DOMAIN_ID、条件等待、生产节点/组装/执行门控；全部输入明确为合成测试数据。

- 静止连续采样超过 1.2 s：仅一条感知，源样本时间持续传到执行桥；内容版本/指纹保持稳定；
  同一个 grant 在多个心跳后保持原值；真实命令收到 STARTED/SUCCEEDED，实际通过 mock action 路径。
- 单独停止机器人/机构，以及不断重发旧时间戳和乱序样本：世界继续发心跳，停止源时间冻结，
  原阈值后不可规划，真实命令分别被 ROBOT_STATE/MECHANISM_STATE_STALE_OR_TIME_JUMP 拒绝。
  机构测试用真正的新合成感知隔离 2 s 超时；正常/原反例窗口不重复发感知。
- 恢复同内容新采样：源新鲜度恢复，机器人/机构内容版本不增加，撤销的 grant 不复活。
  感知已过期时，恢复机器人采样不能清除 OBSERVATION_STALE；只有新合法感知恢复它。
- 真实关节位置或工具身份改变：对应版本/指纹及世界指纹变化，旧授权撤销、旧命令拒绝。
- grant 到期：持续心跳不更改 expires_at，真实命令按现有规则拒绝。
- 停止并销毁世界发布节点，测试状态源和上下文仍持续更新：新命令被
  WORLD_PUBLISHER_STALE_OR_TIME_JUMP 拒绝。执行桥无法在世界失联期间得到新源样本，所缓存样本也会变旧；
  此测试验证原有世界接收超时的优先拒绝，不通过伪造新时间来单独掩盖其他老化。
- 缺机器人/机构状态时无伪造快照；/clock 暂停时只有发布序号推进，所有源时间及正常准入保持不变。

没有新增执行中自动停止、grant 续期/补发或上下文自动更新。测试上下文由明确的测试发布者持续提供。
既有时间、回放、映射和执行接线断言全部保留。Marker 晚订阅测试仅暂时移出世界节点的 executor，保留真实 DDS
publisher，确认订阅获得缓存而非下一次心跳；原“期间发布序号不变”断言仍在，未替换生产发布函数。
共用 /clock fixture 增加独立 domain_id，保持时间边界断言不变。

## 命令、结果与边界

复用 Ubuntu 22.04 / Humble / Python 3.10；没有升级依赖。原服务器检出未修改，使用基线归档及本轮上传文件：
`/root/autodl-tmp/v05-acceptance/world-heartbeat-20260918/repo`。
日志在其上一层：red-heartbeat.log、heartbeat-final.log、related-ros.log、default.log、humble.log；
完整 build/test 记录在 `humble/heartbeat/`。

```bash
# 仓库根目录，已有核心包可导入的 Humble 环境
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=175
ARTIFACT_ROOT=/tmp/world-heartbeat RUN_ID=heartbeat bash tools/run_humble_acceptance.sh
source /tmp/world-heartbeat/humble/heartbeat/install/setup.bash
export PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
python3 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_world_heartbeat.py
# 默认轻量 pytest 使用项目独立 CPU venv，避免引入 ROS pytest 插件依赖。
```

| 检查 | 实际结果 |
|---|---|
| 旧代码上的原始回归 | 1 failed，1.44 s，确认执行桥保存过期样本 |
| 新增真实 ROS 心跳回归 | 12 passed，13.42 s（heartbeat-final.log） |
| 时间、回放、映射、Marker、既有执行接线 | 17 passed，8.81 s |
| 服务器默认轻量 pytest | 568 passed，1 deselected，11.78 s |
| 完整 Humble build/test | 3 packages，53 tests，0 errors/failures/skipped；退出 0 |
| Windows 默认环境 | 567 passed，1 failed，1 deselected，80.92 s |
| Windows PYTHONUTF8=1 | 568 passed，1 deselected，74.31 s |

默认 Windows 唯一失败仍是基线 `tests/test_effective_scene.py:29` 的 GBK 解码错误，未删测试或扩展编码整改。
1 deselected 为原有默认 marker 排除规则；没有新 skip 或放宽断言。当前提交 CI 状态按实际查询在交付中报告。

没有运行 Isaac、SAM/MoGe、渲染、任务扫描、真实机器人或完整卸货仿真；短轨迹只用于现有 mock 控制器门控验证。
未来状态时间戳、仿真回退自动恢复、离线载荷、连续推理及执行中自动停机仍属未处理边界。本轮止于第二步。
