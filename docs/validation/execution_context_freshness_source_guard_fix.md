# 执行上下文源时间与来源接管修复

## 基线与范围

分支 `feat/v0.5-perception-ros2`；实际基线及 fetch 后的远端均为
`cc64bbd49e4e113f44ee0750b6981a24e452ee1b`。开始时工作区干净。
本轮只修改执行上下文接纳、发命令前校验、对应消息、发布夹具和回归测试。
未修改跟踪器、感知算法、世界桥心跳、规划算法或已发送命令的取消/停止状态机。

## 先复现，再修复

修改生产代码前运行：

```powershell
.venv310/Scripts/python.exe -m pytest -q tests/test_execution_context_admission.py
```

两个断言实际失败，`2 failed in 0.54s`：

- now=100、observed_time=1 的上下文被 `on_context()` 保存。
- publisher-a / execution-a → publisher-b / execution-b 后，
  publisher-c 能重新激活 execution-a。

这是调用生产节点回调、使用 ROS 导入替身的 CPU 测试，不是真实 DDS 验证。
最终测试补上了合法世界、匹配 grant、真实命令映射和 `send_goal_async` 断言，
并用新鲜上下文成功发送作对照。真实 ROS 验证另外执行，见下文。

## 时间和身份规则

- 源时间复用 `state_time_to_float()` 检查 ROS 编码：非负 int32 sec，
  `0 <= nanosec < 10^9`。随后要求 observed_time > 0、有限、clock_domain=`ros`，
  当前时钟已初始化，且 `0 <= now - observed_time <= context_source_max_age_seconds`。
- 新增源年龄阈值默认 **1.0 s**；原接收年龄阈值仍独立为 **1.0 s**。
  两个参数都必须有限且严格为正，在初始化、接纳和发送检查中验证。
  没有放宽其他阈值、未来容差或添加 epsilon。
- 接纳时检查源年龄，命令进入原有授权门控前再次检查源年龄和接收年龄。
  保存原始消息时间，本地接收时间仅由合法新消息更新；不把 now 写入 observed_time。
- 传输身份是 publisher_epoch / publisher_sequence / publisher_restart，
  业务身份是 session_id / epoch / planning_generation / allowed_plan_id / predecessor_plan_id。
  planning_generation 不承担消息序号职责。
- 首次接入允许新鲜、结构合法、身份非空的消息从任意非负序号开始，以支持晚加入。
  若声明 restart，则序号必须为零。
- 当前发布者的序号和源采样时间都必须严格递增，且不得再次声明 restart。
  正常业务 session/epoch 切换由当前发布者按此顺序发布；旧业务身份随之退役。
- 新发布者接管必须显式 restart=true、sequence=0，且源采样时间继续严格递增。
  发布者重启不能绕过同一 session/epoch 的 generation 回退检查，
  也不能重新激活已退役的 session/epoch。
- 所有结构、时间、顺序、业务、容量检查全部成功后才提交守卫状态。
  非法消息不推进序号、不占有来源、不退役旧来源，也不刷新上下文或 grant。
- 上下文局部守卫分别最多保留 128 个退役发布者和 128 个退役业务身份。
  满额时明确拒绝需要新增对应退役记录的转换；当前合法来源的心跳仍可继续。
  未改动其他调用方使用的 `SourceEpochGuard` 及其滚动历史语义。

保证仅覆盖当前接收进程生命周期；没有实现跨进程持久化防重放或来源安全认证。
时钟回退不会自动清空身份历史；本轮不增加跨时钟域恢复机制。

## 授权与兼容性

权限五元组中任一字段变化（包括 predecessor_plan_id），或合法发布者接管，
都会调用既有 `ExecutionGate.revoke_all()` 撤销旧 grant 和未提交的发送预留。
纯心跳不撤销权限，不延长 grant.expires_at。
已经发送的活动命令仍保留，继续走原有身份绑定取消、CANCEL_ACCEPTED、
取消结果、实际停止事实和停止确认链路。未增加执行期自动停止 watchdog。
`enable_hardware=false` 保持不变。

`ExecutionContext` 消息契约从 **1.1.0 → 1.2.0**，新增三个 publisher 字段。
`unloading_interfaces` 和 `unloading_ros_bridge` 包版本（含 setup.py）同步为 **1.2.0**。
根项目仍是 **0.5.0.dev0**，独立 Python `unloading-contracts` 仍为 **1.1.0**，
其他消息契约和 bringup 版本保持不变；没有 release tag 或新的 v0.6 版本线。

这不是旧消息的自动升级：旧 schema 或空 publisher_epoch 均拒绝。
必须重新 `colcon build` interfaces、bridge 及依赖包，并协调重建所有外部发布者。
当前分支搜索到的 ExecutionContext 发布者只有
`test_bridge_integration_launch.py` 和 `test_world_heartbeat.py`，均已更新；
没有另一个生产发布节点可更新。新增 DDS 测试也显式填写新字段。

## 实际验证

CPU 测试直接加载生产执行桥，使用 ROS 导入替身；纯守卫没有 ROS/GPU 依赖。
覆盖边界、接收后源时间过期、重复/倒序、合法心跳、generation 回退、
两层身份退役、接管失败的原子性、历史容量、五类权限变化、
预留撤销与已发送命令保留、非法时间/参数以及合法发送对照。

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/context-final-all
.venv310/Scripts/python.exe -m pytest -q tests/test_execution_context_admission.py --basetemp=tmp/context-final-targeted
```

复用现有服务器 Ubuntu 22.04 / ROS 2 Humble / Python 3.10，独立目录：
`/root/autodl-tmp/v05-acceptance/context-20260922/repo`。
由核对过的本地代码上传构建，不改服务器已有工作区。
ROS 新测试使用真实消息、DDS、/clock 和事件条件等待；
测试时钟显式推进，不靠增大 sleep 或阈值通过。
命令链路沿用现有真实 ROS mock action launch 测试，包含取消接受、
取消结果和 StopAcknowledgement；这不是硬件动作验收。

```bash
ARTIFACT_ROOT=/root/autodl-tmp/v05-acceptance/context-20260922 \
  RUN_ID=context bash tools/run_humble_acceptance.sh
```

初始验收上传漏了 tools，首次全包测试收集失败；补齐后执行 74 项，
其中 6 项因缺少原有历史 manifest 夹具而失败，68 项通过。
原有 manifest 原样补齐，没有改写任何历史数据。
新增 DDS 测试最初因 Humble 的日志常量为 bytes 而出现两项测试代码失败，
已改为按节点名及日志前缀等待拒绝事件；原有 17 项定向 ROS 测试当时已通过。

最终实际结果：

| 检查 | 结果 |
|---|---|
| 修改前两个 CPU 反例 | 2 failed，0.54 s |
| 最终上下文 CPU 定向 | 52 passed，0.75 s |
| 最新 Windows UTF-8 全量 `pytest -q` | 698 passed，1 deselected，119.35 s |
| Humble 消息和依赖重建 | 3 packages finished，7.65 s |
| 最终真实 DDS + mock action + 心跳 + imports 定向 | 19 passed，16.53 s |
| 补齐夹具后的完整 Humble 测试 | 73 passed，1 failed，0 skipped，69.28 s |
| 原始基线单独取消 launch 对照 | 初次及随后 5 次均通过 |
| 受控停止/取消响应顺序诊断 | 基线与修复后均复现同一停止时间拒绝 |

完整 Humble 测试仍有一次取消 launch 失败，不能记为全包通过：
已收到 STARTED、CANCEL_ACCEPTED 和 CANCELED，但 StopAcknowledgement 等待超时；
日志为 `stop fact is stale, pre-cancel, or from the future`。
该 launch 此前和最终定向运行均通过。

为区分本次回归与原有时序问题，在独立 `baseline` 目录从原始 SHA 重新构建
旧消息及三个 ROS 包。额外诊断脚本 `../probe_cancel_order.py` 分别加载基线和修复后
真实 ExecutionBridgeNode：建立合法 gate/grant/goal，构造已发生的停止事实，
先交付取消响应，再交付停止事实。两者均由同一未改动的检查拒绝：
没有提前缓存 stop fact 时，桥将取消响应的本地接收时刻记为取消接受时间，
早于该接收时刻发生的停止随后被判为 pre-cancel。
这项诊断直接调用真实节点回调，使用合成事件，不是额外的 DDS 或硬件验收。
`execution.py`、mock 控制器和执行桥的取消/停止方法均未修改。
依本轮明确边界保留该既有缺口，不通过放宽阈值或重写停止时间隐藏失败。

最终 ROS 定向命令（先 source Humble 和本轮 install，设置 src、contracts、bridge PYTHONPATH）：

```bash
python3 -m pytest -q \
  ros2_ws/src/unloading_ros_bridge/test/test_execution_context_transport.py \
  ros2_ws/src/unloading_ros_bridge/test/test_bridge_integration_launch.py \
  ros2_ws/src/unloading_ros_bridge/test/test_world_heartbeat.py \
  ros2_ws/src/unloading_ros_bridge/test/test_humble_imports.py
```

服务器日志位于独立 repo 目录的上一层，保留 `humble.log`、`targeted-ros.log`、
`humble-final.log`、`humble-complete.log`、`targeted-ros-final.log`、
`baseline-cancel*.log`、`baseline-stop-order.log`、`fixed-stop-order.log`。

未执行 Isaac、GPU 推理、SAM/MoGe、几何重建、录像、完整卸货或真实硬件任务。
未验证外部分支发布者、跨进程防重放或运行期自动停止；这些不在本轮范围内。
