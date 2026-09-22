# 取消响应与停止事实交付顺序修复

## 基线、反例与范围

分支 `feat/v0.5-perception-ros2`，实际基线和 fetch 后远端均为
`386260cbb1e558f8a213b7a243b3ccff6f944808`；开始时工作区干净。
保留上一轮上下文新鲜度、来源身份、权限撤销和心跳规则。

先新增并运行调用生产 ExecutionGate 和执行桥回调的 CPU 测试：

| 事件 | 固定时间 / s |
|---|---:|
| 本地发送当前 goal 的取消请求 | 99.990 |
| 控制器实际接受取消 | 100.000 |
| 控制器实测停止 | 100.010 |
| 桥交付取消响应 | 100.030 |
| 桥交付停止事实 | 100.040 |

基线实际结果：**4 failed、3 passed，2.92 s**。
单独反例及三个“取消响应早于停止事实交付”的排列失败；
停止事实先缓存的三个排列通过。
根因是桥将响应本地接收时间误用为停止源事件的下界，
或从尚未完整验证的缓存中取 min 时间作为权威取消时间。
最小复现和全部回归保存在 `tests/test_cancel_stop_event_order.py`，不依赖服务器临时脚本。
这些测试使用 ROS 导入替身、合成事件和受控时间，不是真实 DDS 或硬件验收。

## 时间、身份与状态提交

- 新增 `ExecutionGate.request_cancel()`：仅接受已提交命令及已绑定 goal，
  在调用真实 cancel_goal_async 前记录本地请求时刻和时间域。
  重复请求不移动该时刻、不再发送一次；非法时间不消耗 pending cancel。
- ROS 响应必须 `return_code == 0`，且 goals_canceling 明确包含当前绑定 goal。
  桥同时核对完整活动命令及确已发出的本地取消请求；非空的其他 goal 列表不足以证明取消。
  领域 `accept_cancel()` 也要求匹配的 goal、已有请求和合法时间域。
- CANCEL_ACCEPTED 的 event_time 明确表示桥接收有效响应的时刻，
  不表示控制器源事件发生时刻，更不表示已经停止。
- 停止事实保持原始 controller_id、controller_epoch、goal_id、sequence、
  cancel_accepted_time、sample_time、clock_domain、关节顺序、实测状态及证据。
  ROS 时间编码复用已有 `state_time_to_float()`，不修正非法编码。
- 在同一已约定的时间域内要求：
  `请求发送 <= 控制器接受取消 <= 停止采样 <= 当前接收/处理时间`，
  并要求 `控制器接受取消 <= 有效取消响应接收时间`。
  **停止采样可以早于响应接收时间**，两者不再有错误的先后约束。
- 停止样本年龄仍不超过 **1.0 s** 默认阈值，边界包含等号；实测关节速度仍须
  `abs(v) <= 1e-3 rad/s`。没有加 epsilon、扩大阈值、改写源时间或从期望轨迹推定停止。
  时间参数/当前时间的非法非有限值拒绝；不跨时间域做减法。
- 桥与现有同机 mock 控制器使用共同 ROS 时间基准；本轮没有提供任意远端时钟同步证明。
  `enable_hardware=false` 保持不变，不能将本实现当作未知时钟硬件适配器。

事实在进入缓存前就检查请求关联、控制器实例、goal、时间、关节顺序、
停止序号和实测速度。只有响应尚未到达时才缓存有效匹配事实，每 goal 最多 8 条，
重复事实不重复占位；原总缓存上限 128 保留。
响应到达后逐条重新验证，过期/非法记录单独拒绝，不抛出未捕获异常，也不挡住下一条合法记录。
不再读取未验证缓存的 min 时间。

确认缓存保存完整源事实和确认；只有内容完全相同的重复事实才能返回原确认，
重复不刷新有效期、不推进停止序号、不释放新命令的资源。
相同键但不同内容拒绝。控制器停止序号历史最多 history_capacity（默认 128）个身份，
容量满时拒绝新增控制器身份，不静默遗忘历史。

既有 `complete()` 与晚到终态关联机制保留：
只有 CANCELED 结果仍保留活动命令；合法停止确认后才清理桥端活动关联。
停止确认先发生时，领域终态上下文继续关联随后到达的 CANCELED 结果。
正常未取消 SUCCEEDED 路径、世界指纹、grant、轨迹起点、命令去重及单线程互斥均保留。

## 验证与中间失败记录

固定同一命令、源取消时间和源停止事实，六种合法排列均有 CPU 与真实 ROS 参数化测试：

| 回调交付顺序 | CPU | 真实 ROS mock action |
|---|---|---|
| 响应 → 结果 → 事实 | 通过 | 通过 |
| 响应 → 事实 → 结果 | 通过 | 通过 |
| 结果 → 响应 → 事实 | 通过 | 通过 |
| 结果 → 事实 → 响应 | 通过 | 通过 |
| 事实 → 响应 → 结果 | 通过 | 通过 |
| 事实 → 结果 → 响应 | 通过 | 通过 |

CPU 还覆盖错误 goal/controller/命令关联、拒绝响应、缺少本地请求、错误时间域、
过期/未来/请求之前的源事件、倒置源事件时间、非法 ROS 编码、非零速度、错误关节顺序、
乱序停止序号、缓存容量及过期记录后恢复、重复事件、仅取消结果、正常成功和精确阈值边界。

真实测试 `ros2_ws/src/unloading_ros_bridge/test/test_cancel_stop_delivery.py` 使用
真实 DDS 命令/取消/停止事实和真实 mock action future，测试接收端缓存三类回调，
按指定排列调用原生产回调。它控制的是应用层交付顺序，不宣称控制网络本身。
控制器使用与其生产入口一致的 MultiThreadedExecutor；执行桥仍在单线程执行器中。
控制器实现、时间戳和 sleep 均未修改。等待依据端点和事件条件，不增加时间阈值或重试次数。

开发期间保留以下失败，未以随机重跑代替定位：

- 首次新 ROS 夹具把同步 mock execute 和取消服务放进同一个单线程执行器，
  六个顺序用例等待事件超时；改为生产控制器入口已有的执行器方式。
- 随后的六个严格 ROS nanosec 相等断言失败：现有领域契约使用 float 秒，
  ROS → float → ROS 的表示不能保证保留纳秒整数。
  改为该领域表示下的精确相等断言，没有加入容差或改动任何源时间。
- 合并测试中的第二个上下文用例发现失败。实际诊断为 clock_subscribers=2、
  context_subscribers=1，但 rosout 仅剩 source，执行桥同名日志发布者丢失。
  单独上下文两项通过。增加显式 action 资源清理和新夹具节点名隔离后，该问题仍存在；
  最终将原上下文夹具每个图的节点名也设为唯一，保留所有原发现/拒绝断言及超时。
  随后同一组合 **10 passed**，没有选择性跳过失败项。

## 实际命令、结果和版本

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q tests/test_cancel_stop_event_order.py tests/test_execution_context_admission.py tests/test_perception_integration.py tests/test_ros2_workspace_static.py --basetemp=tmp/cancel-targeted-final
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/cancel-final-all
```

服务器复用既有 Ubuntu 22.04 / ROS 2 Humble / Python 3.10，独立目录：
`/root/autodl-tmp/v05-acceptance/cancel-order-20260922/repo`。
source `/opt/ros/humble/setup.bash` 和 `../install/setup.bash`，PYTHONPATH 包含本目录
src、packages/unloading_contracts/src、ros2_ws/src/unloading_ros_bridge 后执行：

```bash
colcon --log-base ../build-log build --base-paths ros2_ws/src --build-base ../build --install-base ../install --symlink-install
python3 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_cancel_stop_delivery.py ros2_ws/src/unloading_ros_bridge/test/test_bridge_integration_launch.py ros2_ws/src/unloading_ros_bridge/test/test_mock_action_launch.py ros2_ws/src/unloading_ros_bridge/test/test_execution_context_transport.py
colcon --log-base ../full-test-log test --base-paths ros2_ws/src --build-base ../build --install-base ../install --return-code-on-test-failure
colcon test-result --test-result-base ../build --verbose
```

| 验证 | 实际结果 |
|---|---|
| 修复前确定性 CPU | 4 failed、3 passed，2.92 s |
| 最终 CPU 定向（含上一轮上下文） | 133 passed，1.98 s |
| 最终轻量全量 CPU | 751 passed、1 deselected，117.04 s |
| 最终真实 ROS 定向 | 10 passed，5.25 s |
| Humble 构建 | 3 packages finished，10.2 s |
| 本轮一次完整 Humble 回归 | 80 tests，0 errors，0 failures，0 skipped，退出码 0 |

完整 Humble 没有残留失败，包含上一轮曾失败的取消/停止集成用例。
中间和最终日志均保留在上述 repo 的上一层：
`targeted-ros*.log`、`context-isolated.log`、`dds-discovery-diagnostic.log`、
`build.log`、`full-humble.log`、`full-humble-results.log`。

仅 `unloading_ros_bridge` 的 package.xml/setup.py 从 **1.2.0 → 1.2.1**。
ExecutionContext 契约及 unloading_interfaces 仍为 **1.2.0**，所有 ROS 消息结构不变；
根项目 **0.5.0.dev0** 和独立 contracts **1.1.0** 不变，无新 tag。
Python gate 调用方需要先显式 request_cancel，再传 goal_id 给 accept_cancel；
当前分支 demo 与现有测试调用点已全部更新。

未执行 Isaac、GPU 推理、SAM、几何重建、录像或真实硬件。
未增加运行期 watchdog、完整异常恢复、跨进程防重放、跨主机时钟同步或跟踪/感知/规划改造。
