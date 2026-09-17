# 感知时间准入修复（2026-09-17）

仅修改 `feat/v0.5-perception-ros2` 的第二步时间准入。开始时读取 AGENTS.md、执行
`git fetch origin`：本地/远端均为 `a052997f5f5914bc6ce1c05c6a8005556f6bd62f`，工作区干净。
提交前再次 fetch，远端未变化。第一步的采集来源、非零速度、历史缺字段拒绝及 JointState 检查保留。

## 原因与实现

旧 `build_scene_update()` 只拒绝过期，未拒绝未来时间；watchdog 虽发现负年龄，
最终快照却没有对应阻断原因。在线入口也未在更新跟踪/来源序列前校验感知时钟域。
原始反例使用完整可规划合成几何：`now=100, capture_time=110, max_age=2`。
修改生产代码前运行新回归，断言不可规划失败，实际仍可规划且无阻断原因，见
[修复前日志](evidence/perception-time-admission-20260917/red-cpu.log)。同一几何在 capture_time=100 时确实可规划。

- `src/unloading_perception/scene.py`：统一纯函数校验，保持轻量核心无 ROS/Isaac/GPU 依赖。
  准入为 `-future_tolerance <= now - capture_time <= max_age`，边界包含等号；
  按浮点数直接比较，无隐藏 epsilon。测试用 `math.nextafter` 检查紧邻边界的失败值。
  now/capture_time 必须有限，max_age 有限且正，future_tolerance 有限且非负。
- `world_bridge_node.py`：新增参数 `observation_future_tolerance_seconds=0.0`；
  `snapshot_freshness_seconds=2.0`、机器人 0.5 s、机构 2.0 s 的原默认值保持。
  启动和动态参数入口均拒绝非法时间配置。入口、快照提交和 watchdog 共用同一判据。
- `mapping.py`：复用精确源时间及 ROS Time 一致性校验，在正式状态变化前检查时间；
  非有限值、processed_time 早于 capture_time 等畸形输入拒绝为 `OBSERVATION_TIME_INVALID`。
  processed_time 不用于年龄计算；不裁剪、重写或刷新任何源时间。
- `package.xml`：声明参数校验返回消息所用的 `rcl_interfaces` 依赖。

在线显式提供当前时钟域及初始化状态：`use_sim_time=true` 要求 ROS now>0，域为
`ros_sim_time`；普通 ROS 系统时间域为 `ros`。未初始化返回 `CLOCK_NOT_INITIALIZED`；
域缺失/不匹配返回 `OBSERVATION_CLOCK_DOMAIN_MISMATCH`，不会先把两个域的数值相减。
合法域内分别报告 `OBSERVATION_TIME_IN_FUTURE`、`OBSERVATION_STALE`。
没有时间上下文的离线用法保持兼容；离线数值零不等同于 ROS 时钟已经初始化。
所有在线 WorldBridge 调用都显式传入完整时间上下文，不能借离线默认值绕过检查。

新消息的时间检查先于 TF、ObservationTracker 和算法 SourceEpochGuard。异常消息不改变
last valid observation、跟踪结果或来源 epoch/sequence；拒绝原因和被拒来源保留在
`last_time_rejection` 中。已有完整快照时立即发布 `planning_admissible=false` 及原因，
保留上一合法几何、来源和 source_capture_time；尚无完整输入时不生成伪造快照。

时间异常保持阻断，只有后续时间合法且通过原有来源序列规则的观测才能解除。
重复/倒序消息、仅时钟恢复、机器人或机构心跳均不能解除。解除后仍合并未知区域、几何及
机器人/机构新鲜度门控；测试明确验证未知区域仍使场景不可规划。

watchdog 使用 Humble 支持的稳态时钟定时器每 0.1 s 唤醒；年龄始终只用 ROS 时间。
即使回拨或归零后 /clock 停在该值、没有新观测，仍会发布阻断更新。正值 ROS 时间单纯暂停
不会按墙钟人为增加年龄；本轮不引入时钟失联超时或同步框架。时间回拨后的恢复仍需现有来源
序列规则成立，不清空守卫、不自动重置仿真 epoch；不能满足时须使用既有显式重启流程。

## 验证与边界

全部测试在授权服务器 CPU、Ubuntu 22.04.5、ROS 2 Humble/Python 3.10 运行。
新增 CPU 测试在 `tests/test_perception_time_admission.py`；真实 ROS 测试在
`ros2_ws/src/unloading_ros_bridge/test/test_perception_time_transport.py`。
后者通过 DDS 和 /clock 驱动真实 WorldBridgeNode，无 monkeypatch 或手动调用 watchdog，
检查实际发布的 planning_admissible、blocking_reasons、source_capture_time、来源守卫及几何保留。
普通跟踪和算法 provider 来源守卫各跑一遍，另测 ROS 参数与 watchdog 对未来容差的一致性。

| 测试 | 实际结果 |
|---|---|
| 旧生产代码上的原始反例 | 1 failed（预期暴露缺陷） |
| 新时间 CPU 回归 | 25 passed |
| 新真实 ROS 时间回归 | 3 passed，1.53 s |
| 第一轮 CPU 契约及相关感知/快照/freshness | 49 passed |
| 第一轮真实适配器接线及 Humble 映射 | 6 passed，5.50 s |
| 默认轻量 pytest | 444 passed，1 deselected，5.82 s |

日志和按 LF 规范化的源码 SHA-256 在 [证据目录](evidence/perception-time-admission-20260917/tested_sources.json)。
默认排除项来自既有 pytest 配置；未添加 skip、continue-on-error 或放宽断言。
新增测试由现有默认 pytest 和 Humble CI 的 colcon test 自动收集，无需另设入口。
当前提交的 GitHub CI 状态须实际查询后在交付回复报告，本表不把服务器结果称为 CI 成功。

```bash
# 在服务器仓库根目录，使用第一轮已构建的当前消息接口。
CPU=/root/autodl-tmp/v05-acceptance/cpu/venv/bin/python
env -u PYTHONPATH "$CPU" -m pytest -q tests/test_perception_time_admission.py
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/v05-acceptance/joint-contract-20260917/install/setup.bash
export PATH="/usr/bin:/bin:$PATH"
export PYTHONPATH="$PWD/ros2_ws/src/unloading_ros_bridge:$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
export ROS_DOMAIN_ID=175
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_perception_time_transport.py
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_isaac_joint_state_transport.py ros2_ws/src/unloading_ros_bridge/test/test_humble_imports.py
env -u PYTHONPATH "$CPU" -m pytest -q tests/test_isaac_joint_capture.py tests/test_perception_integration.py
env -u PYTHONPATH "$CPU" -m pytest -q
```

新增时间测试的感知、关节和机构状态全部为明确合成 fixture；算法 provider 名称仅用于测试其
来源守卫，不代表算法感知验收。第一轮真实适配器回归仍由适配器读取合成采集文件并发布 JointState。
本轮未启动 Isaac、GPU 推理、SAM/MoGe、渲染或真机；新增测试不启动执行节点，不产生执行授权
或轨迹，`enable_hardware=false` 保持。已有 CI 的 mock 执行 launch 回归未改动。
真实采集时钟同步、跨仿真 epoch 自动恢复和硬件行为未验证；没有开始第三步载荷哈希修复。
