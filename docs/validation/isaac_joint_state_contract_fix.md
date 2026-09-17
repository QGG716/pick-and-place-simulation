# Isaac JointState 采集契约修复（2026-09-17）

范围仅为 `feat/v0.5-perception-ros2` 的采集机器人状态发布。开始时已读取 AGENTS.md、
执行 `git fetch origin`：本地/远端均为 `9900cec920e8c76910b134a0c55774e51d42d004`，工作区干净。

## 根因与来源

适配器原来从 manifest 的 `joint_names/q_rad` 构造 JointState，仅发布 name/position。
世界桥要求同序、等长且有限的 velocity，因而拒绝该消息；robot_state 一直为空。
`build_scene_manifest()` 的 q 来自快照并覆盖 J1，是**配置的运动学目标**，不是仿真采样值。
现有归档 binding 没有关节速度；部分 capture_configuration 虽有实际位置读回，也没有
与每个采集帧绑定的速度记录。这些历史数据不能升级为已验证的静态状态。

原 ROS launch 回归的测试节点直接发布完整 JointState；
`probe_algorithm_world_handoff.py` 也自行用目标 q 和 `[0.]*6` 发布，未经过真实适配器。
新增回归先在旧生产代码上运行，实际收到 `velocity=[]`，与 fixture 的 `[0.25,-0.5]` 不符而失败：
[修复前日志](evidence/joint-state-contract-20260917/red-ros.log)。

## 实现选择

- `scripts/isaacsim_perception_capture.py`：在设置位置、零速度、位置目标并执行仅渲染的
  `world.render()` 后，从同一 articulation 读回位置和速度。按 DOF 名称一起重排到 manifest 顺序，
  写入各模组及兼容主采集目录的 `capture_binding.json.robot_state`。
- `src/unloading_perception/isaac_validation.py`：新增两个轻量函数，生成/验证这项可选字段，
  不引入 Isaac/ROS/GPU 依赖。沿用 capture binding v1 的加性扩展方式，子记录标识为
  `isaac_robot_state_v1`。校验机器人模型/资产、manifest 指纹、epoch、frame、采样时间、时钟域，
  以及关节名称唯一性/顺序、位置/速度长度和有限值。数量由名称推导，单位为 rad、rad/s。
- `IsaacSensorAdapterNode`：只发布验证后的**采集读回**位置和速度，不再把 manifest 目标 q
  当作采样位置；effort 保持空。每次加载均先清空缓存。历史缺字段或无效记录会明确打印
  `robot JointState suppressed` 原因，保留图像回放，不发布该帧 JointState，也不沿用上一帧缓存。
- `probe_algorithm_world_handoff.py`：移除自行补零，要求 `--capture-binding`，使用同一验证器。
  该 probe 不作为新增真实适配器接线测试的替代品。旧归档命令缺少新参数会明确失败；不改写历史证据。

**WorldBridgeNode 生产代码未修改。** 原名称、数量、有限值和采样顺序检查全部保留。
JointState 使用原采集时间；不改为发布时间。重复/倒序样本仍由世界桥拒绝。
零时刻也不伪装成正时间：它不能形成世界桥接受的机器人状态。

## 静态速度依据与边界

当前采集端已有明确的运动学保持行为：设置零速度，然后仅 render，不推进物理步。
新增记录将来源写为 `KINEMATIC_HOLD_READBACK`，保存该次保持命令、无物理步的执行路径证据、
显式速度读回，以及本次采集绑定。静态记录要求读回/命令速度均为零、位置读回与命令相符。
缺少保持证据会拒绝；即使有证据，缺少显式速度也不会推算或补零。
一般显式关节采样用 `ARTICULATION_READBACK`，合法非零速度原样保留。

时间仍是工程已有的逻辑采集 simulation_time，**不声称是物理引擎积分时间或控制器实测时间**。
`synthetic_fixture` 区分测试 fixture；来源为 Isaac 读回的记录仍是仿真状态，并非硬件反馈。
旧屋顶/满垛采集未补写新字段。真实新采集的写入和传感器同步尚未运行验收；未来需要一次带新记录的
真实 Isaac 小型采集验证读回、帧绑定与渲染同步。本轮不为补齐这一边界重跑仿真。

## 验证

所有运行在服务器 CPU 上完成：Ubuntu 22.04.5、ROS 2 Humble、Python 3.10。
未启动 Isaac、未使用 GPU 计算、未运行 SAM/MoGe、未重新渲染、未连接真机。
新增接线测试不启动执行节点、不生成执行授权或轨迹；`enable_hardware=false` 的默认门控未修改。

| 检查 | 实际结果 |
|---|---|
| 修复前真实适配器→ROS 回归 | 1 failed，实际 velocity 为空 |
| 针对性 CPU 契约测试 | 24 passed |
| 真实适配器→DDS topic→真实世界桥 | 2 passed，5.54 s |
| 现有 Humble 导入/映射回归 | 4 passed |
| 默认 `pytest -q` | 419 passed，1 deselected，5.81 s |
| 捕获脚本/probe 的 Python 编译检查 | 通过；未执行 Isaac import |

默认排除项来自原有 pytest 配置，未新增 skip 或放宽断言。日志及测试源码哈希在
[证据目录](evidence/joint-state-contract-20260917/tested_sources.json)。
服务器运行目录：`/root/autodl-tmp/v05-acceptance/joint-contract-20260917`。

`test_isaac_joint_state_transport.py` 使用真实节点和 ROS 消息传输，正例 JointState 只由
适配器读取文件并发布，无 monkeypatch。两关节 fixture 刻意使采样 q 不同于 manifest 目标 q，
并覆盖非零速度、完整静态证据、缺证据/缺速度、数量不符、重复/错序名称、NaN/Inf、重复/倒序时间。
接收端畸形消息来自明确的负例注入器。无效输入不替换最后有效状态；缺机器人记录时仍收到图像。
世界快照的 current_q、actual_velocities、sample_time 均核对；独立的合成感知/机构输入保留未知区域，
故完整快照形成后仍不可规划。这是**合成数据的接口验收，不是算法感知通过**。

测试位于现有 `tests/` 和 ROS 包 `test/`，由默认 pytest 与现有 Humble CI 的 colcon test 自动收集，
无需另建测试入口。已有执行/动作 launch 测试未改动，本轮本地仅运行上述相关 ROS 测试。
GitHub 当前提交的实际 CI 状态在最终回复报告，不能用本地通过代替 CI 成功。

```bash
# 在服务器仓库根目录；先构建当前消息接口并 source 当前 install/setup.bash。
# 构建使用 PATH=/usr/bin:/bin:$PATH 和 Python 3.10，避免选中服务器 Conda Python。
CPU=/root/autodl-tmp/v05-acceptance/cpu/venv/bin/python
env -u PYTHONPATH "$CPU" -m pytest -q tests/test_isaac_joint_capture.py
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/v05-acceptance/joint-contract-20260917/install/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/unloading_ros_bridge:$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
export ROS_DOMAIN_ID=174
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_isaac_joint_state_transport.py
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_humble_imports.py
env -u PYTHONPATH "$CPU" -m pytest -q
/usr/bin/python3.10 -m py_compile scripts/isaacsim_perception_capture.py tools/probe_algorithm_world_handoff.py
```

本轮未扩展未来时间戳准入、图像/深度/点云载荷哈希、几何判断、双模组融合、自动检测、RViz 或连续流水线。
