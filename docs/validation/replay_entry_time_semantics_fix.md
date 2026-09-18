# 单文件历史回放入口与时间语义

基线 `7efdc76081e717ae372312aaff95044a03de62ed`，分支 `feat/v0.5-perception-ros2`。
开始时读取 AGENTS.md、fetch 并确认本地/远端一致、工作区干净。本轮只修复只读历史回放接线。

## 修复前证据

真实 `replay_mock.launch.py` 以 use_sim_time=false 启动 PerceptionNode 和 WorldBridgeNode。
PerceptionNode 调用 CargoJsonReplayBackend 从磁盘读取 `cargo7_minimal.json`，无 SensorFrame 时后端
产生 replay 时钟域。默认世界节点要求 ros，首条消息在 last_observation 更新之前被拒绝，无法组装世界。

新回归直接执行仓库真实 launch 文件和安装后的生产节点，不复制 launch、不手工发布替代感知消息。
旧代码实际结果：`1 failed in 12.63s`；感知发布已收到多条，世界快照为 0；日志持续出现
`rejecting observation time: OBSERVATION_CLOCK_DOMAIN_MISMATCH`。
证据保留于服务器隔离验证目录 `replay-entry-20260918/red-launch.log`。

## 模式和时间约定

世界节点新增只读参数 `observation_mode`，合法值只有 `online`（默认）和 `replay_display_only`。
启动配置非法直接报错，运行时不能切换模式。真实 replay launch 显式选择后者。

- online 保持既有域一致、初始化、有限时间、未来/过期检查及故障锁存，阈值未改。
  replay 域仍被拒绝；声明 replay 元数据但把外层时间改写成 ros 的消息也被拒绝。
  provider、synthetic 或其他布尔字段均不能开启回放模式。检查发生在 TF、跟踪器或来源守卫更新前。
- replay_display_only 要求完整、一致的 `coverage.replay` 元数据；使用独立 SourceEpochGuard 校验发布顺序和
  显式会话重启。绑定该会话的文件、原记录元数据、cargo 和 unknown_regions，不能在同一会话换记录。
  不清空普通在线跟踪器，不执行跨帧关联，不查询当前 TF。非 world 几何仍保留缺变换事实。
- 领域 `build_scene_update` 对显式只读模式或已声明的合法 replay 元数据加入
  `HISTORICAL_REPLAY_DISPLAY_ONLY`。元数据本身只能增加阻断，不能使在线入口接纳历史。
  即使完整 pose、米制尺寸和 candidate_eligible=true，也始终 planning_admissible=false。
  原有未知区域、尺度、缺 TF 和状态不完整条件继续保留。缺少外部机器人/机构状态仍不能组装完整快照。
- 阻断先进入领域 scene_snapshot，再生成 domain_payload_json、ROS 显式字段和 world_fingerprint。
  没有在序列化后单独修改 ROS 顶层状态；测试验证 payload/显式字段/指纹一致，并验证移除领域阻断会改变世界指纹。

## 来源与重复发布

PerceptionNode 每会话仅通过真实后端读一次文件，UTF-8 解码、从同一份原始字节计算 SHA-256。
该会话固定所读记录；编辑文件后需重启加载，不在每次 poll 中重新假装采集。
`coverage.replay` 为加性元数据，不改 ROS IDL：

| 字段 | 含义 |
|---|---|
| source_uri / source_sha256 | 实际读取文件的绝对 file URI、实际文件内容 SHA-256；不是 fixture 内上游祖先文件的哈希 |
| session_id | 本次回放进程会话 UUID，与 observation.source_epoch 一致 |
| record_id | 原记录 record_id/observation_id；未提供时使用 `sha256:<实际内容哈希>` 的稳定内容身份 |
| original_capture_time / original_clock_domain | 源文件确实提供的原始时间/域，缺失保持 null，不使用 mtime/ROS now/启动时间补造 |
| original_source_sequence | 源文件原采集序号；未提供为 null，区别于传输序号 |
| original_time_status | PROVIDED 或 NOT_PROVIDED；有时间但缺失域、非法数值等明确拒绝 |
| capture_time_semantics | `replay_session_static_zero`：数值契约要求的 capture_time 固定为 0，域为 replay，明确是占位值 |
| session_elapsed_seconds | 本回放进程 monotonic 已逝秒数，与 processed_time 一致；不是传感器采集时间 |
| publication_sequence | 每次发布递增，与 observation.source_sequence 一致，不代表新采集 |
| published_time / publication_clock_domain | 感知消息发布时 ROS 时间/域；未发布的后端记录两者为 null |

源文件可在 `source_record` 对象中提供 record_id、capture_time、clock_domain、source_sequence；
没有该对象时读取同名顶层字段（record_id 也可用 observation_id）。现有旧 JSON 没有原始时间，明确保持未知。
`observation_id=replay-record-<内容哈希>`、原记录身份、原始时间和几何在重复发布及会话重启时保持稳定。
只有会话身份、传输序号、会话已逝时间和发布时间变化。世界消息的 published_time 是世界发布时刻，
source_capture_time 仍是 replay 占位 0；真实原始时间在领域 payload 的来源 coverage 中追溯。
重复发布不改变 scene 的几何指纹；世界指纹仍按其全部正式字段计算。

Marker 只增加历史只读标签和灰色状态，明确 capture=0 不是传感器时间；没有改生命周期。
默认 mock 机器人/机构发布器继续由 launch 提供，保留 synthetic-robot、synthetic_fixture 等来源标记；
世界节点内部没有补零关节、工具或机构状态，也没有生成执行授权。

## 修改与运行

- `backends.py` / 新增 `replay.py`：单文件内容身份、来源时间、会话发布元数据及一致性验证。
- `perception_node.py`：缓存会话记录，仅更新发布元数据。
- `world_bridge_node.py` / `scene.py`：显式模式、来源守卫及领域层不可规划状态。
- 真实 `replay_mock.launch.py`、少量 Marker 标签与 Ubuntu 启动说明。
- CPU 回归、真实 launch/DDS 和模式边界回归；未更改已有时间断言。

在仓库根目录、已有 Ubuntu 22.04 / Humble / Python 3.10 环境执行：

```bash
source /opt/ros/humble/setup.bash
ARTIFACT_ROOT=/tmp/replay-validation RUN_ID=replay bash tools/run_humble_acceptance.sh
source /tmp/replay-validation/humble/replay/install/setup.bash
export PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
export ROS_DOMAIN_ID=177
ros2 launch unloading_bringup replay_mock.launch.py \
  replay_path:="$PWD/tests/fixtures/vision_upstream/cargo7_minimal.json"
# 另一个 source 同环境、使用相同 ROS_DOMAIN_ID 的终端：
ros2 topic echo /unloading/world_snapshot
```

真实 launch 测试使用单独 ROS_DOMAIN_ID、实际 DDS 发现和条件等待。不是固定 sleep 后猜测结果；
也不要求 world/marker 两个 topic 的回调先后次序。缺失/损坏文件测试检查生产感知进程实际退出日志、无感知和世界伪成功。

```bash
python -m pytest -q tests/test_replay_time_semantics.py tests/test_perception_time_admission.py tests/test_perception_integration.py
python3 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_replay_entry.py \
  ros2_ws/src/unloading_ros_bridge/test/test_replay_mode.py \
  ros2_ws/src/unloading_ros_bridge/test/test_perception_time_transport.py \
  ros2_ws/src/unloading_ros_bridge/test/test_humble_imports.py
```

## 实际验证与边界

验证使用授权服务器的现有 ROS/CPU 环境；隔离目录为
`/root/autodl-tmp/v05-acceptance/replay-entry-20260918/`，原检出未修改。
首批定向 CPU 61 passed；真实入口/模式边界及既有时间、映射、Marker 共 33 passed（5.74 s）。
真实 launch 覆盖原始时间有/无、重复发布、领域/ROS 一致、历史标签，以及缺文件/坏 JSON。
CPU 用完整米制候选证明历史阻断独立于几何质量，ROS 模式测试证明异常消息不污染来源状态，缺状态不造快照。

| 检查 | 实际结果 |
|---|---|
| 最终服务器定向 CPU | 61 passed，0.41 s |
| 真实入口/模式与时间、映射、Marker 回归 | 33 passed，5.74 s |
| 最终服务器默认轻量 pytest | 568 passed，1 deselected，12.20 s |
| 最终 Humble build/test | 3 packages；41 tests，0 errors/failures/skipped；退出 0 |
| Windows 默认编码全量 | 567 passed，1 failed，1 deselected，84.76 s |
| Windows PYTHONUTF8=1 全量 | 568 passed，1 deselected，67.48 s |

日志为 targeted-cpu.log、related-ros.log、default-final.log、humble-final.log；最终 colcon 记录在
`humble/replay-final/`。Windows 默认已有 GBK 失败仍在 `test_effective_scene.py:29`，与基线同一原因，
本轮未修复或删除；UTF-8 模式全量通过。1 deselected 为项目原有默认 marker 排除项，没有新增 skip 或放宽断言。
当前提交 CI 状态在交付回复中按实际查询结果报告，不拿历史 CI 代替。

未运行 Isaac、模型、渲染、任务扫描、机械规划或真机；未升级依赖。没有通用多文件播放器、倍速、历史 TF 载入器、
连续推理或跨帧跟踪。已知文件本身缺少真实采集时间时仍无法还原该时间。
没有新增 RViz 截图；没有开展机器人/机构心跳新鲜度传播和仿真回退恢复等后续步骤。
