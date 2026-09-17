# RViz observed surfaces and Marker lifecycle — step 7

基线：`1f352725cbc97c08b0403001d077436419cc0fe8`，分支
`feat/v0.5-perception-ros2`。只修改 ROS 显示适配、配置、合成演示和测试。
前六步的数据、融合、时间准入、执行门控和脚本退出语义未改。

## 原始反例

先在基线生产 `WorldBridgeNode.publish_markers()` 上运行三个真实 ROS 消息回归：

| 输入 | 修复前实际结果 | 修复后 |
|---|---|---|
| world 面片，无完整 pose/dimensions | `len(geometry)==0`，测试失败 | 按原始四点绘制 LINE_STRIP |
| 完整体 A/B → B | 没有 DELETE，`('cargo',0)` 残留 | A 及附属文字显式 DELETE，B 完整 ADD |
| A/B → B/A | 同一 `cargo,0/1` 对应的位置互换 | ns/id 与顺序无关 |

旧代码三项均失败（3 failed，0.19 s），日志为隔离验证目录中的 `red-markers.log`。
测试只捕获传输输出，未替换生产绘图逻辑；另有真实节点/DDS 测试验证实际发布路径。

## 显示语义

- 完整体：仅已有 pose/full_dimensions，原样保留位置、四元数、尺寸；不以面片范围补箱体。
  标签同时显示 candidate_eligible、association_status、geometry_validity；完整几何不代表执行授权。
- 正式 `observed_surface_v1`：直接画 world `corners_3d_m` 四点闭合边界，单位米，线宽 0.012 m，
  Marker pose 为零平移/单位四元数。没有二次 T_W_C、轴对齐替换、虚构厚度或认证箱缝声明。
  标签为 `OBSERVED PATCH / full volume UNKNOWN`，保留完整来源和 surface.capture_time。
- 优先匹配 `raw_result.fusion_diagnostics.face_reduction.representatives` 的完整
  module/capture/source_instance/face 键。未提供映射时明确标为 raw display；映射存在但缺失或歧义时
  报 DISPLAY DIAGNOSTICS，不按 face_id 猜测。标签分别统计代表引用数、原始来源面片数。
  不再次融合或去重，不修改 raw evidence；重复对象身份也明确拒绝显示，避免顺序决定赢家。
- 冲突：association/geometry 冲突或已有 raw conflict diagnostics 均突出红色及 CONFLICT 文字，
  保留原始冲突端点和原因；即使只剩一个代表也不隐藏。未额外画交集多边形。
- UnknownRegion 当前只有 region_id/frame_id/reason/可选二维 bbox：只在固定说明位置显示数量和原因，
  明确 `UI LEGEND / fixed anchor, NOT a spatial region`，不使用像素坐标构造三维未知箱体或自由空间。
- 绿色为可规划且候选可用的完整几何，青色为可用面片，琥珀色为阻断/候选不可用，红色为冲突，灰色为历史。
  全局横幅明确 planning_admissible 和全部 blocking_reasons；颜色不是新质量评分或执行许可。
- 时间拒绝或 watchdog 发布的旧合法几何仍可见，但变灰并标明
  `HISTORY / TIME INVALID - retained observation`。保留 snapshot.source_capture_time、surface.capture_time
  和源身份；显示刷新不改变世界状态或清除时间锁存。时间恢复后仍保留其他未知/阻断原因。
- JSON 损坏/错误结构、坐标非有限或溢出、错误 frame、非单位完整体四元数等只影响显示记录，
  跳过误导几何并记录诊断，不修改输入或执行状态。深层错误 JSON 同样隔离。

## 身份、删除、QoS 和正常重启

`MarkerScene` 在 ROS 适配包内。ns 使用发布节点全名的编码前缀，加 JSON 编码的完整身份；id 恒为 0。
对象身份依次为 object_id、track_id、帧内 fusion/source 身份，均包含 source_epoch；帧内身份另含原采集时间。
面片及其文字再包含完整来源四元组和表示类型。编码不截断、不使用 Python hash/int32 摘要，避免摘要碰撞。
新帧内身份删除旧标记并创建新标记，不宣称跨帧跟踪。

每次消息含全部当前 ADD 和已经消失的所属 ns/id DELETE，覆盖对象减少、完整体→面片、无几何、epoch 切换。
为使同一发布进程存活期间的重新订阅也清理错过的旧标记，删除键保留到该进程结束，并进入最新完整消息。
这个小型方案的历史键数量会随新来源身份增长；本轮没有建设长期连续流水线或有界崩溃恢复存储。
不发送 DELETEALL，不依赖 lifetime、仿真时间前进或短时过期。

发布端和 RViz MarkerArray 均为 Reliable / Transient Local / Keep Last / Depth 1，topic 为
`/unloading/markers`，Fixed Frame 为 `world`。晚启动直接获得缓存的完整当前场景，不要求再次采集。
显示状态文字也属于当前完整集合；没有用只有增量 DELETE 的消息替代运行中的当前场景。

正常 `destroy_node()` 在 DDS context 仍有效时删除所有已拥有标记，使用 Humble 实际存在的
`Publisher.wait_for_all_acked(Duration(seconds=.5))` 等待有限时间，失败记录 warning 后继续资源释放。
普通 Ctrl-C 的入口保持 context 活到清理完成。真实 DDS 测试验证正常关闭、接收 DELETE、重启后再次 ADD；
安装后的演示 CLI 也实际通过 SIGINT 正常退出并清除所属标记。

同一节点名/命名空间应只有一个本显示发布者；其他工具使用自己的 ns。没有保证强杀、SIGTERM、进程崩溃、
失效 DDS context、或订阅者离线跨越发布者重启的清屏。持久缓存属于存活发布者，不是跨进程可靠存储。

## 测试和实际环境

全部新输入为合成数据，不是 SAM/MoGe 推理或算法质量验收。默认核心不新增 ROS/RViz 依赖。

| 检查 | 实际结果 |
|---|---|
| 基线显示回归 | 3 failed，0.19 s，已确认旧缺陷 |
| 首轮显示 + 真实 DDS 测试 | 17 passed，0.44 s |
| 最终显示 + 真实 DDS | 19 passed，0.44 s |
| DDS 跨 topic 到达顺序修正后复核 | 5 次独立运行，每次 2 passed，0.42 s |
| 相关消息映射和时间传输 | 7 passed，1.54 s |
| Windows 默认 GBK 全量 | 556 passed，1 failed，1 deselected，75.84 s |
| Windows PYTHONUTF8=1 全量 | 557 passed，1 deselected，76.14 s |
| 服务器轻量 CPU 全量 | 557 passed，1 deselected，11.75 s |
| tests_metric | 31 passed，1.06 s |
| 首轮完整 Humble build/test | 3 packages；32 tests，0 errors/failures/skipped |
| 最终完整 Humble build/test | 3 packages；34 tests，0 errors/failures/skipped；退出 0 |
| 安装后的 marker_demo CLI | 实际 DDS 收到五种场景；SIGINT 返回 0；收到所有所属标记 DELETE |

Windows 唯一默认失败仍是既有 `tests/test_effective_scene.py:29` 未指定编码读取 UTF-8 JSON 导致 GBK 解码错误；
未删测试或扩大编码整改。1 deselected 来自项目原有默认 marker 排除规则，没有新增 skip。

补测曾出现一次测试接收器先收到灰色 Marker、随后才收到对应 world snapshot 的时序失败。
两个 DDS topic 不保证回调先后；等待条件现同时确认两者到达，原状态断言全部保留。
最终定向测试、五次独立 DDS 复核及完整 Humble 均通过。

真实 `test_marker_transport.py` 通过 perception/joint/mechanism DDS 输入建立 WorldBridgeNode 快照，
订阅真实 `/unloading/markers`，确认不可规划面片、删除和完整 ADD、晚订阅 QoS、仅 /clock 前进时 watchdog
灰化、未来感知拒绝保留原来源，以及恢复后未知区域仍阻断。未替换 publish_markers 或准入函数。
`test_marker_display.py` 逐项检查实际 Marker pose/scale/四点、颜色文字、来源代表、无输入修改、坏记录和生命周期。

```powershell
.venv310/Scripts/python.exe -m pytest -q --basetemp .test-tmp/rviz-default-20260917
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q --basetemp .test-tmp/rviz-utf8-20260917
```

服务器使用基线隔离归档加本轮上传文件，原检出未修改；验证根为
`/root/autodl-tmp/v05-acceptance/rviz-display-20260917/`。
日志：`red-markers.log`、`marker-final.log`、`related-ros.log`、`default.log`、`metric.log`、
`humble-verified.log`、`demo-cli.log`；最终完整 colcon 输出在 `humble/step7-verified/`。

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
ARTIFACT_ROOT=/tmp/step7-validation RUN_ID=step7 bash tools/run_humble_acceptance.sh
source /tmp/step7-validation/humble/step7/install/setup.bash
python3 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_marker_display.py \
  ros2_ws/src/unloading_ros_bridge/test/test_marker_transport.py
# 用项目的独立 CPU/metric 虚拟环境分别执行 pytest -q 和 pytest -q tests_metric。
```

现有 Humble CI 未改；当前提交 CI 的实际状态和链接在交付回复中报告。

## RViz 演示与未验证边界

服务器在 source Humble 后没有 `rviz2` 命令，DISPLAY/WAYLAND_DISPLAY 为空，没有 X11 socket。
未重装图形环境；**RViz 目视效果未验证**，没有生成或伪造截图。RViz 配置的实际字体/布局/遮挡需要图形会话复核。
已经实际运行的是生产 Marker 发布、真实 ROS 消息/DDS、安装后的合成演示 CLI。

在已安装 Humble/RViz、有可用图形会话的机器上，先按上述命令 build/source，然后在两个终端分别执行
（两端均使用隔离的 ROS_DOMAIN_ID，避免与真实系统同名发布者混用）：

```bash
export ROS_DOMAIN_ID=77
ros2 run unloading_ros_bridge marker_demo
```

```bash
export ROS_DOMAIN_ID=77
rviz2 -d "$(ros2 pkg prefix unloading_bringup)/share/unloading_bringup/rviz/unloading.rviz"
```

每 3 秒轮换：两个完整体 → 一个完整体加面片 → 冲突面片 → 历史灰化 → 无几何；循环并测试删除。
状态全部明确为 SYNTHETIC display-only，没有发布新执行授权或运行模型、Isaac、机械规划、任务扫描。
正常重启以外的崩溃/断网清屏和真实 RViz 目视效果仍未验证。本轮止于第七步。
