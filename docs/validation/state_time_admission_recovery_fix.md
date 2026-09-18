# 状态时间准入与受控恢复

基线 `8c88c16225ca4b201225958c4349032c98dc2544`，分支 `feat/v0.5-perception-ros2`。
开始时读取 AGENTS.md、fetch，确认指定分支本地/远端一致、工作区干净。
保留默认 online、显式只读回放、感知时间故障锁存和 0.1 s 稳态 watchdog。

## 原始反例与修复

先在未修改的基线上运行真实 WorldBridgeNode、DDS、/clock 测试，得到 **3 failed，0.42 s**：

| now=100，最后合法机器人/机构时间均为 100 | 旧代码实际污染 |
|---|---|
| JointState stamp=1000，位置改为 9 | last_joint_stamp=1000，机器人内容版本增加 |
| 机构 A/999999，stamp=1000 | mechanism_stamp=1000，来源 sequence=999999 |
| 机构 B/0/restart，stamp=1000 | B 接管，A 进入 retired，mechanism_stamp=1000 |

三个反例在修复后通过：坏消息不改变任何已接纳时间、内容、版本、assembler 状态或来源守卫；
随后 now=100.1、stamp=100.1 的机器人和 A/11 正常接纳，无需重启。
不是 monkeypatch 时间、守卫或手工修复内部状态。

每个状态回调只读取一次 now，并将同一个 now 传给快照发布：

- 机器人先检查名称、数组和有限值；验证 ROS 时间编码、当前时钟及有效期、单调性；
  使用局部版本构造完整 RobotStateRevision 成功后才提交时间、内容和版本。
- 机构检查必要身份和时间，解析全部 JSON，构造局部 bundle/config/content 并验证其可生成领域指纹；
  检查时间单调性后，最后调用有副作用的 SourceEpochGuard.accept，再提交已构造的数据。
  嵌套 NaN 等内容也不能先消耗 sequence 或退役旧 epoch。
- 复用纯 CPU `state_time_reasons()` 和窄范围 ROS 编码检查，不重写共享契约或来源守卫。

ROS sec 必须处于非负 int32 范围，nanosec 必须在 [0, 10^9)；随后要求 sample_time>0 且有限。
仿真时钟必须已经初始化；机构显式 clock_domain 必须匹配节点的 ros/ros_sim_time。
合法年龄范围为 **0 <= now - sample_time <= 对应源有效期**：机器人默认 0.5 s，机构默认 2 s。
上限包含，未来容差固定为零，不继承感知的未来容差，不加 epsilon，不裁剪时间。
与既有快照和 watchdog 的 `age > limit` / `age < 0` 边界相同。

## 故障状态与来源状态分离

`state_time_blocking_reasons` 分机器人/机构各保存一组故障；
`last_state_time_rejection` 各保留一条最近诊断，包含拒绝时刻、原始 sec/nanosec、时钟域及可用来源身份。
原因使用 `ROBOT_STATE_` / `MECHANISM_STATE_` 前缀和
`TIME_INVALID`、`TIME_IN_FUTURE`、`CLOCK_NOT_INITIALIZED`、`CLOCK_DOMAIN_MISMATCH`、`STALE`。

完整输入已存在时，故障经 `_commit_snapshot()` 写入领域 scene_snapshot，随后生成一致的 ROS 显式字段和 payload：
planning_admissible=false；原合法几何和源采样时间保留。
坏消息不改变内容指纹/内容版本，但故障改变准入状态，因此世界指纹变化、执行桥撤销旧授权。
只有通过时间、结构、领域构造和来源顺序全部检查的新采样才能清除对应源故障。
重复/乱序、另一个源、定时器和时钟追平都不能代替该采样；普通顺序拒绝仍保留原规则。
最近拒绝诊断作为记录保留，是否仍阻断以 blocking reasons 为准。
恢复状态不清除感知故障，也不恢复已经撤销的 grant。

## 暂停、短暂回退与归零新会话

暂停时仅稳态 watchdog 继续唤醒，所有年龄仍使用 ROS 时间。未混用墙钟。
短暂回退沿用原有行为：旧样本位于未来时阻断；时钟追平不能清除感知锁存；
满足原顺序要求的新合法状态和新感知可以恢复，不一律要求重启。

真正归零到较小时间轴采用以下受控操作，不提供 reset 服务或自动跨 epoch 恢复：

1. 停止提交新命令，按已有控制流程确认旧执行任务结束；退出/杀死进程本身不是物理停止证明。
2. 停止旧状态源、感知/TF 发布者、世界桥、执行桥及相关测试控制器，退出旧 DDS 参与者。
3. 使用新隔离 ROS_DOMAIN_ID 启动新进程；所有必要源和消费者使用同一新 domain。
   重新建立 /clock、JointState、机构、感知及 TF 输入。复用现有 `world_bridge_node` 启动入口，
   默认 online；仿真世界节点显式设置 use_sim_time=true。等待时钟初始化和完整输入。
4. 机构/感知使用新的合法来源身份；世界桥由新进程生成新 publisher_epoch。
   执行侧重新建立上下文、控制器身份与新有效授权；不复制旧 grant 或上下文缓存。
   当前执行桥的系统 ROS 时间边界未扩展为 /clock 执行支持。

真实测试依次启动、等待退出两个独立子进程，使用不同隔离 domain，重新发布合成状态、感知和真实静态 TF。
旧进程从 100 归零后到 1，旧时间及守卫保留且不可规划；新进程在 1 建立完整合法世界。
缺机器人或机构时均不产生虚构完整世界。测试未启动执行任务，所以不存在把进程退出当作任务停止的断言。
授权另在系统 ROS 时间、真实 ExecutionBridgeNode 和已有 mock 控制器接线上验证：
故障撤销 grant，恢复后旧授权仍缺失；第二步合法短轨迹回归继续真实获得 STARTED/SUCCEEDED。

JointState 没有 clock_domain 或 source_epoch，本轮假设每个会话只有一个受控状态源，
按节点明确配置解释时间；没有借 frame_id 编码身份。只换 UUID 不能隔离旧 JointState。
同一 domain 残留旧发布者/消息的安全自动恢复、真实硬件停止、完整仿真执行均未实现或验证。

## 实际验证

复用 Ubuntu 22.04 / ROS 2 Humble / Python 3.10，不升级依赖。
隔离目录：`/root/autodl-tmp/v05-acceptance/state-time-20260918/repo`。
服务器 fetch 网络超时后，用已有父提交归档和本地已提交补丁重建精确基线，再上传本轮变更；原工作区不改。
日志在上一层：red-state-time.log、state-green.log、state-expanded.log、related-ros.log、default-final.log、humble.log。

```bash
# CPU venv，无 ROS/ML 依赖
python -m pytest -q tests/test_state_time_admission.py tests/test_ros2_workspace_static.py
python -m pytest -q

# 已构建的 Humble 环境，从仓库根目录运行
source /opt/ros/humble/setup.bash
source /path/to/humble/install/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/unloading_ros_bridge:$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
python3 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_state_time_recovery.py
ARTIFACT_ROOT=/tmp/state-time RUN_ID=state-time bash tools/run_humble_acceptance.sh
```

真实 ROS 使用端点发现和条件等待，不修改生产门控、时间/守卫或快照组装。
测试输入均明确为合成数据；不将 mock 执行或 /clock 世界测试称为硬件、感知算法或完整仿真验收。
复用时间 fixture 仅增加可选隔离 domain 的 context manager；原有断言全部保留。
静态接线测试更新为检查 `_commit_snapshot(..., now=now)`，保留且明确了一次回调共用 now 的要求。

| 检查 | 结果 |
|---|---|
| 原基线未来时间回归 | 3 failed，0.42 s |
| 新 CPU 时间边界/编码 | 28 passed；含静态接线共 31 passed，0.62 s |
| 新真实 ROS 定向 | 11 passed，3.70 s；最终断言随下行复跑 |
| 新测试及时间、心跳、回放、映射、执行回归 | 40 passed，25.29 s |
| 服务器默认轻量 pytest | 596 passed，1 deselected，11.90 s |
| 完整 Humble build/test | 3 packages，64 tests，0 errors/failures/skipped，退出 0 |
| Windows UTF-8 | 596 passed，1 deselected，48.94 s |
| Windows 默认编码 | 595 passed，1 failed，1 deselected，48.91 s；仅既有 GBK 解码失败 |

最初全量检查发现静态测试匹配旧调用形式，已同步为更明确的 now 参数检查并复跑。
Windows 既有 `tests/test_effective_scene.py:29` 的 GBK 解码问题不扩大整改，不删除测试。
未运行 Isaac、SAM/MoGe、渲染、任务扫描、完整卸货仿真或真实硬件。
本轮仅第三步；离线载荷与后续算法改造未开展。当前提交 CI 以交付时实际查询为准。
