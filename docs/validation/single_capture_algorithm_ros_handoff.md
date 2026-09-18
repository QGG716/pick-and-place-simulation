# 单组双 RGB-D 实际算法 → ROS 只读世界

基线 `1362521e8c5a7b68b034357a4c6117bc6ea465f5`，仅 `feat/v0.5-perception-ros2`。
开始读取 AGENTS.md、fetch，确认本地/远端一致、工作区干净。前四步的时间准入、历史隔离、
心跳、原始载荷验证和执行门控保留。本轮未升级依赖，未调参、训练、渲染或运行运动规划。

## 实际数据和调用链

使用既有带顶板、补光的 Isaac 归档 `roof-mast-20260916/capture-f24f63a`，不是新合成的 CPU 接线 fixture。
服务器原目录为 `/root/autodl-tmp/v05-acceptance/roof-mast-20260916/capture-f24f63a`。
两个模组的 PNG、float32 米制深度、标定、元数据、GT 注释和评价 masks 均通过同一
`load_capture_payload()` 原始 binding 校验。运行后重新核验并逐字节比较本次隔离快照与原始消费文件，全部一致。
没有改写历史数据、binding 或历史算法结果；没有重渲染或补造历史证据。

- epoch：`carton-assets-capture-f24f63a`；sequence：`100`。
- 两侧 capture_time：`1.6666666666666667`，clock：`ros_sim_time`，模组时间偏差为 0。
- capture_id：上述 epoch + `:module_0_upper:100` / `:module_1_lower:100`。
- manifest fingerprint：`291ee4b5b3ad4a95555a57791912aaab973cb1fea0fc3f406369770419fb34848`。
- 算法运行 ID：`80d4988224e04069bdee66eab0f5695f`。
- 输出根：`/root/autodl-tmp/v05-acceptance/single-handoff-20260918/algorithm-run-01`。

复用 `run_workcell_perception_once.py` → `oracle_proposals/infer` → 既有 SAM resident runtime →
`_worker_artifacts` → `_run_secondary_module` → 既有米制几何与每模组领域观测。
新增 `--output-directory`，保留原默认目录和拒绝覆盖规则；输入检查后、模型初始化前，复用现有 fusion 契约检查采集组。
每模组只调用一次既有 SAM 和一次几何处理，无自动重试、无 MoGe 比较。

本次返回的 observation / observed_face_sets 直接构成 `ModuleFaceBatch`，调用
`fuse_module_face_batches()` 和 `fused_algorithm_observation()`。未使用 GT 身份进行关联。
融合 coverage 补充保留单模组 provenance、oracle 声明和未知区域；空结果也明确未知空间。
单侧失败不发布完整组索引，不读取旧侧结果补齐。已有原子 UTF-8 汇总和非零失败退出保持有效。

新增 `algorithm_artifact.py` 仅负责正式产物和回放边界：原子保存单模组观测、fusion_result、
fused_algorithm_observation，最后保存带文件哈希的 algorithm_artifact 索引。
索引关联运行 ID、模型 manifest、上游提交、配置身份、两侧观测及融合文件；原始 binding 和隔离文件身份
沿 module_coverage.input_provenance 保留。原始 PNG 不重编码，RGB NPY 不作为输入。

## 本次真实模型结果

SAM：`facebook/sam-vit-base@70c1a07f894ebb5b307fd9eaaee97b9dfc16068f`。
视觉上游：`1d208f2ed380a207e6e46b4a62d2ac640edfe477`；几何沿用 seed=17 和既有配置。
仍为 `ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL`，`raw_image_automatic=false`；GT masks 仅用于既有评价。

| 模组 | SAM 实例 | SAM 调用/耗时 | 几何调用/耗时 | 观测面片 | 无可接受面实例 | 完整 cuboid |
|---|---:|---:|---:|---:|---:|---:|
| 上 | 16 | 1 / 10.230 s | 1 / 142.341 s | 23 | 0 | 0 |
| 下 | 31 | 1 / 19.153 s | 1 / 264.484 s | 48 | 1 | 0 |

耗时为生产编排阶段的实际 wall time，包括该阶段现有读写/检查；完整阶段记录见 summary.json。
首次真实算法运行退出 0，无技术失败或模型重跑。最终代码另补记 handoff 完成阶段的耗时，未为此重复推理。
融合为 37 个对象记录，其中 10 个关联对象；保留 71 个原贡献面片、71 个代表面、0 个冲突。
原始融合观测保留 84 条未知区域记录；零完整体没有被提升为完整箱体或可执行结论。

| 实际图像 | 上模组 | 下模组 |
|---|---|---|
| 原始 RGB | [PNG](evidence/single-handoff-20260918/algorithm/module_0_upper/sensor_rgb.png) | [PNG](evidence/single-handoff-20260918/algorithm/module_1_lower/sensor_rgb.png) |
| SAM 分割边界 | [PNG](evidence/single-handoff-20260918/algorithm/module_0_upper/sam_boundaries.png) | [PNG](evidence/single-handoff-20260918/algorithm/module_1_lower/sam_boundaries.png) |
| 米制面片 | [PNG](evidence/single-handoff-20260918/algorithm/module_0_upper/v4_validation/final_metric_faces_overlay.png) | [PNG](evidence/single-handoff-20260918/algorithm/module_1_lower/v4_validation/final_metric_faces_overlay.png) |

这些均为原始输入或实际算法输出的字节副本，无人工修饰边界。
[融合观测](evidence/single-handoff-20260918/algorithm/fused_algorithm_observation.json)、
[融合诊断](evidence/single-handoff-20260918/algorithm/fusion_result.json)、
[算法汇总](evidence/single-handoff-20260918/algorithm/acceptance_summary.json) 和
[文件哈希清单](evidence/single-handoff-20260918/evidence_files.json) 一同归档。
产物内部绝对引用保留服务器原路径，不为下载副本改写身份。全量深度、SAM masks/worker 产物仍保存在服务器本轮目录。

## 正式 ROS 接线与实际接收

`PerceptionNode backend=algorithm_replay` 校验期望索引哈希、领域类型、原始算法身份和所有引用产物哈希，
不使用 CargoJsonReplayBackend 解析领域观测，不推理或再次融合。
先验证原始观测，再生成显式回放 envelope：外层 clock=replay、capture=0、独立 session，
原 provider/epoch/sequence/采集时间/时钟域在 replay.original_identity 中保留，面片自身的时间、sensor_epoch、T_W_C 不变。
WorldBridgeNode 重建原始身份并执行算法校验和指纹比较后才接纳该回放，未删除或绕过算法校验。

新 `algorithm_replay.launch.py` 只启动领域文件发布、已有 mock_state_publisher 和真实 WorldBridgeNode。
原始绑定没有完整历史机器人状态，因此 mock 仅为 `DISPLAY_ONLY_MOCK_NOT_MEASURED_AT_CAPTURE` 展示辅助，
不是采集时刻实测状态。无 Isaac 真值发布者、执行桥、轨迹控制器或 /clock 操纵。

`probe_algorithm_replay.py` 在 ROS_DOMAIN_ID=171 实际订阅 `/unloading/perception`、
`/unloading/world_snapshot`、`/unloading/markers` 和执行授权话题：

- 实际只有 `unloading_perception` 一个感知发布者；收到本次运行 ID 和原融合文件哈希。
- ROS mapping 往返后的完整观测指纹一致；world 的面片坐标、米制单位、时间、来源、代表映射和冲突诊断一致。
- 世界保留 37 个对象、71 个贡献面；MarkerScene 显示 71 个代表面，逐点比对确认未再次乘 T_W_C。
- 世界保留 121 条未知区域记录：84 条原始记录加既有世界组装器对 37 个无完整 pose/volume 对象的提示。
- `planning_admissible=false`，包含 `HISTORICAL_REPLAY_DISPLAY_ONLY`、`UNKNOWN_OR_UNTRANSFORMED_REGIONS`；
  本次接收还存在 `ROBOT_STATE_STALE_OR_TIME_JUMP`，如实保留，未扩大 freshness 或修改状态时间。
- 重复发布的采集身份、来源时间、几何内容不变；晚启动 Marker 订阅者通过；执行授权为 0。

首次 ROS 探测手工转录摘要时多写一位哈希，在文件边界被拒绝；保留 ros-run-01.log。
第二次从算法 summary 直接读取期望值后通过，仅重新启动轻量 ROS，没有重复模型运行。
[实际 DDS 报告](evidence/single-handoff-20260918/ros/report.json)、
[世界快照](evidence/single-handoff-20260918/ros/world_snapshot.json)、
[Marker 数据](evidence/single-handoff-20260918/ros/markers.json) 已归档。
服务器 DISPLAY 为空，未做 RViz 目视检查，没有伪造截图或安装图形环境。
探测结束主动发送 SIGINT 后，既有 mock/perception 节点日志有重复 rcl_shutdown 异常；
运行中的 DDS/Marker 断言全部通过，探测器确认进程已退出。本轮不扩大为节点退出处理重构。

## 回归、复现与边界

- 新增 CPU 接线 9 项通过；使用明确合成 fixture，实际执行生产融合、领域产物校验和回放封装。
- 载荷、单次退出、handoff、融合及回放相关回归 168 passed；最终补充复核 60 passed。
- Windows PYTHONUTF8=1：628 passed，1 deselected；服务器默认轻量：628 passed，1 deselected。
- 完整 Humble：3 packages，67 tests，0 errors/failures/skipped；包括新增真实 launch/DDS、坏哈希、在线隔离与原始身份回归。
  首轮坏哈希测试因 ROS 把全零值推断成整数而失败，修正字符串参数后完整复跑通过，保留首轮日志。
- 真实模型、合成回归、真实 DDS 为上述三种不同证据；没有进行连续性能或传感器准确率验收。
  历史 Isaac 图像不是硬件相机实测；普通文件哈希不证明传感器真实性或抵御文件与 binding 联合篡改。

在本轮服务器代码目录运行；NEW_RUN 和 NEW_ROS 必须是不存在的路径：

```bash
CAPTURE=/root/autodl-tmp/v05-acceptance/roof-mast-20260916/capture-f24f63a
NEW_RUN=/root/autodl-tmp/v05-acceptance/NEW_ALGORITHM_RUN
/root/v05-gpu-venv/bin/python tools/run_workcell_perception_once.py \
  --capture "$CAPTURE" --vision /root/vision-fixed \
  --models /root/autodl-tmp/v05-acceptance/model-manifest.json --output-directory "$NEW_RUN"

source /opt/ros/humble/setup.bash
source /root/autodl-tmp/v05-acceptance/single-handoff-20260918/humble/single/install/setup.bash
export PYTHONPATH="$PWD/ros2_ws/src/unloading_ros_bridge:$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
DIGEST=$(/usr/bin/python3 -c 'import json,sys; print(json.load(open(sys.argv[1],encoding="utf-8"))["algorithm_artifact"]["sha256"])' "$NEW_RUN/summary.json")
/usr/bin/python3 tools/probe_algorithm_replay.py --artifact "$NEW_RUN/algorithm_artifact.json" \
  --artifact-sha256 "$DIGEST" --output-directory /root/autodl-tmp/v05-acceptance/NEW_ROS --domain-id 171
# 持续只读显示：同样使用专用 ROS domain，无需模型进程
ROS_DOMAIN_ID=171 ros2 launch unloading_bringup algorithm_replay.launch.py \
  artifact:="$NEW_RUN/algorithm_artifact.json" artifact_sha256:="$DIGEST"

python -m pytest -q tests/test_algorithm_artifact.py tests/test_workcell_perception_once.py
python -m pytest -q
ARTIFACT_ROOT=/tmp/single-handoff RUN_ID=single bash tools/run_humble_acceptance.sh
```

完成单组贯通后停止，不扩展多组连续处理。
