# 有限多组算法处理与同一 ROS 图内切换（2026-09-18）

基线为 `c2b43022f70f1be1248767b5f45a07bd899939c5`，分支
`feat/v0.5-perception-ros2`；开始时工作区干净，本地与 fetch 后的远端一致。
本轮只实现有限独立采集组的顺序处理，不是连续采集、实时系统或跨帧关联。

## 实际结果

一次启动处理 A → B → C，使用一个 WorldBridge、一个 mock 状态源和一个领域发布进程。
A、B 的两组不同可信 RGB-D 输入完成了算法产物到 ROS 的交付；C 的下模组技术失败，
没有正式融合索引、没有提交 ROS 切换，保留 B 为最后接纳组。批次如实返回 **1**，
`PARTIAL_OR_FAILED`，没有重新推理、替换失败组或调整几何阈值。

| 原始组 | 本轮动作 | SAM / 米制几何调用 | 算法耗时 s | 对象 / 原始面片 / 代表面 | 冲突 / 算法未知区域 | ROS / 首条接收延迟 s | 错误 |
|---|---|---:|---:|---:|---:|---|---|
| A `roof-mast-20260916/capture-f24f63a` | `REUSED_VERIFIED` | 0 / 0 | 不适用（复用） | 37 / 71 / 71 | 0 / 84 | 接纳 / 2.375 | 无 |
| B `roof-mast-20260916/capture-acc6b8c` | 新算法运行 | 2 / 2 | 401.802 | 37 / 57 / 57 | 0 / 84 | 接纳 / 2.657 | 无 |
| C `carton-assets-20260915/capture-08748ff` | 新算法运行 | 2 / 2 | 305.268 | 无完整组结果 | 无完整组结果 | 未提交 | 下模组 `metric_geometry`: `ValueError: FACE_BEHIND_CAMERA` |

调用数为每组上下模组合计；B、C 每个模组各一次 SAM、一次几何调用。C 上模组完成，
下模组既有几何流程抛出异常，不能伪造成合法空结果。原始 summary 保留 1 完成 / 1 失败；
组级 handoff 原因为 `MISSING_CURRENT_RUN_MODULE`。本轮不修改该几何流程来挑选成功结果。
技术失败原因与“合法零完整体”不同；合成空结果仍正常接纳、清除旧几何并保留未知空间。

实际批次 ID：`6004445febb64e33bdde5d4bce591d5d`。
世界发布者 epoch：`4c34ea47-e5c9-4eba-9cc6-e99837a92dd1`；
mock epoch：`e75174ec-20bb-4225-8b45-979b298b98e4`；领域发布 PID：`71741`。
A、B 的上述身份一致，接纳 artifact hash 分别为：

```text
A c91e80018c7b46b98bcf7b701da5faa2869e5b22f4e97993bc8721bd5ebc7b58
B 9b792fb16245485ab5d43ddd1cf0cbb3a3ae1802ed2f32faa2efb1064885d463
```

B 接管时实际 Marker 消息包含 **142 个旧几何键的 DELETE**，只保留 B 的 57 个代表面。
世界中的未知区域为每组 121（算法 84，加领域组装的保守未知区域），没有将未知解释为自由空间。
全部记录 `planning_admissible=false`、`HISTORICAL_REPLAY_DISPLAY_ONLY`；执行授权为 0。
辅助状态只是当前展示用 mock 状态，不冒充历史采集时刻实测值。

## 输入与复用边界

[采集核验清单](evidence/finite-multi-20260918/capture-inventory.json)记录实际检查过的原始采集。
选择的三组均通过已有 `load_capture_payload(..., with_instance_masks=True)`，逐模组核验 RGB、
米制深度、标定、元数据及实际消费的评价文件；通过已有双模组融合身份/时间契约。
原始 `T_W_C`、capture_id、epoch、frame_sequence、capture_time 不变。
appearance-ab 的两个旧组缺原始 metadata 绑定，不使用，也未补算历史 expected hash。
三个独立实验恰有相同数值帧时间，不代表同一连续时间轴；其 epoch、原始 RGB 内容和 binding 不同。

A 来自第五步 `single-handoff-20260918/algorithm-run-01`，原索引及 summary 未改写。
新入口对复用与新产物执行同样的完整 artifact 校验，再比较当前选定输入的原始绑定、
消费文件哈希、上下模组名、模型 manifest/config identity 和 run_id。
启动前核对实际 SAM 文件哈希和既有 vision 提交
`1d208f2ed380a207e6e46b4a62d2ac640edfe477`。
SAM 仍是既有固定模型，MoGe 未运行；GT mask 仅用于既有评价。
保持 `ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL`、`raw_image_automatic=false`，不声称自动检测。

## 实现与运行

- `tools/run_finite_workcell_sequence.py`：1..16 组有序清单、逐组校验、独立输出、调用既有单组 CLI、原子 UTF-8 进度；重型算法使用独立 Python，不继承 ROS 的 Python 路径。
- `src/unloading_perception/finite_sequence.py`：有限 manifest、原始输入与复用一致性检查；逐模组释放输入数组。
- `finite_replay_node.py` / `finite_algorithm_replay.launch.py`：一个持久图，后台单线程校验完整产物；每组新回放 UUID，用既有 sequence-zero/source_restart 接管，不重置来源守卫。
- WorldBridge/Marker 仅添加展示用进度文字。原有 ReplayRecordCache、HeartbeatTemplate、容量上限、时间判断、几何生命周期和执行门控不变。

进度区分 `PENDING`、`VALIDATING_INPUT`、`ALGORITHM_RUNNING`、`ARTIFACT_READY`、
`ROS_ACCEPTED`、`FAILED`，保留阶段、错误、模块 summary、实际调用计数。
`target_task` 与 `displayed_task` 分开；Marker 明示目标状态和最后经 ROS 确认的组。
坏输入/算法失败不生成索引，独立后续组可继续；最终任一失败非零。
输出目录已存在直接拒绝，不覆盖原始输入或旧结果；没有自动重试。

发布器保持 seq=0 直至收到新 epoch 的实际世界消息，随后推进传输序号。
在实际身份接纳后使用固定 5 秒初始化 + 5 秒稳定观察，并完整比较产物来源、领域对象/面片、
未知区域和 Marker 坐标/删除键后写收据。Runner 最多等待收据 35 秒；轮询的是明确确认文件，
不是用固定 sleep 猜 topic 顺序。传输会话变化不修改原始传感器身份。

在已有 Ubuntu 22.04 / Humble / Python 3.10 验收服务器，先使用现有脚本构建，再一次启动：

```bash
ROOT=/root/autodl-tmp/v05-acceptance/finite-multi-20260918
cd "$ROOT/repo"
ARTIFACT_ROOT="$ROOT" RUN_ID=finite bash tools/run_humble_acceptance.sh
source /opt/ros/humble/setup.bash
source "$ROOT/humble/finite/install/setup.bash"
export PATH="/usr/bin:/bin:$PATH"
export PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
/usr/bin/python3 tools/run_finite_workcell_sequence.py \
  --manifest configs/isaac/finite_capture_sequence.example.json \
  --output "$ROOT/batch-01" \
  --models /root/autodl-tmp/v05-acceptance/model-manifest.json \
  --vision /root/vision-fixed --algorithm-python /root/v05-gpu-venv/bin/python \
  --domain-id 174
```

这是本轮已执行的命令；`batch-01` 已存在，重复执行会被拒绝。新运行必须选择新目录，
RUN 会执行实际模型；复核本次结果只需读取归档，不要无意义重跑。
示例清单为该服务器的真实绝对路径；移植时修改清单的路径，不修改原始 binding。
离线统计命令（不运行模型/ROS）：

```bash
/usr/bin/python3 tools/summarize_finite_sequence.py "$ROOT/batch-01" \
  --output "$ROOT/timing-report.json"
```

## 完整切换窗口

[timing-report.json](evidence/finite-multi-20260918/timing-report.json)含产物完成、提交、首条接收的
同主机 monotonic 时间，以及 p50/p95/max。收据保留提交前最近 10 条与从提交到核验完成的
全部世界消息；未删过期消息。初始化固定取首条对应世界消息后 5 秒，稳定段从其后开始，
不重复延长窗口寻求通过；异步完整核验期间收到的额外消息也保留。

| 窗口 | 世界条数 | 接收间隔最大 s | 评估间隔最大 s | 机器人评估/接收年龄最大 s | 机构评估/接收年龄最大 s | 状态过期 |
|---|---:|---:|---:|---:|---:|---|
| A 冷切换/初始化 | 10 | 1.221 | 2.445 | 1.512 / 2.854 | 1.512 / 2.854 | 1 条机器人过期 |
| A 稳定 | 19 | 0.878 | 0.900 | 0.070 / 0.175 | 0.070 / 0.175 | 无 |
| B 切换前后/初始化 | 30 | 1.237 | 2.052 | 1.192 / 2.155 | 1.192 / 2.155 | 1 条机器人过期 |
| B 稳定 | 28 | 0.635 | 0.700 | 0.068 / 0.172 | 0.068 / 0.172 | 无 |

两个稳定窗口内，机器人 0.5 秒、机构 2 秒的原阈值在评估和接收端均满足。
冷切换接收年龄仍可越限，不能宣称新内容实时切换，世界心跳也不是算法频率。
C 没有合法完整产物，因此没有 C 切换或稳定统计，不拿 B 的消息冒充 C。

## 证据、图像与测试

[证据目录](evidence/finite-multi-20260918/)保存原始 progress、模型/有限清单、单组 summary、
原始 binding/provenance、完整 DDS 收据与压缩 world/Marker JSON、算法产物副本及日志。
`sha256.json`校验归档字节；压缩 JSON 解压后的内容未改写，原索引中的服务器绝对引用也未改写。
大尺寸深度/mask/逐点审计仍保留于服务器本次新运行目录，不重复提交到 Git。
Windows PowerShell 测试日志仅转存为 UTF-8，保留完整输出和失败信息；ROS launch 日志无损 gzip 保存。

| 新处理组/模组 | 原始 RGB | 实际 SAM 边界 | 实际最终米制面片 |
|---|---|---|---|
| B 上 | [RGB](evidence/finite-multi-20260918/B_acc6b8c/module_0_upper/sensor_rgb.png) | [SAM](evidence/finite-multi-20260918/B_acc6b8c/module_0_upper/sam_boundaries.png) | [面片](evidence/finite-multi-20260918/B_acc6b8c/module_0_upper/final_metric_faces_overlay.png) |
| B 下 | [RGB](evidence/finite-multi-20260918/B_acc6b8c/module_1_lower/sensor_rgb.png) | [SAM](evidence/finite-multi-20260918/B_acc6b8c/module_1_lower/sam_boundaries.png) | [面片](evidence/finite-multi-20260918/B_acc6b8c/module_1_lower/final_metric_faces_overlay.png) |
| C 上 | [RGB](evidence/finite-multi-20260918/C_08748ff/module_0_upper/sensor_rgb.png) | [SAM](evidence/finite-multi-20260918/C_08748ff/module_0_upper/sam_boundaries.png) | [面片](evidence/finite-multi-20260918/C_08748ff/module_0_upper/final_metric_faces_overlay.png) |
| C 下 | [RGB](evidence/finite-multi-20260918/C_08748ff/module_1_lower/sensor_rgb.png) | [SAM](evidence/finite-multi-20260918/C_08748ff/module_1_lower/sam_boundaries.png) | **未生成：几何阶段技术失败** |

C 下模组保存的 `rgbd_cuboids_baseline.png` 仅为失败前诊断，不冒充最终米制面片。
B 原始 RGB 明显偏暗，原字节及绑定哈希已复核，没有为了展示而提亮或改边界。
没有 RViz 图形环境，交付实际 DDS/Marker 证据，没有伪造 GUI 截图。

- 新 CPU 定向回归：7 passed，调用生产有限编排与单组编排，仅模型/worker 和 CPU 用运输替身被明确替换。覆盖 A 成功/B 输入坏/C 成功、单组 CLI 技术失败后继续、复用身份、模组不匹配、重复输入拒绝。
- payload/artifact/replay 相关回归：69 passed（含最初 6 个有限编排测试）；补充 CLI 失败用例后定向 7 passed。
- 真实 DDS 合成回归：1 passed，24.22 秒；同图 A 成功/B artifact hash 坏/C 合法空、失败显示标签、旧 Marker 删除、缓存换组、晚订阅者、重复 seq0/迟到已退役会话、辅助源停止后过期与恢复。未替换生产时间、守卫、映射、组装或发布代码。
- 现有 Humble build/test：3 packages、70 tests、0 errors / 0 failures / 0 skipped。
- Linux 默认轻量 pytest：636 passed、1 deselected（仓库既有配置），18.41 秒。
- Windows 最终 `PYTHONUTF8=1` 默认轻量测试：636 passed、1 deselected，106.81 秒。此前默认 GBK 实测 634 passed、1 failed、1 deselected（当时尚未追加第 7 个定向用例）；既有 `test_effective_scene.py` 历史 JSON 解码失败仍保留，没有编码范围扩张或删除测试。

初次手工 colcon 构建误选系统 PATH 中的 miniconda Python，缺 `em`；保留失败日志。
随后用仓库既有 Humble 验收脚本选择 `/usr/bin/python3.10` 成功，未升级依赖。
补充 CPU 回归首次使用默认临时根时被 Windows ACL 拒绝，改用工作区独立 `--basetemp` 后通过，
没有修改测试断言。
批次退出时 SIGINT 清理还触发既有 mock 节点重复 `rclpy.shutdown()` 的 RCLError；
它发生在 A/B 接纳和 C 失败之后，不是切换期间状态源重启。原始 launch 日志保留，
本轮未扩大修改 mock 生命周期；批次仍以 C 的原始技术失败非零退出。

未验证三组全部算法成功、真实在线相机、自动 proposals、连续跟踪、跨组融合、实时性能或机器人执行。
两组真实交付成功不等于算法质量或可执行性验收；C 的技术失败及冷切换过期明确保留。
