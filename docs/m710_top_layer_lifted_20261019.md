# 连续顶层任务：coupled 初态阻塞（2026-10-19）

**本轮未完成所要求的连续卸层流程，完成 0/5。** 完整 40 箱和原始 q 下，新增 coupled 机构的初态几何门拒绝。没有运行抓放路径求解、附着、释放、名义下落或清空；不能把本报告、初态检查或几何查看器当成连续批次成功。`IDEAL_RECEIVER_DRAIN` 使用次数为 0，名义首次触带为 0，实测接收为 0。

[三维阻塞查看器](experiments/m710_top_layer_lifted_20261019/initial-rejection.html) · [离线 ZIP](experiments/m710_top_layer_lifted_20261019/offline-inspection.zip) · [初态诊断视频](experiments/m710_top_layer_lifted_20261019/initial-rejection.mp4) · [机器汇总](experiments/m710_top_layer_lifted_20261019/summary.json) · [原始记录](experiments/m710_top_layer_lifted_20261019/raw-records.zip)。视频仅为 8 秒静态场景检查展示，640×360、5 fps；不是完整批次视频，没有机器人或箱体运动。

## 阻塞证据

初始横带面高为 0.600 m。同源 `coupled_conveyor_state()` 生成 139 个对象，其中 40 个为原箱体。保留所有杆、销、导轨、支座、横带、纵带、底盘和环境；没有切回旧纯 Z 单盒场景。

| 实体对 | 原始低位结果 |
|---|---|
| `conveyor_longitudinal` / `lift_rear_stand_0` | 实体 OBB 相交 40×80×70 mm，体积 0.000224 m³ |
| `conveyor_longitudinal` / `lift_rear_stand_2` | 实体 OBB 相交 40×80×70 mm，体积 0.000224 m³ |
| `chassis` / `lift_longitudinal_rear_foot_1` | 未声明接触，间距 0；非装配对要求 5 mm |
| `chassis` / `lift_longitudinal_near_foot_1` | 同上 |
| `chassis` / `lift_rear_bearing` | 同上 |

两个相交体的世界坐标范围分别为：

- `stand_0`：X=[−3.000,−2.960]，Y=[−1.080,−1.000]，Z=[0.480,0.550] m。
- `stand_2`：X=[−3.000,−2.960]，Y=[−0.500,−0.420]，Z=[0.480,0.550] m。

这是现有项目概念盒体的精确轴对齐交集，**不是原始 CAD 穿透证明**。一般倾斜姿态采用 OBB SAT 和欧氏表面距离复核，不把 AABB 包络相交当作窄相碰撞，不把 SAT 数值当作一般穿透深度。后端支座与轴承、轴承与纵带分别具有具名连接；不能据此豁免支座与纵带的第三对接触。

从实际低位到 Hmax 的同步几何诊断覆盖完整高度区间，147 个状态，最大高度步长约 9.932 mm；147 个均被拒绝。额外支持上升、下降及保持高度的定向回归。该诊断没有执行升降；离散状态之间的碰撞仍未被连续证明，机器人运动扫掠也未评估。已知冲突不能通过换种子、换抓面或延长搜索预算消除。

附件要求“不得通过……修改尺寸、改初态……制造 5/5”，并要求初始模型不相容时报告证据。因此保留这些冲突，没有缩短支座、挪动底盘、删除机构或扩大碰撞豁免以启动抓放。后续必须先解决上述实体/装配模型问题；本轮没有擅自变更其几何或权限。

## 原生检查与批次口径

种子集合在运行前固定为 `[71070,71071]`。两份独立原生初态检查均为 `SUCCESS`，官方机器人与完整工具在原始 q 下通过；它们与 CPU 机构检查收到同一个完整世界。排序规范化后的世界 SHA-256 为：

`963b0e2a501b516f304e62022044ec974503438b2e4ce7a270be446ef1ead504`

原生 `check_start` 检查机器人/工具，新增门检查机构与场景及机构之间的实体关系；两个结果不能相互替代。没有 IK、OMPL、LIN、PTP 或先验图调用，没有可接受的非零运动边。

| 种子 | 批次状态 | 完成 | 剩余 | 实际卸箱顺序 |
|---|---|---:|---|---|
| 71070 | `BATCH_START_REJECTED` | 0/5 | 初始顶层全部 | 空 |
| 71071 | `BATCH_START_REJECTED` | 0/5 | 初始顶层全部 | 空 |

顶层由最高几何顶面与支撑关系识别，冻结得到 `carton_l07_c00` 至 `carton_l07_c04`；算法不解析这些名字。两份记录均保持 40→40 箱，下方 35 箱完整保留。冻结左右端角色，避免只剩单箱时重写原角色；新暴露下层不进入本批任务。

当前几何下，c02/c03/c04 的正面直出候选使用 Hmax=2.050 m；c00 和 c01 的完整底面超出横带 −Y 边界，正抓候选使用 `+Y→−X / Hmin=1.750 m`。原右端 `y+ / Hmax` 与原左端 `y− / Hmin` 作为侧抓候选保留。这里记录的是上游几何工艺选项，**没有宣称这些选项已规划成功**。

这不是两个已执行完整批次，也不是 0% 真机成功率或全局不可达证明。问题位于 `INITIAL_COUPLED_WORLD`，与释放准备或区域搜索耗尽不同。

## 实施范围与尚未交付部分

- 已实施：冻结顶层集合、原层角色与高度；按完整投影分派正面直出/横移和侧抓候选；使用当前 q 的排序基础；同源 coupled 具名装配检查、精确冲突记录、双向高度区间诊断；服务器原生初态探针；离线三维阻塞查看器。
- 尚未实施/验证完整闭环：新的原生连续批次入口、原生 coupled 外部轴阶段提交、无先验 RELEASE 及 Hmin 名义下落、必要分离和 IDEAL_RECEIVER_DRAIN 的运行整合、逐箱终态提交与回退。
- 未交付完整批次轨迹、连续运动回放和完整批次视频。所附查看器及视频明确标为初态诊断，不能替代这些交付项。没有调用旧 `plan_box` 充当新流程。

停止原因是已验证的固定模型冲突，并非已完成一个里程碑后把剩余目标视为可选。未对尚未到达的释放、接收和后缀环节作成功声明。

## 测试、源码与计时

起始远端与本地均为 `71b693a84f6c8ff7f932d9f6910e4fe4090178dd`，开始前已重新 fetch 核对。分阶段代码：

1. `a93ebbb4a9ed866c552f28ef5e86f885f6a840fe`：实测规划前置源码；冻结目标、工艺分派、coupled 门和原生探针。
2. `58ca94fe604330de1a1206def4dec9aa32e2ee6a`：同世界离线阻塞查看器。
3. 后续交付提交包含浏览器录帧脚本、报告和实测证据；最终 SHA 与推送状态在交付回复中给出。

服务器使用现有 Ubuntu 22.04 / ROS Humble / `/opt/planning-opt-20261003-venv`，未升级依赖。源导出逐项校验 254 个实现/配置文件；复用官方模型与已有 worker，worker SHA-256：

`0c5c94fef7f01a460c1a96f15c9be6cc2960b1da44171c1b01b0de19a67771472`

服务器工作位于独立 `/work-top-layer-lifted-run-20261019`。既有 Git 对象不完整，普通 worktree 检出失败；随后使用本地实测提交的 Git archive 和逐文件清单建立隔离源快照。历史源码和资产未改动，未切换其他分支。

- 定向测试：**155 passed，2.87 s**。新增数值相交证据、具名权限、名称伪装不豁免、上下行/保持、顶层冻结和中间箱越界，并回归旧单箱接触、抽离、Hmax、右侧和完整候选接受。
- 旧 `test_workcell_layout.py`：**9 passed / 4 failed，1.36 s**；失败名称与 `71b693a` 报告一致。相关旧测试、布局实现和配置相对基线逐文件未变；没有删除断言。详见原始日志。
- 浏览器检查：无脚本异常；完整世界、两处相交特写、原 q 与世界身份保持；40 帧的 640×360 检查展示。
- 保留开发阶段错误：隔离源资产软链接触发根路径保护、数值测试支座左右编号写反，以及未完成传输时的测试启动错误；均不计入正式通过数量。初版视频面板定位问题修复后重新检查，旧产物仍在本地 artifacts 中。

| 计时项目 | 实测 |
|---|---:|
| 初始化 | 0.458 s |
| 初始机构世界检查 | 0.056 s |
| 两份原生初态探针合计 | 1.354 s |
| 147 状态诊断检查 | 9.716 s |
| 原生机器人轨迹时长 | 0 s（未产生轨迹） |
| 真实升降/接收时长 | 未知/未运行 |
| 诊断视频展示时长 | 8 s（不表示工艺时间） |

未把嵌套时间重复累加，未声称完整节拍、吞吐量或成功率改进。

## 复现

在源提交的完整检出根目录、现有 ROS/规划环境中运行；输出目录必须是新目录：

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT="$PWD"
/opt/planning-opt-20261003-venv/bin/python tools/audit_m710_coupled_batch_start.py \
  --source-commit "$(git rev-parse HEAD)" \
  --worker-command /tmp/right-inner-20261019/worker.sh \
  --worker-binary /tmp/right-inner-20261019/build/m710_moveit_worker \
  --seeds 71070 71071 --height-step-m .01 --output /tmp/new-coupled-start-audit
```

预期退出码为 **2**，状态为 `BLOCKED_BY_INITIAL_COUPLED_WORLD`。服务器实测隔离快照另外传入 `--source-manifest source-manifest.json`，实测命令保存在原始记录的 `run.sh` 中。

```bash
python -m pytest -q tests/test_coupled_lift_checks.py tests/test_top_layer_lifted.py \
  tests/test_conveyor_lift.py tests/test_lifted_reception.py tests/test_single_contact.py \
  tests/test_single_extraction.py tests/test_single_lifted_receiver.py \
  tests/test_right_receiver.py tests/test_receiver_candidate_acceptance.py
python tools/export_m710_coupled_blockers.py /tmp/new-coupled-start-audit \
  --output /tmp/initial-rejection.html
node tools/record_m710_blocker_inspection.mjs /tmp/initial-rejection.html /tmp/blocker-frames /path/to/chrome
ffmpeg -framerate 5 -i /tmp/blocker-frames/frames/frame-%03d.png \
  -c:v libx264 -pix_fmt yuv420p -movflags +faststart /tmp/initial-rejection.mp4
```

所有结果保持 `qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`、`execution_ready=false`。
