# ECO65-B 四箱紧凑桌面样机：单箱完整任务

本轮从 `23e539097c1c91b6f045e44ba88745d860c899ca` 继续；开始时本地和远端 HEAD 一致，工作区干净。只修改 `feat/v0.6-eco65-desktop`，不修改来源锁、不合并 main、不重新转换 CAD。

九箱配置 `configs/workcells/eco65_desktop_unloading_layout_v1.yaml`、九箱 `layout_v1_20260917_candidate02` 输出，以及原单箱 `outputs/eco65_desktop_round1/round1` 全部保留。当前默认配置为 `configs/workcells/eco65_desktop_unloading_compact_v1.yaml`，layout_id 为 `eco65_desktop_unloading_compact_v1`。

## 尺寸与假设

这是一版 DESIGN_CANDIDATE。1400×900 mm 是设计约束，不是用户办公桌实测尺寸。所有机器人、关节、真实吸具和转接架均保持 1:1。

| 项目 | 本轮值（米） |
|---|---|
| 桌面 | X [-1.000, 0.400]，Y [-0.450, 0.450]，顶面 Z=0，假设厚度 0.030 |
| 厢体净空 | X [-0.600, 0.300]，Y [-0.380, 0.380]，Z [0, 0.650] |
| 外板 | 左、右、后、顶全部保留，12 mm 向净空外布置；透明只影响显示 |
| 厢底 | 与桌面共用一个实体，没有额外 12 mm 底板抬高 |
| ECO65-B 基座原点 | [-0.360, 0.100, 0.180] |
| 固定安装座 | 0.140×0.140×0.180；无轮子、移动轴或在线升降 |
| 箱体 | 0.220×0.220×0.160，1 排×2 列×2 层，列间隙 0.010，层间支撑接触 |
| 箱中心 | X=0.110，Y=±0.115，Z=0.080 / 0.240；每箱 0.30 kg 为假设 |
| 横带 | X [-0.260,-0.020]，Y [-0.090,0.310]，顶面 Z=0.180，沿 -Y |
| 纵带 | X [-0.920,-0.020]，Y [-0.350,-0.090]，顶面 Z=0.180，沿 -X |
| 移载区 | X [-0.260,-0.020]，Y [-0.350,-0.090]，是纵带前端逻辑分区，没有叠加第二实体 |
| 最终停箱中心 | [-0.780,-0.220,0.260]；整箱 X [-0.890,-0.670] 在厢口 -0.600 之外 |

上述布局参数未偏离用户内嵌输入。安装座无需扩大：官方底座网格相对基座的 XY 包络约 X [-54.973,61.250] mm、Y [-54.957,54.957] mm，在 140×140 mm 顶面内。底座没有缩放。完整静态部件（含桌面、外板、安装座、机架、支脚）占地为 1.400×0.900 m。

带面厚 4 mm、机身厚 40 mm、支脚宽 18 mm，端部 20 mm 接收禁区单独记录。横纵带零间隙共边；纵带直线段与前端移载区在功能记录中分开，物理带面没有重复建模。本版不创建旧方案出口桥板或独立承接台。静态整箱底面支撑检查覆盖 A/B 两条路线；实际执行结论另列，不能由支撑检查代替。

真实桌面承载、安装座强度、板厚设计与螺栓、90°移载驱动、摩擦、压缩、气路独立性、工具质量及实测 TCP 均未验证。沿用此前已明确的有限质量、转接架和 ideal_suction 几何假设，12 条真实 CAD 轮廓全部检查，不关闭部分吸盘。

## 代码修复

- 布局箱数、最高层、支撑依赖、功能分区、来源映射由配置生成。默认四箱，九箱通过原 YAML 显式调用。HOME 由当前场景搜索，不读取历史 HOME 文件。
- 静态碰撞、绘图、规划、验证、执行和录像使用同一哈希校验的场景与资产。选中目标同步更新 ID、尺寸、活动位姿和 `scene["box"]`。
- `PlanningResult.failed` 改为关键字参数，分别处理超时、逻辑取消、无 IK、碰撞候选耗尽与路径搜索失败。有限搜索失败不写为不可达。
- 完整任务校验拒绝空段、缺阶段/事件、错序、数组维度错误、NaN/Inf、不连续、目标身份不符、声明轨迹与执行元数据不符、无附着带载、无支撑释放撤离，以及当前模型/工具/场景/起点不符。
- 保留六阶段兼容，额外允许明确的 `front_separation`。新版显式声明 ATTACHED、SUPPORTED_RELEASE、RETREAT_COMPLETE 事件。
- 独立复算最终分段五次插值的解析速度、加速度和 jerk 极值；限位及停稳连接单独检查。速度同时受官方 URDF 与当前设计限制约束，不信任 metadata 中已有 timing 审计。时间压缩并重新计算哈希仍会被拒绝。
- 执行使用唯一 execution_id，拒绝并发启动；逐步保存实际 q、qd、qdd、模拟时间及同一箱体状态。停止接受与停止完成分开，停止后不推进或报成功；异常转入 FAULTED。
- 规划、验证和实际执行各自构建独立世界。实际贴合后保存 `inverse(T_world_tcp) @ T_world_box`，带载全过程使用同一附着变换；释放计算指定输送带的整箱底面支撑。其余三箱保持原位。
- 理想送出单独预检下游支撑与全部几何，并在实际释放、撤离和占用检查后才推进同一箱体。几何抓放与 OUTFED_ASSUMED 分别计数。

碰撞余量仍为 2 mm，另加每体 0.2 mm 网格误差储备；合法接触穿透容差 0.4 mm。没有 FANUC 手腕/整工具/邻箱豁免。最终轨迹按时间 ≤20 ms 及关节 L1 名义 0.0008 rad 细分进行五次插值检查；这是密集几何检查，不是连续扫掠体形式化证明，也不是硬件安全认证。

## 本轮真实运行

首先建立 `compact_20260918_01`，完成尺寸、轮廓、支撑、底座和 HOME 检查并展示真实模型布局图。四个靠近基座的 HOME 候选被顶板或机械臂/工具碰撞拒绝，随后在箱堆上方找到合法初始姿态，保留全部失败记录。

初始搜索对两上层箱顶吸到横带，以及左上层箱顶吸到纵带，各给 300 s 候选预算。三次均在 loaded_transfer 阶段超时（前两次分别 7,222 / 9,498 次 RRT 状态检查）；工具与模型未缩小，墙体未移除。第四个随机搜索候选在转运阶段主动取消，以切换到有几何依据的引导搜索。原记录状态和搜索 termination 保持不改，`TIME_LIMIT_REACHED` 明确解释为预算耗尽。

随后建立独立 `compact_20260918_02`，尺寸及资产与 01 一致。对右上层箱 `carton_r00_l01_c00`、顶吸、纵带路线 B，尝试有限笛卡尔引导；每个 IK 状态及关节连接仍完整碰撞检查。获得六阶段候选，约 124.9767 s：接近、接触、抬升、带载转运、支撑放置、空载撤离。转运 82 个路点，撤离由 RRT 找到 51 个路点。密集复核到第 9,432 个通过状态后，在撤离中发现 `part_012_lip` / 目标箱距离 2.2144706 mm，小于该状态要求的 2.4 mm，候选被拒绝且没有执行；失败关节状态和图片均保留。

最终使用独立 `compact_20260918_03`，配置和几何不变。空载撤离先反向抬离，再按释放后的场景重新检查返回通道；该候选带载转运 82 个路点、撤离 112 个路点，六段合计 137.3106517 s。全部 **17,893 个状态**通过独立完整复核，未放宽接触规则或余量。机器人、工具和箱体的采样保守 XY 包络为 X [-0.61026,0.22106]、Y [-0.33040,0.22500] m，未超出桌面设计边界；这不代表桌面承载或人员安全通过。

| 独立结果 | 实际结论 |
|---|---|
| 几何抓放 | **GEOMETRIC_PICK_PLACE_COMPLETE：1**，右上层箱 `carton_r00_l01_c00`，顶吸，路线 B |
| 实际附着 | 4.546504 s，12 环贴合后保存实际 TCP→箱体矩阵 |
| 实际支撑释放 | 63.234273 s，`conveyor_longitudinal_belt`，底面覆盖 100%，最大底面高度误差 2.78 μm |
| 实际撤离完成 | 137.310652 s，终态由实际推进取得，未照抄预期终点 |
| 理想送出 | **OUTFED_ASSUMED：1**，约 5.599999 s，整箱连续沿纵带越过厢口 |
| 最终箱中心 | [-0.780000, -0.220000, 0.2600023] m；最终 X 包络约 [-0.8900001,-0.6699999] m |
| 另外三箱 | ID 和位置全部保留，与初始快照逐一匹配 |
| 动力学/真机 | NOT_EVALUATED；无 Isaac 物理执行、无硬件连接 |

执行 ID：`f1075518e0ae4d9fbe2473b1d6622ebc`。轨迹 SHA-256：`9bbd271473bbbbe01a973ad96881b6c2f911f3fbe2c41479be4def288ff2ce47`。主任务时长 137.310652 s，连同理想送出共 142.910651 s 模拟时间。抓放验证与执行、送出预检与推进使用相互隔离/明确区分的世界状态，均保持同一目标身份。


横带和转角区始终保留；若执行 B，不宣称 A 已执行。送出只针对该目标箱，结束后不继续剩余三箱。

## 测试与复现

直接相关测试共 **55 项通过**：

- `tests/test_eco65_compact_protocol.py`：44 passed；不依赖私有 CAD，包括配置、支撑、出口、桌面边界、失败/取消、完整任务拒绝、重算哈希后的时间压缩拒绝、实际初始状态、停止/异常及硬件禁用。
- `tests/test_eco65_compact_assets.py`：5 passed；真实 12 环、完整初始几何、安装座、已验证完整轨迹、实际同箱事件、三箱保留与理想送出，并用已记录的真实边界复核最终送出身份门禁。
- `tests/test_eco65_unloading_layout.py`：5 项九箱静态回归及 1 项旧单箱 HOME/接触兼容回归通过；没有重跑九箱 IK 或旧单箱完整路径。

私有资产缺失只会跳过对应资产测试；纯配置/协议测试不随之跳过。资产测试显式使用 `ECO65_COMPACT_RUN_ID`，未硬编码历史输出路径。

实际使用的命令（PowerShell，已有本地 .venv 与私有资产）：

```powershell
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py build --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py home --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe tools/run_eco65_compact_task.py plan --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03 --candidate-index 3 --cartesian-guide
```

`plan` 在接受候选前，已经在独立世界调用完整 `ECO65PlanValidator`，不是仅求六个姿态。`validate` 子命令也可独立复核已保存计划；为避免重复完整几何扫描，无变化时使用 `plan` 产生的验证证据。已有搜索记录拒绝覆盖，重新搜索需新 run_id。规划取消通过该 run 目录的 `CANCEL` 文件，取消不会计成功。

其余实际运行命令：

```powershell
& .venv\Scripts\python.exe tools/run_eco65_compact_task.py execute --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe tools/run_eco65_compact_task.py replay --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py render --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py export --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py verify --config configs/workcells/eco65_desktop_unloading_compact_v1.yaml --run-id compact_20260918_03
& .venv\Scripts\python.exe -m pytest tests/test_eco65_compact_protocol.py -q
$env:ECO65_COMPACT_RUN_ID='compact_20260918_03'
& .venv\Scripts\python.exe -m pytest tests/test_eco65_compact_assets.py -q
& .venv\Scripts\python.exe -m pytest tests/test_eco65_unloading_layout.py -q -k 'layout_topology or nine_cartons or transfer_owner or outlet_has or functional_regions'
& .venv\Scripts\python.exe -m pytest tests/test_eco65_unloading_layout.py -q -k shared_model_old_single
```


没有全量 pytest、104/129 历史验收、整排/整堆、视觉、ROS、真机或性能扫描。本地再次检查未发现可用 `isaacsim` / `isaaclab`，未安装大型环境；交付模式为 GEOMETRIC_REPLAY_ONLY，动力学 NOT_EVALUATED，硬件始终禁用，supports_continuous_handoff=false。

## 本地证据与来源

最终本地目录：`outputs/eco65_desktop_layout/compact_20260918_03/`。

- `scene_snapshot.json`：生效配置、完整场景、工具与逐资产哈希。
- `reports/layout_audit.json`、`reports/base_footprint.json`、`reports/home_search.json`：静态支撑、底座与合法初始姿态。
- `task/plan.json`、`task/validation.json`：实际附着矩阵、分段轨迹、解析计时审计与 17,893 状态完整复核。
- `task/execution.json`、`task/outfeed.json`：唯一执行 ID、实际事件、关节/箱体状态、另外三箱及送出结论。
- [尺寸俯视图](../outputs/eco65_desktop_layout/compact_20260918_03/figures/02_dimensioned_top.png)、[侧视高度图](../outputs/eco65_desktop_layout/compact_20260918_03/figures/03_height_side.png)、[总览](../outputs/eco65_desktop_layout/compact_20260918_03/figures/04_assembly_overview.png)。
- [实际顶吸目标](../outputs/eco65_desktop_layout/compact_20260918_03/figures/06_selected_grasp.png)、[实际支撑放置](../outputs/eco65_desktop_layout/compact_20260918_03/figures/07_supported_place.png)。
- [单箱完整回放](../outputs/eco65_desktop_layout/compact_20260918_03/replay/single_box.mp4)：640×360、5 fps、1 倍模拟时间；录像降频未降低碰撞检查频率。成片实际解码为 716 帧、143.2 s，结尾包含按帧取整的短暂停留；模拟事件时间为 142.910651 s。
- `backend/scene.xml`：同场景几何导出及重读检查，不是动力学证明。
- `reports/acceptance_code_fingerprints.json` 与 `reports/delivery_evidence.json`：执行前工作区源码、依赖、配置和最终交付文件指纹。两次源码快照分别记录；最终增加的恢复异常兜底与送出身份门禁有专项协议测试，几何模型、配置和已接受轨迹未改动，也未追加运动任务。

`compact_20260918_01` 的初始/随机搜索失败与 `compact_20260918_02` 的密集撤离拒绝均保留；后者 `figures/08_rejected_retreat.png` 和 `task/retreat_failure_pose.json` 能重现 2.214 mm 余量不足。历史九箱与原单箱全部输出的哈希保护记录也保留。本地文件含私有几何，不随公开仓库提交。

来源锁保持：feasibility `9db77cb8a51e9bf1821c6e84e90a7632fbce6b26`；perception `f7668c37019c31b85cc848243231f7c99a6d171d`；online `f7f7e0934ad425ab62cd2df8201812df1ba8e505`。官方 ECO65-B 与已解析真实 STEP 的哈希在冻结快照逐文件校验；未重新网格化。STEP、派生网格、详细工具配置、含私有几何的场景/图片/录像仍由 .gitignore 排除，未上传远端。
