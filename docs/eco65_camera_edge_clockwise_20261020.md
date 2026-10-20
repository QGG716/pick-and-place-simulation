# ECO65 相机改位、板边对齐和首次顺时针：2026-10-20

基线 `ea7fbc4212152f57b1b77356c67e6d7062dbb5bb`，工作分支
`feat/v0.6-eco65-moveit2-native-single-carton`。本轮没有修改参考分支、官方机器人、
工具安装角、TCP、箱体尺寸或箱间距。

**结果：三项修改已实现，定向检查通过，已运行新的原生任务；完整循环未完成。**
本次阻塞在首箱 `carton_r00_l03_c01` 的接触态，尚未附着。
原生返回具体未启用吸盘与邻箱净空不足；未扩大碰撞豁免以制造成功。
这与上一轮首箱成功的旧斜吸/旧方向策略结果是两次不同运行。

主要入口及修改：

| 文件 | 作用 |
|---|---|
| `configs/workcells/eco65_camera_axis_yaw180_v1.yaml`、`src/unloading_sim/eco65/camera_installation.py` | 真实杆轴旋转、派生连接件、统一名义外参 |
| `tools/build_eco65_coupled_carton.py`、`tools/eco65_camera_mount.py`、`tools/eco65_mast_attachment.py` | 场景、原生附件和安装诊断共用新定义 |
| `src/unloading_sim/eco65/native_cups.py`、`native_contact.py`、`native_model.py`、`ros2/m710_moveit_backend/src/seal_contours.h` | 正交候选及实际 FK 边对齐验收 |
| `src/unloading_sim/eco65/coupled_state.py`、`coupled_task.py`、`native_task.py`、`ros2/m710_moveit_backend/src/eco65_cycle.h` | 首次顺时针、有限初态/IK 分支和完整流程回退 |
| `tools/export_eco65_coupled_replay.py`、`tools/search_replay/eco65_coupled_viewer.html` | 同源安装、边轴、原始 J1 和失败请求显示 |
| `tools/export_eco65_warehouse_workstation.py`、`tools/summarize_eco65_coupled_run.py` | USD 及实际运行指标导出 |

## 相机安装

`camera_installation.py` 从原始 `mast_part_019` 下端圆面读取杆轴，
并核对实际原始视觉网格的下端环（其网格只有端环，没有杆中段顶点）。
轴经过 mast 局部 `[0.1045, -0.00405, 0.008] m`，方向 `[0,0,1]`，
杆半径 0.0125 m。没有使用整个偏心 U 形支架的 AABB 中心、mast 原点或 J1 轴。

在已经下俯的 CAD/optical 安装上左乘同一绕轴 180° 刚体变换：

| 项目 | 旧值 | 新值 |
|---|---:|---:|
| optical 水平朝向相对 link_1 局部 −X | −90° | +90° |
| 实际下俯角 | 30° | 30° |
| optical 原点 mast X | 0.140981695 m | 0.068018305 m |
| optical 原点 mast Y | 0.028812291 m | −0.036912291 m |
| optical 原点 mast Z | 0.467642795 m | 0.467642795 m |

CAD 与 optical 相对刚性变换保持；下俯轴和枢轴元数据同步变换。
原立杆、偏心上部 U 形支座、下部座保持不动。两块项目派生 riser 的 X 宽由 2 mm
变为 19 mm，连接原支座壁和反转后的轴承位置；轴承和 saddle 随相机转动。
供应商 CAD 未改写。全部附件 ID 及 2954 个机器人/桅杆豁免对保持不变，
但改变的支撑块几何及安装源身份重新生成。

`eco65_camera_mount.mount_check()` 从实际变换独立计算角度，与配置目标比较。
`eco65_mast_attachment.evaluate()`、工作站 USD、原生附件和两种查看器读取同一
`permanent_camera` 定义，旧 mast 光学字段仅保留为一致的兼容别名。

当前外参是新安装推导的名义外参，`prior_calibration_valid=false`。
`camera_world_transform()` / `nominal_camera_point()` 绑定当前安装并拒绝旧矩阵，
不会继承旧标定通过状态。本次目标仍来自冻结场景，没有进行拍摄或视觉识别闭环。
第一次运行的外参矩阵字段为 `transform`；最终导出接口同时支持这一已捕获的
精确场景定义及带安装指纹的 `T_parent_child`，不会回退到旧安装外参。

## 抓取及 J1 合同

吸具固定板边为 CAD X/Z，对应既有 TCP X/Y。候选仅使用箱面坐标系下的
0°、90°、180°、270°，仍允许面内平移和实际吸盘组合。
Python `check_contours()` 与原生 `checkSealContours()` 都由实际法兰 FK、
固定安装和当前目标箱姿态计算两条板边，接受直接或互换匹配，绝对点积角容差 1°。
法向、间隙、穿透及完整密封检查保留；即使轮廓能密封，45° 仍被拒绝。
新的连续入口要求边对齐合同，独立于是否执行相机改位。

方向策略版本为
`RAW_J1_FIRST_CW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V2`：
FIRST_APPROACH = −1、OUTBOUND = −1、RETURN = +1。Python/C++ 同步，
保留区间累计反向行程 1e-6 rad，使用官方 J1 `[-3.106, 3.106] rad`，
不取模，不对关节解加减 2π。旧策略记录不按新策略重新解释。

新建仿真可从基于箱面法向和箱高的有限待抓姿态求本次原生 IK，再尝试旧 HOME。
这些是明确记录的仿真初态候选，不是未显示的初始化运动。继承起点模式要求
明确提供 q，拒绝在其非法时静默换成另一个初态。合法目标优先较小原始 J1 行程，
再比较其他关节代价；接触 IK 保留有限分支回退。

连续任务增加返回高度/抓取候选回退，返回失败可回到先前抓取分支；
共用任务绝对截止时间，单连接与单次原生求解仍有有限预算。
本次实际运行尚未进入带载及返回，因此这些后段恢复分支只能列为代码已实现，
不能列为完整原生流程已验证。

## 实际运行与失败证据

原生运行：`camera_cycle_20261020_run01`。
实际运行源码部署：`camera_cycle_20261020_build03/source_deployment.json`；
worker 构建：`camera_cycle_20261020_build01/build.log`。
场景：`camera_cycle_20261020_scene03`，快照：
`b338cea6f2b2087e0f65bd364e2cd3bae93d31216840e5c17adc4068383fbd24`。

| 记录 | 实际结果 |
|---|---|
| 总规划墙钟（包括 Python、原生和 IPC） | 190.423727 s |
| 原生初始化往返 | 2.919357 s |
| IK 往返总计 | 56.080687 s |
| 状态检查往返总计 | 29.090667 s |
| FK 往返总计 | 0.002433 s |
| 原生调用 | 511 |
| IK 成功/失败 | 51 / 405 |
| 状态检查成功/拒绝 | 10 / 40 |
| 接触候选最大边角误差 | 3.9198835e-7 rad，约 0.00002246° |
| 接受运动/机构时长 | 0 / 0 s，没有运动占位段 |
| world / attached / completed 箱数 | 12 / 0 / 0 |
| 首箱附着、释放、下一箱附着 | 均未达到 |

几何清单共 628 个候选，本次按 `max_grasps=24` 检查有限候选与有限种子。
39 个接触候选被保留的吸盘/邻箱净空检查拒绝；另外一个状态拒绝是待抓候选自碰撞。
实例包括 `part_006_lip ↔ carton_r00_l03_c02`、
`part_002_lip ↔ carton_r00_l03_c00`，要求 2.4 mm，实际间隙接近零。
实际返回状态为 `STATE_REJECTED` / `CLEARANCE_INSUFFICIENT`，
末次运行报告错误为 `FIRST_CARTON_BOUNDED_CANDIDATES_EXHAUSTED`。

测得实际杯轮廓直径约 41.114 mm，杯中心节距 45 mm，三杯一行跨度约 131.114 mm；
当前中列箱宽 100 mm，两侧各留 10 mm 缝，邻箱之间的空隙总宽为 120 mm。
这解释了正交姿态下未启用杯的邻箱接触；几何排序提示不能替代全局不可行性证明。
没有把“负载箱与邻箱脱垛接触许可”扩展到吸盘，也没有改箱间距或改取其他首箱。

运行检查的旧 HOME 提案、以及最终保存的尝试初态 q 为：

```text
[-0.5031953406584946, -0.30537401468411235, -1.3414169996571763,
  0.07599468559211806, 1.5707926322178625, 4.17131980283628]
```

有限待抓初态 IK、成功/拒绝状态及其真实 q 均保存在请求包中。
没有被接受的首抓 q，也没有首段实际轨迹，因此首段 J1 实际转角和原生路径方向结果为
**未产生**，不能把累计值零当作首段成功。完整任务 `complete=false`。

## 定向检查和查看器

- 32 项 Python 检查通过：相机轴线、刚性关系、诊断、外参失效、
  top/front 与旋转箱的实际板边检查、J1 累计反向量等。
- 25 项原生协议/几何检查通过：FIRST_APPROACH 顺时针接受、逆时针拒绝、
  旧零方向拒绝、原始角度、板边直接/互换/45° 等。
- Edge 实际打开离线页面，2813 项相关页面检查通过，
  Python/JS FK 最大矩阵误差 `1.1102230246251565e-16`；已查看截图。
- 查看器含实际杆轴、optical +Z、link_1 朝向、工具/箱面边轴、实际边角误差、
  原始 J1 限位与区间指标、请求自身的世界和失败对象对。具有俯视、三高度和图层控制。
- 本次在接触 IK/状态检查就失败，没有调用路径搜索；RRT 树明确为不适用，不伪造树。
- 相同不可变大对象按引用存储，实际记录无损；页面约 45 MB，资源全部内嵌。
- 同源工作站 USD 已导出，包含新相机和统一联动实体。

最终源码还补充了名义外参接口字段、继承起点禁止重选、完整循环有限回退和
边对齐合同必填检查。这些没有被追记为 run01 已执行的后段证据；
实际运行与最终源码的来源分别保留。

## 文件与复现命令

本地交付目录：`outputs/eco65_native/camera_cycle_20261020_result/`。
`cycle.html`、`cycle.zip` 为实际失败诊断查看器及原始记录包；
`browser01/checked.png` 为实际浏览器截图；
`records/native/summary.json`、`records/native/run_report.json` 为运行摘要。
`records/cycle.json` 是 `complete=false` 的实际部分计划。
`evidence.zip` 保留相机安装报告、当次名义外参、同源 USD、
原生构建/测试日志、运行源码部署清单和当次源码包；它也包含在 `cycle.zip` 中。
`python_tests.txt` 与 `browser01/checks.json` 分别记录定向 Python 和页面检查。
生成文件和私有 CAD 保持忽略，不加入 Git。

实际远端运行命令（沿用既有环境，没有升级）：

```bash
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
S=outputs/eco65_native/camera_cycle_20261020_scene03
RUN=outputs/eco65_native/camera_cycle_20261020_run01
WORKER='chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /usr/bin/env M710_MOVEIT_INSTALL_PREFIX=/work-eco65-20261015/install-coupled-20261020 /bin/bash /work-eco65-20261015/ros2/m710_moveit_backend/worker.sh'
chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c \
  'source /opt/ros/humble/setup.bash; cmake --build /work-eco65-20261015/build-coupled-20261020 --target m710_moveit_worker eco65_cycle_test -j2 && cmake --install /work-eco65-20261015/build-coupled-20261020 && /work-eco65-20261015/build-coupled-20261020/eco65_cycle_test'
$P tools/build_eco65_coupled_carton.py \
  --source outputs/eco65_native/small_20261019_run61/build/snapshot.json \
  --home outputs/eco65_native/small_20261019_run61/build/home_candidate.json \
  --camera-config configs/workcells/eco65_camera_axis_yaw180_v1.yaml \
  --output "$S" --native-asset-root /work-eco65-20261015 --usd
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs/work-eco65-20261015
mkdir -p "$R/$S"
cp -a "$S/model" "$S/camera_attachments" "$R/$S/"
$P tools/run_eco65_coupled_cycle.py --snapshot "$S/snapshot.json" \
  --home-candidate "$S/home_candidate.json" --worker-command "$WORKER" \
  --output "$RUN" --task-seconds 900 --connection-seconds 100 \
  --solver-seconds 8 --rpc-seconds 100 --ik-seeds 8 --max-grasps 24 --search-trace
$P tools/summarize_eco65_coupled_run.py --run "$RUN" --snapshot "$S/snapshot.json"
$P tools/export_eco65_coupled_replay.py --snapshot "$S/snapshot.json" \
  --plan "$RUN/partial_plan.json" --run "$RUN" \
  --output outputs/eco65_native/camera_cycle_20261020_replay02/cycle.html
$P tools/export_eco65_warehouse_workstation.py --snapshot "$S/snapshot.json" \
  --home "$S/home_candidate.json" --output outputs/eco65_native/camera_cycle_20261020_usd01
```

重跑须使用新的输出目录；完整任务成功时使用 `plan.json`，不能把 partial 改名冒充成功。
编译沿用 `/work-eco65-20261015/build-coupled-20261020` 的 CMake 工程，
目标 `m710_moveit_worker eco65_cycle_test`，再安装到既有 install 前缀。

本地定向 Python 检查：

```powershell
$env:PYTHONPATH='src;tools'
.venv/Scripts/python.exe -m pytest -q tests/test_eco65_camera_and_edge_alignment.py tests/test_eco65_coupled_state.py tests/test_eco65_independent_cups.py
```

`planning_only=True`、`physical_execution=NOT_RUN`、`execution_ready=false`。
未做动力学、Isaac、力矩、jerk 认证、硬件执行或额外全轨迹精细审计。
