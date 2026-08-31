# Trailer Unloading Geometric Simulator v0.2

面向厢式货车卸垛的六轴工业机器人数字样机。v0.2 以 FANUC M-20iD/35 为主要验证机型，在几何规划基础上增加 Isaac Sim 动力学回放、万泰三分区吸具、动态 L 形输送带、RGB-D、关节跟踪/力与 fail-closed 验收证据。当前定位仍是“可审计数字样机”，不是生产级验收通过版。

核心规划器只依赖 CPU，不依赖 ROS 2、Isaac Sim 或 GPU，适合快速算法迭代、回归测试和生产方案前期验证。

![FANUC 最上层卸垛演示](outputs/fanuc_m20id35/fanuc_m20id35_top_layer_v11.gif)

## v0.2 功能

- FANUC M-20iD/35 与 KUKA KR 50 R2500 URDF 运动学适配。
- 世界坐标约定：`+X` 向车厢内部、`+Y` 向左、`+Z` 向上。
- 车厢、纸箱和输送带使用 OBB；机器人连杆使用胶囊体近似。
- 机器人—环境、自碰撞、所抓箱体—机器人及箱体—车厢/其他箱体检测。
- 正面、侧面和顶面吸取候选；吸盘正面始终与被吸箱面法向对齐。
- 圆形吸盘绕法向的腕部滚转候选，用 J4–J6 冗余降低 IK 和路径搜索难度。
- 预抓取、接触、脱垛、净空调整、转身和放置阶段约束。
- 双向 RRT-Connect、直接边优先、碰撞复核捷径优化和轨迹加密。
- 纵向与横向输送带联合规划，并按当前输送带负载进行滚动分流。
- 纸箱在输送带上方释放并模拟自由落体。
- 固定底座优先：同一停靠位可用时不会频繁移动 AMR。
- 确定性预抓取可达性/碰撞热图与 IK 热启动缓存。
- 轨迹缓存重新验证和局部优化，支持与机器人执行流水线并行。
- 显式关节速度、加速度和 jerk 限制的确定性时间参数化与周期审计。
- PyBullet 无窗口 GIF、PNG 回放以及交互式查看。

## v0.2 验证结果

最上层 8 箱回归使用同一个 AMR 停靠位 `[0.80, -0.30, 0.0]`：

- 6 箱进入纵向输送带，2 箱进入横向输送带。
- 横向带方案分别使用顶面抓取和侧面抓取。
- `carton_front_l2_c2` 在 `carton_front_l2_c3` 之前被移除。
- 8/8 段通过逐轨迹点机器人、箱体和车厢碰撞复核。
- 优化轨迹的碰撞复核总耗时约 6.8 秒，平均约 0.85 秒/箱。
- 旧版 50 Hz 固定路点周期估算约 2.59 秒/箱，但它不构成可执行性证据。
- 新的离散路点约束审计使用 URDF 速度上限、显式加速度/jerk 假设和 35% 降额；同一批轨迹约为 67.4 秒/箱、53.4 箱/小时，暴露出未经控制器级平滑的密集折线路径无法达到生产节拍。

这些数据说明缓存验证可以放入 6 秒/箱的计算预算，但不等同于真机已经达到 600 箱/小时。当前时间模型已计入显式运动限制和传感/真空/释放固定延迟；PLC 握手、输送带节拍、控制器插补、动力学和故障恢复仍需通过 HIL 与实测闭环补齐。

`execution` 配置中的 URDF 速度来源和工程假设会写入计划清单。缺失、维度错误或非正的限制会直接报错。在线规划保留兼容的 `online_trajectory.csv`，并额外生成带每箱相对时间戳的 `online_timed_trajectory.csv`；每段的峰值和总周期记录在 `execution_summary` 中。

完整冷启动回归规划仍需要数分钟，仅用于离线生成和验证动作原语；生产在线流程应使用：

```text
场景签名/热图查表
  -> 当前姿态优先的最近 IK 分支
  -> 整条缓存轨迹批量碰撞复核
  -> 仅对失效局部进行限时 RRT 修补
```

下一箱规划应与当前箱执行并行。

## 安装

需要 Python 3.10 或更高版本。

```bash
python -m venv .venv

# Linux/macOS
source .venv/bin/activate

# Windows PowerShell
.venv\Scripts\Activate.ps1

python -m pip install -e ".[dev,viz]"
```

核心算法不需要 PyBullet：

```bash
python -m pip install -e ".[dev]"
```

## 快速开始

### 1. 生成 FANUC 最上层卸垛方案

在线限时配置：

```bash
python -m unloading_sim.online_unload \
  --config config/fanuc_m20id35.yaml \
  --max-picks 8 \
  --top-layer-only \
  --output-dir outputs/fanuc_m20id35/top_layer
```

离线回归和缓存生成配置允许更长搜索时间：

```bash
python -m unloading_sim.online_unload \
  --config config/fanuc_m20id35_validation.yaml \
  --max-picks 8 \
  --top-layer-only \
  --output-dir outputs/fanuc_m20id35/top_layer_validation
```

### 2. 优化并复核轨迹缓存

```bash
python -m unloading_sim.trajectory_cache \
  --plan outputs/fanuc_m20id35/top_layer_validation/online_plan.json \
  --output outputs/fanuc_m20id35/top_layer_validation/online_plan_optimized.json \
  --attempts 800

python -m unloading_sim.trajectory_cache \
  --plan outputs/fanuc_m20id35/top_layer_validation/online_plan_optimized.json \
  --validate-only
```

### 3. 生成 GIF

```bash
python -m unloading_sim.pybullet_online_kuka \
  --plan outputs/fanuc_m20id35/top_layer_validation/online_plan_optimized.json \
  --direct \
  --gif outputs/fanuc_m20id35/top_layer.gif \
  --snapshot outputs/fanuc_m20id35/top_layer_final.png \
  --frame-stride 8 \
  --width 800 \
  --height 500 \
  --camera-eye -3.0 -0.55 3.5 \
  --camera-target 1.15 0.05 1.0 \
  --show-trajectory
```

GIF 使用 PyBullet CPU TinyRenderer。分段串行渲染不会减少总计算量；只有并行渲染或降低分辨率、帧数时才会明显提速。核心规划和碰撞检测不需要 GPU。

### 4. 生成预抓取点热图

```bash
python -m unloading_sim.reachability \
  --config config/fanuc_m20id35.yaml \
  --output-prefix outputs/fanuc_m20id35/pregrasp_heatmap \
  --y-samples 21 \
  --z-samples 17
```

输出包括：

```text
pregrasp_heatmap.npz   # 网格、状态、IK 解和误差
pregrasp_heatmap.png   # 可视化热图
pregrasp_heatmap.json  # 摘要统计
```

## 配置

- `config/common_unloading.yaml`：公共车厢、纸箱、输送带和规划参数。
- `config/fanuc_m20id35.yaml`：FANUC 在线限时配置。
- `config/fanuc_m20id35_validation.yaml`：离线回归/缓存生成预算。
- `config/kuka_kr50.yaml`：KUKA KR 50 配置。

配置支持递归 `extends`，并在出现循环继承时明确报错。

## 代码结构

```text
src/unloading_sim/
├── geometry.py             # OBB、胶囊体、SO(3) 工具
├── robot.py                # 六轴机器人、URDF FK/Jacobian/碰撞体
├── scene.py                # 场景与递归 YAML 配置
├── ik.py                   # 阻尼最小二乘多起点 IK
├── planner.py              # RRT-Connect、捷径和平滑
├── grasp.py                # 抓取/放置候选和完整抓放规划
├── perception.py           # 确定性 OBB 感知接口
├── online_unload.py        # 连续卸垛、排序、AMR/输送带调度
├── reachability.py         # 预抓取可达性与碰撞热图
├── timing.py               # 关节时间参数化、运动限制审计和周期估算
├── trajectory.py           # 碰撞复核的控制器级拐角融合
├── calibration.py          # 真实控制器日志导入与执行限制标定
├── collision_backend.py    # 胶囊/URDF 网格碰撞后端及差异回归
├── support.py              # 箱体支撑、遮挡关系图和移除顺序
├── dynamics.py             # 动力学后端、定时指令和 RGB-D 数据契约
├── trajectory_cache.py     # 缓存优化、碰撞复核和延迟统计
├── pybullet_sim.py         # 公共 PyBullet 可视化工具
└── pybullet_online_kuka.py # FANUC/KUKA 在线方案回放
```

`geometry.py`、`robot.py`、`scene.py`、`ik.py`、`planner.py` 和 `grasp.py` 保持可独立测试。所有随机规划入口均接受确定性种子。

## FANUC 在线实时规划

冷启动 IK/RRT 用于离线生成动作库，不能进入生产在线主链路。先把碰撞通过的 FANUC 方案编译成带场景状态、关节连续性和轨迹摘要的运行时证书：

```bash
python -m unloading_sim.realtime_planner \
  --plan outputs/fanuc_m20id35/top_layer_v11/online_plan.json \
  --output outputs/fanuc_m20id35/fanuc_runtime_library.json
```

编译是离线慢任务，会重新验证机器人与搬运箱体碰撞。在线运行只进行支撑图更新、证书匹配、关节起点检查和完整性校验：

```bash
python -m unloading_sim.online_unload \
  --config config/fanuc_m20id35.yaml \
  --max-picks 8 --top-layer-only \
  --runtime-library outputs/fanuc_m20id35/fanuc_runtime_library.json \
  --output-dir outputs/fanuc_m20id35/realtime_run
```

也可独立运行延迟基准：

```bash
python -m unloading_sim.realtime_planner \
  --library outputs/fanuc_m20id35/fanuc_runtime_library.json \
  --benchmark --repetitions 100
```

当前 FANUC 认证库回放记录的在线规划均值为 2.65 ms、P95 为 3.24 ms，满足 50 ms 硬截止。基准命令默认重复 100 轮并报告样本数、P95 和最大值；正式证据应同时保存运行库哈希、机器环境和原始 JSON。当前证书只接受与离线名义场景一致的箱体位姿；真实相机位姿有偏差时会立即 cache miss。后续必须增加带碰撞净空证明的扰动包络和末端视觉伺服，不能直接放宽容差。

## 实时动力学与标定

几何回放仍使用普通 `trajectory.csv`。需要验证控制器跟踪时，传入带时间戳的轨迹并启用电机驱动动力学：

```bash
python -m unloading_sim.pybullet_sim \
  --config config/kuka_kr50.yaml \
  --robot-urdf assets/robots/kuka_kr50_r2500/kr_50_r2500.urdf \
  --package-root third_party/kr_50_r2500/kr_50_r2500_description \
  --end-effector-link flange \
  --trajectory outputs/kuka_kr50/trajectory_timed.csv \
  --dynamic --hold
```

命令会实时显示 GUI，并输出各关节 RMS/峰值跟踪误差。`capture_rgbd_frame` 提供带米制深度、实例分割、内参和仿真时间戳的相机帧。真实控制器日志可用于替换工程假设：

```bash
python -m unloading_sim.calibration \
  --log controller_joint_log.csv \
  --event-log controller_process_events.csv \
  --percentile 99.5 \
  --headroom 1.15 \
  --output outputs/calibration/controller_limits.json
```

仿真器取舍、Isaac Sim 迁移路线和可信度验收门槛见 [`docs/digital_twin_backend.md`](docs/digital_twin_backend.md)。

## 测试

```bash
pytest -q
```

当前发布回归：129 项通过，1 项失败。失败项是 L 形横向输送带距机器人基座/J2 保护包络仅 275 mm，低于测试要求的 300 mm；V0.2 保留该失败作为已知布局阻断项，未伪装成全绿回归。其余测试覆盖矩形吸盘覆盖约束、fail-closed 仿真验收、碰撞安全拐角融合、连续路径复核、控制器与工艺事件日志标定、支撑/遮挡图、碰撞后端差异矩阵、定时动力学指令、相机内参和 FANUC 认证轨迹实时检索。

## 当前边界

v0.2 是可审计数字样机，仍未完整模拟或标定：

- 纸箱柔性、破损、挤压或连锁坍塌；
- 真空流量、吸盘密封、漏气和脱落；
- 完整刚体动力学、控制器轨迹插补、柔顺控制和力控；
- 视觉噪声、遮挡下的检测误差和标定漂移；
- 工业控制器的速度前瞻、安全 PLC 和真机认证。

进入真机前，应使用厂商精确碰撞网格、关节动力学约束和实测吸盘模型重新验证全部轨迹。

## 机器人资源

- FANUC M-20iD/35 资源来源和许可证见 `assets/robots/fanuc_m20id35/SOURCE.md` 与 `LICENSE.txt`。
- KUKA KR 50 R2500 资源位于 `third_party/kr_50_r2500/`。

## 版本

当前基线：`V0.2`（Python 包版本 `0.2.0`），发布分支：`release/v0.2`。完整变更、验证证据与已知问题见 [`docs/releases/V0.2.md`](docs/releases/V0.2.md)。
