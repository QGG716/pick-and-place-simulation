# Trailer Unloading Geometric Simulator v0.1

面向厢式货车卸垛的轻量级六轴工业机器人几何仿真与运动规划原型。v0.1 以 FANUC M-20iD/35 为主要验证机型，覆盖纸箱抓取点生成、逆运动学、碰撞检测、在线卸垛顺序、双输送带放置、轨迹缓存和 PyBullet 回放。

核心规划器只依赖 CPU，不依赖 ROS 2、Isaac Sim 或 GPU，适合快速算法迭代、回归测试和生产方案前期验证。

![FANUC 最上层卸垛演示](outputs/fanuc_m20id35/fanuc_m20id35_top_layer_v11.gif)

## v0.1 功能

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
- PyBullet 无窗口 GIF、PNG 回放以及交互式查看。

## v0.1 验证结果

最上层 8 箱回归使用同一个 AMR 停靠位 `[0.80, -0.30, 0.0]`：

- 6 箱进入纵向输送带，2 箱进入横向输送带。
- 横向带方案分别使用顶面抓取和侧面抓取。
- `carton_front_l2_c2` 在 `carton_front_l2_c3` 之前被移除。
- 8/8 段通过逐轨迹点机器人、箱体和车厢碰撞复核。
- 优化轨迹的碰撞复核总耗时约 6.8 秒，平均约 0.85 秒/箱。
- 50 Hz 离散轨迹的模拟执行估算为 1.96–3.58 秒/箱，平均约 2.59 秒/箱。

这些数据说明缓存验证可以放入 6 秒/箱的计算预算，但不等同于真机已经达到 600 箱/小时。真机还必须计入传感器、真空建立与确认、关节速度/加速度/加加速度、PLC 握手、输送带节拍和故障恢复。

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
├── trajectory_cache.py     # 缓存优化、碰撞复核和延迟统计
├── pybullet_sim.py         # 公共 PyBullet 可视化工具
└── pybullet_online_kuka.py # FANUC/KUKA 在线方案回放
```

`geometry.py`、`robot.py`、`scene.py`、`ik.py`、`planner.py` 和 `grasp.py` 保持可独立测试。所有随机规划入口均接受确定性种子。

## 测试

```bash
pytest -q
```

v0.1 当前测试状态：40 项通过；5 项旧 KUKA 场景布局断言仍需与现配置同步。新增的腕部滚转、最近等价关节分支和递归配置继承测试均通过。

## 当前边界

v0.1 是几何与规划原型，不模拟：

- 纸箱柔性、破损、挤压或连锁坍塌；
- 真空流量、吸盘密封、漏气和脱落；
- 完整刚体动力学、柔顺控制和力控；
- 视觉噪声、遮挡下的检测误差和标定漂移；
- 工业控制器的速度前瞻、安全 PLC 和真机认证。

进入真机前，应使用厂商精确碰撞网格、关节动力学约束和实测吸盘模型重新验证全部轨迹。

## 机器人资源

- FANUC M-20iD/35 资源来源和许可证见 `assets/robots/fanuc_m20id35/SOURCE.md` 与 `LICENSE.txt`。
- KUKA KR 50 R2500 资源位于 `third_party/kr_50_r2500/`。

## 版本

当前基线：`v0.1.0`，发布分支：`release/v0.1`。
