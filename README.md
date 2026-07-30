# 物流车辆卸货：第一层快速几何仿真

这是一个可直接交给 Codex 继续开发的最小可运行仓库。它实现的是卸货仿真体系的**第一层**：不追求纸箱变形、吸盘密封和箱墙坍塌等高保真物理，而是用轻量几何模型快速完成机械臂构型评估、可达性分析、碰撞检测、抓取候选生成和路径规划。

## 已实现

- UR5e尺度的六轴工业机械臂标准DH模型
- 正运动学与6×6几何雅可比矩阵
- 阻尼最小二乘数值逆运动学，多初值重启
- 机械臂连杆胶囊体近似
- 车厢、纸箱、输送机OBB模型
- 胶囊体—OBB碰撞检测和机械臂自碰撞检测
- 面向纸箱正面的3×3解析式吸盘抓取候选
- 预抓取位姿、接触位姿和直线接近检查
- 双向RRT-Connect关节空间规划
- 路径快捷平滑、加密采样和CSV输出
- 3D场景及末端轨迹绘图
- YAML参数化场景

## 不在第一层解决

- 纸箱柔性、破损、挤压和连锁坍塌
- 真空流量、吸盘密封、漏气和脱落
- 关节动力学、柔顺控制和力控
- RGB-D渲染、域随机化和神经网络感知
- 实际机器人标定误差

这些内容应分别放到 Isaac Sim、高保真实体测试台和实机闭环中。

## 安装

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## 运行

```bash
unloading-layer1-demo --config config/demo.yaml
```

也可以不安装包：

```bash
PYTHONPATH=src python -m unloading_sim.demo --config config/demo.yaml
```

默认输出：

```text
outputs/demo/
├── metrics.json
├── trajectory.csv
└── plan.png
```

## PyBullet交互可视化

安装可选可视化依赖：

```bash
conda run -n madpro python -m pip install -e ".[viz]"
```

加载睿尔曼 RM65-B URDF、YAML 场景、单吸盘和 demo 轨迹：

```bash
conda run -n madpro python -m unloading_sim.pybullet_rm65 \
	--config config/demo.yaml \
	--trajectory outputs/demo/trajectory.csv \
	--hold
```

加载 KUKA KR 50 R2500、YAML 场景、单吸盘和 demo 轨迹：

```bash
conda run -n madpro python src/unloading_sim/demo.py --config config/kuka_kr50.yaml

conda run -n madpro python -m unloading_sim.pybullet_kuka_kr50 \
	--hold
```

`demo.py` 是离线规划入口，不会打开窗口；它会输出检测 OBB、抓取候选、IK、RRT 和放置规划进度。PyBullet 入口负责可视化回放：目标箱在抓取后会跟随末端吸盘运动，并最终落到传送带固定放置点。

KUKA 满载场景使用 `config/kuka_kr50.yaml`：车厢横截面由 4 列 × 3 层箱子填满，沿 +X 前后码两排；传送带靠机械臂底座的一端与底座 x 坐标平齐，目标箱统一放到传送带前端固定位置。

在线从上往下卸货，并连续回放全过程：

```bash
conda run -n madpro python src/unloading_sim/online_unload.py \
	--config config/kuka_kr50.yaml \
	--max-picks 2

conda run -n madpro python src/unloading_sim/pybullet_online_kuka.py \
	--direct \
	--plan outputs/kuka_kr50/online_plan.json \
	--gif outputs/kuka_kr50/online_unload.gif \
	--snapshot outputs/kuka_kr50/online_final.png \
	--frame-stride 6
```

要看真实窗口，把第二条命令里的 `--direct --gif ... --snapshot ...` 去掉，改成：

```bash
conda run -n madpro python src/unloading_sim/pybullet_online_kuka.py \
	--plan outputs/kuka_kr50/online_plan.json \
	--hold
```

当前 `DISPLAY=:1` 的 NVIDIA GLX 有驱动/用户态库版本不一致问题，PyBullet 入口默认强制 Mesa 软件 GL：

```text
__GLX_VENDOR_LIBRARY_NAME=mesa
LIBGL_ALWAYS_SOFTWARE=1
```

如果以后修好了 NVIDIA 驱动，可以加 `--native-gl` 使用原生 GPU GL。

无窗口验证：

```bash
conda run -n madpro python -m unloading_sim.pybullet_sim --direct --loops 1 --dt 0
```

生成离屏可视化快照：

```bash
conda run -n madpro python src/unloading_sim/pybullet_sim.py \
	--direct \
	--loops 1 \
	--dt 0 \
	--config config/demo.yaml \
	--trajectory outputs/demo/trajectory.csv \
	--snapshot outputs/demo/pybullet_snapshot.png
```

KUKA KR 50 R2500 快照：

```bash
conda run -n madpro python src/unloading_sim/pybullet_kuka_kr50.py \
	--direct \
	--loops 1 \
	--dt 0 \
	--snapshot outputs/kuka_kr50/perception_pick_place.png
```

如果当前桌面 OpenGL 不能创建 PyBullet GUI 窗口，可以用 Xvfb 验证 GUI 后端：

```bash
xvfb-run -s "-screen 0 1280x800x24" \
	conda run -n madpro python src/unloading_sim/pybullet_sim.py \
	--config config/demo.yaml \
	--trajectory outputs/demo/trajectory.csv \
	--loops 1 \
	--dt 0
```

默认机械臂 URDF 来自：

```text
/home/zy0004-lr/下载/code/manipulation-main/ros_ws/src/models/RM65/urdf/RM65-B/urdf/RM65-B.urdf
```

KUKA KR 50 R2500 描述来自公开仓库 `JRL-CARI-CNR-UNIBS/kr_50_r2500`，已下载到：

```text
third_party/kr_50_r2500/
```

PyBullet 使用的静态 URDF 由该仓库的 xacro 渲染得到：

```text
assets/robots/kuka_kr50_r2500/kr_50_r2500.urdf
```

末端默认挂接简化单吸盘；如需隐藏工具可加 `--tool none`。

## 坐标系

- 世界坐标 `+X`：从车门向车厢内部
- `+Y`：车厢左侧
- `+Z`：向上
- 车厢在 `x=0` 处开口，沿 `+X` 延伸
- 吸盘工具坐标 `+Z` 指向被吸取表面

## 代码结构

```text
src/unloading_sim/
├── geometry.py       # OBB、胶囊体、SO(3)工具
├── robot.py          # 六轴DH模型、FK、雅可比、碰撞体
├── scene.py          # 车厢和纸箱场景
├── ik.py             # DLS逆运动学
├── planner.py        # RRT-Connect
├── grasp.py          # 解析式吸盘候选和抓取规划
├── visualization.py # 3D绘图
├── pybullet_sim.py  # PyBullet公共可视化后端
├── pybullet_rm65.py # RM65独立PyBullet入口
├── pybullet_kuka_kr50.py # KUKA KR50独立PyBullet入口
└── demo.py           # 命令行入口
```

## 替换成真实工业机械臂

当前模型只是UR5e尺度的算法原型。选定发那科、KUKA、安川或国产机械臂后，应优先替换：

1. `DHRobot6.ur5e_like()`中的DH参数和关节限位；
2. 各连杆胶囊半径；
3. 基座安装位姿；
4. 末端吸盘长度；
5. 真实碰撞模型与厂商URDF的交叉验证。

建议下一步新增 `URDFRobotAdapter`，用 Pinocchio 负责运动学、hpp-fcl负责网格碰撞，同时保留当前胶囊体后端用于大批量快速筛选。

## Codex建议任务

仓库根目录包含 `AGENTS.md`。把仓库放入 Codex 后，可依次要求：

```text
1. 增加批量可达性扫描，输出车厢内三维热力图。
2. 增加多个机械臂安装位姿对比和最优基座搜索。
3. 增加纸箱支撑关系图与确定性卸货顺序评分。
4. 增加Pinocchio/hpp-fcl可选后端，并保持现有API不变。
5. 增加ROS 2消息导出和Isaac Sim场景参数导出。
```

## 验证

```bash
pytest -q
```

默认示例在本仓库生成时已通过运行测试，规划结果会写入 `outputs/demo`。
