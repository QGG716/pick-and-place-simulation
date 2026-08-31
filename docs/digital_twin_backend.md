# 可信数字样机与仿真后端决策

## 决策

不替换几何核心，采用“双后端验证”：

1. PyBullet 作为每次提交都能运行的快速后端，负责实时 GUI、关节电机跟踪、基础接触与 RGB-D/分割冒烟测试。
2. Isaac Sim 作为高保真验收后端，负责相机成像、材质/光照随机化、关节力、接触和纸箱动力学验证。
3. 两个后端消费相同的 SI 制配置、URDF、定时轨迹和相机标定，结果统一输出审计 JSON。

核心包不引入 Isaac Sim、ROS 2 或 GPU 依赖。Isaac Sim 代码必须放在适配器或独立扩展中。

当前部署条件为独立 RTX 4090 服务器。FANUC 在线规划继续在 CPU/工控机运行；认证后的定时轨迹异步发送给服务器，Isaac Sim 通过 WebRTC 提供实时画面并返回 RGB-D、分割、关节状态、接触力和箱体位姿审计。网络或渲染延迟不得进入机器人安全控制闭环。

## 为什么选择 Isaac Sim 作为第二后端

截至 2026-08，Isaac Sim 官方文档提供 URDF 导入、PhysX articulation、RGB/深度/分割、关节状态/力和接触传感器，以及 Windows/Linux ROS 2 工作流。它最符合本项目后续需要验证的相机误差、吸取接触、纸箱滑移和合成数据。代价是 RTX GPU、安装体积、启动时间和版本 API 迁移成本较高，因此不适合取代轻量核心。

Gazebo Harmonic 的相机和 ROS 2 集成成熟，更适合以 ROS 2/Linux 为中心的系统联调；MuJoCo 的刚体动力学和实时交互很强，但物流视觉传感器与合成数据工具链不如 Isaac Sim 完整。若目标机器没有合适的 NVIDIA GPU，优先落地 Gazebo 适配器，而不是削弱核心模型。

官方依据：

- [Isaac Sim Sensors](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/index.html)
- [Isaac Sim ROS 2 Cameras](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_camera.html)
- [Isaac Sim Joint State Sensor](https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_joint_state.html)
- [Gazebo CameraSensor](https://gazebosim.org/api/sensors/8/classgz_1_1sensors_1_1CameraSensor.html)
- [MuJoCo Overview](https://mujoco.readthedocs.io/en/stable/overview.html)

## 当前可信度分层

| 层级 | 当前能力 | 验收证据 |
|---|---|---|
| 几何 | URDF FK/IK、胶囊-OBB、RRT、搬运箱体检查 | 单元/回归测试 |
| 轨迹 | 碰撞复核的 C2 五次 Bezier 拐角融合，速度/加速度/jerk 时间审计 | `corner_blending`、`execution_summary` |
| 控制 | PyBullet 定时位置控制与指令/实际关节误差 | 动力学回放 JSON |
| 标定 | 控制器 CSV 导入、实测百分位限制与峰值 | 标定 JSON，保留来源 |
| 场景逻辑 | 支撑关系、四向遮挡、确定性卸箱顺序 | 支撑图审计与测试 |
| 高保真 | Isaac Sim 已完成 URDF 碰撞网格、动态载荷、RGB-D、关节力、接触与轨迹回放；Surface Gripper 适配待服务器复验 | 回放结果、证据清单、后端差异矩阵 |

## 近期验收门槛

- 平滑后路径必须逐边重新碰撞检查；不安全的角点不得被融合。
- 胶囊模型相对 URDF 网格模型的 `reference_only` 漏检率必须为零；误报率单独统计。
- 动力学回放记录 RMS/峰值关节跟踪误差、接触、箱体最终位姿和实时率。
- 相机输出至少包含 RGB、米制深度、实例分割、内参、外参与仿真时间戳。
- 卸箱顺序不得先移走仍支撑其他箱体的纸箱。

## 后端迁移顺序

1. 用 PyBullet 动力学回放标定控制增益、力矩上限和轨迹跟踪误差。
2. 将 FANUC、AMR、输送机、拖车和纸箱导出为 USD，并在 Isaac Sim 中复现同一世界坐标系。
3. 接入固定 RGB-D 相机，先做几何真值对齐，再加镜头噪声、曝光和材质随机化。
4. 对同一组轨迹并行运行胶囊、PyBullet 网格和 Isaac/PhysX，建立碰撞与最终位姿差异基线。
5. 最后再接 ROS 2/真实控制器，避免通信层掩盖模型问题。
