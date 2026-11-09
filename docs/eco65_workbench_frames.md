# ECO65-B 坐标、关节及 TCP 映射

对比实际授权参考库 `f0353d7` 与主工程 `269ab67` 的 URDF、FK 和控制器读取代码。
可重跑 `python tools/audit_eco65_workbench_frames.py --reference /path/to/reference
--snapshot outputs/workbench/scene_snapshot.json --output outputs/workbench/frame_audit.json`。
完整数值见 [审查证据](evidence/eco65_workbench/frame_audit.json)。这是代码/模型对比，现场零位验证 NOT_RUN。

| 项目 | 主工程与参考的映射依据 |
|---|---|
| 基座 | 两模型根节点均为 base_link，零位、关节 origin 和 axis 逐项一致 |
| 关节顺序 | SDK `state['joint']` 的 J1–J6 → joint_1–joint_6，索引 0–5 |
| 单位/方向 | SDK 度转弧度；6 个正号，6 个零偏移；与参考 telemetry/GUI 的转换一致 |
| link_1 | J1 后坐标；零姿态原点为 base_link 下 [0,0,0.1625] m；相机固定到此节点 |
| link_6 | 两模型均为 J6 后法兰，运行时只用主工程 URDFRobot FK |
| 外参 | parent=link_1，child=camera_optical；matrix 将光学坐标点变到 link_1 |
| 光学坐标 | x 向图像右，y 向图像下，z 向前；不是 CAD 外壳原点 |
| D / W | 桌面规划场景 D 与外部世界 W 分开；方向均为 +X 入厢、+Y 向左、+Z 向上。T_D_base 平移 [-0.44,0,0.24] m、绕 Z 转 π；T_W_D 平移 [0.32,-0.1,0.8] m，合成 T_W_base 平移 [-0.12,-0.1,1.04] m |

参考 URDF SHA-256 为 `f842fc3e6973e37d622553e8cc614e8eb2e133a955e8648541ac4ea854ae6faf`；
主模型为 `08b33a0588839f9fef4f08e13bb04ff27071c332f5ac40e688949a1ea5e9970b`。
两文件哈希不同，不能宣称同一资产。关节 origin/axis/parent/child 一致，但限位精度不同，
参考 J3–J6 速度上限 3.926991 rad/s，主模型为 3.14 rad/s。保留主模型限位；超限实测值仍显示为原始反馈，拒绝计算场景姿态。

三组姿态逐一比较 base_link、link_1–link_6 的完整齐次矩阵：

| SDK 角度（J1–J6，°） | 比较结果 |
|---|---|
| [0,0,0,0,0,0] | 全链一致至浮点舍入误差 |
| [20,-30,45,10,25,-15] | 全链一致至浮点舍入误差 |
| [-35,15,-40,20,-30,45] | 全链一致至浮点舍入误差 |

所有样本最大分量差 `2.5263315854746805e-16`。方向/零位映射有模型与读取代码依据，尚无控制器实测独立确认。
不通过名称相似、模型拟合误差或训练残差推断真实抓取精度。

实际使用链为：

```text
T_base_camera(q_capture) = T_base_link1(q_capture) @ T_link1_camera
T_world_base = T_W_D @ T_D_base
T_world_camera(q_capture) = T_world_base @ T_base_camera(q_capture)
p_world = T_world_camera(q_capture) @ p_camera_optical
T_world_tcp(q) = T_world_base @ T_base_link6(q) @ T_link6_tcp
```

数值区 world 明确指 W；原场景渲染器使用 D，所以显示箭头前显式乘 `inverse(T_W_D)`，
不把 W 的点直接叠到 D 场景。未加载场景时才明确采用 identity world=base_link。

q_capture 是原始 RGB-D 接收时刻附近的关节候选；只有人工静止证据时才用于阶段一近似可视化。
无共同曝光/控制器时钟，本阶段不存在“已可靠同步”状态。推理结束后的最新关节不替换原观测关联。

训练外参保持 `SOLVED_REQUIRES_VALIDATION / TRAINING_ONLY_TEST`，界面持续显示“待独立验证”。
`training_extrinsics.example.json` 是原文件的可运行摘录，记录原文件 SHA、模型 SHA，明确不是重新求解。
原 `session_file` 所指的 Windows 采样会话没有随参考库提交，MISSING 状态保留，无伪造验证记录或会话哈希。
原 `hand_eye.json` 的旋转包含 0.5 剪切项，非正交，刚体校验拒绝；没有自动 SVD 修正。

## 安装冲突与完整工具

参考探针为 link_6 下 [0,0,0.07] m，参考吸具为 [0,0,0.14] m，分别配置。
`tool_tip.json` 没提供工具姿态，当前方向只允许作为 IDENTITY_ASSUMPTION_REQUIRES_PHYSICAL_VALIDATION 的显示假设。
换工具只改显示 TCP，不修改原外参 `settings.probe_tip_m`。

主仿真 TCP 为 z=0.14275043380451888 m，且含约 -0.121818° 的 Z 旋转。
`T_simulation_tcp_reference_tool` 明确记录约 -2.7504338 mm 的 Z 差以及反向旋转差；不把仿真 CAD 接触面自动当作实际 TCP。
场景沿用完整工具的 57 个碰撞体和 12 杯模型，工具 marker 与原几何分别显示，没有缩减成 TCP 点碰撞模型。

训练相机与原快照的 `T_link1_optical` 有显著安装差异。显式变换
`T_simulated_optical_calibrated_optical = inverse(T_link1_optical_nominal) @ T_link1_camera_training`
记录于证据；其平移约 [0.0895716,0.2772678,-0.0547559] m。
原相机 CAD 使用 `T_mast_CAD`，原光学坐标使用 `T_mast_optical`，二者的相对变换由快照明确保存。
工作台保留原 CAD 外壳和安装组件，并单独画训练光学坐标，界面显示安装不一致及完整差异矩阵。
**没有叠加历史 180° 补偿，也没有为了显示重合修改训练标定。**
