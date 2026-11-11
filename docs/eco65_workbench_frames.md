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

## 2026-11-11 本机安装定义

用户最终确认：正式吸具，相机安装在 `link_1`，world 原点采用机器人底座。
早先的“外部固定支架”描述已被纠正。本机将 world 的轴也定义为 base_link 的轴，
因此 `T_world_base = I`；这是坐标定义，不是现场标定结果。历史规划场景 D 的方向、
冻结快照和规划器保持原定义，显示时使用 `T_base_D = inverse(T_W_base) @ T_W_D`。

安装入口为 `--installation configs/workbench/link1_base.example.json`。
本机启动器使用 `outputs/workbench/installation.local.json`，并加载原有
`configs/workbench/training_extrinsics.example.json`。加载文件不连接设备。
相机链为 `T_world_camera = T_base_link1(q_capture) @ T_link1_camera`；
相机随 J1 运动，J2–J6 不改变这条安装链。仍要求原始观测的关节关联与人工静止证据，
不把接收时间近似称为曝光同步。训练外参状态及缺失会话证据保持不变。

已检查用户提供的 `DuuQue/Eco65-RoboArm-System-Prototype`，当前公开分支提交为
`c1a303d10f86f1c21ccd57ec20591b3f0e4c3f12`。其中
`docs/eco65_tilted_camera_touch.md` 的标定输入是渲染图像；
`docs/eco65_camera_edge_clockwise_20261020.md` 明确后续安装的旧标定失效。
这些仿真记录不能作为当前 D435i 的实机验证记录。

正式吸具 STEP 的记录位置是
`assets/tools/desktop_suction/cad/raw/KVGL180-130-260909-01.STEP`，
2,998,104 字节，SHA-256
`aa8d820d56fe3853b1b052e0c76dd77e666abd562427f4dff1b07ef1d6f6cf6c`。
两份仓库均忽略该原件及派生产物，本机未找到该文件；
`res/上海皖泰真空吸盘三分区.STEP` 是历史 FANUC 资产，不能替代十二杯 ECO65 吸具。
已有审查记录的 CAD TCP 轴向距离约 142.750434 mm，但安装滚转仍是仿真假设。
原件恢复后可复核 CAD 定义，不能直接声称实测 TCP 已完成。

本机安装配置的工具矩阵保持 `null / AWAITING_TCP_MEASUREMENT`，界面不回退到
70 mm 探针或 140 mm 参考工具。相机坐标计算可独立使用；实际工具 TCP 计算明确拒绝。
“加载实测工具 TCP”接受显式 `name / parent_frame=link_6 / unit=m / matrix / source / status`，
校验刚体矩阵并展示原状态；错误替换会清除旧值。未执行运动或新的实机标定。

定向检查覆盖安装父坐标系匹配、J1 数值变化、world/base 恒等关系、场景显示变换、
未知 TCP 拒绝、无设备自动连接及原始标定状态保留。

## 2026-11-11 下载资产后的复核

此节更新上面的“原件缺失 / 本机工具矩阵为空”记录。资产发布提交 `00d2e75`
已下载，4,084 项发布文件的大小和 SHA-256 均通过。正式吸具 STEP、派生清单、
工具配置及工作台快照均已在本机。新增可复跑的只读检查：

```bash
.venv-workbench/bin/python tools/audit_eco65_installation.py \
  --output outputs/calibration/20261111/downloaded-assets-frame-audit.json
```

本轮没有重新解析/离散化 STEP、重求解相机标定、运行规划或连接硬件。
核对原件到派生清单的哈希、毫米到米的单位记录、全部 43 个视觉实体和 57 个碰撞分区文件，
从 CAD 曲面特征清单重新提取并验证全部 12 条原始接触轮廓。
另从官方 link_6 STL 重新读取六孔位置，与 CAD 法兰面/孔位重新配准；
最大孔位残差约 `2.9553e-9 m`。该值是模型配准误差，不是实机装配精度。

`T_flange_tcp = T_flange_tool_cad @ T_tool_cad_tcp` 一致。
TCP 原点为十二个接触线中心的均值，法兰下平移约 `[0,0,142.7504338] mm`，
绕法兰 Z 的参考角约 `-0.12182005°`，全部唇缘落在 TCP 的 Z=0 平面。
六孔仍存在六种等价安装角，`hardware_indexing=NOT_VERIFIED`，
`actual_tcp_calibration=null`，`independent_control_verified=false`。
本机安装配置现加载 `outputs/calibration/20261111/cad-tool-reference.json` 的相同内容，
标记 `CAD_REFERENCE_ONLY / SIMULATION_MOUNT_ASSUMPTION`，界面名称为
“正式吸具 CAD参考 TCP（未实测）”。这只供只读显示，不提升实机标定状态。

world/base 按用户定义保持恒等。四组姿态使用独立 XML 变换连乘与现有 FK 对照，
最大分量差约 `3.33e-16`。历史 `T_W_base` 平移仍是 `[-0.12,-0.1,1.04] m`、
绕 Z 180°；转到本机 world=base 后，`T_base_D` 平移为 `[-0.44,0,-0.24] m`、
绕 Z 180°。相机和 TCP 经 D/base 往返结果一致；J2–J6 不影响 link_1 相机变换。

相机证据必须分开保留：

| 外参来源 | 本轮结论 |
|---|---|
| 工作台训练外参 | 刚体矩阵/父子坐标合法；仍为 `SOLVED_REQUIRES_VALIDATION / TRAINING_ONLY_TEST`，原始会话缺失 |
| 快照名义外参 | 由仿真安装推导，原状态 `NOMINAL_FROM_CHANGED_INSTALLATION_NOT_CALIBRATED` 不变 |
| 快照内嵌求解外参 | 6 张照片/9 个触点的嵌入元数据与求解结果绑定一致；来源明确为仿真，原图未纳入本次资产清单，未重做图像验证 |

训练外参与快照名义外参相差约 `296.477 mm / 170.817°`；与快照求解外参相差约
`294.826 mm / 170.924°`。这反映不同安装/来源的差异，不能当作 D435i 的实测误差或
叠加补偿量。快照报告的约 1.180 mm 留出点误差属于历史仿真结果。
当前 D435i 的安装身份和定位精度仍需独立现场验证。

快照自身指纹通过，但运行依赖中 2,407 项资产通过 2,404 项、19 项配置通过 14 项：
共 5 个原始依赖缺失、3 个历史配置哈希不匹配。完整场景验收仍被拒绝，
未改写历史配置、快照或指纹以消除失败。完整路径/期望哈希以及 6 张仿真标定图的清单
位于 `outputs/calibration/20261111/missing-original-evidence.json`。
本轮 81 项定向测试通过，本机实际 Tk 界面确认显示“CAD参考 / 未实测”，设备均未连接。
