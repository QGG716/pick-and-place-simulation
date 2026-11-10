# ECO65-B Ubuntu 只读工作台（阶段一）

本轮基于实际 HEAD `269ab67f8932771b6f1ad1f43ed73beaf6f90602`，在
`feat/v0.6-eco65-ubuntu-real-integration` 实施。授权访问的参考库实际 HEAD 为
`f0353d720374066528f258de6f4de24502408535`；两者与用户此前审查基线相同，未回退分支。
在线规划参考分支 HEAD 为 `f7f7e0934ad425ab62cd2df8201812df1ba8e505`，本轮不导入调度器。
版本锁见 `configs/workbench/source_versions.json`。

## Ubuntu 安装与启动

部署目标是 Ubuntu 22.04，保留主工程现有 ROS 2 Humble / MoveIt 2 环境。
工作台、采集、视觉、状态查询均在同一 Ubuntu 工作台运行；无需 Windows 前端。
先在工程根目录执行：

```bash
bash scripts/install_eco65_workbench_ubuntu.sh
source .venv-workbench/bin/activate
eco65-workbench
```

启动无硬件时正常显示 DISCONNECTED。没有摄像头、SDK、串口或模型权重也不会阻止最小界面启动。
不自动连接，不因连接失败产生模拟数据。连接配置使用
`configs/workbench/stage1.example.json`；界面导入旧配置仅接受五个观测配置字段。
`enable_hardware`、`mode`、自动连接、阀门、报警回调等旧字段均被忽略并列入日志。

明确标记的离线演示：

```bash
eco65-workbench --make-demo outputs/workbench/demo
eco65-workbench --replay outputs/workbench/demo \
  --calibration configs/workbench/training_extrinsics.example.json
```

demo 是程序编写的 RGB-D、三组关节、四路压力及矩形分割夹具，来源为
`REPLAY / original=SIMULATED`，设备时间 UNKNOWN，**不是实物采集或模型推理结果**。
加载回放先校验完整首条记录，成功后直接显示首帧；每次“下一条回放”只推进一条观测。
过期后显示 STALE；末尾保留末帧并明确标记 `HISTORICAL_END_OF_REPLAY`，不再参与配对；
读取错误和停止回放均清除预览、检测关联和配对。切换开始后加载失败也不会恢复旧实测数据；
取消目录选择保留当前观测。连接设备前需先停止回放（包括已到末尾的回放）。
没有人工确认静止时不计算观测到机器人坐标的配对；人工确认仍只标记为近似，可靠同步标志恒为 False。

坐标计算需要主工程固定官方 URDF（SHA-256 `08b33a0588839f9fef4f08e13bb04ff27071c332f5ac40e688949a1ea5e9970b`）。
沿用已有私有资产恢复流程，不把 CAD、训练会话、权重或账号数据提交进 Git。
缺少模型时保留 RGB-D/压力/关节离线路径，坐标区明确报告模型不可用，不生成替代机器人。

完整三维场景复用 `NativeSceneWorld`、已有 URDF FK 和 MuJoCo 几何显示：

```bash
eco65-workbench --replay outputs/workbench/demo \
  --calibration configs/workbench/training_extrinsics.example.json \
  --scene-snapshot outputs/workbench/scene_snapshot.json
```

本次采用既有 `outputs/transverse_process_20261023/scene04/snapshot.json` 作为显示快照，
SHA-256 `02c1384bae3638a04324714d2a6cc83689c2dba390587ceaf87a75331e848ed9`。
应同时恢复快照列出的 `asset_sha256`、`config_sha256` 路径及原私有 CAD；显示前逐文件验证，缺失或变化明确报错。
场景保留完整机器人、57 个工具碰撞体、12 杯几何、相机外壳、立柱、厢体、箱体和横向传送带。
坐标箭头显示加载的光学坐标和所选 TCP，原仿真相机外壳和工具几何保留各自安装变换，差异在坐标页显示。
只调用几何更新 `mj_forward`，不调用 `mj_step`。
旧通用渲染器只接收 native world 的凸体；工作台将原有 box 基本形状精确转换为八顶点，
在内存中生成标注 `DISPLAY_ONLY_NOT_PLANNING` 的显示副本，尺寸、姿态、对象和原快照均不改变。
显示原机器人/工具表面及完整碰撞几何；相机、立柱、传送带的凸体显示保留原装配关系。

历史快照的冻结配置可能与当前源码配置不同。本次有三份文件不同：
`configs/workcells/eco65_desktop_4layer_v1.yaml`、`configs/isaac/conveyor_assets.json`、
`configs/workcells/eco65_small_carton_setback_base_v1.yaml`。
需显式追加 `--scene-input-root /path/to/archived-source-repo` 校验原始冻结输入。
不会重写当前分支配置，也不会跳过哈希：先完整验证归档输入，再验证当前渲染器使用的全部几何资产。
场景几何来自快照中冻结的 scene/tool，归档配置只作为来源证据。
本次归档根为服务器原有 `eco65-native-20261015/repo`，新工作台使用独立 `eco65-real-integration-20261109/repo`。

## 可移植界面与复用范围

已审查参考 `gui.py`、`mesh_view.py`、`workflow_ui.py`、`camera.py`、`camera_geometry.py`、
`rgbd_geometry.py`、`robot_telemetry.py`、`controller_arm.py`、`vacuum.py`、`kinematics.py` 和实际标定文件。
`gui.py` 的 Tk 数据组织可移植，但 `_vacuum_alarm` 会触发 `emergency_stop`；不导入旧 GUI。
`mesh_view.py` 使用 Windows ctypes/wintypes、WGL 和原生窗口；`workflow_ui.py` 直接依赖它及另一套 FK。
因此采用 Tk 最小页面，复用参考界面的分组方式与观测业务，使用 Linux 字体；三维入口复用主工程。
没有挂接旧自动卸箱页、假按钮或未迁移的完整标定向导。旧参考库标定算法保持原状，阶段一只加载、校验和显示外参。

`workbench/reference/` 仅包含经审查的纯函数/视觉适配：GroundingDINO、SAM、camera_geometry 原文件；
rgbd_geometry 的输入验证；vacuum 的接收解析器和协议映射。精确来源与选择范围见 `reference_sources.json`。
重新导入用 `python tools/import_eco65_observation_reference.py /path/to/authorized/reference`，HEAD 不符时拒绝。
未复制串口 writer、RobotExecutor、GUI 报警运动回调、Windows OpenGL 或旧保存配置。

## 真设备观测接口

现场在上述环境按参考库所用 SDK 安装 `pyrealsense2>=2.54`、`Robotic_Arm`；先验证 SDK 与实际固件兼容。
串口权限按 Ubuntu 现场账户设置（dialout），设备路径按实际枚举填写。
没有安装硬件 SDK 时，用户点击连接才会看到明确的模块/设备错误，离线模式继续可用。

* D435i：点击连接只枚举设备和实际支持的 BGR8/Z16 配对 profile，通过 SDK `can_resolve` 后列入选项；
  点击开始才采集。深度经 SDK 对齐 RGB，保存 uint16 原深度和真实 `depth_scale`。
  保存设备标识、流配置、RGB/depth 各自帧号、SDK 时间与时钟域、接收 monotonic/UTC、处理完成时间、内参与畸变。
  SDK 时间不可用时标 UNKNOWN，主机时间不冒充曝光时间。颜色预览为 RGB。
* 机器人：独立 `RobotObserver` 只允许 `rm_create_robot_arm`、`rm_get_current_arm_state`、`rm_delete_robot_arm`。
  后两者分别是状态查询与关闭连接。无 enable、move、stop、坐标系设置或拖动指令。
  SDK `joint` 的度值保留为原始观测，再经显式映射转弧度；控制器采样时钟 UNKNOWN。
* 真空板：115200、8N1，串口只 read/close，DTR/RTS 在打开前设为 false。
  被动接收 EE16 心跳，无串口 write，连 `0x31` 查询也不发送。
  0x61–0x64 为吸具、储气罐、真空泵、压缩机，小端 float32/kPa，校验失败不更新。
  保留源码硬件核对映射 0x71 正压、0x72 负压；本阶段不发送这些指令。
  0x51/52、53/54、55/56 分别对应空压机、吸具总阀、真空泵，绝不解释为三个吸盘。
  每个通道在 receive/parse 时记录独立时间，GUI 消费不更新时间；旧心跳不会被新 ASCII 值默默替代。
  ASCII 标记 `ASCII_DEBUG_UNCHECKED`，二进制 `CHECKED_BINARY`。连接期间显示锁存报警及年龄，只记日志；
  断连后不沿用旧压力/报警缓存作为当前显示，重连清空锁存。

窗口始终显示来源、连接、过期状态。实测姿态、仿真姿态和规划轨迹预览分别选择；轨迹滑块只选预览样本。
退出只结束采集/关闭连接；没有本系统发起的真实运动任务，故不调用旧机器人 stop。
这是一条应用层允许列表边界；没有硬件现场报文抓取证据，不能称 SDK 内部或电气副作用已验收。

## 检测、回放与原始观测

已有检测结果可随 `eco65_observation_replay_v1` manifest 导入，实例由 `--make-demo` 生成。
每条 record 绑定 frame（无 pickle 的 RGB/depth NPZ）、joint、pressure 与 detections。
detections 必须引用原 `observation_id`，bool NPY mask 必须与原图同尺寸；越界路径、错配 ID 拒绝。
保留原始 receive/process/device 信息，另加 replay presentation time 用于回放新鲜度，不重写为真实采集时间。
`source=REPLAY` 只表示展示方式，`original_source` 标识原始采集来源；只允许原始 LIVE/LIVE 或
SIMULATED/SIMULATED、同一非空原始 host_session 的候选继续检查接收时间差和人工静止条件。
已经标记 REPLAY 的输入缺少原始来源时保留未知，不能将 REPLAY 当作采集来源，也不能与另一未知来源配对。
回放切换会使未完成的旧推理结果失效；实测数据有效性始终依赖设备实际连接和新鲜度。

外参文件必须显式声明 `unit="m"`；毫米、缺失、null、空值或其他单位直接拒绝，不换算或修正矩阵。
非法替换立即停用旧外参，清除相关动态坐标及旧场景显示。训练外参仍显示待独立验证。

实时推理通过 `--detector-config configs/workbench/detector.example.json`，先将路径换成现场已有权重和配置，
并安装参考环境的 torch、GroundingDINO、segment-anything。点击“开始检测”才加载模型。
直接调用参考适配器；DINO 框/类别/分数和 SAM 轮廓转为只读结果，保留当时 RGB-D 以及对应关节候选。
分割区域深度中值是 camera_optical 下观测点，不是抓取姿态。
标定投影复用原畸变有效性检查，未知模型或不支持的非零逆畸变不假设为零。
不导入跟踪覆盖层坐标，不重写检测算法，不自动下载或伪造模型权重。

坐标页只在数据未过期、来源/主机时钟域一致、接收时差不超过 100 ms 且人工确认静止时显示近似变换。
关节候选从接收历史中选取并与原检测一起保留，不能在推理完成时换成最新机器人姿态。
接收时差不是曝光同步证据，人工确认也不是自动静止判据；阶段一不提供运动视觉闭环。

## 后续执行接口

`policy.ExecutionPort` 预留 `submit(trajectory)`、`feedback(task_id)`、`cancel(task_id)`。
当前唯一注册实现 `DisabledExecution` 对三个调用均抛 `ReadOnlyViolation`，cancel 不映射为机器人停止。
下一阶段提交合同应包含唯一 task_id、joint_1–joint_6 顺序、弧度/秒轨迹、模型/工具/外参版本、
world/base 变换、规划碰撞验证与现场授权标识。反馈需来自控制器，区分接收、实际开始、完成、失败和取消。
GUI 预览 JSON 只含 `joint_names`、N×6 `q_rad`、N 个非递减 `times_s`，不可作为执行许可。
后续 scheduler 只能经这条边界接入，不把动画时间当实际进度。

本轮不改抓取候选、IK 搜索、放置策略或原 MoveIt 入口。
箱型保持 150×100×100 mm；既有厢体和横向输送布局保留，不强制 40 箱；宽高按完整循环可达性另行确定。
没有新增箱底低于 100 mm 的顶吸开关，也没有去程/回程 J1 方向约束。
历史快照只作为几何来源；历史任务策略字段不由工作台执行。

复现检查、实际截图与现场待验证项目见 [验收记录](eco65_workbench_validation.md)，
坐标细节见 [坐标映射](eco65_workbench_frames.md)。
