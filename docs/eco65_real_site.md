# ECO65-B 真实现场单箱工作台（2026-11-12）

本入口自动连接已配置设备并持续显示视觉结果；真实运动只由操作者点击“执行单箱”启动。默认只读 `eco65-workbench`、空载 `eco65-workbench-controlled` 和历史仿真查看器仍是独立入口。

## 启动

在仓库根目录、现有 Ubuntu / Humble / `.venv` 环境运行，无需重建接入或标定：

```bash
source /opt/ros/humble/setup.bash
cmake --build build/moveit --target m710_moveit_worker m710_clearance_test -j2
cmake --install build/moveit --prefix "$PWD/install/moveit"
.venv/bin/python -m unloading_sim.workbench.single_carton.ui --check-config
.venv/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config configs/workbench/single_carton.example.json
```

安装过当前包时也可运行 `eco65-workbench-single-carton`。`--ui-smoke` 专用于开发，禁用自动连接并自动关闭；`--check-config` 只检查文件。旧进程不会热更新；现场先按原工作台的停止/持箱处理流程结束旧会话，再启动新版本。串口或机器人被其他进程占用时会报告失败，不抢占连接。

主窗口出现后，三个独立设备线程分别连接。相机优先使用已保存序列号，未指定且只有一台 D435i 才自动选择；多台时错误详情列出序列号。默认 RGB-D 为 **640×480@30fps**，不可用时保留原因，进入“相机配置 / 原始预览”或“现场配置”更改后重启，不自动降分辨率或帧率。

机械臂连接线程会先加载并启用系统中**已保存的 NetworkManager 有线网络配置**，再连接控制器。当前配置为 `ECO65-robot-eno1` / `eno1`，保存的本机地址为 `192.168.1.100/24`，控制器为 `192.168.1.19:8080`。已经激活时仅检查路由，不断开重连；应用不新建网络、不猜地址、不修改配置或 Wi-Fi 默认路由。配置必须为专用有线网络且 `ipv4.never-default=yes`。缺少配置、网卡冲突、权限不足、启用超时或路由仍走 Wi-Fi 时，设备详情显示具体错误，其他设备照常工作；“重试机械臂”会重新检查该网络。

检测和分割两个窗口自动打开，可移动、缩放、独立关闭及重新打开。关闭只隐藏窗口。检测结果含原帧、框、类别和分数；分割结果含对应原帧、掩码和轮廓。分别显示 RGB/depth 帧号、原始帧年龄、处理状态及实际显示刷新率。模型加载和推理失败直接显示，可“重试模型”；不存在演示结果兜底。

设备摘要区始终可见，详情显示地址/串口和失败原因。通信连接、等待首条数据、数据有效、数据过期和执行条件分别显示。某个设备失败不会阻塞另两个设备或视觉窗口。机械臂故障码、关节反馈年龄、吸具压力及其原始年龄显示在任务区。重试连接按钮不会执行运动或阀门命令。

## 配置位置

| 内容 | 文件或字段 |
|---|---|
| 单箱入口 | `configs/workbench/single_carton.example.json` |
| 已保存连接信息 | `device_config`，默认 `outputs/workbench/stage1.local.json`；存在时复用 robot_host、robot_port、serial_port、camera_serial |
| 已保存机器人网络 | `robot_network.manager=NetworkManager`、`connection` 为系统连接名称或 UUID、`interface` 为网卡；`device_config` 中同名对象优先 |
| 连接信息缺省 | 原 `controlled.example.json` 的机械臂地址/端口，单箱配置的串口；不改现场原文件 |
| 相机采集 | `camera.serial / width / height / fps`，缺省空序列号、640、480、30 |
| 箱型先验 | `geometry.prior_size_m`，默认 `[0.15,0.10,0.10]`；`dimension_tolerance_m` 默认 0.008 |
| 实际机器人、TCP、映射 | 原 execution 配置指向的 installation、mapping、snapshot；snapshot **只提供硬件模型** |
| 原始外参 | `configs/workbench/reference/active_extrinsics.json`，原字节及历史状态不变 |
| 真实现场 | `site_scene`，默认 `configs/workbench/real_site.example.json` |
| 本地模型 | 原 `detector.dino_directory / sam_directory`，HF config/safetensors；初始化一次，默认 CPU 2 线程，可配置已有 CUDA 设备 |
| 记录 | `outputs/single_carton/<task_id>/` |

界面“现场配置”可编辑、滚动和复制单箱、现场、连接 JSON；保存后要求重启，不在活动任务或持箱时应用。建议将现场配置另存到忽略的 `outputs/workbench/real_site.local.json`，并修改单箱配置的 `site_scene`。所有数值用米、弧度、秒、千克；界面关节以固定精度度数显示。

标定和安装文件中旧的 `SOLVED_REQUIRES_VALIDATION` 等标签是原始证据，用户本轮现场确认单独绑定，不改成伪造的实测验收报告。当前现场约定 world=base_link、相机在 link_1；不会读取历史仿真的 `T_W_D` 或旧基座位置来移动现场障碍。

## 现场环境与承接位置

真实场景由冻结的当前 RGB-D、目标完整包络、其他检测物、未分类可见障碍和少量现场测量组成。未成功恢复的邻箱保留为深度障碍，不再因为一个邻箱无法唯一恢复而删除它或统一拒绝全部目标。机器人自身深度点仅在落入同次 q_capture 的源 CAD 凸实体内部时去除；机器人所有碰撞实体仍在原生模型中。

`workspace_bounds_m` 指定有限工作范围，默认只是搜索范围，不是现场空旷声明。原生检查覆盖每个机器人连杆和附着包络，不能从未建图范围绕路。`voxel_m` 默认 0.04 m，粗网格可能产生保守拒绝；可在配置资源上限内减小。只有整个体素投影均有更远有效深度，才记为已观测自由空间。无深度、遮挡后方、视野外区域按 **OCCUPIED** 保留；不得通过清空 world 跑通。

少量看不到的固定桌体、支架使用 `fixed_obstacles` 补充，必须有测量依据。已核对为空旷的盲区可用 `measured_clear_regions` 指定，必须记录实际依据；该配置不会抹去本次观察到的障碍。禁止把整个工作区随意标为空旷。

现场模板的 `fixed_obstacles`、`measured_clear_regions` 为空、`receiver=null`，因为本轮没有提供桌体、盲区和承接面的实测尺寸。**这些值不能用仿真传送带或猜测坐标代填。** 观察和视觉会正常启动；缺少承接面时，规划明确返回 `SITE_RECEIVER_NOT_CONFIGURED`。示意格式如下，数字仅说明格式，必须换成现场测量：

```json
{
  "fixed_obstacles": [{
    "id": "site_receiving_table",
    "size_m": [0.50, 0.40, 0.04],
    "pose": [[1,0,0,0.35],[0,1,0,-0.30],[0,0,1,0.08],[0,0,0,1]],
    "measurement_basis": "填写现场测量日期、方法及实际数值来源"
  }],
  "measured_clear_regions": [{
    "bounds_m": [[-0.10,-0.15,0.15],[0.20,0.15,0.50]],
    "measurement_basis": "填写该盲区已核对无固定障碍的现场依据"
  }],
  "receiver": {
    "support_ids": ["site_receiving_table"],
    "bounds_m": [[0.12,-0.48,0.06],[0.58,-0.12,0.10]],
    "margin_m": 0.003,
    "measurement_basis": "填写承接面实测依据"
  }
}
```

receiver 上边界 Z 必须等于已绑定支撑物的水平顶面，水平范围必须落在实际桌面内。返回同一 q_start 后，完整包络须在该范围内、底部与顶面间隙不超过 5 mm；所有保留的几何假设也必须满足承接高度。它是基于现场几何的检查，不等于接触传感器已证实支撑。**本轮不再使用此前的承接区延期政策。**

## 部分可见纸箱

从同帧 SAM 掩码提取深度，去除跳变、无效深度和离群点，确定性 RANSAC 拟合局部平面。真实可见的平面到更远背景的深度棱边，以及可见侧面法向，可以约束方向与中心；图像边界、前景遮挡边缘和 SAM 自身边缘不自动变成箱边。

尺寸先验在配置中读取，没有写死在恢复算法内部。无法唯一恢复中心/方向时，保留尺寸排列、连续 yaw 区间及中心区间，输出所有可行假设的保守包络。输出明确区分 `observed_faces`、`inferred_faces`、`uncertainty`、`nominal_size_m` 和 `size_m`（用于碰撞的完整包络）。厚度来自先验时 `hidden_extent_measured=false`。

`dimension_tolerance_m` 应填写现场箱型量测的实际误差界。默认 8 mm 会保守扩大箱体；如果这使返回后的底部高度区间超过承接面的 5 mm 允许范围，系统会报告 `GEOMETRY_UNCERTAINTY_AFFECTS_RECEIVER`。补充尺寸/可见边证据后重新准备任务，不会在执行时自行缩小包络。

候选只在实际观察到的深度连续面上生成。真实 CAD 密封轮廓及周围 3 mm 边缘带都检查深度支持，未观测孔洞不补成密封面。预计密封杯和允许目标边缘接触的被动杯唇分开，全部 12 杯共用一个阀门，未密封杯不能算成独立控制或密封证据。带载原生检查使用完整不确定包络，并在固定附着关系中复核可见接触区、密封间隙和所有允许杯唇的穿入限值。

部分出画面、被邻箱遮挡或只看到局部平面本身不是拒绝原因。可见区域不足、法向不可靠、观测与尺寸先验冲突、环境未知体素挡路、带载包络碰撞或假设的承接高度不一致，会给出各自原因。可见面已经能密封，不代表未知厚度和现场盲区已经消失。

## 现场操作与验收

1. 在控制器中核对单箱设定为工具 1.5 kg + 箱子 2 kg，即 3.5 kg；本入口只读取设定，不自动写工具配置。机械臂额定 5 kg。打开新入口，检查三类设备状态、故障详情、实时检测和分割窗口。
2. 配好实际箱型、现场障碍和承接面。选择停稳、带箱能返回并释放的位置；此处就是本任务起点，不使用零位或 HOME。
3. 准备一个**部分出画面或部分遮挡，但可见面足以密封**的真实箱体。点击“准备观测 / 锁定起始点”。关节持续停稳至少 0.4 s，RGB-D 年龄不超过 100 ms，采集关联关节接收时差不超过 30 ms。q_start 只冻结一次；相机坐标使用 q_capture，绝不使用推理结束时姿态。
4. 查看冻结图像、恢复几何和不确定包络，选择检测编号或留空自动选择，点击“规划所选箱”。查看完整预览；计划包含接近、接触、带载脱离、独立规划的带载返回，并先验查带载终态及释放包络。
5. 操作者点击“执行单箱”。执行前再次停稳取景，检查关节起点、目标深度覆盖、外观、场景深度、会话及文件版本；变化时作废计划，重新准备。局部连续深度变化独立检查，小障碍不会因只占全图很少像素而被忽略。后台视觉继续，绝不替换任务目标或轨迹。
6. 实测接触 FK 合格后才建立真空；吸具通道至少两条新心跳、跨度至少 1 s 且 ≤−60 kPa 才脱离。带载压力 >−30 kPa 停止。带箱实测返回同一 q_start 后，复核释放几何并发释放；两条新心跳跨度至少 1 s、≥−10 kPa 才完成。吸附/释放各超时 3 s，压力最大年龄 3 s，保留原始心跳时间，不插值假高频数据。
7. 验收记录应包含实际箱体与遮挡方式、原始 RGB-D/掩码、可见支持区、所有规划阶段、关节/压力反馈、带箱回起点及释放结果。只有实际完整链路才标记 `PHYSICAL_CYCLE_COMPLETE`。另由现场安排低速停止验收，核对反馈确认、停止后不续跑和不自动释放。

停止处理和压力监测独立于视觉。发送仍为纯 5 ms CANFD，高跟随参数、迟发/反馈过期/故障停止阈值不变。取消不会等待 SAM 完成。关闭结果窗口只隐藏显示；关闭主窗口在后台处理停止与资源关闭，可能持箱或 `STOP_UNCONFIRMED` 时保留连接并提示现场处理，不自动释放、回家、恢复或重试任务。

点击准备后立即显示“已受理”，准备按钮在当前操作结束前禁用，任务栏显示用时。
前 4 s 取景期间，详情每 250 ms 显示停稳窗口、关节位置范围/速度、RGB-D 年龄及配对时差。
停稳仍要求位置范围 ≤0.02°、速度 ≤0.5°/s；优先使用控制器原 UDP 的有效实测速度，
缺少时回退到实际位置样本差分，绝不补零。反馈窗口至少 0.4 s、样本间隔不超过 100 ms。
这样避免编码器极小跳动除以不规则短采样间隔产生虚假速度峰值，同时保留实际位置变化检查。

取景成功即显示原始冻结图像、q_start、q_capture 和帧号；随后复用已加载模型处理这同一帧。
若模型仍在处理实时帧，会显示“等待当前帧推理结束后处理已锁定帧”，接着显示检测或 SAM 阶段及用时。
实时窗口已有结果不代表新任务已经冻结；不能拿几十秒前的框/掩码套到新图像。
取景失败会在任务详情及提示栏显示具体未通过条件；不用重复连点按钮。
调整现场状态或等待设备恢复后可重新准备；新任务会清空旧图像对应的分割、几何及计划。

## 任务记录

每次准备建立独立 `outputs/single_carton/<task_id>/`。重新观测建立新任务，保留旧任务与拒绝原因。

| 文件 | 证据 |
|---|---|
| `observation.json/.npz` | 原 RGB-D、内参/尺度、帧号与时钟、q_capture、q_start、采集变换；`capture_checks` 保存停稳、实测速率来源及配对数值 |
| `segmentation.npz`、`geometry.json` | 同帧掩码、检测、实测接触区、推断部分、有限假设、拒绝原因 |
| `site-scene.json` | 冻结现场障碍、未知空间、测量区域、自身深度去除统计；仿真物体导入数为 0 |
| 候选子目录及 `planning-failures.json` | 发出前保存的每次 Pilz/OMPL 请求、独立回复、超时类别和完整发送路径检查 |
| `complete-plan.json` | 四段计划、固定附着、现场释放检查及代码/模型/配置 SHA-256 绑定 |
| `pre_execute_observation.json/.npz` | 执行前新采集；旧观测的时间戳不变 |
| `events.jsonl`、`motion/<motion_task_id>/` | 阶段、原始压力心跳、实际关节、阀门字节、5 ms 发送时间与抖动、停止反馈 |
| `result.json` | 最终状态和实机验收结论；取景超时的 `evidence` 保留具体等待条件与数值；未运行、失败或假设备不能产生实机完成结论 |

目标附着时原生场景移除其同 ID 的世界物体，改用完整附着包络；邻箱、现场固定物和未知障碍保留。持续视觉仅更新显示，不能把附着目标重新写入正在执行的场景。

## 软件验证与实机范围

定向测试命令：

```bash
xvfb-run -a .venv/bin/pytest -q tests/test_single_carton_*.py \
  tests/test_controlled_status_ui.py tests/test_controlled_read_ui.py \
  tests/test_controlled_execution.py tests/test_controlled_payload_setting.py \
  tests/test_controlled_goal_planning.py tests/test_workbench_readonly.py \
  tests/test_workbench_coordinates.py tests/test_workbench_installation.py
source /opt/ros/humble/setup.bash
build/moveit/m710_clearance_test optimized
build/moveit/m710_clearance_test reference
.venv/bin/python tools/check_eco65_single_carton_native.py
```

最终结果 **271 passed，0 failed，0 skipped**。测试覆盖自动连接与独立失败、640×480@30fps 无静默切换、当前帧+最新待处理帧、检测先于 SAM、重连作废、GPU 忙时立即取消、Tk 控件像素位置稳定、滚动位置保留、分窗隐藏/重开、部分出视场/遮挡/孔洞、源 CAD 密封、完整不确定包络、现场场景贯通四段请求和原执行/停止回归。

原生 optimized/reference 两种碰撞检查均 PASS，保留 715 个机器人/工具碰撞形状及 12 杯。原生组件规划及 491 个最终轨迹点碰撞检查 PASS，不连接硬件，也不签发执行许可；它不是四段完整实机任务。本地真实 HF 权重已完成拆分后的 DINO 与 SAM 前向 API 检查，输入为合成图像；这不是现场识别质量验收。

开发期间检测到本机 D435i 和串口，串口由已有 Python 会话占用；未接管现有会话，未进行新实机抓放。**本轮自动连接实际设备、真实连续识别质量、部分可见实箱完整抓放及带载停止验收均为 NOT_RUN**。软件报告见 `docs/evidence/eco65_real_site_20261112/software_report.json`；历史成功/失败记录保留原样。

### 2026-11-12 晚间：恢复已保存的机器人网络

现场连接失败定位为 `eno1` 链路正常但保存的机器人网络未激活，没有 IPv4 地址，控制器访问错误地经过 Wi-Fi。启用原 `ECO65-robot-eno1` 后，`192.168.1.19:8080` TCP 可达，只读状态查询返回六关节及 `err:[0]`；现有工作台随后显示机器人数据有效，保留其连接，没有抢占。该记录仅证明现场网络与关节反馈恢复，抓放验收仍 NOT_RUN。

新增启动逻辑在机械臂设备线程中自动读取、必要时激活上述保存配置，再进入原 SDK 连接。92 项定向回归通过，覆盖激活顺序、已有连接不重复激活、错误路由、缺少配置/命令/权限、超时、其他网卡配置冲突及网络恢复期间相机和真空不阻塞。`--ui-smoke` 仍不触碰网络或设备。见同目录证据 `network_startup_report.json`。

### 2026-11-12 晚间：准备观测反馈与停稳检查

现场最近两次准备均进入后台后在 4 s 取景等待中拒绝，错误为
`NO_FRESH_STATIONARY_RGBD_JOINT_PAIR_WITHIN_30_MS`，尚未进入箱体恢复。
旧版该错误同时包含停稳、帧年龄及配对差等条件，没有保存子项数值，不能单凭错误码断定一定是 30 ms 配对超限。
只读 TCP 采样约 3 s、120 条，六关节位置范围为 0.004–0.007°；这不是同步 UDP/RGB-D 验收。

本次修复包括优先使用实测 UDP 速度的停稳判断、逐项取景进度、锁定后立即显示原始帧、
同一模型队列的任务进度、旧任务数据清空，以及失败诊断落盘。未知/无效速度不冒充实测速度。
219 项定向回归通过，覆盖编码器量化跳动、真实速度或位置超限、反馈间断、帧过期/未来时钟、
配对超限、会话变化、按钮即时反馈、失败可重试、旧掩码隔离和取消不等待推理。
Tk 实际像素位置回归及假设备截图检查通过；见 `capture_feedback_report.json` 和 `capture_feedback_pytest.xml`。

原工作台进程没有被中断或接管。重新打开软件后再点击准备，新版本才能显示以上诊断。
本次开发没有运动/阀门命令；修复后的现场冻结观测及真实抓放仍 **NOT_RUN**，不能由合成测试替代。
