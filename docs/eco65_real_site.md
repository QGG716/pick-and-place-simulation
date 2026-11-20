# ECO65-B 真实现场单箱工作台（2026-11-12）

本入口自动连接已配置设备并持续显示视觉结果；真实运动只由操作者点击“执行单箱”启动。默认只读 `eco65-workbench`、空载 `eco65-workbench-controlled` 和历史仿真查看器仍是独立入口。

**2026-11-19 当前工具间隙：** 用户明确将 `tool_robot_clearance.clearance_m` 从 0.040 m 改为 **0.005 m**，共享单箱配置和本机 CPU 配置均已更新。下文历史 40 mm 结果保留原条件。此项是工具与机器人之间的间隙，不是纸箱尺寸不确定性；其余 2.4 mm 对间隙与完整碰撞几何保持。

**2026-11-19 当前点云排除：** 按用户选择，仅使用任务 `d6d985f1` 失败记录中的 53 个具名点云体原始范围，不外扩 6 cm。`site_scene.upper_carton_pointcloud_artifacts` 显式启用并绑定清单哈希；下箱掩码、支撑区域、固定结构和完整载荷保留。清单是用户现场确认，不是实测空闲空间；详情见文末本次验收。

**2026-11-19 当前尺寸路线：** 默认 `geometry.size_policy=MEASURED_PER_INSTANCE`，不使用尺寸先验，也不要求所有箱子先测出完整尺寸。优先从可见面和侧面测量；仅有可靠局部面时，尝试用同帧 RGB-D 中证实为空的空间约束完整碰撞包络，仍可生成并验证吸取候选。没有唯一尺寸并不等于不可抓。尺寸未知的邻箱保留为保守现场障碍，由本次目标及完整带载路径判断是否可行。历史170×128×101 mm不参与当前恢复。`UNKNOWN_PER_INSTANCE` 仅面诊断和 `CONFIGURED_SINGLE_TYPE` 已知箱型仍是显式独立策略。

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
| 尺寸策略 | 当前 `geometry.size_policy=MEASURED_PER_INSTANCE` 及 `size_policy_basis`：逐箱实测或同帧 RGB-D 保守范围，不要求唯一尺寸，不使用箱型先验。保留的 `prior_size_m=[0.17,0.128,0.101]`、`dimension_tolerance_m=0.008` 与任务 `7ccdb15e` 来源均不生效，也可从无先验配置中删除。`UNKNOWN_PER_INSTANCE` 为仅面历史诊断；`CONFIGURED_SINGLE_TYPE` 仅用于明确已知且适用的同箱型 |
| 发送速度 | 单箱配置 `time_scale`，2026-11-12 按用户要求由 10 改为 **1.0**：按原生规划速度发送（规划限速 0.10 rad/s，见 `eco65_controlled_execution.md`），5 ms 周期、跟踪误差、反馈过期和停止阈值不变；空载受控入口仍默认 10，可在其界面单独修改 |
| 实际机器人、TCP、映射 | 原 execution 配置指向的 installation、mapping、snapshot；snapshot **只提供硬件模型** |
| 原始外参 | `configs/workbench/reference/active_extrinsics.json`，原字节及历史状态不变 |
| 真实现场 | `site_scene`，默认 `configs/workbench/real_site.example.json` |
| 本次点云清单 | 现场配置 `upper_carton_pointcloud_artifacts.mode=USER_CONFIRMED_NAMED_LIST`，`manifest` 指向 `configs/workbench/field/upper_carton_artifacts_20261119_d6d985f1.json`，`manifest_sha256` 绑定原始范围；设为 `{"mode":"KEEP"}` 可完整保留点云。缺省为 KEEP，不支持扩大邻域式忽略 |
| 本轮承接策略 | 单箱配置 `release_policy=USER_DEFERRED_RECEIVER_QUALIFICATION` 及 `receiver_qualification_basis`；延期不改变环境碰撞检查 |
| 本轮未观测区域 | 现场配置 `unknown_policy=USER_CONFIRMED_CLEAR` 及 `unknown_clearance_basis`；按用户“未观测区域都已清空”确认，不生成未知体素障碍，实测障碍仍保留 |
| 检测模型 | `detector.dino_directory / sam_directory`，HF config/safetensors；初始化一次。`detector.backend` 选 `local`（本机 CPU 2 线程或本机 CUDA）或 `remote_ssh`（远程 GPU 服务器，2026-11-13 起默认），两者之间不静默切换 |
| 远程 GPU 连接 | 主机、端口、用户、密钥写在被忽略的 `outputs/workbench/remote_detector.local.json`（`detector.remote_config`），不进仓库；Python、目录、设备、超时在单箱配置 `detector.remote` |
| 记录 | `outputs/single_carton/<task_id>/` |

界面“现场配置”可编辑、滚动和复制单箱、现场、连接 JSON；保存后要求重启，不在活动任务或持箱时应用。建议将现场配置另存到忽略的 `outputs/workbench/real_site.local.json`，并修改单箱配置的 `site_scene`。所有数值用米、弧度、秒、千克；界面关节以固定精度度数显示。

标定和安装文件中旧的 `SOLVED_REQUIRES_VALIDATION` 等标签是原始证据，用户本轮现场确认单独绑定，不改成伪造的实测验收报告。当前现场约定 world=base_link、相机在 link_1；不会读取历史仿真的 `T_W_D` 或旧基座位置来移动现场障碍。

## 现场环境与承接位置

真实场景由冻结的当前 RGB-D、目标完整包络、其他检测物、未分类可见障碍和已有现场测量组成。未解邻箱同时保留实测表面和可能隐藏的体积，不因其无法唯一恢复而统一拒绝全部目标。局部面内向半空间与已有可见范围约束决定可能体积，仅传感器证实为空的完整格可以排除；无可靠面的实例保留更宽范围。静态障碍只查询强制工作范围内的交集，这不表示纸箱在工作范围边界结束，也不能将该裁剪结果用于附着载荷。机器人自身深度点仅在落入同次 q_capture 的源 CAD 凸实体内部时去除；机器人所有碰撞实体仍在原生模型中。

仅正面可见时，算法保留包含实际观测面点的全部非自由连通区域：前景遮挡、无深度和画面边界均不作为箱边。若同帧远背景围住该区域，可得面内范围；沿真实观测点向箱内延伸，若遇到有完整投影覆盖的实测自由空间，可得厚度上界。二者都有限时，使用包含所有可容许尺寸、中心和面内方向的碰撞包络，界面标注“保守包络，非实测箱尺寸”。3 m搜索距离只是有限搜索预算，无空隙证据时不会变成3 m箱长。该方法沿用可信局部平面模型和6 mm自由空间余量；不会把推断的隐藏面当成吸盘密封面。

若同帧仍不能约束待抓目标的完整载荷范围，保留接触候选并说明缺失边界；其他目标仍可规划。未知邻箱可能体积与目标包络冲突时，返回几何不确定性原因，不报告实物穿透，也不把未搜索候选算作物理无解。新观测可重新收紧这些范围，当前任务的冻结数据不会被实时推理替换。

`workspace_bounds_m` 指定有限工作范围，范围数值本身不是现场空旷声明。原生检查覆盖每个机器人连杆和附着包络，并保留工作范围边界。`voxel_m` 默认 0.04 m，粗网格可能产生保守拒绝；可在配置资源上限内减小。只有整个体素投影均有更远有效深度，才记为已观测自由空间。

用户最新明确确认“未观测区域都已清空，无需检查碰撞”。当前现场配置因此采用
`unknown_policy=USER_CONFIRMED_CLEAR`，并在 `unknown_clearance_basis` 中记录本次用户确认。
无深度、遮挡后方和视野外的**未知体素**不再生成碰撞障碍；这是用户现场确认，不伪称传感器已观测。
RGB-D 实测障碍、邻箱、固定障碍、机械臂/吸具、完整箱体及工作范围边界仍参与碰撞检查。
显式改为 `OCCUPIED` 可恢复未知占用策略；缺少策略或清空依据会报告配置错误，不静默清空。

后续新增的固定桌体、支架可使用 `fixed_obstacles` 补充。局部实测空旷区可用 `measured_clear_regions` 指定并记录依据。以上配置及本轮整体未知区清空确认，均不会抹去本次观察到的障碍。

现场模板的 `fixed_obstacles`、`measured_clear_regions` 为空、`receiver=null`，因为本轮没有提供桌体、盲区和承接面的实测尺寸。**这些值不能用仿真传送带或猜测坐标代填。**

本轮按用户明确要求，单箱配置采用 `release_policy=USER_DEFERRED_RECEIVER_QUALIFICATION`，
并用 `receiver_qualification_basis` 绑定该要求。此时 `receiver` 可为空，规划不再要求补填承接区测量；
计划、实际释放检查和最终任务状态记录 `RECEIVER_QUALIFICATION_DEFERRED_BY_USER`、`support_confirmed=false`。
延期只涉及承接区资格核验：完整箱体及不确定包络、带载终态、独立带载回程、全部发送路径检查、
现场已观测障碍均保留，未知区域采用上文单独的用户清空策略；不会生成虚假支撑物，也不给载荷与实际障碍增加接触豁免。

以后显式选择 `MEASURED_SITE_RECEIVER_REQUIRED` 时才必须填写实测 `receiver`；缺少配置会在候选搜索前
直接报告 `SITE_RECEIVER_NOT_CONFIGURED`，不会静默延期或重复报告 32 次轨迹不可行。
以下是固定障碍、已测空旷区及可选承接区的格式示例，数字仅说明格式，必须换成现场测量：

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

选择实测承接策略时，receiver 上边界 Z 必须等于已绑定支撑物的水平顶面，水平范围必须落在实际桌面内。返回同一 q_start 后，完整包络须在该范围内、底部与顶面间隙不超过 5 mm；所有保留的几何假设也必须满足承接高度。它是基于现场几何的检查，不等于接触传感器已证实支撑。

## 部分可见纸箱

从同帧 SAM 掩码提取深度，去除跳变、无效深度和离群点，确定性 RANSAC 拟合局部平面。真实可见的平面到更远背景的深度棱边，以及可见侧面法向，可以约束方向与中心；图像边界、前景遮挡边缘和 SAM 自身边缘不自动变成箱边。

当前 `MEASURED_PER_INSTANCE` 使用同帧点云的主平面、近似正交侧面及可见边界逐箱测量，显式忽略历史尺寸。实测面、尺寸与位姿保存在 `measured_box`；碰撞使用单独的保守包络，直接边界加入像素与噪声区间，缺失边界采用用户给定的 0.40 m 最长边上界。未见背面不标为观测，未能恢复完整箱体时仍保留深度连续实测面与实际 CAD 密封候选。存在完整目标但邻箱尺寸未解时，邻箱按视线延伸为保守障碍，任务可以进入 `TARGET_READY`；后续碰撞或场景审计仍可拒绝。只有没有完整目标时才进入 `GEOMETRY_UNRESOLVED`，未搜索候选不报为无解。具体范围及未验证项见文末步骤 2、3。

明确适用的 `CONFIGURED_SINGLE_TYPE` 模式继续从配置读取尺寸先验，保留尺寸排列、连续 yaw 区间及中心区间和所有可行假设的保守包络，区分 `observed_faces`、`inferred_faces`、`uncertainty`、`nominal_size_m` 和碰撞用 `size_m`；厚度来自先验时 `hidden_extent_measured=false`。该模式没有作为无先验测量失败后的回退。`UNKNOWN_PER_INSTANCE` 只提取实测面，历史诊断脚本可通过 `--config` 显式指定此模式的文件。

`dimension_tolerance_m` 应填写现场箱型量测的实际误差界。默认 8 mm 会保守扩大箱体；在显式实测承接策略下，如果这使返回后的底部高度区间超过承接面的 5 mm 允许范围，系统会报告 `GEOMETRY_UNCERTAINTY_AFFECTS_RECEIVER`。本轮延期策略仍保留相同的完整碰撞包络，不会为跑通路径在执行时自行缩小它。

候选只在实际观察到的深度连续面上生成。真实 CAD 密封轮廓及周围 3 mm 边缘带都检查深度支持，未观测孔洞不补成密封面。预计密封杯和允许目标边缘接触的被动杯唇分开，全部 12 杯共用一个阀门，未密封杯不能算成独立控制或密封证据。带载原生检查使用完整不确定包络，并在固定附着关系中复核可见接触区、密封间隙和所有允许杯唇的穿入限值。

部分出画面、被邻箱遮挡或只看到局部平面本身不是拒绝原因。可见区域不足、法向不可靠、观测与尺寸先验冲突、带载包络碰撞，会给出各自原因。只有显式占用策略才检查未知体素，只有实测承接策略才检查承接高度。用户确认盲区清空不改变箱体自身的尺寸和姿态不确定包络。

## 现场操作与验收

1. 在控制器中核对单箱设定为工具 1.5 kg + 箱子 2 kg，即 3.5 kg；本入口只读取设定，不自动写工具配置。机械臂额定 5 kg。打开新入口，检查三类设备状态、故障详情、实时检测和分割窗口。
2. 配好实际箱型和现场障碍。选择停稳、带箱能返回并释放的位置；此处就是本任务起点，不使用零位或 HOME。本轮界面应显示“承接区资格核验延期；支撑未确认”及“未观测区域：用户已确认清空”，不要求补填 receiver 或盲区测量来开始规划。
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
| `detections.json`、`segmentation.npz`、`geometry.json` | 同帧检测来源、掩码、实测接触区、推断部分、有限假设、拒绝原因；检测与掩码在场景构建前保存，后续场景失败也可按原帧回放 |
| `site-scene.json` | 冻结现场障碍、未知空间策略及用户确认依据、未知总数/参与碰撞数、测量区域、自身深度去除统计；仿真物体导入数为 0 |
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

### 2026-11-12：承接区策略冲突与保存任务复核

现场任务 `4cde3b86824046f99bc8085d419deb83` 已完成新版本的冻结观测：停稳窗口约 0.405 s、
实测最大速度 0.03°/s、位置范围 0.006°、RGB-D 年龄约 46 ms、关节配对差约 3.83 ms。
恢复 1 个箱体、生成 32 个接触候选后，全部在承接配置检查处退出，尚未进入带载终态碰撞检查。
本次将默认配置、校验、实机规划和界面统一恢复为用户明确要求的承接区资格核验延期策略，
显式实测策略仍保留；延期记录不声明支撑或现场抓放已验证。

使用该任务原始 RGB-D、SAM 掩码和 q_start 离线复核时，还定位到 OpenCV 矩形边长的
float32 归一化引入了约 2.04e-8 的旋转正交误差，超过原生 1e-8 刚体检查阈值。
已将由测量生成坐标轴的归一化改为 float64；不修改原标定、不修补输入变换、不放宽原生检查。
同帧重新恢复后正交误差约 1.51e-15，箱角最大变化约 5.3e-11 m；原任务文件保留原样。

在用户后续确认清空之前，32 个候选均进入原生带载终态检查并全部拒绝：`base_link` 与未知区域
`rgbd_unknown_18_18_0` 的间隙约 1.63 mm，小于要求的 2.4 mm。
该未知柱在 base_link 下范围为 X=[−0.08,−0.04] m、Y=[−0.08,−0.04] m、Z=[−0.15,1.33] m。
这是保守未知空间检查，不是已经观测到真实物体碰撞；界面现在用 `LOADED_Q_START_UNKNOWN_SPACE`
说明区域及所需信息。现场场景仍保留 1705 个未知区域物体、40 个观测物体和 6 个范围边界，
没有加入假承接面或沿用仿真设备。该结果保留为清空确认之前的诊断证据。

更新后重新打开工作台、重新准备观测和规划；旧任务不会直接恢复执行。
用户随后确认所有未观测区域已经清空，当前配置据此仅移除未知体素碰撞物，
实际观测及固定障碍保留；不再要求补填盲区测量才能开始规划。
开发诊断不签发执行许可、不发送硬件命令；实机抓放 **NOT_RUN**。

最终定向回归 **217 passed，0 failed，0 skipped**，覆盖延期策略的实际现场入口、明确实测策略仍拒绝缺失配置、
用户清空策略只移除未知物体而逐项保留实际障碍、明确占用策略、完整带载回程拒绝、float64 刚体精度、现场部分观测及执行/压力/停止回归。
报告见 `docs/evidence/eco65_real_site_20261112/receiver_policy_report.json` 和 `receiver_policy_pytest.xml`。

按最新用户清空确认，使用同次 RGB-D、原掩码和 q_capture 重新构建场景：移除 1705 个未知物体，
40 个观测障碍及 6 个工作范围边界与原记录逐项完全一致。32/32 带载起点原生检查通过。
另对第一个候选离线检查完整链路，其接近阶段 Pilz 未通过、OMPL 在 25 s 求解预算内未找到路径；
该结果只涉及此候选，不表示其余 31 个候选不可行，也不是可执行完整计划。
工作台规划仍按顺序继续搜索候选，每个候选保留独立原生记录。实际任务需重启应用后重新冻结观测、规划和预览，
仅操作者点击“执行单箱”才能启动；本轮没有实机运动或阀门指令。

### 2026-11-12 深夜：全链路离线贯通与原速发送

现场任务 `bb88010f39ae4abdbccfeac31472d687` 已完成冻结观测、检测和 SAM，但 32 个候选全部在接近段耗尽求解预算，
整体规划约 16 min 后拒绝。用该任务原始 RGB-D、原掩码、q_capture 和 q_start，用新工具
`tools/replay_single_carton_record.py` 离线复核（不导入设备、不签发许可、源记录哈希前后一致），定位并修复：

1. **箱型先验**：按任务 `7ccdb15e` 实测改为 170×128×101 mm，`prior_measurement_basis` 必填。
   可见面另行贴合到箱体正下方实测的水平支撑面（泡沫托盘顶面），只去掉支撑面以下的包络体积。
2. **场景体素**：目标自身平面、侧面与深度跳变飞点不再生成障碍；目标附近的实测点只取实测范围。
   在用户已确认未观测区域清空的策略下，远离目标的观测体素只把**朝向相机一侧**收紧到实测点 +3 mm，
   背面保留整格；显式 `OCCUPIED` 策略不收紧。
3. **两阶段候选搜索**：先对全部候选做带载起点、接近/接触/脱离终态的原生 IK 与碰撞预检；
   吸具或箱体与环境相撞的候选直接排除（与 IK 分支无关），终态全部通过的候选优先完整规划，
   接近段以预检得到的关节目标规划。IK 种子只用本次任务自己的解。
4. **Pilz LIN 降速重规划**：直线段因关节加速度越限失败时，依次以 0.5/0.25/0.1 倍速度重新原生规划，
   不退回自由空间搜索；每次请求单独保存。
5. **脱离方案**：沿法向直线外拉被前方托盘隔条阻挡时，依次尝试 45° 斜向（外 40 mm、上 40 mm）
   和先抬升（外 10 mm、上 50 mm），全部带完整附着箱体和全部检查。
6. **原速发送**：按用户要求，`time_scale` 由 10 改为 1。原生轨迹的节点速度已经顶满 0.10 rad/s 和 0.20 rad/s² 限值，
   五次插值在节点之间会略微超出，因此只在越限时按 ×1.1/1.25/1.5/2 逐级放慢到满足限值为止；仍越限则拒绝。
   实际使用的倍数逐段写入计划证据并在预览中显示。

复核结果（**离线，不可执行，不是实机验收**）：恢复箱体包络 0.200×0.151×0.109 m；预检 32 个候选中 18 个
因吸具与托盘相撞被排除，6 个终态全部通过。第一个终态通过的候选 11 得到完整四段计划：

| 段 | 规划器 | 发送缩放 | 发送点数 | 时长 |
|---|---|---|---|---|
| 接近 | Pilz PTP | ×1.25 | 8205 | 41.02 s |
| 接触 | Pilz LIN 0.25 倍 | ×2 | 682 | 3.41 s |
| 脱离（45° 方案） | Pilz LIN 0.25 倍 | ×2 | 1103 | 5.51 s |
| 带载返回 q_start | Pilz PTP | ×1.25 | 7929 | 39.64 s |

四段原生规划和最终发送路径碰撞检查均 PASS；接触 FK 6 杯密封、最大间隙 0、边缘余量 21.3 mm，被动杯唇未穿入。
承接区仍为 `RECEIVER_QUALIFICATION_DEFERRED_BY_USER`、支撑未确认。运动总时长约 90 s（另加真空等待）。

离线规划耗时约 22 min，其中大部分是每段原生输出的全模型稠密距离检查（接近段约 95 s）以及排在候选 11 前面、
已证实无解的候选消耗的 OMPL 预算；本机 CPU 同时被正在运行的工作台持续推理占用。这一耗时仍需继续优化。

**GPU 推理**：新建并行环境 `.venv-workbench-cuda`（torch 2.5.1+cu124，其余包与 `.venv-workbench` 完全一致，
原环境未改动）。RTX 3050 Ti 4 GB 上 DINO 约 0.8 s；SAM fp32 显存不足，fp16 约 0.32 s，掩码与现场 CPU 结果 IoU 0.9998；
同负载下 CPU 2 线程 SAM 约 20.7 s。现场配置仍为 CPU，尚未切换。

定向回归 **328 passed**，原生 optimized/reference 碰撞检查 PASS。实机冻结观测、规划、执行及抓放仍 **NOT_RUN**：
需要重启工作台加载新代码，重新准备观测、规划、预览，由操作者点击“执行单箱”。
报告见 `full_chain_replay_report.json` 和 `full_chain_pytest.xml`。

### 2026-11-13：现场检测切到远程 GPU 服务器

按用户要求，单箱配置的 `detector.backend` 改为 `remote_ssh`：Grounding DINO + SAM 在 GPU 服务器
（RTX 4090 49 GB）上运行，现场电脑只负责采集、几何恢复、场景、规划与执行。LiveVision 仍是唯一的模型持有者，
调度方式（当前帧 + 一个最新待处理帧）、冻结帧推理、同帧掩码绑定均不变；推理结果不授权任何运动或阀门命令。

- **连接**：SSH 密钥认证，严格校验主机密钥，不在服务器开放端口。主机、端口、用户、密钥写在被忽略的
  `outputs/workbench/remote_detector.local.json`，仓库只保存 Python、目录、设备与超时。
- **一致性**：本机和服务器共用 `hf_detector_core.py`（推理逻辑与原本地实现相同）。每次连接都上传核心与服务端脚本，
  服务器报告的两份代码 SHA-256 以及全部模型文件 SHA-256 必须与现场电脑一致，否则拒绝推理。
  图像以无损 PNG 上传并校验摘要，掩码以位压缩 npz 返回、禁止 pickle，形状、数量、数值逐项检查。
- **故障**：单次请求超过 `request_timeout_s`（20 s）、连接断开、服务端崩溃或结果格式错误时直接报错，
  检测窗口显示故障，“重试模型”会重新连接并重新校验；**不静默回退**到本机 CPU 或其他后端。
- **服务器环境**：`/root/eco65-gpu-infer-venv`，按 `tools/remote_detector/closure.txt` 离线 `--require-hashes` 安装，
  版本与现场 `.venv-workbench-cuda` 完全一致（torch 2.5.1+cu124、transformers 4.44.2）。部署步骤见 `tools/remote_detector/README.md`。

用保存的现场帧 bb88010f 自检（`tools/remote_detector/selftest.py`，**不是现场验收**）：连接、校验、加载 6.5 s；
之后每帧检测往返约 0.30 s、分割约 0.18 s（服务器内推理约 0.19 s / 0.11 s）。检测框与现场 CPU 结果相差 0.01 像素、
分数 0.4652 对 0.4656，掩码 IoU 1.0。此前现场 CPU 的同一任务感知约 65 s；切换后现场电脑 CPU 不再被持续推理占用，
对原生规划耗时也有利。

`tools/replay_single_carton_record.py --detector config` 可在保存记录上用当前配置的后端重新检测分割，再跑几何、场景和规划。

用远程 GPU 重新检测分割后，在同一保存记录上跑完整离线链路（`--detector config`，**不可执行、不是实机验收**）：
检测 + 分割约 1.1 s（含首帧预热），几何 + 场景约 20.8 s，规划约 17 min（上一轮本机 CPU 同时推理时约 22 min）。
恢复的箱体包络、候选编号（候选 11，6 杯密封）、四段规划器、发送缩放和时长都与上一轮相同，四段原生规划及最终发送路径检查全部 PASS，
运动总时长约 90 s。定向回归 **335 passed**。现场实际取景、规划、执行仍 **NOT_RUN**。
报告见 `gpu_detector_selftest.json`、`gpu_full_chain_replay_report.json` 和 `gpu_detector_pytest.xml`。

### 2026-11-13 晚：规划提速（A～D）

现场任务 `2814e675` 用远程 GPU 检测，感知约 20 s，但规划约 23 min 后 32 个候选全部被拒。逐项统计原生调用耗时，
慢在四处：OMPL 预算耗尽 46%、最终发送路径稠密检查 38%、Pilz 直连规划 14%，其余不到 2%。按用户要求做了以下四项，
**任何间隙、余量、检查密度、接触和密封规则都没有改**，原生规划程序也没有改：

- **A 原位包络先检**：规划前先检查箱体在原位时的完整碰撞包络。它与吸取候选无关，若与非支撑实测物体确定相交（深度 > 1 mm），
  或超出现场工作范围，立即报告 `TARGET_PAYLOAD_INTERSECTS_AT_REST` 和具体物体，不做任何路径规划。
  脱离终态或脱离起点的箱体与环境冲突超过逆解容差时，整个目标停止搜索（`TARGET_PAYLOAD_BLOCKED_AT_REST`）。
- **B 多起点逆解**：预检逆解除 q_start 和本次任务自己的解外，再用 64 个固定伪随机起点（每次运行相同），
  按与 q_start 的距离排序，最多检查 3 个不同分支。每个位姿只写一个记录文件，含全部起点和回复。
- **C 先规划、后稠密检查**：四段都规划出来，且起点、终点都合格之后，才按检查量从小到大做最终发送路径检查。
  预检得到接近终点关节角的候选，先从该关节角规划接触和脱离，再在两个进程上同时规划接近和带载返回。
- **D 并行检查**：最终发送路径按连续区段拆给主进程和 `native_check_workers`（默认 6）个常驻原生进程，相邻区段共享边界状态，
  每个状态仍被检查且只检查一次。辅助进程在后台启动，没就绪时只是并行度降低，不会降低检查要求。
  规划失败汇总区分阶段和失败类型，例如 `approach:NATIVE_SOLVER_BUDGET_EXHAUSTED` 与 `return:SENT_PATH_REJECTED`。

离线复核结果（**不可执行、不是实机验收**）：

| 记录 | 改前 | 改后 |
|---|---|---|
| `2814e675`（原位包络与实测物体相交 20.5 mm） | 规划约 23 min 后全部拒绝 | 0.14 s 直接给出原因和物体编号 |
| `bb88010f`（可行） | 规划约 17 min 得到完整计划 | 约 11.6 min 得到完整计划 |

bb88010f 这次多起点逆解找到 11 个终态全部通过的候选（改前 6 个），最终用候选 18（4 杯密封、45° 脱离），
四段发送路径检查共约 3 min（改前约 13.5 min），运动总时长约 80 s。剩余时间主要是 Pilz 规划器自带的输出检查
（每个 PTP 段约 1 min、单进程，合计约 4.9 min）和不可行候选耗尽 OMPL 预算（约 4.7 min）。
定向回归 **352 passed**（运行时工作区里还有另一会话未提交的执行侧改动）；报告见 `planning_speed_ad_report.json`。

### 2026-11-13 晚：多候选同时规划，去掉重复的规划器输出检查

按用户要求做两项改动，**间隙、余量、检查密度、接触与密封规则均不变**：

- **多候选同时规划**（`parallel_candidate_lanes`，默认 7）：候选按优先级轮流固定分配给原生进程（第 k 个给第 k mod N 个），
  每个进程按顺序只规划分给自己的候选。主线程严格按优先级逐个做发送路径检查，并占用全部进程；
  选中的仍是"优先级最高、规划和检查都通过"的候选，与逐个规划完全相同。
  OMPL 随机种子每个进程只设一次，结果取决于该进程处理过的请求顺序；固定分配保证每个进程的请求顺序可复现。
  检查期间暂停新的候选规划；脱离或返回段的发送路径被拒时，该候选回到原进程换下一个脱离方案重规划。
  进程出错或被取消后会关闭，不再分配请求；任何退出都会停止并等待全部规划线程结束。
- **去掉重复检查**：原生规划程序在规划成功后，会对自身返回的路点按关节线性插值做一次稠密间隙检查。
  这与我们随后对**实际发送的五次插值路径**做的检查是同一个函数（`clearance->checkPath`）、同一个 `output` 阶段和分辨率；
  发送路径包含全部返回路点且更密，所以前者是重复检查。新增请求字段 `defer_output_check_to_sent_path`：
  只有实机单箱范围（`PHYSICAL_SINGLE_CARTON`、strict 档、非 planning_only）才允许，其他范围直接报错
  `OUTPUT_CHECK_DEFERRAL_SCOPE_INVALID`；回复状态为 `DEFERRED_TO_CALLER_SENT_PATH_CHECK`。
  Python 只在规划程序声明能力 `single_carton_output_check_deferral` 时才发送这个字段，并只在发了字段时接受推迟状态；
  每段签发前必须有发送路径检查 PASS，否则 `SENT_PATH_CHECK_MISSING`。不带这个字段时，规划程序行为与原来完全相同，
  旧版工作台照常使用。`tools/check_output_check_deferral_scope.py` 用真实规划程序验证这一范围限制。

规划程序已重新编译并安装到 `install/moveit`，旧二进制备份在 `outputs/setup/worker-backup-20261113/`。

bb88010f 离线复核（**不可执行、不是实机验收**）：规划 695 s → **338～361 s**（两次独立运行）；选中的候选（18）、四段规划器、点数、时长、
失败统计与逐个规划完全相同。剩余时间约 51 s 预检、约 91 s 并行规划、约 196 s 发送路径检查（接近段 73 s、返回段 107 s）。
本机为 12 核 16 线程的 i5-12500H，发送路径检查已用 7 个进程。定向回归 **397 passed**（运行时工作区里也有另一会话未提交的执行侧改动），原生 optimized/reference 碰撞检查及组件检查 PASS。报告见 `parallel_planning_report.json`、`output_check_deferral_scope.json`。

### 2026-11-13 夜：用户确认的箱旁误检，先跑通全流程

现场任务 `1980356201804419a9e66e4063b9d732` 在规划前报 `TARGET_PAYLOAD_INTERSECTS_AT_REST`：箱体原位的完整包络与 4 个实测障碍相交（最大 18.7 mm）。
`tools/visualize_single_carton_rest_check.py <task_id>` 生成的诊断图显示：真正落在包络内的障碍点只有 72 个，
都在箱体左下角和右侧边缘、SAM 掩码外 2～5 像素、高出托盘面 0～10 mm（右上另有 6 个约 80 mm 高的点）。
场景把每个 4 cm 网格的点合成一个轴对齐方块，与下方托盘格子里的点连在一起，相交深度因此被放大。
（此前对同位置任务 2814e675 说"托盘隔条高出约 2 cm"是看了合成方块的范围，并非实测点本身。）

用户看图后确认：**箱子旁边的物体都属于误检，原因稍后再查，先把全流程跑通。** 据此新增显式现场配置：

- `target_envelope_points`：默认 `OBSTACLE`（原行为）。现场配置改为 `TARGET_USER_CONFIRMED`，必须填 `target_envelope_points_basis`
  （记录用户确认及任务号）和 `target_envelope_points_margin_m`（0～0.02 m，现填 0.008）。
  落在目标完整包络内、及向外 8 mm 以内的未归属实测点归为目标本身，不作障碍；**不向包络底面以下扩展**，箱子站着的托盘面仍是实测支撑。
  包络外的实测障碍、未重建邻箱、固定障碍、工作范围边界照常保留。`site-scene.json` 记录策略、依据和每个目标被归入的点数，
  并标注 `USER_FIELD_CONFIRMATION_FALSE_OBSTACLES_NOT_SENSOR_OBSERVATION`。8 mm 覆盖障碍方块的 3 mm 测量外扩与带载起点要求的 2.4 mm 间隙。
- 目标附近的障碍若仍与包络相交，按 1 cm 子格只包住各自的实测点重新生成（每个实测点仍被覆盖），不再用整格方块填满空角。

用该任务原始 RGB-D、掩码和 q_start 离线复核（**不可执行、不是实机验收**）：504 个点归为目标，原位包络检查通过；
规划 251 s 得到完整四段计划（候选 7、6 杯密封、45° 脱离），四段原生规划及发送路径检查均 PASS，运动约 80 s。
定向回归 **405 passed**；报告见 `false_obstacle_policy_report.json`。误检根因（掩码边缘、箱体侧边点归属）仍待排查。

### 2026-11-13 夜：控制器带载负载设为 3.5 kg

执行准备报“控制器负载未核对一致”：控制器当前工具为空载测试用的 `Empty3kg`（3.0 kg），单箱要求 3.5 kg（吸具 1.5 kg + 箱子 2 kg）。
用户授权一次性 SDK 写入。工作台关闭后运行 `tools/set_controller_payload_once.py --expected-tool Empty3kg --apply`：
复制当前工具（位姿、质心均为 0，保持不变）新建 `Load3p5kg`，负载 3.5 kg，并选为当前工具；`Arm_Tip`、`Empty3kg` 原样保留。
写入前后均停稳、`idle`、无报警，读回一致，**未发送任何运动指令**。记录：`outputs/controller_configuration/payload-3p5kg-20261113-222907/record.json`。
该脚本在其他进程连着控制器（如工作台）或占用反馈 UDP 端口时拒绝运行。
注意：空载受控入口仍按 3 kg 核对，使用它之前需在控制器中切回 `Empty3kg`。

### 2026-11-13 夜：按用户要求关闭执行前画面复核

现场任务 `6b648ffcdb35403cb9d992e2ed8554be` 规划成功（232 s，四段全部 PASS），点“执行单箱”后在执行前复核报
`TARGET_OR_SCENE_CHANGED_REPLAN`，未运动。用同一位置、场景未动、相隔数分钟的真实帧对比：全图约 41% 的像素深度变化超过 5 mm
（1 m 以外远处背景 72%～87%，为 D435i 远距离深度噪声；全场景要求 ≤1%），工作范围内也有 0.5%～0.75%，
最大连续变化区 131～319 像素（局部变化要求 <9）。即全场景比较在现场静止场景下必然失败。

用户要求取消执行前复核。新增单箱配置 `pre_execute_visual_recheck`：
- `ENABLED`（缺省，原行为）：停稳重新取景，全场景及目标局部比较。
- `TARGET_ONLY`：重新取景，核对会话、起点关节（0.01°）、相机身份及目标局部深度 / 颜色，跳过全场景比较。
  同样 4 组真实静止帧上目标局部比较均 PASS：深度变化 >4 mm 的像素 2.5%～2.9%（限 5%），颜色 0%。
- `DISABLED_BY_USER`（现场配置，依据记录本任务号与用户指令）：不重新取景。

后两者必须填写 `pre_execute_visual_recheck_basis`，事件记录 `pre_execute_recheck` 写明状态和依据。任何档位都保留：
任务记录可写、观测会话未失效、文件版本绑定，以及 `Cycle.start()` 的实时关节检查（数据 ≤0.1 s、已使能、无报警、与 q_start 相差 ≤0.01°）。
**关闭后，规划完成到执行之间如果箱子或场景被改动，程序无法发现，须由操作者确认目标与场景未变。**
启用复核时，新取的帧在比较前先保存为 `pre_execute_observation`，失败时也可复查。
定向回归 **422 passed**。

### 2026-11-13 夜：按用户要求取消起点关节核对

现场任务 `71ce26a3e9cf458e906b065bcb8275e7` 规划成功、画面复核已关闭，点执行后报 `START_CHANGED_REPLAN`，未运动。
原规则要求执行时实时关节与规划锁定的 q_start 相差 ≤0.01°；而机械臂静止时编码器读数本身在 0.001～0.006° 之间跳
（所有保存的停稳窗口共 1939 次非零变化，最大 0.006°），规划又需约 4 分钟，超限必然发生；机械臂并未被移动。

用户选择取消起点核对。新增单箱配置 `start_joint_check`：`ENABLED`（缺省，0.01°）或 `DISABLED_BY_USER`（须填 `start_joint_check_basis`）。
现场配置为后者，依据记录本任务号与用户指令。实现上：
- 规划时把这项设置绑定进**第一段**轨迹包（随包摘要签发，改动即失效），后续三段不带；
- `Cycle.start()` 每次都记录 `start_joint_check` 事件（实时关节、q_start、最大差值、是否限制）；
- 运动引擎只对包内带有 `START_JOINT_CHECK_DISABLED_BY_USER` 及依据的第一段放行起点差值；
  空载受控入口和未绑定的轨迹仍按 0.01°，后续段仍按 0.5° 到位续接规则。

始终保留：反馈 ≤0.1 s、已使能、无报警、关节速度 ≤0.5°/s（静止）；运动中跟踪误差、到位、反馈过期、停止监控不变。
**若规划后机械臂被移动过，第一段的第一个指令会从当前位置跳向规划起点，由操作者负责确认机械臂未动。**
定向回归 **436 passed**。

### 2026-11-16：实时化第 1 步，基线测量（只加计时，不改行为）

按 AGENTS.md 11-16 节第 7 条，本步只加计时。所有检查、阈值和判定都不变，也没有做任何优化。
报告见 `docs/evidence/realtime_20261116_baseline/report.json`。

**开关**：`ECO65_TIMING=1` 打开 Python 侧计时，同时给之后启动的原生 worker 设 `M710_NATIVE_TIMING=1`。两者默认都关闭。
关闭时，每个包装只执行原调用，不记录任何东西，记录文件、回复和判定都与改前相同。
打开时，低频分段先存在内存里；高频分段（监视 tick、RPC、管道、逐文件重哈希、Python GC 停顿）累加到固定大小的直方图。
每个阶段结束后，由后台线程写一次 `timing-<阶段>.json`（observe / prepare_plan / execute，执行子进程另写 `submit-timing.json`）。
原生回复里的 `native_timing` 在交给调用方之前就被去掉。热路径上没有同步写盘。时钟：Python 用 `time.monotonic`，C++ 用 `steady_clock`。

- **原生 worker**：按 init（模型、管线、策略校验）、world 比较与导入、场景 diff、Clearance 与工作区重建、IK、Pilz、OMPL、输出检查、
  validate、checkPath、请求解析和回复序列化分段计时。工作区按阶段累计粗筛（树节点数、AABB 测试数与跳过数）和 FCL 精确距离查询的次数与耗时。
  每次完整的单状态精确检查都计入对数直方图，按阶段和通过/拒绝分开；Python 侧再按 approach / contact / detach / return / loaded-start 标注。
- **规划**：原生调用往返与 JSON 大小、`context()` 中的 `digest(world)`、每次写盘（序列化和写入分开）、`binding()` 逐文件重哈希
  （文件大小与耗时）、`Authority.register` 的摘要、`complete-plan.json`、主 worker 与检查池的启动、预检、各候选通道、主线程等待与发送路径检查。
- **视觉**：freeze 的停稳等待与取帧配对、冻结帧排队、检测和 SAM（远程拆成 PNG 编码、往返中的服务器推理与传输协议、掩码解码）、
  逐箱 reconstruct、`build_scene` 各段（体素角点投影、逐体素自由空间循环、目标点归属、两次 `self_filter.contains`、体素列合并等），以及各次落盘。
- **调度**：execute 入口到首个点发出；每段 submit 的各步（binding 重哈希、authority claim 摘要、管道传输、
  `trajectory-package.json` 序列化与写入、控制器负载查询）；`Cycle.tick` 每次的 RPC 次数与耗时；段间交接拆成规则要求的停留和其余部分。
- **新工具**：`tools/measure_scheduling_fake_io.py` 使用真实的 `Cycle`、真实启动的执行子进程（Supervisor、Authority、记录线程）和真实的 `binding()`，
  只把 SDK 传输和真空串口换成内存 fake（`tools/fake_io_transport.py`）。`tools/summarize_eco65_timing.py` 负责汇总各计时文件。

**测量干扰（已修正）**：第一版把每个分段都保存成对象，计时打开时 5 次 fake IO 周期里有 4 次在 CONTACT 段报
`LIVE_ROBOT_STALE_DISCONNECTED_OR_FAULT` 停止。原因是持有 68 MB 源快照的进程里 gen-2 GC 停顿约 0.12 s，超过了原有的 0.1 s 反馈年龄规则。
这是 fake IO 上的安全停止，没有涉及硬件。改为固定大小的聚合后，计时开和关各 5 次全部完成，结果为 `FAKE_DEVICE_CYCLE_COMPLETE`。

**行为零变化**（远程 GPU 服务器，现有 Humble chroot 与 `/root/v05-gpu-venv`）：
- 原生检查：`m710_clearance_test optimized` 和 `reference` 改前改后都 PASS；去掉计时字段后，两次输出完全一致。
- 原生组件检查：`tools/check_eco65_single_carton_native.py` 改后 PASS。改前代码加改前二进制，与改后计时关、改后计时开相比，5 个记录文件逐项一致。
- 定向 pytest：改前、改后计时关、改后计时开都是 **435 passed，1 failed，0 skipped**。
  失败的是 `test_exact_prior_face_keeps_a_bounded_envelope_that_covers_the_prior`，在本服务器的改前代码上就同样失败（`recover_partial` 返回 140×186×108 mm，数值库版本与现场 `.venv` 不同），不是本步引入的，也没有修改。
- pytest 和组件逐项对比跑在倒数第二个修订上。最后一个修订只改了计时打开时的高频分段聚合和 GC 停顿记录，计时关闭时的路径与原代码等价。
  按用户要求，最后一个修订没有重跑全量检查，只用 10 次 fake IO 周期验证过。

**三个口径的基线**：
- **规划**（`prepare_plan` 入口到 `PLAN_READY`）：**NOT_RUN**，原因是仓库、本机和服务器上都没有保存的现场记录（见下文）。
  报告中给出了 232–361 s 分解与各字段的对应关系：预检对应 `search.screen_candidate`，并行规划对应 `main.wait_planned_candidate` 和
  `lane.plan_candidate`，发送路径检查对应 `sent_check` 以及按段的 `<stage>:output` 单状态直方图。
  合成组件检查（一次小 PTP 加 491 个发送状态，**不是** prepare_plan）：单状态精确检查均值约 2.1 ms，p99 约 2.8 ms；
  validate 请求约 0.33 MB，往返 3.51 s，其中 3.08 s 是 checkPath。
- **视觉**（observe 入口到 `TARGET_READY`）：**NOT_RUN**，原因是没有冻结帧、没有相机，远程检测也未连接。插桩已完成。
- **调度**（fake IO，合成 1.5 s 段，5 次）：execute 入口到首个点发出 **3.25–3.88 s**。
  每次 `binding()` 重哈希 807 个文件约 1.86 s，其中检测模型两个目录（1.31 GB）约 1.67 s。执行路径每个周期调用 5 次
  （execute 入口 1 次，每段 submit 各 1 次），所以第一段要等两次。每段从状态进入到控制器受理约 1.9 s，几乎都是这次重哈希，期间 Cycle 监视不 tick
  （执行进程仍在监控运动）。规则要求的停留另算：到位停留 0.4 s，压力停留 1 s 加上等待第一条合格心跳的时间。
  子进程侧 `trajectory-package.json` 序列化 0.04–0.06 s，管道约 5 ms。每次 tick 有 3 次 RPC，通常每次 <1 ms。
- **原有风险（未修）**：计时关闭时，持有快照的进程 gen-2 GC 停顿仍有 0.04–0.135 s。若恰好落在 `Cycle.tick` 校验 0.1 s 反馈年龄之前，会触发安全停止。

**保存记录**：仓库的 `outputs/` 被忽略，没有任何任务目录；`docs/evidence/` 下只有报告，没有 `observation.npz` 或 `segmentation.npz`。
本机和 GPU 服务器上也都没有。文档提到的 bb88010f、6b648ffc、19803562、71ce26a3、4cde3b86、2814e675、7ccdb15e 均不在这里，
因此同一记录回放的改前改后对比 **NOT_RUN**。用户 2026-11-16 确认：现场电脑上的任务记录和现场本地配置文件都已删除。
记录是原始传感器数据，不重建；这些任务的回放对比和 232–361 s 分解核对因此无法再做。
回放所需的配置可以从仓库取得：`single_carton.example.json` 和 `real_site.example.json` 已含文档记录的现场设定，
`tests/fixtures/eco65_installation_cad_reference.json` 是经用户同意保存的现场 `installation.local.json` 原字节副本（本次组件检查即用它）；
现场未写入仓库的改动已无法找回。

**下一次现场基线的做法**：用 `ECO65_TIMING=1` 启动单箱工作台，只做“准备观测”和“规划所选箱”（规划不发运动或阀门命令），
每个任务目录会生成在现场电脑上测得的 `timing-observe.json` 和 `timing-prepare_plan.json`。计时打开的会话里不要点“执行单箱”
（计时与实机执行同时打开的安全性尚未建立），执行前先关闭计时重启。保留任务目录并上传，即可在同一记录上做 0783e86 与当前提交的
改前改后回放，并补齐规划和视觉基线。

**环境**：没有改内核，也没有动系统 ROS / MoveIt。在现有的 `/root/v05-gpu-venv` 里离线新增了 7 个包，没有升级任何已有包，
新增包的版本和 wheel SHA-256 记在报告 `environment` 中（pytest 8.4.2、pyserial 3.5 及其依赖）。
Python 跑在主机上，原生 worker 通过仓库外的测试用 `env` 包装在现有 chroot 中启动。所有数字都来自共享 GPU 服务器（负载约 20），
不是现场 i5-12500H。

**未做或未验证**：现场记录回放、规划与视觉基线及成功率、真实场景下按 approach/contact/detach/return 分段的单状态分布、
现场电脑上的耗时、真实 SDK/UDP/串口心跳节奏与控制器负载查询延迟、计时打开时的实机执行（安全性未建立），以及任何实机运动、阀门命令或验收，
均为 **NOT_RUN**。

### 2026-11-16：实时化第 2 步，量具与统一截止时间

按 AGENTS.md 11-16 节 §3d 实现，未发现冲突。报告见 `docs/evidence/realtime_20261116_deadline/report.json`。

**量具** `tools/bench_planning_latency.py`：
- `--discover` 用 `ECO65_TIMING=1` 部分回放 `outputs/single_carton/` 和 `outputs/realtime-field-*/` 下完整输入；也可用 `--discover-root DIR` 指定发现根。完整入口基线使用 `--timing` 读已有 `timing-*.json`，回放不等于 prepare_plan 入口计时。
- 按规划、视觉、调度三个口径分别给出 p50 / p95 / max（最近秩，不插值）、成功率、逐环节分解，以及超出预算的环节。
- 没有输入时报 `NO_SAMPLES`，不编数字。不连接设备，不改记录。

**预算表** `configs/workbench/planning_budget.json`：
- 规划总预算 2.0 s，分成 9 个环节，每个环节标明由哪些计时分段测量。
- 状态为 `ALLOCATION_AND_REPORTING_ONLY_NOT_ENFORCED`，只用于分配和报告，不强制执行。
- 视觉、调度口径只测量，不设目标。没有现场基线，第一次现场计时后需要修订。

**统一截止时间**：单箱配置新增可选项 `planning_deadline_s`（0 < s ≤ 3600）。**不填时，所有请求、超时和收尾都与原来完全相同**；示例配置不填。
填写后：
- `prepare_plan` 入口生成绝对截止时间（`time.monotonic()`），传给 `plan_cycle`。
- 每个原生请求的 `request_budget_s` 取原值与剩余时间（减去 0.1 s 回复余量）中较小的一个；同时附带 `absolute_deadline_monotonic_s` 和时钟参考。
- Pilz / OMPL 的 25 s 求解时间、IK 的 110 s，以及 120 / 180 / 240 / 600 s 的 IPC 和检查池等待，都由截止时间截短。
- worker 只在 `PHYSICAL_SINGLE_CARTON` 范围接受绝对截止时间，用它收紧本请求的 `NativeDeadline`。
  时钟参考晚于收到时刻，或截止时间已过 60 s 以上，按 `DEADLINE_CLOCK_DOMAIN_MISMATCH` 拒绝（两边都是 Linux CLOCK_MONOTONIC）。
- 到期时调用方立即返回。没有回复的 worker 不再分配请求，它的回复在后台收走。
- `search_box` 在辅助线程运行，所以不可抢占的 Python 步骤不会拖住返回；辅助线程到期后不再发出请求，也不签发许可。
- 截止时间模式下，worker 在后台关闭；上一次规划遗留的检查池 worker 不会进入下一次的检查池。

**归类**：到期抛出 `PlanningBudgetExhausted`。它不是 `ValueError`，候选和目标的失败处理都不会把它记成不可行；
它也不同于 IPC 失败。worker 返回的 `NATIVE_ABSOLUTE_DEADLINE_EXPIRED`，以及由截止时间截短后出现的 `NATIVE_SOLVER_BUDGET_EXHAUSTED`，都归为预算耗尽。
原有 25 s 单次上限造成的求解耗尽，仍按原来那样记为该候选的结论。
`planning-failures.json` 改为 `{failures, planning_budget}`，逐目标给出状态、候选数、已搜索（已有结论）和未搜索数。**未搜索的不算不可行。**

**未设置时行为不变**：
- `m710_clearance_test optimized` 和 `reference` 都 PASS。
- 原生组件检查 PASS。与第 1 步改前的记录相比，只有 `config.py` 的绑定哈希和提交号不同（本步改了 `config.py`），状态、关节值、点数、检查结果都一致。
- 合成场景 search_box（候选 0、1、2，7 条车道）：改前代码加第 1 步 worker，与本步代码加新 worker 相比，24 个记录文件逐项一致。
- 定向 pytest **465 passed，1 failed，0 skipped**。失败的仍是那一项原有失败；比第 1 步多出的 30 项是本步新增的测试。

**设置很小的值**：
- fake worker：24 项测试连续 3 次全部通过。慢原生请求（0.15 / 0.3 / 0.6 s）、无回复 worker、并行车道，都在截止时间 + 50 ms 内返回，归类和计数正确。
- 真实 worker、合成场景，截止时间取 0.5 – 30 s 共 16 个值：15 个在 +27 ms 内返回。30 s 那次晚了 **122 ms**：
  7 条车道被截短的 OMPL 求解同时在截止前 0.1 s 结束，各自序列化约 270 kB 的记录（`json.dumps` 持有 GIL），和主线程的归类争用 GIL。
- 另测到 `contact_candidates`（约 1 s 纯 Python）里的 numpy 调用最长占用 GIL 83 ms；截止时间落在这一步时，返回会相应延后。
  这两处都没有为了满足 50 ms 去改候选生成或记录内容；第 4 步的异步证据和第 7 步的复合原生请求会把它们移出截止路径。
- 第一次运行出现过晚 0.965 s：上一次规划的检查池 worker 在下一次规划期间才完成初始化，持有池锁时关闭（5 s）。截止时间路径上已修复。

**量具结果**：规划和视觉都是 `NO_SAMPLES`，因为现场记录已全部删除。调度（第 1 步的 5 次 fake IO）：p50 3.83 s，p95 3.88 s，成功率 5/5。

**测到的其他问题（未修，留给第 4 步）**：
- worker 不响应 SIGTERM，每次关闭要等 5 s 才 SIGKILL。不设截止时间时，`plan_cycle` 在 `PLAN_READY` 之前逐个关闭主 worker 和检查池，
  按示例配置约 5 s × 7 = 35 s。
- worker 冷启动 3.8 s；不设截止时间时，检查池存在上一次 worker 进入下一次检查池的竞争。
- 找到完整计划后的收尾（binding 重哈希约 1.86 s、签发、写记录）不受截止时间中断，耗时记入预算表的 finalize 环节。

**未做或未验证**：现场记录上的量具结果、规划与视觉基线；真实现场场景下的截止时间行为；2.0 s 截止时间下的成功率
（worker 冷启动就要 3.8 s，第 4–8 步之前没有意义）；现场电脑上的耗时；任何实机运动、阀门命令或验收，均为 **NOT_RUN**。

### 2026-11-16：实时化第 3 步，接口冻结

只写文档，不改代码。`docs/native_boundary.md` 给出五个边界的契约：A 规划 worker、B 轨迹准备、C 现场场景构建、D 执行进程、E 连续作业层。
每份契约列出字段与单位、错误码、取消与截止时间语义、线程与重入、Python 保留部分，以及按 AGENTS.md 不得改变的字段和判据；第 F 节把每组字段对应到代码中的文件和函数。

跨语言通道只选一种：现有 worker 管道上的二进制帧，大块数据走共享内存；不用 pybind11（保留进程隔离、按进程并行，且开发服务器上 worker 在 chroot 中运行）。旧 JSONL 协议保留为 PYTHON 实现的通道。
模块开关 `vision_module`、`planning_module`、`scheduling_module` 取 PYTHON | NATIVE，缺省 PYTHON；机制开关（`resident_workers`、`scene_handles`、`async_evidence`、`path_endpoint_reuse`、`binding_hash_cache`、`collision_check_mode`、`sent_path_audit`、`planning_deadline_s`）缺省都保持现行为。

报告见 `docs/evidence/realtime_20261116_boundary/report.json`。本步无代码改动，测试未运行；共享内存传输只写入契约、尚未实现；契约 E 对应第 13 步，代码尚不存在。

### 2026-11-16：实时化第 4 步，低风险五项

按 `docs/native_boundary.md` 契约 A 实现五项机制，各有一个开关，**缺省全部关闭，保持现行为**；示例配置都不打开。报告见 `docs/evidence/realtime_20261116_low_risk/report.json`。

1. `resident_workers`：入口启动后在后台拉起并初始化主 worker 和检查池，任务之间不关闭。每个 worker 在初始化后和每个任务结束后都执行 `sc_task_reset`：清空世界物体、场景句柄、计数器和任务会话，重建 OMPL 管线并重新设置固定随机种子。取消和超时改为按请求编号生效：worker 的读取线程把正在运行请求的 `NativeDeadline` 置为到期，worker 回复后进程继续可用。5 s 内不回复、初始化或重置失败的 worker 退役，不再分配请求，并在后台补充。
2. `scene_handles`：每次观测只对 world 做一次 digest。每个 worker 上传一次（`sc_scene_upload`），之后的请求只带句柄；worker 把保存的 world 放回请求，各项策略看到的内容不变。
3. `async_evidence`：请求仍在发出前同步落盘。回复、分段与发送路径证据、observation 的 npz、`site-scene.json`、`geometry.json`、`complete-plan.json` 由一个按顺序写入的后台线程完成。`plan_cycle` 返回前先 flush（`PLAN_READY` 必须在计划证据落盘之后），execute 前再 flush 一次。任何写入失败都会拒绝 `PLAN_READY` 和执行。
4. `path_endpoint_reuse`：`checkPath` 原本逐边采样并包含两个端点，相邻两边共享的端点会被检查两次。打开后，与前一个已通过的采样点逐位相同的点不再重复检查；被检查的状态集合和每条边的采样规则不变，与空载入口已有的规则相同。只在现场单箱范围生效。
5. `binding_hash_cache`：文件的设备号、inode、大小、mtime_ns、ctime_ns 都没变时，复用上次的内容哈希；有任何变化就完整重算；0.1 s 内刚修改过的文件始终重算（时间戳精度）。计划和执行日志逐文件标注 REUSED / REHASHED。

**验收**：
- 合成可达场景 A、B（同一箱体，两个 q_start），真实 worker，生成完整四段计划。现场记录已删除，用这两个场景代替。
- 开关全开时，候选编号、四段规划器、发送点数、时长、检查状态数、脱离方案、首末关节和每条失败原因，都与全关时逐项一致：A 选中候选 0，B 选中候选 4。
- 同一驻留 planner 连续规划 A 两次，结果逐项一致；先规划 A 再规划 B，与新开 planner 单独规划 B 的结果逐项一致，说明任务间没有残留、随机种子已重置。
- `m710_clearance_test optimized` 和 `reference` 都 PASS。原生组件检查 PASS，与第 1 步相比，只有本步改动的源文件哈希和提交号不同。
- 定向 pytest **501 passed，1 failed，0 skipped**。失败的仍是那一项原有失败；新增 36 项测试。

**三个口径的改前改后**：
- 规划（合成场景，`plan_cycle` 总时长）：A 129.8 s → 116.6 s，B 166.8 s → 136.8 s。
- 调度（fake IO，`binding_hash_cache`）：点击执行到首点发出 3.25–3.88 s → **0.12–0.15 s**；每段从状态进入到控制器受理约 1.9 s → 0.06–0.17 s。
- 视觉：无冻结帧，**NOT_RUN**。

**提交**：五项各一个提交。共享骨架放在最先用到它的那个提交里：开关校验和计划字段在 hash cache 提交；`plan_cycle` 的 worker 生命周期重构在 async evidence 提交；场景上传和句柄展开的原生代码与任务重置在同一代码块里，放在 resident 提交，由 scene handles 提交启用。中间提交只做了 Python 编译检查，没有逐个做原生构建。测试、工具、报告和本节放在第六个提交。

**未做或未验证**：现场记录上的回放和规划 p95；视觉耗时；完整工作台 UI 下的 `resident_workers`（只验证了 `plan_cycle` 和 `TaskManager` 的启动钩子）；实机运动、阀门命令或验收，均为 **NOT_RUN**。

### 2026-11-16：实时化第 5 步，两级碰撞检查

新增开关 `collision_check_mode`，取值 `EXACT` 或 `TWO_LEVEL`，**缺省 `EXACT`，保持现行为**；示例配置不改。`TWO_LEVEL` 只对现场单箱请求生效，其他范围返回 `COLLISION_CHECK_MODE_SCOPE_INVALID`。worker 没有 `two_level_collision` 能力时，planner 在发出任何请求之前就拒绝。报告见 `docs/evidence/realtime_20261116_two_level/report.json`。

1. **代理层**：只用来排除物体对，从不拒绝。包括：
   - 每个物体的包围球：以局部 AABB 中心为球心，取凸体各顶点到球心的最大距离为半径；非凸形状取 AABB 半对角线；再按相对 1e-9 加绝对 1e-9 m 放大。
   - 刚体组：由固定关节连起来的物体，用一个组球。
   - 静态物体的层次包围。
   - 相对位姿不变的物体对，每个场景只判一次。

   代理距离必须大于该对的限值加 1 mm（search 阶段还要加带载余量），才排除这一对。
2. **精确层**：没被代理排除的物体对，按原来的物体对编号顺序，走不变的保守 AABB 检查和精确 `fcl::distance`，限值相同。所以第一个失败的物体对和失败原因与 `EXACT` 完全相同。
3. 机器人凸体几何按进程缓存；碰撞变换没有更新标记时，不再复制 `RobotState`。

**验收**：
- `m710_clearance_test optimized`、`reference` 都 PASS。新增的 `two_level` 模式也 PASS：3 种布局的 4500 个状态（1831 个有效，2669 个无效）上判定逐个相同；300 个分离位姿上，包围球给出的都是保守下界。
- 新工具 `tools/check_collision_equivalence.py` 把第 4 步全关合成计划 A、B 的全部 validate 请求重新发给 worker，共 316 条记录，用 `EXACT` 和 `TWO_LEVEL` 各检查一遍。检查的状态包括：
  - 原有的端点和发送路径探测状态；
  - 每个状态一个 ±0.02 rad 的扰动副本；
  - 每条记录 200 个均匀随机状态。

  共 520,340 次检查：**反例（`TWO_LEVEL` 通过而 `EXACT` 失败）0，多拒 0，首个失败差异 0**。
- 合成场景 A、B 用 `TWO_LEVEL` 重新规划，候选、四段规划器、点数、时长、检查状态数、首末关节和每条失败原因都与第 4 步的 `EXACT` 结果逐项一致：A 选中候选 0，B 选中候选 4。
- 原生组件检查 PASS。与第 1 步相比，只多了新的证据计数（`collision_check_mode`、代理与精确计数），以及源文件哈希和提交号不同。
- 定向 pytest **510 passed，1 failed，0 skipped**。失败的仍是原有那一项；新增 9 项测试。

**耗时（发送路径 output 阶段，单状态中位数，`EXACT` → `TWO_LEVEL`）**：

| 阶段 | 中位数 |
|---|---|
| 接近 | 1.90 → 0.55 ms |
| 接触 | 1.57 → **0.46 ms** |
| 脱离 | 1.59 → **0.49 ms** |
| 返回 | 1.62 → 0.49 ms |

- 接触和脱离 ≤ 0.5 ms 的目标达到。
- 精确 FCL 只占物体对检查的约 1e-5 到 2e-4；代理层排除约 99.7% 到 99.9% 的物体对。
- 需要至少一次精确计算的状态比例因阶段而异：接触 output 为 1.0，返回 output 为 0.19。
- 合成场景规划总时长：A 129.8 s → 66.6 s，B 166.8 s → 81.0 s。各只跑了一次，不是 p95。
- 调度未改动；视觉没有冻结帧，**NOT_RUN**。

**未做或未验证**：
- 现场记录上的等价检查和规划 p95：现场记录已删除，改用合成场景的记录和随机状态。
- 视觉耗时。
- 任何示例配置都没有打开 `TWO_LEVEL`。
- 实机运动、阀门命令或验收，均为 **NOT_RUN**。

### 2026-11-16：实时化第 6 步，发送路径按距离自适应审计

新增开关 `sent_path_audit`，取值 `SAMPLED` 或 `ADAPTIVE`，**缺省 `SAMPLED`，保持现行为**；示例配置不改。推导和适用范围见 `docs/sent_path_adaptive_audit.md`，报告见 `docs/evidence/realtime_20261116_adaptive_audit/report.json`。

**做法**：ADAPTIVE 走与 SAMPLED **完全相同的采样点序列**。
- 每个精确检查通过的采样点作为锚点，计算一组约束：
  - 每对运动刚体组一个；
  - 每个运动组对全部静止物体一个；
  - 每个连杆包围盒和载荷角点对现场工作范围一个。
- 余量取 `距离下界 − 物体对限值`，截断在 0.05 m。距离下界取包围球距离、保守 AABB 距离、本点精确 FCL 距离三者中的最大值。系数是各关节到该组最远点的力臂上界。
- 之后的采样点，只有当所有约束在该点都满足 `余量 − Σ系数·|Δq| ≥ 1 µm` 时才跳过；其余采样点一律做精确检查。

所以凡是 SAMPLED 会拒绝的采样点，ADAPTIVE 不可能跳过，判定与首个失败都和 SAMPLED 相同。

适用范围按条件判断，不按阶段名：没有接触许可、也没有支撑物的段才用 ADAPTIVE。接触段和带支撑段仍用 SAMPLED，worker 对这类请求返回 `SENT_PATH_AUDIT_SCOPE_INVALID`。合成场景中箱体下方没有支撑物，所以脱离段也符合条件。

每段证据记录方法、采样点数、精确检查数、跳过数和最小距离下界。`continuous_swept_proof` 按实际保证如实填写：通过时为 `BOUNDED_DISPLACEMENT_1_4_MM`，SAMPLED 与 ADAPTIVE 相同；拒绝时为 null。

**验收**：
- `m710_clearance_test adaptive` PASS：
  - 12000 个物体位移都不超过力臂上界；
  - 10342 个被证明可跳过的采样点全部通过精确检查；
  - 600 条随机路径，判定和首个失败逐项相同，检查数从 4,018,259 降到 29,641；
  - 擦碰和穿入的柱子，两种方法在同一采样点拒绝。
- `optimized`、`reference`、`two_level` 仍 PASS。
- 保存记录逐段复核：用 `tools/check_adaptive_audit.py` 把第 4 步合成计划 A、B 中所有符合条件的发送路径请求，各用 SAMPLED 和 ADAPTIVE 发一次。变体包括原样、加入带种子的随机障碍、宽松和收紧的工作范围。
  - **EXACT**：228 条全部一致；116 条 ADAPTIVE 通过，用 SAMPLED 复核也全部通过。
  - **TWO_LEVEL**：152 条全部一致。
- 人为构造的障碍：沿工具轴二分到间隙阈值。
  - 再近 0.5 mm（擦碰，杯唇对障碍 2.398 mm < 2.4 mm）和再近 12 mm（穿入），两种方法都在同一采样点拒绝；
  - 再远 2 mm 时两种方法都通过。
- 合成场景重新规划，候选、四段规划器、点数、时长、检查状态数、首末关节和失败原因都与 SAMPLED 逐项一致（A 选中候选 0，B 选中候选 4）：
  - ADAPTIVE + EXACT：A 129.8 s → 108.5 s，B 166.8 s → 105.0 s；
  - ADAPTIVE + TWO_LEVEL：A 62.5 s，B 65.7 s。
- 原生组件检查 PASS。与第 1 步相比，只有新增证据计数、源文件哈希和提交号不同。
- 定向 pytest **526 passed，1 failed，0 skipped**。失败的仍是原有那一项。

**检查次数与耗时（EXACT，原样记录，SAMPLED → ADAPTIVE）**：

| 阶段 | 精确检查数 | 检查耗时 | 请求总时长 |
|---|---|---|---|
| 接近（14 条） | 91,236 → 131 | 132.5 s → 0.25 s | 141.0 s → 6.2 s |
| 脱离（14 条） | 131,082 → 89 | 188.8 s → 0.17 s | 198.3 s → 6.3 s |
| 返回（10 条） | 59,445 → 49 | 85.1 s → 0.09 s | 90.8 s → 4.3 s |

剩下的请求时长主要是每个 validate 请求重新导入场景、构建碰撞工作区的固定开销，每个请求约 0.3–0.4 s。

**量具（规划口径，合成场景）**：
- 全部机制打开（第 4 步五项、TWO_LEVEL、ADAPTIVE），4 次 `plan_cycle`：p50 76.7 s，p95 105.2 s，**离 2.0 s 还差约 103 s**。
- 差在以下环节（p95）：
  - 终态预检 40.9 s：每次规划约 2500 次原生往返，主要是多种子 IK；
  - 候选规划 21.6 s：其中 Python `prepare` 每次约 0.5 s，累计 38–55 s 车道时间；
  - 发送路径检查 10.6 s：主要是每个请求的固定开销；
  - 原位检查与候选生成 1.6 s，收尾 1.4 s。
- 驻留 planner 背靠背规划第二个任务时，要等上一个任务的 worker 重置，多花 25–28 s。现场两次任务之间有执行时间，通常能覆盖。
- 视觉和调度未测（本步未改动）。

**未做或未验证**：
- 现场记录上的复核与规划 p95：现场记录已删除，改用合成记录、随机障碍和构造障碍。
- 任何示例配置都没有打开 ADAPTIVE。
- 在截止时间下，ADAPTIVE 可能在 SAMPLED 会超时的地方完成。这属于预算差异，不是几何判定差异。
- 实机运动、阀门命令或验收，均为 **NOT_RUN**。

### 2026-11-17：实时化第 7 步，复合请求、二进制数据面、prepare 下沉

新增开关 `planning_module`，取值 `PYTHON` 或 `NATIVE`，**缺省 `PYTHON`，旧协议和旧流程不变**；示例配置不改。worker 没有声明 `composite_requests`、`native_prepare`、`attempt_journal` 能力时，NATIVE 在发出任何请求之前就被拒绝。契约见 `docs/native_boundary.md`，报告见 `docs/evidence/realtime_20261116_native_planning/report.json`。

**做法**：
1. **prepare 下沉**：`native_prepare.h` 在 worker 中实现五次插值、位置 / 速度 / 加速度极值检查、5 ms 发送网格和检查路径，按 scipy / numpy 的运算顺序逐步计算。另有单独的 `sc_prepare` 操作。`SEND_SLOWDOWN_STEPS` 的逐级放慢规则不变。
2. **复合请求**（只在 `PHYSICAL_SINGLE_CARTON` 范围；不复用仿真链路的 `eco65_transition` 和 `process_policy` 契约）：
   - `sc_endpoint_screen`：一个目标的全部候选一次完成。内容包括带载起点校验、多种子 IK（本任务解及 J6 四分之一转变体，加 64 个固定种子）、最多 3 个分支的接近 / 接触 / 脱离终态校验，以及工具或载荷被挡、目标级被挡的判定。
   - `sc_plan_candidate`：每个候选分两段。
     - `CONTACT`：接近（终态预检已给出接近位形时跳过），接触直线及 0.5 / 0.25 / 0.1 降速重试。
     - `DETACH_AND_VERIFY`：三种脱离方案依次尝试（含降速重试和目标被挡判定）、接近、带载返回（先直连再 OMPL）、prepare、未检查段按路径长短依次做发送路径检查，被拒时换下一种脱离方案。
   - 两段之间的密封、附着、承接和脱离目标由 Python 计算。
3. **结论来源**：worker 记录每个子请求和结果；Python 用回放客户端运行**原样不变**的 `screen_candidate`、`plan_candidate`。
   - Python 发出的每个请求都必须与记录一致，否则报 `NATIVE_FLOW_DIVERGED`，不回退。
   - TargetBlocked 判定、CLEAR 候选优先、"按优先级第一个规划和检查都通过的候选" 都由原代码给出。
4. **并行**：每个 worker 一条车道，候选按固定轮转分配，按优先级收结果，首个通过即停止其余车道。
5. **数据面**：NATIVE 客户端在 init 确认 `binary_protocol=2` 后改用二进制帧，轨迹和路径以 float64 块传输，不再以 JSON 传递。
   - 取消帧为 kind 4。
   - `Authority` 在签发和核对两端用同一函数，对规范二进制形式计算 SHA-256。
   - 共享内存未实现。
6. **证据**：复合请求在发出前落盘。worker 把每个子请求（IK、Pilz、OMPL、校验）的开始和结束追加写入自己的日志 `$M710_WORKER_JOURNAL_DIR/m710_worker_journal_<pid>.jsonl`，超时或崩溃后仍可查。

**验收**：
- 合成场景 A、B，新旧两条流程的候选、四段规划器、发送点数、时长、检查状态数、脱离方案、首末关节和每条失败原因，在 1e-12 内逐项一致（A 选中候选 0，B 选中候选 4）。两种配置都验证了：
  - 缺省（EXACT + SAMPLED）；
  - TWO_LEVEL + ADAPTIVE。
- prepare 逐点比较：60 个原生规划结果，每个都在 5 级放慢和两种压缩时间轴下比较，另加构造的错误输入，共 424 例。
  - 越限拒绝 292 例，错误串全部一致。
  - 准备成功的 132 例，点数、细分数、检查状态数全部相同。
  - **q 与检查路径最大相差 3.7e-14**，满足 1e-12。
  - **v 最大相差 5.4e-12，a 最大相差 8.4e-10，未达到 1e-12**。原因：scipy 在数组上计算 `dx**s`，这台 AVX-512 服务器上 numpy 的 SIMD `pow` 与 libm 约 27% 的值相差 1 ulp（已实测），Python 自身的系数因此随 CPU 变化；幂基导数求值中的相消把这 1 ulp 放大到 v、a。Python 的 prepare 没有改动。
- 二进制帧与 JSON 行：同一条 1094 状态的发送路径请求，两种格式的回复除计时字段外完全相同。驻留 worker 的二进制取消帧在 0.25 s 内以 `cancelled_by_request` 返回，worker 之后仍可用。
- `tools/check_output_check_deferral_scope.py` PASS。
- `m710_clearance_test` 的 optimized、reference、two_level、adaptive 都 PASS；原生组件检查 PASS。
- 定向 pytest **540 passed，1 failed，0 skipped**。失败的仍是原有那一项；新增 14 项测试。

**往返次数与传输字节（合成场景，每次 `plan_cycle`）**：

| 配置 | 场景 | 往返次数 | 发送字节 | 接收字节 | 时长 |
|---|---|---|---|---|---|
| 缺省 | A | 2535 → 23 | 136.5 MB → 94.0 MB | 2.7 MB → 30.2 MB | 126.0 s → 238.0 s |
| 缺省 | B | 2595 → 31 | 158.7 MB → 100.6 MB | 3.2 MB → 42.1 MB | 150.7 s → 386.2 s |
| TWO_LEVEL + ADAPTIVE | A | 2530 → 23 | 135.2 MB → 94.0 MB | 2.5 MB → 76.7 MB | 51.8 s → 47.6 s |
| TWO_LEVEL + ADAPTIVE | B | 2601 → 33 | 160.4 MB → 102.2 MB | 3.7 MB → 109.4 MB | 75.5 s → 66.1 s |

接收字节变多，是因为复合请求的回复里带了每个子请求的完整结果（含规划点），供 Python 回放。

**三个口径**：
- **规划**：
  - TWO_LEVEL + ADAPTIVE 下 NATIVE 比 PYTHON 快 8–12%。
  - **缺省配置下 NATIVE 反而慢约一倍**。原因是发送路径检查：PYTHON 把每段分给 6 个检查 worker 并行；NATIVE 在每条车道的单个 worker 内依次做全部分块，而 7 条车道同时运行，抢占 CPU。排名 1 的候选，脱离段复合请求 165.5 s，其中约 90% 是发送路径检查。开启 ADAPTIVE 后检查只需毫秒级，这一差距消失。
  - 终态预检复合请求：45.5 s（缺省）、14.7 s（TWO_LEVEL）。其中 2305 次 IK 只占约 1 s，其余是 132 次逐状态校验，每次都重建场景和碰撞工作区。
  - 离 2.0 s 仍远。
- 调度：除包摘要在 NATIVE 下改为规范二进制外未改动。
- 视觉：**NOT_RUN**。

**提交**：三个提交，分别是 prepare 下沉、二进制数据面、复合请求与 `planning_module`；测试、工具、报告和本节放在最后一个提交中。中间提交只做了 Python 编译检查，原生构建与全部检查在最终状态上进行。

**未做或未验证**：
- 现场记录上的等价检查：现场记录已删除，改用合成场景的记录。
- v、a 的 1e-12：未达到，原因见上。
- 共享内存。
- 单车道时，NATIVE 把接近和返回放在同一个 worker 上依次规划，PYTHON 顺序流程用两个 worker；若需要 OMPL，结果可能不同。
- 缺省配置下 NATIVE 的车道内发送路径检查未并行。
- 任何示例配置都没有打开 NATIVE。
- 实机运动、阀门命令或验收，均为 **NOT_RUN**。

## 2026-11-17：第 1—7 步审查 A–F 修复

推送前发现并合入并行提交 `6477f32c19bd2bf8aac91ba88fc5ea1522a5f67c`。其事故执行锁、40 mm 工具—机器人间隙和腕部限制完整保留；这些策略来自该提交，不是 A–F 修复对原 2.4 mm 判据的改写。报告分别保存合并前与合并后验证，历史数字不直接冒充最终合并版本验收。

从目标分支实际 HEAD `abe36ae01abdeb9010f4012eaa76689380a437c8` 开始，在独立 worktree 实施；没有覆盖原工作区的未提交内容。机器可读证据见 [修复报告](evidence/realtime_20261117_review_fixes/report.json)。本轮没有连接机器人、相机或串口，实机验收统一 `NOT_RUN`。

A：execute 在本次复验 NPZ、绑定和任务记录提交后，再对观测、规划及 journal 做可取消的写入屏障；写失败、线程失效或超时拒绝启动。运行中记录失效继续使用既有停止和持箱语义。回归使用真实 execute/save_observation/EvidenceWriter，注入磁盘错误。

B：原生读取端按 JSON 结构识别取消，保留二进制取消；回调贯穿 NATIVE 模板、场景上传及复合请求。错误、重复、迟到 ID 不产生污染后续请求的回复。客户端及失败记录区分调用结果失效、取消发出、worker 确认结束；无确认仍退役。实际新旧 native worker 对照见报告。

C：GC 回调只向有界队列追加，take 时归并；不等待 aggregate 的锁，不关闭 GC。计时聚合键和 GC 事件都有上限；后台计时写失败可从返回线程的 error 观察。

D：一个绝对 deadline 覆盖整个 prepare_plan。每个任务拥有独立可变状态、代次、临时计划和暂存许可；必要写入与 Cycle.ready 完成且预算、session、代次仍有效才提交。旧任务不能撤销新任务许可、发布迟到计划或交还未确认可用的 worker。清理可后台继续，状态在 last_task 和 planning-task-lifecycle.json；清理时间不伪装成调用者已等待的时间。假阻塞场景按 deadline + 50 ms 检验，真实原生试验逐次保留超时量；不宣称硬实时。

E：deadline 回放正确传递候选筛选。规划成功口径为 prepare_plan 入口至 PLAN_READY，失败保留入口至返回；局部回放明确为部分口径，不用于判定完整入口达标。嵌套/并行时间按区间并集统计，10 秒区间中的 2 秒收尾不重复加。调度拆出首点与四个交接点超额等待，保留原到位/压力时标并避免重复扣减；缺字段不补造。失败、取消、超时和无样本均保留。

F：补齐原生 prepare 的限值检查；内部 START/END 日志打开、写入和 flush 失败拒绝成功；完整检查路径用既有规范二进制 SHA-256 绑定。ADAPTIVE 字段明确只报告动态约束精确锚点，整段最小距离为 NOT_COMPUTED；实际碰撞判据不变。

本轮保留的限制：已有 `test_exact_prior_face_keeps_a_bounded_envelope_that_covers_the_prior` 在原始 HEAD 与修改后均失败，未更改几何断言或包络；原生 prepare 的加速度等价性仍有超过 `1e-12` 的样本。fake IO 调度运行的失败和不完整样本也保留，不能当作现场验收。完整数据和逐项测试状态以报告为准。这次修复不表示前七步全部验收完成；真实记录等价、单车道 OMPL 等价、完整二进制轨迹布局、代理资产交付、共享内存和未达标数值精度仍是待办。

合并后定向 pytest：611 通过、1 项上述既有几何失败；四种 clearance、实际取消协议、单段 native 组件和范围隔离检查均通过。另一次 lifecycle 补测为 18 通过、1 项 finalize 时限失败：100 ms 预算在约 184.5 ms 返回，超过 150 ms 门槛；该用例单独复核通过，原失败仍保留，deadline 时限标为未稳定达标。该次与编译/其他定向测试并行，不能据此断言具体根因或扩大预算。

下一轮碰撞工作区复用仅拟定范围，本轮未实施：

- 在现有 worker 的 Clearance/ ClearanceWorkspace 构造边界增加构建计数和完整上下文身份，先测每个任务的构建次数 N 与不同身份数 K；每个 worker 保有独立可变状态，不跨进程共享 FCL 对象。
- 身份至少包含模型/工具和碰撞资产哈希、场景版本及完整几何、目标身份及包络、附着变换、阶段接触权限、物体对规则/间隙、工作范围、碰撞模式和相关配置版本。只按 world handle 命中不合法。
- 任一上述字段变化、场景重传、task reset、worker 重启或撤销必须失效；FK 与各查询计数、deadline、取消状态不能从旧请求继承。先考虑同任务同完整上下文复用，再单独论证跨任务只读几何复用。
- 对照无复用基线，逐请求比较判定、失败对/阶段、发送路径、候选选择、原始间隙及许可绑定；覆盖模型、场景、附着、接触权限、包络和工作范围逐项变更，以及取消/reset/并行车道。继续运行四种 clearance 回归和范围隔离检查。
- 预期把重复构建从 N 减到 K，减少 N−K 次。审查中的 132 次校验是历史例子，不是本轮实测：若它们确属同一完整身份，理论上可由 132 次减到 1 次；若身份不同则只能减到实际 K。先用修正后的量具测量，不能预先宣称 131 次或固定加速比已经实现。
## 2026-11-14 现场事故：接近段吸具撞击 J6，TCP 连接件断裂

任务 `d99b1a8f5d71445395c158b3c353ca42`：接近段（约 32 s）第 15 s 左右吸具撞到 J6 附近，TCP 连接件断裂；随后接触段完成，吸附压力超时停止。
发送点与规划点逐点一致，控制器全程无报错。用同一原生检查复查实际执行的 350 个姿态，**在模型中全部合格**：
小臂 `link_3` 与吸具在 9～26 s 内仅 13～28 mm（2.4 mm 要求），最近的是光电传感器 `part_027`（E3Z-LS81，2 m 线缆）及其支架；
**传感器线缆、出线口和接头 `part_037` 上的真空气管不在 CAD 中**。该接近段手腕翻转：J5 从 +87° 经 0° 到 −58°，J4 转 86°，
起因是 `e2d31a2` 的多起点逆解选了关节空间最近、但在手腕另一侧的解，且未加分支约束。画面复核和起点核对的关闭与本次无关（起点差 0.008°）。
分析记录：`outputs/incident-20261114-d99b1a8f/`，报告 `docs/evidence/eco65_real_site_20261112/incident_20261114_report.json`。

按用户要求做了三项修复（2026-11-17）：

1. **执行硬锁** `execution_lock`：现场配置 `locked=true` 并记录依据。“执行单箱”在任何取景前拒绝（`EXECUTION_LOCKED_BY_USER`，写入事件），
   执行准备显示锁定原因；锁定状态写入计划和**每段**轨迹包，运动引擎对带锁标记的包一律拒绝（格式不对也拒绝）。规划、预览照常，
   停止通道、停止按钮不受影响。解锁只能显式改配置为 `locked=false` 并重启。
2. **吸具对机械臂间隙 40 mm** `tool_robot_clearance`：原生检查新增仅限实机单箱范围的部件对间隙，吸具零件与非吸具机械臂连杆之间要求 40 mm。
   豁免：`link_6`（吸具固定安装面），以及与 J6 同轴、距离恒定的转接件 `part_041_region_1/2` 对 `link_5`（28.9 / 36.9 mm）。
   其余吸具零件对腕部在 J5、J6 全范围扫描中均 >40 mm（最小 `part_039`–`link_4` 44.5 mm）。原生规划程序已重编译安装，旧二进制备份在 `outputs/setup/worker-backup-20261113/`。
3. **手腕分支限制** `wrist_branch_limits`：相对冻结的 q_start，J5 不得变号、|J5| ≥ 10°（不进入奇异带），J4 转动 ≤ 90°，J6 转动 ≤ 180°。
   逆解阶段直接剔除违规分支；每段**全部发送状态**检查一次，签发前再对四段整体审计一次。

验证：事故实际执行的姿态在新规则下 326 个中 205 个被拒（首个在 8.3 s，小臂对吸具 39.3 mm），翻腕检查也拒绝；
用事故记录离线重规划，得到不翻腕的完整计划（候选 11，J5 87→58°，四段发送路径检查 PASS，约 476 s）。原生 optimized/reference 检查 PASS，定向回归 **463 passed**。
**执行仍锁定**：需先补全传感器线缆、气管、接头等实物几何，修复并核对连接件，再显式解锁，首次以慢速空载、有人守急停验证。


### 2026-11-17：按第 0–7 步审查记录进行现场机核验

已在隔离 worktree 重建 `abe36ae` 和基线 `0783e86`，保留主工程事故执行锁和相关修改。四种原生模式与组件检查 PASS；定向集首次 538 通过、3 项因隔离检出中的 LFS 指针失败，补齐本机资源后均复测通过，R15 在本机通过。新增两个采集/工具回归通过。

两条真实历史记录完成新旧默认流程对比：一条 32 项拒绝原因一致，另一条完整四段通过、选中候选 18、胜出原生 q/v/a/t 数组完全一致。真实轨迹 prepare 193 例在 `1e-12` 内一致，多障碍现场终态两级碰撞 1616 次无反例，接近段自适应审计 6 请求判定一致。全部关闭 PYTHON 与全部开启 NATIVE 的两组合成场景对比及连续任务历史检查通过；实际现场全部开关组合仍未验证。

新增 `tools/collect_realtime_field_records.py` 仅允许观测与规划，独立拒绝运动提交和真空序列，固定禁用执行按钮。新 20 条箱位/遮挡采集仍等待原设备会话正常退出及保存 GPU SSH 地址恢复。9 条历史数据已本地打包，服务器连接拒绝，尚未传输；没有用回放冒充新采集。未修订 2 s 目标或放宽容差、碰撞及监控规则。

详见 [本轮复核与采集记录](realtime_review_followup_2026-11-17.md) 和 `docs/evidence/realtime_review_20261117/report.json`。物理抓放、真实执行时延及新现场 p95 验收均 NOT_RUN。第 8 步检查点尚未通过。

## 2026-11-17 最新 HEAD 现场采集前检查

基于 `2bd36a9` 重新编译安装现场主目录 worker，optimized/reference/two_level/adaptive 全部 PASS，原生单段组件 PASS（491 个发送点；不是完整物理周期）。定向 pytest 568 通过、0 失败，采集保护测试另有 2 项通过。专用采集入口修复父类处理 close_done 销毁窗口后仍刷新执行按钮的问题，无设备 UI 冒烟通过。报告：`docs/evidence/realtime_20261117_field_preflight/report.json`。

当前普通工作台三类设备显示数据有效；须正常关闭其窗口释放设备，再从当前主目录启动下列入口。该入口启用计时，拒绝运动提交、任务执行及真空阀门命令，所有实验开关保持缺省关闭：

```bash
.venv/bin/python tools/collect_realtime_field_records.py
```

至少 20 个不同真实箱位/遮挡案例，每次只进行准备观测和规划，保留失败记录。输出：`outputs/realtime-field-20261117/<task_id>/`。采集结束正常关闭计时窗口。此检查时新采集数仍为 0，等待旧工作台关闭及操作者摆箱；没有物理抓放，验收为 NOT_RUN。旧隔离 worktree 和历史记录保留，不以旧固定审查版本替换当前 HEAD。

## 2026-11-17 两个现场箱位采集完成（用户本轮缩减样本数）

用户在确认“2号已摆好”时明确要求本轮只采两条，覆盖先前不少于 20 条的本轮采集数量。专用采集入口基于 `3bc95a1`，三类设备取得新有效数据后逐次准备观测和规划，未提交运动或阀门动作。采集完成已正常退出计时窗口，未自动打开普通执行入口。

| 箱位 | 任务 ID | 准备观测至 TARGET_READY | 规划入口至失败返回 | 结果 |
|---|---|---|---|---|
| 1 | `313f32ad66a240798df17905025edc89` | 21.014 s | 119.270 s | 13 终态阻挡、10 INTERPOLATION_LIMIT、9 搜索预算耗尽 |
| 2 | `6966ce4b0141495993727b6d72815637` | 19.848 s | 118.801 s | 11 终态阻挡、9 INTERPOLATION_LIMIT、12 搜索预算耗尽 |

两条均为真实 640×480@30fps RGB-D，恢复 1 个箱体；现场碰撞物体分别 39 / 37，仿真设备导入数均为 0。恢复中心相差约 6.63 cm。每条原 RGB-D、分割、几何、现场场景及两阶段计时完整保留在 `outputs/realtime-field-20261117/`；另一个仅准备观测的目录 `fba38e2500aa45d48144df3b5c2c25f4` 保留但不额外计作不同箱位。

两条均无完整可执行计划；119 秒左右是失败返回时间，不是 PLAN_READY 时延。21 个候选搜索预算耗尽不证明物理无解。两个样本不能确认有代表性的 p95，不修订预算或放宽限值。新输入的旧版/新版完整回放及 native 等价性检查仍待运行，物理抓放与部分遮挡实箱抓放验收均 NOT_RUN。摘要及输入 SHA-256：`docs/evidence/realtime_20261117_two_field_cases/report.json`。

## 2026-11-17 复审 R2-01：最终证据 barrier 后的启动边界

真实 EvidenceWriter/NPZ 阻塞期间使观测、会话或许可失效，旧代码 3 项失败；最终 barrier 后重新验证取消、观测代次及全部许可，并用后台观测锁排序代次更新与首次启动。许可仍在 submit 原子领取；停止信号在等待锁前发出。定向集最终 573 passed；首轮 2 个缺少新锁的 fake fixture 失败保留，已补齐初始化。optimized/reference 和原生组件均 PASS。见 `docs/evidence/realtime_20261117_review2/r2-01.json`。实机、2 s p95 与物理验收 NOT_RUN；执行锁与保护配置未改。

## 2026-11-17 两次失败规划的离线可视化

`tools/visualize_field_planning_failures.py` 从原记录导出两张概览图和离线自包含三维页面，不连接设备、不重新规划。页面保留每次全部 32 个候选、39/37 个原现场物体及目标、715 个碰撞实体和 12 杯，可切换实测起点、保存的终态和局部原生输出采样。带载输出按原请求的 flange-to-carton 附着矩阵显示完整包络，不将目标重复加入环境。两条记录分别有 40/36 份含点列的原生输出记录，可能包含同一路径的别名或重试，不称为独立完整通过路径。

静态图各选择一个真实终态阻挡例，并列出全部候选分类。箱位1所示为吸盘唇与支撑障碍相交/接触；原生负距离状态码不当作穿入深度。箱位2所示间隙 2.20 mm，小于要求 2.40 mm。这些位置不代表其他候选也在同处失败。原生局部轨迹未经整条发送路径审计，不能执行；搜索预算耗尽不证明物理不可行。

```bash
.venv/bin/python tools/visualize_field_planning_failures.py \
  outputs/realtime-field-20261117/313f32ad66a240798df17905025edc89 \
  outputs/realtime-field-20261117/6966ce4b0141495993727b6d72815637 \
  --output outputs/field-failures-20261117
xdg-open outputs/field-failures-20261117/index.html
```

Edge 软件渲染浏览器定向检查通过：两箱位、64 个候选及保存采样切换、附着矩阵、715 碰撞实体、12 杯；Python/browser FK 最大矩阵误差 1.11e-16。概览中文及布局已逐图检查，源记录 SHA-256 前后保持一致。此步骤未改规划或设备控制代码，未重跑规划/碰撞验收或实机动作。报告：`docs/evidence/realtime_20261117_failure_visualization/report.json`。生成的原始照片、CAD 页面和 PNG 仅保留 outputs，不提交 Git。

## 2026-11-17 复审 R2-02：复合原生腕部判据与取消传播

复合请求绑定当前腕部策略与冻结 q_start；C++ 在 IK 排序后过滤，并在 prepare 后、下一请求前执行同一 J5/J4/J6 判据。实际 C++ 复合输出驱动实际 Python 回放：修复前 14 failed/4 passed，修复后 18 passed，覆盖全部解拒绝、部分保留、接触解拒绝、四段路径中间越界及正常链路；拒绝路径也检查未消费调用，禁止吞掉分歧。CONTACT/DETACH_AND_VERIFY 的 scene handle 全部传取消回调，相关 21 项通过。必需定向集 591 passed，重编译 worker 后 optimized/reference 与原生组件 PASS。见 `docs/evidence/realtime_20261117_review2/r2-02.json`。几何响应为 fake，实机与完整现场等价 NOT_RUN；默认模块/开关、限值及执行锁未改。

## 2026-11-17 复审 R2-03：提交 I/O 与调用线程预算

提交 rename、失败撤回和记录互斥等待转入任务后台；调用线程的任务锁/许可锁等待使用同一绝对截止时间，只发布已写入记录且仍有效的任务。复现前 4 项失败，100 ms 预算与原 +50 ms 阈值保留；修复后新提交/锁/胜出计数和既有生命周期等联合 63 项通过，必需定向集 609 passed、碰撞双模式及原生组件 PASS。首次定向组的错误标签兼容性失败保留并恢复 BEFORE_DISPATCH。已有胜出候选的收尾超时记 SEARCH_SUCCEEDED_FINALIZATION_BUDGET_EXHAUSTED，胜出候选计入已搜索；迟到结果不能覆盖新许可。调用返回、请求结束、清理结束分别记录。另将观测代次递增本身置于同一锁，防止两个设备失效丢一次递增。见 `docs/evidence/realtime_20261117_review2/r2-03.json`；这是 fake IO 应用预算检查，不是操作系统硬实时保证，物理验收 NOT_RUN。

## 2026-11-17 复审 R2-04：关闭与预热/补池的资源归属

关闭在任务锁下更新 closed/代次；预热和 pool 创建结果发布前再次核对，拒收资源在锁外关闭。补池、初始启动和重复 close 均覆盖，清理异常可查询，wait_closed 供后台验证清理结束。5 个修复前失败均重现；resident 相关 41 项通过，扩充关闭/清理集 7 项通过；共同必需集 609 passed、optimized/reference/原生组件 PASS。见 `docs/evidence/realtime_20261117_review2/r2-04.json`。默认驻留开关未改，物理 NOT_RUN。

## 2026-11-17 复审 R2-05：GC 溢出统计不参与时间戳比较

8193 次 gc_pause 后带 since 的 write_later 原会同步 TypeError；溢出计数现作为本次 drain 的无时间戳 aggregate 单独追加，仍保持 8192 上限和原 GC 行为。overflow/重入/写失败可见性 3 项通过，共同必需集 609 passed，碰撞双模式及原生组件 PASS。见 `docs/evidence/realtime_20261117_review2/r2-05.json`；这是计时器正确性验证，物理 NOT_RUN。

## 2026-11-17 复审 R2-06：全样本口径与采集目录

完整/部分/缺失、无计时、成功/失败/取消/超时均按全部输入统计；部分或缺失计时不能给出全组达标。成功完整样本 p95 单列，失败返回耗时保留但不叫 PLAN_READY 时延；成功率基线仍需另行比较。原有及新增统计检查 13 项通过，新增发现目录和成功子集用例也纳入共同 609 项必需集，碰撞双模式及原生组件 PASS。见 `docs/evidence/realtime_20261117_review2/r2-06.json`。

保存现场完整入口计时（只读，不回放、不连接设备）：

```bash
.venv/bin/python tools/bench_planning_latency.py --timing outputs/realtime-field-20261117 --output outputs/review2-field-timing
```

另行离线部分回放：`--records TASK_DIR` 或 `--discover-root outputs/realtime-field-20261117`。该根目录包含仅观测任务时，观测数与规划数分别报告，不能追加为新的完整箱位案例。默认优化开关、2 s 目标及原验收阈值未改，物理 NOT_RUN。

### 2026-11-17 R2-03 离线回放计数补充

最终复核发现回放入口还有一处独立的 DONE 赋值，现同步记录 accepted 与 SEARCH_SUCCEEDED；真实回放入口的 fake 回归断言胜出候选已搜索且 unsearched=0。相关 10 项通过，原生产规划修复保持不变。后续固定提交统一验证结果见本轮总报告；物理 NOT_RUN。

## 2026-11-17 复审 R2 最终代码与独立验证

固定代码 `48043b4e84a6c2e4f52b850590e2ccabbee29670`：本地完整定向集 656 passed；GPU 使用已有依赖环境的最终完整组 655 passed、1 个既有 prior 几何失败，无跳过/环境错误。本地与 GPU 的 optimized/reference/two_level/adaptive、实际 C++ limits/journal/path 契约、单箱原生组件均 PASS。GPU 首轮 167.733 ms 返回超过原 100 ms + 50 ms 门槛，虽最终组通过仍保留 NOT_STABLY_PASSED。

两份真实保存帧各 32 候选的终态预检 PYTHON/NATIVE 一致，2413/2379 次回放调用均 unused=0；这是终态预检，没有完整四段规划或许可。本地 27 条保存轨迹的 193 个 prepare 案例在原 1e-12 门槛下通过；GPU 同一输入仅 160/193 一致，33 个数值不合格，最大加速度差 1.83836e-8；旧服务器样例的 9.31583e-12 失败亦重现。两端输入/构建/环境及失败分别保留，未放宽阈值。

R2-01～R2-06 与三个集成补充项已实现；执行锁、40 mm 间隙、腕部限制及默认模块/优化开关未改。本轮未连接现场设备、未采集、未运动或写阀。steps_1_to_7_accepted=false、完整调度 NO_COMPLETE_SAMPLES、新 20 输入未完成、2 s 完整规划目标未证明、物理 NOT_RUN，跨请求工作区复用未开始。详见 [复审修复总报告](realtime_review2_fixes_2026-11-17.md) 与 [结构化证据](evidence/realtime_20261117_review2/report.json)。

## 2026-11-18 两个现场箱位的姿态与高度诊断

用户提出接近和脱垛时软偏好“大臂向上接近竖直、小臂接近水平”，不使用固定关节角或必须满足的姿态命令。本次先核对原生 IK 保存记录，加入只读诊断工具 `tools/diagnose_field_postures.py`；没有修改生产规划的候选/分支顺序，也没有上线新的偏好代价。当前仍按离冻结 q_start 最近的分支排序、过滤腕部限制，至多检查三个分支。姿态软偏好应以实际连杆方向衡量，同时考虑连续性；它不能使违反现行腕部限制、碰撞或完整带载路径检查的解成为可执行方案。

两条记录都只有一个用于吸取的近似竖直观测面，中心高度分别为 base_link 下 405.95 / 405.70 mm，接近 TCP 高度范围均约 359.5～450.0 mm。没有顶吸候选，不能把失败归因为“顶吸太高”。相同接近 TCP 位置和方向已有更接近偏好的逆解，因此“正吸高度太低导致没有这种逆解”也不成立。这里仅证实逆解存在，不证明路径可行；TCP 高度也不能直接当作肘或腕的高度。

下表是三维查看器默认显示的候选 3，前两种分支为完全相同的 TCP 目标（z 约 382.4 mm）。角度来自 URDF 关节原点向量：大臂为 link_2→link_3 与 base +Z 的夹角，小臂为 link_3→link_4 与 base XY 平面的夹角；这是方向诊断，不冒充 CAD 中心线测量。

| 记录 | 被拒绝终态：大臂 / 小臂偏离 | 更接近偏好的原始逆解 | 当前腕部拒绝原因 |
|---|---|---|---|
| 313f32ad，箱位 1 | 122.1° / 75.9° | 7.3° / 14.8° | J5 从起点 +85.8° 变为 -60.8° |
| 6966ce4b，箱位 2 | 121.8° / 76.2° | 3.6° / 14.8° | J5 从起点 +85.8° 变为 -59.9° |

另有 J5 同侧、大臂较竖直的分支，小臂偏离水平约 39.5° / 38.6°，但相对起点 J4 转动 149.3° / 147.9°，超过现有 90° 上限；箱位 1 的该原始解还带有 J6 绕转，未人为修改或折算。两份记录逐目标去重后的原始 IK 共 225 / 219 个，腕部过滤后仅 29 / 30 个；在已记录的保留分支中，按上述两个方向偏角之和最小者，大臂仍偏离向上 104.1° / 104.6°。这说明仅对现有合格分支增加软排序不能消除目前的姿态问题，不等于穷尽全部可能的 IK 解。

保存的脱垛局部原生输出也呈现这个姿态问题：箱位 1 的末采样大臂偏离向上 94.2°～101.8°，箱位 2 为 94.9°～105.0°。分别读到 20 / 18 份输出，包含别名或重试，不能当作 20 / 18 条完整通过路径。本轮没有验证替代逆解能沿脱垛和带载回程保持偏好。

以原场景、原碰撞规则对两组候选 3 的三个分支各作一次原生静态查询：箱位 1 均报告 `part_006_lip` 与原 RGB-D 支撑障碍接触/相交，箱位 2 均报告该唇与 `rgbd_observed_7_18_10` 间隙约 2.19672 mm，小于 2.40 mm。工具的世界位姿相同，换肘部分支没有消除这个端点阻挡。负距离 -1 是接触/相交状态码，不是穿入深度。查询返回第一个失败，不证明其他碰撞对通过，也不证明整条接近、脱垛或带箱返回路径通过。其余候选的插值加速度限值及搜索预算失败仍保留，不能统一改称高度问题。

```bash
.venv-workbench/bin/python tools/diagnose_field_postures.py \
  outputs/realtime-field-20261117/313f32ad66a240798df17905025edc89 \
  outputs/realtime-field-20261117/6966ce4b0141495993727b6d72815637 \
  --wrist-policy-report docs/evidence/realtime_20261118_posture_diagnosis/report.json \
  --output outputs/field-posture-historical-recheck --validate
xdg-open outputs/field-posture-historical-recheck/posture-comparison.png
```

定向核验：444 个保存逆解的 Python FK 全部满足原 2e-5 m / 2e-4 rad 容差（最大误差 1.372e-6 m / 6.209e-7 rad）；两条记录的腕部过滤结果与归档完全一致；6 次原生静态查询完成；源文件 SHA-256 前后不变，中文对比图已检查。摘要：[姿态诊断证据](evidence/realtime_20261118_posture_diagnosis/report.json)。这不是生产规划改动，未重跑完整规划回归，未修改腕部阈值、40 mm 间隙或执行锁；未运行顶吸实测、替代分支的完整路径及实机抓放，物理验收 NOT_RUN。

## 2026-11-18 偏好逆解的 CAD 三维对比与腕部分支解释

可视化导出器新增可选 `--posture-report`，将上述两箱位候选 3 的原始替代逆解加入完整 CAD 查看器。默认显示 A（更接近偏好、J5 负侧），可直接切换 B（J5 正侧、J4 转动较大）、实测起点、原拒绝终态；显示 J4/J5/J6 转轴及当前关节角。原 64 个候选和保存采样仍保留，全部 715 碰撞实体及 12 杯不变。切换只显示离散姿态，不插值生成或暗示一条已验证的运动路径。A/B 都被现行腕部规则拒绝，原吸取点仍存在已记录的工具/环境阻挡；不能执行。

```bash
.venv-workbench/bin/python tools/visualize_field_planning_failures.py \
  outputs/realtime-field-20261117/313f32ad66a240798df17905025edc89 \
  outputs/realtime-field-20261117/6966ce4b0141495993727b6d72815637 \
  --posture-report outputs/field-posture-20261118/report.json \
  --output outputs/field-posture-viewer-20261118
xdg-open outputs/field-posture-viewer-20261118/index.html
```

“需要翻 J5”只描述这个已保存的 A 分支，不代表大臂竖直必然要求 J5 翻侧。这里的正/负是关节相对于零位的两侧，不是吸盘面指向相反方向。固定同一 TCP 位置、面法向和面内朝向时，A/B 通过肩肘和腕部共同变化保持末端姿态。以箱位 2 为例：

| 原始逆解 | J2+J3+J4 | J5 | J6 | 工具目标 |
|---|---:|---:|---:|---|
| A | -177.88765° | -59.87551° | -181.31029° | 同一个候选 3 接近位姿 |
| B | +2.11259° | +59.87551° | -1.31004° | 同一个候选 3 接近位姿 |

两组 J2+J3+J4 约相差 180°，J5 符号相反，J6 也约相差 180°，因此不是独自改变 J5 还能保持吸具不动。这里按已保存的原始角显示，不把箱位 1 的 J6 绕转改写成另一条轨迹。若从起点 J5=+85.781°连续到 A 的负值，关节值必然经过 0；该模型在 J5=0 时 J4/J6 转轴平行，当前防翻腕/奇异区规则禁止这类跨越。零位轴关系仅作离线 FK 计算，未生成零位运动指令，也未验证跨越路径。

J4 的约 148°是箱位 2 的绝对关节差值，不是指定动作幅度，也不是全局最小转角的证明。该 ECO65 模型 J2/J3/J4 轴平行；在 B 这一固定 J1、J5、J6 分支中，工具朝向要求 J2+J3+J4≈+2.11259°。较直的大臂构型给出 J2+J3≈-129.94774°，故 J4≈132.06033°；相对实测起点 -15.809°，变化为 147.86933°。箱位 1 对应值为 149.28999°。这解释了具体分支为何被 90°旅行限值拒绝，不能推广为所有目标位姿或所有逆解都必须转这么多。

定向检查：4 个新增姿态与原始 IK 回复逐值一致，并与原拒绝终态满足同一 TCP 位姿容差；J2/J3/J4 平行轴和 J5=0 时 J4/J6 轴关系由现有 URDF/FK 核对。浏览器覆盖两箱位 A/B/起点/原终态按钮、全部 64 候选和保存采样；4 个替代姿态的浏览器/Python TCP 矩阵最大误差 1.596e-16。两张 CAD 截图已检查。证据见 `docs/evidence/realtime_20261118_posture_viewer/report.json`；生产规划、腕部限值与执行锁未改，未重跑完整规划/碰撞回归，实机 NOT_RUN。

## 2026-11-18 复审续修：返回路径与验收工具

从实际 HEAD `2d5b5b5` 保留已完成的 R2 六项修复，新增 `b64ed90`：超时证据复用既有任务线程，完整 prepare_plan 的拒绝和计时复用任务 journal，避免在已过期调用者上等待新线程启动；回执失败及 journal 接收后死亡可查询。实际线程启动延迟 250 ms 的反例原返回约 351–352 ms、完整入口约 603 ms，保留 100 ms + 50 ms 原门槛，修复后的启动门/计时定向组 22 passed。

`f1d2a98` 使 native prepare 比较失败返回 1，保留判定分歧及缺失数值统计，并在 worker 异常时关闭 client；`8041a3c` 加入只读插值系数诊断及工具回归（4 passed）。没有修改 Python 参考或 native 数值算法，也没有放宽 1e-12。

同一固定代码 SHA `8041a3c687f5e0b72d7e5c02cd0668d647c2d09b` 在新 GPU 端口对应的已有隔离环境运行：完整定向集 663 passed / 2 failed / 0 errors / 0 skipped；保留既有 prior 几何失败，以及一次在 SCREENING 到期、未进入注入 plan 请求的 unanswered-request 用例失败。后者原用例单独复测通过且确实进入阻塞 plan，但不覆盖完整组失败。四种碰撞模式、limits/journal/path、实际四格式取消协议、单箱原生组件与 deferral 范围均通过。

当前源码对两份保存现场帧各 32 个候选完成 PYTHON/NATIVE 端点预检等价；分别消费 2412 / 2405 次回放调用，unused=0，源记录哈希未变，许可数为 0。首次 chroot 外目录不可见导致的 journal 拒绝已保留；改用共享 checkout 内新目录完成比较。当前环境的调用数单独记录，不作为跨环境一致性、完整路径或现场验收依据。

27 条保存轨迹的 193 案例仍为 160 一致、33 数值失败，最大加速度差 1.83836e-8；旧服务器组件仍为 8/11 一致、最大差 9.31583e-12。只读诊断将最大差定位到约 1.584 ms 末区间的数组/标量幂舍入及系数相消；诊断副本重现 C++ 系数和加速度差，不计作通过。

执行锁、40 mm 间隙、腕部限制及默认模块/优化开关未改；无新设备采集或实机动作。steps_1_to_7_accepted=false、稳定 deadline 门槛与数值等价仍未验收，完整调度/新 20 输入/2 s 目标未完成，物理 NOT_RUN；本轮未实施工作区缓存。详见 [续修报告](realtime_review2_gpu_followup_2026-11-18.md) 和 [结构化证据](evidence/realtime_20261118_review_fixes/report.json)。

## 2026-11-18 旋转、上移与偏好构型重试

本轮按用户要求取消 J5 翻侧/零位带限制；随后用户明确回复“同时取消 J4 相对起点限幅，再离线试算”，因此 `max_j4_travel_deg` 改为显式 `null`。J6 相对起点 180°限幅、官方关节范围、0.10 rad/s / 0.20 rad/s²、2.4 mm 碰撞规则、40 mm 工具/机器人预留间隙、全部 715 碰撞实体/12 杯、压力门与事故执行锁均保留。本轮没有设备连接、运动或阀门动作。

配置仍为 `configs/workbench/single_carton.example.json`：

- `wrist_branch_limits`：`keep_j5_sign=false`、`j5_min_abs_deg=0`、`max_j4_travel_deg=null`；依据保存在 `basis`。历史数值限幅配置继续支持。
- `contact_search`：以此前查看器候选 3 的面内基准（`reference_face_roll_deg=90`）为参考，绕实际 flange/J6 轴旋转 180°；随后沿观测面上的向上方向搜索 0～60 mm、间隔 10 mm 的接触位置。最多 32 个候选，按高度组交错保留，避免较少密封杯的上移候选全部被截断。没有转动 CAD 安装、改 TCP 或改标定。
- `posture_preference`：基于源 URDF 的 link_2→link_3 向上方向和 link_3→link_4 水平方向软评分，并保留原始关节角的连续性代价；用于端点 IK 分支和候选的优先级，仍最多检查三个 IK 分支。不是固定关节命令或必须满足的姿态阈值。每次保存实际评分和候选顺序。

每个变换后的候选重新检查实际 CAD 密封轮廓、观测深度支持、3 mm 边缘余量及接触法向/间隙。只需实际覆盖的杯形成密封；其余杯按既有共同阀门、被动自闭规则处理，保留刚性件和唇部全部几何。绿色只表示**预计密封**，不能代替实测压力。带载仍使用原完整不确定性包络。

离线入口新增 `--frozen-scene`，复用保存记录的原 RGB-D、掩码、几何、现场障碍、q_capture 与 q_start，隔离本轮规划策略变化；源文件前后 SHA-256 核对。重试各使用 120 s 总预算，失败返回时间不称为 PLAN_READY 时延。默认模块保持 PYTHON，未启用原生加速或改变监控阈值。

```bash
ECO65_TIMING=1 .venv-workbench/bin/python tools/replay_single_carton_record.py \
  outputs/realtime-field-20261117/6966ce4b0141495993727b6d72815637 \
  --frozen-scene --planning-deadline-s 120 \
  --output outputs/posture-retry-new-case2
# 对箱位 1 使用原目录 313f32ad66a240798df17905025edc89，输出另一个新目录。
.venv-workbench/bin/python tools/visualize_field_planning_failures.py \
  outputs/field-posture-retry-20261118/j4-free/case-1 \
  outputs/field-posture-retry-20261118/j4-free/case-2 \
  --output outputs/field-posture-retry-20261118/j4-free-viewer
xdg-open outputs/field-posture-retry-20261118/j4-free-viewer/index.html
```

查看器可切换偏好逆解、实际预检接近/接触姿态、实测起点和保存的脱垛输出末点，并显示上移量、预计密封杯数、关节角与偏好角度。被腕部规则排除的分支明确标注；未经查询的分支不标作碰撞通过。`detach 起始带载检查` 是吸附后的原位检查，不冒充脱垛终点；没有输出点列时“脱垛输出末点”不可用。所有搜索输出仍与完整发送路径审计区分，无执行许可。

普通现场入口在下次启动时读取新配置（本轮未启动它）：

```bash
.venv-workbench/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config configs/workbench/single_carton.example.json
```

执行锁仍保持。新现场观测/规划与实际抓放是独立步骤，不能执行保存帧重试输出；实机及部分遮挡箱完整抓放验收仍为 NOT_RUN。

本轮最终重试结果（接近/接触均为实际保存且通过静态检查的姿态，非完整路径通过）：

| 箱位 | 优先候选 | 沿面上移 / 预计密封杯 | 接近：大臂 / 小臂偏角 | 接触：大臂 / 小臂偏角 |
|---|---:|---|---|---|
| 1 | 3 | 30 mm / 6 杯 | 5.34° / 21.93° | 0.08° / 21.94° |
| 2 | 13 | 60 mm / 3 杯 | 2.93° / 12.10° | 2.18° / 11.91° |

箱位 1 的 32 个候选：10 个工具/环境终态阻挡、8 个插值加速度限值拒绝、5 个接近搜索单次预算耗尽，另有 9 个未取得最终结论；箱位 2：9 个终态阻挡、19 个插值加速度限值拒绝、2 个接近搜索单次预算耗尽，另有 2 个未取得最终结论。总预算各 120 s，原计时口径下搜索阶段返回 117.62 / 117.36 s；不是成功 PLAN_READY 时延，也不能证明全局无解。两次均没有完整通过的四段计划，未签发许可。箱位 2 候选 13 的脱垛原生输出末点大臂/小臂偏角为 26.59° / 3.14°，但该候选最终为 `INTERPOLATION_LIMIT:1:2`；末点展示不能替代发送轨迹审计。

仅取消 J5、尚保留 J4=90° 的第一次重试也保留在 `outputs/field-posture-retry-20261118/case-1`、`case-2` 与 `viewer` 中；该阶段箱位 2 上移 30 mm 的偏好解要求 J4 相对起点 94.83°，因此提出澄清，随后按用户明确回复取消该限幅。本轮没有通过增加 J6 限幅、清空障碍或提高速度/加速度阈值来制造通过。

验证：规定 Python 定向集 **592 passed**；最终补充的姿态与实际 C++ 复合回放组 **34 passed**（与前组重叠，新增覆盖四阶段 J4 相对限幅取消，共 10 项）。包括 128 个源 URDF 随机状态的 Python/C++ 评分与排序一致、实际 C++ 复合输出驱动 Python 回放且未消费调用为 0；这不是完整现场记录的原生数值等价验收。重新构建/安装 worker 后，`m710_clearance_test optimized/reference` 与单箱原生组件 **PASS**，后者检查 491 个最终发送点；配置静态检查无问题。

最终浏览器检查覆盖 2 个箱位、64 个候选、448 次保存姿态显示、125 次附着姿态核对；64 个偏好逆解的浏览器/Python TCP 最大矩阵误差 4.441e-16。715 个碰撞实体、12 杯和 39/37 个原现场环境物体保持不变。接触及脱垛 CAD 截图已检查；源记录哈希不变。代码、配置、测试日志与重试结果的绑定见 [本轮证据](evidence/realtime_20261118_posture_retry/report.json)。既有 native prepare 数值分歧、2 s 完整规划目标及实机抓放验收仍未完成。

## 2026-11-18 当前吸具姿态再逆时针旋转 180° 的只读对比

用户追加“吸具在现有基础上逆时针旋转180°，做完后可视化”。新增 `tools/rotate_field_tool_preview.py`，直接采用上一版查看器默认候选（箱位1 #3、箱位2 #13）的实际保存姿态；保留 J1～J5，J6 增加 π，不对原始关节角作 ±360° 改写。逆时针从 J6 正轴端朝轴心看定义。数学上是上一版再加 180°，没有重排候选、再次上移或把原角重新设为绝对 180°。生产规划配置、CAD 安装、TCP、标定、执行锁均未改。

默认显示箱位2接触姿态：J6 从 -181.21433° 到 -1.21433°；肩肘、大臂/小臂偏角、上移60 mm和箱体世界位置保持不变。按旋转后的实际 CAD 轮廓重新检查密封，预计3杯改为 `part_012_seal`、`part_018_seal`、`part_024_seal`，不能沿用旧杯编号。带载末点的替代抓取重新计算固定附着矩阵，让箱体世界位置保持不变；不是模拟带箱原地翻转。

```bash
.venv-workbench/bin/python tools/rotate_field_tool_preview.py \
  outputs/field-posture-retry-20261118/j4-free/case-1 \
  outputs/field-posture-retry-20261118/j4-free/case-2 \
  --output outputs/tool-rotation-new
xdg-open outputs/tool-rotation-new/index.html
```

本轮已生成 `outputs/field-tool-additional-180-20261118/viewer/index.html`。按钮切换“旋转前 / 逆时针+180°后”；灰色与粉色箭头分别表示原吸具和当前吸具 +Y，便于辨认对称阵列的半周旋转。没有插值动画或转动路径，原箱位1没有可用的脱垛输出末点时禁用该按钮。

10个前后姿态分别通过原生 IK/FK 核对，并独立执行静态碰撞查询；9个静态查询通过。箱位2新接触姿态通过，原脱垛输出末点的新朝向为 `link_3 ↔ part_027` 间隙37.372 mm，小于保留的40 mm要求，显示拒绝。箱位1的新增正向旋转相对实测起点 J6 旅行量为接近183.118°、接触183.252°，仍超过现有180°策略，未放宽或通过角度绕回隐藏。这些结论不证明原位转动或完整抓放路径可行；物理执行 NOT_RUN，许可数0。

浏览器核对2箱位、10个前后姿态，FK最大矩阵误差2.220e-16，箱体世界姿态误差1.249e-16；全部715个碰撞实体及12杯保留，截图已检查。定向回归及证据见 `docs/evidence/realtime_20261118_additional_rotation/report.json`。首次工具导出因展示用字段与原生初始化字典共享而被 CAD 绑定检查拒绝；已通过深拷贝隔离修复，首次失败目录与日志保留。未改变原生校验规则，也未重新规划完整路径。

## 2026-11-18 新现场帧重规划与实测起点顺时针半周预览

用户确认“实测起点处再顺时针180°”仅指三维预览。本轮使用当前工作台最新保存的现场记录 `outputs/single_carton/a9a1eaac54624c93ad96c5e35d33166b`：2026-11-18 21:13:51（北京时间），RGB/depth 帧号3608/3605、640×480@30fps。原始 RGB-D、q_capture、相机外参及 q_start 全部保持原值，并核对源文件哈希。本轮没有连接机器人或真空板，没有运动、阀门命令或执行许可；这是已保存新现场帧的离线重规划，不是新一次实机执行准备。

实测 q_start（度）为 `[80.998, -12.253, -54.206, -15.759, 85.987, -5.147]`。只在独立预览中令 J6 减去180°，得到 -185.147°，J1～J5 不变、不作360°绕回。顺时针从 J6 正轴端朝轴心看定义。两个端点分别通过原生 FK 与静态机器人/现场碰撞查询；全部715个碰撞实体、12杯、40 mm工具/机器人预留及既有对规则均保留。没有验证两端点之间的旋转路径，旋转预览不能作为实测起点或可执行轨迹。

原记录的检测失败原因是旧 GPU 端口拒绝连接。离线回放现支持对**尚无分割结果**的真实冻结帧显式使用 `--detector config`；缺失旧掩码的比较结果标记 `NOT_AVAILABLE`。重新推理后把真正用于几何恢复的新掩码与检测写入输出，避免查看器仍读到旧掩码。生产 GPU 选择不变；本轮独立的 `outputs/field-replan-20261118-new-start/offline-cpu.json` 明确选择本机 CPU、相同已保存权重和适配器，没有演示输出。模型加载2.26 s，检测9.344 s，分割7.713 s；这些是该冻结帧离线数据，不是实时刷新率。

CPU 重规划结果：两个箱体均恢复出可信局部观测面，但它们的完整不确定性包络在原位检查中相交102.026 mm，返回 `TARGET_PAYLOAD_INTERSECTS_AT_REST`。没有进入候选生成、IK 或路径搜索，未生成完整计划。该相交量描述保守模型包络，不能解释为真实纸箱互相穿入102 mm，也不能据此声称现场物理上无解。没有缩小包络、移除邻箱或放宽碰撞规则。25个现场物体加两个箱体全部保留。原报告的0.25 s是早期拒绝检查返回时间，不是成功 PLAN_READY 时延。

查看器现在可显示候选生成前的目标级失败；每个目标使用自己的同帧掩码，其他恢复箱体作为邻箱保留，不伪造缺失逆解或轨迹。新增“实测起点”“顺时针180°”“查看本次规划拒绝位置”按钮，灰/粉箭头显示旋转前/后的工具+Y，青点显示实际平面的深度支持，黄色/红色透明体显示含推断部分的不确定性包络。页面已生成并打开：

```bash
xdg-open outputs/field-replan-20261118-new-start/viewer/index.html
# 对另一份已经完成离线回放的记录生成同类起点预览：
.venv-workbench/bin/python tools/preview_field_start_rotation.py \
  outputs/field-replan-20261118-new-start/replay --j6-deg -180 \
  --output outputs/field-start-preview-new
# 重新推理必须显式选择配置；输出目录必须是新目录。
ECO65_TIMING=1 .venv-workbench/bin/python tools/replay_single_carton_record.py \
  outputs/single_carton/a9a1eaac54624c93ad96c5e35d33166b \
  --config outputs/field-replan-20261118-new-start/offline-cpu.json \
  --detector config --planning-deadline-s 120 --output outputs/field-replay-new
```

随后按用户提供的新地址验证 GPU SSH 与已有密钥，将端口41684保存到被忽略的 `outputs/workbench/remote_detector.local.json`，旧配置已备份；没有保存密码。当前已打开工作台仍持有启动时配置，需要正常关闭并重启才能应用新端口。用户后续要求“不用GPU复算”，因此不再启动 GPU 复算，也不以 GPU 结果替换本轮 CPU 交付；已经启动的那次进程在检查停止请求时已结束，原始产物和说明保留在 `gpu-replay/user-cancellation.json`，不作为本轮结果依据。

验证：规定 Python 定向集605 passed；离线回放及指标补充组8 passed（与前组有重叠，不相加）；`m710_clearance_test optimized/reference`、单箱原生组件与配置静态检查通过。浏览器核对两个目标、六个显示状态，FK最大矩阵差1.110e-16，q_start不变，715实体/12杯/27现场物体与箱体保留，起点和包络截图已检查。证据见 [本轮记录](evidence/realtime_20261118_new_start/report.json)。原位旋转路径、完整新箱位抓放、完整规划2 s目标和实机验收仍未验证 / NOT_RUN。

## 2026-11-18 上下叠箱的 102.026 mm 包络误判诊断

用户确认两箱上下叠放，上箱分割准确。本轮直接使用上一节同帧 RGB-D、掩码、q_capture 及标定，不重新推理、不使用 GPU，也不采集或控制设备。原始文件 SHA-256 未改变。恢复配置仍为已有 170×128×101 mm 尺寸先验及 8 mm 容差；未把此配置改称本轮实测尺寸。

102.0260905 mm 可由原位 SAT 查询精确复现，它是**两个保守外包盒的交集**，不是测得的纸箱穿入量。原算法保留上箱12组、下箱16组尺寸排列、面内转角和中心区间，再分别求一个共同外包盒。上箱包络 base_link 高度273.7～472.4 mm，下箱150.5～377.5 mm；可信观测面点的高度分别为318.1～429.6、219.8～309.5 mm。后者仅描述可见局部平面，不能当成完整箱体的真实上下边。

造成过大包络的具体信息损失：

- 边缘只在约2像素邻域寻找有效远背景，上箱轮廓附近恰有深度缺口，结果没有识别出可信棱边。分割掩码正确并不保证这些位置有有效深度。
- 侧面的法向用于约束面内转角，但侧面截距/与主面的交线还没有用于约束箱体位置。
- 两箱独立恢复，没有联合约束“上箱底面—下箱顶面”的叠放关系。下箱可见桌面高度217.028 mm，但顶部被挡，原 `NO_CREDIBLE_TOP_EDGE_FOR_RESTING_HEIGHT` 保留了未绑定支撑的中心区间。
- 合并中心、尺寸和转角的不确定性时，独立极值扩大了包络；原失败分支又将包络阻挡归为“全部候选不可行”，超出了该几何证据能证明的范围。

本轮修正：保留原直接深度边缘，并在最多12像素的短深度缺口两侧寻找平面/远背景证据；搜索线上出现近处遮挡则否决。每条新增边缘保留完整位置区间和角度余量，不把中点当作精确实测棱边，不以图像边界或遮挡轮廓代替真实棱边。求外包盒时保留已有边缘的外侧界限，避免再次组合独立极值而丢掉约束；这种区间证据不能单独认证承托高度。完整尺寸、碰撞间隙和对规则未改变。

同一现场帧的上箱新增3条区间边缘，保留6组假设；下箱保留16组。新包络 SAT 交集为88.653 mm，**剩余歧义没有被解决**。规划继续保留包络碰撞门，但返回 `GEOMETRY_UNCERTAINTY_OVERLAP_AT_REST` / `GEOMETRY_UNRESOLVED`，明确不证明实物穿入或所有假设均无解；候选未生成时记录 searched=0、unsearched=`ALL_NOT_GENERATED`，不虚报全部搜索过。独立接触几何检查两箱各产生32个观察面候选，所有预计密封轮廓仍通过可见支持区域检查；这不是完整计划，也不是该规划门后的候选搜索。

离线线性约束诊断另找到8对原保存转角分组存在不穿透的尺寸/位置组合，满足原先编码的中心区间、尺寸容差、观测面3 mm噪声范围及下方观测支撑高度。其中一组在三维图中显示。它只反证“原约束必然导致两箱穿入”，不是实测唯一位姿；该组 SAT 分离仅1.061 mm，不能声称满足保留的2.4 mm碰撞间隙，更没有完整带载路径认证。没有将该示例替换成生产碰撞模型。

诊断入口（输出目录必须新建，不使用相机、模型、机器人或真空接口）：

```bash
.venv-workbench/bin/python tools/diagnose_field_envelopes.py \
  outputs/field-replan-20261118-new-start/replay \
  --output outputs/field-envelope-diagnosis-new
xdg-open outputs/field-envelope-diagnosis-new/index.html
```

本轮图在 `outputs/field-geometry-diagnosis-20261118/viewer-final/index.html`，可切换旧包络、补入边缘约束后的包络、一组不穿透假设及仅观测面；同时显示原图同帧掩码。界面明确区分实测点、推断包络和非唯一示例。

验证：规定 Python 定向集 **610 passed**，包含3个带轮廓深度缺口的完整真值长方体包容测试、近处遮挡边缘否决、包络歧义与搜索计数测试；原部分出视场/遮挡用例继续通过。`m710_clearance_test optimized/reference` 和单箱原生组件 **PASS**，后者检查491个最终发送点。浏览器四种模式、点云和数字绑定通过，旧包络及不穿透示例截图已检查。首次新增假设备测试因缺少 partial-observed 能力声明而失败，补齐夹具后通过；生产能力检查没有弱化。证据见 [几何诊断记录](evidence/realtime_20261118_cuboid_diagnosis/report.json)。上下叠箱的联合几何恢复尚未实现，本帧完整路径仍未通过；2秒成功规划目标、硬件动作及实机抓放均未验证，物理验收 **NOT_RUN**、执行许可0。

## 2026-11-19 用户确认不同箱型、尺寸未知：停用不适用的共同先验

用户先确认“两个箱子尺寸不同”，随后确认“尺寸未知”。历史任务 `7ccdb15e` 的170×128×101 mm不能作为当前两箱共同尺寸的依据。上一节102.026 mm、88.653 mm及不穿透示例仍是原假设模型下的数学结果，但其箱型适用性未经确认，不能据此保证当前箱体的包络保守性。历史产物原样保留，新的诊断不再将这些数值当作当前现场几何。

默认现场配置新增 `geometry.size_policy=UNKNOWN_PER_INSTANCE` 和本次用户确认依据。历史尺寸、容差和来源保留，恢复与未恢复邻箱的隐藏深度估计均不再使用它们。旧记录及明确已知同箱型的配置可显式使用 `CONFIGURED_SINGLE_TYPE`；该模式继续保留尺寸/容差/来源校验。配置或旧工作台进程不会热更新，按正常停止/持箱处理流程结束旧会话后重启才应用。

新的观测分支从每个同帧掩码独立提取深度连续支持点、可信局部平面及可见覆盖范围。部分出视场、遮挡、深度缺口不自动成为“整箱不完整”的拒绝理由。`observed_face` 可用于实际 CAD 密封轮廓检查，但 `full_dimensions_m=null`、`full_extent_bounded=false`；没有根据掩码矩形或一张局部平面伪造厚度/中心/完整长方体。生产工作台仍持续采集、检测、分割；准备观测会保存这些表面证据和密封几何候选，显示 `GEOMETRY_UNRESOLVED`（实测面已保留，箱体尺寸未解）。

原位场景保留实测障碍，并记录 `unbounded_carton_indices`、`INCOMPLETE_INSTANCE_EXTENTS`、`hidden_carton_volume_assumed_clear=false`。不存在用历史箱型给未知邻箱限定一个过小厚度，或将未知完整体积当作空旷的动作。此场景不能进入路径搜索；规划入口直接说明 `UNBOUNDED_INSTANCE_GEOMETRY`，未搜索候选不标为物理无解。已知箱型的既有完整包络、全部715实体/12杯、2.4 mm对间隙、40 mm工具预留、压力/停止及执行锁规则均未调整。

本轮仍使用2026-11-18的同一冻结帧和原分割，无模型重推理、GPU或设备调用。实际保留上箱局部面覆盖约111.0×172.2 mm、下箱146.2×89.8 mm；这些是局部支持点范围，**不是两个纸箱的长宽高**。两面各生成32个密封几何候选，所有预计密封轮廓仍有深度支持；上箱预计密封2/3/4/6杯，下箱1/2/3杯，仅是共同阀门下的被动密封预测。没有独立杯控制、完整路径或执行许可。

```bash
# 只读复用已保存 RGB-D 和掩码；输出必须为新目录。
python3 - <<'PY'
import json
from pathlib import Path
c=json.loads(Path('configs/workbench/single_carton.example.json').read_text())
c['geometry'].update(size_policy='UNKNOWN_PER_INSTANCE',size_policy_basis='EXPLICIT_HISTORICAL_SURFACE_ONLY_DIAGNOSTIC')
p=Path('outputs/workbench/single_carton.unknown_diagnostic.json')
p.parent.mkdir(parents=True,exist_ok=True)
p.write_text(json.dumps(c,ensure_ascii=False,indent=2))
PY
.venv-workbench/bin/python tools/diagnose_unknown_cartons.py \
  outputs/field-replan-20261118-new-start/replay \
  --config outputs/workbench/single_carton.unknown_diagnostic.json \
  --output outputs/unknown-carton-surfaces-new
xdg-open outputs/unknown-carton-surfaces-new/index.html
```

本轮更正图为 `outputs/field-mixed-sizes-20261119/viewer-final/index.html`：默认只显示两个实测面点云和同帧掩码，各箱完整尺寸明确标为未知，当前箱体重叠量不计算。历史包络及示例不会被当成当前恢复结果显示。混合箱型的完整边界恢复、上下箱联合约束与完整带载规划仍未完成；物理抓放 **NOT_RUN**，许可0。

定向验证：规定集合 **617 passed / 0 failed / 0 skipped**（213.70 s）；新增覆盖不同大小局部面不继承共同先验、部分出视场/遮挡时保留真实 CAD 密封候选、策略依据校验、未定邻箱体积阻止规划、完整准备观测流程保存候选并显示未解状态。旧启动测试要求“尺寸先验可配置”的文案与本轮默认策略冲突，更新为必须显示“各箱尺寸未知”和“历史统一箱型先验停用”，原无硬件连接/无执行检查保留。早期失败和一次SIGTERM中断日志保留，未计为通过。`m710_clearance_test optimized/reference`、单箱原生组件（491发送点）与配置静态检查均PASS。

本节 `5f80edf8` 的 UNKNOWN 模式还实际执行了 `tools/replay_single_carton_record.py ... --detector saved --scene-only`：保留54个实测障碍体和6个工作范围边界，完整箱体数0，`unbounded_carton_indices=[0,1]`，不应用任何共同尺寸或邻箱厚度先验，状态为 `GEOMETRY_UNRESOLVED`，搜索0、许可0。源RGB-D、掩码及旧几何文件哈希均不变。新版浏览器点云开关、零完整箱体网格、候选数及未知尺寸字段通过检查，截图已检查。详见 [不同箱型更正证据](evidence/realtime_20261119_unknown_mixed_sizes/report.json)。这不是下节 MEASURED 的结果；本次文档归位没有重新执行该现场回放。

### 2026-11-19：接入逐箱无先验尺寸恢复

用户进一步指定“箱体尺寸计算走不依赖先验那条路”。本轮将现场入口默认策略改为 `MEASURED_PER_INSTANCE`，显式调用已有 `recover(..., prior=None)`，无论历史 `prior_size_m` 是否仍在文件中，都不参与该分支或未恢复邻箱的厚度估计。原 `UNKNOWN_PER_INSTANCE` 的仅面诊断与已知箱型分支保持可显式选择，不作为测量失败的自动回退。

测量失败会保留具体原因，并另存深度连续实测面和实际 CAD 密封候选。成功箱体与未解邻箱并存时，任务保存全部证据后进入 `GEOMETRY_UNRESOLVED`，规划和执行按钮均不可用；避免将一箱测量成功误当作整个碰撞场景已经完整。界面当前模式标明“逐箱 RGB-D 实测尺寸（不依赖先验）”，成功尺寸不再被标作“含先验不确定性”。

对原 `a9a1eaac` 任务 CPU 分割回放记录再次计算，仅复用原 RGB-D 和掩码，未重新推理、未使用 GPU：

- 上箱（检测0）：视觉测量约 **169.51 × 110.83 × 109.85 mm**；主/侧平面 RMS 约0.727/1.095 mm，点云表面一致性99.05%。这是无先验算法估计，实际箱体尺寸未知，尚无实物量具对照精度验收。
- 下箱（检测1）：`BOX_THICKNESS_UNOBSERVED_SHOW_SIDE`；保留约146.25 × 89.83 mm的可见面覆盖范围，**该范围不是整箱尺寸**，厚度不补猜测值。用户随后确认当前无法露出侧面。
- 两箱各有32个实测密封几何候选，上箱预计2/3/4/6杯、下箱1/2/3杯；均不是完整规划。场景保留48个实测障碍、1个实测支撑对象与6个工作范围边界，邻箱1仍是未定完整体积。没有产生新的两箱重叠数值；搜索0、未搜索ALL_NOT_GENERATED、许可0。

625项规定定向测试全部通过（231.31 s）；覆盖不同箱型独立测量、历史尺寸/容差不影响无先验结果、无先验配置不要求箱型尺寸、单面/出视场/遮挡/深度孔洞仍保留实测接触候选，以及成功箱体与未解邻箱同时存在时阻止误规划。`m710_clearance_test optimized/reference` 和单箱原生组件检查均PASS，后者491个发送点，仅为离线组件检查。配置静态检查无问题。

旧工作台已通过正常关闭流程结束。本机已启动新的现场配置 `outputs/workbench/single_carton.prior_free_cpu.20261119.json`，使用本机CPU、2线程，不使用GPU；模型复用，采集仍为640×480@30fps，CPU推理刷新率远低于采集率。该本机文件依据用户“已修复并完成复核”确认将事故锁设为false，并保留原事故依据；这是用户现场确认，未修改标定原文件或伪造自动验收。共享示例的事故锁仍保留，现场使用下面的本机入口。

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config outputs/workbench/single_carton.prior_free_cpu.20261119.json
```

重新启动后的实际相机、检测、分割已有同帧结果。机器人专用保存配置已加载，路由192.168.1.100/eno1→192.168.1.19正确，但网卡载波为off、控制器连接失败；真空串口打开后尚无压力心跳。未用旧关节/压力填充新会话，新任务观测尚未取得，未开始运动、未写阀门。现场恢复连接后可点各设备重试，再点“准备观测”；若下箱仍无厚度证据，任务会保留接触面并说明几何缺项。完整带载路径、2秒成功规划目标和实机抓放仍未验证，物理验收 **NOT_RUN**。证据见 [无先验路线记录](evidence/realtime_20261119_prior_free/report.json)。

### 2026-11-19 后续现场验证：连接恢复与准备记录修复

用户确认通电接好后，实查eno1载波恢复、路由仍为原保存配置；通过工作台重试机器人后，实际关节、压力心跳和相机全部显示数据有效，控制器无报警。地址、串口、网络配置、标定及阀门协议均未改动。上节的连接失败保留为当时状态，现已恢复。

继续“准备观测”发现 `Object of type int64 is not JSON serializable`。任务 `87736fb9`、`ac5706f3` 的原始RGB-D及失败记录保留。原因是靠近目标的障碍细分分支执行 `refined_boxes += which.max()+1`，使场景统计计数变成NumPy整数；写入严格JSON时失败。改为显式整数计数，未修改细分物体、尺寸、位置或碰撞规则。已有真实几何夹具新增整场景严格JSON写入检查，改前复现同一错误，改后通过。

同时将已校验同帧的 `detections.json` 与 `segmentation.npz` 保存提前到场景构建前。场景失败仍保留原检测来源和掩码；同步及异步记录模式下的场景失败均有假设备覆盖。回放脚本在没有 `geometry.json` 时读取已保存 `detections.json`，核对原始观测ID并纳入源文件哈希，错帧拒绝，不重新调用推理模型。

验证：规定定向集 **629 passed / 0 failed / 0 skipped**（238.43 s）；focused回归16项通过；optimized/reference和单箱原生组件均PASS，后者491个最终发送点。仍未把开发检查计作实机抓放。

正常重启后，使用同一无先验本机CPU配置完成新的现场准备任务 **`b8992c098eaa476284ed1c107d7e7ccf`**：

- 三设备自动连接后均收到有效新数据；实际 `q_start=[81.140,-12.513,-54.221,-15.754,85.987,-5.149]°`，`q_capture=[81.142,-12.516,-54.220,-15.754,85.987,-5.155]°` 分别保存，未使用旧关节或推理结束姿态替换。
- 原始RGB/depth为1076/1073；检测、掩码、几何及场景观测ID一致。模型由本机CPU复用，没有GPU推理或真实运动。
- 上箱无先验视觉估算 **167.987 × 110.449 × 109.081 mm**，与先前保存帧的169.512 × 110.832 × 109.850 mm是不同观测结果；实际尺寸仍未用量具核验。
- 下箱仍为 `BOX_THICKNESS_UNOBSERVED_SHOW_SIDE`，保留32个深度支持的密封几何候选；未推断完整隐藏厚度、未给出两箱实物重叠结论。用户已说明当前无法露出侧面。
- 实际场景触发1次靠近目标的细分，产生9个细分障碍，严格JSON成功保存；任务正确进入 `GEOMETRY_UNRESOLVED`，规划与执行按钮禁用，三设备和实时视觉继续运行。

当前本机入口仍为 `outputs/workbench/single_carton.prior_free_cpu.20261119.json`。新的完整观测记录已可直接回放；完整带载规划、抓取返回释放、2秒成功规划目标均未完成，物理验收 **NOT_RUN**。见 [现场验证与修复记录](evidence/realtime_20261119_prior_free_live_fix/report.json)。

MEASURED 历史回放以 [prior_free/report.json](evidence/realtime_20261119_prior_free/report.json) 为准：检测 0 恢复成功，`unbounded_carton_indices=[1]`。UNKNOWN 的 `[0,1]` 记录已归回前节；本次未重算历史记录或声称完成现场规划。

### 2026-11-19 仅当前 RGB-D：取消所有箱子必须完整测量的门槛

用户明确说明卸箱初期可能只能看到最上层正面，并要求“没有额外测量，先用当前 RGB-D”。本节覆盖此前“存在任何未解邻箱就进入 GEOMETRY_UNRESOLVED”的限制；不再要求用户露出每个箱子的侧面或提供统一尺寸。仍保留无先验策略、实际 CAD 接触覆盖、完整载荷范围和所有碰撞/执行检查。

实现分三层：

1. **接触面**：只用冻结 RGB-D 的可信局部平面、深度连续支持点和真实吸盘密封轮廓。局部面足够就保留候选，不以整箱尺寸唯一可解为前提。
2. **目标载荷**：优先逐箱实测；失败后用同帧自由空间约束面内连通范围和向内厚度上界。前景遮挡、深度缺失与图像边界都保留为可能区域。两个范围都有证据时，以 `BOUNDED_CUBOID_UNION / RGBD_VISIBILITY_INTERVALS` 进入既有完整原生规划；`full_dimensions_m=null`，界面明确显示包络而非实测长宽高。新包络不能吞并其他实例的观测点，也不能凭包络底面产生支撑接触权限。
3. **静态邻箱**：未知尺寸不再全局阻止规划。保留其可能体积在强制工作范围内的全部体素，只有同帧证实为空的完整体素可以排除。`USER_CONFIRMED_CLEAR` 和旧箱型都不用于缩小它；工作范围仅裁剪静态查询域，不是假设的箱体末端，更不用于附着载荷。未知体素的顶面不能获得支撑接触豁免。缺少这种障碍表示的旧记录仍会被拒绝。

当前场景未引入新的尺寸、墙体、货位或桌面测量。单箱入口仍沿用本机配置：

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config outputs/workbench/single_carton.prior_free_cpu.20261119.json
```

现场操作：等待三类设备新数据有效 → 点“准备观测 / 锁定起始点” → 查看实际接触面、实测尺寸或标明的不确定包络 → 为可规划的目标点“规划所选箱”。存在尺寸未知邻箱时该入口仍可用；它们继续参与本次目标的检查。只有完整计划和全部执行条件通过后，才由操作者点“执行单箱”。未能界定目标载荷或可能邻箱体积确实阻挡路径时，会报告具体几何不确定性；不能将其解释为真实物体穿透，也不能跳过碰撞来执行。

保存现场任务 `b8992c098eaa476284ed1c107d7e7ccf` 使用原始掩码复算，未使用GPU、模型重推理或现场写命令：上箱仍约167.987×110.449×109.081 mm；下箱面内可能区域连到未观测射线，向内3 m搜索内也没有自由空间证据，因此没有厚度上界。搜索距离没有被当成尺寸。下箱保留359个可能体积障碍（10069个原始体素），另保留54个实测障碍、1个原有实测支撑和6个工作范围边界。源记录全部哈希不变。

上箱现已进入逐目标预检查，未再被“邻箱尺寸未知”全局门拦截；原位载荷包络与下箱可能占用格的交集仍阻挡后续搜索。记录为 `GEOMETRY_UNCERTAINTY_OVERLAP_AT_REST`，不是两个真实箱体重叠；候选/路径searched=0、unsearched=ALL_NOT_GENERATED、infeasible=0、许可0。几何与场景（含模型构建）15.91 s，原位检查0.19 s；没有生成PLAN_READY，不能据此声称满足2 s成功规划目标。

验证结果和随后现场入口检查见 [本轮证据](evidence/realtime_20261119_rgbd_extent_bounds/report.json)。完整抓取、带载返回、释放及单面箱实机验收仍为 **NOT_RUN**。单面只能证明接触面时，单帧存在无法消除的隐藏厚度歧义；本轮保留并明确报告这一限制，没有伪造完整实测尺寸或成功抓放。

本轮定向检查 **644 passed / 0 failed / 0 skipped**（249.15 s），包含15个新增几何、邻箱和运行入口用例。optimized/reference 以及单箱原生组件均PASS，后者491个最终发送点。初次界面文案断言失败与一次既有截止时间竞争均保留日志：更新旧文案断言；超时测试认可原有外层TASK_WAIT竞争结果，原50 ms返回限值、工作进程废弃检查均保留，并增加已进入假工作进程和零许可断言。未更改生产截止时间、压力或停止阈值。

同日新现场入口验证：正常重启后，相机、机械臂和真空全部自动连接并收到有效数据，无控制器报警。新任务 `31e4617215ca46fba96f8087181f3e47` 冻结RGB/depth 696/693，实际q_start与q_capture分别保存。该帧检测编号1的上箱恢复约168.213×110.617×103.968 mm；编号0的下箱保留32个实测密封候选（预计1/2杯），隐藏范围仍未定。任务进入 **TARGET_READY**，显示“可规划目标1箱；另1箱尺寸未解，已作为保守现场障碍保留”，规划按钮正常启用。编号属于各自冻结帧，不与前一记录的编号混用。

通过正常“规划所选箱”入口检查后，仍因下箱可能体积与上箱原位载荷包络冲突而拒绝；这次记录的73.927 mm是格体与包络的数学交集，不是实物重叠量，也没有生成候选路径或完整计划。执行按钮未调用，运动/阀门动作0，物理验收NOT_RUN。三类设备和后台CPU检测/分割继续运行，新工作台保持打开。最终保存帧复算另外记录几何恢复0.495 s、模型构建4.889 s、自身过滤器构建1.594 s、场景8.777 s；总观测几何/场景15.78 s，规划入口到搜索结束2.812 s（原位检查0.396 s），未到达PLAN_READY，不能视为成功规划p95。


### 2026-11-19：两箱叠放关系、局部点云包络与脱离路径

针对任务 `67dbcaac6b39429f91ccaa7b98136b4b` 的74.274 mm报告，本轮复用原始RGB-D、原分割及q_capture/q_start，不调用GPU。上箱仍由无尺寸先验的可见双面恢复约168.388×110.938×107.696 mm；下箱的可靠深度支持只给出正面，未声称测出其隐藏厚度。错误阻挡来自把下箱可能体积向上扩入上箱所在区域，而非实物相交的测量结论。

新增显式 `geometry.confirmed_stack_support`：共享示例为 `DISABLED`；本机现场配置根据用户“两箱上下叠放、实际未相交”的说明选择 `USER_CONFIRMED_TWO_CARTON_STACK` 并记录依据。仅在同一冻结帧恰有两个实例、上箱完整边界由RGB-D测得、下箱有可信局部面且接缝深度相符时应用。原记录有1185个接缝对应点。支撑面来自**已测上箱底面与用户确认的叠放结构关系**，不是传感器测出了下箱顶面。缺少这些条件时保留原有未知体积，不推断支撑关系。

下箱所有可能隐藏体积在强制工作范围内继续保留，仅以上述支撑面约束其顶部。工作范围不是箱体末端、该静态大包络不是下箱尺寸，也不用于下箱附着。下箱实例已被支撑体包含的深度点去重；其他实例、未标记物及体外点保留，既有实测支撑处理继续生效。原生检查将既有载荷—支撑物规则推广至同一近水平有向平面：绑定对象ID、目标ID、依据及对象几何顶面，每个载荷角点仍遵守0.4 mm穿入限制。此权限仅随本目标的带载脱离请求使用，不豁免机器人、吸具或其他障碍。

另查出上箱右侧37个未归属深度点距箱体至少8.000 mm；合并后大盒体却给出0.06 mm间隙。近目标细分现在也覆盖小正间隙的合并体，不再只在已经相交时细分。所有原始点、3 mm点云包络余量及2.4 mm对间隙保持；没有用删点或降低间隙制造通过。

新现场帧还定位到恢复器的另一处错误：它用侧面法向确定方向，却只用正面矩形宽度作为载荷宽度。纸板侧面相对正面有约1.6–2.2°倾斜，实际深度点可伸出旧矩形5–7 mm，于是纸箱自身侧面被重新作为障碍。现在保留原正面接触矩形，将所有通过原侧面平面判据的观测点纳入侧向碰撞外包；只扩展、不缩小，不增加密封面积。原记录正面/厚度尺寸仍为168.388×110.938×107.696 mm，带载检查外包为175.212×110.938×107.696 mm；两者分别记录在geometry.json，界面显示“实测外包尺寸”。原生请求逐项核对外包扩展量、接触矩形和相对位置，旧worker明确拒绝。新增倾斜侧面及原生附着测试覆盖这一差别。

保留原接触候选及优先级，原OUTWARD/DIAGONAL_45/LIFT_FIRST之后增加沿箱体近竖直棱边的纯抬升尝试。先用原LIFT_FIRST的50 mm高度，必要时尝试一箱可见垂直高度加10 mm；随后仍单独原生规划带载返回q_start。整段仍是带完整载荷的LIN，新的几何路径必须通过全部最终发送轨迹检查。Python与原生复合流程使用相同目标。原最低0.1倍率仍出现J3速度0.10066 rad/s的保存证据，因此两条流程均增加0.05倍率尝试，速度0.10 rad/s、加速度0.20 rad/s、5 ms周期未改变。

可选 `native_solver_budget_s` 缺省25 s、上限100 s；本机离线对照曾用90 s验证25 s耗尽是否只是搜索不足，仍未找到完整路径，现场入口保留25 s。统一 `planning_deadline_s` 仍优先限制每次请求，搜索耗尽不等于物理无解。这是搜索时间配置，不是运动/监控阈值；增加预算不满足2 s延迟目标。

本机启动入口（本机CPU，已保存的三设备配置，启动不运动）：

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config outputs/workbench/single_carton.prior_free_cpu.20261119.json
```

现场配置增量：

```json
{
  "geometry": {
    "confirmed_stack_support": {
      "mode": "USER_CONFIRMED_TWO_CARTON_STACK",
      "basis": "USER_FIELD_CONFIRMATION_2026_11_19_TWO_DIFFERENT_CARTONS_STACKED_LOWER_SUPPORTS_UPPER_NO_OCCLUSION_CURRENT_RGBD_ONLY"
    }
  },
  "native_solver_budget_s": 25.0
}
```

只合并这些字段，保留原geometry中的无先验策略及其他配置。新现场仍须新冻结观测，旧回放不签发执行许可。正常流程为三设备数据有效→准备观测→规划所选上箱→查看完整轨迹→操作者点击执行单箱；失败/停止/关闭继续保持可能持箱的真空。四段完整规划、实际抓放与性能结果见[本轮证据](evidence/realtime_20261119_stack_geometry/report.json)，离线结果不替代实机验收。

合并前基线：上述实现以 `265b29f` 为源。保存双箱帧完整复算32个候选，21个通过终态筛选，仍未得到四段完整计划；225.17 s结束，10个候选为原生搜索预算耗尽，22个候选有明确拒绝记录。执行许可0、实机动作0。新工作台任务 `22424c791df4426cb0e26bc86efd9800` 的本次画面未恢复完整上箱，保留接触面后停止，未自动替换冻结图像。提交前发现origin新增5个提交至 `035b388`，涉及实测/外包分离、邻箱阴影、承载能力、起点/时效与现场CAD；最终交付以合并后的验证为准，下面的后续段落覆盖冲突实现。

## 2026-11-19 起点核对与冻结计划有效期（步骤 1）

从最新 `265b29f` 开始，在隔离工作树实现。示例恢复 ENABLED，采用用户给定的 0.02°（静止抖动最大 0.006°）；缺省仍为 0.01°。Cycle 与 Supervisor 使用同源策略，首段包摘要包含策略。非续接起点始终受 0.5° 硬上限约束，关闭核对也不能豁免。续接保持既有到达判据。

120 s 从冻结曝光的主机单调接收时刻计时，最终启动屏障超时返回 `PLAN_EXPIRED_REPLAN`，作废许可；界面显示剩余秒数。现场只能收紧本轮批准的参数。执行锁、腕部限制、姿态偏好、工具预留间隙及设备阈值未改。

既有 Humble 隔离环境构建安装通过，optimized/reference 间隙检查通过；原生组件检查通过（491 个最终发送状态，仅离线检查），配置静态检查通过。定向集合首次 630 通过、4 失败；补齐并核验快照/LFS 资源后复测修复 3 项，剩余尺寸先验回归在未修改的 `265b29f` 上同样失败，未掩盖。最终改动策略回归 47 通过；本步新增 12 个测试。完整集合合并结果为 633 通过、1 个既有失败，并非全绿。

证据见 [report.json](evidence/20261119_start_expiry/report.json)。远程检测器私有 SSH 参数未写入共享示例；静态检查使用被忽略的本地检测器开发配置。设备连接、运动、阀门写入和物理验收全部 `NOT_RUN`。未验证现场执行效果。用户尚未提供真实 STEP，后续完成步骤 5 合成检查后等待真实文件，连续作业入口本轮不实现。

## 2026-11-19 实测箱体与保守碰撞包络（步骤 2）

`measured_box` 保留原始实测面、侧面、尺寸及位姿，用于接触候选、点归属和显示；顶层 `size_m/pose/corners_m` 与 `collision_envelope` 一致，用于原位、附着、带载路径和释放检查。目标点归属继续使用原有余量，并排除其他检测掩码，不能用放大的包络吸收邻物。质量采用配置值，惯量采用实测尺寸的均匀箱体估计，重心偏移明确表达在包络坐标中。它不是实测质量分布。

边界证据写入 `uncertainty`。直接深度边界采用 1 个像素足迹与 3 mm 深度噪声项；缺失边界采用用户给定的最长边 0.40 m 上界。该包络是多个可能箱体的并集外包络，可能大于单个箱体的 0.40 m 边长。108 个独立光线投射真值组合全部包含真值，详细数值见 [synthetic-sweep.json](evidence/20261119_measured_envelope/synthetic-sweep.json)。

| 轮廓深度缺失 | 组合数 | 完整包含 | 最大单边外扩 |
|---|---:|---:|---:|
| 0 像素 | 27 | 27 | 11.03 mm |
| 1 像素 | 27 | 27 | 347.55 mm |
| 2 像素 | 27 | 27 | 347.42 mm |
| 3 像素 | 27 | 27 | 347.12 mm |

无缺失的 27 组均有边超过 6 mm：斜视侧面的一个像素跨越更大的深度，加上噪声项与有噪观测极值，不能统一截断为 6 mm。缺失边界的大包络可能阻止规划，这是缺少边界证据的结果。常量仅由本次合成扫描校核，尚未完成现场深度误差标定。

叠箱夹具中，上箱底面止于实测支撑平面（世界高度 −0.09 m），穿入量为 0。当前包络采用 OBB；若实测支撑面不与其面平行，会拒绝并记录 `MEASURED_SUPPORT_REQUIRES_CLIPPED_ENVELOPE`，尚未实现任意倾角载荷的多面体裁剪。原生接触判据独立绑定实测面，碰撞体保持完整包络，既有密封和唇部穿入阈值不变。

重建安装、optimized/reference、原生组件和配置静态检查均通过。本地数值检查 123 通过；远程完整集合首次 745 通过、2 失败，更新实测尺寸语义的旧断言后相关 120 项复测通过。合并为 746 通过、1 个步骤 1 已记录的基线失败。见 [report.json](evidence/20261119_measured_envelope/report.json)。设备连接、运动、阀门写入、物理验收全部 `NOT_RUN`。

## 2026-11-19 未恢复邻箱的有限阴影与范围外检测（步骤 3）

无先验策略的邻箱深度改用用户配置的 `geometry.max_carton_extent_m=0.40`，不继承历史箱型。去掉目标归属点、机器人自身点及深度飞点后，沿相机视线向后延伸为 `neighbor_shadow`。用户清空未知空间的确认不能清除这层障碍；OCCUPIED 保留相同区域的占用并记录 `unresolved_neighbor_extrusion`。阴影与目标和已证实的机器人凸实体相交时，使用半空间裁剪保留所有外侧体积，不删除整个相交体素，也不添加碰撞豁免。

完整保守包络的上箱可以进入 `TARGET_READY`，未恢复下箱仅作障碍，实测面和密封候选继续保存；后续原位和带载路径仍可能被阴影拒绝。`GEOMETRY_UNRESOLVED` 仅表示没有任何完整目标。缺少有效深度、不能建模的邻物仍会在规划场景审计中拒绝，不会静默当作空闲。

检测的全部有效深度点位于工作范围之外时标记 `OUTSIDE_WORKSPACE_IGNORED`，不构建障碍且不阻止范围内目标；6 面工作范围边界保留。合成的上下箱、范围外误检、目标裁剪和机器人裁剪均通过。原生两种间隙检查、491 状态组件和配置检查通过；完整定向集合 749 通过、1 个既有失败，额外机器人裁剪加入后 4 项阴影复测通过，合并为 750 通过、1 个既有失败。证据见 [report.json](evidence/20261119_neighbor_shadows/report.json)。

本节更新算法结论，没有重新使用设备或执行现场回放；当前现场是否可以生成完整路径仍未验证。连接、运动、阀门写入与物理验收全部 `NOT_RUN`。步骤 7 中与本节直接相关的历史记录归位和 UNKNOWN 命令修正已完成；连续作业与释放提示仍待后续步骤。

## 2026-11-19 吸附持力预筛与发送状态复核（步骤 4）

按每个预计密封杯的原始 CAD 环内多边形面积计算吸力，被动未密封杯贡献为零。设计压差读取既有 `vacuum.hold_below_kpa`（−30 kPa），摩擦系数 0.5、安全系数 2、有效面积系数 0.8；依据是“工程估计，待现场拉脱试验标定”，现场只可收紧。分别检查实测箱体中心和碰撞包络中心，取法向、切向、线性分配剥离力矩的最不利值。

规划前按 1 g 筛选；不足者在原生预检之前记为 `HOLDING_CAPACITY_INSUFFICIENT`，计入已搜索候选。CLEAR 候选先按 ≥2、≥1.5、≥1 的裕度分档，再用原有姿态排序。C++ 校验分档与 Python 一致，原生复合路径没有另外的跨候选排序。规划后对 detach/return 全部发送状态的实际 FK 重心位置作时间差分，同时保留实际接触位姿的静态复核；失败即拒绝该候选，不发放许可。

典型合成工况：每杯 CAD 面积 0.00132749 m²，每杯设计吸力 31.8598 N。2 kg、100 mm 深箱、6 杯竖直侧吸时，总吸力 191.1588 N、切向重力 19.6133 N、切向裕度 2.4366、剥离裕度 2.19294；最不利杯拉力 7.2642 N，允许值 15.9299 N。1 杯既不满足切向要求，也无法平衡该偏心力矩，拒绝。水平面 6 杯的最小裕度为 4.8732。完整数值见 [candidates.json](evidence/20261119_holding_capacity/candidates.json)。

“上移 60 mm”测试采用用户于 2026-11-19 明确确认的“带法向加速度的竖直侧吸”工况。现有合成夹具使用 217 mm 深箱、0.2 m/s² 法向加速度：移位前剥离裕度 1.00631，移位后 0.99514，后者拒绝。这两个数值是接近持力极限的合成测试设置，不是现场测量或用户另行指定的数值。本次确认与已测试工况一致，仅补记依据，保留原测试结果。严格竖直侧面仅受重力时，沿重力方向平移不会改变力矩；另有回归确认这一事实，没有伪造静态上移导致剥离的结论。

本步构建安装、两种间隙检查、491 状态原生组件与配置检查均通过；定向 pytest 为 770 通过、1 个已在 `265b29f` 复现的基线失败。持力与规划定向复测 70 通过，本步新增 20 个测试，包含 Python/C++ 分档一致性。证据见 [report.json](evidence/20261119_holding_capacity/report.json)。所有数值均为工程估计和合成/假 IO 检查，未做现场拉脱标定、压力实测、实际运动或阀门写入，物理验收 `NOT_RUN`。本步不改变 12 杯几何、控制拓扑、密封规则、715 个碰撞实体或任何原有设备阈值。

## 2026-11-19 现场 layout STEP 静态层（步骤 5，等待真实 STEP）

用户确认真实现场 STEP 尚未提供。本步完成转换器、静态层接入及合成测试后停止，状态为 `WAITING_FOR_REAL_STEP`；没有使用演示或历史仿真布局替代。共享配置的 `site_layout` 保持 `null`，不会自动导入任何布局。

将后续真实文件放入被忽略的 `assets/layout/raw/`，在现有 OCP 环境中运行 `python tools/convert_site_layout.py`。只接受一个 `.step/.stp` 文件；零文件明确等待，多文件报错。沿用现有毫米单位白名单、XCAF 装配遍历和 SHA-256 约定，保留名称、装配实例和层级。输出位于被忽略的 `assets/layout/derived/<源哈希前16位>/`，其中 `site-layout.json` 是加载入口。每个零件保留显示网格；碰撞体使用解析 BRep 的保守 AABB，包含曲面真实极值，不依赖内接三角网格的凸包外缘。源文件和全部派生网格都参与哈希核验与计划输入绑定。

真实 `site_layout` 配置必须明确提供 `manifest`、`T_base_layout`、`transform_basis`、`excluded_part_ids`、`exclusion_basis`、`association_distance_m` 和 `association_basis`。机器人、工具、相机/支架的重复件及纸箱/货物等可移动件，需要根据真实零件清单显式排除；代码不会根据零件名猜测。其余零件全部保留为 `site_layout_cad`，依据固定标明 `CAD_DESIGN_GEOMETRY_NOT_FIELD_MEASUREMENT`。当前缺少真实变换、排除清单及点关联距离，未用历史仿真值填充。

冻结 RGB-D 的有效静态点剔除检测掩码与机器人自身点后，与 CAD 显示三角面的精确距离进行关联。非空关联点必须同时满足 RMS ≤5 mm、p95 ≤10 mm；超限或没有关联点返回 `LAYOUT_REGISTRATION_MISMATCH`。配准结果绑定源/派生哈希、变换、排除清单与配准参数，规划入口重新核对这些身份。本步保留全部 RGB-D 障碍，不做 layout 深度去重，因此没有通过宽松去重删除实测障碍。

路径搜索前，在原生 worker 中检查 `q_start` 与完整静态场景；没有接触豁免或附着载荷。合成远处 CAD 障碍通过，合成障碍与真实模型工具相交时返回 `START_POSE_COLLIDES_WITH_LAYOUT`，碰撞对记录为 `part_000/site_layout_synthetic_intersects_tool`，未发放运动许可。该门禁与步骤 1 的关节起点误差门同时存在。原生证据见 [native-layout-start.json](evidence/20261119_site_layout/native-layout-start.json)，它仅是合成离线检查，不是现场布局验收。

本地使用既有 OCP 7.9.3.1 缓存，12 项合成 CAD/配准测试通过，包括曲面包络、单位、装配、排除、哈希、RMS/p95、零文件/多文件和保留实测障碍。远程 Linux 假 IO 起点门禁 3 项通过。完整定向集合 776 通过、1 个既有基线失败、9 项因远程没有 OCP 而跳过；这 9 项已在上述本地检查中全部通过，跨环境去重后为 785 通过、1 个基线失败。optimized/reference、491 状态原生组件和当前配置静态检查均通过。未安装或升级环境。证据见 [report.json](evidence/20261119_site_layout/report.json)。

步骤 6 的会话级增量场景没有实现，`scene_max_age_s=300` 仅预留配置并校验不可放宽，尚未用于跨周期场景。因此增量/完整构建计时比较为 `NOT_RUN`，没有编造加速比。按本步停止条件，步骤 7 的释放位无支撑提示尚未实现；相关历史文档归位和 UNKNOWN 命令修正已随步骤 3 完成。连续作业入口本轮不实现。

真实 STEP 导入、现场 RGB-D 配准、实际布局起点碰撞、完整现场路径、现场深度误差与持力标定、任意倾斜支撑的载荷多面体裁剪均未验证或未完成。设备连接、运动、阀门写入及物理验收全部 `NOT_RUN`。等待真实 STEP 后再继续，不更改原有校准证据状态。

## 2026-11-19 叠箱恢复与最新五项更新的合并验证

保留本地 `63a77f1` 和 GitHub `035b388` 及其前四项更新。以下是合并后的行为，覆盖前面“实测外包尺寸”的合并前描述。原始标定、现场记录、模型资产、起点/时效门禁、承载估算和现场 CAD 入口均保留。本次继续使用当前 RGB-D；`site_layout=null`，没有导入仿真现场，也没有生成或假填真实 STEP。

`measured_box` 保存原可见面/侧面尺寸；`observed_side_geometry` 单独包含所有通过原侧面平面判据的侧面点，用于点归属，解决纸箱自身侧面被重新加入障碍的问题。它不增加接触面或密封面积，也不替代带载碰撞几何。带载检查始终使用顶层 `collision_envelope`，包含最新提交要求的完整边界不确定性。旧记录的侧向扩展格式仍有独立原生绑定检查，新记录使用最新 `measured_box` 绑定。

合并复算还发现：直接将 SAM 内所有非零深度用于尺寸极值，会把少量远背景/立体匹配离群值纳入箱体，将保存上箱放大至约 2.216 m。现在仅用原恢复器已经接受的同帧可见平面及原 2 mm 残差提取尺寸证据；未通过的深度不据此变成自由空间，仍经过现场障碍处理。保存双箱帧有 42,308 个受平面支持的深度点，2,600 个离面值不参与尺寸极值；新增带远背景污染的独立箱体真值回归仍要求完整真值包含。

本帧的可见面/侧面尺寸约 **168.388×110.938×107.696 mm**；完整可能体积外包仍为 **408.064×407.545×117.965 mm**。后者不是实物尺寸：左侧与底部缺少直接相邻背景深度证据，保留批准的最大边长约束。单独检查原有短缺深度带证据可找到上下边的有限区间，但不能把区间中点当作精确箱边或据此扩大支撑接触权限。不能将正确的 RGB 分割直接当作深度边界已测准。

明确的叠放关系仍可约束具有精确上箱底面的记录及合成测试；本次新的 `MEASURED_BOUNDARY_INTERVALS` 包络不满足这项条件，记录 `NOT_APPLIED`，不会用名义底面裁掉不确定体积。下箱无需先测出完整尺寸，其可见障碍及最多 0.40 m 的相机射线阴影继续保留。当前场景包含 612 个观测体、31 个邻物阴影体和 6 个工作范围边界。原位包络冲突仍阻止本帧下发运动；这不证明两只实箱相交，也不证明每个几何假设都无解。原位检查和预算报告已统一识别 `MEASURED_BOUNDARY_INTERVALS`、有限假设与邻物阴影，记录几何未解、路径未搜索，避免重新显示“所有候选均不可行”。

当前可见接触面单独生成 32 个候选，按保留的 2 kg 配置、CAD 密封面积及新增承载估算，有 9 个通过静态检查，最多使用 6 个预期密封杯。该结果不是拉脱试验，也不是完整路径通过。未改变 12 杯共阀/被动止回语义、压力门槛、715 个碰撞实体、2.4 mm 对间隙、40 mm 工具/机器人预留或运动限值。

启动仍使用上面的本机 CPU 命令及 `outputs/workbench/single_carton.prior_free_cpu.20261119.json`。三设备状态有效后，点击“准备观测 / 锁定起始点”，查看同帧的实测尺寸和碰撞包络，再点击“规划所选箱”。只有新任务产生四段完整计划、最终审查通过且尚未过期，才进入操作者执行步骤；保存帧离线回放不产生执行许可。下箱尺寸未测全本身不再是全场景拒绝条件。当前双箱记录仍没有完整可执行计划，真实抓放验收 **NOT_RUN**，2 s `PLAN_READY` 目标未达成；不能把软件回归或较快的拒绝计作达标。

合并后的定向测试、原生检查、原帧复算和本机启动状态以 [merged-report.json](evidence/realtime_20261119_stack_geometry/merged-report.json) 为准。该文件与合并前 [report.json](evidence/realtime_20261119_stack_geometry/report.json) 分开保留，避免用旧版成功的局部检查替代当前版本结论。

合并后本机新启动的三设备通信和数据均有效。新任务 `045e08d158a64bef9a45d89987c53df1`（RGB 1896 / depth 1893）检测到两箱，但两者都仅得到一个可信平面。上箱 SAM 内 48,309 个像素中 19,591 个有有效深度；人工在图像中选取的侧面诊断小区（389 < x < 418，203 < y < 328，与上箱掩码相交）有 3,330 个像素，仅 135 个有效深度。这是本帧诊断，不是新的算法阈值，也不表示 RGB 中看不到侧面。四个面内方向的额外离线诊断都未找到完整深度支持的 CAD 密封轮廓，排除了“仅交换接触轴即可通过”的猜测；生产接触方向未改。相机发射器启用、交替发射关闭，原曝光/增益/激光设置仅做只读查询，没有写入设置。新任务未发起规划或执行，工作台保留当前观测。当前还存在实际密封覆盖证据不足，不能仅把隐藏尺寸未知作为失败原因；详细诊断在 `merged-report.json` 的 `latest_live.depth_diagnostic`。

## 2026-11-19 深度空洞邻域修复与 USB 3.x 恢复

局部深度连续性检查原来将缺失深度当成 0 m，导致有效点旁的空洞被误判为深度跳变。现在仅用真实有效返回计算邻域最大/最小深度，空洞仍为空洞，实例外的真实深度跳变仍参与检查。保留原 12 mm 跳变阈值、2 mm 平面残差、2.5 mm 点支持半径和全部 CAD 密封轮廓/边缘带检查；新增实际 CAD 环下 9 mm 缺测区域回归，确认不会把该区域补成可密封面。

相机状态现在显示 SDK 实际读取的 USB 模式；采集异常保留设备序列号、请求的分辨率/帧率及原始原因。USB 模式用于诊断，不作为单独拒绝条件，也不会触发降分辨率或降帧率。首次两次改线后仍枚举为 USB 2.1，原参数连续等待 15 秒无帧，分流诊断偶发帧且重复深度帧号。后续现场重新连接成功枚举为 USB 3.2、总线 5000M；640×480@30fps 连续取得 120 帧，预热后 90 帧实测 **29.976 fps**，本次成功测试没有写相机选项。

此前临时激光功率诊断已恢复原值 150 mW，第一次比较与旧工作台关闭过程重叠，不能用于判断功率效果；独占重试也超时。一次相机复位前后高级设置内容一致。未更改外参文件或其历史状态。所有诊断均未调用机械臂运动或气路命令。

启动本机 CPU 入口仍为：

```bash
source /opt/ros/humble/setup.bash
.venv/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config outputs/workbench/single_carton.prior_free_cpu.20261119.json
```

三设备数据有效后，点击“准备观测 / 锁定起始点”；新冻结数据生成完整计划后，才进入操作者执行步骤。重新连接后的任务 `d6d985f17e6f4b6b84eaffb71c5da66b` 使用 RGB 528 / depth 494，时间戳差 0.0171 ms，锁定当时实测关节。上箱可见面生成 32 个接触候选；当前 SAM 上箱掩码还包含下箱，另有独立下箱检测，恢复器明确报告混合实例。离线去除同帧重复掩码的诊断得到约 170.609×109.793×111.211 mm，但完整碰撞包络仍为 407.913×407.585×122.022 mm；该诊断没有修改原始记录或产生执行许可。实例归属、边界不确定性与完整带载路径尚未解决，真实抓放 **NOT_RUN**，不能将相机恢复当作完整流程通过。

本步定向集合 834 通过、9 跳过；相机诊断和最终修改模块复测 87 通过。按当前 848 个测试节点合并为 **839 通过、9 跳过、0 未覆盖、0 失败**，这是基础全套与修改模块复测的合并结果。9 项跳过来自本环境缺少可选 OCP。`m710_clearance_test optimized/reference` 与 ECO65 原生组件检查通过，后者检查 491 个发送状态。完整计划、物理执行、现场持力与深度误差标定均未验收，2 s `PLAN_READY` 指标未达成。原始结果及源文件哈希见 [本步证据](evidence/realtime_20261119_depth_and_usb/report.json)。

## 2026-11-19 重叠实例、同帧边界约束及用户指定 5 mm 间隙

仅当原始掩码被既有检查判为混合邻箱时，才利用同帧独立的较小嵌套实例掩码重试平面拟合。原始掩码继续用于碰撞包络边界证据，原始深度、下箱检测和现场障碍均保留。排除于拟合的像素不被标记为空间已清空。新增回归覆盖叠箱、非嵌套检测、同物重复检测、原始区域保留及非法缩小包络证据的拒绝。

对于已有方向的矩形可见面，只要同帧深度在另一轴已观测跨度内证明某条射线的面位置为空，就可以约束该侧边界，不必要求整圈掩码外缘都恰好有深度。保留既有 6 mm 自由空间余量、2 像素保护带及原像素/3 mm 深度噪声外扩；前景遮挡、无深度和画外像素不提供证据。108 组独立光线投射真值包容回归继续通过。

本轮用户指令“取消碰撞包络40mm，将其设置为5mm”对应现有工具/机器人预留字段，已改为 0.005 m 并保存原依据。已重启本机 CPU 工作台应用该值，三设备数据有效。原生假几何确认 4.9 mm 拒绝、5.1 mm 通过；原 40 mm 条件另作历史回归保留。没有修改实体、固定装配对、箱体体积、密封、压力或运动限值，也未发送运动/阀门命令。

保存 USB3 帧复算得到上箱约 **170.609×109.793×111.211 mm**，包络为 **407.913×125.744×122.022 mm**；上下方向的无依据放大已由同帧自由射线约束缩小，左侧仍无充分证据。规划入口仍发现与 `rgbd_observed_5_15_12_s1` 的模型包络重叠 **21.290 mm**，未搜索路径，不表示实物相交。用户随后要求忽略上箱附近的点云伪障碍，这项排除尚未计入本节复算，另行记录其范围与依据。

完整定向集合 **851 通过、9 跳过**（383.84 s）；数值/间隙定向组 159 通过，属于前述集合的子集。两种原生间隙入口及 491 状态组件检查通过。物理 ECO65 使用显式凸体工作区；新增物理 5 mm 用例通过该规定检查器运行，历史 reference 检查仍保留。原始失败日志和修正说明见 [本步证据](evidence/realtime_20261119_nested_bounds_5mm/report.json)。完整现场路径、实机抓放、2 s 完整规划目标均未通过，物理验收 **NOT_RUN**。

## 2026-11-19 用户指定的本次上箱点云清单

用户在解释原有 6 cm 邻域后，选择“仅本次报错的上箱伪点云清单”。当前配置不采用 6 cm 忽略范围；`NEAR_TARGET_M=0.06` 仍只用于已有近目标细分和支撑查找。新排除清单复制任务 `d6d985f17e6f4b6b84eaffb71c5da66b`、观测 `80078bf9-a465-454b-bc26-7a905ebc3656` 在 5 mm 工具间隙复算中报告的 **53 个 observed 点云体**，保存原 ID、base_link 位姿、原尺寸及失败/场景/几何文件 SHA-256。清单不会随新帧、目标移动、体素重编号或后续失败自动增加、平移或外扩。

仅排除这些原始 OBB 内尚未归属的有效点。其他实例掩码、实测上箱底部及既有 4 mm 支撑保护带以下的点、既有支撑平面噪声窗口全部保留。较高实例未恢复、上箱身份不明确或上箱实测中心已离开参考实测箱体范围时，清单不应用；工作台显示原因。清单只影响场景中的点云，原始 RGB-D、分割、密封点支持、实测尺寸、完整不确定性包络、邻箱阴影、固定几何、工作范围与机器人/吸具实体均保留。

启用字段为 `USER_CONFIRMED_NAMED_LIST`，需非空现场依据及清单 SHA-256；缺省和显式 `KEEP` 均不排除点。清单文件进入配置加载、规划及执行前的源文件绑定；文件被改动后，旧计划不能继续使用。每次 `site-scene.json` 记录来源观测、清单哈希、逐项命中/新增排除数、最终排除掩码哈希和世界范围。记录的依据为 `USER_FIELD_CONFIRMATION_NOT_SENSOR_FREE_SPACE`，不改写为传感器证实空旷。

同帧复算排除 **229 个点**，observed 障碍体从 **182 降至 167**；91 个邻箱阴影、1 个观测支撑和 6 个工作范围边界逐项不变，几何及原始观测/掩码不变。最大原位包络冲突从 21.290 mm 降至 **9.699 mm**，仍有 38 个观测体与上箱保守包络相交，未搜索候选或路径。首项 `rgbd_observed_5_16_9_s26` 的原始范围包含上下箱交界像素：190 个有效返回中 106 个位于下箱掩码，131 个在支撑保护高度以下（统计可重叠）。不能把整个清单对象直接删掉而抹去这些证据，也不把保守模型相交称为实箱穿透。

另用用户随后保存的任务 `bb81d99e52bc4c8d93b41ba6d9c6c3a6` 独立复算，固定清单在该帧没有可排除点。上箱实测约 169.453×110.782×109.315 mm，但深度方向边界证据缺失，完整包络约 407.924×126.177×408.060 mm，原位检查首先与工作范围边界相交 202.699 mm。没有为新失败扩充清单、减小完整载荷或改动工作范围。两次保存帧复算均为离线，不产生运动许可；未使用 GPU。

现场使用方式：正常结束无持箱的旧会话，运行上述本机 CPU 启动命令；三设备数据有效后点击“准备观测 / 锁定起始点”，在任务详情核对清单应用状态和排除数，再点击“规划所选箱”。清单不允许复用旧观察或旧计划。完整四段计划与最终审查仍是执行前提，真实运动只由操作者点击“执行单箱”启动。改变现场摆放后需要重新审视这份固定清单，不能将其作为通用过滤器。

定向回归、原生检查及当前启动状态详见 [本次证据](evidence/realtime_20261119_named_artifacts/report.json)。完整现场路径与真实抓取—带载返回—释放仍未通过，物理验收 **NOT_RUN**；25.02 s 的离线观测重建和 0.44 s 的原位拒绝不能算作 2 s 完整规划目标达成。

本步完整定向集合 **883 通过、9 跳过**（408.23 s），随后补齐支撑噪声带用例的定向组 **33 通过**；按节点去重合计 **884 通过、9 跳过、0 失败**。9 项跳过仍为可选 OCP 缺失。optimized/reference 与 491 状态 ECO65 原生组件均通过；现场配置静态检查无问题。源代码提交后已正常结束无持箱旧会话并启动本机 CPU 入口，D435i（USB 3.2）、机械臂及真空均显示数据有效，工作台待机、无有效计划。本次重启未点击准备或执行，没有运动/阀门写入。

## 2026-11-19 上箱正面吸取的独立检查（未执行）

针对用户“先忽略箱体几何是否准确、直接吸上箱正面”的请求，对最新冻结任务 `6e13d059fc1443f2ae1c8c56962d21fe` 做只读诊断，不改变生产配置或运动许可。已观测正面约 169.473×110.585 mm，法向在 base_link 下为 `[0.983373, -0.181491, -0.006181]`。原候选器得到 32 个通过实际 CAD 密封轮廓及所有允许接触唇部检查的候选，最多 6 个杯预期密封；这是几何证据，不是实际吸附试验。正面候选不依赖整箱唯一精确尺寸。

独立检查未上抬、上抬 40 mm、50 mm 三个六杯候选。每个接近位得到 4 个逆解，按现有顺序检查前三支均通过静态碰撞；每个接触位得到 5 个逆解，前三支均未通过。较优分支的刚性杯芯 `part_002_rigid` 与目标碰撞包络模型间隙约 **0.669 mm**，低于保留的 2.4 mm 对间隙；另一分支出现 `link_3` 与已建模支架相交。所有查询保留原场景、715 个实体和完整箱体包络，没有执行轨迹搜索或动作。

原位完整包络仍与 `site_boundary_0_0` 相交约 202.491 mm。持力工程估算在完整包络重心条件下，32 个候选均未通过；上述六杯候选最小余量约 0.543。仅作为离线敏感性对照，把重心换为未经本轮完整体积验证的名义箱体中心，余量约为 1.925–1.927；该对照不进入碰撞世界、附着模型或任何执行许可，不能据此声称实际吸力不足或已证明能够安全带载。

本次明确区分“正面密封候选成立”“接近终态静态可行”“接触与完整带载路径已验证”。后两者没有得到完整通过，未以忽略几何误差的方式放行真实抓放。源记录哈希不变，无机器人运动、阀门命令或许可发放；物理验收 **NOT_RUN**。未改代码或配置，未重复运行无关回归；18 次原生静态查询及 FK 误差检查见 [本次诊断](evidence/realtime_20261119_front_face_check/report.json)。

## 2026-11-19 当前阻塞的同源三维可视化

针对“现在到底卡在哪”，新增只读导出工具 `tools/visualize_current_carton_blockers.py`。使用上节任务 `6e13d059fc1443f2ae1c8c56962d21fe` 的冻结 RGB-D、同帧掩码、实测 q_start/q_capture、几何、场景和 18 条原生静态查询，不重新推理、规划或连接设备。导出前后核对源文件哈希、原生模型身份、目标、查询姿态和原始碰撞世界。机器人、吸具及 link_1 支架使用当前真实硬件模型，保留 715 个碰撞实体和 12 个吸盘；未加载仿真环境。显示开关只隐藏显示，不能修改检查记录或发放许可。

第一阻塞仍发生在完整路径搜索之前。可见面/侧面拟合箱体约 **169.473×110.585×111.685 mm**，完整保守包络为 **407.903×123.771×408.059 mm**。面内 −u 与背面缺少边界证据，恢复器使用 `MISSING_BOUNDARY_MAXIMUM_EXTENT`；0.40 m 上界不是实测箱型。包络最小 X 为 −1.002491 m，越过配置 X≥−0.800 m 边界 **202.491 mm**。红色 `site_boundary_0_0` 是工作范围配置，不是实际墙面；该模型越界不能解释为两只实箱相交。其余现场对象仍可开启显示，首项拒绝也不代表其他 42 项包络相交已经消除。

独立接触诊断现在能显示经过验证的源 CAD 顶点与包络面点对。候选 #0 的第一支接触逆解中，刚性杯芯顶点距拟合观测平面 **6.696893 mm**，包络前界向外扩 **6.027591 mm**，两者差 **0.669302 mm**，与原生记录吻合。点对只有在投影落入真实包络面、独立距离与原生距离误差≤1 µm 时才显示；否则仅高亮碰撞对，不由距离标量编造最近点。三候选的第一/第三支共六个点对满足条件。这是模型位置关系，不是实物间距测量，也不证明实物已碰撞。

页面的四个视图分别展示包络越界、接触间隙、另一支逆解的连杆/支架相交及持力估算对重心假设的敏感性。离散姿态选择限于冻结起点和已保存的接近/接触状态，不插值出未经验证的路径。原生相交返回 −1 明确显示为状态码，不解释为穿入深度。持力余量 0.543 与约 1.926 的对照沿用上节估算，后者仍是未认证的名义重心假设，不进入生产规划。吸具/机器人 5 mm 与刚性杯芯/箱体 2.4 mm 两种间隙分别标注。

本机生成与打开方式（输出目录须未存在）：

```bash
.venv-workbench/bin/python tools/visualize_current_carton_blockers.py \
  --record outputs/single_carton/6e13d059fc1443f2ae1c8c56962d21fe \
  --diagnostic outputs/front-face-check-20261119-6e13d059 \
  --config outputs/workbench/single_carton.prior_free_cpu.20261119.json \
  --output outputs/current-carton-blockers-export
.venv-workbench/bin/python tools/open_eco65_diagnostic_viewer.py \
  outputs/current-carton-blockers-export/index.html
```

已导出的本机文件位于 `outputs/field-current-blockers-20261119-6e13d059-v6/`：`index.html` 为无需服务器的交互页面，`overview.png` 为四图概览。左上四个按钮切换问题；“接触间隙”内可勾选“只显示冲突对”放大点对；“全景”恢复机器人全景。页面只读，使用保存帧，不能替代最新现场观测。

浏览器定向检查覆盖独立 FK 一致性、18 个保存姿态、源数据不变、715 实体/12 杯保留、显示开关、同帧图像、状态切换及稳定布局；数值回归覆盖点对证明与拒绝不符证据。检查结果见 [本次可视化证据](evidence/realtime_20261119_blocker_viewer/report.json)。完整路径、发送前审查、真实吸附与抓放均 **NOT_RUN**；本次未修改任何生产门禁或阈值，未调用运动或阀门接口。

本步完整定向集合 **890 通过、9 跳过、0 失败**（498.71 s）；其中包含 6 项点对验证回归。9 项跳过仍为可选 OCP 缺失。35 项浏览器检查全部通过，浏览器与独立 Python FK 的最大矩阵误差为 3.33×10⁻¹⁶；四个视图及概览 PNG 均经过渲染检查。optimized/reference 与 491 状态 ECO65 原生组件检查通过。首次定向运行在最终汇总前以 143 退出，未计作通过；随后完整重跑取得上述结果。物理验收仍为 **NOT_RUN**。

## 2026-11-19 箱体几何恢复鲁棒性与共面接缝策略

用户 2026-11-19 20:41 决定：共面接缝可在显式策略下采信“掩码边界与强 RGB 边缘重合”，误差界 ±5 mm；相机安装位置不变；其余修改按实施方意见。随后用户明确由本会话（而非其他会话）完成。全程离线：未连接设备、未重新推理、未使用 GPU，没有运动或阀门命令，物理验收 **NOT_RUN**。

### 修改内容

- **P0-A 倾斜支撑面**：实测支撑面允许 ≤5° 倾斜（与 `observed_support_plane` 一致）。保守裁剪使新底面的最高点位于支撑面高度再向下一个支撑高度稳健 σ（MAD）处，不移除支撑面以上的体积；倾斜落差加该余量超出既有支撑噪声窗口 max(4 mm, 4σ) 时不裁剪，记录原因。`site_scene._support_below` 与 `planner.detach_supports` 继续以包络最低点绑定 0.4 mm 规则。
- **P0-B 角点不能代证边界**：直接深度边界的证据像素必须在另外两轴的实测范围内（各减 pad，与自由射线条带规则一致），并且至少 10 像素、沿边覆盖 ≥15 mm。
- **P1-C 第二平面**：取消 4% 相对比例，改为全分辨率支撑 ≥300 像素、RMS ≤ 残差，且与正面在交线 4 mm 内有 ≥20 mm 连续共同支撑；不合格的候选记录后继续搜索。顶面与侧面都可作厚度来源。
- **P1-D 厚度**：只用与“正面—侧面交线”图像连通的侧面片，按 5 mm 行取远端，再取 95% 分位；排除点数写入证据。
- **P1-E 厚度方向自由射线**：对每个已接受的可见平面（正面、侧面、顶面）应用同一自由射线与条带规则，背侧边可由侧面远端约束；前景、无返回和画外像素不提供证据。
- **P1-F 共面延续**：Canny 边缘改为在掩码边界两侧 ±3 px 带内统计，仍要求 70% 覆盖。
- **P1-G 嵌套实例**：较小检测掩码 ≥95% 落在本掩码内时一律先用差集拟合，原掩码只作包络证据；检测层 NMS 丢弃“含两个及以上子框、并集覆盖 ≥60%”的整垛框并记录，只含一个子框时保留。远程 GPU 检测每次连接都会上传当前 `hf_detector_core.py`，工作台重启后生效。
- **P1-H 叠箱支撑**：接受 `MEASURED_BOUNDARY_INTERVALS` 包络；实例计数在去重、整垛掩码识别与 `OUTSIDE_WORKSPACE` 忽略之后进行；支撑面取上箱实测底面、下箱可见面 99.5% 顶部（排除上箱掩码）和接缝线减误差界中的最低者；估计相差超过 15 mm 不应用。上箱包络底面在点归属之前就地设到该面，规划器的 0.4 mm 绑定随之成立。
- **共面接缝策略**：`geometry.coplanar_seam_policy={mode,uncertainty_m,basis}`。详见下节。
- **缺边诊断**：每条仍为缺边的边记录外侧分类（共面延续 / 无效深度带 / 前景遮挡 / 画面边缘 / 贴检测框边 / 无 RGB 边缘 / 其他），任务详情显示六条边的方法与分类。

未改变：2.4 mm 对间隙、5 mm 工具/机器人间隙、1 mm 原位重叠阈值、0.4 mm 支撑穿入、0.40 m 最大边长、6 mm 自由余量、2 px 保护带、3 mm 深度噪声、2 mm 平面残差、715 个碰撞实体、12 杯与 CAD 密封轮廓、压力门、运动限值、5 ms 周期、停止与执行锁语义、相机外参、`unknown_policy` 及点云具名清单。

### 共面接缝策略

`mode` 为 `DISABLED`（缺省，共享示例保持此值）或 `MASK_RGB_EDGE_COINCIDENT`；后者要求 `uncertainty_m` 在 0.005–0.02 m 之间、`basis` 非空，`--check-config` 校验。只用于实测正面的四条面内边，且仅在直接证据与自由射线都缺失之后；厚度方向不用；底边以叠箱支撑为准。同时满足以下条件才生效：

1. 边界外 1–6 px 的有效深度多数位于主平面上（max(3 mm, 4×RMS)），任何样本在平面前方 >8 mm 即判为前景遮挡。实施时另加两条更保守的条件：样本须在 RGB 边缘直线外侧 ≥2 px；共面延续在 7–14 px 带内必须持续存在。
2. 掩码边界 ±3 px 内能拟合强直线边缘：中位梯度 ≥40 且 ≥ 面内纹理 90% 分位的 2 倍，覆盖 ≥70%，与投影箱边夹角 ≤3°。
3. 掩码边界与直线重合：中位距离 ≤2 px，≥80% 在 3 px 内。
4. 边界段不在画面边缘 2 px 内；落在检测框边 2 px 内的比例 <20%。

生效后外界取直线与掩码边界中更靠外的位置再加 `uncertainty_m`，记录 `COPLANAR_SEAM_MASK_RGB_EDGE_POLICY`、各项判据数值、依据及 `USER_POLICY_RGB_SEAM_NOT_DEPTH_BOUNDARY`；实测尺寸不变。本轮不新增碰撞豁免：原位重叠若全部落在接缝带内，单独报告 `COPLANAR_SEAM_CONTACT_NOT_PERMITTED`，仍阻断规划。

本机现场配置 `outputs/workbench/single_carton.prior_free_cpu.20261119.json`（被 git 忽略）已启用策略，`uncertainty_m=0.005`。当前运行中的工作台进程在修改前启动，仍使用旧代码与旧配置，重启后才生效；本次未重启。

### 合成场景（`tests/field_like_scene.py`，现场外参、J1=1.41609 rad）

改前（`89514c2`）15 个场景只有 3 个恢复；改后 15/15 恢复，接缝关闭与开启下真值都完整包含。厚度误差除掩码内缩 2 px 的 `stack_mask_-2px`（−9.7 mm，包络仍包含真值）外均在 ±1 mm 内。

| 场景 | 改前 | 改后（接缝关闭）包络 mm | 改后（接缝开启）包络 mm |
|---|---|---|---|
| ideal_stack | OK 178×114×123 | 177.5×117.4×122.7 | 同左 |
| noisy_visible_support / flush_stack | MEASURED_SUPPORT_REQUIRES_CLIPPED_ENVELOPE | 212.4×118.1×124.9 / 212.4×118.3×124.9 | 同左 |
| flush_stack_mask_+1px | COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED | 216.2×118.4×134.2 | 同左 |
| stack_mask_+1px / +3px / −2px | MEASURED_SUPPORT_REQUIRES_CLIPPED_ENVELOPE | 216.2×118.1×134.2–134.3 | 同左 |
| stack_samlike_0.35 | 同上 | 212.5×118.1×124.9 | 同左 |
| frontal_yaw_0 / yaw_−12 / yaw_+20 | BOX_THICKNESS_UNOBSERVED_SHOW_SIDE | 厚度误差 +0.25 / +0.20 / +0.05 mm | 同左 |
| coplanar_touching_left | OK 408×119×125 | 408.4×122.4×124.9 | **180.7**×122.4×124.9 |
| left_gap_30mm | OK 180×119×125 | 186.9×122.4×124.9 | 不变 |
| parent_mask_with_child | BOX_ENVELOPE_CLIPPED_BY_IMAGE | 213.9×125.5×127.6（差集拟合） | 同左 |
| packed_wall | BOX_THICKNESS_UNOBSERVED_SHOW_SIDE | 407.8×637.3×124.0 | 119.6×181.4×124.0 |

945 组光线投射真值扫描（3 尺寸 × 3 姿态 × 噪声 0/0.5/1 mm × 35 种掩码缺陷：单侧缺 0–60 px、SAM 式阈值 0.35/0.5/0.65、平移 ±2 px）接受 791 组，**0 组越界**；去掉 10 像素/15 mm 覆盖门槛只保留条带规则时 1 组越界 0.2 mm，这是门槛的标定依据。原 108 组真值包含扫描继续通过。

### 现场保存帧复算（离线，源文件哈希不变）

`tools/compare_geometry_replay.py` 对 7 条记录重建几何、构建冻结场景并运行原位检查（与 `search_box` 第一步相同），不启动原生规划、不搜索候选。`a9a1eaac` 使用 `outputs/field-replan-20261118-new-start/replay` 的离线 CPU 掩码。下表列出恢复的上箱（mm）；接缝开启与关闭结果完全相同，点云清单改为 KEEP 只改变 `d6d985f1` 的首个阻挡。

| 任务/检测 | 改前实测 / 包络 | 改后实测 / 包络 | 改后六边方法 | 原位检查（改后） |
|---|---|---|---|---|
| 31e46172/1 | 168.2×110.6×104.0 / 223.3×184.7×**407.7** | 168.2×110.6×104.5 / 223.8×188.3×**123.6** | 叠箱支撑、厚度自由射线 | observed 体 12.5 mm ×46 |
| 67dbcaac/0 | 168.4×110.9×107.7 / 228.4×123.3×118.0 | 168.4×110.9×107.5 / 224.6×128.5×122.5 | 叠箱支撑 APPLIED | observed 体 12.7 mm ×39 |
| a9a1eaac/0 | 169.5×110.8×109.8 / 225.5×125.9×**407.7** | 169.5×110.8×112.5 / 223.4×128.5×**127.8** | 叠箱支撑、厚度自由射线 | observed 体 13.2 mm ×42 |
| b8992c09/0 | 168.0×110.4×109.1 / 222.2×123.7×119.8 | 168.0×110.4×109.3 / 222.7×129.9×125.5 | 叠箱支撑 APPLIED | observed 体 12.6 mm ×45 |
| bb81d99e/2 | 169.5×110.8×109.3 / **407.9**×126.2×**408.1** | 169.5×110.8×111.0 / **408.9**×129.2×**154.9** | 叠箱支撑；w− 缺边（贴检测框边） | observed 体 13.5 mm ×61 |
| d6d985f1/1 | 170.6×109.8×111.2 / **407.9**×125.7×122.0 | 170.6×109.8×111.6 / **409.5**×127.8×129.1 | 叠箱支撑；w− 缺边（疑似深度外扩） | observed 体 11.8 mm ×46；KEEP 时 21.3 mm ×61 |

改前 `31e46172`、`a9a1eaac`、`bb81d99e` 的厚度方向缺边（约 408 mm），首先因越出工作范围 156–203 mm 被拒；改后厚度方向都由侧面或自由射线约束（124–155 mm），不再越界。所有目标仍在原位检查被阻挡，均未进入候选搜索。`045e08d1` 上箱侧面有效深度只有约 135 点，仍为 `BOX_THICKNESS_UNOBSERVED_SHOW_SIDE`；各记录的下箱和画面底部大纸箱只见一个面，同样保持该原因，相机不改位置时这是明确的失败原因。

叠箱支撑在除 `045e08d1`（上箱未恢复）外的 6 条记录全部 `APPLIED`。下箱按“与上箱底面有同帧接触面的唯一实例”选取，画面底部的大纸箱等列外实例不再阻止应用，也不加入叠放关系；有两个候选时拒绝为歧义。6 帧中下箱可见顶面都比上箱实测底面低 9.6–11.3 mm，按最低值规则底面下移约 10 mm。

### 原位阻挡的来源与待用户决定事项

1. **接缝带内的下箱掩码点**：叠箱帧的首批阻挡位于包络底部前沿、高度在下箱可见顶面与上箱实测底面之间（例如 `67dbcaac` 为 310–318 mm，部分在上箱实测箱体内）。它们属于下箱掩码，按既有规则其他实例掩码内的点不能归给目标，于是成为障碍。是否允许在已确认叠放关系下对这条接缝带做点归属，需要用户决定；本轮未改。
2. **D435 左侧阴影带**：上箱左边沿外是立体遮挡造成的无效深度带，最近的有效自由射线在 43–47 mm 外，使 w− 多出约 45 mm。无深度不能当作空闲；若要缩小需要新的显式策略。
3. **轮廓边深度外扩**：`d6d985f1` 左边沿外 1–6 px 在 RGB 中是白墙、深度却等于箱面（立体匹配外扩），满足用户给定的 1–6 px 共面条件，但并非共面邻箱。实施中加严判据使其不生效；如需对“强 RGB 轮廓边 + 背景”另设策略，需要用户决定。
4. **贴检测框边**：`bb81d99e` 的左边界段 31% 落在检测框边 2 px 内，按规格不适用。SAM 掩码通常贴着 DINO 框，这条规则可能使策略在现场很少生效。
5. **接缝接触**：本轮复算没有出现“重叠全部位于接缝带内”的情况，未触发 `COPLANAR_SEAM_CONTACT_NOT_PERMITTED`；若出现，是否增加仅载荷、仅接触/脱离段、仅接缝带内的接触规则仍需用户决定。

### 验证与未验证

分阶段提交，每阶段在独立 worktree 中用该阶段源码运行完整定向集合：P0 `d09e2a5` **890 通过、9 跳过**；P1 `71dc245` **922 通过、9 跳过**；接缝策略 `56c8466` **935 通过、9 跳过**（9 项跳过均为缺少可选 OCP）。每阶段 `m710_clearance_test optimized`（17 项）/`reference`（14 项）与 ECO65 原生组件检查（491 个发送点）均 PASS；本次没有 C++ 改动。详见 [report.json](evidence/20261119_geometry_robustness/report.json)。新增 `tests/test_single_carton_geometry_robustness.py`（field-like 15 场景、掩码缺陷扫描子集、各项负例和接缝策略）。未验证：现场深度误差与掩码误差的系统标定、P1-C 阈值的系统标定（只用 field-like 三视角和背景平面负例检验）、完整路径与最终审查、实际抓取。运行中的工作台未重启，现场尚未使用新代码。物理验收 **NOT_RUN**。

## 2026-11-20 接缝带点归属、立体阴影带、箱体自身点与正面前界

用户 2026-11-20 对 11-19 待决事项的决定：
1. 叠箱接缝带内的点以上箱可见面为准绳。
2. D435 左侧阴影带是箱后的电脑屏幕，可用深度差区分。
3. 同意轮廓边深度外扩的加严判据（共面延续必须在 7–14 px 外仍成立）。
4. 贴检测框边规则按实施者理解，规格该改就改。
5. 右后方剩余阻挡点都是箱子自身的点。
6. 现场记录 `6e13d059` 的接触模型间隙不足：杯芯到拟合观测面 6.697 mm，包络向外扩 6.028 mm，剩余 0.669 mm < 2.4 mm。"包络外扩"与"2.4 mm"中不合理的都要改。

全部为离线工作：无设备、无运动、无阀门命令，物理验收 **NOT_RUN**。

### 规则变更

- **叠箱支撑面（决定 1）**
  - 支撑面改为上箱可见面底边的直接深度边界，即最低观测面采样点下移一个像素足迹再加 3 mm 深度噪声余量；若该边由接缝策略约束，取两者较低者。
  - 下箱可见顶面受上箱前沿遮挡（现场比上箱实测底面低 9.6–11.3 mm），不再拉低支撑面，只做一致性检查：高于上箱底面即冲突；三者差超过 15 mm 即拒绝。
  - 接缝带归属：上箱包络内、介于支撑面与上箱实测底面 +4 mm 之间、位于上箱掩码 15 px 带内、标签为上箱/下箱/无标签的回波，归上箱；支撑面以下的归下箱。其他实例从不认领。
  - 上箱正面若需包含这些接缝带点，最多外扩到拟合面 +4 mm（沿用 `TARGET_PLANE_BAND_M`）。
  - 下箱支撑体前界扩到包含其接口带内的自身回波，再加 3 mm 测量外扩余量（`a9a1eaac` 下箱正面不是单一平面）。支撑体顶面仍等于支撑面，原生绑定不变。
- **`geometry.stereo_shadow_boundary_policy`（决定 2）**
  - 模式为 `DISABLED`（默认，共享示例）或 `INVALID_BAND_BEFORE_FARTHER_RETURN_IS_BACKGROUND`（须给出依据）。
  - 只用于正面在图像左侧的面内边，且仅在直接边界不成立时：沿图像行依次出现箱面、至多 6 px 共面外扩、无返回带，然后是比箱面远 12 mm 以上的返回。
  - 条带、覆盖门槛和 pad 与直接边界相同；只在比自由射线更紧时采用。记录为 `USER_POLICY_INVALID_BAND_IS_BACKGROUND_BEYOND_DEPTH_STEP`；无返回带本身不变成空闲空间。
  - 现场配置启用此策略，依据为用户 11-20 决定；配置 SHA-256 由 `622de8…` 变为 `ef384d…`，备份为 `…before-stereo-shadow.json`。
- **贴检测框边（决定 4）**：接缝段贴框时，只有在外侧 1–6 px 带内被其他检测实例覆盖不足 50% 时才排除。被截断的掩码在截断处另一侧没有实例；有相邻实例时，"贴框"就有了解释。
- **箱体自身回波（决定 5）**
  - 自身回波的定义：目标掩码内的回波，以及掩码 15 px 带内的无标签回波，前提是与已接受面采样 4 连通、且不跨 12 mm 深度跳变（与场景飞点判据相同）。
  - 包络只外扩、不收缩，扩到包含这些回波再加 pad。
  - 正面方向最多到拟合面 +4 mm；超出的只记录（如 `67dbcaac` 角上 95 px、在正面前方 4–12 mm 的噪声）。
  - 包络内的自身掩码回波认领为目标。包络内不再要求深度连通；深度连通只用于决定包络范围，防止背景泄漏把包络撑大。
- **正面前界（决定 6）**
  - 正面是拟合平面，不是像素之间的边界。原规则"最远采样 + 一个像素足迹 + 3 mm"把噪声重复计算，前界达到拟合面前方约 5.8–6.0 mm。
  - 现改为拟合面 + 3 mm 深度噪声余量，且不小于任何已接受采样；被接受采样都在平面 ±2 mm 内。
  - **2.4 mm 对间隙保持不变**：它保护刚性杯芯不碰真实箱面，吸附时唇边还会压缩。不合理的只是叠加在多算了约 3 mm 的包络上。
  - 按同一杯芯距离推算，`6e13d059` 剩余间隙由 0.669 mm 变为 2.70–3.70 mm（前界 3.0 mm，封顶 4.0 mm）。未经原生接触检查确认：该目标在新代码下先因左边沿无法测量（贴近画面边缘）被原位检查阻挡。
- **轮廓边外扩（决定 3）**：维持 11-19 的 7–14 px 判据，记录为用户确认。

### 现场保存帧复算（8 条，离线，源文件哈希不变）

尺寸单位 mm。"改前"为 `bf5676e`（接缝策略开启）。

| 任务/检测 | 改前包络 / 原位检查 | 改后包络 | 改后六边要点 | 改后原位检查 |
|---|---|---|---|---|
| 31e46172/1 | 223.8×188.3×123.6 / 12.5 mm ×46 | 189.2×182.3×121.8 | w− 阴影带 5.2；正面 +3.9 | **通过，进入候选搜索** |
| 67dbcaac/0 | 224.6×128.5×122.5 / 12.7 mm ×39 | 189.5×124.0×120.7 | w− 阴影带 5.7 | **通过，进入候选搜索** |
| a9a1eaac/0 | 223.4×128.5×127.8 / 13.2 mm ×42 | 193.0×123.9×126.7 | w− 阴影带 8.5 | **通过，进入候选搜索** |
| b8992c09/0 | 222.7×129.9×125.5 / 12.6 mm ×45 | 188.9×123.6×123.7 | w− 阴影带 6.3 | **通过，进入候选搜索** |
| bb81d99e/2 | 408.9×129.2×154.9 / 13.5 mm ×61 | 413.9×124.0×152.1 | w− 缺边（贴检测框边，无相邻实例） | 2.7 mm ×1 |
| d6d985f1/1 | 409.5×127.8×129.1 / 11.8 mm ×46 | 414.3×124.4×126.8 | w− 缺边（深度外扩） | 3.3 mm ×4 |
| 6e13d059/2 | 未复算 | 414.5×123.7×147.7 | w− 缺边（贴检测框边，无相邻实例） | 3.0 mm ×3 |
| 045e08d1 | 两箱均 `BOX_THICKNESS_UNOBSERVED_SHOW_SIDE` | 不变 | — | — |

宽度方向由约 223 mm 收到 189–193 mm：左侧阴影带不再外扩约 45 mm；右侧外扩约 10 mm，以包含右后方外鼓的自身回波。

剩余 3 个目标的左边沿位于画面左缘：
- `bb81d99e`、`6e13d059` 的无返回带一直延伸到图像边界，画面内看不到更远的返回，深度差无从判断。
- `d6d985f1` 左侧白墙前的箱面深度连续外扩超过 6 px。

相机位置不变，这三个都是明确的失败类别。

### 验证与未验证

验证结果：
- 定向集合 **949 通过、9 跳过、0 失败**（9 项跳过为缺少可选 OCP）。
- `m710_clearance_test optimized/reference` 均 PASS，输出与 11-19 相同（除计时）。
- ECO65 原生组件检查（491 个发送点）PASS。本轮无 C++ 改动。
- 945 组真值包含扫描接受 791 组，**0 组越界**。
- 新增测试覆盖：
  - 立体阴影带策略的配置、生效、真值包含，以及无更远返回/共面延续/非左侧时不适用；
  - 贴框规则的两种情形；
  - 侧面外鼓与正面带内/带外鼓出；
  - 深度跳变切断自身连通；
  - 接缝带归属；
  - 正面前界规则。

证据见 [report.json](evidence/20261120_seam_shadow_own_surface/report.json)。

未验证：
- 4 个通过原位检查目标的完整路径搜索与最终审查；
- 新正面前界下的原生接触间隙；
- 现场深度/掩码误差标定。

运行中的工作台未重启，现场仍在用旧代码。物理验收 **NOT_RUN**。

### 补充：颜色边界、叠箱接口带代表与新现场帧（同日）

用户补充决定：RGB 颜色可轻易区分箱体与周围环境。

- **`geometry.rgb_color_boundary_policy`**
  - 模式为 `DISABLED`（默认，共享示例）或 `CARTON_COLOUR_DIFFERS_FROM_SURROUNDINGS`，不确定量 0.005–0.02 m，须给出依据。
  - 适用范围：深度、自由射线、阴影带、接缝都无法约束、仍为缺边的正面面内边。
  - 生效条件：
    - 掩码内侧是箱面色度（Lab 的 a、b 通道，不看亮度）；
    - 外侧 3–14 px 内像箱面色度的像素不超过 10%；
    - 外侧带内没有前景；
    - 外侧 7–14 px 没有共面深度延续；
    - 该边不贴图像边界。
  - 外界 = 掩码边界 + 3 px 足迹 + 不确定量。
  - 现场标定数据：箱面色度离散 p90 为 3.6–7.1，外侧最近 p10 ≥ 19.4，外侧像箱面比例 0–2%，内侧 83–94%。
  - 现场配置启用，不确定量 5 mm。
- **叠箱接口带代表**：支撑体内、支撑面以下 15 mm 以内、上箱掩码 15 px 带内的上箱实例回波和无标签回波，由下箱支撑体代表。更深处的无标签物体和其他实例保留各自的碰撞体，已有测试保留。
- **检测后端**：现场配置的检测后端改为 GPU 服务器（`remote_ssh`）。新主机只写在被忽略的本地文件中，登录使用既有密钥。

#### 新现场帧 `7fc259a5`（新代码重启后拍摄）

- 在现场阈值（框 0.35 / 文本 0.30）下，DINO 只给出整摞一个框（0.465），恢复器判为 `MASK_MIXED_NEIGHBOR_OR_NONCUBOID`。
- 离线把阈值降到 0.25/0.25 做诊断（现场配置未改）：多出下箱（0.290）和画面底部大纸箱（0.346）两个框。差集拟合恢复出上箱：实测 170.6×110.3×110.7 mm，包络 202.8×124.8×129.6 mm，叠箱支撑 APPLIED，原位检查通过。
- 同一诊断掩码下的离线完整规划没有得到完整计划，32 个失败候选：
  - 23 个持力能力不足（3 个吸盘密封，剥离裕度 0）；
  - 5 个 approach 用完求解预算；
  - 4 个接触终态唇边与叠箱支撑体间隙 0.45–0.56 mm。

#### 复算结果（9 条记录）

| 记录 | 改后包络（mm） | 原位检查 |
|---|---|---|
| 31e46172 | 189.2×182.3×121.8 | 通过 |
| 67dbcaac | 189.5×124.0×120.7 | 通过 |
| a9a1eaac | 193.0×123.9×126.7 | 通过 |
| b8992c09 | 188.9×123.6×123.7 | 通过 |
| bb81d99e | 192.5×124.0×152.1（w− 颜色边界） | 通过 |
| d6d985f1 | 202.5×124.4×126.8（w− 颜色边界） | 通过 |
| 6e13d059 | 193.2×123.7×147.7（w− 颜色边界） | 通过 |
| 7fc259a5（0.25 诊断掩码） | 202.8×124.8×129.6 | 通过 |
| 045e08d1 | — | 两箱均未恢复（侧面不可见） |

验证：
- 定向集合 **955 通过、9 跳过、0 失败**；
- 真值扫描 945 组 **0 越界**；
- `m710_clearance_test optimized/reference` 退出码 0。

完整路径和物理验收 **NOT_RUN**。

### 补充：新帧规划阻挡的排查与修复（同日）

对象为新帧 `7fc259a5` 的上箱，使用 0.25/0.25 诊断掩码离线完整规划。无设备、无许可。

**唇边贴下箱支撑体（4 个候选，间隙 0.45–0.56 mm）**
- 原因：前一版把叠箱支撑体的前界整体外扩到包含下箱接口回波（前移 27 mm）。由于支撑体是一个统一半空间，它在上箱正面前方、支撑面以下伸出约 186 mm。
- 处理：撤销该外扩。改为把上箱包络正下方、支撑面以下 15 mm 内、未被支撑体包含的本摞回波，按 10 mm 分组做成与支撑面对齐的支撑块：顶面严格在支撑面上，类别与主支撑体相同。机器人和工具仍与它们碰撞检查，载荷只在 detach 阶段按 0.4 mm 支撑规则放行。
- 另外，支撑面以下的近目标观测盒，外扩余量不越过支撑面，但始终包住点本身。

**唇边贴下箱粗格子（2 个候选）**
- 原因：下方一排吸盘悬在下箱正面前约 15 mm。但下箱区域的世界坐标轴对齐粗格子在旋转后的上箱坐标里伸出约 25 mm。
- 处理：近目标 60 mm 内的格子列全部细化（`NEAR_TARGET_REFINE_GAP_M` 由 6 mm 改为 60 mm）。细化盒子仍包含全部点并保留 3 mm 外扩余量。本帧观测体 54 → 61 个。

**持力不足（23 个候选）**
- 原因：原剥离模型把吸盘当作点，3 个吸盘排成一行时没有抗力矩的力臂，裕度恒为 0。
- 处理：
  - 改为绕全部密封吸盘 CAD 圈凸包边缘的倾覆模型：吸力作用于吸盘中心，载荷作用于质心，使用实际发送状态的 FK 加速度。
  - 安全系数 2、摩擦系数 0.5 不变。
  - 新增可选 `holding_capacity.cup_suction_force_n`（须给出依据，上限为 CAD 面积 × 抓取阈值 60 kPa）。现场配置按用户给出的单杯 42 N 启用；共享示例仍为面积估计（单杯 31.9 N）。
  - 向量化后每次评估约 1.8 ms。
- 结果：本帧持力不足由 23 个降为 10 个，剩余的都是只有 2 个吸盘的候选。

**approach 求解超时（现 17 个，需用户决定）**
- 全部是吸具零件 `part_026`（PL1202，位于法兰附近）与 `link_3`：RRTConnect 能找到几何路径，但稠密校验发现实际接触（1079 次）。
- 本目标的逆解都要求 J1 转约 66°、J5 由 +1.50 翻到 −1.12 rad。唯一保持 J5 同号的逆解（J4≈0）是第 4 个分支，按"最多检查 3 个分支"的规定未被检查。
- 参考仿真分支对这一对有用户授权的仿真豁免（2026-10-22，`explicit_link_3_part_026_simulation_exemption_v1`）。该分支的 `outputs/receiver_all_20261102/run01_replay.html` 在 GitHub 各分支和本地均不存在。
- 可选做法均触及现有不变量，本轮未改：
  - 检查第 4 个逆解分支；
  - 加密 OMPL 运动校验分辨率；
  - 在真机配置中豁免该对。
- 另有 4 个候选在 ×2 放慢后仍超过 J1 加速度限值；诊断中放慢到 ×4 也只减少 1 个，未采用。

本帧仍无完整计划。

验证：完整定向集合（含持力与独立吸盘测试）**996 通过、0 失败**；规划诊断报告见 `outputs/geometry-robustness-20261120/planning_diagnostic/`。物理验收 **NOT_RUN**。

## 2026-11-20 真实现场 STEP 转换（未配准、未导入）

用户提供 `all layout.stp`（SOLIDWORKS 2020 整体模型经 NX 12 导出，AP203，毫米），复制到被忽略的 `assets/layout/raw/`。按用户要求在 `.venv` 安装与 `requirements/eco65-cad.txt` 一致的 `cadquery-ocp-novtk==7.9.3.1.1`。NX 把装配压成单个 PRODUCT，但保留了每个实体的名字。

原转换器在开放曲面体上报 `MISSING_TRIANGULATION` 退出。布局（`cad_aabb`）模式现在：
- 实体之外的开放壳、壳之外的散面都作为独立零件保留；
- 零件名取自 STEP 实体；开放壳名取自所属 SHELL_BASED_SURFACE_MODEL，按 STEP 顺序与累计面数逐段核对（OCCT 会拆分不连通的壳），不一致则不命名并记录 `UNVERIFIED_LEFT_UNNAMED`；
- 无法三角化或 BRepCheck 无效的曲面体，只有其解析包围盒完整落在某个有效实体零件的碰撞盒内时才接受，记为被该零件包含并随其保留/剔除；否则 `SITE_LAYOUT_UNREPRESENTED_SURFACE_BODY`。实体拓扑门不变，mast/camera 的凸包模式不变。

结果：702 个零件（563 实体 + 139 开放壳）全部拓扑有效、全部有名字，开放壳命名 `STEP_ORDER_FACE_COUNT_VERIFIED`；另有 8 个亚毫米到 10 mm 的伺服电机曲面碎片包含在 3 个伺服电机实体内。CAD 坐标 +Y 向上，纸箱垛在 −Z 方向。

CAD 观察（未处理）：48 个 200 mm 立方纸箱（3×2×8）；顶层纸箱与`上盖板`在 CAD 中相交最多 46 mm；机械臂底座固定在机身 AE40-4080 横梁上，不在升降机构上；剪叉升降带动 BAS01 短输送带和 BEZ01 L1000 斜输送带下端铰点，上端铰点滑块在 HG30 导轨上，铰点距 983.0 mm、倾角 15.23°，滑块已在导轨 +Z 末端。

验证：布局测试 17 通过（新增开放壳与 STEP 名解码用例）；规定定向集合 **969 通过、0 失败、0 跳过**（262.74 s，OCP 用例不再跳过）；`m710_clearance_test optimized/reference` 与单箱原生组件检查通过。证据见 [report.json](evidence/20261120_site_layout_step/report.json)。

未完成：零件角色分类与显式剔除（机械臂、工具、纸箱）、由 CAD 机械臂底座求 `T_base_layout`、可动轴运动学模型（用户已要求，等待机构确认）、冻结 RGB-D 配准、由清单生成仿真场景、任何现场导入与运动。物理验收 **NOT_RUN**。
