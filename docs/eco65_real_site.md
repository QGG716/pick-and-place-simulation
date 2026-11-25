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

## 2026-11-21 现场布局零件角色、升降机构与仿真/现场共用场景

用户确认机构理解并要求建成可动轴：剪叉升降是唯一主动自由度 h（顶板沿 CAD +Y 相对 CAD 位置的位移），顶板带动 BAS01 短输送带和 BEZ01 L1000 斜输送带下铰点；上铰点滑块沿 HG30 导轨（CAD Z）被动滑动；机械臂底座固定在机身 AE40-4080 横梁上。

- 角色规则 `configs/workcells/eco65_site_layout_roles_v1.yaml`（绑定源 STEP SHA-256），按顺序首条命中、每条带依据；未命中的零件一律保留为固定障碍。结果：固定 74、升降顶板组 71、斜输送带 27、上铰点滑块组 20、剪叉内部 407；剔除纸箱 48、机械臂连杆 9、相机支架 3（与 `assets/mast` 尺寸一致：板 128×74×20 mm，杆 443 mm）、吸盘工具 43。穿过顶板的紧固件（下端不低于 340 mm）随顶板运动。
- `src/unloading_sim/eco65/site_mechanism.py`：闭链解析解。CAD 量得铰点距 0.983 m、倾角 15.23°；h = −0.1 m 时倾角 21.37°、滑块 −33.1 mm、板间距 226 mm。剪叉内部用一个包络盒（底板顶面到 h 处顶板下表面，覆盖全部内部零件的 XZ 范围）。保留零件为随组运动的有向解析包围盒。
- 现场行程与运行时 h 是现场输入：`site_layout.lift={h_m,h_basis,range_m,range_basis}`，无默认值，缺失即 `SITE_LIFT_STATE_REQUIRED`；范围两端都必须是有效闭链状态。滑块越过导轨端的距离只作诊断（CAD 位置已越过 24.6 mm）。
- `T_base_cad`：`tools/fit_site_layout_robot_base.py` 把官方 `base_link.STL` 用 ICP 配到 CAD 本体 `11`，RMS 0.90 mm、p95 1.55 mm；CAD +Y→base +Z，纸箱垛在 base −X，与现场记录方向一致。仍是 CAD 设计几何，现场导入前必须完成冻结 RGB-D 配准。
- 现场加载器：配置给出 `roles` 时进入机构模式（同一模型），否则原行为不变；配准面使用 h 处的非内部保留零件。仿真：`eco65_site_layout_sim_v1.yaml` + `site_layout_scene.build_site_scene()` 生成 base 系 193 个环境盒与 48 个纸箱初始箱垛（2 kg），h 取 CAD 位置并记录依据。
- 本地查看页 `tools/render_site_layout_view.py` → 被忽略的 `outputs/site_layout/site-layout-view.html`（私有，不发布），h 滑块下的倾角/滑块/板间距与 Python 一致。

验证：新增机构/场景测试 9 通过；共享工作区定向集合 978 通过、1 失败（`test_single_carton_posture_retry.py` 复合与 Python 分支一致性），原因是另一会话尚未提交的 `planner.py` 改动——在只含 HEAD 与本步改动的干净 worktree 中该文件 6/6 通过；`m710_clearance_test optimized/reference` 与单箱原生组件检查通过。证据见 [report.json](evidence/20261121_site_mechanism/report.json)。未完成：现场升降行程与 h 来源、现场配置（未修改）、RGB-D 配准、`机械臂底面连接座` 与 base_link 的起点接触检查、新场景中的规划或仿真运行。物理验收 **NOT_RUN**。

## 2026-11-21 第 4 个逆解分支与直线段降速重规划：新帧上箱得到完整离线计划

用户决定（11-21）：approach 阻挡改用第 4 个逆解分支。本节修改：

- **逆解分支**
  - `IK_BRANCHES_CHECKED` 由 3 改为 4，用户决定取代 11-16 第 3h 条"最多检查 3 个分支"的限制；分支排序（姿态偏好、离起点最近优先）不变。
  - 终点预检的行为不变：在前 4 个分支内找到第一个合格分支就返回，与原生组合预检逐步一致；C++ `sc_composite.h` 的上限同步由 3 改为 4，worker 已重新编译安装。其余未检查的分支记为待用。
  - 某分支的 approach 路径失败时，对下一个待用分支做同样的终点检查（证据后缀 `-alt1` 等），合格则接触、脱离、返回各段重新规划。这一回退目前只在 PYTHON 规划路径中实现。
- **OMPL 运动校验**：现场 worker 的 `longest_valid_segment_fraction` 已是 5e-05，不需要加密，未改。
- **直线段降速重规划**
  - 现象：Pilz LIN 段在均匀放慢到 ×2 后，关节加速度仍在节点间超限（`INTERPOLATION_LIMIT`）。
  - 处理：以 0.5/0.25/0.1/0.05 倍速度和加速度重新原生规划，同一场景、同一直线；发送轨迹仍做全段稠密审查。
  - 运动限值 0.10 rad/s、0.20 rad/s² 不变，只会更慢。

**新帧 `7fc259a5` 上箱的离线完整规划**（0.25/0.25 诊断掩码、单杯 42 N；无设备、无许可，用时 587 s）

候选 6 得到 `COMPLETE_FOUR_STAGE_NATIVE_PLAN_OFFLINE_NOT_EXECUTABLE`：
- 使用第 4 个逆解分支 `[0.26, −1.83, 1.81, −0.03, 1.12, 1.59]`：J5 保持正号，不翻腕，approach 不再扫过 `link_3`。
- 3 个吸盘密封（002/004/006）；持力最小裕度静态 1.058、发送轨迹全程 1.048（安全系数 2、摩擦系数 0.5）。
- 四段最终发送轨迹稠密检查全部 PASS：

| 段 | 时长 | 说明 |
|---|---|---|
| approach | 35.0 s | |
| contact | 3.4 s | LIN，0.25 倍 |
| detach | 27.2 s | LIN，0.1 倍，OUTWARD |
| return | 40.2 s | |

另外 27 个候选未成功：13 个 approach 超时、10 个只有 2 个吸盘持力不足、3 个插值超限、1 个 detach 各方向均失败。

**尚未完成**
- 用户 11-21 决定现场检测阈值改为框 0.25 / 文本 0.25（原 0.35/0.30），现场配置已改并记录 `detector.threshold_basis`，改前文件备份为 `…before-detector-025-20261121.json`；共享示例不变。阈值降低后可能多出低分检测（本帧画面底部大纸箱 0.346），它们按现有规则作为未恢复邻箱参与场景。
- 规划耗时远超 2 s 目标。
- 现场工作台需重启才使用新代码。
- 机械臂与真空系统当前关闭；物理验收 **NOT_RUN**。

验证：
- 完整定向集合 **999 通过、0 失败**，含新增的分支回退测试，原生组合与 Python 预检等价测试保持通过；
- `m710_clearance_test optimized/reference` 与 ECO65 原生组件检查（491 个发送点）PASS。

为安装新 worker，关闭了运行旧代码的诊断工作台（当时机械臂与真空已关）；它从 11-20 11:28 起持续运行，期间没有自行退出。

证据见 `outputs/geometry-robustness-20261120/planning_diagnostic/replay-report-ik-branch4-complete.json`。

## 2026-11-21 升降行程

用户给定升降行程 160–300 mm，按顶板到底板之间的最大间距量（底板底面到顶板顶面），以此为准，取代此前的 150–500 mm。CAD 位置该高度为 346 mm，因此 h 的机器行程为 −0.186 至 −0.046 m（CAD 位置在行程之上 46 mm）。行程写入角色文件，仿真与现场共用；现场配置只能收紧（否则 `SITE_LIFT_RANGE_WIDENS_MACHINE_RANGE`），运行时 h 仍须现场给值与依据。行程比较使用 1 µm 容差以吸收解析包围盒容差（CAD 总高 346.0001 mm）。

两端状态：下限倾角 26.87°、滑块 −71.6 mm、板间距 140 mm；上限倾角 18.03°、滑块 −13.7 mm、板间距 280 mm，滑块组包围盒（含端封与小螺栓）仍越过 HG30 导轨端 10.9 mm，仅作诊断。仿真 h 取 −0.046 m（行程上限，距 CAD 位置最近的有效状态），记为仿真选择。本地查看页滑块改为机器行程，并可选绘制官方 URDF + 吸盘工具 + 相机支架（`--robot-config` 与 `--q-record` 给出的冻结 q_start，仅显示）。验证：定向集合 **982 通过、0 失败**（442.87 s），`m710_clearance_test optimized/reference` 与单箱原生组件检查通过。证据见 [report.json](evidence/20261121_lift_range/report.json)。

未完成：运行时 h 来源、现场配置（未改）、RGB-D 配准、`机械臂底面连接座`起点接触检查、新场景中的规划或仿真运行。物理验收 **NOT_RUN**。

### 现场重启后新帧 `cda4e336`（同日）：末尾重复节点与预接触距离

机械臂上电后用新代码重启工作台（阈值 0.25/0.25、单杯 42 N）。操作者完成"准备观测 / 锁定起始点"和"规划所选箱"。上箱这次单独检出（0.268），实测 170.5×110.4×109.2 mm，包络 201.4×122.9×151.0 mm。现场规划没有完整计划：持力不足 23、approach 超时 5、插值超限 4。

**插值超限：Pilz 直线末尾重复节点**
- Pilz LIN 轨迹在终点前 0.01–0.1 ms 重复同一关节位置，但带非零速度。五次多项式插值等效出几十 rad/s² 的加速度，任何均匀放慢都无法消除。
- 处理：位置相同且间隔不足 1 ms 的相邻节点合并，保留后一个终点状态（`merge_coincident_knots`）。路径几何不变，发送轨迹仍做全段稠密审查。合并后本例最大加速度 0.167 rad/s²，在 0.20 限值内。

**approach 超时：预接触距离过近**
- 合并节点后，能持力的 9 个候选全部卡在 approach。以候选 0 为例：关节插值弧线在抵达预接触位姿（箱面前 30 mm）前，外悬吸盘的唇边擦过目标包络，间隙 1.4–2.3 mm。
- 处理：新增可选 `contact_search.precontact_m`（0.02–0.15 m，默认 0.03 不变），现场配置设为 0.06，改前备份为 `…before-precontact-60mm-20261121.json`。
- 结果：通过现场配置离线重规划同一记录（无设备、无许可），候选 14 得到 `COMPLETE_FOUR_STAGE_NATIVE_PLAN_OFFLINE_NOT_EXECUTABLE`：

| 项 | 值 |
|---|---|
| 密封吸盘 | 6 个（014–024），两行 |
| approach | 33.7 s |
| contact | 5.8 s（0.25 倍） |
| detach | 17.9 s（0.25 倍，OUTWARD） |
| return | 33.5 s |
| 规划用时 | 368.08 s |
| 其余失败 | 23 个 2/3 吸盘候选持力不足 |

3 吸盘单排候选的剥离裕度为 0.906：持力以包络中心为最坏质心，本帧包络厚 151 mm，比实测厚 42 mm；安全系数 2 不变。

**验证**：定向集合 **1001 通过、0 失败**。

**未完成**：
- 现场运行的工作台在这两项修复之前启动，需要重启才能使用；
- 现场重新规划、执行与物理验收均为 **NOT_RUN**。

证据：`outputs/geometry-robustness-20261120/planning_diagnostic/replay-report-cda4e336-precontact60-complete.json`。

## 2026-11-21 拉线传感器只读接入：协议无关部分

用户选择用只读拉线传感器提供运行时 h，并要求先做协议无关部分。现状：本机唯一串口 `/dev/ttyUSB0` 是真空板（被运行中的工作台独占），拉线传感器尚未接入本机，通信手册待提供；本步不打开任何端口、不写任何数据。

- `workbench/single_carton/lift_sensor.py`：协议层只交出 `LiftSample`（拉绳伸出米数、原始字节、校验状态、本地单调接收时刻）。配置 `site_layout.lift={source: DRAW_WIRE_SENSOR, sensor:{max_age_s, stationary_window_s, stationary_tolerance_m, uncertainty_m（各带 *_basis）, calibration: TWO_POINT 或 ONE_POINT_SCALE（带依据）}}`，全部无默认值；可选 `range_m` 只能收紧机器行程。两点标定不外推。
- 冻结：在冻结 RGB-D 的单调时刻要求全部样本已校验、最新样本不超过 `max_age_s`、样本覆盖静止窗口且窗口内极差不超过容差；h 取最新读数，区间为窗口极值换算后再加减不确定度，名义 h 必须在（收紧后的）行程内。
- 现场加载器：传感器模式必须拿到 `observation.lift_state`（`SITE_LIFT_FROZEN_SENSOR_STATE_REQUIRED`），且与当前传感器配置哈希一致（`SITE_LIFT_SENSOR_CONFIG_CHANGED`）；运动零件碰撞盒覆盖整个 h 区间：平移精确扫掠，斜输送带按 |dp/dh| ≤ 1 + r/dz 的速度上界加密采样，遗漏不超过 0.1 mm。真实清单逐高度核验 401 个高度全部包含，±5 mm 时盒体最多增大 10.4 mm。

验证：新增 36 项通过；定向集合 **994 通过、0 失败**（508.17 s）；`m710_clearance_test optimized/reference` 与单箱原生组件检查通过。证据见 [report.json](evidence/20261121_lift_sensor_contract/report.json)。未完成：协议适配器（等通信手册）、物理接线（独立 USB-RS485）、现场阈值与标定点、运行时在冻结时采集样本（`observation.lift_state`，本步未改运行时）、现场配置、RGB-D 配准。物理验收 **NOT_RUN**。

### 现场重启后新帧 `9b51a37e`（同日）：下箱选择足迹与外扩像素归属

使用修复后的代码重启工作台，操作者完成观测与规划。阈值降到 0.25 后检出 7 个框。现场结果：原位检查被 86 个观测体阻挡，最深 15.8 mm，未进入候选搜索。

**原因 1：叠箱支撑被判"下箱不唯一"**
- 上箱右后方的黑色物体（检测 5，得分 0.261）在上箱底边高度附近有 90 个接口点。
- 下箱选择沿用了场景"近目标邻域" `NEAR_TARGET_M=0.06`，即把上箱足迹四周放宽 60 mm。这个范围是 11-12 为点云细化表示引入的，11-19 P1-H 重写时被拿来在多个实例中挑下箱，没有标定依据。
- 改为：下箱的接口点必须在上箱实测足迹正下方，只放宽 3 mm 测量外扩余量（`MEASURED_EXTENT_PAD_M`）。"接口点是否足够"的佐证检查同样改用这一足迹。
- 改后真正的下箱有 1191 个接口点，黑色物体为 0。

**原因 2：上箱右侧面外扩点被另一实例掩码"占用"**
- 黑色物体的 SAM 掩码贴着纸箱右侧，344 个像素离上箱掩码 1–5 px，深度与箱侧面连续（立体匹配外扩）。另有 7 个深度跳变像素同属两个掩码。
- 改为：其他实例掩码中满足以下两项的像素，按本箱自身表面认领：
  1. 离本箱掩码不超过 6 px（`MAX_FATTENING_PX`），或是本箱箱面色度（中性色箱面不使用颜色判断）；
  2. 与本箱已测面深度连续。
- 处在深度跳变处的这类像素，按目标带内飞点剔除。只认领包络内的点，其他实例的其余像素不变。

**结果**
- 原位检查通过。
- 通过现场配置离线完整规划，候选 18 得到 `COMPLETE_FOUR_STAGE_NATIVE_PLAN_OFFLINE_NOT_EXECUTABLE`：6 吸盘（008–018）；approach 31.2 s、contact 5.8 s、detach 17.6 s（0.25 倍）、return 30.1 s；规划 389.04 s。
- 其余失败：23 个 2/3 吸盘候选持力不足，1 个接触终态唇边与观测体间隙 1.66 mm。

**验证**：定向集合 **1014 通过、0 失败**。

**未完成**：现场工作台需再次重启才使用本修复；执行与物理验收 **NOT_RUN**。

证据：`outputs/geometry-robustness-20261120/planning_diagnostic/replay-report-9b51a37e-complete.json`。

## 2026-11-21 解析有向包围盒：收紧斜输送带碰撞体

用户同意先收紧斜输送带碰撞盒。转换器（布局模式）为每个零件增加解析有向包围盒：方向取 OCCT `Bnd_OBB`，尺寸取零件 BRep 在该方向下的解析 `AddOptimal` 包围盒，保守程度与原轴对齐解析包围盒相同。机构模块对每个零件取两者体积较小者；平移扫掠按位移向量在盒轴上的投影加宽（精确），斜输送带扫掠对有向盒角点按同一速度上界加密采样。

结果（h = −0.046 m）：斜输送带碰撞体积 0.1354 → 0.0353 m³（−74%），主框架件 250×293×995 → 30×250×1030 mm，另一件 210×295×997 → 124×210×1032 mm；全部保留零件 0.3185 → 0.2182 m³。机械臂底座 |x| < 0.15 m 范围内斜输送带碰撞盒最高点降到 base z −0.237 m。真实清单 401 个高度扫掠核验全部包含（最大越界 3.6e-16 m）。重新转换后零件编号不变，702 个零件均有有向盒。证据见 [report.json](evidence/20261121_site_layout_obb/report.json)。物理验收 **NOT_RUN**。


## 2026-11-21 保存现场记录的加速对照与回放入口修复

本轮先在独立 worktree 固定 `e02e816`，避免把当时另一会话的几何修改混入性能对照。两组均使用冻结的实测箱体/保守包络、全部场景物体、原 q_start 和当前现场 60 mm 预接触距离。`cda4e336` 直接用保存场景；`7fc259a5` 使用之前保存的 0.25/0.25 掩码，按该固定源码重建一次场景后供两组共用，没有重新采集或重新推理。历史 368/587 s 没有混入本轮加速比。

发现并修复 `tools/replay_single_carton_record.py` 的测量入口问题：原脚本自行创建客户端，未经过正式入口的驻留 worker、任务重置、场景/证据服务及能力检查。现在使用 `start_task_services/start_main_worker/acquire_workers/release_workers`，输出实际选择的开关和重置记录，诊断 worker 日志写入本次输出目录。新增显式 `--prewarm-workers`，预热成本单列；驻留进程的最终退出清理不计作生产任务边界，单列并等待完成，防止两次测量重叠。保存帧入口始终不发放执行许可。

本轮只在诊断配置启用 `resident_workers=true`、`collision_check_mode=TWO_LEVEL`、`sent_path_audit=ADAPTIVE`，`planning_module=PYTHON`；没有设置统一规划截止时间，保留原请求预算、四分支回退和各档 LIN 降速重规划。本轮未修改现场文件、共享示例、120 s 有效期或任何几何/运动/压力阈值。收尾只读核对发现，另一会话已经将现场文件设为上述开关及 scene_handles/async_evidence/path_endpoint_reuse/binding_hash_cache=true、planning_module=PYTHON、planning_deadline_s=90；这些并发修改保留。本轮两个诊断配置的 SHA-256 从开始到结束均未变化，不把那一组全开配置的结果混入本次表格。

| 保存帧 | 基线搜索及审查 | 加速搜索及审查 | 加速比 | 加速组回放入口至收尾 | 另列预热 |
|---|---:|---:|---:|---:|---:|
| cda4e336 | 273.64 s | 85.51 s | 3.20× | 90.04 s | 6.72 s |
| 7fc259a5 | 521.19 s | 293.41 s | 1.78× | 298.74 s | 8.42 s |

回放入口指标包含 worker 获取、搜索、最终审查、记录尾部测量及任务释放，仍是 `PARTIAL_REPLAY_NOT_PREPARE_PLAN`，不包含冻结/推理/现场观测处理，也不是正式 `prepare_plan → PLAN_READY`。每个输入/配置只测一次，保留运行中的现场工作台，未控制背景负载；不据此统计 p95。

两帧的源文件哈希、候选排序、最终候选（分别为 14 / 18）、q_start、附着、释放结果都相同；八对阶段的完整 prepared 数据逐值相等，包括发送点的 t/q/v/a、碰撞检查路径和限值。两组所有最终阶段审查均 PASS。这个结论仅覆盖这两份固定输入的被选中计划及候选排序，不等同于全部分支/请求、跨机器 NATIVE prepare 或通用等价验收。低优先级工作因结束时刻不同可能留下不同数量的取消请求。

实际阶段耗时说明了收益和剩余问题：

- cda4e336 接近/返回最终审查从 73.94/90.81 s 降为 2.96/4.27 s；精确检查状态分别从 127719/161391 降为 828/308，其余状态仍由原位移下界证明覆盖。接触/脱离继续全采样，13953/67734 个检查状态不变，耗时从 10.51/30.27 s 降为 4.01/19.54 s。
- 7fc259a5 加速后等待候选仍为 226.29 s，接触/脱离审查为 5.60/23.01 s，接近/返回为 2.20/11.98 s。多轮有界接近/返回搜索仍主导整体耗时。
- 加速后仍记录到 169 / 316 次碰撞工作区构造，中位数 0.056 / 0.062 s，跨所有请求/worker 的累计构造时间 11.37 / 23.36 s；这些是并行请求累计值，不能直接当作整体可节省的墙钟时间。跨请求工作区复用尚未实施；即便只消除这些构造，也不能解释如何把第二帧从约 293 s 降到 100 s。

120 s 规则已经写在 11-19 的有效期章节：从冻结曝光接收时计时，视觉处理、操作者等待和执行前复核也消耗它。第一帧预热后的回放入口已用约 90 s，只剩约 30 s 给这些开销；第二帧仍直接超过有效期。低于 100 s 只是中间目标，不能单凭规划耗时认定可以执行。本轮正式成功 PLAN_READY 新样本仍为 0，至少 20 条现场样本、p95≤2 s、跨环境数值等价及物理验收均未完成。

逐次输入/源码/二进制哈希、实际阶段计时、工作区构造统计和 prepared 摘要见 [加速对照报告](evidence/realtime_20261121_field_ab/report.json)，分析脚本随证据保存。原始输出在 `outputs/realtime-field-ab-20261121/`。基准完成后将隔离工作树快进到最新已提交的 `8aa63d8` 再验证工具修改，原生/规划源码相对基准版本没有变化。本轮未连接设备、未运动、未写阀门，执行许可 0，物理验收 **NOT_RUN**。

验证：文首规定定向集合首次 **988 通过、1 跳过、0 失败**；补齐隔离工作树中实际 STEP/派生资源后，缺失的 1 项通过，规定集合按节点合并 **989 通过**。另加复审回归首次 **100 通过、7 失败**：4 项为旧测试替身接口（其中 1 项随本轮客户端入口调整），补齐后通过；3 项为真实的 150 ms 返回门槛超标（164.06 / 175.24 / 174.74 ms），原测试未改，定向复测通过，但首轮失败保留，稳定返回时限仍未验收。合并运行首轮总计 1088 通过、7 失败、1 跳过；后续重点组 51 通过、1 项私有资源路径失败，完整复制资源后的单项通过。没有把复测覆盖成“首轮全绿”。`optimized/reference/two_level/adaptive` 与单箱原生组件均 PASS（491 个发送状态）。完整命令、首轮/复测日志、源码哈希和失败分类见 [验证记录](evidence/realtime_20261121_field_ab/verification.json)。

### 现场新帧 `8b97fcd2`（同日）：轮廓前方深度伪影（用户 11-21 确认）

现场结果：叠箱支撑生效，原位检查通过，进入候选搜索。第一个可持力候选在 detach 终态被载荷包络与观测体 `rgbd_observed_5_15_13_s0` 相交挡住（原位检查只统计 >1 mm 的重叠，原生检查要求 2.4 mm）。

该体位于上箱左边沿中部，共 623 点：301 点在掩码内侧边缘，322 点在掩码外的白墙上。深度沿图像行从箱面平滑过渡到箱面前方约 10 mm，最多 14 mm，而 RGB 在掩码外是白墙。用户确认这是立体匹配的轮廓伪影。

**规则**（`site_scene._target_claims`）：同时满足以下全部条件才剔除，任何一条不满足即保留为障碍。
1. 距目标掩码边界 14 px 以内（与 7–14 px 外扩带同一上限），内外两侧都算；
2. 位于实测正面前方超过 4 mm 面带（`TARGET_PLANE_BAND_M`）；
3. 与已测面深度连续，即不跨 12 mm 跳变；
4. 不属于其他检测实例；
5. 掩码外的部分不是本箱面色。

剔除的点单独计数，记为 `silhouette_front_artifact_points_removed`，依据为 `USER_DECISION_2026_11_21_SILHOUETTE_FRONT_DEPTH_RAMP_IS_STEREO_ARTIFACT`。

**结果**：本帧剔除 217 点。按现场配置（含另一会话打开的实时化开关、规划期限 90 s）离线完整规划，候选 12 得到 `COMPLETE_FOUR_STAGE_NATIVE_PLAN_OFFLINE_NOT_EXECUTABLE`：

| 项目 | 结果 |
|---|---|
| 密封吸盘 | 6 个（002–012） |
| approach | 28.0 s |
| contact | 5.8 s |
| detach | 17.6 s |
| return | 26.3 s |
| 规划耗时 | 70.3 s |

其余失败：23 个候选持力不足，4 个接触终态唇边贴叠箱支撑体。

**验证**：新增 4 项测试，包括伪影剔除，以及箱面色、超过 14 px、深度跳变三种保留情形。定向集合 **1023 通过、0 失败**。

**证据**：`outputs/geometry-robustness-20261120/planning_diagnostic/replay-report-8b97fcd2-artifact-complete.json`

执行与物理验收：**NOT_RUN**。

## 2026-11-21 新起点/释放点、相机转向、取消 J6 限幅与 J4→J5 朝 −Z 软偏好

用户 11-21 决定：(1) 起点沿用原起点，吸具绕 J6 顺时针转 180°；(2) 取消腕部旋转限制；(3) 以 8b97 的计划为基础，让 J4→J5 朝 −Z（偏好，不写死）；(4) 释放点改到 J1 绕现有起点顺时针转 180° 处（base +Y 侧），直接从释放点规划抓下一箱。核算发现，原安装下释放点处箱堆在相机后方（光轴 +X），用户随即决定：相机在现有基础上绕 Z 顺时针转 180°，俯仰与桅杆底座不变。

**循环结构不变**：释放位姿即观测位和下一次冻结的 q_start，即原起点 J1、J6 各减 180°，`[-98.6, -13.1, -55.0, -15.8, 86.0, -185.1]°`，由操作者摆位；仍是接近、接触、脱离、带箱回 q_start、释放。

**代码**（仅 PYTHON 规划流程）：
- `wrist_branch_limits.max_j6_travel_deg` 可为 `null`，只关闭 J6 旅行门；官方关节范围保留。
- `posture_preference.wrist_down_weight`（可选）：评分加 `w·(1 − down)`，`down` 为 base 下 link_4→link_5 单位向量的 −z 分量；指标新增 `wrist_from_down_deg`。不填时评分完全不变。只参与排序，不拒绝任何状态。
- `planning_module=NATIVE` 与上述任一选项同时出现时，配置检查报错：C++ 复合请求未扩展，worker 未重编。

**相机**：新增名义外参 `configs/workbench/reference/nominal_camera_rot180_extrinsics.json`，由已标定外参绕经过光心的 link_1 Z 轴转 180° 得到，状态 `NOMINAL_USER_CAMERA_ROTATION_2026_11_21_REQUIRES_RECALIBRATION`；原标定文件字节和状态不变。以 8b97 帧核算视角：转向后相机位于 base (0.151, −0.001, 0.453)，朝 −X；机器人约占画面 6%，目标箱可见 4666 像素，被机器人遮挡 0（原相机为 13164 像素）。实际转轴位置未知，**现场转动后必须重新标定**。另外，碰撞模型中的桅杆 CAD 在 link_1 下 y ≤ 0.032 m，而标定光心在 y = 0.149 m，相机本体/支架模型与实际位置不符；这是既有问题，本次未改。

**配置**：在用的现场配置不变（J6 180°、无 wrist_down、原外参，保留今日实时化开关）。新设置放在 `outputs/workbench/single_carton.camera_rot180_nominal.json`（J6 null、wrist_down_weight 2.0、名义外参），仅在相机转好、重新标定、起点摆好后使用。

**离线验证**（不可执行，许可 0；冻结场景仍是原相机拍摄；不设截止时间）：

| 帧 / 组 | 规划 | OMPL（全失败） | 选中 | 接近终点 大臂/小臂/J4→J5 偏离 −Z | J6 最大转动 | 四段运动 |
|---|---|---|---|---|---|---|
| 8b97 原起点、原配置（用户认可） | 31.7 s | 0 | #12，6 杯 | 2.1° / 22.0° / 177° | — | 77.7 s |
| 8b97 新起点、新配置 | 68.4 s | 4 | #21，4 杯 | 2.0° / 29.5° / **2.9°** | 97.7° | 107.2 s |
| 8b97 原起点、新配置（对照） | 222.1 s | 17 | #28，4 杯 | 0.4° / 23.4° / 177° | **263.4°** | 139.2 s |
| cda4e336 新起点、新配置 | 239.3 s | 23 | #21，4 杯 | 1.7° / 29.1° / 2.6° | 96.8° | 107.7 s |
| 7fc259a5 新起点、新配置 | 216.2 s | 25 | #21，4 杯 | 1.8° / 29.3° / 2.7° | 96.8° | 107.6 s |

新起点下三帧在接近、接触、脱离终点的 J4→J5 都朝 −Z（2.5–3.4°）。接近段和带载返回段各翻一次腕（J5 变号），都是 Pilz PTP 规划，四段最终审计 PASS。8b97 的 rebuild 两组用 9edfbb5 重建场景：保存的现场场景早于轮廓伪影修复，原位即被拦。

**风险与未完成**：
- cda4e336、7fc259a5 有 23–25 次 OMPL 各耗尽 25 s，在现场 90 s 截止时间下拿不到计划。原因是偏好的朝下分支在接近段扫过下箱支撑（`part_006/012_lip`），4 个分支都试过后才轮到 #21。
- 新设置不能配原起点用：对照组 J6 转动 263°。取消 J6 限幅后，未建模的真空管与传感器线缆可能缠绕（11-14 事故背景）。
- 翻腕经过 J5 = 0；现场转相机、重新标定、摆新起点，以及新视角下的识别质量都未验证。

验证：干净 worktree（9edfbb5 + 本次改动）定向集 **997 通过、1 跳过**（升降传感器测试缺私有布局产物，与本次无关）；`m710_clearance_test optimized/reference` 与单箱原生组件 PASS；新增 `tests/test_single_carton_wrist_down.py` 5 项。查看器（仅本地）：`outputs/new-start-20261121/viewer/index.html`，起点侧别实验 `outputs/start-pose-experiment-20261121/viewer/index.html`。证据：[report.json](evidence/20261121_new_start_wrist_down/report.json)。物理验收 **NOT_RUN**。

## 2026-11-21 OMPL 避让搜索预算降到 5 s

用户依据"现场全部保存记录里 OMPL 从未成功"决定把 OMPL 预算降到 5 s。新增可选 `native_ompl_budget_s`（0 < s ≤ 100，须写 `native_ompl_budget_basis`），只用于 Pilz 直连失败后的 OMPL 请求；Pilz 仍用 `native_solver_budget_s`（25 s），统一截止时间照常截短。不填时行为不变；`planning_module=NATIVE` 时配置报错。在用现场配置与相机转向变体都设为 5 s（在用配置只增加这两行，改前备份 `…before-ompl-5s-20261121.json`）。

同一输入、同一代码，只改 OMPL 预算（不设截止时间，离线、不可执行）：

| 组 | 25 s | 5 s | 判定 |
|---|---|---|---|
| cda4e336 新起点 | 239.3 s | 128.5 s | 一致（#21） |
| 7fc259a5 新起点 | 216.2 s | 135.5 s | 一致（#21） |
| 8b97fcd2 新起点 | 68.4 s | 55.1 s | 一致（#21） |
| cda4e336 原起点 | 76.9 s | 59.1 s | 一致（#14） |
| 7fc259a5 原起点 | 240.0 s | 139.8 s | 一致（#18） |
| 8b97fcd2 原起点（无 OMPL） | 31.7 s | 39.3 s | 一致（#12），差值为负载不同 |

六组的选中候选、四段规划器、点数、时长、失败记录全部相同，发送轨迹逐点差值为 0；OMPL 仍无一次成功。剩余瓶颈：新起点下 cda4e336、7fc259a5 排名 1–4 的候选各试 4 个逆解分支，每个分支约 21 s（重规划接触/脱离约 7 s、接近直连 5–8 s、OMPL 5 s），轮到第 5 名 #21 时已约 130 s，仍超过 90 s 截止时间。

验证：干净 worktree（75230b7 + 本次改动）定向集 **1000 通过、1 跳过**（升降传感器测试缺私有布局产物，与本次无关）；`m710_clearance_test optimized/reference` 与单箱原生组件 PASS；新增 `tests/test_single_carton_ompl_budget.py` 3 项。证据：[report.json](evidence/20261121_ompl_budget_5s/report.json)。物理验收 **NOT_RUN**。

## 2026-11-22 起点/释放点分离的抓放循环与规划提速

用户 11-22 决定：从起点 S 开始规划抓箱路径，经过接近 → 吸住 → 脱垛，带箱**顺时针**转运到释放点 R，释放后再从 R **逆时针**回到 S，进行下一轮。运动规划流程参考 `feat/v0.6-eco65-moveit2-native-single-carton` 分支和历史上的实时化 15 步计划（第 0–14 步）、执行与修正记录，目标 2 s 以内。

**参考分支的结论**（只读 git 对象）：
- 该分支最终的 ECO65 循环每箱规划 300–360 s，没有现成的 2 s 流程。
- 本轮借用三点：
  - 10-20～10-22 的 J1 方向策略 V2：去程 J1 不增、回程不减，用符号检查实现，不用 MoveIt 路径约束；
  - Pilz PTP/LIN 优先；
  - M710"静态先验"的思路（离线路网，每次按当前场景重检）。第三点本轮未实现，需要用户决定。
- 历史实时化计划：第 0–7 步已提交；复审 A–F、R2-01～06 是修正；第 8–14 步未做。复审 R3（每请求重建碰撞工作区）、R6（接触/支撑段不能自适应审计）、R7（Python 开销）在本轮现场记录上同样是主要瓶颈。

**新循环**（`cycle_structure.mode=START_RELEASE_SEPARATE`，只用于 PYTHON 规划流程；NATIVE 配置报错）：

| 段 | 规划 | 说明 |
|---|---|---|
| approach | PTP（失败转 OMPL） | S → 预接触位姿 |
| contact | LIN | 吸附前，接触规则不变 |
| detach | LIN | 脱垛方案顺序不变 |
| transfer | PTP | 带载到 R = S 的 J1 −180°；J1 只减（顺时针） |
| withdraw | LIN | 释放后沿吸附面法向退离本候选的预接触距离 |
| return_empty | PTP（失败转 OMPL） | 空载回 S；J1 只增（逆时针） |

- **J1 方向**：先检查目标的符号，再检查探测路径和 5 ms 发送点的每个状态；累计反向不超过 1e-6 rad（与参考分支 V2 相同）。这是符号检查，不放宽任何碰撞规则。退离段不受方向约束，只记录 J1 变化。
- **释放后的箱子**：
  - 按完整包络停在释放位姿建模，退离段对它沿用原生接触段规则（目标位姿、面、实测箱整体刚体移到释放位姿；0.4 mm 压入和法向判据不变），回程段对它没有任何接触许可；
  - 箱子原来在垛上的位置记为 `VACATED_BY_TRANSFER`，不再作为障碍：带走的包络在每个带载状态都检查过，包络内的点按现有规则都归目标所有。这一条是实现者的建模选择，待用户确认。不保留时空载回程会扫过该位置（cda4e336 首次试算被它挡住）；
  - 箱子释放后去向不建模，承接区认证仍按用户推迟。
- **状态机**：TRANSFER_LOADED → 在 q_release 上释放（压力判据不变）→ WITHDRAW_EMPTY → RETURN_EMPTY → 回到 q_start 才 COMPLETE。停止或故障从不自动释放已持住的箱子。
- **观测位置（实现者的理解，待用户确认）**：在 S 观测。S 的 J1 与历史拍摄相同，因此用未转向的相机和在用外参；11-21 的相机转向变体对应在 R 观测。
- **配置**：在用现场配置不变；新设置在 `outputs/workbench/single_carton.start_release_cycle.json`（在用配置 + J6 不限幅 + J4→J5 朝 −Z 软偏好 + `cycle_structure`）。

**提速开关**（均可选，缺省不变，仅 PYTHON 规划流程）：

| 开关 | 作用 | 判据 |
|---|---|---|
| `parallel_screening` | 候选终态预检在常驻进程上并行 | 每个候选只用自己的 IK 种子，结果与线程时序无关；与串行时共享种子的结果可能有细微差别 |
| `lane_clients`（1–3） | 同一候选内互不依赖的段在固定进程上并行（接触→脱垛→转运 / 退离→回程 / 接近） | 每段规划与审计不变 |
| `concurrent_audits` | 待审计的段同时审计，各用不相交的进程子集（按估计耗时贪心分配） | 每段探测路径仍完整覆盖一次 |
| `sent_path_probe=GEOMETRIC` | 探测状态按五次多项式各节点区间的解析速度上界取，不再强制包含每个 5 ms 发送点 | 相邻探测状态任一物体对位移仍 ≤ 1 mm（11-16 §3b）；发送网格不变；暴力细分核对最大 0.9999 mm |

阈值、间隙、碰撞几何、规划器搜索密度、Pilz/OMPL 预算、运动限值、压力判据均未改。

**离线对照**：
- 记录：保存的三帧，q_start 改为 S（原起点 J6 −180°），冻结场景与原相机一致，计划不可执行。
- 环境：干净 worktree（90c01df + 本次改动），每组各跑一次；每次回放前等 1 分钟负载降到 2 以下；worker 预热另计；不设截止时间。
- 分组：B = 新循环 + 原机制；F = 新循环 + 上述四个开关，检查进程 12 个，车道 4 条。

| 帧 | B 规划 | F 规划 | 加速比 | 选中候选 | 六段审计 |
|---|---:|---:|---:|---|---|
| cda4e336 | 48.86 s | 24.94 s | 1.96× | #14 / #14，6 杯 | 全部 PASS |
| 7fc259a5 | 60.92 s | 28.99 s | 2.10× | #14 / #14 | 全部 PASS |
| 8b97fcd2 | 44.42 s | 24.65 s | 1.80× | #21 / #21 | 全部 PASS |

- 各帧两组的规划器、速度比例、发送放慢倍数和失败统计都相同。
- cda4e336 与 8b97fcd2 的接触段多 1 个发送点：并行预检改用每个候选自己的 IK 种子，接触起点的逆解有微小差别。
- 以 cda4e336 为例，各段的运动时长：

| 段 | 规划器 | 运动时长 |
|---|---|---|
| approach | PTP | 32.9 s |
| contact | LIN 0.25 倍 | 5.8 s |
| detach | LIN 0.25 倍 | 17.9 s |
| transfer | PTP | 30.3 s |
| withdraw | LIN 0.05 倍 | 25.0 s |
| return_empty | PTP | 39.8 s |

- 转运段 J1 从 37.3° 只减到 −98.6°；回程从 −98.1° 只增到 81.4°。

**F 组剩余耗时**（约 25 s）：

| 环节 | 耗时 |
|---|---|
| 候选生成（纯 Python） | 约 1.7 s |
| 并行预检 | 至 4–9 s |
| 车道规划 | 9–11 s |
| 六段审计 | 10–11 s |

车道规划的关键路径是"接触 → 退离 → 空载回程"：
- 退离在 R 姿态下 Pilz LIN 依次尝试 1.0/0.5/0.25/0.1 倍都失败，0.05 倍才通过，约 2.5 s；
- 空载回程 PTP 扫过 J1 180°，worker 在规划内按 0.5 mrad 边分辨率检查，约 3.5 s。

审计中三段采样审计约占 90 进程·秒（复审 R6），三段自适应审计约 27 进程·秒；每个原生请求还有重建工作区的固定开销（复审 R3）。

**离 2 s 仍差约 12 倍。** 后续都要改 C++ 或需要用户决定：
1. worker 按场景只建一次碰撞工作区（R3）；
2. 接触/支撑段的自适应审计：许可对逐状态精确检查，其余由位移下界证明（R6）；
3. 规划内对 Pilz 输出的检查与最终审计重复（改变规划内检查密度属于 11-16 §2/§3b 的不变项，需用户决定）；
4. S/R 固定位姿段的离线路网，每次按当前场景重检（参考 M710 静态先验；与 11-16 §6"计划不跨轮次"相关，需用户决定）；
5. 直线段速度比例不再逐级试，改为对直线几何做时间参数化；
6. 候选生成向量化（R7）；
7. 新循环的 NATIVE 复合请求（第 7 步扩展）。

验证（干净 worktree）：
- 新增 `tests/test_single_carton_start_release_cycle.py` 21 项：配置、J1 方向、退离几何、刚体移动、六段状态机与拒绝情形、规划链与世界、固定进程分配、并发审计覆盖、几何探测 ≤ 1 mm。
- 定向集合首轮 **1019 通过、2 失败、1 跳过**，两项失败都由本轮引起，已修复：
  - 并发审计测试留有旧分配规则的断言；
  - 预览颜色扩到 6 个后，原 UI 测试按全部颜色逐一检查线条。改为原 4 色不变，另加两色。
- 复跑 **1021 通过、1 跳过**（升降传感器测试缺私有布局产物，与 11-21 相同）。
- `m710_clearance_test optimized/reference` 与单箱原生组件检查 PASS。
- 详见 [verification.json](evidence/20261122_start_release_cycle/verification.json)。

证据：[report.json](evidence/20261122_start_release_cycle/report.json)。原始输出在 `outputs/start-release-cycle-20261122/`。

**未完成**：
- 现场摆 S、确认相机安装、在 S 观测；
- 至少 20 条现场样本的 p95；
- 2 s 目标；
- NATIVE 实现；
- 释放箱承接；
- 任何一段的实机执行。

无设备连接、无运动、无阀门命令，执行许可 0，物理验收 **NOT_RUN**。

## 2026-11-22 C++ 侧提速：解路径间隙判据、接触段自适应审计与请求精简

用户 11-22 先要求"继续 C++ 提速"，随后决定：请求接收与分发、机器人模型、场景管理、正逆运动学、运动规划、碰撞与间隙检查、抓放过程规则、轨迹准备与验证、任务管理与诊断，需要时都可以改成 C++。界面、配置、设备连接、网络恢复、模型推理和报告仍用 Python。迁移仍须显式开关（缺省不变），在保存记录上有等价报告才可以改为缺省，失败时报错、不静默回退。

**剖析**（cda4e336 起点 S，上一节 F 组配置，带原生计时）：
- MoveIt 规划管线默认开着 `check_solution_paths`，对 Pilz/OMPL 返回的每个点再做一次全网格相交检查，每点 9–12 ms。长 PTP 每次因此要 1.9–2.9 s，其中我们自己的间隙判据只占 0.3 s。
- 接触、脱垛、退离三段的审计逐点采样，detach 一段就要 36 进程·秒。
- 每个规划/审计请求约 0.8 MB JSON，其中 715 KB 是 worker 不读的接触面支撑点和实测箱字段。每次规划的原生记录在发送前同步写约 100 MB，完成后再写约 100 MB，合计 JSON 6.5 s、写盘 11 s（多线程累计，与车道争 GIL）。
- 每个任务约 585 次单条 IK 往返。
- 每请求重建碰撞工作区（复审 R3）只占 0.06–0.34 s，不是主因。

**新机制**（PYTHON 规划流程，均为可选开关，缺省不变）：

| 开关 | 位置 | 内容 |
|---|---|---|
| `solution_path_check=CLEARANCE_PREDICATE` | worker.cpp | 对规划器返回点只跑本请求的间隙判据（间隙 + 过程规则），不再跑 MoveIt 的网格相交检查。对 ECO65，每个间隙物体对的要求间隙都为正（2.4 mm，吸具-机器人 5 mm），豁免规则与相交检查相同（ACM 与 touch links），且在包含原网格的凸体上量距，所以间隙判据通过必然不相交。起点例外和失败状态与管线一致；作用域限 ECO65 单箱，要求间隙 > 0。 |
| `sent_path_audit=ADAPTIVE_CONTACT` | clearance.h | 接触段和带支撑段也只跳过由上一个精确点的力臂上界证明满足的物体对距离（证明不变；许可的唇边-目标对、载荷-支撑对属 ACM 豁免，不在证明内）。唇边压入与法向、密封覆盖、载荷支撑穿入、工作范围等随状态变化的单箱规则逐点精确计算，任何一项不满足即转完整精确检查。采样点与 SAMPLED 相同。 |
| `native_ik_batch` | worker.cpp `sc_ik_batch` | 一个请求带一个位姿的全部种子，worker 对每个种子运行完全相同的 `ik` 请求，按顺序返回全部结果，不提前停止。 |
| `compact_requests` | planner.py | 请求只带原生策略读取的面字段（中心、法向、u、v、尺寸）和实测箱位姿、尺寸。审计探测路径每段以 float64 `.npy` 落盘一次（发送前，带 SHA-256），各分块记录只记文件名和起止下标。 |
| `binary_transport` | planner.py | PYTHON 流程也使用协议 2 二进制帧。 |

另外，候选生成里的逐杯密封判定改为每个位姿只校验一次刚体矩阵、KD 树查询合并成一批（逐杯运算不变）。三帧候选输出逐位相同，2.1 s 降到 1.9 s。

**等价核对**（开发版 worker，`build/moveit-dev` → `install/moveit-dev`，不覆盖在用安装）：

| 核对 | 样本 | 结果 | 耗时 |
|---|---|---|---|
| 解路径检查 PIPELINE vs CLEARANCE_PREDICATE（每个请求在任务重置后各跑一次） | 本轮 86 个规划请求 | 86/86 状态与返回点逐值相同 | 79.4 → 23.1 s |
| 同上 | dev2 回放 205 个，含 9 个 Pilz 空载回程被路径检查否决、18 个 OMPL 预算耗尽 | 205/205 相同 | — |
| 同上 | 11-21 两份记录 163 个，含 4 个接近段 PTP 碰撞被否决、12 个 OMPL 预算耗尽；该记录来自另一 checkout，只重绑模型指纹，策略与几何仍由 worker 逐字节核对 | 163/163 相同 | 204.2 → 105.5 s |
| 审计 SAMPLED vs ADAPTIVE_CONTACT | 三帧接触/脱垛/退离整条探测路径，加"末端前推""起点回退"两种构造变体，共 27 例 | 27/27 判定、失败原因与首个失败点相同；其中 8 例被拒（密封面穿入、间隙不足、相交） | 182.9 → 26.9 s |

**回放耗时**（主目录 + 开发版 worker，每帧一次，每次回放前等负载降到 2 以下）：

| 帧 | 上一节 F | + 解路径间隙判据、ADAPTIVE_CONTACT | + 批量 IK、请求精简、二进制帧 |
|---|---:|---:|---:|
| cda4e336 | 24.94 s | 15.26 s | 12.57 s |
| 7fc259a5 | 28.99 s | 18.45 s | 11.87 s |
| 8b97fcd2 | 24.65 s | 14.16 s | 12.66 s |

三帧在三种配置下的选中候选、各段规划器、速度比例、发送点数和失败统计全部相同，六段审计全部 PASS。

**最新一版的剩余耗时**（约 12 s）：

| 环节 | 耗时 |
|---|---|
| 候选生成 | 约 1.4 s |
| 并行预检 | 至 3–4 s |
| 车道规划 | 5–6 s |
| 六段审计 | 2.5–3.8 s |

车道规划的关键路径仍是"接触 → 退离 LIN 降速阶梯 → 空载回程"：失败的 LIN 尝试每次约 0.19 s，主要是每请求的固定开销；此外还有 Python 端的发送网格准备与证据写盘。

**关于"Python 端是否都要改 C++"**：不需要整体翻译。上面剩下的 Python 开销主要来自数据量和往返次数，已用精简和批量处理去掉大半。下一步只需把每个候选的"规划链 + LIN 降速阶梯 + 发送网格准备 + 审计"循环做成 worker 内一个复合请求（扩展现有 `sc_composite`），每个候选约 100 次往返和 Python 侧的 prepare 都会消失，这是继续接近 2 s 的主要手段。

**验证**（干净 worktree，90c01df 之后的本次改动）：
- 定向集合首轮 **1023 通过、1 跳过**（升降传感器测试缺私有布局产物）。
- 开发版 `m710_clearance_test optimized/reference/two_level/adaptive` PASS，单箱原生组件检查 PASS。
- 新增测试：精简请求与探测路径引用、批量 IK 与逐条结果一致。

**未完成**：
- （已补做）用户随后要求安装并打开开关：确认现场工作台未运行，备份旧 worker（SHA-256 `3195bf51…`）后安装正式 `install/moveit`（新 SHA-256 `e43650a2…`）。`m710_clearance_test optimized/reference` 与单箱原生组件检查 PASS。新循环配置变体 `outputs/workbench/single_carton.start_release_cycle.json` 已打开上述开关并写明依据，改前备份 `…before-latency-switches-20261122.json`；现场在用配置未改（SHA-256 `7c3a7d59…`）。用正式 worker 和该变体（含 90 s 截止时间）离线回放 cda4e336 起点 S：14.36 s，候选 #14，六段审计 PASS，发送点与开发版测量逐一相同，执行许可 0；
- 复合请求；
- 2 s 目标；
- 至少 20 条现场样本的 p95；
- 实机执行。

证据：[report.json](evidence/20261122_cpp_latency/report.json)，原始输出在 `outputs/start-release-cycle-20261122/`（`cpp/`、`cpp1/`、`cpp2/`、`verification-cpp/`）。无设备、无运动、无阀门命令，执行许可 0，物理验收 **NOT_RUN**。

## 2026-11-23 六段循环规划 2 s 目标：第 0 步测量与第 1 步调度

任务见用户给出的 `eco65_planning_2s_prompt.md`：`prepare_plan` 入口到 `PLAN_READY` p95 ≤ 2.0 s，成功率不低于基线，不削弱任何检查。用户随后要求"做完第 1 步就停下并推送"，本节只含第 0、1 步；第 2 步以后未提交、未推送。

**测量口径**
- 保存记录离线回放（起点 S，计划不可执行）；`--frozen-scene`（8b97fcd2 重建场景，与 11-22 相同）；worker 预热另计；每次回放前等 1 分钟负载降到 2 以下；`ECO65_TIMING=1`（入口测量另加线程采样 `ECO65_PYSAMPLE=1`）；不设截止时间；每个配置每帧 5 次，基线与第 1 步按轮次交替。
- 三个数分开报：
  - 搜索：`plan_cycle.search_box` 一段（回放工具的 `planning_s` 另从取得 worker 之后算起）；
  - 搜索之后到 PLAN_READY：绑定、六个包注册摘要、`complete-plan.json`、证据 flush、`Cycle.ready`；
  - 入口到 PLAN_READY：新增 `--plan-cycle-entry`，走生产 `plan_cycle`（生命周期任务副本、搜索、注册、完整计划、flush、`finalize` 里的 `Cycle.ready`、记录提交），观测带本进程合成 LIVE 戳（与 `tools/check_step4_mechanisms.py` 相同做法），进程内许可返回后立即作废。不含工作台 `prepare_plan` 外壳的日志 flush 和视觉。
- 基线 = `59115a0`（HEAD 规划代码 + 测量工具）+ `config-base.json`（起点/释放点分离变体及其提速开关，无截止时间）；第 1 步 = `8b25f5b` + `config-s1-a10.json`。两者都用在用 worker `install/moveit`（第 0、1 步未改 C++）。

**第 0 步：现状（基线，5 次）**

| 帧 | 搜索 中位/最大 | 搜索之后→PLAN_READY | 入口→PLAN_READY |
|---|---:|---:|---:|
| cda4e336 | 9.69 / 9.87 s | 2.10 / 2.13 s | 11.76 / 11.98 s |
| 7fc259a5 | 11.39 / 11.59 s | 2.14 / 2.17 s | 13.57 / 13.77 s |
| 8b97fcd2 | 10.18 / 10.24 s | 2.18 / 2.20 s | 12.36 / 12.45 s |

另用 `measure_plan_tail` 的回放（`base5`，5×3）：`planning_s` 中位 10.05 / 11.80 / 10.57 s，最大 13.79 / 14.06 / 14.57 s（这一组方差大，有两轮整体偏慢）；等效尾部 2.14–2.57 s。

- 搜索内各阶段（中位）：候选生成 1.33–1.35 s；预检 1.27–2.80 s；车道等待 4.14–4.55 s；验证 2.65–2.97 s（其中并发审计 1.49–1.80 s，持力后检等 0.96–1.17 s）。
- 搜索之后约 2.1 s，全部是 Python：六个包的规范 JSON 摘要 0.64–0.67 s；证据 flush 0.68–0.72 s，基本就是 46 MB `complete-plan.json` 的序列化（0.65–0.69 s），写盘只要 0.03 s；`Cycle.ready` 对整份计划再做一次摘要 0.70–0.75 s。
- 逐请求开销（15 次回放全部请求的中位数）：

| 请求 | 墙钟 | 固定开销 | 其中工作区 | 场景复制+ACM | 网格检查 | worker 其他 | Python 序列化 | 请求体 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 终态预检 validate | 0.148 s | 0.149 s | 0.074 | 0.013 | 0.009 | 0.023 | 0.008 | 279 KB |
| 失败的规划 | 0.177 s | 0.154 s | 0.068 | 0.013 | 0.008 | 0.024 | 0.008 | 293 KB |
| 成功的规划 | 0.383 s | 0.147 s | 0.063 | 0.012 | 0.008 | 0.023 | 0.007 | 289 KB |
| 审计分块 | 0.915 s | 0.256 s | 0.079 | 0.015 | 0.010 | 0.031 | 0.052 | 1.76 MB |

  - 工作区构造中几何准备只有 1–2 ms，其余是 900 个物体、约 22 万有效对 / 16.8 万豁免对的逐对过滤和两级/自适应准备。
  - 新发现：每个请求约 280 KB 里有 263 KB 是 12 个吸盘的 CAD 轮廓，worker 在 init 时已绑定，每次只做逐字节比对；Python 端每个请求要为它做 PENDING 记录、二进制帧编码和最终记录三次序列化。
- 审计（选中候选，中位）：approach/transfer/return_empty（ADAPTIVE）探测 1.9–5.5 万个状态，精确检查 650–730 个，每个精确点 2.2–2.8 ms；contact/detach/withdraw（ADAPTIVE_CONTACT）精确 145–360 个，每个 3.9–6.7 ms（含逐点单箱规则）。`adaptiveConstraints` 的单独耗时本轮未拆分。
- Python 线程：入口测量全程 Python 线程 CPU 合计 10.2–11.8 s，墙钟 12.0–13.8 s，GIL 接近饱和。主要线程：规划任务线程（候选生成、注册摘要、`Cycle.ready`）3.8–4.0 s，证据线程 1.8–2.4 s，车道 1.9 s，车道内段链 1.5–1.7 s，预检 0.4–1.3 s。证据队列在搜索期间多数时刻为 0–9 条，flush 本身不长，长的是完整计划的序列化。
- 持力后检对脱垛和转运约 9600 个发送点逐点调用 Python FK，单线程约 2.9 s（核实了审阅里的事实 9）。

可回放记录：`outputs/` 下有观测、RGB-D 和掩码的记录共 42 个文件夹。与当前起点同一类（J1≈81°，11-17 之后）的独立帧 21 个：原有 3 个起点 S 记录，本次按同一规则（J6 −180°）新做 18 个（`outputs/planning-2s-20261122/records/`）。11-11～11-14 的 15 个是旧现场/旧起点，26ff3c7d 起点不同，未纳入。

审阅推断的核对：
- "车道阶段比原生请求多出的约 3 s 在 Python 侧"：**证实**。入口全程 Python 线程 CPU 达墙钟的 85%；第 1 步把车道从 4 条减到 1 条、Python CPU 降约 3.4 s，车道等待随之缩短约 1 s。
- "固定开销的主要成分是逐对枚举加按名字查 ACM"：**部分证实**。工作区构造是最大单项（约占固定开销一半），但其余一半来自请求拷贝与几何比对（worker 其他 0.023 s）、场景复制 0.013 s、网格检查 0.008 s、Python 序列化与传输；另有审阅未提到的 263 KB 轮廓重复传输。
- "审计精确点多是因为按组的约束过于保守"：本轮**未评估**（第 4 步）。
- "search_box 之后到 PLAN_READY 在秒级"：**证实**，约 2.1 s，单这一段就超过 2 s 目标。

**第 1 步：调度（只改 Python，判定不变）**

| 开关 / 改动 | 内容 |
|---|---|
| `pipelined_audits` + `pipelined_audit_workers` | 固定一部分常驻 worker 只做审计，车道不向它们发请求。首位候选（尚未结论的最高优先级候选）每段一规划完就按块排队审计；首位候选失败后下一个成为首位，已规划的段立即排队。规划全部结束后才下结论：规划失败即候选失败（审计结果作废）；否则按原来"探测路径短的先看"的顺序取第一个被拒的审计（脱垛/转运换下一个脱垛方案，其他段候选失败），再做持力后检。车道分配与 `parallel_candidates` 相同（rank % 车道数，固定 worker 组），每个规划 worker 的请求历史与 OMPL 结果可复现。 |
| `early_scene_upload`（需 `scene_handles`） | 候选生成前就向所有常驻 worker 并行上传本任务场景，原本首个请求才懒上传。 |
| 持力后检批量 FK | 同一 URDF 链、同一运算顺序在堆叠数组上计算，与逐点 `CoordinateModel.tcp` **逐位相同**（含有限性与官方限位检查）。 |
| 候选生成复用密封检查 | `candidate_contact_check` 的密封部分就是刚算过的同一 `seal_check`（同位姿、杯环、法向、缺省余量/间隙），直接复用，不再重建 k-d 树重算。21 个记录全部 17 个箱的候选输出（完整 JSON 的 SHA-256）与 HEAD 相同，生成合计 61.4 s → 46.6 s。 |

后两项是逐位相同的计算重写，与 11-22 候选生成的逐杯批量化同类，未设开关。审计 worker 数用两帧（cda4e336 / 7fc259a5）各一次的探索选定：10 个审计进程 + 1 条 3 进程车道 8.56 / 8.37 s；4 个 + 3 条车道 8.25 / 10.51 s；7 个 + 2 条车道 10.78 / 12.24 s（这两次与建 worktree 的磁盘负载重叠，可能偏慢）。取最大值最小的 10 个。车道少则 GIL 争用少，代价是首选候选失败时后续候选串行规划。

结果（5 次）：

| 帧 | 搜索 基线 → 第 1 步 | 搜索之后→PLAN_READY | 入口→PLAN_READY 中位 | 最大 |
|---|---:|---:|---:|---:|
| cda4e336 | 9.69 → 6.25 s | 2.10 → 2.07 s | 11.76 → **8.33 s** | 11.98 → 8.40 s |
| 7fc259a5 | 11.39 → 7.66 s | 2.14 → 2.10 s | 13.57 → **9.79 s** | 13.77 → 9.84 s |
| 8b97fcd2 | 10.18 → 6.51 s | 2.18 → 2.15 s | 12.36 → **8.65 s** | 12.45 → 8.74 s |

- 各阶段（中位）：候选生成 1.33–1.35 → 0.98–0.99 s；预检 1.27–2.80 → 1.22–2.67 s（提前上传只省 0.04–0.13 s）；车道等待 4.14–4.55 → 3.13–3.25 s；验证 2.65–2.97 → 0.65–0.99 s（审计已与规划重叠，剩下的是最后一段的审计尾巴和持力后检）。尾部未动。
- Python 线程 CPU 10.2–11.8 → 6.8–8.4 s。
- 等价核对（每帧 5 次对基线第 1 次）：选中候选、脱垛方案、各段规划器与速度比例、发送放慢倍数、发送点数、失败统计**全部相同**；六段发送轨迹逐点最大差 **0**；六段审计全部 PASS。证据字段的差异：自适应审计在分块边界重新取锚点，精确检查点数略有不同（如 cda4e336 transfer 719 → 726、withdraw 145 → 142）；被取代段的审计记录文件可能多写一次后被覆盖。

**失败样本（18 个新起点 S 记录，基线与第 1 步各 1 次，入口到返回）**

| 类别 | 记录 | 基线 | 第 1 步 a10 | 第 1 步 a4 |
|---|---|---:|---:|---:|
| 无可规划箱（不进入规划） | 045e08d1、22424c79、bd23329a、d6d985f1（无箱）、b8992c09（几何未定） | — | — | — |
| 原位包络即与场景重叠 | 075ea6ff、31e46172、5eab967d、67dbcaac、6e13d059、9b51a37e、bb81d99e、fd9ebc00 | 0.07–0.19 s | 0.07–0.19 s | — |
| 预检中目标在原位被阻挡 | 829d69ef | 2.88 s（崩溃，见下） | 2.42 s | — |
| 全部候选规划后失败 | f7ec9d1a | 46.1 s | 107.7 s | 53.7 s |
|  | 313f32ad | 95.0 s | 239.7 s | 118.2 s |
|  | fba38e25 | 96.4 s | 250.1 s | 129.4 s |
|  | 6966ce4b | 136.2 s | 396.7 s | 175.5 s |

- 21 帧里成功 3 帧（即上面三帧），18 个新记录无一成功；两组的失败统计逐项相同（829d69ef 除外）。
- 最后一类是 11-17 的四帧，每个候选都要把脱垛方案、OMPL 回退试完。以 6966ce4b 为例：99 次 OMPL 回退全部用满 5 s 预算（`NATIVE_SOLVER_BUDGET_EXHAUSTED`，合计约 500 s worker 时间，0 次成功），655 次 Pilz 失败；候选生成本身 7–9 s。
- **第 1 步在这类帧上变慢**：基线 4 条车道分摊候选，a10 只剩 1 条车道，慢 2.3–2.9 倍；a4（3 条车道 + 4 个审计进程）慢 17–35%。流水审计要从规划车道划出进程，成功帧上的收益是用失败帧的吞吐换来的。所以本轮**不建议在变体配置里打开 `pipelined_audits`**，开关保持缺省关闭，变体配置未改。可行的改法（未做）：审计块不独占进程，而是插到空闲的规划进程上；或先解决失败分支本身（E 项 OMPL 回退、第 6 步让注定失败的候选尽早否决）。
- **既有缺陷（HEAD，本轮未修）**：基线跑 829d69ef 时得到 `KeyError: 0` 而不是"目标在原位被阻挡"。原因在 `parallel_screen`：某候选触发 `TargetBlocked` 后其他预检车道被取消，被取消的请求抛出 `RuntimeError`（不是 `ValueError`），该车道线程直接结束、没有登记它的候选，随后按候选取结果时报 `KeyError`。与时序有关，第 1 步那次碰巧没有触发。影响打开 `parallel_screening` 的配置（起点/释放点分离变体打开了，在用现场配置没有）。

**离 2 s 还差多少**（第 1 步后，成功帧中位）：入口到 PLAN_READY 8.3–9.8 s，约 4–5 倍；全部候选失败的帧 46–136 s（基线），这一类决定 p95。按环节：搜索之后 2.1 s；候选生成 1.0 s；预检 1.2–2.7 s；车道 3.1–3.3 s；验证 0.65–1.0 s。工作树里已写好但**未提交、未做 5 次测量**的第 2 步及尾部改动（`process_geometry_ref`、LIN 阶梯单请求、工作区按名字对记忆、结构化计划摘要、二进制完整计划记录）留待下一轮单独测量和提交。

验证（第 1 步提交 `8b25f5b` 的干净 checkout）：
- 定向集合（同 11-22 命令）首轮 **1031 通过、1 跳过**（升降传感器测试缺私有布局产物，与之前相同）。开发中新测试文件首轮曾有 1 项失败，是测试本身的假 worker 未声明 `scene_handles` 能力，已改测试后通过。
- `build/moveit/m710_clearance_test optimized/reference` 退出码 0；用在用 worker 跑第 1 步代码的单箱原生组件检查 PASS。
- 新增 `tests/test_single_carton_planning_2s.py` 6 项：开关校验；流水审计与顺序核对得到相同计划，规划与审计 worker 不相交，每段探测路径被连续分块恰好覆盖一次；四种被拒组合下与"短路径先看"的结论相同；提前上传到达每个 worker；批量 FK 逐位相同。

**未做 / 未验证**
- 第 2–6 步；A–F 各项待用户决定，本轮未走到。
- 车道/审计进程的分法只做了单次探索和失败帧单次对比，未做 5 次；不改变失败帧吞吐的审计调度方式未实现。
- `parallel_screen` 取消竞态（见上）未修。
- 至少 20 条现场样本的 p95；现场实机。

证据：[report.json](evidence/20261123_planning_2s_step1/report.json)、[verification.json](evidence/20261123_planning_2s_step1/verification.json)；原始输出在 `outputs/planning-2s-20261122/`（`base5/`、`base-entry/`、`s1-entry/`、`scan-*`、`verification-step1/`）。无设备连接、无运动、无阀门命令，执行许可 0，物理验收 **NOT_RUN**。

## 2026-11-23（续）六段循环规划 2 s 目标：第 2 步测量、第 4/5 步与调度改进

接上节。本轮做了：第 2 步的 5 次测量与等价核对；修了两个缺陷；第 5 步候选生成；第 4 步审计约束的验证；几项调度和原生改进；A–F 的离线诊断。第 3 步（进程内原生候选循环）**没有做**，原因和估计见文末。所有新机制都是显式开关、缺省关闭，在用现场配置和起点/释放点分离变体都未改；原生改动只编进开发版前缀（`install/moveit-dev`、`moveit-dev2`、`moveit-dev3`），正式 `install/moveit` 未动。

**缺陷修复**
- `structured_plan_digest` 一打开就崩：计划里有 `numpy.float64` 字段，树摘要按 `type(x) is float` 判断而拒收（规范 JSON 摘要能接受，因为它是 float 子类）。第一次冒烟回放即返回 NO_PLAN。改为与 `json.dumps` 一致，并保证"摘要 = JSON 往返后的摘要"（c5aeb2c，加回归测试）。上一轮 901ed41 提交的这个开关此前从未在回放上跑通过。
- `parallel_screen` 取消竞态（上节记录的 `KeyError: 0`）：被取消的预检请求抛 `RuntimeError` 时记为 STOPPED，其他错误记为 ERROR，全部车道结束后依次抛出目标级阻挡、取消、错误（dc5d9ff，加回归测试）。

**第 2 步测量**（采样器开，与第 1 步同口径；代码 c5aeb2c，开发版 worker `install/moveit-dev` = 901ed41 原生代码；每帧 5 次，按轮次交替；入口到 PLAN_READY）

| 配置 | cda4e336 中位 / 最大 | 7fc259a5 中位 / 最大 | 8b97fcd2 中位 / 最大 | 搜索之后→PLAN_READY（中位） |
|---|---:|---:|---:|---:|
| 基线（第 1 步报告） | 11.75 / 11.98 s | 13.57 / 13.77 s | 12.36 / 12.45 s | 2.10–2.18 s |
| 第 1 步 a10（第 1 步报告） | 8.33 / 8.40 s | 9.79 / 9.84 s | 8.65 / 8.74 s | 2.07–2.15 s |
| b0 | 11.02 / 12.48 s | 12.61 / 13.17 s | 11.18 / 11.22 s | 2.09–2.21 s |
| b2 | 9.10 / 9.33 s | 10.17 / 11.25 s | 9.91 / 11.20 s | 2.02–2.14 s |
| b3 | 7.61 / 7.66 s | 8.62 / 8.75 s | 7.76 / 8.26 s | 0.49–0.50 s |
| s3 | 5.54 / 5.82 s | 6.09 / 6.40 s | 5.67 / 5.76 s | 0.48–0.49 s |

- b0 = 起点/释放点分离变体原有开关（第 1 步的逐位等价改写和原生按名字对记忆是无条件的）；b2 = b0 + `early_scene_upload` + `process_geometry_ref` + `native_lin_ladder`；b3 = b2 + `structured_plan_digest` + `binary_plan_record`；s3 = 第 1 步保留进程流水审计配置（10 个审计进程、1 条车道）+ 上述四个开关。
- 各段（中位）：`process_geometry_ref` + LIN 阶梯单请求使预检 1.3 → 0.9 s、车道 4.3 → 3.0 s；结构化摘要 + 二进制计划记录使搜索之后 2.1 → 0.5 s（6 个包摘要 0.62 → 0.15 s，`Cycle.ready` 0.67 → 0.18 s，完整计划写盘改为 JSON 骨架 + 内容寻址 `.npy`）。
- 每请求固定开销（b2）：端点校验 0.149 → 0.088 s，其中工作区构造 0.044 s、场景复制 + ACM 0.013 s、网格检查 0.008 s；规划请求里最大的单项是规划器返回路点的间隙判据，中位 140 ms。
- 等价核对：60 次回放的选中候选、脱垛方案、各段规划器与速度比例、发送放慢倍数、发送点数、失败统计与基线**全部相同**，六段发送轨迹逐点最大差 **0**，六段审计全部 PASS。证据字段的差异：LIN 阶梯的各档尝试记在一个请求的 `lin_retry` 里，`mtc_attempt_index`、`pipeline_calls` 与逐档请求时不同。
- 失败帧（b3，1 次）：入口到返回 829d69ef 2.0 s、f7ec9d1a 31.9 s、313f32ad 74.4 s、fba38e25 76.7 s、6966ce4b 116.1 s（基线 2.9 / 46.1 / 95.0 / 96.4 / 136.2 s；b0 2.3 / 41.8 / 89.4 / 89.8 / 131.2 s），结局全部 NO_PLAN，与基线相同。b0 在 829d69ef 上再次出现 `KeyError: 0`（c5aeb2c 尚未含竞态修复；b3 那次碰巧没有触发），修复后的结果见队列 3。

**新机制与改写**

| 项 | 开关 | 内容 | 判定 |
|---|---|---|---|
| 第 5 步候选生成 | 无（逐位等价改写） | 吸盘环点合并成一个数组做变换与投影（3000 个随机位姿上逐环逐位相同）；k-d 树只查询通过矩形/间隙/角度精确检查的吸盘，查询上界设在支撑半径之上，环点未覆盖就不查边距带；环点列表每次调用只转换一次；每个面只建一棵树 | 21 个记录全部 17 个箱的候选完整 JSON SHA-256 与改前相同；合计 45.2 → 18.8 s，成功帧每箱 0.93 → 0.28 s |
| 首请求凸体预建 | 无（同一缓存，提前建） | worker 初始化时就建好 TWO_LEVEL 工作区复用的 715 个机器人/吸具凸体进程缓存，原来由每个 worker 的第一个请求付 0.2–0.3 s | cda4e336 每个 worker 首个校验 287–422 → 75–122 ms；clearance 测试与原生组件检查 PASS |
| 空闲推测进程上的流水审计 | `pipelined_audit_mode=IDLE_SPECULATIVE_WORKERS` | 不划出审计进程，保留 4 条车道；首位候选的审计块在其他车道进程的请求间隙执行，推测性车道在请求边界让路；首位候选失败后其审计作废。规划请求在各自固定进程上的顺序不变 | 假 worker 测试：1/2 条车道与顺序搜索同一计划，四种被拒组合与"短路径先看"结论相同 |
| 解释器切换间隔 | `python_switch_interval_s` | 只在规划期间把 GIL 切换间隔调小，返回时恢复 | 只影响调度 |
| 提前持力后检 | `early_holding_postcheck` | 在等审计时就计算持力后检，审计全部通过后才按原顺序采用；审计被拒则丢弃、不写证据 | 三种情况（通过/持力不足/审计被拒）、顺序与流水两种调度下结局相同 |
| 物体对列表缓存 | `workspace_pair_cache` | worker 进程内按（场景指纹、allowed pairs、附着物 id 与 touch links、间隙策略）缓存过滤后的物体对列表，物体序列签名也一致才复用；任务重置时清空 | `ECO65_VERIFY_PAIR_CACHE=1` 下两帧回放每次命中都与直接计算逐项一致；命中率只有 31% / 55%（每个 worker 对每种阶段上下文只见一两次），收益有限 |
| 发送网格准备放进子进程 | `prepare_processes`（1–8） | 常驻 spawn 子进程池执行同一个 `prepare`（五次插值 + GEOMETRIC 探测网格），输入输出按值传递，Python 主进程不再在 GIL 下算它 | 队列 4 的 39 次回放（b7n / b9n / b8n）发送点逐点差 0；车道等待 −0.65～−0.87 s |
| 按实体的自适应约束（第 4 步） | `adaptive_constraints=PER_BODY` | 近距离物体对 / 运动实体各用自己的下界与力臂，按 1/3/10/25 mm 余量分桶合并；每组保留"50 mm 上限 × 组力臂"兜底 | 见下 |

**尝试后未采用**：规划器返回路点的自适应判据（`CLEARANCE_PREDICATE_ADAPTIVE`，用审计的力臂下界跳过可证明的路点）。在 445 个保存的 Pilz 请求上状态、返回点、无效路点下标与首个失败全部相同，但只有 10–34% 的路点能被证明（Pilz 路点间距相对障碍附近的余量太大），记录模式与锚点约束的开销大于省下的检查：80.2 → 84.9 s。补丁未提交，留作证据。

**第 4 步：审计**
- 审阅推断"精确点多是因为组内最小间隙 × 组内最大力臂过于保守"：**部分证实**。按实体约束在三帧 × 六段 × 三种路径（原路径、末端前推、起点回退）54 例上与 SAMPLED 的判定、失败原因、首个失败边/比例/状态/物体对**全部相同**（含 8 例被拒），精确检查 23582 → 19678（−16.6%，转运段约 −35%）。但逐对建约束（未分桶）时每个探测状态的证明要遍历几百条约束，反而更慢；分桶后总审计耗时基本持平（转运段 −6% 到 −22%，其余持平）。不建议改缺省。
- 合成测试（`m710_clearance_test adaptive` 扩展）：按实体证明的 10370 个样本全部通过精确复检，组约束能证明的样本按实体约束都能证明，600 条随机路径与 SAMPLED 判定、首个失败点相同。

**队列 3 / 4 测量**（不开线程采样器；队列 3：代码 b37f01f，worker `install/moveit-dev2` = 第 2 步原生 + 首请求凸体预建，12:50 提前停止，b3ni / b4ni / b5ni 各只有 1 次；队列 4：代码 a144fe0，worker `install/moveit-dev3` = 92824fe 原生；b3n 第 1–2 次来自队列 3、第 3–5 次来自队列 4，两者新开关都关闭；入口到 PLAN_READY，中位 / 最大）

| 配置 | 次数 | cda4e336 | 7fc259a5 | 8b97fcd2 | 车道等待（中位，三帧） | 校验（中位，三帧） |
|---|---:|---:|---:|---:|---:|---:|
| b3n | 5 | 6.30 / 6.69 s | 7.27 / 7.38 s | 6.88 / 6.97 s | 2.73 / 3.31 / 3.20 s | 2.05 / 1.71 / 2.02 s |
| b7n | 5 | 5.43 / 5.54 s | 6.35 / 6.65 s | 5.82 / 5.89 s | 2.08 / 2.44 / 2.45 s | 1.89 / 1.70 / 1.92 s |
| **b9n** | 5 | **5.13 / 5.24 s** | **5.92 / 5.96 s** | **5.25 / 5.39 s** | 2.05 / 2.47 / 2.27 s | 1.69 / 1.34 / 1.50 s |
| b8n | 3 | 5.38 / 5.52 s | 6.35 / 6.46 s | 5.56 / 5.71 s | 3.03 / 3.37 / 3.15 s | 0.87 / 0.82 / 0.90 s |
| b3ni | 1 | 6.34 s | 7.55 s | 7.08 s | 2.47 / 3.37 / 3.52 s | 2.42 / 2.02 / 2.05 s |
| b4ni | 1 | 6.83 s | 7.50 s | 7.16 s | 3.99 / 4.01 / 4.56 s | 1.33 / 1.29 / 1.03 s |
| b5ni | 1 | 6.58 s | 7.59 s | 7.01 s | 4.54 / 4.54 / 4.17 s | 0.54 / 0.75 / 1.32 s |

- 配置：b3n = b3（不开采样器）；b3ni = b3n + `python_switch_interval_s=0.0005`；b4ni = b3ni + `pipelined_audits` + `pipelined_audit_mode=IDLE_SPECULATIVE_WORKERS`；b5ni = b4ni + `early_holding_postcheck`；b7n = b3n + `prepare_processes=4`；b9n = b7n + `early_holding_postcheck` + `adaptive_constraints=PER_BODY` + `workspace_pair_cache`；b8n = b5ni + `prepare_processes=4`。各配置相对起点/释放点分离变体文件的差异与 SHA-256 见 `configs.json`（回放配置按测量规程去掉了 `planning_deadline_s`；变体文件本身未改）。
- 等价核对：队列 3 / 4 共 63 次回放，选中候选、脱垛方案、各段规划器与速度比例、发送放慢倍数、发送点数、失败统计与基线**全部相同**，六段发送轨迹逐点最大差 **0**；全部到达 PLAN_READY（六段审计全部 PASS 是其前提）。同一配置内各次的判定也全部相同。
- b3 → b3n（7.61 → 6.30 s 等）主要在前段：候选生成 1.00 → 0.27 s（第 5 步 + 不开采样器）、预检 0.9 → 0.6 s（首请求凸体预建 + 不开采样器）；车道与校验基本不变。采样器本身的开销没有单独测。
- **`prepare_processes`（b3n → b7n）是本轮最大的单项收益**：车道等待 −0.65～−0.87 s，入口到 PLAN_READY −0.87～−1.06 s，直接证实 GIL 把 `prepare` 放大了 10–20 倍。
- b7n → b9n 再 −0.30～−0.57 s，主要来自提前持力后检（校验里持力部分 0.31–0.43 → 0.01–0.02 s），8b97fcd2 的车道等待另少 0.18 s；按实体约束与物体对缓存在并发审计上测不出（1.59 / 1.31 / 1.48 → 1.61 / 1.33 / 1.50 s），与前面单独核对时"精确点 −16.6%、总审计耗时持平"一致。
- 空闲推测进程上的流水审计（b4ni / b5ni / b8n）：校验阶段降到 0.5–1.3 s，但车道等待增加 0.6–2.1 s（审计块与规划请求抢 CPU 和 GIL），b4ni / b5ni 相对 b3ni 为 −0.07～+0.49 s（各 1 次）；加上 `prepare_processes` 后（b8n）与 b7n 持平或略好，三帧都不如 b9n。不建议采用。
- `python_switch_interval_s`（b3ni 1 次对 b3n 5 次中位）：6.34 / 7.55 / 7.08 对 6.30 / 7.27 / 6.88 s，没有收益。
- 尝试后未采用：并发审计按实测各段审计耗时重新分配进程子集（`audit_cost_estimate=MEASURED_20261123`，配置 b10），两帧各 1 次，并发审计 1.58 / 1.98 s，同帧 b9n 为 1.69 / 1.57 s，没有改善；代码已撤回，未提交。

**剖析结论（现在时间花在哪里）**

最好配置 b9n 的中位（cda4e336 / 7fc259a5 / 8b97fcd2）：

| 阶段 | cda4e336 | 7fc259a5 | 8b97fcd2 | 说明 |
|---|---:|---:|---:|---|
| 入口 → 车道启动 | 0.98 s | 1.58 s | 0.99 s | 其中候选生成 0.26 s、预检 0.58–1.22 s；7fc259a5 预检了 22 个候选（cda4e336 9 个）才凑满 4 条车道，整次回放原生请求 196 对 104 |
| 车道等待 | 2.05 s | 2.47 s | 2.27 s | 选中候选关键链上的原生请求合计约 0.9 s（b3 上量得），其余是 Python 间隙、发送网格准备的排队与等待 |
| 校验 | 1.69 s | 1.34 s | 1.50 s | 并发审计 1.33–1.61 s；持力后检已提前算完 |
| 搜索结束 → PLAN_READY | 0.44 s | 0.44 s | 0.45 s | 6 个包摘要、注册、二进制计划记录、`Cycle.ready` |
| 入口 → PLAN_READY | 5.13 s | 5.92 s | 5.25 s | |

**审阅推断核对（更新）**
- "车道阶段多出的约 3 s 在 Python 侧"：证实（上节）；本轮进一步定位并部分消除：GIL 下 I/O 与原生回复后重新拿 GIL 要排队，73 次 PENDING 记录写盘合计 1.66 s（数据只有 2.2 MB），单线程 23–83 ms 的 `prepare` 在回放里变成 0.06–0.6 s；把它移到子进程后车道等待少 0.65–0.87 s。
- "固定开销的主要成分是逐对枚举加按名字查 ACM"：部分证实，见上节；按名字对记忆和物体对缓存之后，工作区构造仍占端点校验 0.088 s 中的一半，缓存命中率受 worker 分散限制。
- "审计精确点多是因为按组约束保守"：部分证实，见第 4 步。
- "search_box 之后到 PLAN_READY 在秒级"：证实（上节 2.1 s），本轮降到约 0.5 s。

**A–F：离线诊断数据（待用户决定，未实施，诊断未进入任何配置）**

- **A 退离方向改为吸具自身法向**：只有空载回程链是关键链时有收益。b3 三帧各 5 次：cda4e336 空载链比转运链晚完成 0.35–0.64 s，退离若能在接触前规划约可省接触段时长（约 0.3 s）；7fc259a5 转运链才是关键链（空载链早 0.29–0.61 s，收益 0）；8b97fcd2 两链相差 −0.39～+0.15 s。跨轮次复用另涉及 11-16 §6。我的建议：暂不做。
- **B 规划内逐路点间隙判据并入审计**：b3 回放（3 帧 × 5 次）全部规划请求的判据耗时中位：contact 90 ms、detach 200 ms、transfer 187 ms、withdraw 102 ms、return_empty 150 ms、approach 246 ms；关键链 contact→detach→transfer 合计约 0.48 s。成功帧上判据一次也没有否决（三帧各 1 次，73 个规划请求）。失败帧（基线，4 帧）上，转 OMPL 的 PTP 几乎都是被判据否决的：approach 79 次 PTP 中 78 次被否决、转 OMPL 78 次；transfer 69 次中 68 次、转 OMPL 69 次；return_empty 79 次中 79 次、转 OMPL 79 次；detach LIN 被否决 166 次。并入后这些 PTP 不再转 OMPL，失败改在审计发现，失败原因标签改变；保存记录上这些回退 0 次成功，结局预计相同，但未逐例重放验证。发送轨迹已包含全部路点并被稠密审计。另试过不改语义的加速（用力臂下界跳过可证明的路点），无收益，见上。
- **C prepare 的等价验收方式**：第 3 步没有开始，本轮没有走到，无新数据。
- **D 直线段用时间参数化代替降速阶梯**：三帧 contact/detach/withdraw 的阶梯剖面在节点处只能再均匀压缩 1.0–1.6 倍（已接近关节限值）；发送时为消除节点间插值超限又放慢 ×2.0，时长翻倍。直接沿直线做时间参数化，估计发送时长缩到约 40–60%（退离 25 s → 约 10 s，脱垛 17.9–31.9 s → 8.8–15.4 s）。GEOMETRIC 探测网格与时间缩放无关，审计不变；对规划延迟只减少 `prepare` 与持力后检的点数，收益小，对节拍收益明显。
- **E 2 s 截止时间下的 OMPL 回退**：基线失败帧回放里 226 次 OMPL 回退（approach 78、transfer 69、return_empty 79），0 次成功（216 次预算耗尽、10 次请求失败），中位 5.12 s，全部出现在失败帧上；b3 / b3n / b9n 也各 226 次、0 次成功。跳过回退的离线诊断（回放工具 `--diagnostic-skip-ompl`）：b9n 配置，5 个失败帧各 1 次：结局全部仍是 NO_PLAN，失败统计（各类别计数与原因）与带回退的 scan-b9n **逐项相同**；入口到返回 f7ec9d1a 25.8 → 11.9 s、313f32ad 65.6 → 19.4 s、fba38e25 67.5 → 19.2 s、6966ce4b 106.5 → 24.9 s（分别少跑 32 / 71 / 73 / 99 次回退），829d69ef 没有回退（1.0 s 不变）。成功帧上从未触发回退；2 s 截止时间内也放不下一次 5 s 预算的 OMPL 搜索。可选：(a) 维持现状，截止时间到时报预算耗尽并给出未搜候选数；(b) 截止时间下剩余时间不足 OMPL 预算时不启动回退，该候选记为未搜（不是不可行），其他不变；(c) 被判据否决的 PTP 不再转 OMPL（与 B 相连）。我的建议是 (b)：不改变任何判定，只把搜不完如实记为未搜。
- **F PLAN_READY 前必须落盘的证据范围**：第 2 步之后 flush 只比计划记录本身的写盘多约 0.01 s（b3：flush 0.12 s，其中二进制计划记录 0.11 s），按原文收窄已几乎没有收益。

**离 2 s 还差多少**

- 成功帧（b9n）：中位 5.13 / 5.92 / 5.25 s，最大 5.24 / 5.96 / 5.39 s；目标 p95 ≤ 2.0 s，**还差 3.1–4.0 s，未达到**。相对第 1 步基线（11.75 / 13.57 / 12.36 s）约 −56%，但基线带线程采样器、b9n 不带，下降里含采样器开销（未单独测）。成功率：3 个成功帧 15/15 到达 PLAN_READY，失败帧结局与基线相同，不低于基线。
- 按阶段：车道等待 2.05–2.47 s（关键链原生时间约 0.9 s）、审计 1.33–1.61 s、入口到车道启动 1.0–1.6 s、尾部 0.44 s。推算（未测）：即使第 3 步把车道降到原生关键链（约 1.0 s）、预检降到约 0.3 s、尾部降到约 0.2 s，合计仍约 3.2 s；要到 2 s 还需要审计再少约 1 s 或与规划重叠（GIL 下的共享流水已证明不行，需在原生线程里做），以及 B 项把规划内判据（关键链上约 0.5 s）并入审计。
- 失败帧（b9n，1 次）：入口到返回 829d69ef 1.0 s、f7ec9d1a 25.8 s、313f32ad 65.6 s、fba38e25 67.5 s、6966ce4b 106.5 s（基线 2.9 / 46.1 / 95.0 / 96.4 / 136.2 s）；不跑 OMPL 回退时 1.0 / 11.9 / 19.4 / 19.2 / 24.9 s，剩下的是失败帧要把全部候选的六段规划搜完。2 s 截止时间下这些帧会以预算耗尽结束（未搜候选计数可见），不会被报成不可行。

**第 3 步为什么没做**

第 3 步要把"预检 → 六段规划 → 发送网格（含 GEOMETRIC 探测网格）→ J1/腕部规则 → 审计 → 持力后检"放进一个进程内用线程完成，Python 只发一个请求。现有 `sc_composite` 只覆盖四段流程，C++ `native_prepare` 没有 GEOMETRIC 网格，持力后检、J1 方向、释放后场景、IK 分支回退只有 Python 实现；按规则还要先在保存记录上做等价报告，并先解决 C 项。这是多日工作量，本轮没有开始，以免留下半成品。按本轮剖析，它针对的是：车道阶段的 Python 间隙（选中候选关键链的原生请求合计约 0.9 s，而车道等待在 b9n 上仍有 2.1–2.5 s）、预检的逐请求往返（约 0.4 s）和尾部的包摘要（约 0.3 s）。

**验证**

- 最终代码 HEAD a144fe0 的干净分离检出（`/tmp/eco65-2s-wt`），原生用 `install/moveit-dev3`（= 92824fe 原生；a144fe0 只改 Python）：`m710_clearance_test` optimized / reference / adaptive / two_level 全部 PASS；单箱原生组件检查 PASS；定向 pytest（55 个文件，不加载 ROS 环境）**首次运行 1056 passed、1 skipped**（跳过的是需要私有布局产物的升降传感器测试，与之前相同）。
- 92824fe 上的首次运行在加载了 ROS 环境的 shell 里收集出错（launch 测试插件缺 `lark`），日志保留为 `verification-final/pytest-collection-error-ros-env.log`；不加载 ROS 环境重跑 1054 passed、1 skipped。每个原生提交前的 clearance 与原生组件检查在 `outputs/planning-2s-20261122/verification-*`。
- 正式 worker `install/moveit/lib/m710_moveit_backend/m710_moveit_worker` 的 SHA-256 仍为 e43650a2…（与第 1 步记录相同），未覆盖；在用现场配置与起点/释放点分离变体文件未改。

**未做 / 未验证**

- 第 3 步未做，C 项因此没有走到；A、B、D、E、F 未实施，等用户决定。
- 新开关都没有进起点/释放点分离变体配置（变体配置未改），只在离线回放上测量；至少 20 条现场样本的 p95 仍未做；现场实机未做。
- "请求内不再拷贝 world JSON"（第 2 步清单最后一项）未做：句柄展开时仍把约 76 KB 的 world 复制回请求，未单独计时，归在 worker 其他开销（中位约 20 ms，含解析）里。
- 释放后场景（withdraw / return_empty）没有场景句柄，每个 worker 第一次见到时整份导入（中位 35 ms）。
- 物体对缓存和按实体约束只在 b9n 里与提前持力后检一起测了 5 次，没有单独测；从阶段数据看它们的墙钟收益接近 0。
- `prepare_processes` 只测了 4 个子进程，没有扫进程数；失败帧每个配置只跑 1 次。
- 正式 worker 未更新；本轮新提交尚未推送。

证据：`docs/evidence/20261123_planning_2s_step2/`（`report.json` 全部测量与等价核对、`decision-data.json` A/B/E 数据、`decision-d-diagnostic.json` D 数据、`configs.json` 回放配置差异、`verification.json` 最终检查）；`docs/evidence/20261123_planning_2s_step2b/`（自适应判据尝试与撤回补丁、物体对缓存核对、首请求凸体预建）；`docs/evidence/20261123_planning_2s_step4/per-body-constraints.json`；`docs/evidence/20261123_planning_2s_step5/candidates.json`；原始回放在 `outputs/planning-2s-20261122/`（不入库）。无设备连接、无运动、无阀门命令，执行许可 0，物理验收 **NOT_RUN**。

## 2026-11-24 视觉模块：看不到的厚度、共面边界、实体不互穿、通用放置接触（用户 2026-11-23 决定）

起因：11-23 晚现场 9 帧（7d34bd14、cbfee7bc、564cfcde、c07b50b5、8d733bfc、90c7aaed、8af1e4ef、ede9b41e、b7037b72）没有一帧得到完整计划，失败都在视觉：看不到侧面、测不出厚度（近 7 天单项最多，53 次）；补到 0.40 m 的包络撞上邻物或工作区，或者把重心推远，导致吸附力判不过；"确认两箱叠放"的 15 mm 高差、唯一下层等条件；共面邻箱整箱拒绝；同一箱子重复检测。用户决定：(1) 厚度上界按现场箱型，0.40 m 改为 0.20 m；(2) 共面边界不再拒绝，包络延伸到下一个真实边界；(3) 两个实体不能互相穿入；(4) 用通用"放置接触"规则替代确认两箱叠放。

全部是开关，默认关闭，共享示例和在用现场配置都没改；现场变体配置为 `single_carton.observe_release_vision5_20261123.json`（证据目录里有副本）。

| 开关 | 内容 |
|---|---|
| `max_carton_extent_m=0.20` + 依据（原有参数） | 箱子任一维上界：缺边补长、只见正面时的厚度、未识别邻箱沿视线的遮挡深度都用 0.20 m；实测超过 0.20 m 的物体（后方大板、底层大扁箱）不再当已测箱体，按未识别障碍处理 |
| `single_face_depth_policy={mode: BOUND_BY_MAX_CARTON_EXTENT}` | 只拟合出正面时不再报 `BOX_THICKNESS_UNOBSERVED_SHOW_SIDE`：实测箱体厚度用 1 mm 占位（原生绑定要求正值，记 `depth_measured=false`、`measured_thickness_m=null`），包络背面一律按缺边处理（正面轮廓像素不能当背面直接边界），由最大外形或空域射线封口；面后面的回波（斜看到的顶面）不算不一致 |
| `coplanar_continuation_policy={mode: EXTEND_ENVELOPE_TO_NEXT_BOUNDARY}` | 正面在掩码外共面延续、RGB 边线不足 70% 时不再报 `COPLANAR_NEIGHBOR_BOUNDARY_NOT_OBSERVED`，记下延续像素和边线覆盖率；那几条边按缺边处理，由空域射线、接缝策略、他物实测面或最大外形封口；吸盘仍只放在实测正面上 |
| `non_interpenetration_policy={mode: OTHER_INSTANCE_SURFACES_CUT_ENVELOPE}` | 新模块 `non_interpenetration.py`：其他检测实例的有效回波是别的实体的实测表面。一个回波只在一个轴、一侧超出本箱实测箱体（加边距），另两轴都在实测范围内时，才截短那一侧；同时超出两轴的回波可以从任一方向避开，不截。本箱掩码、6 px 膨胀带、飞点、无标签回波都不参与截短；取第 10 近的回波减去边距（少数误标像素只会让包络偏大）；永远不截进实测箱体 |
| `resting_contact_policy={mode: GENERAL_RESTING_CONTACT}` | 放置接触：支撑面只由上箱自己决定（可见面底边界，或接缝线减不确定度）；去掉 15 mm 高差、唯一下层、场景里只能有一个已测箱等条件，下层可见顶只做"高于上箱"的冲突检查。上箱足迹下有同帧交界回波的每个未识别实例都是下层；每个下层的完整可能体（工作区内、支撑面以下、其正面以内）替代它的邻箱阴影，作为该上箱的支撑（只在脱离段允许载荷接触，穿入不超过 0.4 mm，同原规则）。下层只看到顶面时，朝相机一侧用它最靠前的实测回波沿上箱正面方向封口（回放 8af1e4ef：不封口的体积碰到 base_link）。压在另一个已测箱下面的已测箱先降为下层（`RESTING_UNDER_RESOLVED_UPPER`，本轮不能先抓） |
| `own_mask_claim_policy={mode: OWN_MASK_FIRST_BRIDGE_NO_RETURN}` | 配合 (4) 和胶带：已测箱自己的掩码优先于与之重叠的未识别实例掩码（包含它的重复父实例、嵌在里面的胶带/标签小实例）；自身掩码内的无回波像素（胶带、斜看的顶面）不再切断自身回波的连通（包络只会长大，深度跳变仍然隔开）；正面平面带上、掩码外 14 px 内（11-20 确认的轮廓膨胀带）的回波归本箱，并在包络、截短和认领三处一致；同一箱子被测两次时（父掩码差集拟合失败后用原掩码，与子实例实测箱体每个角都在 10 mm 内）只保留子实例，父实例剩余像素照样作为场景证据。其他已测箱的像素一律不动 |

**离线回放**（保存的现场记录，现场变体配置 + 上述开关，`install/moveit-dev5`，逐帧单进程；每帧都用最终代码重跑）

| 帧 | 现场（11-23） | 回放 |
|---|---|---|
| 7d34bd14 | 叠放高差 17 mm 判不过，邻箱阴影贴包络 | **完整计划**，候选 11 |
| 8af1e4ef | 同一箱子重复检测，包络互相重叠 124 mm | **完整计划**，候选 6 |
| 90c7aaed | 共面下箱，RGB 边线 55% < 70%，整箱拒绝 | **完整计划**，候选 28 |
| b7037b72 | 立体伪影挡住全部端点 | 视觉通过；吸附力 23 / 端点被挡 9（规划层面） |
| 8d733bfc | 顺丰箱背面缺边，包络越出工作区 113 mm | 视觉通过；吸附力 27、接近段与端点（规划层面） |
| c07b50b5 | 侧面看不到 | 识别出目标；邻箱阴影与包络间隙 0（仍被拦） |
| ede9b41e | 共面白箱，边线 66% | 识别出目标；下箱顶面边角与包络重叠 3.8 mm（仍被拦） |
| cbfee7bc / 564cfcde | 检测器把上下两箱合成一个实例 | 同上（检测问题，本轮未处理） |

能识别出目标的帧 4 → 7，完整计划 0 → 3。

**未做 / 未验证**
- 未识别邻箱的阴影与目标包络侧向相贴（c07b50b5）、下箱顶面略高于上箱底面的边角（ede9b41e）还会挡住；检测器合并实例（cbfee7bc、564cfcde）未处理。
- 侧向相贴的邻箱（两箱并排紧靠）没有接触许可，本轮没改。
- 现场变体配置未上线（现场界面当前未运行），现场实机未做；新提交未推送。
- 只在这 9 帧上回放；更早的现场记录没有重放。

**验证**：新增 `tests/test_single_carton_vision_policies.py`（13 项：开关默认关闭与配置校验、只见正面的厚度封顶、共面延续、单轴截短与不截的情形、稳健秩、重复检测去重、已测下层降级、通用放置接触无需高差限制、只见顶面的下层封口）；视觉与规划的定向测试 459 项全部通过。证据：`docs/evidence/20261124_vision_policies/`（`report.json` 逐帧现场结果与回放结果、`field-config-vision.json`）；原始回放在 `outputs/physical-test-20261123/vision-replays/`（不入库）。无设备连接、无运动、无阀门命令，物理验收 **NOT_RUN**。

## 2026-11-24 安全修正：顶面空域射线在掠射角下把背面截短

用户量得现场棕色箱真实尺寸 174 × 133 × 110 mm（正面 174 × 110，厚 133）。对照今天识别出该箱的 6 帧：4 帧（8af1e4ef、90c7aaed、ede9b41e、现场 49c7361d）的保守包络背面比真实背面浅 5–23 mm，即真箱子不在包络内；背面全部由 `SENSOR_FREE_RECTANGLE_SPAN_BOUND` 的顶面空域射线决定。这是 11-19 起就有的规则（8af1e4ef 现场原始记录已是 131 mm），不是本轮开关引入的。原因：相机只比箱顶高约 6 cm，视线与顶面夹角 7–10°；拟合顶面只要比后段真实顶面高几毫米（下凹、鼓起、胶带），越过顶面的视线就会看到后方背景，被当成"箱子到此为止"，误差放大约 1/tan(8.5°) ≈ 7 倍。顺带：顶面测得的厚度只有 96–113 mm（后半段胶带无有效深度）；正面宽 168–171 mm（真实 174），高 107–111 mm（真实 110）。

修正（默认生效，不是开关，只会让包络变大）：`plane_free_witnesses` 在面平面向内移 `FREE_PLANE_INWARD_M = 5 mm`（2 mm 平面残差 + 3 mm 深度噪声余量）处判定视线是否为空。扫描 0/3/4/5/6 mm：3 mm 时 90c7aaed 仍差约 0.5 mm，4 mm 最紧余量 7 mm，5 mm 全部包住、最紧余量 14 mm，6 mm 时 4 帧退回 0.20 m 最大外形。`test_thickness_back_edge_bounded_by_free_rays` 的紧度要求由 +25 mm 放宽到 +60 mm，并新增真值包含断言。

未做：在线界面需重启才生效；修正后的包络变大，会让吸附力和碰撞判定更严，计划成功率需重新回放；相机俯视角是根本原因，安装是否改动由用户决定（11-19 决定安装不变）。证据：`docs/evidence/20261124_free_ray_plane_band/`（`report.json`、`sweep.txt`）。物理验收 **NOT_RUN**。

## 2026-11-24 放置接触：下层顶面伸到上箱旁边的部分算支撑面（用户决定）

现场 49c7361d：上箱压在比它宽的白箱上，白箱顶面伸到上箱右侧，比支撑面高 2–6 mm（支撑面取上箱可见底边再向下一个边距），离上箱实测侧边约 3.4 mm，载荷包络右下角与之重叠 3.6 mm。截短包络不可行（要把它们分开需 ≥ 9.0 mm：3.6 mm 真箱余量 + 3 mm 点云外扩 + 2.4 mm 间隙）。用户决定：这部分是支撑面。

开关 `resting_contact_policy.lower_top_beside='SUPPORT_SURFACE'`（需 `lower_top_beside_basis`，默认无）：该支撑下层自己的回波中，位于上箱实测正面平面之后、离上箱实测足迹 60 mm 以内、高度在支撑面以下 15 mm 到上箱实测底面 + 4 mm（原有交界带）之间的，做成 10 mm 支撑片，顶面正好在支撑面上（只有载荷在脱离段按支撑规则接触，机器人和工具照常避让）。表示高度最多降低交界带宽度（49c7361d 5.6 mm、ede9b41e 6.9 mm），逐次记录 `beside_support_pieces`。其他实例的回波、正面之前的回波不动。

回放（含 93ba224 安全修正，`single_carton.observe_release_vision6_20261124.json`）：ede9b41e **完整计划**（候选 25）；49c7361d 原位检查通过，候选搜索中被吸附力（23）和接近终态与实测障碍间隙 2.24 mm（要求 2.4 mm）挡住。新增单元测试 1 项。物理验收 **NOT_RUN**。

## 2026-11-24 相邻接触：紧挨的邻箱侧面是接触平面（用户决定）

开关 `geometry.adjacent_contact_policy={mode: LATERAL_CONTACT_PLANES, max_gap_m: 0.010, basis}`（默认无；`max_gap_m` 0.002–0.02），新模块 `adjacent_contact.py`：

- 只看目标实测箱体左右两侧（正面水平轴两端），不设背面：实测厚度是下界（今天 96–113 mm 对真实 133 mm），实测背面之后没有确定无疑的位置。
- 邻物证据只取其他检测实例的回波（不含本箱掩码、6 px 膨胀带、自身掩码策略的 14 px 正面带、无标签回波），超出实测侧面不超过 10 mm，且在另两轴上落在实测箱体本身之内；至少 10 个点时，在第 10 近的点处立竖直接触平面 Q（法向指向目标）。目标包络这一侧截到 Q（不进实测箱体）。
- 场景里离 Q 不到 3.4 mm（2.4 mm 间隙 + 1 mm）、并与目标包络另两轴范围重叠的障碍（观测体素、邻箱阴影等；支撑、工作区边界、固定物除外）被切到 Q 的邻物一侧，或延伸到正好贴住 Q，变成 `adjacent_contact` 对象绑定该目标与平面。
- 原生 worker：支撑新增 `kind: ADJACENT_CONTACT`，法向必须水平（与竖直夹角 ≥ 80°），对象类别必须是 `adjacent_contact`，其余与支撑相同（对象与平面精确绑定、只在带载脱离段允许载荷接触、载荷任一角越过平面不超过 0.4 mm，机器人和工具照常 2.4 mm）；能力标志 `adjacent_contact_plane_v1`，Python 启用开关时缺能力即拒绝。只装进开发版 `install/moveit-dev6`，正式 `install/moveit` 未动。

同时修正一处偏不保守的选点条件（影响 f83c65d 的 `non_interpenetration`）：判定邻物回波"在目标确定存在的范围内"时原用实测箱体加边距，边距带里的点可能在箱子下面或旁边；现在只用实测箱体本身。只会少截、包络只会变大。

验证：`m710_clearance_test optimized/reference` 全部通过（新增 `adjacent_contact_plane_and_04mm_guard`）；`tools/check_eco65_single_carton_native.py`（dev6）通过；新增 Python 单元测试 3 项；定向测试 434 项通过。回放（dev6，`single_carton.observe_release_vision7_20261124.json`，含 93ba224 安全修正与支撑面规则）：完整计划 7d34bd14（候选 11）、90c7aaed（28）、ede9b41e（25）；识别出该箱的帧包络全部包住真实 133 mm 深度。这些帧里没有真正左右紧挨的邻箱：49c7361d、ede9b41e 各立了一个平面（距实测侧面 7.2 / 3.6 mm），没有需要切的障碍；c07b50b5 挡住它的是右侧大箱沿视线外推 0.2 m 的假设体积，压在目标包络右上后角，不属于相邻接触。

未做：两个都已识别的箱子并排紧挨时（双方都是目标候选），对方包络由规划器加入、不在场景对象里，本规则暂不处理；现场界面需重启并改用 dev6 才生效；没有真实并排紧挨的现场样本。证据：`docs/evidence/20261124_adjacent_contact/`。物理验收 **NOT_RUN**。

## 2026-11-24 纸箱轮廓边缘的混合像素（用户决定）

现场 9707b9cc：纸箱直接放在大扁箱上（扁箱只见顶面）。先修正通用放置接触：只见顶面的下层就是支撑面本身，不再套用"下层正面须伸到支撑面以下"的冲突检查（8bedc99）。之后剩下约 49 个点悬在纸箱右下边外 10–19 mm、比底面高 4–9 mm，RGB 里那里只有平整的箱面——D435 在深度边缘把前后两面混成中间值的像素，斜坡每步不到 12 mm，旧的飞点过滤漏掉了。

开关 `geometry.edge_mixed_pixel_policy={mode: REMOVE_INTERPOLATED_SILHOUETTE_RETURNS, basis}`（默认无，新模块 `edge_mixed_pixels.py`）：同时满足才移除——在目标掩码外 14 px 内；在目标实测正面之后超过 4 mm；不在目标任一拟合平面、也不在所属实例的观测面上（各 4 mm）；深度严格夹在窗口（15×15）内最近的目标回波与最远的非目标回波之间（各留 4 mm）；不高于目标实测顶面；离目标实测箱体不超过 30 mm（第一版没有这条，回放 9707b9cc 误删 731 点、包括 2.9 m 外的桌面，加上后只剩边缘 84 点）。逐目标记录数量、图像与空间范围。

同时修正相邻接触延伸出的凸网格含非凸包顶点、原生拒收的问题（只保留凸包顶点）。

回放（dev6，`single_carton.observe_release_vision8_20261124.json`）：9707b9cc 原位检查通过，进入候选搜索，被吸附力（23）和工具下沿与下方扁箱间隙 2.31 mm（要求 2.4 mm，纸箱直接放在扁箱上、工具下沿低于纸箱底边，属实际几何限制）挡住；其他 6 帧结果不变（完整计划仍为 7d34bd14、90c7aaed、ede9b41e）。新增单元测试 1 项。物理验收 **NOT_RUN**。

## 2026-11-24 吸附力判据先屏蔽；吸盘唇边可碰邻箱（用户决定）

- `holding_capacity.check_mode='RECORD_ONLY_DISABLED_BY_USER'`（需 `check_mode_basis`，默认 `ENFORCED`）：吸附力仍逐候选、逐发送状态计算并记录（含候选排序用的裕度），但不再否决候选或计划，记录里标 `check_mode`。
- `neighbor_lip_contact={mode: CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS, basis}`（默认无）：所有连接吸盘的柔性唇边可接触邻箱对象——其他纸箱包络、支撑体与支撑片、相邻接触对象、未识别纸箱体积与邻箱阴影、离目标包络 10 cm 内的观测点；目标本身、固定物、工作区边界、实测支撑（`observed_support`）不在内；刚性芯、工具本体和机器人照常保持全部间隙规则。原生 worker 逐项核对对象存在、类别、目标不在其中、依据非空，允许对必须与请求逐一相同；能力 `neighbor_lip_contact_v1`，装入开发版 `install/moveit-dev7`（含 dev6 全部改动），正式 worker 未动。

回放（dev7，`single_carton.observe_release_vision9_20261124.json`，全部开关打开）：完整计划 6/9 帧——7d34bd14（候选 19）、8af1e4ef（6）、8d733bfc（15）、90c7aaed（28）、9707b9cc（30）、ede9b41e（25）；49c7361d、b7037b72 被工具刚性件与实测障碍挡住，c07b50b5 仍被右侧大箱假设体积挡住。原生测试新增 `neighbor_lip_contact_exact_pairs`，optimized/reference 通过；规划相关测试 144 项通过。屏蔽吸附力后计划不再保证吸得住，现场以真空压力门限为准。物理验收 **NOT_RUN**。

## 2026-11-24 软件调试模式：目标旁的点云障碍不参与规划（用户决定“做软件调试模式”）

- 目的：先跑通一次完整流程。今天现场的 3 次失败（9707b9cc、49c7361d、38a5ba54）都停在候选搜索之前，原因是目标保守包络与邻箱阴影、观测点簇或单个飞点体素重叠，或间隙不足 2.4 mm。
- `debug_near_target_obstacles={mode: REMOVE_RGBD_OBSTACLES_NEAR_TARGET_DEBUG, near_m, front_corridor_m, basis}`（默认无，行为不变；`near_m` 0–0.10 m，`front_corridor_m` 0–0.30 m，`basis` 必填）。规划前，从场景中移除点云派生对象（`observed`、`neighbor_shadow`、`adjacent_contact`、`unresolved_carton_volume`），条件是满足以下之一：
  - 与本次搜索目标的包络距离下界 ≤ `near_m`（分离轴投影，`NEAR_ENVELOPE`）；
  - 落在接近通道内：目标前面（包络 +Z），四边各外扩 `near_m`，沿法向向外延伸 `front_corridor_m`（`FRONT_CORRIDOR`）。
- 不移除：其他检出纸箱包络、实测/堆叠支撑及目标自身的脱离支撑、工作区边界、布局 CAD、固定几何、机器人和工具实体。留下的对象的全部间隙规则和阈值不变。PYTHON 与 NATIVE 两种规划流程都用过滤后的同一场景。
- 证据：每个任务写 `debug-near-target-obstacles.json`，逐个记录被移除对象的 id、类别、规则和距离下界，状态为 `USER_DEBUG_PREMISE_NOT_SENSOR_FREE_SPACE`。完整计划的 `cycle_binding` 中带有被移除对象的列表。`tools/replay_single_carton_record.py` 的默认搜索路径也走同一过滤。
- 现场配置：`outputs/workbench/single_carton.observe_release_vision9_debug_20261124.json`（vision9 全部开关，加 `near_m=0.01`、`front_corridor_m=0.25`；副本在 `docs/evidence/20261124_debug_near_target_obstacles/`）。

回放（dev7，`docs/evidence/20261124_debug_near_target_obstacles/replay-summary.json`）：
- 38a5ba54：仅 `near_m=10 mm` 就够，移除 3 段邻箱阴影后得到完整计划（候选 17）。
- 49c7361d、b7037b72：都被同一个孤立 4 cm 观测体素挡住，位于目标前面正前方约 0.17 m（世界坐标约 x=-0.38、y=-0.14~-0.16、z=0.35~0.39 m，两帧位置几乎相同，可能是飞点或悬垂的气管/线缆）。加上 0.25 m 前方通道后，各只移除这一个体素，分别得到完整计划（候选 0、14）。
- c07b50b5：原先的右侧大箱假设体积不再阻断，但候选随后被吸盘唇边与支撑接触、以及脱离/返回时 link_2/link_3 与立柱的规划失败挡住。这些属于运动几何限制，不是点云问题。该次回放最后因负载高、证据写出超时而结束。

风险：被移除的空间没有经过检查。若其中有真实物体（例如邻箱侧面、气管），工具可能碰到。执行时须低速运行，并有人手持急停在场。该模式只用于调试，不作为任何安全证明。物理验收 **NOT_RUN**。

## 2026-11-24 空载演练：完整轨迹、不吸附（用户决定）

- 用户原话：“需要不带吸附的空载运动”，并选定“完整轨迹，不吸附”（真空系统暂未打开）。
- `dry_run_no_vacuum={mode: FULL_TRAJECTORY_NO_VACUUM, basis}`（默认无，行为不变）。打开后，单箱循环按计划依次执行接近、接触、脱离、带载返回四段轨迹：
  - 不占用真空，因此任何阀门写入都不可能发生（写入必须先占用）；不读压力，不检查压力门限。
  - 接触后仍做实际接触 FK 检查，返回后仍做释放位姿检查。
  - 结束状态为 `COMPLETE`，结果标 `DRY_RUN_NO_VACUUM_TRAJECTORY_COMPLETE_NOT_PHYSICAL_ACCEPTANCE`，`physical_result` 保持 `NOT_COMPLETE`。
  - 界面不再要求“真空数据有效”，执行准备一栏前面加“空载演练”提示。机械臂使能、报警、负载核对、计划有效期、预览等其他执行条件不变。
- 箱子不会被带走。吸盘会实际压到箱面；脱离时吸盘唇边可能沿箱面滑动，箱子可能被轻微带动。带载段的轨迹按带箱包络审计过，空载工具占用的空间是它的子集。
- 现场配置：`single_carton.observe_release_vision9_debug_dryrun_20261124.json`，在调试模式配置基础上加本开关。测试：假 IO 下四段轨迹全部执行、没有真空占用、阀门指令和压力读取。物理验收 **NOT_RUN**。

## 2026-11-24 停止后复位回起点（用户决定“增加一个复位按钮”）

- 界面新增“↺ 复位回起点”。只有同时满足以下条件才可用：本任务循环停止已确认（`FAILED`/`CANCELLED`，不含 `STOP_UNCONFIRMED`）、没有可能持箱、机械臂已使能且无报警、处于静止、不在起点。
- 不是直接发回原位指令，分两步：
  1. 规划（`TaskManager.reset_plan` → `SinglePlanner.plan_reset`）：在本任务冻结的场景里（目标仍按规划时位置作为障碍，调试模式移除的对象同样不在），从实测关节规划一条新的空载路径回到本任务冻结的 `q_start`。先尝试直接回起点；如果起点与箱子相交或间隙不足（`INVALID_START`），先沿吸取面法向用直线退出该候选的预接触距离（接触阶段规则，与 withdraw 相同），再回起点。每段都做完整的发送轨迹碰撞审计和腕部检查，然后以当前绑定登记许可。
  2. 执行（`reset_execute`）：界面弹出确认框，写明分几段、预计时长、当前离起点的偏差，并说明使用的是冻结场景。确认后逐段提交，等实测到位再发下一段；任何一段停止或失败，后续段不再发送。停止按钮（Esc）照常有效。
- 记录：任务目录写 `reset-plan.json`（含各步耗时），任务日志写 `reset_plan`、`reset_motion`、`reset_complete` 或 `reset_failed`。复位不涉及真空。持箱时不允许复位，仍须现场处置。
- 重新“准备观测”会清除本次计划，之后不能再复位到本次起点。
- 离线验证（dev7，帧 38ab81b1，`--plan-cycle-entry --reset-check`）：
  - 现场界面空闲时：从接近段中途、接触位姿、脱离段中途三处出发，都得到复位计划。接触位姿那次是“退出 + 回起点”两段。
  - 现场界面同时在规划时（CPU 竞争）：等待规划进程就绪用了 9–50 s；接触位姿那次的直线退出在 Pilz 降速重试中请求超时（`MOVEIT2_REQUEST_TIMEOUT`），复位失败但没有发出任何运动。
- 假 IO 测试 7 项：两段依次执行到起点；停止确认前、持箱、已在起点时拒绝；某段被停止后不再发送后续段，复位计划只能用一次。物理验收 **NOT_RUN**。

## 2026-11-25 IK 绕圈孪生分支后移（用户决定“先去重分支”）

- 背景：现场任务 4ab96638、e526ba18、06da3c04、38ab81b1 的规划都用了 22–25 s。优先级前 6–7 个候选全部在 approach 发送轨迹审计中失败，最后都由候选 17 的第 4 个 IK 分支完成。每个候选只检查 4 个 IK 分支，但其中常有两个只差 J6 ±360° 的“绕圈孪生解”。它们是同一个手臂构型，却占了两个名额。
- `ik_winding_twins={mode, basis}`（默认无，行为不变）：
  - `KEEP_ORDER`：等同于不设置。
  - `AFTER_DISTINCT_CONFIGURATIONS`：在 `endpoint_ik` 原有排序（姿态偏好、连续性、腕部策略）之后，把各关节模 2π 相同（容差 1e-3 rad）的后续解移到全部不同构型之后。不删除任何解，同类中先出现的那个保持原位。
  - IK 记录写入 `winding_twins_after_distinct`，内容包括被后移的解、容差和策略。
  - 不改变终态检查、路径审计、阈值、候选生成或候选优先顺序。只影响前 4 个检查名额里有哪些分支，以及 approach 审计被拒后后续分支的尝试顺序。
  - 仅支持 PYTHON 规划流程，`planning_module=NATIVE` 时配置校验直接拒绝。
- 离线对比（`docs/evidence/20261125_ik_winding_twins/branch-order.json`）：只用已保存的 IK 记录重新排序，没有启动 worker，也没有重放规划。四个任务结果一致：
  - 候选 17 成功用的分支族（J4 94–109°、J5 +90°、J6 −90°）在优先级第一的候选 11 中原本排第 5，在名额之外；开关打开后进入第 4 位。
  - 候选 17/18/19 中，这一族由第 4 位提前到第 3 位。
  - 候选 14/12/10/15/13/16 不变。
- 未验证：候选 11 的这个分支能否通过终态和路径审计、规划时间实际降多少，都要等现场界面空闲后用 `--plan-cycle-entry` 重放确认。本次测量时现场界面正在规划，所以没有运行重放。
- 测试：新增 3 项（字面值重排、配置校验、`endpoint_ik` 开关关闭/KEEP_ORDER/打开三种情况）。`test_single_carton_planning_2s.py`、`test_eco65_posture_preference.py`、`test_single_carton_planner.py` 共 117 项通过。未运行 `m710_clearance_test` 和 `check_eco65_single_carton_native.py`，因为原生代码没有改动。物理验收 **NOT_RUN**。

## 2026-11-25 候选搜索策略六项（用户决定）

用户原话：“一是候选几乎重复，二是IK 分支名额被浪费，三是预检只查终态，不查路径，四是要跨轮继承，五是偏好姿态是‘大臂竖直、小臂水平、j4→J5沿-Z’这三条（近似即可，不要写死成固定的值），六是取消‘不翻腕优先’，腕部可自由旋转。把这6条都改过来”。

背景：现场任务 4ab96638、e526ba18、06da3c04、38ab81b1 的规划都用了 22–25 s。每次都是候选 11、14、12、10、15、13、16 依次在 approach 发送轨迹审计中失败，最后由候选 17 的第 4 个 IK 分支（J4≈99°、J5=+91°，与 q_start 同侧）完成。失败的分支都是手腕朝下的翻腕构型，Pilz PTP 的关节直线会让工具扫过 `link_3`，或让刚性吸盘芯扫进目标箱。

下面每一项都是可选开关，默认不设，行为不变；都只支持 PYTHON 规划流程，`planning_module=NATIVE` 时配置校验直接拒绝。所有开关都不改动任何检查、阈值、碰撞规则或运动限值。最终计划仍须通过全部规划检查和必做的发送轨迹审计。实现在 `single_carton/candidate_strategy.py`、`posture.py` 和 `planner.py`。

1. `candidate_family_order={mode: INTERLEAVE_LIFT_SIBLINGS, basis}`：由同一个基准吸取位姿（`contact_search.reference_tcp_pose`）派生、只差上移量（每档 1 cm）的候选算作同一家族。在“终态结论、接近直线结论、吸持档位”三者相同的同一类里，先试每个家族的第一个，再试第二个，依此类推。不删除任何候选。`candidate-priority.json` 记录 `family` 和 `family_rank`。
2. `ik_winding_twins`：见上一节。现场变体设为 `AFTER_DISTINCT_CONFIGURATIONS`。
3. `screen_approach_path={mode: APPROACH_JOINT_LINE, basis}`：预检时，某个分支的终态（接近、接触、带载脱离）通过后，再检查 q_start 到该接近构型的关节直线。这正是 Pilz PTP 的几何路径。检查使用 approach 阶段的场景和策略，用原生 `validate` 的 `probe_path=[q_start, q_approach]`；原生检查器按 `edge_resolution_rad` 在线上插样，审计方式（ADAPTIVE 等）与最终审计相同。
   - 直线通过的分支才被选中。直线被挡的分支记为 `APPROACH_JOINT_LINE_REJECTED`，继续检查下一个分支。
   - 如果所有分支的直线都被挡，就按原来的方式返回第一个终态通过的分支，标为 `approach_path_screen=FAIL`。候选不会因此被拒，仍可走 OMPL 回退等原有流程。
   - 候选排序：CLEAR 候选中，直线通过的排在前面，然后才看吸持档位和姿态。
   - 预检记录的 `approach_line` 条目记录结论、碰撞对、审计方式和耗时。
   - 带载返回段的起点（脱离终点）在预检时还不知道，所以不在这项检查范围内。
4. `cross_cycle_inheritance={mode: IK_SEEDS | IK_SEEDS_AND_FAMILY_FIRST, basis, max_entries=3, family_tolerance_rad=0.5, max_q_start_deviation_deg=1, store}`：本项取代 11 月 16 日第 6 节和 11 月 19 日中“IK 种子不跨轮继承”的规定，仅限 IK 种子和分支顺序；观测、场景、计划和路径仍然不跨轮。
   - 每次 PLAN_READY 时，把接近、接触构型、候选编号、吸盘组合、上移量、q_start 和循环结构，原子写入任务目录同级的 `cross-cycle-inheritance.json`（最多保留 20 条）。
   - 新任务只读取 q_start 偏差不超过 1° 且循环结构相同的最近 `max_entries` 条：
     - `IK_SEEDS`：这些接近构型作为额外的 IK 种子，排在 q_start 之后。
     - `IK_SEEDS_AND_FAMILY_FIRST`：此外，与继承构型在每个关节上（按 2π 折算）相差都不超过容差的接近分支排在最前，优先于姿态偏好。
   - 读取结果写入任务目录的 `cross-cycle-inheritance-input.json` 和计划记录的 `cross_cycle_inheritance`。
   - 每个解都在当前任务重新做全部检查。
5. `posture_preference.tolerance_deg={upper_up, forearm_horizontal, wrist_down}`（每项 0–60°）：大臂偏离竖直、小臂偏离水平、J4→J5 偏离 −Z 的角度在容差内不计代价，超出的部分按原来的形式计分。不设时分数与原来完全相同；容差为 0 时数值等价。不涉及任何固定关节值。
6. 不实现“不翻腕优先”：分支和候选排序中没有 J5 符号或翻腕判据。`wrist_branch_limits` 保持不保持 J5 符号、J4/J6 行程不限、只保留官方关节范围（2026-11-18/21 已如此）。现场变体只在 basis 里记录本次确认。

现场变体：`outputs/workbench/single_carton.observe_release_candidate_strategy_20261125.json`。它在 `single_carton.observe_release_vision9_debug_dryrun_20261124.json` 的基础上打开 1–4 项：
- 第 4 项设为 `IK_SEEDS`，让姿态偏好（第 5 项）继续决定分支顺序；`IK_SEEDS_AND_FAMILY_FIRST` 可以改选。
- 姿态容差暂取 20°/20°/20°，是实现者的取值，待用户确认。

副本在 `docs/evidence/20261125_candidate_strategy/field-config.json`。原现场配置文件没有改动。物理验收 **NOT_RUN**。

### 补充：唯一吸取位姿（用户决定 2026-11-25）

用户原话：“吸盘就只要一种选项（对准箱子待吸面的中心，吸箱子的时候吸盘长边平行于Y轴，吸盘短边平行于Z轴）就行，二是取消‘32 个候选就是吸盘组合（6 盘、3 盘、2 盘）乘以上移量 0–0.06 m（每档 1 cm）’”。

- 新增 `contact_search.single_contact={mode: FACE_CENTER_LONG_SIDE_ALONG_BASE_Y, long_side_toward: +Y|-Y, basis, max_normal_from_x_deg=45}`（默认无，行为不变；仅 PYTHON 规划流程，NATIVE 时配置校验拒绝）。设置后只生成一个候选，不再生成吸盘偏移组合和上移档位：
  - 待吸面：法向最接近基座 ±X 的可见面，即由 Y、Z 张成的面。夹角超过 `max_normal_from_x_deg` 时返回 `SINGLE_CONTACT_NO_FACE_SPANNING_BASE_Y_Z`。
  - 位置：12 个连通吸盘唇边轮廓的中心对准该面中心。
  - 朝向：阵列长边（4 个吸盘方向，唇边轮廓 0.176 m）沿基座 Y 在面内的投影；短边（3 个吸盘方向，0.131 m）随之沿 Z 的投影。吸取方向指向面内。
  - 正反两种朝向（绕法向差 180°）由必填项 `long_side_toward` 指定，不做猜测。
  - 密封、接触、唇边检查全部不变。能完整落在可见面上的吸盘才算密封，其余吸盘按被动自闭阀处理。一个都吸不上时报 `CONNECTED_CUP_SEALS_DO_NOT_FIT_VISIBLE_FACE`，不再改用其他位置。
  - 候选记录的 `contact_search.single_contact` 记下长边、短边的世界方向，以及它们与 Y、Z 的夹角。
- 与以前的差别：以前的候选（横滚 90° 加 J6 转 180°）实际是长边竖直（TCP x≈+Z）、短边沿 Y。本项把吸盘绕吸取法向转了 90°。
- 现场记录 4ab96638 的目标面实测 0.168 m（Y）× 0.110 m（Z），比阵列唇边轮廓 0.176 × 0.131 m 还小。面中心、长边沿 Y 时只有中间 2 个吸盘（part_010、part_016）完整落在面上。以前 32 个候选里最多有 6 个吸盘可以吸上。现场配置的吸持能力检查是 `RECORD_ONLY_DISABLED_BY_USER`，所以不会因此拒绝；若改回 `ENFORCED`，这个候选可能因吸持余量不足被拒。
- 现场变体取 `long_side_toward=+Y`（实现者的取值，待用户确认）。理由：在成功过的 J6≈−90° 那一族分支里，长边朝 +Y 对应 J6≈0°，与 q_start 的 J6=−8.9° 最接近。`lift_offsets_m=[0]`，`max_candidates=1`。前面第 1 项（近重复交错）在只有一个候选时不起作用。

### 第一轮重放（6 项开关，尚未加唯一吸取位姿）

代码 39124bd，worker dev7，`--plan-cycle-entry --frozen-scene`。重放时现场界面已关闭，现场任务目录 5 分钟内无写入，1 分钟负载 < 2。

| 记录 | 现场配置（开关关） | 6 项开关打开 |
|---|---|---|
| 4ab96638 | 24.22 s，候选 17 | 7.19 s，候选 11 |
| 06da3c04 | 24.16 s，候选 17 | 7.25 s，候选 11 |

日志在 `outputs/candidate-strategy-20261125/`（本地，不提交）。

### 第二轮重放（6 项开关加唯一吸取位姿）

代码 3702a25，条件同第一轮。两条记录都在 0.4–0.5 s 内结束，没有得到计划（4ab96638、06da3c04 各重放一次，再连续重放一次以检查跨轮继承）。原因相同：

- 终态预检中，接触终态的 `part_006_lip` 与目标箱下方支撑体 `rgbd_support_…` 相交（`ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED`）。
- 吸盘阵列短边（唇边轮廓 0.131 m）比待吸面高度（0.110 m）大，面中心对准时上下各伸出约 10 mm。下排唇边因此压进了下面那只箱子。
- 现有规则只允许唇边接触目标面和邻箱，不允许接触下方支撑。以前的上移档位正是用来避开这一点的。

是否允许下排唇边接触下方支撑，或改用其他位置，由用户决定；本轮未做改动。没有计划就没有执行，不涉及任何设备动作。定向测试 213 项通过。物理验收 **NOT_RUN**。

## 2026-11-25 唇边可碰下方支撑（用户决定"允许唇边碰下方箱"）

- 新增可选项 `neighbor_lip_contact.observed_support_basis`（非空依据；只在 `mode=CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS` 下可用，默认无）：目标下方的实测支撑（`observed_support`，即 `rgbd_support_<目标>`）加入"连接吸盘柔性唇边可接触"的对象名单。刚性芯、工具本体和机器人的全部间隙规则不变；其他碰撞几何、阈值、压力门限不变。
- 原生 worker 只有在该依据非空时才接受 `observed_support` 类别（能力 `neighbor_lip_contact_v2`），旧 worker 由 Python 流程拒绝；开发版装入 `install/moveit-dev8`（含 dev7 全部改动），正式与在用安装未动。`m710_clearance_test optimized/reference` 新增用例 `neighbor_lip_contact_observed_support_basis` 通过，`tools/check_eco65_single_carton_native.py`（dev8）通过，定向 pytest 128 项通过。
- 现场空闲时离线重放（唯一吸取位姿 + 前述六项开关，代码 de4b121，证据 `docs/evidence/20261125_lip_support/replay.json`）：
  - 4ab96638：PLAN_READY，入口到就绪 3.1 s（之前 0.4 s 无计划；原配置 24.2 s）。但只有 part_010、part_016 两个吸盘吸上，吸持余量 0.44（需 ≥ 1，切向受限），因现场变体为 RECORD_ONLY 才未被拒绝。
  - 06da3c04：无计划（3.2 s 返回）。同一族手腕分支的 Pilz PTP 发送路径上 link_3 与工具 part_026 距离 4.999 mm，比 5 mm 工具/机器人间隙少 1 µm；推迟 Pilz 解检查（决定 B）后发送审计失败只换下一个 IK 分支，不再对同一分支做 OMPL 避让，其余分支终态被挡。
  - 跨轮继承链式两条结果相同。
- 未运行：物理验收 NOT_RUN；现场配置未替换。

## 2026-11-25 Pilz 直连被发送审计拒绝后同一分支 OMPL 回退（用户决定"加 OMPL 回退"）

- 新增可选开关 `pilz_sent_path_ompl_fallback`（布尔，缺省 false；需要 `defer_pilz_solution_check=true`；仅 PYTHON 流程）。决定 B 之前，Pilz PTP 路点未通过 worker 检查就会转同一目标的 OMPL；推迟到发送审计后，审计拒绝只换下一个 IK 分支。打开后，approach 的 Pilz PTP 发送路径被审计拒绝时，先在同一 IK 分支用原有 OMPL 回退（同一请求、预算 `native_ompl_budget_s`、统一截止时间与决定 E 规则、OMPL 自身路点检查）重新规划，再照常做完整发送审计；仍失败才轮到下一个分支，下一分支仍先用 Pilz。记录名 `approach-after-pilz-audit-ompl`，计划证据中标 `pilz_sent_path_ompl_fallback` 与 `ik_branch.approach_planner`。任何检查、阈值、碰撞规则不变。
- 离线重放（代码 69cbf8e，dev8，证据 `docs/evidence/20261125_pilz_audit_ompl/replay.json`）：4ab96638 不受影响（PLAN_READY 3.2 s，Pilz 直接通过）；06da3c04 的回退确实启动，但 OMPL RRTConnect 在 5 s 预算内未找到路径（NATIVE_SOLVER_BUDGET_EXHAUSTED），仍无计划，返回时间 3.2 s → 8.8 s。链式继承两条结果相同。定向 pytest（候选策略、2 s 规划、单箱规划三个文件）131 项通过；本次只改 Python，原生沿用 dev8。
- 未运行：物理验收 NOT_RUN；现场配置未替换。

## 2026-11-25 现场变体配置全部优化开关打开（用户决定"把优化开关全都打开"）

- `outputs/workbench/single_carton.observe_release_candidate_strategy_20261125.json`（副本 `docs/evidence/20261125_candidate_strategy/field-config.json`）：此前已打开的全部规划提速开关保持不变，另把 `cross_cycle_inheritance` 从 IK_SEEDS 改为 IK_SEEDS_AND_FAMILY_FIRST；`pilz_sent_path_ompl_fallback`、`neighbor_lip_contact.observed_support_basis`、`contact_search.single_contact` 均已打开。`prepare_processes=4` 与 `prepare_implementation=NATIVE_LIBRARY` 互斥，保留前者。配置校验无错误。需原生 `install/moveit-dev8`。
- 现场启动脚本仍用原现场配置，未替换；`dry_run_no_vacuum` 与吸持检查"只记录"保持原样。物理验收 NOT_RUN。

## 2026-11-25 吸盘整体柔性、闲置吸盘避开邻箱、下两排吸盘吸、相机俯仰座转向（用户决定）

用户原话（看完三维查看器中的 #7、#28 后）：“一是#7暴露出的第一个显著的问题是，吸盘是柔性的，可以允许它触碰下一箱……二是……现在各个吸盘只有统一控制来一起吸，没有单独控制。所以在规划时，应考虑要吸盘尽量不附着到邻箱上。对于#7路径，应该将吸具沿着+Z上移，用下两排吸盘来吸，最上面一排吸盘空着（吸盘有止回阀，可以对着没有邻箱的地方），三是#28，在layout中将视觉桅杆的底座在现有基础上绕Z轴逆时针旋转90°，然后将相机沿着+z逆时针旋转180°”；补充：“吸盘从唇边到最后到吸具的连接器，都是柔性对，刚性芯是不对的”；“现场用的标定文件先不变，改布局是为了碰撞检测”。看过按“底座连同桅杆绕 J1 转 90°、相机转 180°”生成的布局后，用户更正：“我说的相机底座是指竖直的那根杆，与J1相连的水平连接件保持现有状态不变”，“竖杆和横杆都不转，俯仰座绕Z轴顺时针旋转90°”，“竖杆不转”。第 4 项按更正后的说法实现，前一个布局已撤销，不提交。

全部是可选项，默认不设，行为不变；仅 PYTHON 规划流程（NATIVE 时配置校验拒绝）。阈值、间隙、压力门限、运动限值都不变。

1. `neighbor_lip_contact.cup_body_basis`（非空依据，仅在 `mode=CUP_LIPS_MAY_TOUCH_NEIGHBOUR_CARTONS` 下可用）：每个连接吸盘的吸盘体（同一 CAD 吸盘零件除唇边带以外的那块碰撞体，即原来的 `part_XXX_rigid`）与唇边一样可接触邻箱对象（同一名单：其他纸箱、支撑体与支撑片、相邻接触、未识别纸箱体积与阴影、目标 10 cm 内的观测点，以及 `observed_support_basis` 时的下方实测支撑）。吸盘上的螺母（连接器，`part_003` 等奇数件）、安装板、工具其余部分和机器人照常保持全部间隙规则；对目标本身的接触规则不变。映射 `cup_body_by_contour` 由工具 CAD 绑定（`passive_cups.cup_bodies`：零件、唇边带、工具连杆逐一核对），进入过程几何。原生 worker 能力 `neighbor_lip_contact_v3`：按对象先全部唇边、再全部吸盘体，允许对必须与请求逐一相同；开发版 `install/moveit-dev9`（含 dev8 全部改动），正式与在用安装未动。`m710_clearance_test optimized/reference` 新增用例 `neighbor_lip_contact_cup_body_basis` 通过。
   - 注意：吸盘体实际可压缩行程未知（CAD 中 `real_compression_stroke_m` 为空）。许可后，吸盘伸进邻箱包络的深度只受螺母和安装板限制，最深约 5 cm，没有单独的压缩量上限。如需限制，请给出吸盘可压缩量。
2. `passive_cup_neighbour={mode: PREFER_PASSIVE_CUPS_OFF_NEIGHBOURS, basis, toward_tool_m=0.02, into_face_m=0.005}`：每个候选在接触位姿下，把各个被动（不密封）吸盘的唇边环及其中心沿吸取方向从“伸入面内 5 mm”扫到“朝工具 20 mm”，落入目标以外任一对象（盒体或凸包）即记为“对着邻箱”。在终态全部通过的候选中，对着邻箱的被动吸盘少的排在前面；只是偏好，不删候选、不改检查。记录在 `*-candidates.json` 的 `passive_cup_neighbour` 和 `candidate-priority.json` 的 `passive_cups_facing_neighbour`。20 mm / 5 mm 是实现者的取值，待用户确认。
3. `contact_search.single_contact.vertical_placement=LOWER_TWO_ROWS_ON_FACE`（需 `vertical_placement_basis`；缺省 `FACE_CENTER` 不变）：唯一吸取位姿沿面内向上（基座 +Z 在面内的投影）平移，使最下两排吸盘中心的中点落在面中心，最上一排空出。密封、接触、唇边检查不变，能完整落在面上的吸盘才算密封。候选记录 `vertical_placement` 和 `up_shift_in_face_m`（ECO65 吸具为半个排距 22.5 mm）。
4. 相机俯仰座转向（仅碰撞模型）：`configs/workcells/eco65_camera_pitch_mount_cw90_v1.yaml`（schema `eco65_pitch_mount_yaw_v1`，`yaw_deg: -90`），由 `camera_installation.apply_pitch_mount_yaw` 应用于现场碰撞快照：
   - 俯仰座（两侧立板、两个轴颈、托板）连同其上的相机绕竖杆轴线（+Z）顺时针（俯视）转 90°；相机相对俯仰座不另外转。
   - 竖杆（part_019，含杆顶卡槽）、与 J1 相连的水平连接件和安装垫都不动；只有 19 个相机碰撞体和 5 个俯仰座碰撞体位姿改变。
   - 结果：模型相机光心在 link_1 中由 (0.068, −0.037, 0.504) m 移到 (0.072, 0.032, 0.504) m，朝向由 link_1 −Y 变为 −X（手臂前方），下倾 30° 不变。
   - 生成工具 `tools/remount_eco65_field_mast.py`（也支持增量桅杆重装配置），产物 `outputs/eco65_native/camera_pitch_mount_cw90_20261125/snapshot.json`（本地，不提交，可由工具重现）。执行配置 `configs/workbench/controlled.camera_pitch_cw90_20261125.json` 只把 `snapshot` 指向新快照。
   - **现场视觉标定文件 `configs/workbench/reference/active_extrinsics.json` 未改**，感知仍用它（其相机在 link_1 中位于 (−0.022, 0.149, 0.291) m、朝 +Y 偏 8°、下倾 10°，与模型相机不同）。改动只影响碰撞模型和拍照时的自滤除，不影响箱子定位。
   - #28 中被撞的是竖杆本身（part_019），竖杆不动，这一项碰撞关系不变。

现场变体（本地，不提交）：`outputs/workbench/single_carton.pitch_cw90_cup_body_20261125.json`，在 `single_carton.observe_release_candidate_strategy_20261125.json`（全部优化开关、唯一吸取位姿、空载演练、调试模式）基础上加上第 1–4 项；`..._32cand_20261125.json` 在原现场空载演练配置（32 个候选）基础上加第 1、2、4 项。副本在 `docs/evidence/20261125_cup_body_pitch_mount/`。需原生 `install/moveit-dev9`（`M710_MOVEIT_INSTALL_PREFIX`）。

离线重放（`--frozen-scene --plan-cycle-entry`，dev9，证据 `docs/evidence/20261125_cup_body_pitch_mount/replay.json`）：

| 记录 | 起点 J1…J6（°） | 唯一位姿（下两排） |
|---|---|---|
| 38ab81b1（昨天） | 90.6, 8.6, −93.4, 78.3, 95.8, −8.9 | **PLAN_READY 15.8 s**，4 盘（010/012/016/018）密封，被动吸盘都不对着邻箱 |
| 649adfc8（今天 13:55） | 68.6, 1.4, −92.7, −85.7, −98.9, −4.3 | 终态全部通过，接触、脱离审计通过；接近 Pilz 直连 `part_018_lip`↔目标 2.3986 mm（要求 2.4 mm），同分支 OMPL 5 s 未找到；带载返回审计 载荷↔下方支撑 2.396 mm。无计划 43.9 s |
| 8ba19477（今天 13:51） | 同上 | 终态全部通过，接触、脱离审计通过；接近 Pilz 直连 link_2↔`part_008_lip` 4.995 mm（工具/机器人要求 5 mm），OMPL 5 s 未找到。无计划 23.6 s |

- 32 候选变体：649adfc8 无计划 234.5 s（接近段唇边↔目标 13、安装板↔下方支撑 5、link_3↔part_039 2；脱离后 link_3↔竖杆 7），8ba19477 无计划 208.2 s（接近段 link_3↔part_039 9、安装板↔下方支撑 8；脱离后 载荷↔观测点 6、link_2/link_3↔竖杆 7）。
- #7 一类“刚性芯压进下层箱”的终态拒绝已全部消失。
- 今天两条记录剩下的失败都发生在从今天的起点（J1 68.6°、J4/J5 与昨天正负相反，手腕在另一侧）出发或回到它的路径上；唯一位姿下的差距在微米级。从昨天的起点出发能规划成功。
- 未运行：物理验收 NOT_RUN；现场启动脚本和原现场配置未替换。

### 补充：水平连接件绕 J1 逆时针 90°（用户决定 2026-11-25）

用户原话（看过俯仰座转向后的三维查看器）：“箱位3的姿态是最好的，水平连接件 part_001要在现有基础上绕Z轴逆时针旋转90°，竖杆、俯仰座和相机都保持现有状态不变”。

- `configs/workcells/eco65_mast_bar_ccw90_v1.yaml`（schema `eco65_mast_bar_yaw_v1`，`yaw_deg: 90`，`pivot: J1_AXIS`），由 `layout_installation.apply_bar_yaw` 应用于上面的俯仰座转向快照：只有 part_001 的 145 个凸块绕 J1 轴逆时针（俯视）转 90°，它与 link_1 之间的安装垫按新位置由官方 link_1 表面重新生成（174 → 170 块，高度不变；托板最低处离 link_1 表面 3.4 mm，由安装垫填满）。竖杆（part_019）、下夹座（part_010）、俯仰座和相机位姿都不变。
- 结果：part_001 在 link_1 中由 x −0.041…0.087 m、y −0.041…0.032 m 变为 x −0.032…0.041 m、y −0.041…0.087 m（伸向 +Y）。模型中横杆与竖杆不再相连，这是按用户要求只改碰撞模型的结果。
- 产物 `outputs/eco65_native/mast_bar_ccw90_pitch_cw90_20261125/snapshot.json`（本地，不提交，`tools/remount_eco65_field_mast.py --source <俯仰座快照> --config configs/workcells/eco65_mast_bar_ccw90_v1.yaml` 重现）；执行配置 `configs/workbench/controlled.mast_bar_pitch_20261125.json`；现场变体 `single_carton.bar_pitch_cup_body_20261125.json`（唯一位姿下两排）与 `..._32cand_20261125.json`。现场视觉标定文件仍未改。证据 `docs/evidence/20261125_mast_bar_ccw90/`。
- 离线重放（dev9）：38ab81b1 **PLAN_READY 15.7 s**（脱离 OUTWARD 方案 link_3↔竖杆 2.393 mm 未过，改用 DIAGONAL_45 通过）；649adfc8、8ba19477 与上表相同，仍差微米级（唇边↔目标 2.3986 mm、link_2↔唇边 4.995 mm），无计划。物理验收 NOT_RUN。

### 补充：竖杆连同俯仰座和相机挪到横杆末端（用户决定 2026-11-25）

用户原话：“竖杆（及其上面的俯仰座和相机），挪到横杆远离J1的那一端，二者连起来”。

- `configs/workcells/eco65_mast_pole_to_bar_end_v1.yaml`（schema `eco65_mast_pole_to_bar_end_v1`），由 `layout_installation.apply_pole_to_bar_end` 应用于上面的横杆转向快照：下夹座 part_010 固定在横杆末端，随横杆绕 J1 逆时针转 90°；竖杆 part_019、俯仰座和相机整体平移到新的横杆末端，各自朝向不变（圆竖杆可在夹座中任意分度）。安装垫和横杆不变。
- 结果：竖杆轴线在 link_1 中由 (0.1045, −0.0041) m 移到 (0.0041, 0.1045) m，即横杆远离 J1 的一端；夹座 y 0.082…0.121 m 与横杆末端（y 至 0.087 m）相接。模型相机光心 (−0.029, 0.141, 0.504) m，仍朝 link_1 −X、下倾 30°。现场视觉标定文件仍未改。
- 产物 `outputs/eco65_native/mast_pole_at_bar_end_20261125/snapshot.json`（本地；资产清单同时引用俯仰座快照和横杆快照目录）；执行配置 `configs/workbench/controlled.mast_pole_at_bar_end_20261125.json`；现场变体 `single_carton.mast_final_cup_body_20261125.json`（唯一位姿下两排）与 `..._32cand_20261125.json`。证据 `docs/evidence/20261125_mast_pole_at_bar_end/`。
- 离线重放（dev9）：

| 记录 | 唯一位姿（下两排） | 32 候选 |
|---|---|---|
| 38ab81b1（箱位 3，昨天的起点） | 无计划 14.0 s：竖杆挪走后第 3 个逆解分支（J1 ≈ −147°）终态和接近直线都通过而被选中，但它的接触直线段（最后 6 cm）原生规划失败；规划器只在接近段失败时换下一分支，接触段失败直接判候选失败（第 4 分支在上一布局中成功） | **PLAN_READY 38.0 s**（候选 17） |
| 649adfc8 | 无计划：接近唇边↔目标 2.3986 mm；带载返回 载荷↔下方支撑 2.396 mm | 未跑 |
| 8ba19477 | 无计划：接近 link_3↔俯仰座立板 2.392 mm | 未跑 |

- 物理验收 NOT_RUN。是否让“接触段规划失败也换下一个逆解分支”，待用户决定。

### 补充：接触段失败也换下一个逆解分支（用户决定 2026-11-25，“加上换分支”）

- 新增可选开关 `contact_failure_next_branch={mode: NEXT_IK_BRANCH, basis}`（缺省或 `DISABLED` 行为不变；仅 PYTHON 规划流程，NATIVE 时配置校验拒绝）。原来只有接近段失败（规划失败，或推迟 Pilz 检查后的发送轨迹审计拒绝）才检查并改用下一个终态通过的逆解分支（2026-11-21 决定）；打开后，接触直线段（预接触点到接触点）规划失败或其发送轨迹审计被拒，也同样换下一个分支。各项检查不变；`ik_branch.earlier_failures` 记录失败阶段。实现在 `planner.contact_next_branch`、`plan_candidate_segments`、`replan_after`。
- 现场变体 `single_carton.mast_final_cup_body_20261125.json` 与 `..._32cand_20261125.json` 已打开（副本更新到 `docs/evidence/20261125_mast_pole_at_bar_end/`）。
- 离线重放（dev9，竖杆接到横杆末端的布局，证据 `docs/evidence/20261125_contact_next_branch/replay.json`）：
  - 38ab81b1（箱位 3）唯一位姿：**PLAN_READY 16.2 s**（第 3 分支接触段失败后换到第 4 分支；之前无计划）。
  - 38ab81b1（箱位 3）32 候选：PLAN_READY 37.2 s（候选 17，与不开时相同）。
  - 649adfc8、8ba19477：仍无计划。各分支接近段 Pilz 直连审计差微米级、同分支 OMPL 5 s 未找到，分支用完。
- 物理验收 NOT_RUN。

## 2026-11-25 优化开关改为代码缺省打开；相机下移；规划耗时核对（用户决定）

用户原话："一是除了相机沿着-Z向下移动了一段距离外，其余都按新模型改装了，所以标定文件保持不变。二是运动规划我记得改过一轮搜索策略优化，不用再搜32个点了。三是运动规划的耗时应该大幅提升了才对。四是初始位置应该是相机正对箱堆的位置，而不是箱位3，我只是说箱位3的接近构型是好的。五是所有的优化在代码里都把开关打开，而不是做到现场配置里。"；关于标定与模型相机朝向差 82°："就用现在的标定文件，先忽略"。

- **代码缺省**：新模块 `single_carton/optimization_defaults.py`，在 `config.load` 时给配置里没写的键补上"打开"：全部规划提速开关（含 `audit_chunking=WORK_QUEUE`，本次预热对比 2.41/2.41/2.86 s 对 SUBSETS 2.43/2.51/2.65 s，没有变慢）、候选搜索六项、`contact_failure_next_branch`、`pilz_sent_path_ompl_fallback`、`passive_cup_neighbour`、唯一吸取位姿（下两排）、姿态容差 20°、吸盘唇边/吸盘体/下方支撑接触、11-19 至 11-24 的全部视觉策略。配置文件里显式写的值（包括 `mode: DISABLED`、`false`）仍然优先；依赖的前提被显式关掉时不补；`planning_module=NATIVE` 不补。调试模式、空载演练、吸持力只记录、起点核对/执行前复核、执行锁、循环结构和现场参数（0.20 m 最大外形、42 N 等）不是优化，仍由现场配置决定。补上的键逐项记在 `code_defaults.applied`。
- 现场配置 `outputs/workbench/single_carton.field_camera_drop_20261125.json`（副本 `docs/evidence/20261125_code_defaults_camera_drop/field-config.json`）只保留现场项，删去 63 个优化键；加载后生效设置与原先全写满的变体逐项相同（只有依据文字不同）。
- **相机下移**（仅碰撞模型）：`configs/workcells/eco65_camera_drop_v1.yaml`，只有相机 19 个碰撞体沿 link_1 −Z 下移 0.21274 m（模型光心高 0.50372 m → 标定光心高 0.29098 m），俯仰座、竖杆、横杆、夹座不动；快照 `outputs/eco65_native/mast_camera_drop_20261125/`（本地），执行配置 `configs/workbench/controlled.mast_camera_drop_20261125.json`。标定文件未改；朝向差异按用户指示先不处理。
- **耗时**：上一节记录的 38ab81b1 "16.2 s"里约 12.8 s 是离线回放冷启动 13 个 MoveIt 进程。用 `--prewarm-workers`（与现场常驻进程相同）入口到 PLAN_READY 为 2.4–3.4 s；新模型 + 代码缺省为 3.43 s。
- **旧桅杆复核**：a08eec2 的 38ab81b1 计划放到旧快照（空场景）里，脱离段 link_3 与竖杆 2.390 mm（要求 2.4 mm），返回起点 link_3 与下夹座相交；新快照四段都通过。实物已按新模型改装，因此只作记录。
- **观测起点**：按标定外参，相机航向为 link_1 的 98.2°；箱堆中心方位约 168°，相机正对箱堆时 J1 ≈ 70°。今天的 649adfc8/8ba19477（J1 68.6°）正是这个位置，但手腕翻转（J4 −85.7°、J5 −98.9°）。新模型下两条都仍无计划。离线变体（649adfc8 的 RGB-D 与 J1，J2–J6 取 38ab81b1 起点、不翻腕）也无计划：今天的目标在另一侧（接近构型 J1 ≈ −47°，箱位 3 为 0.6°），各分支接近段末端吸盘擦过目标包络，差 0.002–0.016 mm（唇边/刚性芯对目标 2.384–2.398 mm），同分支 OMPL 5 s 未找到。
- 三维查看器：`outputs/viz/start_pose_camera_drop_20261125/`（`.claude/launch.json` 的 `start-pose-camera-drop-viewer`），含三个起点与各段路径。
- 证据：`docs/evidence/20261125_code_defaults_camera_drop/`（`replay.json`、`remount_report.json`、`field-config.json`）。测试：新增 `tests/test_single_carton_optimization_defaults.py`；原有用假 worker 固定旧缺省行为的测试改为 `load(..., code_defaults=False)`。物理验收 **NOT_RUN**。
