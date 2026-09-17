# ECO65-B 桌面样机第一轮正式适配

2026-09-17，本地分支 `feat/v0.6-eco65-desktop`。在准备轮提交 `14626db9b13f655de79c99a1d96bf98b03da22b9` 上继续，未重新初始化工程、未整体合并来源分支、未连接真机、未上传私有资产、未推送。

## 实际完成的结果

| 项目 | 结果与边界 |
|---|---|
| 真实 STEP | XCAF 成功解析，36 个有效实体、21 个装配节点、12 个吸附组件；原件 SHA 未改变 |
| 装配/TCP | 工作面由 CAD 推导；法兰至吸具的安装依赖明确的仿真转接架，尚未实测 |
| 官方机器人 | rm_models 的 **ECO65-B** 六轴 URDF；用户已确认型号，控制器代际仍未知 |
| 数值检查 | 独立 SciPy/XML FK、中心差分 Jacobian、SE(3)、限位、渲染/碰撞帧一致性通过 |
| 完整规划 | 真实 IK + RRT-Connect，6 个阶段；最终五次插值轨迹 **38,930 个状态**通过完整几何复核 |
| 执行链路 | known-pose → 世界快照 → PlannerBackend → PlanValidator → simulation ExecutionBackend，完成同一箱体的附着、支撑释放、撤离 |
| 回放 | **GEOMETRIC_REPLAY_ONLY**，123.329617 s；640×360、5 fps、正常播放速度 |
| Isaac | 本地 Python 未发现 isaacsim/isaaclab；同场景 USD 和轨迹成功导出、重新打开校验；动力学 **NOT_EVALUATED** |
| 真机 | 未连接、未下发；enable_hardware=true 或 hardware 模式直接拒绝，无厂家 SDK 执行提供器 |

这次成功数量为 **1 个完整几何抓放任务**。不是物理抓放成功数量，也不是实机负载、安装或安全资格。没有追加第二次搜索、整排任务或节拍优化。

## 原件、单位与实际几何

实际原件：`assets/tools/desktop_suction/cad/raw/KVGL180-130-260909-01.STEP`，1,746,336 bytes，SHA-256：

`376a6d9e814117a0ced147ca69b21d4a4fc4cb124b9abf8d12fd8aa2b9a57d78`

文件为 SolidWorks AP214 毫米装配，仅发现这一份 STEP。读取器使用毫米系统单位，之后显式乘 0.001 转米；局部/全局实例变换保存于私有清单，旋转不含缩放。只实例化装配叶节点，不再叠加整个装配 compound。全部 36 个实体拓扑有效；未发现外部引用或未处理零件，没有进行几何修复。保存源名称和颜色；部分旧编码中文名称读取后乱码，稳定 ID 与实例路径仍保留，不以猜测名称改变几何分类。

CAD 总包络约 **180 × 105.8004 × 132.7787 mm**。真实工作面是 **4×3 的 12 个吸附组件**，并有主体板、垫片、接头和突出附件。可视模型包含 666,212 个三角形；网格化线性偏差 0.1 mm、角度偏差 0.15 rad。没有用一个长方体替代吸具，也没有缩放装配来适配小箱。

碰撞表示逐零件生成凸体，12 个吸附组件进一步按实际 CAD 环形面切分末端裙边和保守刚性区域，共 48 个吸具碰撞区域；6 个转接架几何体另计。分区包含三角形跨切割面的交点。原网格顶点相对原凸体及最终分区联合覆盖检查通过，容差 1e-8 m；这是有限网格覆盖检查，不能证明原 CAD 未建模的软管或实物附件不存在。未知部分保守按刚性处理。

详细映射、局部/全局变换、逐零件尺寸、表面特征、网格和源指纹在：

- `assets/tools/desktop_suction/derived/manifests/cad.json`
- `assets/tools/desktop_suction/derived/visual/`、`derived/collision/`
- `configs/local/eco65_desktop/tool.json`
- `outputs/eco65_desktop_round1/round1/cad_inspection/cad.json`

## 安装、TCP 和仿真假设

官方法兰尺寸图为归档的 `ECO65_End_Adapter_Hole_Position_Diagram.pdf`：RMU01001 基础版，外径 57 mm、直径 49 mm 节圆上的 6×M4。CAD 安装板为 x=±82 mm、z=±22.5 mm 的四个直径 5 mm 孔，没有直接匹配的 49 mm 孔圈。

本轮选择 `named_four_leg_standoff_v1`：法兰盘、横梁和四根支腿，全部进入可视、碰撞及 USD 负载模型。它是**尚不存在实物证据的仿真转接架**。保留 yaw=180° 对比图，实际安装方向未确认；正式几何轨迹采用 yaw=0°。

统一 `T_A_B` 将 B 坐标表达转为 A 坐标表达：

- `T_flange_tool_cad` 旋转为 `[[1,0,0],[0,0,1],[0,-1,0]]`，平移 `[0,0,0.09] m`。
- `T_tool_cad_tcp` 原点在 CAD 自由唇口共同平面 y≈−0.056750434 m；TCP +Z 为 CAD −Y，+X 为 CAD +X，+Y 为 CAD +Z。
- 两者乘积 `T_flange_tcp` 旋转为单位阵，平移 `[0,0,0.146750434] m`。FK、IK、碰撞、附着与回放共用完整矩阵，无重复 tool_length 补偿。

TCP 是 **CAD 自由状态平面 + 假设安装**，不是实测标定 TCP。末端裙边依据 CAD 环形面切分；真实材质、压缩行程和刚度未知。没有可确认的 CAD 运动副，因此几何回放将支架及未知部件视为固定装配，不宣称实物绝无活动结构。

采用 ideal_suction，仅允许 12 条实际 CAD 轮廓全部在同一箱面内且贴合/法向满足几何条件后附着；不宣称独立气路、独立开关、负压或真实承载能力。工具 1.0 kg、转接架 0.25 kg、纸箱 0.30 kg 都是有限仿真假设。USD 中工具惯性是包络近似，转接架惯性是几何估计；官方机器人惯性及非对角项保持来源。几何体积中心不称为实测质心。

## 场景、碰撞与真实规划

世界 +X 进入模拟车厢、+Y 向左、+Z 向上。台架 1.2×0.9 m，桌面高 0.64 m；前开口，安装左板和后板，右侧及顶部开放。箱体依据吸附轮廓选择 **220×180×100 mm**，源支撑与接收台顶面均高 0.68 m；所有尺寸、位置和质量为本轮设计值，完整内容以 `scene_snapshot.json` 为准。

已知位置 fixture 明确为 SYNTHETIC；检测分数、相机标定均为空。没有调用视觉模型或伪造检测结果。合法 HOME 通过 IK 求得并碰撞检查，关节状态见 `reports/home.json`；全零只用于数值测试。

碰撞余量固定 **2 mm**，另加每体 0.2 mm 网格误差储备，一般碰撞对阈值 2.4 mm；没有因规划失败降低余量。使用官方机器人逐连杆凸包、全部刚性吸具附件、假设转接架、台架/墙板/支撑和同一被抓箱体。机器人非相邻连杆自碰撞与机器人/工具/环境碰撞均检查；相邻关节连杆及固定装配内部关系单独处理。

合法接触仅为明确对：base_link/台架、link_6/转接法兰盘、指定阶段且顶面法向匹配的末端裙边/目标箱、箱体底面/原支撑或接收台。未继承 FANUC J5/J6 豁免，未忽略整个工具或目标箱。其他手腕/转接架对仍检查。接触穿透容差 0.4 mm；附着保存实际 TCP→箱体变换，释放必须计算出接收台支撑，箱体身份始终 `carton_001`。

随机种子 6501；离线规划预算 1200 s，80 个额外 IK 初值/阶段、RRT 上限 8000 次/阶段，支持逻辑取消和进度日志。第一条完整路径即停止搜索：

| 阶段 | 路点数 | 插值时长 s | RRT 迭代 |
|---|---:|---:|---:|
| 空载接近 | 41 | 49.1523 | 75 |
| 低速接触 | 2 | 2.7112 | 直连 |
| 带载抬升 | 2 | 1.8705 | 直连 |
| 带载转运 | 49 | 59.6959 | 34 |
| 放置接触 | 2 | 8.4473 | 直连 |
| 空载撤离 | 2 | 1.4526 | 直连 |

带载抬升距离 104 mm 由当前箱高和余量推得；撤离 50 mm。轨迹按各段停稳的 C2 五次多项式计时。普通速度/加速度/jerk 设计上限为 0.55 rad/s、1 rad/s²、4 rad/s³；接触阶段为 0.1、0.3、1。所有解析峰值审计通过，官方速度 3.14 rad/s 未被超越；没有评估实际力矩。

验证与模拟/视频使用同一个 `sample_quintic_knots`。最终验证的名义细分是 0.0008 rad 关节 L1 总位移和 0.02 s 时间步两者取更密；时间均匀采样下实际最大关节间距可达到名义值约 1.875 倍。**这是密集几何检查，不是连续扫掠体的数学认证**。六个目标最大 FK 位置残差约 13.6 μm、方向残差约 1.93e-5 rad；模拟执行另外每 0.02 s 检查同一几何路径。

实际事件：51.863448 s 附着（轮廓最大平面误差约 4.4 μm），121.877065 s 支撑释放，123.329617 s 撤离完成。回放按几何姿态推进，不冒充动力学；支撑证据为 `GEOMETRIC_SUPPORT_COMPUTED`，支持停稳边界，`supports_continuous_handoff=false`。

## 来源、验证及失败记录

固定来源未改动：feasibility `9db77cb8a51e9bf1821c6e84e90a7632fbce6b26`、perception `f7668c37019c31b85cc848243231f7c99a6d171d`、online `f7f7e0934ad425ab62cd2df8201812df1ba8e505`。来源锁和本地 refs 一致。使用已有 `URDFRobot`、IK、RRT、时间参数化；本轮没有修改公共底层或历史 FANUC 入口。

perception 仅移植 Pose3D/EvidenceKind/指纹依赖，online 仅移植 PlannerBackend、PlanValidator、ExecutionBackend 所需契约闭包。逐定义 AST 与固定提交核对，移植来源及定义列表见 `configs/integration/eco65_round1_ports.json`。同一进程只加载当前工程的一套 unloading_sim，未用其他工作目录拼接导入。模拟后端确实求解/检查/执行轨迹，没有 mock 成功返回。

针对性测试先运行 **10 passed、1 skipped**（当时最终执行未结束），执行完成后补跑该唯一最终任务测试 **1 passed**，共 11 个不同测试完成。覆盖原件/单位/变换、最终凸体覆盖、独立 FK/Jacobian、硬件拒绝、远距吸附、小箱轮廓不足、刚性穿透、非接收台释放、起始状态指纹变化及最终同一箱体事件。没有运行全量 pytest、104/129、整排或历史后端。

早期失败没有算作成功：OCP 8.0.1 接口不匹配后采用 7.9.3.1.1；最初 2 mm 末端分区使刚性凸体距箱约 1.97 mm，小于不变的 2.4 mm 阈值，随后依据真实 CAD 环形面确定裙边边界。修正没有降低碰撞余量，也没有把整个吸盘标为柔性。记录在 `reports/early_findings.json`。

`reports/evidence.json` 保存被测基础提交、运行时原源码哈希、交付代码树哈希、全部官方资产复核、私有输入/配置/轨迹指纹、软件版本、种子和真实路径指标。运行后仅澄清采样注释、把不存在步进 API 的 deterministic-step 能力改为 false，并在核对冻结输入未变后将轨迹/世界指纹绑定到结果、完成单独测试的展示/导出/交付入口；完整路径算法与轨迹未改变。

## 本地成果路径

统一结果根目录：`outputs/eco65_desktop_round1/round1/`。

- [实际 CAD 工作面](../outputs/eco65_desktop_round1/round1/assembly/working_face.png)
- [ECO65-B 与吸具整体](../outputs/eco65_desktop_round1/round1/assembly/robot_tool.png)
- [法兰安装局部](../outputs/eco65_desktop_round1/round1/assembly/flange_mount.png) · [TCP](../outputs/eco65_desktop_round1/round1/assembly/tcp.png)
- [原始几何](../outputs/eco65_desktop_round1/round1/assembly/tool_cad.png) · [碰撞叠加](../outputs/eco65_desktop_round1/round1/assembly/collision_overlay.png) · [180°候选安装](../outputs/eco65_desktop_round1/round1/assembly/mount_yaw180.png)
- [完整 MP4](../outputs/eco65_desktop_round1/round1/replay/single_box.mp4) · [过程缩略图](../outputs/eco65_desktop_round1/round1/replay/contact_sheet.png) · [最终状态](../outputs/eco65_desktop_round1/round1/replay/final.png)
- `trajectory/plan.json`、`scene_snapshot.json`、`replay/states.json`、`reports/validation.json`、`reports/evidence.json`
- `isaac_export/eco65_desktop.usda`、`validated_trajectory.json`、`manifest.json`

USD 含相同真实网格、凸体、六轴关节、官方惯性/限位/有限 effort、重力、有限质量和固定安装；驱动目标单位为度，源轨迹为弧度。它是可重复导出的物理场景起点，**未执行 Isaac 吸附控制器**；后续必须依据实际仿真接触创建/释放约束并检查驱动跟踪，不得逐帧设位姿冒充动力学成功。当前未评价跟踪误差、接触稳定性、力矩或负载资格。

## 已存在并实际使用的命令

在当前工程根目录 PowerShell 使用已有 `.venv`。不重建环境，不安装完整 ROS、Isaac 或厂家 SDK。CAD、轻量回放和 USD 依赖以独立 requirements 保存；不加入核心 pyproject 必需依赖。

```powershell
uv pip install --python .venv\Scripts\python.exe --only-binary :all: -r requirements/eco65-cad.txt -r requirements/eco65-desktop.txt
& .venv\Scripts\python.exe tools/convert_desktop_suction.py
& .venv\Scripts\python.exe tools/build_desktop_suction.py
& .venv\Scripts\python.exe -c "from unloading_sim.eco65.pipeline import plan_and_execute; plan_and_execute()"
& .venv\Scripts\python.exe tools/run_eco65_desktop.py check
& .venv\Scripts\python.exe tools/run_eco65_desktop.py assembly
& .venv\Scripts\python.exe tools/run_eco65_desktop.py replay
& .venv\Scripts\python.exe tools/run_eco65_desktop.py export-isaac
& .venv\Scripts\python.exe -m pytest tests/test_eco65_round1.py -q
& .venv\Scripts\python.exe -m pytest tests/test_eco65_round1.py::test_final_task_evidence_if_finished -q
& .venv\Scripts\python.exe tools/record_eco65_evidence.py
```

规划在首次缺少 scene.json 时从真实轮廓生成场景，已存在时使用本地配置。再次规划会更新 `round1` 派生结果；留存本轮证据时先另存输出目录。检查、装配和回放均不重新搜索路径。统一入口还提供 prepare/plan 分发到同一转换和规划函数；本轮完整求解采用上方实际 Python 函数调用。

## 下一轮最少需要的实测信息

1. 实际法兰/转接板尺寸、安装朝向和实物照片，用于替换假设转接架并标定自由/压缩 TCP。
2. 吸具连同接头、软管和转接件的总质量及大致质心；实际压缩行程。
3. 真空发生/分路方式、是否共用气路及可用 IO；本轮未推断独立控制。
4. 控制器代际/固件、电源与接口铭牌和接线信息，用于后续电气核对；现有 1000 W 标注不代表兼容性已验证。

这些缺项不影响本轮几何结果的复现，但当前仍不授权或支持真机执行。原 STEP、派生网格、详细几何配置、完整场景、图片和视频全部保持本地私有，Git 忽略规则覆盖；提交只包含代码、可选依赖、来源锁和说明。
交付代码树 SHA-256：`9f923c33964cfc0d9bb0c4bf02cb38a0ae547c5f18475ef6e98fd39b3ab44975`。资产/配置/轨迹集合 SHA-256：`d620c97798a4c08750b3df0ad7b74dd3ef7d6f14ca6a57b12f8f5fbfc6d73c0a`。207 个官方文件逐一校验通过。视频解码检查为 618 帧、640×360、5 fps、123.6 s（123.329617 s 轨迹加末帧保持）。
