# S50 任务模型、安装接触与真实网格诊断

本轮基于 `7bd1392d5f3b122817ada116b00afddfb9c1d9cc`，继续在 `feat/v0.5-s50-moveit2-native-single-carton` 工作。真实六轴 CAD 运动模型继续使用既有七个部件、六条测量轴线和两张局部补片。**任务模型生成和安装接触接线已实现，正式初态及单箱卸箱尚未运行，不能称为机械臂替换完成。**

## 冻结布局

| 参数 | 值 |
| --- | --- |
| 横带前沿至箱堆 | 700 mm |
| 真实安装面位置 | `[-1.85, 0.15, 0.90] m` |
| 安装面高度、世界朝向 | 900 mm；沿用冻结朝向 |
| 安装轴线至底盘 +X 前沿 | 450 mm |
| 横带行程、初始高度 | 550–1500 mm、600 mm |
| 工具延长段 | 200 mm |
| 初始箱数 | 完整 40 箱，未继承已卸箱检查点 |
| 工具、单箱质量 | 20 kg、42.5 kg，未修改 |

底盘、支座、传送带、剪叉、导轨、车厢、箱堆和工具固有定义均保持冻结基线。新旧机器人非机器人布局检查见 [layout-freeze-local.json](layout-freeze-local.json)。安装穿透负面测试只移动独立测试副本，不修改任务布局。

Windows 与 Linux 的原始环境指纹不同：仅六处剪叉角度元数据存在最大 `1.11e-16 rad` 的浮点末位差，四个高度的 592 个实体记录及全部工具定义完全一致。两个原指纹均保留；没有改写服务器记录或绕过严格导入检查。[独立比较](platform-layout-comparison.json) 记录精确差异。实际诊断查看器在服务器生成，使用该次查询的原始环境指纹。

## 任务模型及坐标

新增 `tools/build_s50_task_model.py`，按 `q_controller = sign × q_sim + offset` 逐轴转换有来源的控制器限位，生成独立 `task_model.json`、任务 URDF/SRDF 和 `model.yaml`。它们共享限位、连杆、坐标及模型 SHA；轨迹导出前重新核对模型和映射依据。CAD 预览仍独立保留，±180° 滑块范围没有进入任务机械限位。

本轮真实输入仍缺六轴映射，实际生成命令返回 `MODEL_NOT_READY`，没有写出伪造的任务模型。任务模型和轨迹协议测试中的合成映射仅用于软件测试。Windows 跨盘输出已修复：同盘使用相对路径，跨盘使用真实绝对引用并明确标为不可直接迁移的本机产物。

安装面、模型根坐标、物理法兰、原工具安装参考和吸盘接触目标分别记录。已推导的 CAD 变换以及仍缺的具体图纸见 [task-generation-local.json](task-generation-local.json)：

1. **J1–J6 控制器映射**：需要本份 CAD 参考姿态对应的六轴控制器读数及各轴正向点动方向，或对应版本的厂家零位/方向表。特别是 J2 的 `[-190°,10°]` 和 J3 的 `±160°` 不能直接套到 CAD 零位。现有手册 PDF 第 25、33 页的尺寸/运动范围图未给出这组读数。
2. **固定安装绕轴方向**：需要基座电缆出口、定位销或螺孔基准所定义的安装 +X，与冻结世界安装方向的对应关系。第 55 页的安装孔尺寸不足以确定该对应。
3. **S50 法兰至现有工具的物理连接**：需要配合图或实测装配，标出两安装面间距、绕法兰角度、螺孔对应和真实连接实体。第 56 页的裸法兰尺寸不能代替该装配关系。没有新增虚拟长度或连接件。

最高速度的版本冲突继续单独记录，不阻断静态 FK 和几何诊断；本轮未采用临时时序参数。惯量、力矩及动力学仍未验证。S50 额定载荷 50 kg，原工具与箱体合计 62.5 kg；负载资格与几何检查分开，没有降低质量制造通过结果。

## 安装接触与碰撞范围

`qualified_mount_pairs` 已接入 S50。Python/Coal 和原生 MoveIt 都从实际固定基座网格验证安装面高度、法向、接触区域支承范围及非穿透，许可只可能是 `s50_base ↔ mount_plinth`。活动连杆、工具和其他环境对象不获得许可。

原基座网格有且仅有一个源顶点低于真实支承面约 0.54 µm。已在原 STEP 三角化中重现并定位到真实支承 face 19；许可只识别绑定源文件和坐标的这个离散误差，不是整体安装穿透容差。详见 [安装方法](mount-contact-method.md)、[源表面审计](source-collision-audit.md) 和 [源证据](../../../assets/robots/huayan_s50_2000/generated/mount_contact_source.json)。

J1 的残余 BRep 壳组织问题、网格边界/重复/退化三角问题、实际运行时碰撞结果分开记录。没有用 BRep 无效直接代替运行时碰撞，也没有因距离查询成功就宣布完整碰撞覆盖合格。相邻连杆接口和同一连杆补片的原始相交记录保留，未自动扩大 SRDF 排除；物理工具安装尚缺，S50 腕部/工具豁免仍为空。

## 实际验证结果

| 检查 | 实际结果及边界 |
| --- | --- |
| 完整场景 Coal 查询 | 16 个 CAD 构型 × 1,368 对 = **21,888 次**；148 个环境实体，未知查询 0。横带使用冻结的真实斜切 Convex，其余使用各自声明的实体；没有用横带外包盒替代 |
| 安装接触三情形 | 正常贴合仅许可固定 base；下移 1 mm 拒绝；J2=2.4 rad 的 `s50_link_2 / mount_plinth` 碰撞不许可。Coal/MoveIt 的 21 个 link/plinth 结果一致 |
| 核心 / Pinocchio / native FK | 15 姿态，最大矩阵元素差分别为 `1.11e-15`、`4.44e-16`；原生预览请求 IK 被明确拒绝 |
| 独立 JavaScript / 核心 FK | 50 姿态、400 frame，最大差 `4.44e-16` |
| 原生编译及 FANUC 回归 | worker/probe 实际构建；原生 ProcessPolicy 24 例及 FANUC gap 几何合同通过；服务器相关 Python 61 项通过 |
| 任务模型及查看器协议 | Windows 默认临时目录跨盘测试及相关接口 37 项通过；最终查看器任务协议 16 项通过；其他定向运行见 [测试记录](local-test-runs.json)，不将重叠运行相加 |
| 浏览器视觉验收 | 未完成，见下文与 [浏览器记录](browser-validation.json) |

完整来源哈希、程序版本、实际碰撞对象、各姿态横带距离和肘腕范围见 [server-validation.json](server-validation.json)。CAD 零位实际有 `s50_link_3 / trailer_ceiling` 相交；J2=2.4 rad 的折叠诊断还与底盘、纵带和地板相交。这些是保留的诊断构型，未被当成合法初态。

本轮发现并修复了两项实际运行问题：最初诊断漏了 99 个剪叉/导轨实体；安装缓存使用临时 Boost 包装地址会误拒未变化网格。前者已用完整场景重新运行，后者改为实际 BVH 内容签名并完成三情形复测。[旧诊断范围与哈希](retained-diagnostic-correction.json) 及服务器失败日志保留，没有用修复后的成功覆盖失败成本。

加载器实际少于源网格 56 个三角形，数量与源零面积三角形相符；尚未逐三角索引证明全部差异的对应，因此不宣称网格导入逐面无损。运行时查询有效与连续表面/封闭实体覆盖资格仍分开。

## 正式规划结果

[实际单箱入口记录](single-carton-gated-local.json) 为 `NOT_RUN_MODEL_NOT_READY`：初态 IK **0 次**、单箱规划 **0 次**，worker 未为正式任务启动。失败类别是资料与模型准入缺项，不是 IK 超时、环境碰撞导致的任务失败，也不是 S50 结构不可达。

未运行全 40 箱搜索或 Isaac。原 FRONT 抽离后在合格接收区直接释放的流程保持；没有复制 FANUC 初态或轨迹。没有合法成功轨迹，所以不能给出抓取、抽离、释放、退出、返回各阶段的已验收净空。CAD 诊断的横带距离、肘腕包络与对应 q 单独保留，不用于机型能力排名。

## 查看器与复现

离线文件：[viewer.html](viewer.html)。它提供真实六关节调节、坐标轴、冻结完整场景、工具接触目标参考，以及实际诊断构型、碰撞对象与有效最近点。手动改变角度或横带高度会撤下旧碰撞记录，避免把旧查询套到新姿态。实际工具尚未装到法兰，显示为独立目标参考；没有伪造卸箱动画。

浏览器视觉验收本轮仍未完成：CUA 浏览器宿主不可用，随后因无法可靠确认当前 URL 停止，未绕过这一限制。JavaScript 语法、独立 FK 和导出数据校验与浏览器验收分开记录；不能据此声称浏览器验收通过。

本地打开：

```powershell
Set-Location 'D:\code\pick and place\.codex-tools\worktrees\s50'
Start-Process 'docs/experiments/s50_task_collision_20261111/viewer.html'
```

正式入口（当前应返回资料缺失，退出码 2）：

```bash
python tools/build_s50_task_model.py
python tools/check_s50_integration.py
python tools/plan_s50_single_carton.py --mode single-carton \
  --config configs/robots/huayan_s50_2000.yaml --seed 50 \
  --output outputs/s50_task_collision_20261111/single-carton-new
```

服务器独立目录 `/work-s50-task-collision-20261111-v1`，位于 chroot `/root/m710-gap-search-20261107/rootfs`。ROS 使用 `/opt/ros/humble`，Python 使用 `/opt/planning-venv/bin/python`。所有检查点、worker 和输出与 FANUC 分开。完整原始查询与失败日志留在服务器，仅将交付报告和查看器同步到仓库，没有下载证据包。

在上述服务器工作目录内复现 CAD 诊断及查看器（默认 15 姿态；支座折叠构型另行指定）：

```bash
/opt/planning-venv/bin/python tools/diagnose_s50_runtime_collision.py \
  --output outputs/s50_task_collision_20261111/reproduce-full-scene
/opt/planning-venv/bin/python tools/diagnose_s50_runtime_collision.py \
  --pose mount_probe_j2_2p4 --output outputs/s50_task_collision_20261111/reproduce-active-link
/opt/planning-venv/bin/python tools/extract_s50_diagnostic_view.py \
  outputs/s50_task_collision_20261111/reproduce-full-scene/runtime-collision-report.json \
  outputs/s50_task_collision_20261111/reproduce-active-link/runtime-collision-report.json \
  --output outputs/s50_task_collision_20261111/reproduce-viewer-records.json
/opt/planning-venv/bin/python tools/export_s50_cad_motion_viewer.py \
  --diagnostics-report outputs/s50_task_collision_20261111/reproduce-viewer-records.json \
  --output outputs/s50_task_collision_20261111/reproduce-viewer.html
```

查看器摘录保留每姿态全部相交对及原始距离最低的 40 对；原始完整 21,888 对、构型文件和查询日志留在服务器。摘录及 HTML 均绑定完整报告 SHA，摘录不冒充完整证据包。正式任务资料补齐后再使用任务模型及原生单箱入口，不把本节 CAD 诊断当成初态搜索。

本轮按用户授权清理了可重新下载的缓存；数据盘实际回收约 1.25 GiB。FANUC/S50/ECO 实验输出、检查点和已安装环境均保留。[清理记录](server-cache-cleanup.json) 列出精确范围和前后空间。
