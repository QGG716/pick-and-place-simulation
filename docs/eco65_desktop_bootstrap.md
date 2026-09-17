# ECO65-B 桌面样机 BOOTSTRAP 报告

准备日期：2026-09-17（Asia/Shanghai）。用户确认实机为 **ECO65-B**。

**源码/资料准备完成，等待用户提供 STEP 并指示继续。**
所选资料均已实际下载；官方许可证文本缺项和声明差异单列如下，不能视为全部资料无缺项。
ECO65 功能适配、三来源功能集成与运动学/动力学验证均未完成。

## 本地源码

工程真实根目录：`D:\code\simulation\pick-and-place-simulation-v0.6-eco65-desktop`。
当前分支：`feat/v0.6-eco65-desktop`。
目标仓库：<https://github.com/QGG716/pick-and-place-simulation.git>。
本轮的本地提交主题为 `chore: bootstrap ECO65-B desktop sources environment and official assets`，
实际提交号可用 `git log -1 --oneline` 查看；本报告包含在该提交中。不推送。

| 来源 | 分支 | 固定提交 | 本地引用 | 状态 |
|---|---|---|---|---|
| feasibility | feat/v0.5-feasibility-core | 9db77cb8a51e9bf1821c6e84e90a7632fbce6b26 | refs/sources/eco65-desktop/feasibility | 已检出为主基线 |
| perception | feat/v0.5-perception-ros2 | f7668c37019c31b85cc848243231f7c99a6d171d | refs/sources/eco65-desktop/perception | 已获取锁定，未集成 |
| online | feat/v0.5-online-continuous | f7f7e0934ad425ab62cd2df8201812df1ba8e505 | refs/sources/eco65-desktop/online | 已获取锁定，未集成 |

初始目录经隐藏文件/Git 根检查确认为空且没有父级仓库。首次远程克隆传输停滞，停止该任务后，
从同机三个既有源目录通过 Git fetch 复制指定提交及其历史；三个目录的 origin 均核实为目标仓库。
新仓库不是 shallow clone，没有 alternates，不依赖旧工作目录。固定引用及历史对象连通性检查通过。
没有 merge、cherry-pick、改动来源分支，也没有跟随来源分支的新 HEAD。
`src/`、`pyproject.toml`、`uv.lock` 与指定 feasibility 基线完全一致。

Windows 沙箱创建的 `.git` 所有者与用户不同，系统不允许归还所有者；已仅为本工程的精确绝对路径
增加用户级 Git `safe.directory`，普通 Git 命令可用。没有通配信任或其他全局 Git 设置修改。

## 最小 CPU 环境和检查

Windows 11（10.0.26200），Python 3.12.10，独立 `.venv`。
使用已有 uv 管理器和基线 uv.lock：

```powershell
$env:UV_CACHE_DIR = Join-Path (Get-Location) '.uv-cache'
uv sync --frozen --extra dev --python 'C:\Users\liukai_xd\AppData\Local\Programs\Python\Python312\python.exe'
.venv\Scripts\python.exe tools/check_eco65_bootstrap.py
```

安装 18 个锁定包（含项目）：numpy 2.5.2、PyYAML 6.0.3、matplotlib 3.11.1、pytest 9.1.1 等。
没有重写 uv.lock，没有系统 Python 包安装或批量升级。
未安装 Isaac、ROS2 全套、CUDA/PyTorch、SAM/MoGe、厂家驱动或 CAD 图形工具。
项目环境未发现 OCP/OCC/cadquery/FreeCAD；PATH/常见 Program Files 位置未发现 FreeCAD。
此为有限可用性检测，不代表扫描了整台机器。Ubuntu/ROS/Isaac 后续环境单独待定。

已通过：
- 工程根、origin、目标分支、三个固定 SHA/本地引用、非浅历史与连通性检查。
- 主基线代码/依赖定义未修改，锁定文件与实际引用一致。
- numpy/yaml/matplotlib 与 geometry、robot、scene、ik、grasp、planner 纯 CPU 导入。
- 207 个清单文件大小/SHA-256 一致；GitHub 原件另核对上游 Git blob。
- STEP/PDF/STL 类型和边界、XML 解析、网页图片解码检查。
- 20 个 XML/URDF/xacro/launch 文件解析、134 个 mesh 引用存在（不执行 xacro、运动学或动力学）。
- STEP 投放目录及占位存在；中文、空格、嵌套目录和 `.step/.stp/.STEP/.STP/.StEp` 忽略规则通过；
  `.gitkeep` 可跟踪。没有为测试创建伪 STEP。
- `git diff --check` 通过；开发说明明确当前仅 BOOTSTRAP。

## 实际下载的官方资料

归档根：`assets/robots/realman_eco65/official/`。
逐文件来源、官网入口、真实下载 URL、下载时间、适用范围、版本、大小、SHA-256、许可证和完整性状态
见 [SOURCE_MANIFEST.json](../assets/robots/realman_eco65/official/SOURCE_MANIFEST.json)。
共 **207 个条目，93,465,377 字节**，其中 206 个非空；唯一空文件是官方 API2 原有 `__init__.py`
包标记，已核对 Git blob，绝不是模型占位。CAD/PDF/mesh 全部非空。
参考 6F STEP 初次传输中断，按同一官方固定 URL 重试成功，无未解决的下载失败。

| 内容 | 实际文件/范围 |
|---|---|
| ECO65-B 机械 CAD | `cad/rm_models/ECO65/robot_model/ECO65-B_robot_model.STEP`，9,790,914 字节 |
| 其他变体参考 | 同层 ECO65-6F 和 ECO65-6FB STEP，分别保留；不作为本机配置 |
| 官方机械模型 | `description/rm_models/ECO65/urdf/ECO65-B/`，以及独立 6F/6FB 子目录；完整关联 mesh、配置、package.xml、根 LICENSE |
| ROS2 模型 | `description/ros2_rm_robot/rm_description/` 的 ECO65 标准/6F/6FB URDF、xacro、mesh、launch、rviz、构建说明 |
| 尺寸图 | `ECO65_Base_Mounting_Hole_Position_Diagram.PDF`、`ECO65_End_Adapter_Hole_Position_Diagram.pdf`、`ECO65_Full_Arm_Dimension_Specification.pdf` |
| 用户手册 | RobotGen3 与 RobotGen4 分目录保存 `睿尔曼超轻量机械臂(ECO系列)-V1.2.0-用户使用说明.pdf`；同名不同内容，分别为 10,734,051 / 10,082,450 字节 |
| 网页及附件 | 三/四代模型、手册、ECO65 参数/D-H、硬件接口、ROS2 模型、二次开发入口共 12 原 HTML；另有 45 图片和 4 仓库发现清单 |
| 后续软件接入参考 | ROS2 rm_driver 源码/说明及原随附库、API2 Python 接口/C 头文件/README；JSON 协议官方入口保留在二次开发原页面中 |

官网入口沿链指向的固定版本：
- rm_models main：`bdb12ca3db532cb677ae33fa94795ac9c1b01f98`。
- ros2_rm_robot humble：`c941b565e4f9174afa36561f143ef5fbbb744750`。
- Dev_Center main：`d07dd82f3e5aef3f8c4d7d6f82c4fa0dc3b32f9f`。
- RM_API2 main：`9d75cc995f52095837dddca594531621be18cf7b`。

原件保留文件名和相对目录；本轮直接下载文件，无 ZIP 解压关系。
HTML 原链接未改写，图片原件位于 web/attachments，可由清单映射阅读。
没有用整理稿冒充官方原件，也未把 CAD 转为新模型或修改 URDF。
厂家源码仅参考归档，未安装、导入、编译、运行；不是完整可启动 ROS/SDK 环境。
大型原件、二进制、网页、`.venv` 和缓存仅本地保存；Git 跟踪清单、说明和两个准备脚本。

## 缺失和待确认项

1. **许可证缺项/范围差异**：rm_models 根 LICENSE 是 Apache-2.0 声明，ECO65 URDF package.xml 写 BSD。
   ros2_rm_robot 的 rm_description/rm_driver package.xml 写 `TODO: License declaration`，其固定清单未提供适用 LICENSE；
   RM_API2 固定清单没有 LICENSE。未拿示例包许可证替代。保留原声明并等待官方确认，不宣称授权资料齐全。
2. **控制器代际/固件/法兰修订待确认**。官网四代模型页明示部分模型尺寸图以三代控制器为例；
   已分别归档 [三代手册入口](https://develop.realman-robotics.com/robot/download/manual/)
   和 [四代手册入口](https://develop.realman-robotics.com/robot4th/download/manual/)，不推断用户控制器。
3. **官方模型参数差异**：机械库 ECO65-B URDF 的 J1 effort=60，ROS2 标准模型为 100；
   J3 velocity 分别 3.14 与 3.92；J2/J3 坐标系原点/姿态声明及 J5 平移也不同。
   这些是原文件字段差异，尚未证明物理等价或哪一套适用于实机；未选择参数写入运行配置。
   [三代参数页](https://develop.realman-robotics.com/robot/robotParameter/ECO65OntologyParameters/)
   区分 6FB（d6=96.7 mm）与旧式 6F（108 mm）；
   [四代参数页](https://develop.realman-robotics.com/robot4th/robotParameter/ECO65OntologyParameters/)
   用 6F 标记 96.7 mm。此差异属于其他变体，不能套给 ECO65-B；原文完整保留。
4. 用户小吸具 STEP 尚未放入；质量、材质、TCP、法兰关系、杯数、真空气路 unknown/null。
   用户提供的 1000 W 电源仅记录铭牌功率，未确认输出电压、电流、接线或实机兼容性。

## STEP 投放位置与停止点

唯一真实绝对路径：

`D:\code\simulation\pick-and-place-simulation-v0.6-eco65-desktop\assets\tools\desktop_suction\cad\raw`

复制已有 `.step`/`.stp` 原件，保留中文、空格、大小写和装配目录关系，无需转换。
原件及 meshes/collision 后续派生文件默认仅本地保存；本轮 meshes/collision 只有 `.gitkeep`。

未开展：吸具解析/网格化/OBB/安装推断、模型适配、功能移植、规划、单箱任务、仿真/回放、Isaac、
全量 pytest、104/129 历史验收、整堆测试、实机连接（含只读）、IP 扫描、IO/真空控制。
没有文件监听、后台任务或自动继续机制。本轮结束，等待用户放入 STEP **并明确指示继续**。
