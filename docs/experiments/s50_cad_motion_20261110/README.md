# S50-2000 真实 CAD 六轴模型与原生链路（2026-11-10）

**本轮已实现真实七部件、六关节 CAD 运动模型和关节查看器，并在 GPU 服务器运行核心 / Pinocchio / 原生 worker 的 FK 核对。正式安装下的工作初态 IK 与单箱规划尚未运行，不能写“机械臂替换完成”。**

工作分支为 `feat/v0.5-s50-moveit2-native-single-carton`，延续 `cff77cebcad2bd6566e7f0d5897f454de8738e04` 及其资产发布提交 `c4aae37ffad5fa93288f9b530be5a5e6a7c34860`。真实 STEP、手册及派生资产沿用用户已经授权的发布范围；原来源及许可说明继续适用。没有修改 FANUC 源分支、检查点或运行任务。

## 已运动的真实模型

模型来自原 `S50.STEP`（SHA-256 `7296244abc02bca236a7ed26a0f10f5cc56fc63b7381fc1293b2edbb92509bc3`）、七个独立 CAD 部件和 `interface_measurements.json` 的六条实测轴线。没有使用其他机型、缩放模型或把整机作为一个活动连杆。

- 七个视觉网格转为各自连杆局部坐标；六个独立 revolute joint 由测得轴线建立。所有几何使用米，角度使用弧度。
- 按本轮授权，原 CAD 装配姿态定义为仿真 `q=0`，测得轴的右手方向为正。这是仿真约定，不是厂家控制器零位或符号映射。
- 安装支承面原点为 CAD `[0,-0.003,0] m`；安装轴线、末端法兰面和工具安装参考分别定义。未虚构物理连接件或 TCP 加长量。
- 显示范围 `[-π,π]`、解析用 `velocity=1` / `effort=0` 明确属于 `cad_preview_fk_only`，不当作机械范围、额定速度或力矩。原生预览会话实际拒绝 IK / 运动请求。
- CAD 零姿态叠加 900 mm 安装高度后，本体最高约 3.259 m，超过 2.7 m 顶棚。这不是合格作业初态；查看器默认独立 CAD 视图，世界叠加需要明确启用，标注安装定向未验证。

模型定义：[`cad_kinematic_model.json`](../../../assets/robots/huayan_s50_2000/generated/cad_kinematic_model.json)。定义 SHA-256：`38c6f9c501b7539ffc4417a3c397dfc2e363ff85da18bec83afa3cd834b38036`。URDF、SRDF、逐网格 SHA、单位、轴线、坐标变换、生成脚本及原始来源均在同目录的模型清单中绑定。

## 实际验证

| 实际检查 | 结果及范围 |
| --- | --- |
| q=0 重新组合全部原 CAD 顶点及三角索引 | PASS；最大坐标差 `2.22e-16 m` |
| 六轴分别正转 / 反转 | 12 次 PASS；仅正确下游部件运动，绕原测量轴线；未重复套装配变换 |
| 核心 URDF / 递推 FK / 独立空间轴线乘积 FK | 25 姿态 PASS；最大矩阵元素差 `6.66e-16` |
| 查看器独立 JavaScript FK / 核心 URDF FK | 50 姿态、400 frame PASS；最大差 `4.44e-16` |
| 服务器核心 / Pinocchio / C++ 原生 FK | 15 姿态 PASS；Pin 最大差 `1.11e-15`，native 最大差 `4.44e-16`；实际加载 9 个碰撞几何对象 |
| 预览 worker 请求 IK | 实际返回 `CAD_PREVIEW_FK_ONLY_NO_MOTION`；没有调用 IK 求解器 |
| 正式单箱入口 | 实际运行并返回 `NOT_RUN_MODEL_NOT_READY`；在准入阶段停止，初态 IK / 单箱规划次数均为 0 |
| 浏览器画面人工 / 自动视觉核对 | 未完成。CUA 因浏览器宿主不可用及无法确认当前 URL 停止；没有绕过该限制。已完成离线脚本语法、模型数据及独立 FK 检查 |

上述 FK 结果只证明真实 CAD 仿真坐标内的运动链一致，不证明机械限位、工作空间可达或环境无碰撞。查看器为显示而转为 float32 的最大坐标差约 `5.96e-8 m`，与双精度 FK 误差分开记录。

证据：[`kinematics_validation.json`](../../../assets/robots/huayan_s50_2000/generated/kinematics_validation.json)、[`viewer-fk-validation.json`](viewer-fk-validation.json)、[`viewer-checks.json`](viewer-checks.json)、[`native-fk-report.json`](native-fk-report.json)、[`verification-summary.json`](verification-summary.json)。服务器日志与完整运行目录留在服务器；只同步本报告需要的小型 JSON。

## 固定布局

| 参数 | 本轮值 |
| --- | --- |
| 横带前沿至箱堆前面 | 700 mm |
| 真实安装面世界位置 | `[-1.85, 0.15, 0.90] m` |
| 安装面高度 / 原安装世界朝向 | 900 mm / 沿用冻结值；S50 CAD 到实际安装的绕轴对应尚未核定 |
| 安装轴线距底盘 +X 前沿 | 450 mm |
| 横带行程 / 初始高度 | 550–1500 mm / 600 mm |
| 原工具连接延长段 | 200 mm |
| 初始箱数 / 已卸 checkpoint | 完整 40 箱 / 未导入 |
| 世界坐标 | +X 朝箱堆深处，+Y 向左，+Z 向上 |

准入及查看器生成时均比较完整非机器人实体的尺寸、位姿、机构行程和工具固有定义，结果 PASS；详见 [`admission-local.json`](admission-local.json) 的 `layout_difference`。底盘、支座、传送带、剪叉、导轨、箱堆没有移动或变形。新机器人本体不保留 FANUC 包络。当前合格完整安装场景尚未建立，`full_scene_fingerprint=null`；预览使用独立模型 SHA 与冻结环境 SHA，不伪称旧完整场景指纹相同。正式 context 建立后会把 S50 模型身份纳入新完整场景指纹。

## 两张缺面及碰撞资格

J1 面 74、J2 面 75 的原三角化失败已定位并保留。采用原平面和同一原边界 wire 修复 trim 方向，各生成 1023 个三角形，作为所属连杆独立碰撞补片。原七个视觉网格不删面、不缩放、不改变索引；没有 AABB 填充、整臂凸包、额外厚度或法向外扩。

精确边界身份和包围盒保持，8194 点边界采样至网格的最大差约 50 nm；这是采样结果，不冒充连续网格的解析 Hausdorff 证明。J2 替换后整个 BRep 有效、体积保持；J1 仍有 `BRepCheck_SubshapeNotInShape`。进一步定位到内腔壳组合，但两壳精确距离约 64.14 mm、Boolean Common 为空，不能解释成实际穿透。所有原始内外表面仍保留。**局部补面成功不等于整体碰撞资格通过，也不能据此报告机器人与横带最小间隙。**

详见 [`source-evidence.md`](source-evidence.md) 和 [`coverage_report.json`](../../../assets/robots/huayan_s50_2000/collision_repair/coverage_report.json)。真实碰撞加载已跑通；整体覆盖、适用自碰撞排除及物理腕部 / 工具例外仍待核定。

## 已接入的正式流程与仍缺的参数

`plan_s50_single_carton.py` 已接到真实 connector factory、同源 Pinocchio/Coal 网格、安装 / 法兰 / 工具变换、新初态有限种子 IK、原 `run_trial` 以及经过当前 native 轨迹逐点验证的导出。沿用高层 FRONT 合格抽离后直接释放策略；没有复制 FANUC 六角度、历史轨迹、肩肘硬限制或底座碰撞豁免。接线测试使用的合成 fixture 不计为 S50 实际规划结果。详细范围见[原生链路审计](../s50_model_integration_20261110/native-adapter-audit.md)。

现有资料仍不能确定以下物理参数，因此正式初态与单箱链路没有启动：

1. 六轴 `q_controller = sign × q_sim + offset` 的每轴 sign / offset。特别是 J2 的控制器 `[-190°,10°]`，不能直接套到 CAD sim0；J3 的 `±160°` 同样需要映射。配置保留空值及依据缺项，不扩大范围。
2. 实际安装绕 J1 轴的定向，与固定世界安装坐标如何对应。预览坐标约定没有改变实际安装值。
3. S50 法兰至现有吸具安装面的真实刚体关系，包括间距、绕法兰方向、连接实体。旧工具及 200 mm 延长段仍保留；没有虚拟补长度。
4. J1 整体 CAD 拓扑、完整碰撞覆盖与适用自碰撞 / 腕部工具排除仍需验证；这是未完成验证，不只是“缺文档”。

速度版本冲突单独进入时间参数化门槛，不阻断静态 FK、CAD 预览或已经具备定义的几何检查。官网 J1/J2/J6 为 180/120/224°/s，手册为 244/150/180°/s，尚无版本对应依据。未选择临时速度冒充厂家值。质量、质心、惯量、力矩和加速度缺项不阻断几何建模，也不据此宣称动力学已验证。

S50 额定载荷 50 kg；原工具 20 kg、箱体 42.5 kg，共 62.5 kg，负载资格未通过。没有降低质量或提高额定载荷；这个问题与几何可达性独立记录，不用于提前拒绝几何搜索。

## 打开与复现

可直接用现代 Edge / Chrome 打开 [`viewer.html`](viewer.html)，无需服务器和外网。六关节滑块及数字输入使用显示范围；可显示坐标轴、连杆、冻结 40 箱场景和独立工具接触目标。当前没有实际任务轨迹，所以不播放伪造抓放动作。后续加载的原生关节轨迹必须匹配模型、环境、安装变换及 40 箱身份；仅回放机器人关节，箱体与横带仍标明初始静态参考。

本地工作目录：`D:\code\pick and place\.codex-tools\worktrees\s50`。

```powershell
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 assets/robots/huayan_s50_2000/build_cad_kinematic_model.py
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 tools/export_s50_cad_motion_viewer.py
Start-Process 'docs/experiments/s50_cad_motion_20261110/viewer.html'
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 tools/check_s50_integration.py
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 tools/plan_s50_single_carton.py --mode single-carton --output outputs/s50_cad_motion_20261110/single-carton
```

后两个命令当前应非零退出并保存真实缺项。模型生成和预览所需的局部网格以 `.obj.gz` 随库保存，加载器校验 SHA 后解压；不需要 CAD 内核才能打开已导出的 HTML 或加载已生成 URDF。重做 STEP 局部补片的 OCP 复现命令见来源报告。

服务器 chroot 根目录 `/root/m710-gap-search-20261107/rootfs`；S50 独立目录 `/work-s50-cad-motion-20261110-v1`。在该 chroot 工作目录中执行：

```bash
/opt/planning-venv/bin/python tools/plan_s50_single_carton.py --mode preview-fk \
  --config configs/robots/huayan_s50_2000_cad_preview.yaml \
  --output outputs/s50_cad_motion_20261110/native-fk-reproduce \
  --worker-command '/bin/bash /work-s50-cad-motion-20261110-v1/worker-fk.sh'

# 资料补齐且准入通过后，仍只请求一个代表顶层箱；当前会保留 NOT_RUN。
/opt/planning-venv/bin/python tools/plan_s50_single_carton.py --mode single-carton \
  --output outputs/s50_cad_motion_20261110/single-carton-reproduce \
  --worker-command '/bin/bash /work-s50-cad-motion-20261110-v1/worker-fk.sh'
```

`worker-fk.sh` 名称用于隔离本轮工作；FK-only 是初始化请求的能力限制，正式模型具备资料时由正式入口初始化对应运动能力。本轮没有启动全 40 箱搜索或 Isaac 实验。
