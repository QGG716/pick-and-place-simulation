# 厢顶、桅杆与官方皮带资产：2026-09-16

本轮只更新感知场景并执行固定流程，`planning_admissible=false`。历史
`layout_v1`、40 箱实体、完整纸箱资产分配、底盘、横纵带占地、机器人基座和
名义采集关节姿态均保留。未读取设计方案 V1.7，未修改其他分支。

## 图片优先

最终补光场景的完整分辨率原始图：[上模组](evidence/roof-mast-lit-20260916/upper.png)、
[下模组](evidence/roof-mast-lit-20260916/lower.png)、
[整体](evidence/roof-mast-lit-20260916/overview.png)、
[杆顶与厢顶](evidence/roof-mast-lit-20260916/mast_roof.png)、
[A06 带面和交汇](evidence/roof-mast-lit-20260916/conveyor_junction.png)。
均来自实际 Isaac 场景，未做亮度后处理。四盏灯与相机同属桅杆组件。

首轮带厢顶原图已先行交付，且未提亮：[上](evidence/roof-mast-20260916/upper.png)、
[下](evidence/roof-mast-20260916/lower.png)、[整体](evidence/roof-mast-20260916/overview.png)、
[杆顶与厢顶](evidence/roof-mast-20260916/mast_roof.png)、
[交汇](evidence/roof-mast-20260916/conveyor_junction.png)。
它们在原有弱补光下很暗；随后用户明确要求在桅杆上增加补光灯。
首轮数据和已完成的两次感知均保留，不替换为补光后的结果。

## 参数对照

| 参数 | 本轮前 `b040dc9` | 最终场景 |
|---|---|---|
| 世界原点 | 箱墙前面在地板投影 | 保持，X=0 不是车门 |
| T_W_A 平移 | [-1.95, 0.35, 0] m | 保持 |
| 底盘：尺寸；W 中心 | [2.10,1.50,0.60]；[-1.95,0.35,0.30] m | 保持 |
| 横带：尺寸；W 中心 | [0.70,1.50,0.12]；[-0.55,0.35,0.54] m | 保持 |
| 纵带：尺寸；W 中心 | [2.80,0.70,0.12]；[-1.60,-0.75,0.54] m | 保持 |
| 机器人基座 W | [-1.325, 0.35, 0.60] m | 保持，未采用归档的 -1.41 |
| 箱墙 | 8×5，40 箱，单箱 0.60×0.40×0.30 m | ID、位姿、材质与资产分配保持 |
| 安装法兰世界 Z | 1.3293411964 m | 保持；provisional engineering datum |
| 桅杆最高世界 Z | 2.8293411964 m | 2.600000 m |
| 法兰至杆顶 | 1.500000 m | 1.2706588036 m，由安装变换推导 |
| 上光心世界 Z | 2.6293411964 m | 2.400000 m，水平朝箱墙 |
| 下光心世界 Z | 1.6293411964 m | 保持，下俯 20° |
| 上下光心间距 | 1.000000 m | 0.7706588036 m |
| 两光心 X、Y | [-1.325, 0.750] m | 保持 |
| RGB-D 分辨率、FOV | 2592×1944，120°×65° | 保持原内参和重采样规则 |
| 采集/触发频率 | 30 Hz / 2 Hz，同组同步 | 保持 |
| 全车内长×宽×高 | 归档长/高未定义，旧视觉仅局部 | 9.60×2.30×2.70 m |
| 局部 X 范围 | [-2.00, 4.00] m | [-3.80, 4.00] m，门位置未定义 |
| 顶板 | 仅 RoofRim | 完整可见、遮光、静态碰撞顶板，Z=[2.70,2.76] m |
| 桅杆至顶板净空 | 未形成真实顶板净空检查 | 实际 USD 检查约 0.100 m |
| 横带/纵带 | 蓝色方体视觉代理 | 官方 A06 皮带工作段 + 项目低机架，原碰撞包络 |
| 皮带承载面 | Z=0.60 m | 保持；静止，无动态输送结论 |
| 桅杆补光 | 四个半径 0.035 m、强度 2500 的光源代理 | 四盏带实体支架/外壳的灯，半径 0.045 m、强度 150000，5000 K |

补光灯使用未归一化的 USD 光强、exposure=0，与相机曝光参数不同。光源安装在
上下模组左右两侧，模块局部坐标 [0.075, ±0.180, 0] m，方向跟随模块。
这是仿真照明参数，不是实测照度或灯具制造认证。没有移除顶板、添加隐藏环境灯、
修改纸箱自发光或在 RGB 上后期提亮。原有环境灯保持原位置和强度。
参数语义参考 [OpenUSD LightAPI](https://openusd.org/release/user_guides/schemas/usdLux/LightAPI.html)。

## 资产状态

**A06：实际使用原生皮带网格、UV 和材质。** 原资产整机高约 2.311 m，未整体缩放。
只适配皮带工作段，保留约 0.0390 m 厚度；横带一段 1.50 m，纵带两段各 1.40 m，
带宽均 0.70 m。机架为原包络内的项目低机架，原有实体碰撞保留。

**A43：检查后未适配、未放入最终场景。** 官方分类为 ROLLER/Y_MERGE，主支路
端点约 [4,0,0] m，另一端点约 [3.283,-1.852,0] m、方向 -45°，整机约
4.000×2.831×1.166 m。它与当前 90° L 接边的方向、分支数量和占地不符。
没有隐藏支路后假称成功，也没有任何转向输送功能验证。

**官方车厢：未替换。** 在本轮限定的官方来源检索中未取得适合且可核验的内景资产，
最终使用明确标注的参数化厢体。没有将一般箱、桶、托盘资产声称为车厢。
来源、适配说明与限制见 [WORKCELL_USD_SOURCES](../../assets/materials/industrial/WORKCELL_USD_SOURCES.md)，
文件哈希与完整 A06 依赖见 [conveyor_assets.json](../../configs/isaac/conveyor_assets.json)。

## 数据与合同链

[workcell_v2.yaml](../../configs/isaac/workcell_v2.yaml) 定义当前生效的车厢、局部范围与
基座安装；[perception_sensing_pose.yaml](../../configs/isaac/perception_sensing_pose.yaml)
定义世界杆顶目标、相机和灯具安装。`load_vision_rig_spec` 从同一安装变换推导杆长和
上模组高度。`derive_effective_scene` 验证归档父合同后生成新场景及指纹；bundle index
明确指定新的 effective contract，采集脚本不再直接使用旧合同决定本轮实体。

光学相机、深度、TF 记录和 manifest 使用同一 T_W_C。ROS sensor adapter 仍从采集
metadata 读取变换；本轮仅导出检查数据，未运行完整 ROS 验收。新增顶板、地板和
侧壁进入 manifest 与 handoff 的静态环境描述。派生场景没有继承其他分支的执行认证。

GT/oracle 框提示仍是现有流程输入；实际渲染的 GT mask 只用于事后评价。
扫描纸箱的名义盒体角点/面不能当成精确网格真值，因此本轮使用实际渲染对象 mask
评价分割边界，不认证实体角点或完整箱体。没有模型新增、训练、阈值调参或规划。

## 最小验证

GPU 服务器执行 `pytest -q`：**395 passed, 1 deselected，5.71 s**。
详见 [测试日志](evidence/roof-mast-lit-20260916/pytest-f24f63a.log)。
最终采集代码为 `f24f63a`，采集时工作区干净；首轮暗图代码为 `acc6b8c`。
总共两次最小采集，第二次由用户看过暗图后明确追加的实体补光要求触发。
每个场景各模组只运行一次既有感知流程，首轮暗图及其结果独立保留。

[实际 USD 检查](evidence/roof-mast-lit-20260916/workcell_checks.json)为 `STATIC_GEOMETRY_PASS`：

- 整个视觉组件最高 Z=2.5999999998 m，至顶板内表面净空 0.1000000009 m。
- 地板、两侧壁、顶板均为可见且启用碰撞的实体；局部覆盖 X=[-3.8,4.0] m。
- 临时穿顶立方体被启用碰撞的顶板包围盒交叠检查检出，负例随即移除。
- 两相机实际 USD 变换与 manifest 一致，前向光轴与杆体/壳体无交点。
- 四盏灯的实体安装、光强、半径、方向和高度通过检查。
- A06 实际网格的工作面 Z≈0.60 m，横纵带边界符合原占地。
- 名义姿态下机器人网格与侧壁/顶板无 AABB 交叠候选。

这些是名义静态 USD 检查，没有运行动态 PhysX 接触验收或全姿态/轨迹验证。
[纸箱与 RGB-D 检查](evidence/roof-mast-lit-20260916/basic_checks.json)确认 40 箱资产与位姿、
原始 RGB/深度绑定、实际多网格实例合并、内参和采集元数据一致。
脚本沿用状态名 `BASIC_CONSISTENCY_PASS_IMAGE_REVIEW_PENDING`，它本身不是感知算法通过声明。

| 观测 | 原无完整顶板场景 | 有顶板、原弱光 | 最终实体补光 |
|---|---:|---:|---:|
| 上模组 RGB 三通道均值（0–255） | 66.84 | 2.69 | 111.44 |
| 下模组 RGB 三通道均值（0–255） | 64.51 | 2.56 | 99.88 |
| 上模组实际可见箱数 | 15 | 16 | 16 |
| 下模组实际可见箱数 | 31 | 31 | 31 |

这是整图像素统计，不是物理照度。最终上/下图 RGB 第 99 百分位分别为 228/227。
历史到本轮同时改变了顶板、上相机高度和输送带外观，不能把可见性差异独立归因于某一项。
实际遮挡、裁切和阴影全部保留；可见箱数也不等于可执行抓取箱数。
逐箱像素数量见 [可见性记录](evidence/roof-mast-lit-20260916/visibility_brightness.json)。

## 既有流程的一次性感知结果

最终 [汇总](evidence/roof-mast-lit-20260916/perception_summary.json)记录两模组各 1 次 SAM、
各 1 次 RGB-D 几何处理，均完成，无技术失败。**输入含 GT/oracle proposal，不能称为自动检测。**
模型与权重身份随汇总归档，沿用同一提示方式和处理阈值。

| 最终补光结果 | 上模组 | 下模组 |
|---|---:|---:|
| 可见 GT 对象 / oracle 提示 / SAM 掩膜 | 16 / 16 / 16 | 31 / 31 / 31 |
| 合并预测 / 拆分或重复 GT | 0 / 0 | 0 / 0 |
| 无显著预测重叠的可见 GT | 0 | 0 |
| 最佳 IoU < 0.5 的 GT | 1 | 1 |
| 实际渲染 mask 边界 2 px 精度 / 召回 | 0.5129 / 0.7434 | 0.6010 / 0.8226 |
| 几何输出观测面数 | 23 | 48 |
| 没有可接受面的实例 | 0 | 1 |
| 被接受的完整 cuboid | 0 | 0 |

查看 [上 SAM 边界](evidence/roof-mast-lit-20260916/module_0_upper/sam_boundaries.png)、
[下 SAM 边界](evidence/roof-mast-lit-20260916/module_1_lower/sam_boundaries.png)，
以及 [上几何面](evidence/roof-mast-lit-20260916/module_0_upper/final_metric_faces_overlay.png)、
[下几何面](evidence/roof-mast-lit-20260916/module_1_lower/final_metric_faces_overlay.png)。
GT 对照图单独以 `rendered_gt_boundaries_evaluation_only.png` 命名，仅用于事后评估。

目视检查中，大部分 SAM 外轮廓沿真实箱间缝隙和箱体外缘走，宽胶带没有普遍变成额外箱界。
但印刷标签处仍有小闭合内轮廓：例如上图顶排从左数第二箱，以及下图部分标签位置。
它们表现为实例掩膜内部的小孔/边界伪影，不能因“拆分计数为 0”就忽略。
折皱箱的 SAM 外缘能跟随不规则形状，但下图倒数第二排中间折皱箱的几何面输出明显碎片化；
部分普通箱也出现窄条和内缩的面边界。绿色几何面图不是精确箱界通过图。

上模组低 IoU 对象是 `carton_l04_c03`，仅 437 个可见 GT 像素，IoU≈0.206；
下模组是 `carton_l00_c03`，仅 197 像素，IoU≈0.289。
下模组对应 mask 1 的几何输出明确拒绝：670 个选中像素中，320 个被边缘过滤、
350 个被深度不连续过滤，最终没有保留支持点。
完整 cuboid 的 `accepted=false` 均保留，47 个实例的完整可观测性均为
`UNRESOLVED_PHYSICAL_BOUNDARIES`；观测到若干面不等于完整箱体或抓取可执行性通过。

合并/拆分判据为交集至少 50 像素，且占两个区域各至少 10%；这些计数不是零像素误差证明。
边界统计针对实际渲染对象 mask 的轮廓，包含遮挡和图像裁切边缘，并非全部物理箱缝。
沿用工具输出中的 `gt_visible_physical_edge_pixels` 字段名不改变这一评估口径。

首轮暗图结果在 [独立汇总](evidence/roof-mast-20260916/perception_summary.json)：
上模组无合并/拆分，1 个低 IoU；下模组各 1 个合并和拆分、2 个低 IoU。
暗图边界 2 px 精度/召回为上 0.0620/0.0825、下 0.1062/0.1247。
保留失败表现；本轮未通过调阈值或重复模型运行消除它们。

## 复现与数据位置

以下命令在 GPU 服务器仓库根目录执行。`RUN` 应使用新的空目录；最后的单次感知入口
会拒绝覆盖已有 `perception-once`，避免无记录重跑。全流程未运行长视频、104 任务扫描、
整机卸货仿真或 ROS 验收。bundle 中的其他场景仅生成描述，未执行采集。

```bash
CPU=/root/autodl-tmp/v05-acceptance/cpu/venv/bin/python
ISAAC=/root/autodl-tmp/envs/isaacsim-clean/bin/python
ML=/root/v05-gpu-venv/bin/python
RUN=/root/autodl-tmp/v05-acceptance/roof-mast-reproduction
FEASIBILITY=/root/autodl-tmp/m710-official-dynamics-20260910/repo-feasibility-core-648e177
USD=/root/autodl-tmp/v05-acceptance/dual-rgbd-v4-221b296/usd
CARTONS=/root/autodl-tmp/v05-assets/cartons/mirror
CONVEYORS=/root/autodl-tmp/v05-assets/workcell/mirror

$CPU -m pytest -q
$ISAAC tools/fetch_carton_assets.py --config configs/isaac/carton_assets.json --cache "$CARTONS"
$ISAAC tools/fetch_carton_assets.py --config configs/isaac/conveyor_assets.json --cache "$CONVEYORS"
$CPU tools/build_isaac_perception_bundle.py --output "$RUN/bundle" --run-id roof-mast-lit
$ISAAC scripts/isaacsim_perception_capture.py \
  --project-root "$PWD" --feasibility-root "$FEASIBILITY" \
  --bundle-directory "$RUN/bundle" --usd-directory "$USD" \
  --output "$RUN/capture" --carton-assets configs/isaac/carton_assets.json \
  --asset-cache "$CARTONS" --workcell-asset-cache "$CONVEYORS"
$ISAAC tools/verify_workcell_capture.py --capture "$RUN/capture" \
  --historical-manifest /root/autodl-tmp/v05-acceptance/carton-assets-20260915/capture-08748ff/manifest.json
$ISAAC tools/verify_carton_capture.py --capture "$RUN/capture" \
  --source-manifest "$RUN/bundle/FULL_STACK_NOMINAL.manifest.json"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 $ML tools/run_workcell_perception_once.py \
  --capture "$RUN/capture" --vision /root/vision-fixed \
  --models /root/autodl-tmp/v05-acceptance/model-manifest.json
```

既有官方机器人 USD 缓存来自所列 feasibility 只读版本；纸箱与 A06 缓存由项目的
`fetch_carton_assets.py` 按配置中来源及依赖哈希校验。官方 USD、纹理和模型权重不进入普通 Git。
最终采集服务器目录为 `/root/autodl-tmp/v05-acceptance/roof-mast-20260916/capture-f24f63a`；
暗图目录为同级 `capture-acc6b8c`。

最终 [manifest](evidence/roof-mast-lit-20260916/manifest.json)、
[TF](evidence/roof-mast-lit-20260916/tf_at_capture.json)、
[新快照](evidence/roof-mast-lit-20260916/effective_scene_snapshot.json)、
[新合同](evidence/roof-mast-lit-20260916/effective_isaac_contract.json)、
[环境 handoff](evidence/roof-mast-lit-20260916/environment_handoff.json)和各模组 camera_info 一并归档。
原始 float32 NPY 深度（光轴 Z，单位 m）与同组 RGB 的压缩包留在服务器，
并下载到本地 `.audit/roof-mast-lit-rgbd-depth.tar.gz`；
路径、大小和 SHA256 见 [RGB-D 数据包记录](evidence/roof-mast-lit-20260916/rgbd_archive.json)。
完整捕获 USD、实例掩膜和深度文件的大文件哈希见
[large_artifacts.json](evidence/roof-mast-lit-20260916/large_artifacts.json)。
逐像素的几何筛选审计文件体积较大，仅保留在服务器，路径与哈希见
[large_perception_artifacts.json](evidence/roof-mast-lit-20260916/large_perception_artifacts.json)。
