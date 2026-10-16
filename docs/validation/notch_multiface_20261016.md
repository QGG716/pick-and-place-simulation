# 箱堆豁口与多面几何验证 — 2026-10-16

**真实采集与冻结 DINO/SAM 主链接通，单面门禁完成；完整箱体的实际精度尚未达标。** 新场景确实出现双面、三面目标。74 个预测实例全部保留：56 个单面不进入完整体求解，16 个多面边界不足，1 个多面来源歧义被阻断，1 个双面完整体通过深度自检但未通过实际网格精度阈值。不能把退出码 0 或深度自检通过解释为几何质量通过。

入口：[本地轻量首页](../../outputs/notch-multiface-20261016/review-final/index.html)、[全部实例表](../../outputs/notch-multiface-20261016/review-final/all_instances.csv)、[完整结果包](../../outputs/notch-multiface-20261016/delivery-results.tar.gz)。Git 内保存[精简证据](evidence/notch-multiface-20261016/delivery-summary.json)。服务器目录 `/root/autodl-tmp/v05-acceptance/notch-multiface-20261016`；完整包 SHA-256 为 `d9357b064602964cac7256f73bca1763fa545ace74365e53e447cc07b120c4b1`，约 195 MiB，不含权重。完整包有原始 RGB/深度/标定/绑定/GT、实际网格、全部 detector/SAM 结果、初始标签、几何修改前后结果及评审页；大型 USD 和派生点云仍在服务器原采集目录。

## 工作区与冻结边界

开始时工作区干净，本地和 fetch 后远端均为 `feat/v0.5-perception-ros2@ceda56f80758ec16d8b6f1fe1fe6cd1bfa2f0c12`；提交前再次 fetch，远端未变化。没有回退、覆盖他人修改、合并规划分支或改写历史。读取根目录 AGENTS.md，几何核心仍无 Isaac/ROS/GPU 导入。

使用新提供服务器的 RTX 4080（驱动报告 32760 MiB），Isaac Sim 6.0.1.0，现有 `/root/v05-gpu-venv`：Torch 2.7.1+cu128、Transformers 4.53.2、OpenCV 4.10.0。没有安装或升级模型依赖。CPU 测试仅将本机已有 pytest 的纯 Python 文件复制到隔离 `test-tools`，禁用 CUDA 和 pytest 外部插件；未改 GPU 环境。

- Grounding DINO：`IDEA-Research/grounding-dino-base@12bdfa3120f3e7ec7b434d90674b3396eccf88eb`。
- SAM：`facebook/sam-vit-base@70c1a07f894ebb5b307fd9eaaee97b9dfc16068f`；原 upstream `1d208f2ed380a207e6e46b4a62d2ac640edfe477`。
- `cardboard box.`、0.25/0.25 阈值、0.9 近重复 IoU、800/1333 检测缩放、300 上限、seed 17 不变；SAM 2592×1944、五组扰动、选择/后处理/去重/编码复用不变。
- 关键 runtime、SAM session、proposal 代码及两份生效配置对基线 diff 均为空，实际文件 hash、完整模型 manifest 写入 `baseline/frozen_plan.json`。
- SAM 嵌套去重任务仍暂停。没有清除 `supporting_proposal_ids`，没有 ORACLE/旧 mask/人工框补救，没有 MoGe、尺寸先验、跨模组融合。

当前评审清单固定为 frame32 和 STACK_NOTCH_MULTIFACE。`review_selection.py` 按 `sensor_epoch=carton-assets-recording-01`、frame_sequence=602、两个 module 身份排除原坏采集；并非删除所有数值 602。新批次在推理前、页面组装前均检查；新首页和统计无该输入。旧采集、旧报告和显式历史工具原样保留。

## 真实场景与采集

从实际坐标确认 5 列 Y={−0.84,−0.42,0,0.42,0.84}、8 层向 +Z 堆叠；不是根据 ID 猜上下方向。独立配置 [`stack_notch_multiface.json`](../../configs/isaac/stack_notch_multiface.json) 从中间三列顶部依次移除 2、4、5 个对象，保留 29 个。

移除：`l06_c01,l07_c01`；`l04_c02,l05_c02,l06_c02,l07_c02`；`l03_c03,l04_c03,l05_c03,l06_c03,l07_c03`（均带 `carton_` 前缀）。完整保留/移除记录、实际 Z 坐标、父场景指纹见 `bundle/notch_scene_audit.json`。保留对象字典与原始场景逐项相等；从 snapshot 和 collision primitives 同时移除实体。未转动、缩放、平移剩余箱子，未制造悬空层。原 USD 材质/资产选择不变。

沿用正常 nominal rig，两个模组相对外参不变。新的身份为 `carton-assets-capture-design1c:{module}:100`，没有冒用旧 frame32 身份；不宣称机械臂可达性验收。`capture-design1`、`design1b` 分别因输送机/纸箱缓存目录少了 `mirror` 而在依赖检查阶段停止；修正路径后 `design1c` 成功，场景和视角设计没有改动。

先查看两个模组原始 RGB/深度，确认台阶和箱墙；再用实际组成后的 USD 网格顶点/三角形、世界变换和渲染深度检查可见面。采用原有 50 点、15 mm 短边、0.0003 m² 面积、6 mm 表面距离及局部法向一致性；避免把相邻面附近的窄像素带重复算作一面。原图中的暗色车厢背景不被当作无效输入。检查结果先于模型运行保存于 `visibility/visibility.json`。

三面代表：上模组 `carton_l05_c01`，下模组 `carton_l03_c02`。双面代表：下模组 `carton_l05_c01`。周边单面对象仍保留。全景见首页和 `capture-design1c/overview.png`；它是另一个真实观察相机，不是推理输入。

## 实际调用关系与代码变化

`build_isaac_perception_bundle --notch-config` → `derive_effective_scene` → `derive_notch_scene` → 原 `isaacsim_perception_capture --scene STACK_NOTCH_MULTIFACE`。新增 `rendered_surface_truth.py` 仅导出事后评价用实际 mesh。

`run_notch_geometry_review` → `run_workcell_perception_once --stop-after geometry --capture-scene …` → 原 `RuntimeCache/ResidentRuntime` → DINO → 原 SAM → `dispatch_geometry` → `_run_secondary_module` → `metric_depth_runner.run_metric_depth` → 原平面提取 → `fit_metric_faces` → `validate_metric_record` → observation。两组四张图真实串行执行，常驻模型只加载一次。完整历史加载器仍验证 GT 归档完整性；GT 不传入检测、提面或完整体求解。

先在基线几何代码上生成 `baseline/`，随后 `replay_notch_geometry.py` 通过同一个生产 `dispatch_geometry` 在精确冻结的 SAM NPZ/JSON 上生成 `final/`，逐文件 hash 不变。没有重跑/挑选模型输出。主入口、常驻入口只增加 scene 参数转发；`--stop-after geometry` 跳过算法融合交付，吸附、接触、融合、ROS、规划、执行全部 NOT_RUN。

几何修改范围：

1. `metric_faces.py` 在完整体求解前，以质量合格面片的法向与双向平面距离做完整链接共面分组，保留 support label / initial plane 来源。共面连通碎片计一个可靠面；单面明确 `NOT_ATTEMPTED_SINGLE_FACE`。不同实例不会合组；来源歧义继续阻断。
2. 最终验证若失去多面支持，撤销完整体并清除正式 `corners_3d/2d, center_3d, shape_dimensions, orthogonal_axes_3d, completion_mode`；旧候选只能进入独立的拒绝诊断字段。观测面片及 unknown 继续保留。
3. 共同边新增两侧实际支持检查：15 个沿边位置中至少 10 个，同时在两侧冻结标签中找到 6 像素范围内、满足原 6 mm 平面残差且接近同一有限 3D 线段的点。遮挡、裁切、冲突不被覆盖；无两侧支持的理论交线失败。原 3 mm 均值、6 mm P95、物理边界及 held-out 检查没有放宽。
4. `observed_faces.py` 用真实可靠平面数表达单面；`run_isaac_rgbd_geometry.py` 将有可靠多面但体积未定的状态明确为 UNKNOWN，保留对象和阻断原因。

共同边的前后证据：豁口上模组 mask 1/6 的同一共享边原来 15/15 深度探针均为连续/未知。合成连续正交棱边证明旧深度跳变规则会遗漏真实共同边；新检查能验证实际两侧支持，并拒绝只有无限平面交线的情况。然而本批真实候选的冻结支持域未能到达候选棱边，新增检查仍拒绝。全部 38 条有方向的共享边诊断记录中，新增判定没有让任何真实实例变为成功；完整体数量修改前后都是 1。没有为提高成功数扩大搜索范围。

## 全部结果与代表实例

| 输入 | 检测 | SAM | 单面跳过 | 多面边界不足 | 多面歧义阻断 | 深度通过完整体 | 实际网格精度通过 |
|---|---:|---:|---:|---:|---:|---:|---:|
| frame32 上 | 16 | 15 | 15 | 0 | 0 | 0 | 0 |
| frame32 下 | 31 | 30 | 23 | 7 | 0 | 0 | 0 |
| 豁口上 | 8 | 7 | 4 | 2 | 1 | 0 | 0 |
| 豁口下 | 23 | 22 | 14 | 7 | 0 | 1 | 0 |
| 合计 | 78 | 74 | 56 | 16 | 1 | 1 | 0 |

评价采用预测冻结后最大总 mask IoU 的一对一分配，IoU≥0.5 才匹配。全部 74 个预测、未匹配 GT 和无 mask 检测都保留；没有多个预测占用同一个 GT。GT 不回写推理身份。平面匹配也是一对一，片段冗余单列。frame32 没有本次导出的完整原组成 mesh，故其 mesh 几何精度标 NOT_RUN；其 mask 评价和深度验证均运行，不拿名义尺寸代替真实 mesh。

- **单面**：豁口下 mask 1，仅一个可靠面，保留预测及 unknown；其 proposals 1/20 来源合并信息仍在，未清空。
- **双面**：豁口下 mask 2 / proposal 2，对应 `carton_l05_c01`，mask IoU 0.9879。两个平面方向误差约 0.00204°/0.000029°，位置误差低于 0.005 mm。深度自检接受完整体，但实际 mesh 包围几何的中心误差 **16.19 mm > 15 mm**，最大尺寸误差 **28.63 mm > 20 mm**，最大角点误差 **32.66 mm > 15 mm**；姿态误差约 0.00204°。**完整体质量未达标**，页面标红，GT 结果不用于反调求解器。
- **三面且歧义**：豁口上 mask 2，proposals 2/8，对应 `carton_l05_c01`；14 个初始连通片 → 4 个最终片段 → 3 个不同平面。三面误差均通过原平面精度阈值，但 `supporting_proposal_ids` 非空，完整体被阻断。
- **三面且边界不足**：豁口下 mask 6 / proposal 7，对应 `carton_l03_c02`；4 个初始片 → 3 个可靠平面，mask IoU 0.9694。法向误差最高约 0.00517°，平面位置误差最高约 0.0122 mm，仍因边界不足不恢复完整体。
- **可见面遗漏**：豁口下 mask 15、20 的评价各有三个可见面，但最终只提取两个可靠面，分别遗漏侧面、顶面。不能称多面恢复验证通过。扫描纸箱还存在表面非理想平面、可靠面片偏小问题。
- 未匹配 GT：frame32 上/下各 1，豁口上 3、下 1；包含画面边缘只露很窄区域的标注。它们保留在统计和页面中，不从分母剔除。全景不等于每个模组都看见全部 29 箱。

完整体评价对实际 mesh 顶点的世界包围几何计算，不使用配置尺寸。报告明确理想长方体近似；姿态枚举 24 个正定轴交换/符号等价，角点进行一对一匹配，未用纯二维重投影代替三维精度。

## 页面、测试和限制

`build_notch_geometry_review.py` + 小型离线 HTML 复用已有 `sam_review` 图像/轮廓与真实 stage trace。可逐实例看原始 RGB/深度、黄框/绿轮廓、初始支持点、最终面与跳过原因；3D 按相机光学坐标等米制缩放，可旋转；只画可靠面或深度验证接受的完整体。GT 默认关闭，拒绝候选为独立开关。五组原提示扰动仍由原 SAM 页面提供。

- 修改前完整 `pytest -q`：1059 passed / 3 failed / 6 skipped / 1 deselected。
- 最终完整 `pytest -q`：1066 passed / 3 failed / 12 skipped / 1 deselected。
- 三项前后相同：`test_depth_components_single_pass` 两个 legacy timing 断言，`test_effective_scene` Windows 默认 GBK 解码。未删除失败断言或修改它们。
- 相关接口/来源/场景/门禁组合测试：88 passed / 7 skipped。
- 服务器 CPU（CUDA 禁用）几何与适配测试：17 passed。覆盖单面求解前阻断、共面碎片、充分双/三面、遮挡边界、不足三面、最终降级清字段、来源歧义、共同边实际支持/无支持、scene 删除一致性和当前输入排除。
- 本地缺 SciPy/OpenCV 的 6 个新几何测试在完整 CPU 基线中跳过，已在既有服务器依赖上实际执行；替身只用于入口接口测试，不作为模型精度证据。
- 原始两模组 RGB/深度、SAM 原生图、代表实例对照、最终面片图已目视检查；JS 语法及全部页面本地资源/实例数据检查通过。当前工具无可用浏览器会话，真实浏览器旋转/滚动交互验收 **NOT_RUN**。
- 吸附候选、运动规划、机械臂执行、大型 ROS、跨相机融合、性能优化全部 **NOT_RUN**。本轮不宣称完整几何生产验收通过，也不宣称泛化精度。

## 复跑

以下从仓库根目录执行，所有输出目录必须是新目录；沿用已准备模型，不下载或升级依赖。若没有模型，沿用上一轮明确的 `tools/prepare_vision_models.py --add-grounding-dino` 准备命令，本轮未重新准备模型。

```sh
PY=/root/v05-gpu-venv/bin/python
ISAAC=/root/autodl-tmp/envs/isaacsim-clean/bin/python
VISION=/root/vision-fixed
MODELS=/root/autodl-tmp/v05-acceptance/dino-sam-20261015/models.json
OUT=/root/autodl-tmp/v05-acceptance/notch-rerun
OLD=/root/autodl-tmp/v05-acceptance/video-demo-20260920/recording-01/frames/000000

$PY tools/build_isaac_perception_bundle.py --output "$OUT/bundle" \
  --notch-config configs/isaac/stack_notch_multiface.json --run-id notch-rerun
OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1 "$ISAAC" scripts/isaacsim_perception_capture.py \
  --bundle-directory "$OUT/bundle" --project-root "$PWD" \
  --feasibility-root /root/autodl-tmp/m710-official-dynamics-20260910/repo-feasibility-core-648e177 \
  --usd-directory "$OUT/usd" --output "$OUT/capture" --scene STACK_NOTCH_MULTIFACE \
  --carton-assets configs/isaac/carton_assets.json \
  --asset-cache /root/autodl-tmp/v05-assets/cartons/mirror \
  --workcell-asset-cache /root/autodl-tmp/v05-assets/workcell/mirror
$PY tools/check_notch_visibility.py --capture "$OUT/capture" --output "$OUT/visibility"
# 查看此次原始 RGB、深度及 visibility.json，再执行冻结模型。
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 $PY tools/run_notch_geometry_review.py \
  --frame32 "$OLD" --notch "$OUT/capture" --visibility "$OUT/visibility/visibility.json" \
  --models "$MODELS" --vision "$VISION" --output "$OUT/current"
OPENBLAS_NUM_THREADS=1 $PY tools/build_notch_geometry_review.py \
  --baseline "$OUT/current" --final "$OUT/current" \
  --visibility "$OUT/visibility/visibility.json" --output "$OUT/review"

# 单次实际主入口，同样停在几何；不触发后续交付/融合。
$PY tools/run_workcell_perception_once.py --capture "$OUT/capture" \
  --capture-scene STACK_NOTCH_MULTIFACE --models "$MODELS" --vision "$VISION" \
  --stop-after geometry --stage-trace --evaluate-gt --no-legacy-cuboid-diagnostic \
  --geometry-backend subprocess --geometry-blas-threads 1 --output-directory "$OUT/once"
# 对已有冻结模型结果仅重放生产几何函数，用于代码前后对照。
$PY tools/replay_notch_geometry.py --baseline "$OUT/current" --output "$OUT/geometry-replay" --vision "$VISION"
```

`--baseline` 在评审构建器中表示冻结模型结果的来源；没有单独旧几何运行时，可与 `--final` 指向同一新运行，不能宣称这样得到了历史前后对照。本次交付的 baseline/final 确实来自两套明确记录的几何代码。
