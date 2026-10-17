# 双面 / 三面有限边界复算 — 2026-10-17

**已定位并修复错误的边界接受；本轮没有证明真实完整体恢复精度达标。** 使用原豁口的冻结 RGB-D 和 SAM 数组重新运行生产几何。双面 mask 2 的旧错误接受被阻断；三面 mask 6 的原来未找到的后沿已在原始深度中定位，但实际圆角不支持理想尖角长方体的共同棱线，最终保留三个可靠面。没有用 GT 尺寸补偿，也没有把全部拒绝解释为恢复成功。

入口：[轻量首页](../../outputs/physical-bounds-20261017/review-final/index.html)、[逐实例回归](../../outputs/physical-bounds-20261017/review-final/regression.csv)、[六边界完整表](../../outputs/physical-bounds-20261017/review-final/six_boundaries.csv)。完整结果与原始运行身份见 `outputs/physical-bounds-20261017/`；Git 中保留[精简证据](evidence/physical-bounds-20261017/summary.json)。

## 输入、代码与运行边界

开始时 HEAD 和 fetch 后远端均为 `50a9701e19a24ef602735a0e022edfae49eee4ca`，分支 `feat/v0.5-perception-ros2`，工作区干净。读取根 AGENTS.md，没有回退、合并其他分支或覆盖原历史包。

沿用 `/root/autodl-tmp/v05-acceptance/notch-multiface-20261016` 的采集和冻结产物。本轮没有运行 Isaac、DINO 或 SAM，没有移动箱子、相机、rig，也没有改文本、模型、阈值、分辨率、提示扰动、去重或编码复用。对应 runtime/config 文件对基线 diff 为空，五项 SAM artifact 在全部重放中 SHA-256 一致。frame602 不在清单、页面或统计中。

两个代表都绑定到 `carton-assets-capture-design1c:module_1_lower:100`：

| 对象 | 冻结 mask_id | proposal_id | 仅事后评价的关联 |
|---|---:|---:|---|
| 双面 | 2 | 2 | carton_l05_c01 |
| 三面 | 6 | 7 | carton_l03_c02 |

RGB、深度、SAM JSON/NPZ 的实际 hash、bbox 和 capture/module/frame 元数据保存在 `execution-audit.json`。这些 ID 仅用于这份冻结输入，不是跨采集身份。来源歧义字段原样保留。

实际运行链没有替代拟合器：`replay_notch_geometry` → `dispatch_geometry` → `_run_secondary_module` → `run_metric_depth` → `extract_observation_labels` → `fit_metric_faces` → `validate_metric_record`。完整旧采集加载器仍核验 GT 归档；拟合器只接收深度、mask、K、初始标签、算法配置和来源歧义，不接收 GT 几何或对象 ID。实际 mesh 在预测完成后的评审构建器中加载。

本轮新建 `baseline/`，在未修改的 50a9701 源码上实际提面、拟面和求解。四组共 74 个完整几何记录与上轮结果逐项相等，见 `baseline-reproduction.json`。之后保存了边界定位中间结果 `finite-v1/`、共同边检查结果及最终 `delivery-geometry/`，均来自同一冻结输入的实际生产重放。中间错误接受未被隐藏。最终 `geometry_run_identity.json` 记录实际代码文件 hash、配置和冻结输入清单 hash。

交付核验将实际运行源码快照与本地源码逐文件比较：本轮求解器文件逐字节相同，部分未修改文件仅 CRLF/LF 不同，统一换行后全部一致；原始 hash 不做重写，差异列于 `local-source-check.json`。

现有环境：NumPy 1.26.4、SciPy 1.14.1、OpenCV 4.10.0、Torch 2.7.1+cu128、Transformers 4.53.2。没有升级依赖；几何与测试禁用 CUDA。吸附、接触、融合、ROS、规划、执行均 **NOT_RUN**。

## 双面：误差来自哪些端

两个平面没有重调：最大法向误差约 0.00204°，位置误差低于 0.005 mm；前面深度残差均值/P95 为 0.141/0.269 mm。原完整体中心误差 16.19 mm、最大尺寸误差 28.63 mm、最大角点误差 32.66 mm，均复现。

统一采用行轴：`q = camera_xyz @ axes.T`，逆变换 `camera_xyz = q @ axes`。下表误差为预测减实际 mesh 参考，单位 mm。`low/high` 是 q 轴方向，不能直接当作世界坐标的最小/最大端。

| q 轴 / 世界方向 / 端 | 旧优化后有符号误差 | 本轮轮廓投影误差，仅诊断 | 正式边界 |
|---|---:|---:|---|
| 0 / −X / low（箱后端） | +28.641 | +10.927 | 圆角下未确定 |
| 0 / −X / high（前平面） | −0.001 | — | 实测平面 |
| 1 / +Y / low（远侧端） | +15.095 | +11.155 | 圆角下未确定 |
| 1 / +Y / high（可见侧平面） | 约 0 | — | 实测平面 |
| 2 / −Z / low（顶端） | +4.338 | +1.705 | 圆角下未确定 |
| 2 / −Z / high（底端） | −3.959 | 无可靠物理过渡 | 未确定 |

完整表另列原始有效点范围、稳定拟面采样范围、冻结标签范围、分位数初值、旧优化后值、正式值、诊断投影、参考值、来源和区间。点范围仅提供包含关系，不直接成为尺寸。

定位结果：

- 冻结标签/稳定支持向内收缩。旧 silhouette 优化对齐的是这些标签轮廓，不是原始深度的有限物理轮廓。
- 旧自由参数搜索限幅为 `±6 × median(Z) / min(fx,fy)`，本例约 **±10.120 mm**；后端从分位数初值到 mesh 参考相差约 30.86 mm。对侧面掠射方向，统一的 Z/f 近似也不能正确表示箱体切向坐标对像素的变化。新方法在原始像素中定位过渡，再通过固定 K/平面还原，没有统一加毫米或像素。
- 旧边界检查在候选边附近看到较远深度就支持候选，却没有定位过渡本身。底部缝隙是缓慢离开、再接近前平面的深度变化，不能据此确定本箱底面。
- 本例理论共同边的实测双平面距离均值/P95 为 **6.378/6.806 mm**，超过未修改的 3/6 mm 限值。附近存在两侧点不等于它们到达理论交线。

最终没有正式长宽高、中心、姿态或八角点。诊断候选的最大角点误差降为 **16.10 mm**，仍超过 15 mm；而且底端仍借用了旧初始化值，仅可通过诊断开关查看，不能作为恢复结果。

## 三面：原拒绝边并非没有被观察到

原图评价可见三个面，算法仍提取三个可靠平面。法向误差最高 0.00517°、位置误差最高 0.0122 mm；顶面深度均值/P95 约 0.509/1.016 mm，其余面更小。最终平面和冻结支持与基线完全相等。

| q 轴 / 世界方向 / 端 | 旧优化后有符号误差 mm | 本轮轮廓投影误差 mm，仅诊断 | 正式来源/缺失 |
|---|---:|---:|---|
| 0 / −X / low（后端） | +39.963 | +9.151 | 对侧面位置未确定 |
| 0 / −X / high（前面） | −0.013 | — | 实测平面 |
| 1 / +Z / low（底端） | +13.686 | +14.898 | 底面位置未确定 |
| 1 / +Z / high（顶面） | −0.017 | — | 实测平面 |
| 2 / +Y / low（远侧端） | +14.371 | +9.956 | 对侧面位置未确定 |
| 2 / +Y / high（可见侧面） | −0.006 | — | 实测平面 |

逐边图位于评审页的“物理边与两侧原始支持”，静态图在 `stack_notch/module_1_lower/mask-6-edges/`，包含修改前全部候选、每条轮廓搜索、三个共同边、两侧实际支持点和搜索范围。

1. 基线拒绝的是顶面 label 1 / edge 3，即后沿。候选偏内约 40 mm，15 个旧探针均仍处于连续面内。原深度实际有约 1.95 m 到 4.91 m 的过渡；这条边不是完全不可见。新的有限搜索找到 15/15 轮廓过渡。旧优化限幅约 ±11.777 mm，无法从原分位数初值走到该观测。
2. 前面底沿没有可靠外轮廓跳变；侧面底沿提供了另一个可定位的过渡。因此不再要求每个可见面的每条边都直接可见，也不要求十二条棱边全被看到。
3. 仅做这项修复的 `finite-v1` 得到一个内部接受的三面候选：中心 10.05 mm、最大尺寸 14.85 mm、姿态 0.00518°，但最大角点 **20.10 mm**，外部精度失败。
4. 原始深度进一步否定尖角共同边：三个交线的实测双平面距离均值/P95 分别为 **8.689/8.690、7.826/8.448、6.987/7.563 mm**。每条附近都有 15/15 两侧点，却不满足同一真实交线的残差。这是圆角表面，不是缺一个深度跳变。
5. 最终保留三个平面；后端、底端和远侧端的轮廓投影只存为诊断证据，正式值和正式区间为空。没有把六个可写出的数等同于六个已确定的实体边界。

实际 mesh 审计解释了模型偏差：该资产的平面部分距整体网格包围边界，在 X/Y/Z 方向分别内缩约 **11.053 / 11.200 / 14.118 mm**。这是预测完成后对本次组成网格的测量，不是配置尺寸；它没有被用于补偿或重定参考。参考仍是原实际网格包围几何，所有未达标记录保留。

在不引入尺寸或圆角先验的本轮路线下，至少需要直接观测当前缺失的对侧面/底面，或能独立约束这些端位置的真实边界。仅扩大搜索半径、把平面交线延长或给所有边统一补数不足以解决问题。新采集与跨视角关联不在本轮执行范围。

## 修改范围与回归

- `physical_bounds.py`：独立的原始深度边界证据，限于 SAM 附近并要求连接到冻结面支持；区分外轮廓、遮挡、裁切、孔洞、mask/标签截断和未判定。保持平面拟合/验证支持不变。
- `metric_faces.py`：旧标签轮廓优化只作初值/诊断；六端分别记录实测平面或有限轮廓约束。共同边增加实测交线检查，沿用 3/6 mm 门限。非尖角模型下的轮廓投影及区间不能成为正式尺寸。完整体最终验证重新从原深度定位边界，可拒绝“平面正确、切向尺寸错误”的盒子。
- `replay_notch_geometry.py`：增加实际代码/配置身份与固定坏采集排除检查，仍调用原生产 dispatcher。
- `build_notch_geometry_review.py`：验收值改为实际读取共享配置；采集目录使用冻结输入清单，不依赖旧脚本 cwd。GT 绑定继续核验。
- `build_physical_bounds_review.py`、既有 HTML 和小型 JS 扩展：双代表首页、全部实例、六端同轴误差、逐边图、等米制可旋转视图；GT 和拒绝候选默认关闭。

没有修改提面、稳健平面拟合、多面正交拟合、SAM、DINO、机器人、碰撞或规划算法。全部 74 个最终面记录、冻结标签及输入绑定逐项不变。

| 范围 | 预测实例 | 多面 | 单面跳过 | 最终完整体 |
|---|---:|---:|---:|---:|
| frame32 上/下 | 45 | 7 | 38 | 0 |
| 豁口上/下 | 29 | 11 | 18 | 0 |
| 合计 | 74 | 18 | 56 | 0 |

18 个多面中，17 个边界/模型约束不足，1 个保留来源歧义阻断。全部预测中有 4 个来源歧义标记（另 3 个为单面），10 个豁口预测有未匹配的预期可见面；这些标记允许重叠。frame32 没有本次实际组成 mesh，mesh 精度仍 **NOT_RUN**。已有 mask 匹配、未匹配 GT、无 mask proposal 都保留，不能将其从分母剔除。

最终准确完整恢复 0、错误接受 0。前者没有得到真实正向证明；后者来自保守阻断，不能当作完整体拟合已正确。所有 74 个对象仍在 cargo 和 unknown 中，`absence_means_free_space=false`，没有残留正式盒字段。

## 测试、页面与复跑

本轮修改前完整 CPU 测试：**1066 passed / 3 failed / 12 skipped / 1 deselected**。修改后：**1067 passed / 3 failed / 19 skipped / 1 deselected**。原有失败仍为 `test_secondary_timing_preserves_legacy_interval_and_adds_preprocessing[False/None]` 和 Windows GBK 下的 `test_derived_scene_preserves_parent_cartons_and_fixed_layout`；没有删除断言。

服务器禁用 CUDA 的针对性测试 **24 passed / 1 deselected**。排除的场景删除测试由本地完整套件覆盖；隔离源码包未带该场景的大型 fixture。新增测试覆盖不同尺寸、旋转、密度和噪声的充分双/三面、局部三面、遮挡、mask 截断、正确平面但错误切向尺寸、圆角理论交线误接受及配置读取；沿用单面跳过和最终降级清字段测试。三个充分约束合成样例实际通过配置中的中心/尺寸/角点阈值，不是只断言 accepted。旧 50a9701 对圆角交线合成回归确实错误接受 15/15，修复后拒绝。合成结果不代表真实模型精度。

本地缺 SciPy/OpenCV 的新增 7 项测试跳过，已经在服务器已有依赖上执行，没有下载模型或安装/升级依赖。页面脚本语法及本地资源检查另行记录；当前浏览器工具未提供可用会话，实际旋转/滚动交互 **NOT_RUN**。静态逐边图基于原始 RGB 原生像素生成，仅用于展示，没有改推理输入。

最终页面检查覆盖 74 个实例的 1,216 条绘图路径（DOM/canvas 替身），所有 HTML 本地引用存在；代表实例逐边 PNG 已目视检查。替身检查不代表真实浏览器布局或交互验收，详见 `viewer-check.json`。

从仓库根目录复跑，输出必须为新目录：

```sh
PY=/root/v05-gpu-venv/bin/python
OLD=/root/autodl-tmp/v05-acceptance/notch-multiface-20261016
OUT=/root/autodl-tmp/v05-acceptance/physical-bounds-rerun
export CUDA_VISIBLE_DEVICES=''
export OPENBLAS_NUM_THREADS=1

# 在独立 worktree 运行未修改基线，保留当前工作区。
mkdir -p "$OUT"
git worktree add --detach "$OUT/baseline-code" 50a9701e19a24ef602735a0e022edfae49eee4ca
$PY "$OUT/baseline-code/tools/replay_notch_geometry.py" --baseline "$OLD/baseline" \
  --output "$OUT/before" --vision /root/vision-fixed

# 当前代码重新求解；不加载或下载检测/SAM 模型。
$PY tools/replay_notch_geometry.py --baseline "$OLD/baseline" \
  --output "$OUT/after" --vision /root/vision-fixed

# before 必须来自未修改 50a9701 的实际重放；不可把旧盒 JSON 当新基线。
$PY tools/build_physical_bounds_review.py --frozen "$OLD/baseline" \
  --before "$OUT/before" \
  --after "$OUT/after" --visibility "$OLD/visibility/visibility.json" \
  --output "$OUT/review"

# 使用有 SciPy/OpenCV 的既有 CPU 环境；不初始化 GPU。
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/root/autodl-tmp/v05-acceptance/notch-multiface-20261016/test-tools \
$PY -m pytest -q tests/test_physical_bounds.py tests/test_notch_geometry.py \
  tests/test_metric_patch_adapter.py -k 'not notch_removes'
```

服务器交付根目录为 `/root/autodl-tmp/v05-acceptance/physical-bounds-20261017`。完整包包含未修改基线、中间候选、最终生产几何、全部页面/逐边图、源代码快照、输入身份和检查记录；模型权重不在包中，也未提交 Git。

完整包 `delivery-final.tar.gz` 为 201,014,675 字节，SHA-256：`37807370cf960e00887e7e31f0893745febd41a69e5e0e8e3536ea02a7ba6722`。它复用上一轮已交付的冻结采集和模型产物；独立复跑仍需原 `notch-multiface-20261016` 输入目录，不能仅凭本轮几何 JSON 重新拟合。
