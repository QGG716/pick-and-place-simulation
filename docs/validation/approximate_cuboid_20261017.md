# 三面纸箱近似长方体 — 2026-10-17

本轮把“观测约束下的近似模型”与“严格尖角完整体”分开。没有把旧 `accepted` 改为 true，没有重跑 DINO/SAM，也没有调整既有几何精度门槛。

入口：[轻量首页](../../outputs/approximate-cuboid-20261017/review/index.html)、[主实例完整对照](../../outputs/approximate-cuboid-20261017/review/primary_result.json)、[六边界表](../../outputs/approximate-cuboid-20261017/review/six_boundaries.csv)。[Git 精简证据](evidence/approximate-cuboid-20261017/README.md)保留实际运行记录；全部文件位于 `outputs/approximate-cuboid-20261017`。

## 冻结输入与实际链路

开始时本地与 fetch 后远端均为 `d75483e0f5d837512b30cd4ad5cc4c6d189769db`，分支 `feat/v0.5-perception-ros2`，工作区干净。读取根 AGENTS.md，没有回退、覆盖修改或合并其他分支。

选择及身份写在 `configs/perception/approximate_cuboid_review.json`：主对象为 `stack_notch / module_1_lower / carton-assets-capture-design1c:module_1_lower:100` 的 mask 6 / proposal 7；mask 2 / proposal 2 仅负向回归。求解器不读取该选择文件，不含 mask 6 或 GT 对象 ID 分支。

复用上一轮原 RGB、注册深度、标定、原始 SAM NPZ 和来源映射。本轮实际执行三路生产重放：

- `strict-baseline/`：未修改 d75483 源码，重新求解严格路径。
- `uncoupled/`：仅解除近似输出对尖角判断的依赖，保留原轮廓投影参数；是本轮新运行，不是重标记历史 finite-v1。
- `final/`：原始深度及轮廓联合估计的近似模型，并通过生产 observation 入口的二次最终验证。

调用仍为 `replay_notch_geometry → dispatch_geometry → _run_secondary_module → run_metric_depth → extract_observation_labels → fit_metric_faces → validate_metric_record`。三路使用相同冻结输入；初始标签、测量平面、支持域和绑定逐实例比较，记录在 `final-invariants.json`。旧采集加载器仍核验归档 GT，求解函数不接收 GT 几何。mesh 只在预测完成后的评审构建器中加载，参考和匹配口径不变。frame602 不进入本轮输入、页面或统计。

## 两处门禁与结果语义

`metric_faces._complete()` 继续计算严格尖角诊断；独立的 `approximate_cuboid.estimate()` 从原始深度重新取得有限轮廓，不读取被严格路径清空后的自由边界值，也不把 `shared_edges.supported` 或其尖角 `connected` 作为近似输出条件。

`validate_metric_record()` 在严格路径验证后独立调用近似验证。它重新读取深度、冻结标签、同一模型策略和边界约束，不调用严格的 `validate_complete_constraints()`，因此不会以 `FINAL_SHARED_EDGE_LOST_ACTUAL_SUPPORT` 单独否决近似模型。修改切向尺寸、丢失面支持、来源歧义等仍会撤销近似结果。

新块是 `approximate_cuboid`，状态 `VALIDATED_OBSERVATION_MODEL` 表示有稳定的观测模型，不表示 GT 精度已通过。中心、行轴姿态、尺寸、八个虚拟角点和六端来源保存在该块；`physical_corners_certified / conservative_collision_envelope / planning_eligible / execution_eligible` 均为 false。原 `accepted`、严格完整体字段及 unknown 语义不复用。模型欠约束或不稳定时不生成占位角点；GT 不达标不会清除已经形成的近似模型。

尖角失败只记录其数值，不自动分类为圆角。替代实例检查要求三个不同的可靠正交面，并检查同一 SAM 实例内两个面支持之间的有限深度过渡：不能穿过 mask 缺口、无效深度或明显遮挡跳变。相邻稳定面之间允许连续过渡带，不要求交线像素同时落在两个平面上。

## 拟合方法及假设

测量方向和三个主平面的位置全部固定。原轮廓射线投影只作初始化与中间对照，不当作精确隐藏面等式。

最终模型使用六个拟合参数：三个尚未观测的端位置，以及三个轴向过渡半径。形状假设是“内矩形体与椭球过渡表面的组合”；输出的是该近似形状的虚拟长方体。半径由实际深度估计，不使用资产名字、名义尺寸、GT 内缩毫米数或固定膨胀量。它是显式模型假设，不是已证明的真实纸箱结构。

目标由两类观测组成：原始深度点到近似表面的米制残差，以及物理轮廓对应相机射线平面上的米制切触残差。两类总权重相等，使用 soft-L1；表面尺度单独为 3 mm，轮廓尺度由原 2 像素尺度和实测深度/焦距换算。稳定平面内部原 3/6 mm 验证不变，且不用于要求过渡带贴合两平面。

每个自由端必须先有实际有限轮廓；缺少任何端不能用优化初值补齐。共三个固定半径初始化和两个确定性采样扰动。按完整观测上的同一目标比较解，目标在最优值 25% 内视为近似等价；等价解的端位置变化超过独立的 6 mm 稳定性阈值时不输出完整模型。优化触及搜索限制也不宣布确定尺寸。该范围反映算法敏感性，不是物理覆盖置信区间，也不证明全局唯一性。

拟合最多确定性采样 12,000 个原始像素；报告残差覆盖原 mask 内全部有效正深度像素，没有按拟合误差删除像素。远离主平面的区域单独统计，但可能包含遮挡或分割误差，不自动认作圆角。30 mm 模型异常阈值及 5% 最大比例独立于稳定平面门槛，全部异常计数保留。

## 实际结果与限制

结果数值以 `primary_result.json` 和 `six_boundaries.csv` 为准；下面的汇总由最终生产结果生成。

| 路径 | 近似模型 | 中心误差 mm | 最大尺寸误差 mm | 最大角点误差 mm | 姿态误差 ° | 原精度门槛 |
|---|---|---:|---:|---:|---:|---|
| d75483 严格路径候选 | 未输出 | 10.05 | 14.85 | 20.10 | 0.00518 | 未通过 |
| 本轮仅解除耦合 | 已生成并通过近似最终验证 | 10.05 | 14.85 | 20.10 | 0.00518 | 未通过 |
| 本轮原始深度与轮廓联合估计 | 已生成并通过近似最终验证 | **5.12** | **9.39** | **10.26** | **0.00518** | **通过，限本开发样本** |

因此，第一步只改变输出语义，未改善误差；第二步实际改变三个自由端，带来本次尺寸与角点精度改善。最终尺寸按行轴 `[-X, +Z, +Y]` 为 **609.39 / 296.86 / 402.59 mm**，不能把该顺序误作世界 XYZ。对应同轴投影参考尺寸误差为 **+9.37 / −3.20 / +2.58 mm**；原指标保留其对称轴匹配定义，所以与上表排序尺寸误差存在小量姿态投影差异。

五组固定初始化/采样的自由端变化范围为 **3.06 / 0.47 / 0.30 mm**，低于既定稳定性阈值。三个主平面没有移动；三个过渡半径估计约 **22.55 / 16.40 / 19.73 mm**，属于模型参数，不能当作实际资产倒角尺寸。

六端沿共同的行轴 q 表示；预测值单位 m，有符号误差为预测减实际网格参考，单位 mm：

| q 方向 / 端 | 仅解除耦合 | 最终预测 | 最终误差 mm | 来源 |
|---|---:|---:|---:|---|
| −X / low 后端 | −1.915817 | −1.934349 | −9.382 | 深度表面 + 轮廓 + 过渡形状假设 |
| −X / high 前面 | −1.324956 | −1.324956 | −0.013 | 原实测主平面 |
| +Z / low 底端 | −0.714588 | −0.726306 | +3.181 | 深度表面 + 轮廓 + 过渡形状假设 |
| +Z / high 顶面 | −0.429443 | −0.429443 | −0.017 | 原实测主平面 |
| +Y / low 远侧端 | −0.940058 | −0.952595 | −2.581 | 深度表面 + 轮廓 + 过渡形状假设 |
| +Y / high 可见侧面 | −0.550008 | −0.550008 | −0.006 | 原实测主平面 |

相机坐标中心为 `[0.751293, −0.014473, 1.729020] m`；完整行轴矩阵、每条轮廓的像素来源及参数稳定范围保存在主实例 JSON，不把虚拟背面位置描述为直接测量。

所有 **104,354** 个有效原 mask 深度像素的模型残差均值/P95 为 **0.456 / 1.125 mm**。但远离主平面的 **1,127** 个像素均值/P95 为 **14.73 / 22.21 mm**，最大 **24.20 mm**；这些误差没有删除，仍显示模型形状与部分观测不符。大于独立 30 mm 模型异常阈值的像素为 0，不代表小残差覆盖所有局部形状。

全部 **74** 个实例的测量平面、冻结标签、支持域和输入绑定未变。只有主对象生成近似模型；mask 2 仍缺约束，来源歧义三面对象仍阻断，单面不恢复。严格完整体仍为 0；所有 74 个对象仍在 unknown，近似模型在 observation 的二次验证后仍可查看，但没有进入完整体 hypothesis 或执行资格。

本轮对象是开发样本，不作泛化结论。近似形状假设、采样不足和 SAM 边界误差仍可能引入系统偏差。无需每条棱线都被测量，不等于获得保守碰撞包络；缺少对隐藏形变、遮挡区域和模型误差的保守界限，执行继续阻断。

## 测试与复跑

未修改基线完整 CPU 测试：1067 passed / 3 failed / 19 skipped / 1 deselected。三个既有失败为两个 secondary timing 参数化用例及 Windows GBK 场景读取用例，未删除断言。

修改后完整测试：**1068 passed / 3 failed / 25 skipped / 1 deselected**。服务器禁用 CUDA 的针对性测试：**31 passed / 1 deselected**，包含本地因缺少 SciPy/OpenCV 而跳过的新增几何用例。排除的场景删除 fixture 测试由本地完整套件覆盖。

9 个 HTML 页的本地资源均通过检查，74 个实例的 1,364 条绘图路径通过 DOM/canvas 替身检查，原图/近似模型及等米制点云图已目视核对。当前没有可用浏览器提供者，真实拖动旋转/滚动交互 **NOT_RUN**；替身检查不代表真实浏览器布局验收。

针对性服务器 CPU 测试覆盖真实椭球过渡体与平面倒角体、严格/近似语义差异、不同尺寸/姿态/像素密度、欠约束三面、两物体误合并、过渡遮挡、错误切向尺寸、最终扁平 K 转换、来源歧义，以及已有单面/双面和尖角正向回归。没有下载模型、启用 GPU 或升级依赖。

初次 refined 运行在 observation 二次验证处因扁平 K 索引失败；失败目录与日志保留，最终代码统一内参形状并补了该真实调用形态的测试。探索中还修正了近似表面距离在内矩形体内部的定义及两类稳健目标的权重一致性，探查记录保留，没有使用 GT 调参。

从仓库根目录，输出路径必须不存在：

```sh
PY=/root/v05-gpu-venv/bin/python
OLD=/root/autodl-tmp/v05-acceptance/notch-multiface-20261016
OUT=/root/autodl-tmp/v05-acceptance/approximate-cuboid-rerun
export CUDA_VISIBLE_DEVICES=''
export OPENBLAS_NUM_THREADS=1
mkdir -p "$OUT"
git worktree add --detach "$OUT/baseline-code" d75483e0f5d837512b30cd4ad5cc4c6d189769db
$PY "$OUT/baseline-code/tools/replay_notch_geometry.py" --baseline "$OLD/baseline" --output "$OUT/strict" --vision /root/vision-fixed
$PY tools/replay_notch_geometry.py --baseline "$OLD/baseline" --output "$OUT/uncoupled" --vision /root/vision-fixed --approximate-mode outline_projection
$PY tools/replay_notch_geometry.py --baseline "$OLD/baseline" --output "$OUT/final" --vision /root/vision-fixed --approximate-mode raw_boundary
$PY tools/build_approximate_cuboid_review.py --frozen "$OLD/baseline" --strict "$OUT/strict" --intermediate "$OUT/uncoupled" --final "$OUT/final" --visibility "$OLD/visibility/visibility.json" --output "$OUT/review"
# 当前代码也保留显式 --approximate-mode off 严格模式。

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/root/autodl-tmp/v05-acceptance/notch-multiface-20261016/test-tools \
$PY -m pytest -q tests/test_approximate_cuboid.py tests/test_physical_bounds.py \
  tests/test_notch_geometry.py tests/test_metric_patch_adapter.py -k 'not notch_removes'
```

本地完整套件实际命令：`.\.venv310\Scripts\python.exe -m pytest -q`。

主要改动：`approximate_cuboid.py`、`metric_faces.py`、实际重放入口、选择配置、已有评审 HTML/JS、新的轻量评审构建器及针对性测试。未修改检测/分割、测量平面算法、接触、融合、碰撞、规划或执行算法；后续阶段均 NOT_RUN。

完整包 `delivery-final.tar.gz` 为 **206,801,076 字节**，SHA-256 为 `a4b876643e4fa74d959eccd0553eb29b815643bc30119fa3a92403e628e29f32`。服务器交付目录为 `/root/autodl-tmp/v05-acceptance/approximate-cuboid-20261017`。包内没有模型权重；重新求解仍需原冻结采集与 SAM 数组。较早的 `delivery.tar.gz` 是评审 CSV 导出中断时的保留包，不作为最终交付。
