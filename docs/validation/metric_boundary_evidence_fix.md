# Metric 物理边界证据汇总修复（2026-09-17）

仅第四步；分支 `feat/v0.5-perception-ros2`。开始时读取 AGENTS.md 并 fetch，
本地/远端均为 `f7668c37019c31b85cc848243231f7c99a6d171d`，工作区干净。
生产改动仅 `src/unloading_perception/metric_faces.py`，前三步接口代码未改。

## 两个修复前反例

1. 真实 `_boundary_kinds()`：80×80 optical-Z 深度，候选平面 Z=2.0 m，
   polygon 顶边 y=20；外探 2 px 的 y=18 为 1.5 m，4/6 px 为 3.0 m。
   旧代码对每个采样点取 `known[-1]`，输出 `PHYSICAL_EDGE_SUPPORTED`、
   `support_fraction=1.0`，新增拒绝断言失败。
   [旧局部回归日志](evidence/boundary-evidence-20260917/red-boundary.log)
2. 复用原有 500×500 解析单箱 fixture，通过真实 `fit_metric_faces()`、共享平面求解和
   `_complete()` 得到正常可接受完整体。仅将某条候选边外 2 px 的 15 个探测位置改成
   比预测 Z 近 0.5 m 的深度；4/6 px 保留原来明确启用 no-hit 语义的 +Inf。
   所有改动都在 mask 外，内部深度、K、标签和种子不变。旧完整路径仍输出 `accepted=true`，
   关键边为物理支持且比例 1.0。
   [旧完整体回归日志](evidence/boundary-evidence-20260917/red-completion.log)

这两个反例分别证明局部覆盖缺陷和解析完整路径中的实际误接受；第二个使用近处遮挡/+Inf 背景，
并非声称在真实满垛数据中观测到同一事件。测试没有替换或 monkeypatch 被测分类器/门控。

## 两级归约及诊断

**A：同一沿边位置的多个外探距离。** `_reduce_boundary_probe_evidence()` 对决定性标签集合归约：
有且只有一种决定性证据时保留该证据；近处遮挡与远处背景/no-hit 同时存在时，输出
`UNCLASSIFIED_BOUNDARY`，reason 为 `DEPTH_EVIDENCE_CONFLICT`。与顺序、重复次数无关，
“1 个遮挡 + 2 个背景”也不做多数表决。

无效深度和与候选平面连续的深度没有决定性物理支持；没有任何决定性证据时保持未知。
若没有相反证据，允许部分探测无效或连续：例如 `[NaN, 3.0, NaN]`、`[2.0, 3.0, NaN]`
仍提供一票物理支持，不强制三个距离都有效。NaN、-Inf、非正深度均不提供物理支持。
+Inf 仅在 `positive_infinity_is_no_hit=true` 且预测 optical-Z 有限、严格为正时提供 no-hit 支持；
它仍会与近处遮挡形成冲突。没有填充或修改深度数组。

**裁切。** 候选点按原有最近像素采样方式取整，若其自身落在图像边缘像素或画面外，记为候选裁切，
不能认证为物理边界。较远外探点越界只记录 `OUTER_PROBE_OUTSIDE_IMAGE`，不覆盖较近有效证据。
例如候选顶边 y=5、仅 6 px 外探越界，2/4 px 的一致背景仍有效；y=1 时全部外探越界但候选未裁切，
保持未知；y=0 是真正候选裁切。其他边独立判断。

**B：整条边的 15 个位置。** 每个位置只投一次；物理支持、遮挡或候选裁切达到原有 10 票才输出该类别，
否则为未知。15 个位置中最多一个类别达到 10 票，没有类别优先选择问题。冲突位置归入独立 conflict 计数，
不是物理票；unknown 计数不重复包含 conflict，五类计数之和为 15。
不存在“一处冲突就否决整条边”：10 物理 + 3 未知 + 2 冲突仍输出物理支持，
9 物理 + 4 未知 + 2 冲突则保持未知。

保留 `kind`、`support_fraction` 字段和类型。`support_fraction` 现在明确为
**物理支持位置数 / 全部 15 个采样位置**，不再采用旧代码的最终类别占比；不是概率、置信度或精度。
未知/冲突不从分母删除：仅 1 个支持和 14 个未知时为 1/15。
这些是采样位置票数，不宣称是独立像素样本数。
新增 sample_count、五类 sample_counts、显式分子/分母、冲突位置索引，以及每个位置/距离的标签、
原因和像素坐标；仅保留每条边 15×3 个探测记录，没有全分辨率日志或点云。

**参数未变：** 外探仍为 `(2,4,search_px)`、默认 search_px=6；深度比较仍为严格超过 ±0.01 m；
沿边仍取 0.15～0.85 的 15 个位置、门槛仍为 10 票。平面噪声/残差、支持域、相机标定、求解器、
随机种子和 silhouette 优化均未改。测试包括 10 mm 边界及相邻浮点值。

## 完整体门控与面片保留

`_complete()` 继续要求所有需要的候选面边界都为物理支持；未支持则保持 accepted=false。
增加 `completion_diagnostics.rejection=UNSUPPORTED_PHYSICAL_BOUNDARY` 及 rejected_boundaries，
包含 support_label、axis/side、edge_index、kind/reason，可定位对应面和边。
既有 `candidate_faces[].boundary_evidence` 与实测 `camera_facing_faces[].boundary_evidence` 均保留诊断。

真实解析对照的实际结果：正常输入 `accepted=true / OBSERVABLE_METRIC_MULTIFACE`；
仅外探冲突输入 `accepted=false / UNRESOLVED_PHYSICAL_BOUNDARIES`，关键边 conflict=15、物理支持=0。
两者都保留 3 个有效实测面片。测试逐项精确比较面片 2D/3D 坐标、法向、offset、冻结支持域及完整
final_support 字典，均未改变且为 PASS。完整体拒绝没有删除或移动已测面片。

JSON 往返后经真实 `validate_final_record()`→`validate_metric_record()`，拒绝及 completion_diagnostics
仍保留；`observed_faces_from_geometry_record()` 仍输出 3 个面片，状态为
`MULTIFACE_OBSERVED_VOLUME_UNRESOLVED`，不是完整体认证。没有手动改 accepted 制造测试结果。

## 运行与边界

全部计算在授权服务器 CPU。metric 使用独立可选依赖环境；没有新增项目依赖，
SciPy/OpenCV 未变成默认轻量核心的新增必需依赖。原解析 fixture 抽成测试 helper 供两组回归复用，
原正常完整体与平面移动拒绝断言全部保留。

| 检查 | 实际结果 |
|---|---|
| 旧局部分类反例 | 1 failed，错误物理支持 1.0 |
| 旧完整体反例 | 1 failed，错误 accepted=true |
| 新增及全部现有 tests_metric | 31 passed（新增 29、原有 2），1.05 s |
| 默认轻量 pytest，含现有面片/几何及前三步 CPU 回归 | 487 passed，1 deselected，7.02 s |

小型日志及 LF 规范化源码 SHA-256 见 [证据目录](evidence/boundary-evidence-20260917/tested_sources.json)。
新增测试置于现有 tests_metric，现有 Humble CI 的可选 metric 步骤自动收集；未加 skip 或降低断言。
前三步真实 ROS 回归由现有 Humble CI 继续运行；当前提交的实际 CI 查询结果在交付回复报告。

```bash
# 服务器仓库根目录；独立可选 metric 环境初始化一次：
/usr/bin/python3.10 -m venv /root/autodl-tmp/v05-acceptance/boundary-evidence-20260917/venv
METRIC=/root/autodl-tmp/v05-acceptance/boundary-evidence-20260917/venv/bin/python
"$METRIC" -m pip install -e ./packages/unloading_contracts -e '.[dev,perception-metric]'
# 修复前先运行仅含局部反例的测试文件，随后增加完整体反例并用 -k complete_cuboid 运行。
PYTHONPATH=src:packages/unloading_contracts/src:tools:. "$METRIC" -m pytest -q tests_metric
env -u PYTHONPATH /root/autodl-tmp/v05-acceptance/cpu/venv/bin/python -m pytest -q
```

解析 fixture 使用已知标签和种子生成几何输入，但实际平面仍由真实深度求解；它不代表自动检测或满垛感知验收。
未运行 Isaac、SAM/MoGe、GPU 推理、训练、重采集、视频或运动规划；未改写历史归档。
本轮未统计真实满垛接受数量，未扩大到几何精度收紧或第五步双模组融合去重。
