# RGB-D 深度约束几何实测，2026-09-15

本轮已经修改实际求解器。冻结标定箱的实际 SAM 输入，上视角恢复 3 个面、下视角恢复 2 个面，两者均恢复合格完整箱体；最大角点集合误差分别为 3.871、4.275 mm。双箱输出 19 个实测面片，保留一个 SAM 合并来源。满垛同输入只求解一次，有面实例由 38/44 增至 43/44，面片由 38 增至 49；共有面的几何误差下降，但仍有 13 个面超过 10 mm 边界越界门槛。因此这是单箱及小场景几何通过、满垛改善但仍存在边界问题的结果，不是完整机械场景验收通过。

所有数值求解、测试、SAM 推理、Isaac 渲染和视频生成均在用户指定 GPU 服务器完成。正式输入保持 2592×1944、原 K、原捕获时刻 T_W_C 和米制 optical-Z。未训练模型、未更换 SAM、未运行规划或生成执行命令。

**1. 标定箱之前失败的真实原因是什么？**

在起始 HEAD 上核对了旧入口、固定视觉上游及上一轮冻结产物。8 次旧 V4 联合调用返回了结果，但最终独立 V4 度量检查全部拒绝；上下标定视角均没有合格最终面。旧式求解以二维轮廓配准、PnP 和尺寸缩放正则为主，可靠注册深度并非最终目标函数的主数据项。

对同一 RGB、depth、SAM mask、K 重新固定观测面标签，得到下表。数值为各阶段平面在明确排除边缘后的平均深度残差，单位 mm；所有原始值、P95、点数、法向、offset、角点、尺度及可定义的位姿变化见 [上视角阶段 JSON](evidence/metric-geometry-20260915/calibration-c50ff4c/module_0_upper/stages_complete.json)、[下视角阶段 JSON](evidence/metric-geometry-20260915/calibration-c50ff4c/module_1_lower/stages_complete.json)。

| 阶段 | 上视角 | 下视角 |
|---|---|---|
| A 原始注册点，参考初始平面 | 1.534 / 2.818 / 2.287 | 2.539 / 3.090 |
| B 原过滤点，参考初始平面 | 1.517 / 2.866 / 2.243 | 2.546 / 3.136 |
| C 初始平面 | 1.514 / 2.756 / 2.193 | 2.509 / 3.047 |
| D 未 anchoring 多面几何 | 16.045 / 7.540 / 12.386 | 17.419 / 14.710 / 160.414 |
| E 二维目标 | 平均角点移动 4.450 px，最大 7.974 px | 平均 9.349 px，最大 19.191 px |
| F PnP/尺度后 | 31.648 / 16.732 / 19.196 | 69.599 / 59.623 |
| G fallback | 1.515 / 345.318 / 93.625 | 2.509 / 181.277 / 388.598 |
| H 新求解最终观测面 | 0.019 / 0.934 / 0.017 | 0.270 / 0.105 |

误差在 D 就明显增加：独立平面被初始化为共享盒体时，正交化和边界/尺度初始化没有保持实测平面位置。F 进一步偏移；上视角二维 PnP RMSE 仅 0.450 px，仍有厘米级深度误差。下视角 D→F 中心移动 82.846 mm、旋转约 2.029°；三维尺寸改变约 −20.439/−7.891/+17.108 mm。二维拟合好不能证明三维正确。G 只能锚定某一面，不能修复其余面。下视角还缺少可直接观测的顶面，补全平面不能冒充深度观测。

E 是二维目标，没有三维平面残差；非正交旧盒体的合法姿态/尺寸分解标为 `NONORTHOGONAL_CUBOID`，没有伪造数值。A/B 的残差明确以 C 初始平面为参考。各阶段的跨实体判断只用于独立 GT 评价，未进入正式拟合。

**2. 检查器是否存在误拒，如何证明？**

存在支持域语义问题：旧检查可能把一个预测面投影内属于其他可见面、背景或边缘的点一起用于平面评价。CPU 解析正例先于新求解器提交：验证 optical-Z/range、非等焦距、米制单位、像素往返、原生映射、模组 K/T 绑定、三正交面；正确平面通过，平移/旋转后的平面拒绝。

新检查使用“当前实例 ∩ 原始观测面标签”，标签在最终求解前冻结。排除规则仅依赖观测：2 px 边缘、无效值、20 mm 局部深度跳变；不按到最终候选平面的距离筛选。8×8 空间块留出约四分之一点独立检查。JSON 同时保留 raw/interior/excluded 的 mean/P95、数量和排除原因，并绑定 depth/mask/K 哈希。

例如下视角 SAM 的两个面原始均值为 4.371、5.202 mm；排除边缘后为 0.270、0.105 mm；空间留出为 0.269、0.105 mm。第二面排除的 1372 个边缘点平均误差为 56.37 mm，而内部 13775 点支持准确平面。这证明支持域污染确实能误拒。与此同时，D/F 在修正后的同一支持域上仍有明显误差，不能把全部失败归因于检查器。

正式平均残差仍为 3 mm，P95 为 6 mm。新增 GT 面距 5 mm、法向 1°、越界 10 mm、角点/中心 15 mm、尺寸 20 mm、姿态 1°，在新算法运行前冻结于 [配置](../../configs/perception/metric_geometry_acceptance.json)，SHA256 `e30907d8e6dd3755149c28869f6d4f02f97bedeb6999a3a931d1ec34b8e6c0e3`。按 2–4 m 距离及横向像素采样量级制定，未按实例 GT 调权或放宽。

**3. 新求解器加入了哪些真正的 metric 约束？**

新增 `DEPTH_METRIC_PATCHES_V1`，作为 RGB-D 入口默认策略；旧 `metric_v4_runner.py` 保留比较用途。复用固定上游平面提取，新增 NumPy/SciPy/OpenCV 适配器，不复制完整上游工程。

- 点到观测平面的米制残差是主项，以 1 mm 噪声尺度归一化，使用鲁棒损失、按面平衡点数。
- 合法旋转向量生成 SO(3)，共享正交方向；各面 offset 由深度约束。
- 原始标签、连通区域和局部法向避免把整个多面 mask 当成一个面；对较小残余面及不同 offset 的平行面继续提取。
- 完整体由同一组方向和边界生成共享交线、顶点及八角点；二维面区域边界只优化未直接观测的边界参数，按 2 px 归一化、权重 0.05。已观测平面及旋转的辅助优化移动量严格为零。
- 不优化 K、T、深度尺度，也不使用 GT 尺寸。最终检查从输出面片重新计算平面；完整体若脱离观测面也拒绝。

依赖放在可选 `perception-metric` extra。轻量核心不引入 ROS、GPU、Isaac 或 SciPy 必需依赖。

**4. 上、下视角各自恢复了哪些可见面？**

以箱体本体坐标命名，上模组恢复 −X、+Y、+Z；下模组恢复 −X、+Y。每个面直接保存实测平面、二维观测区域和三维面片四点，`ObservedFace` 不再必须从完整八角点中索引。隐藏厚度、完整体中心和面片角点的物理语义分别处理；面片角点不命名为箱体物理角点。

**5. 单箱可见面位置、法向、边界误差是多少？**

下表为实际 SAM 主结果。面距是面片四点到对应 GT 面的平均距离；边界指标是面片超出 GT 面范围的最大距离；P95 是每个冻结面标签内的残差分位数。

| 模组/本体面 | GT 面距 mm | 法向 ° | raw 均值/P95 mm | 内部均值/P95 mm | GT 越界 mm | 面片覆盖观测标签 |
|---|---:|---:|---:|---:|---:|---:|
| 上 −X | 0.0377 | 0.01532 | 0.0198 / 0.0375 | 0.0192 / 0.0366 | 0 | 90.5% |
| 上 +Z | 0.0763 | 0.02072 | 1.4982 / 1.7620 | 0.9343 / 1.7572 | 0 | 90.3% |
| 上 +Y | 0.0327 | 0.01394 | 0.1675 / 0.0340 | 0.0171 / 0.0325 | 0 | 82.8% |
| 下 −X | 0.0097 | 0.00416 | 4.3710 / 0.5228 | 0.2701 / 0.5174 | 0 | 82.2% |
| 下 +Y | 0.0061 | 0.00259 | 5.2016 / 0.2077 | 0.1054 / 0.2008 | 0 | 88.1% |

少量大离群点会使原始均值大于 P95，这是保留的边缘污染，不是数值抄错。面片是保守的观测子区域，“越界为零”不等于完整物理边界全部恢复。完整物理边/角点另由满足边界证据的盒体评价，不能与上述子区域指标混用。

**6. 哪个视角足以恢复完整箱体，哪些参数仍不可观测？**

标定箱两个视角均有足够的实测平面和可见轮廓端点约束。上 SAM 中心误差 1.922 mm、三尺寸误差 3.634/0.915/0.919 mm、姿态 0.02072°、最大角点集合误差 3.871 mm；下 SAM 分别为 2.223 mm、0.731/0.131/3.982 mm、0.00477°、4.275 mm。角点按无标签长方体对称性做同一集合评价，新旧约定一致。

下视角的第三面并未直接观测；隐藏角点由有证据的轮廓端点与共享几何推断，单独标为完成体。单面或缺乏物理边界证据时不补造厚度。双箱 7 个 SAM 输出和满垛 44 个输出均未认证完整体；不是仅按“面数≥2”宣布可观测。

**7. Oracle mask 与实际 SAM 结果差多少？**

| 输入 | 接受面数 | 中心 mm | 最大尺寸误差 mm | 姿态 ° | 最大角点误差 mm |
|---|---:|---:|---:|---:|---:|
| 上 ORACLE_MASK_DIAGNOSTIC | 3 | 0.228 | 0.398 | 0.01386 | 0.516 |
| 上 SAM | 3 | 1.922 | 3.634 | 0.02072 | 3.871 |
| 下 ORACLE_MASK_DIAGNOSTIC | 2 | 2.228 | 3.890 | 0.00607 | 4.260 |
| 下 SAM | 2 | 2.223 | 3.982 | 0.00477 | 4.275 |

Oracle 只用于隔离几何问题，未传给正式 SAM 求解。实际小场景继续使用已有 oracle proposal 输入再运行真实 SAM，并非新检测器或全自动检测验收。GT mask、实体 ID、pose、尺寸只在解析 fixture 输入生成或独立评价中使用，正式拆分未使用 GT。详见 [单箱矩阵](evidence/metric-geometry-20260915/calibration-c50ff4c/summary.json)、[解析三场景矩阵](evidence/metric-geometry-20260915/analytic-final/analytic_matrix.json)。

**8. 相邻箱体跨边界问题是否改善？**

| 场景/模组 | Oracle 每箱面数 | SAM 每实例面数 | 旧版合格面数 | 完整体 |
|---|---|---|---:|---|
| 相邻/上 | 2、3 | 2、3 | 0 | 未认证 |
| 相邻/下 | 1、2 | 1、2 | 0 | 未认证 |
| 部分遮挡/上 | 3、3 | 3、3 | 0 | 未认证 |
| 部分遮挡/下 | 2、2 | 一个合并来源，5 面 | 0 | 未认证 |

总分母为 8 个 GT 可见“模组×实体”、7 个实际 SAM 输出：6 个身份唯一，1 个合并歧义；输出 19 个深度面片。唯一身份的面均满足冻结面距/法向/越界门槛。合并来源的 5 个面逐一列出全部候选 GT 评价，不把评价匹配回灌为正式对象身份；它们均在来源实体集合中找到合格几何支持，但不能据此宣称完成实例分离。

使用观测区域内最大矩形或保守四边形、深度跳变/法向分区，避免凹形遮挡区域被大盒体轮廓跨越。边界明确区分 `PHYSICAL_EDGE_SUPPORTED`、`OCCLUSION_BOUNDARY`、`IMAGE_CROP_BOUNDARY`、`UNCLASSIFIED_BOUNDARY`。仅在已声明 Isaac 理想深度语义时将正无穷视为无命中；通用输入的无效深度仍为 unknown。

部分遮挡上视角有些面片只覆盖标签的约 32%–63%，保守四边形不能表达全部凹区域；完整观测区域仍以 RLE 保留。满垛共面边界亦未全部解决。失败历史没有删除：窄面缺失、局部法向过滤过严和凹区域中心不在支持内等失败，保存在 [failure_history](evidence/metric-geometry-20260915/failure_history) 及对应日志。最终 [小场景门槛报告](evidence/metric-geometry-20260915/small_gate_final.json) 明确保留身份歧义。

**9. 相同满垛输入的新旧误差和有效结果数是多少？**

固定相同 44 个 SAM 实例和修正后的 lineage，无删除失败分母。42 个来源唯一、2 个来源合并。旧版 38 个面来自 38 个实例；新版 49 个面来自 43 个实例。上模组 13 实例/19 面，下模组 31 实例/30 面。新版无面的下模组 mask 1 共 597 点，305 点位于边缘、292 点遇深度跳变，没有可靠内部支持，保持拒绝。44 个完整体全部为 `UNRESOLVED_PHYSICAL_BOUNDARIES`。

对来源唯一且新旧都存在的 **36 个同一 GT 面** 配对，不把新增面的样本变化混入改善量：

| 共有面指标 | 旧版 | 新版 |
|---|---:|---:|
| GT 面距均值 / 最大 mm | 0.548 / 2.384 | 0.00743 / 0.02453 |
| 法向均值 / 最大 ° | 0.22859 / 0.77414 | 0.00440 / 0.02272 |
| 边界越界均值 / 最大 mm | 33.474 / 168.743 | 15.440 / 145.004 |
| 越界超过 10 mm | 35/36 | 13/36 |

新版全部 47 个身份唯一面：平均面距 0.0152 mm、法向 0.00541°、越界 11.881 mm；仍是 13 个越界超限。另有 2 个面来源合并，不强行指定 GT ID，不计入身份唯一几何均值，但保留在 49 面及 44 实例分母中。超过真实箱面范围意味着仍有跨箱风险；不能用准确平面或 SAM 内部区域精度掩盖它。

49 个面中最大 raw 平均深度残差 0.644 mm，最大内部平均 0.523 mm，最大空间留出平均 0.549 mm。观测面片覆盖率范围 62.5%–98.1%。旧检查使用预测多边形、新检查使用冻结观测区域，深度支持定义不同，因此不把两者残差直接当作严格配对改善；上表 GT 几何采用同一评价器。身份修正属于上一轮成果，本轮配对两边均使用同一映射。

完整 [配对表及所有最终检查](evidence/metric-geometry-20260915/frozen-stack-final/paired_stack_report.json)、[统计和 13 个未解决越界案例](evidence/metric-geometry-20260915/delivery/quantitative_summary.json)。双模组关联仍输出 36 个容器，未改融合阈值。唯一一次求解结束后的报告发生 mappingproxy 序列化错误；修复后使用 `--report-only` 读取持久化结果重建报告，没有再次求解满垛。

**10. 哪些场景为无遮挡 ALGORITHM_ONLY？**

复用冻结 `RGBD_CALIBRATION_BOX`，另建 `GEOMETRY_ADJACENT_TWO`、`GEOMETRY_PARTIAL_TWO` 独立算法 bundle。双箱场景保留全部对象定义，仅两个箱体可见；通过算法专用渲染配置省略 `chassis`、`conveyor_transverse`、`conveyor_longitudinal`。实体列表和场景身份见 [独立 manifest](evidence/metric-geometry-20260915/small-scene-bundle/GEOMETRY_ADJACENT_TWO.manifest.json)。图片均标明 ALGORITHM_ONLY。

**11. 哪些遮挡问题延期，有没有改机械？**

原满垛仍使用原有机械遮挡冻结数据。传送带/底盘遮挡区域标为 `DEFERRED_MECHANICAL_OCCLUSION`，没有填成识别成功，没有新增 first-hit 研究。共享机械布局、相机安装、FOV、K、外参、深度尺度均未改；未后移底盘或输送机。未修改 feasibility/online 工作副本或分支，固定 feasibility 资产只读。

**12. ROS 交接和 CI 是否保持通过？**

服务器 `pytest -q`：388 passed、1 deselected（已有默认筛选）、15 warnings；独立可选 metric 测试 2 passed。包含错误平面/错捕获绑定拒绝、独立面片接口、非矩形区域边界、合法 SO(3) 完整体、平移完整体被拒绝等回归。Humble 3 个包构建完成，6 tests、0 errors、0 failures、0 skipped。

新满垛 49 面经过真实 ROS 节点/world smoke，往返指纹一致、capture-time 绑定保留、旧序列和错误 T_W_C 拒绝、未知完整尺寸保留、执行命令 0；`planning_admissible=false`。参见 [ROS 报告](evidence/metric-geometry-20260915/ros-final/ros_handoff_report.json)。禁止真实执行的边界未变。

几何功能提交 `c50ff4c2a6b6ece69aafb1de1f62bce14c68b5d9` 的 [GitHub CI 34931529226](https://github.com/QGG716/pick-and-place-simulation/actions/runs/34931529226) 为 success。原 Humble/轻量安装步骤保留，新增隔离环境运行 `tests_metric`，没有 continue-on-error 或整组 skip。最终归档提交的 CI 及三端最终 SHA 以完成回复中的实际查询为准。

报告工具提交 `1a33dfef5e5ca91d140d42db68650974e54669d2` 的 [CI 34932541781](https://github.com/QGG716/pick-and-place-simulation/actions/runs/34932541781) 也为 success，见 [查询存档](evidence/metric-geometry-20260915/ci_verified_runs.json)。另在该提交上实际运行了未配置/未导入 MoGe 的 SAM→metric 单箱路径，输出 3 面、0 个可执行候选，见 [实测报告](evidence/metric-geometry-20260915/no-moge-final/no_moge_report.json)。SAM 推理约 1.20 s；这不是 30 Hz 端到端声明，也不把相机目标 30 Hz 与算法频率混用。

**13. 图片、JSON、视频与复现命令在哪里？**

所有展示从最终接受/拒绝 JSON 生成，未使用检查前 worker.png。新旧使用同视角、同缩放；3D 图使用真实旋转和角点，GT 为独立橙色虚线，观测面与推断完成体分开。

- 单箱：[上视角六联图](evidence/metric-geometry-20260915/calibration-c50ff4c/module_0_upper/single_box_comparison.png)、[下视角](evidence/metric-geometry-20260915/calibration-c50ff4c/module_1_lower/single_box_comparison.png)、[原始点云与真实旋转 3D 对照](evidence/metric-geometry-20260915/calibration-c50ff4c/module_0_upper/raw_cloud_metric_comparison.png)。
- 双箱：[相邻上视角](evidence/metric-geometry-20260915/small-matrix-final/GEOMETRY_ADJACENT_TWO/module_0_upper/two_box_comparison.png)、[部分遮挡下视角，SAM 合并歧义](evidence/metric-geometry-20260915/small-matrix-final/GEOMETRY_PARTIAL_TWO/module_1_lower/two_box_comparison.png)。
- 满垛：[上视角同输入新旧对照](evidence/metric-geometry-20260915/delivery/module_0_upper_stack_comparison.png)、[下视角](evidence/metric-geometry-20260915/delivery/module_1_lower_stack_comparison.png)。
- [16 秒对照视频](evidence/metric-geometry-20260915/delivery/metric_comparison_16s.mp4)，1440×960、10 fps、160 帧，读取并验证了帧数。
- [产物哈希清单](evidence/metric-geometry-20260915/artifact_manifest.json)、[冻结输入路径及哈希](evidence/metric-geometry-20260915/frozen_input_manifest.json)、[被测环境](evidence/metric-geometry-20260915/server_tested_state.json)。大体积 RGB/depth/SAM 缓存保留服务器；Git 中保存最终 JSON、支持区域、图像和测试日志。

服务器结果根目录：`/root/autodl-tmp/v05-acceptance/metric-343eb22-20260915`。复现应使用新输出目录，避免覆盖本次证据；以下主验证复用实际冻结输入，无需重新采满垛。

```bash
cd /root/autodl-tmp/pick-and-place-simulation-v0.5-perception-ros2
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PWD/tools:."
GPU_PY=/root/v05-gpu-venv/bin/python
F=/root/autodl-tmp/v05-acceptance/dual-rgbd-v4-221b296
L=/root/autodl-tmp/v05-acceptance/lineage-bd00dfe-20260914/v4-final-6f6e2b3
R=/root/autodl-tmp/v05-acceptance/metric-343eb22-20260915
NEW=/root/autodl-tmp/v05-acceptance/metric-reproduction-NEW

"$GPU_PY" -c "import sys; sys.path.append('/usr/lib/python3/dist-packages'); import pytest; raise SystemExit(pytest.main(['-q']))"
"$GPU_PY" -c "import sys; sys.path.append('/usr/lib/python3/dist-packages'); import pytest; raise SystemExit(pytest.main(['-q','tests_metric']))"
"$GPU_PY" tools/validate_metric_analytic.py --vision /root/vision-fixed --output "$NEW/analytic"
"$GPU_PY" tools/diagnose_metric_calibration.py --capture "$F" --legacy "$L" --vision /root/vision-fixed --output "$NEW/calibration"
"$GPU_PY" tools/complete_metric_stage_evidence.py --capture "$F" --legacy "$L" --vision /root/vision-fixed --result "$NEW/calibration"
"$GPU_PY" tools/run_metric_small_matrix.py --capture "$R/small-capture" --bundle "$R/small-scene-bundle" --vision /root/vision-fixed --models /root/autodl-tmp/v05-acceptance/model-manifest.json --output "$NEW/small"
"$GPU_PY" tools/check_metric_small_gates.py --calibration "$NEW/calibration" --small-matrix "$NEW/small" --small-capture "$R/small-capture" --thresholds configs/perception/metric_geometry_acceptance.json --output "$NEW/gates.json"
# 小场景通过后才运行；本次已执行一次，交付阶段没有再次执行此求解。
"$GPU_PY" tools/regress_metric_frozen_stack.py --capture "$F" --legacy "$L" --vision /root/vision-fixed --gate-report "$NEW/gates.json" --output "$NEW/stack"
# 仅重建现有满垛报告，不重新求解：同命令加 --report-only。

ARTIFACT_ROOT="$NEW" RUN_ID=metric bash tools/run_humble_acceptance.sh
source /opt/ros/humble/setup.bash
source "$NEW/humble/metric/install/setup.bash"
PYTHONPATH="$PYTHONPATH" /usr/bin/python3.10 tools/probe_algorithm_world_handoff.py --observation "$NEW/stack/FULL_STACK_NOMINAL/fused_algorithm_observation.json" --manifest "$F/scene_bundle/FULL_STACK_NOMINAL.manifest.json" --output-directory "$NEW/ros"
"$GPU_PY" tools/validate_rgbd_without_moge.py --scene-directory "$F/RGBD_CALIBRATION_BOX" --model-manifest /root/autodl-tmp/v05-acceptance/model-manifest.json --vision-root /root/vision-fixed --output-directory "$NEW/no-moge" --geometry-manifest "$F/scene_bundle/RGBD_CALIBRATION_BOX.manifest.json"
"$GPU_PY" tools/summarize_metric_validation.py --run "$R" --capture "$F" --legacy "$L"
```

小场景补采命令见 `prepare_metric_small_scenes.py` 和 `scripts/isaacsim_perception_capture.py --geometry-algorithm-only`；本次使用现有 Isaac 环境/固定 USD 资产，未升级驱动或重装依赖。`summarize_metric_validation.py` 读取本轮目录约定中的持久化 JSON/图片，完全不调用拟合。

**14. 起始、被测、最终提交 SHA 分别是什么？**

起始经 fetch 核对为 `343eb22b8261d67f724a9c78d119d5ee6200288a`。功能依次提交 `750fe9c`（解析检查）、`7c235e2`（深度面与独立面片）、`c50ff4c`（窄面/遮挡边界、矩阵及 CI）；报告修复/汇总为 `1a33dfe`。固定视觉上游 `1d208f2ed380a207e6e46b4a62d2ac640edfe477` 保持干净。

满垛求解发生于服务器 Git 引用快进前，但执行源码与 `c50ff4c` 逐文件核验相同（仅允许 CRLF/LF 差异），见 [源码等价审计](evidence/metric-geometry-20260915/server_code_equivalence.json)。随后服务器快进到精确提交，重新验证单箱、解析、CPU；完整体检查的输出亦保存在 `calibration-c50ff4c`。满垛未因记录 SHA 而重跑。最终归档提交包含本文件自身，完整最终 SHA 在完成回复中列明，避免自引用 SHA。

**15. 本地/GPU/GitHub 实际同步状态如何？**

仅操作 `feat/v0.5-perception-ros2`。功能及报告工具已推送，服务器通过 Git bundle 先核对所有待替换文件与新提交文本相同，再保存 stash 并快进；保留 stash 备份，没有 reset --hard、force push 或覆盖未知修改。最终证据归档后继续推送/快进并检查三端实际 HEAD、工作树和新 SHA 的 CI，状态在完成回复给出。

本地目录为 `D:\code\simulation\pick-and-place-simulation-v0.5-perception-ros2`；GPU 副本为 `/root/autodl-tmp/pick-and-place-simulation-v0.5-perception-ros2`，连接使用用户本轮更新的 westb:41855。密码未写入仓库或证据。
