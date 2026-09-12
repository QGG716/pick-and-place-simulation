# 双 RGB-D 与 observed-face V4 验收

本记录对应 `feat/v0.5-perception-ros2`，起始审查提交为
`eca9c8ce8a2f17cf99e5e6705f379d3c9651e418`。视觉上游固定为
`QGG716/cargo-detection-and-6D-estimation` 的 `dev` 分支，解析提交
`1d208f2ed380a207e6e46b4a62d2ac640edfe477`。本轮没有修改、合并或推送
`feat/v0.5-feasibility-core` 与 `feat/v0.5-online-continuous`，也没有开发 IK、碰撞、
运动规划或真机驱动。

## 验收结论

- `COVERAGE`：PASS。120° 候选在同一名义姿态下对 40 个箱体前表面的密集采样并集
  覆盖率为 100%，40/40 整面覆盖；最小图像边缘余量 59.033 px，最小投影面短边
  176.083 px，上下视场垂直重叠 0.136 m。
- `RENDERED_VISIBILITY`：保留真实机器人、输送机、墙体、桅杆和灯体后为 36/40，
  即 90%。其余 4 个箱体均在视锥内但被机械结构遮挡；没有箱体因出 FOV 而不可见。
- `OBSERVED_FACE_GEOMETRY`：上下模块独立运行实际 SAM、注册度量深度、固定 K 和
  上游 V4，共发布 43 个直接观测面；世界坐标融合后为 39 个对象。4 组重叠观测由
  世界面几何关联，冲突保留且不平均。
- `COMPLETE_CUBOID_GEOMETRY`：校准箱通过；全栈没有任何可执行资格的完整箱体。
  单面且无来源明确的厚度先验时，发布真实观测面，但完整中心、隐藏厚度与执行资格
  保持未知。上部链保留的 5 个完整估计也明确为 ineligible。

## 对最终问题的直接回答

| 问题 | 结果 |
|---|---|
| 07 为什么缺少右侧 | 主因是安装偏置与 90° 水平投影视场不足，不是水平裁剪。旧相机中心为 `[-1.325,+0.750,2.629341] m`，朝向 `+X`；前平面名义水平范围约 `Y=[-0.575,2.075] m`，覆盖不到货垛的 `Y=-1.04 m` 端和右墙 `Y=-1.15 m`。2592×1944 的 x 映射为恒等映射，无 ROI/letterbox；只存在为实现 VFOV=65° 的确定性垂直重采样。右墙资产存在。旧单相机前表面密集覆盖率 19.982%，仅 6/40 整面覆盖；另有 7 个已投影箱体被机械遮挡。 |
| 最终相机高度、角度、FOV | 桅杆仍在 J1 左侧/+Y 0.400 m、随 J1 运动、与 J1 航向相差 90°，总高 1.500 m。上部光心在 flange 上方 1.300 m（名义世界 z=2.629341 m），俯角 0°；下部在 flange 上方 0.300 m（名义世界 z=1.629341 m），物理光轴向下 20°。两组均为 2592×1944、HFOV 120°、VFOV 65°、目标 30 Hz。 |
| 相比原规格改变了什么 | 只新增下部 RGB-D+双补光模块，并把两组 HFOV 从 90° 改为 120°。上部高度、桅杆左右位置、400 mm 偏置、J1、底座、上部水平光轴、VFOV 和分辨率均未改变。K 已重新计算，`fx=748.245949 px`，没有沿用 90° 的 fx。 |
| 整个前箱面是否被视场并集覆盖 | 是，解析投影为 100%、40/40 整面覆盖。90° 基线只有 83.538% 和 28/40；110° 为 99.819% 和 39/40；115° 已达到 100%，最终保留 120° 以获得 59.033 px 最小边缘余量。 |
| 保留机械结构后的实际可见率 | 36/40 = 90%。这是语义渲染结果，不是用视锥推断的结果。 |
| 哪些箱体仍因遮挡不可见 | `carton_l00_c00`、`carton_l00_c01`、`carton_l00_c02`、`carton_l00_c04`；均为底层箱体。`out_of_fov_count=0`。 |
| `final_observed_faces_v4_candidate` 的来源 | 源图为 `images/厢内货物_8.png`（1290×1188）；精确结果路径为上游 `runs/experiments/15_batch_instance_aware/batch_2_8/cargo_8/07_final/final_observed_faces_v4_candidate.{jpg,json}`，几何产物为同 cargo 目录下 `06_geometry_3d/box_cuboids_observed_faces_v4_candidate.{jpg,json}`。实现入口是 `pipeline/geometry/refine_multiplane_cuboids.py`。历史精确 shell 命令未被上游记录，JSON 指向但 Git 中缺少 `moge2_pointmap.npz`，因此不声称逐像素复现历史图片。当前链调用的是该固定提交上的实际脚本，而不是复制算法。 |
| 主工程原来漏掉了什么 | 原链只运行基础 cuboid recovery，没有调用 V4 的多平面精化、面精度/覆盖门禁、共享边角拓扑、整体固定内参拟合与隐藏面抑制；还把实例全局 median/MAD 深度压成单层、把完成面混作观测面、混用不同版本的 corners/axes/dimensions，并含等价轴交换伪多假设和单相机硬编码。 |
| 已复现的几何根因 | 已复现全局深度过滤会删除真实侧面；基础结果把完成面标成 camera-facing；`unanchored_corners_3d` 与旧 axes/dimensions 混用会产生不自洽几何；轴交换会重复同一空间占据；轴对齐俯视图隐藏真实旋转；共享 `cameras[0]`/K/T 会破坏模块绑定。上述均有回归测试。 |
| SAM+metric depth 的可见面误差改善 | 同帧、同 mask、同 K/depth 下，V4 不移动已注册的真实面，因此 43 个保留面的数值指标变化为 0%，均值仍为 IoU 0.883863、precision 0.940600、coverage 0.936463；上/下模块点到面残差均值分别为 0.217 mm / 0.435 mm。真正的改进是语义纠错：基础链发布 125 个 `camera_facing` 面标签，V4 仅发布 43 个直接观测面，删除 82 个非观测完成面，误标数量下降 65.6%。不把“没有移动正确面”包装成精度提升。 |
| 完整箱体误差与单面不可观测 | 校准箱中心误差 30.985 mm、平均尺寸绝对误差 10.314 mm、D2 长方体对称等价姿态误差 1.631°，门限 100 mm/150 mm/20°，PASS。全栈上模块仅 5 个 ineligible 完整估计：中心误差 1.026 m、平均尺寸误差 70.709 mm，说明隐藏厚度/中心仍不可用；下模块没有完整估计。单面可继续给出面距离、法向与角点，但不推断隐藏厚度。 |
| 对称轴重命名与真实姿态错误是否分开 | 是。报告同时保留原始四元数误差和只允许 D2 正确 180° 符号翻转的长方体对称误差；不允许不等边轴任意置换。校准箱原始误差 179.919°，对称等价误差 1.631°。 |
| 上下重复目标是否正确融合 | 4 组重叠目标按世界位置、共面法向、平面距离和角点距离关联；不使用 Isaac `simulation_object_id`。融合得到 39 个对象、43 个观测面。4 组存在不一致观测，均以 `CONFLICT_RETAINED_NO_AVERAGE` 保留，不做简单平均。混 epoch、超过 1/30 s 不同步和缺模块均 fail-closed/降级。 |
| CPU、Humble、Isaac/GPU 做了什么 | Windows CPU：完整轻量测试 338 passed、1 deselected。Ubuntu 22.04/ROS 2 Humble：3 packages build，5 tests、0 errors/failures/skips。Isaac Sim 6.0.1.0 + NVIDIA RTX PRO 6000 Blackwell：8 个场景、双 render product、上下 RGB/registered depth/CameraInfo/metadata、实际 SAM+MoGe Mode B1、实际注册深度+固定上游 V4、融合、灯光 OFF/ON 和 12/16 s 视频均完成。 |
| 实际帧率与目标帧率 | 传感器与仿真采集时钟目标 30 Hz，physics 60 Hz；触发和目标视觉处理 2 Hz。该验收是有限关键帧，不是持续吞吐测试：全栈注册深度+V4 上/下顺序耗时 13.843 s + 28.661 s，约 0.0235 双模块帧组/s；SAM+MoGe Mode B1 为 31.570 s + 62.550 s，约 0.0106 帧组/s。ROS 实际持续传输 Hz 未测，Humble 仅证明功能；演示视频为 20 fps。两组 RGB8+float32 depth 在 2592×1944@30 Hz 的理论未压缩载荷为 2.1167 GB/s（不含复制/协议开销），PointCloud2 保持按需生成。 |
| PNG、视频、JSON 在哪里 | 代表产物见下节；完整筛选证据在 `docs/validation/evidence/dual-rgbd-v4-gpu-20260912/`。GPU 原始 run 在 `/root/autodl-tmp/v05-acceptance/dual-rgbd-v4-221b296/`，其中逐实例点云过滤审计约 600 MB，保留在 GPU run 中而未塞入 Git。 |

## 设计与实现事实

每个模块拥有自己的 `RGB_i / Depth_i / K_i / T_W_Ci(t_i) / module_id /
capture_id / sensor_epoch`。RGB 与 depth 共分辨率、共采集身份，depth 为
`SIMULATION_IDEAL_REGISTERED_DEPTH`；这不代表真实 ToF 串扰已建模。上下模块的左右灯
均为 5000 K、±0.120 m、同步触发的 `SIMULATION_LIGHT_PROXY`。固定曝光下 LIGHT_OFF/ON
平均亮度为 0.024669 → 0.025867，欠曝比例 0.895319 → 0.895129，mask IoU 与保留率
均为 1.0；提升很小，不能作为真实补光资格。

深度过滤现在按局部深度连续分量保留多个合理层，并保存原始、保留、剔除点及原因。
V4 返回后，角点拓扑、axes、dimensions 和 pose 必须来自同一角点集，并通过
pose+dimensions 反建角点一致性断言。单个不自洽的完整 cuboid 被逐实例拒绝，不再让
整批崩溃，也不连带丢弃其直接观测面。

## 代表性证据

- 16 秒 Isaac 演示：[`dual_rgbd_v4_isaac_demo_16s.mp4`](evidence/dual-rgbd-v4-gpu-20260912/dual_rgbd_v4_isaac_demo_16s.mp4)，320 帧、20 fps、SHA-256 `370d7e854d686bee7f00446cc64a7434fa2577c65f84c55a7aa4fa1bcb9b46f6`。
- 上/下原始 RGB：[`03_upper_rgb.png`](evidence/dual-rgbd-v4-gpu-20260912/03_upper_rgb.png)、[`04_lower_rgb.png`](evidence/dual-rgbd-v4-gpu-20260912/04_lower_rgb.png)。
- 上/下深度：[`upper_depth.png`](evidence/dual-rgbd-v4-gpu-20260912/upper_depth.png)、[`lower_depth.png`](evidence/dual-rgbd-v4-gpu-20260912/lower_depth.png)。
- 分割与直接观测面：[`10_segmentation.png`](evidence/dual-rgbd-v4-gpu-20260912/10_segmentation.png)、[`11_rgbd_cuboids.png`](evidence/dual-rgbd-v4-gpu-20260912/11_rgbd_cuboids.png)、[`lower observed faces`](evidence/dual-rgbd-v4-gpu-20260912/FULL_STACK_NOMINAL/modules/module_1_lower/rgbd_cuboids.png)。
- 融合世界 3D/真实俯视/侧视：[`15_fused_observed_faces_world_views.png`](evidence/dual-rgbd-v4-gpu-20260912/15_fused_observed_faces_world_views.png)。
- 同帧基础链/V4：[`16_upper_baseline_vs_v4.png`](evidence/dual-rgbd-v4-gpu-20260912/16_upper_baseline_vs_v4.png)、[`17_lower_baseline_vs_v4.png`](evidence/dual-rgbd-v4-gpu-20260912/17_lower_baseline_vs_v4.png)。
- 解析覆盖图：[`01_dual_rig_top_view.png`](evidence/dual-rgbd-v4-coverage-v1/01_dual_rig_top_view.png)、[`02_dual_rig_side_view.png`](evidence/dual-rgbd-v4-coverage-v1/02_dual_rig_side_view.png)、[`05_upper_lower_coverage.png`](evidence/dual-rgbd-v4-coverage-v1/05_upper_lower_coverage.png)、[`06_layer_column_coverage.png`](evidence/dual-rgbd-v4-coverage-v1/06_layer_column_coverage.png)。
- 机器可读总指标：[`dual_rgbd_v4_acceptance_metrics.json`](evidence/dual-rgbd-v4-gpu-20260912/dual_rgbd_v4_acceptance_metrics.json)。
- 校准、V4/MoGe、运行、渲染可见性：[`rgbd_calibration_gate.json`](evidence/dual-rgbd-v4-gpu-20260912/rgbd_calibration_gate.json)、[`rgbd_vs_moge_summary.json`](evidence/dual-rgbd-v4-gpu-20260912/rgbd_vs_moge_summary.json)、[`run_status.json`](evidence/dual-rgbd-v4-gpu-20260912/run_status.json)、[`rendered_visibility.json`](evidence/dual-rgbd-v4-gpu-20260912/FULL_STACK_NOMINAL/rendered_visibility.json)。
- Humble 日志：[`colcon-build.log`](evidence/dual-rgbd-v4-gpu-20260912/humble/colcon-build.log)、[`colcon-test-result.log`](evidence/dual-rgbd-v4-gpu-20260912/humble/colcon-test-result.log)。

原始 GPU run 保留完整 8 场景和每实例 `pointcloud_filter/*.npz`、
`metric_pointmap_filter_audit.json`。Git 只纳入代表性可审查证据，避免把数百 MB 的可再生
中间点云写入版本库。

## 上游可追溯性与边界

上游候选图、JSON、基础输入和 SAM NPZ 的路径、SHA-256、可重建命令及缺失历史信息见
[`upstream_v4_pipeline_comparison.md`](upstream_v4_pipeline_comparison.md)。本轮使用
`--suppress-hidden-fallback`，并将上游历史 selector 临时映射后恢复为真实
`registered_metric_depth_plane` provenance。K 和 capture-time 外参从不作为吸收误差的
优化变量。

本验收只证明仿真中的覆盖、直接观测面、证据分级、双模块时序与接口行为。桅杆截面、
支架、相机壳体和灯仍是 `VISUAL_PROXY`；flange 高度仍为工程暂定 datum；不存在真实相机、
真实光度、持续 ROS 带宽、碰撞/可达性或执行资格结论。
