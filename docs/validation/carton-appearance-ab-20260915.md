# 纸箱外观 A/B：第一阶段结果

**结论：本次整体外观修改没有带来稳定的边界改善。** B 的下视角 SAM 边界定位略有改善，但最终面片覆盖率降低、越界增加；上视角分割有所退化。纹理已实际接入 Isaac，不能再把问题归因于“没有加载贴图”，也不能据此证明增加边缘几何一定有效。本阶段完成后暂停，C/D、RGB 箱缝辅助算法、训练和规划均未开展。

**采集链与控制条件**

基线 `d3e92cd1de9c5eefc3f8f36326e54491c4c3fee0` 经本地 fetch、origin 和 GPU 实查一致，工作树干净。原采集入口独立创建纯色 `Carton`，选中箱绑定自发光 `Target`，没有使用 replay 的纸箱纹理。新正式采集取消选中箱高亮；A0 历史目录未修改。

A/B 在同一 Isaac 会话中各采上下模组一帧，分别有真实独立 capture ID 和墙钟时间。均为原 FULL_STACK_NOMINAL，保留全部机械遮挡、机器人/箱体位姿、2592×1944、K/T、米制 optical-Z、原注册重采样方式。原生全图送入同版本 SAM；模型内部预处理保持原样，没有裁剪推理、复用不同 RGB 的 embedding/mask 或引入 MoGe。

A 使用原被动纯色材质；B 使用 [n4 的 CC0 纸板纹理](https://opengameart.org/content/seamless-pattern-pack-carton512x512png)，每 0.25 m 重复，所有同尺寸箱体共享同款图案。50 mm 顶部胶带和小型重复箭头仅为 diffuse 外观假设。B 同时改变颜色、纹理、粗糙度和金属度，因此本报告评价的是**整体外观**，不声称单独纹理贡献。没有法线/凹凸、位移、倒角、裁切透明或实体胶带。

两组都使用相同六面 UV 网格；父级尺寸/位姿保持原值，原单位 Cube 碰撞体作为不参与成像的子 prim 保留。表面位置、朝向、外包络、UV 尺度的轻量测试通过。实际传感器路径为上模组 `/Replicator/Camera_Xform_04/Camera` → `Replicator_04` render product，下模组 `_05` → `Replicator_05`；RGB annotator 经既有注册映射写入 `sensor_rgb.png`。材质绑定在 `/PerceptionValidation/Primitives/p_*_carton_*/Visual`，A 为 `Materials/Carton`，B 为共享 `Materials/Paper_*`。

两组实际 tone map op=6、ISO=100、shutter=50、fNumber=5，自动曝光关闭；记录的渲染模式均为 `RealTimePathTracing`，每次固定关节命令后稳定渲染 16 帧。上下两组均满足：**depth 有效/无效掩码差异 0 像素、有效 depth 平均/最大差值 0、物理对象 GT mask 完全一致、K/T/机器人状态一致**。贴图文件、连接和绑定失败会报错。[控制检查](evidence/carton-appearance-ab-20260915/ab_controls.json) · [A 实际配置](evidence/carton-appearance-ab-20260915/A_capture_configuration.json) · [B 实际配置](evidence/carton-appearance-ab-20260915/B_capture_configuration.json)

A0 只作历史参考：上视角与 A 的同对象深度最大差 8.46 μm；下视角同对象像素最大差 9.42 μm，但两对象边缘发生 2 像素归属变化，另有 2481 个非纸箱像素的深度变化。历史全图最大差为 0.689 m，不能声称 A0/A 全图严格相同，也不把 A0→B 的变化全部归因于材质。[历史差异记录](evidence/carton-appearance-ab-20260915/historical_geometry_difference.json)

**简要量化：分割**

| 模组/组 | 固定 proposal / 可见 GT | SAM 输出 | 每个 GT 最佳 mask IoU 均值 | GT 最佳 IoU <0.5 | 合并预测 / 多预测覆盖 GT |
|---|---:|---:|---:|---:|---:|
| 上 A | 15 / 15 | 13 | 0.6773 | 4 | 2 / 2 |
| 上 B | 15 / 15 | 11 | 0.6367 | 4 | 3 / 3 |
| 下 A | 31 / 31 | 31 | 0.8257 | 2 | 4 / 4 |
| 下 B | 31 / 31 | 31 | 0.8322 | 2 | 4 / 4 |

这是 **oracle proposal 条件下的 SAM/几何实验**。A/B 每个模组复制相同固定 proposal 坐标和数量，仅更新采集绑定；没有自主检测结论。重叠关系用实际预测/GT 可见 mask 事后计算：至少 50 像素，且占两区域各至少 10% 才记为显著关系。一个预测显著覆盖多个 GT 记合并；一个 GT 被多个预测显著覆盖记拆分/重复候选，不能与已证明的物理拆分混同。未用 supporting_proposal_ids 代替重叠评价。

两个模组的所有可见 GT 都有部分 SAM 覆盖，但这不代表准确识别：6 个“模组×GT”最佳 IoU 仍低于 0.5，A/B 失败对象集合相同；合并、重复和身份歧义保留。最佳 IoU 不是强制一对一匹配成功率。

**简要量化：最终面片与真实边界**

| 模组/组 | 合格深度面片 | 越界 >10 mm | 越界均值 / 最大 mm | GT 可见面像素覆盖 | SAM 观测区域覆盖 |
|---|---:|---:|---:|---:|---:|
| 上 A | 18 | 8 | 13.44 / 105.98 | 88.46% | 85.79% |
| 上 B | 17 | 6 | 13.67 / 105.98 | 85.53% | 81.98% |
| 下 A | 32 | 9 | 15.10 / 147.37 | 76.19% | 79.52% |
| 下 B | 30 | 12 | 19.02 / 154.46 | 61.36% | 69.04% |

GT 面由精确相机射线与名义盒体入射面确定，再与实际渲染对象 mask 相交；小于 50 像素的面不作可见面评价。完整分母为上 37、下 55 个可见物理面，包含没有预测覆盖的面。上述覆盖按 GT 面像素加权；逐面等权覆盖为上 A/B 37.51%/36.46%、下 A/B 43.80%/37.71%，细窄面缺失没有隐藏。JSON 按物理对象/面提供 A/B 配对及全部预测，不按 SAM 编号配对。

SAM 轮廓对实际可见物理边的 2 px precision/recall：上 A **53.30%/63.74%**，B **52.56%/60.34%**；下 A **35.86%/55.66%**，B **43.52%/68.74%**。参考边由 GT 盒面投影且要求邻近实际可见区域；预测使用全部轮廓，遮挡/裁剪轮廓也会降低该物理边 precision，不能把它当作普通 mask 边界准确率。面片自身的物理边 recall 仅上 A/B 12.50%/17.40%、下 A/B 4.70%/3.87%：保守面片没有恢复完整四边。

未分类边上 A/B 为 34/37，下为 113/104；四类边证据均原样保留。上下最大内部平均残差分别不超过 0.523/0.187 mm，最大 P95 不超过 0.971/0.355 mm，原 3 mm/6 mm 门槛未变。下视角每组各 1 实例无合格面；全部组完整体接受数为 0，物理角点为 N/A。深度求解器、来源审计、融合阈值和 unknown 语义未修改，`planning_admissible=false`。

**接缝诊断与可直接查看的图片**

历史下模组 mask 10 定位到 `carton_l02_c03`，其与 l01/l03 的竖直间隙在浮点精度内为 0，正面共面。此编号仅用于定位旧案例。穿过下边接触处的原分辨率剖面，A/B 亮度范围约 1.08/1.37（0–255），两侧线性拟合的亮度阶跃约 −0.316/−0.306；depth 剖面逐点相同，拟合阶跃约 −0.129 mm。另两个接触案例亦没有因 B 获得明显深度阶跃。这说明这些采样位置仍缺乏强接缝线索，不能外推所有接缝都不可观测，也不足以在本阶段判定“几何细节一定比算法更重要”。

- 无叠加原始 RGB：[上 A](evidence/carton-appearance-ab-20260915/raw/A_module_0_upper.png) / [上 B](evidence/carton-appearance-ab-20260915/raw/B_module_0_upper.png)；[下 A](evidence/carton-appearance-ab-20260915/raw/A_module_1_lower.png) / [下 B](evidence/carton-appearance-ab-20260915/raw/B_module_1_lower.png)。
- 同视角同缩放 SAM：[上 A/B](evidence/carton-appearance-ab-20260915/evaluation/module_0_upper_sam_AB.png) / [下 A/B](evidence/carton-appearance-ab-20260915/evaluation/module_1_lower_sam_AB.png)。
- 从最终 JSON 绘制面片：[上 A/B](evidence/carton-appearance-ab-20260915/evaluation/module_0_upper_faces_AB.png) / [下 A/B](evidence/carton-appearance-ab-20260915/evaluation/module_1_lower_faces_AB.png)。青色 GT、绿色有支持边、橙色未知/遮挡边，不把面片角点当物理角点。
- 原分辨率 RGB/depth/SAM/面片局部对照：[历史下 mask 10](evidence/carton-appearance-ab-20260915/evaluation/module_1_lower_historical_10_native_AB.png)、[上 mask 6](evidence/carton-appearance-ab-20260915/evaluation/module_0_upper_historical_6_native_AB.png)、[下 mask 27](evidence/carton-appearance-ab-20260915/evaluation/module_1_lower_historical_27_native_AB.png)。
- [mask 10 接缝剖面](evidence/carton-appearance-ab-20260915/evaluation/module_1_lower_historical_10_profile.png) · [量化 CSV](evidence/carton-appearance-ab-20260915/evaluation/summary.csv) · [完整分母与物理对象/面配对 JSON](evidence/carton-appearance-ab-20260915/evaluation/summary.json)。

**运行、版本与复现**

全部渲染、推理、评价及测试在指定 GPU 进行。被测运行源码与 `294b007a3264543c7e89d07ba16c2d8f1a5c7759` 核验一致；渲染时 Git HEAD 仍为基线，文件等价核验及快进记录见 [tested_source.json](evidence/carton-appearance-ab-20260915/tested_source.json)。固定视觉上游仍为 manifest 的 `1d208f2ed380a207e6e46b4a62d2ac640edfe477`。在精确被测提交上 `pytest -q` 为 **390 passed、1 deselected（既有筛选）、15 warnings**。没有等待或扩大为 ROS/机械全套验收。一次过早汇总遇到 B 下视角尚未写出的文件，保留 [错误日志](evidence/carton-appearance-ab-20260915/evaluation-v1.log)；正式汇总读取全部已完成结果，没有重跑或替换 SAM。

服务器根目录为 `/root/autodl-tmp/v05-acceptance/appearance-ab-d3e92cd-20260915`，正式采集 `capture-v1`、交付评价 `evaluation-delivery`。本地查看目录是本报告旁 `evidence/carton-appearance-ab-20260915`。大型 USD、原始 depth、模型和完整推理缓存留在服务器，[清单](evidence/carton-appearance-ab-20260915/large_artifacts_manifest.json) 提供路径/哈希。Git 仅保存本阶段代码、配置、来源、少量图和紧凑结果；最终提交 SHA、同步状态与 CI 实际状态在完成回复列出。

复用现有环境，在服务器运行以下命令；`NEW` 必须为未使用的目录。本轮已完成，不自动再跑。

```bash
cd /root/autodl-tmp/pick-and-place-simulation-v0.5-perception-ros2
export OPENBLAS_NUM_THREADS=1 PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PWD/tools:."
F=/root/autodl-tmp/v05-acceptance/dual-rgbd-v4-221b296
NEW=/root/autodl-tmp/v05-acceptance/appearance-ab-NEW
GPU_PY=/root/v05-gpu-venv/bin/python
OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1 /root/autodl-tmp/envs/isaacsim-clean/bin/python scripts/isaacsim_perception_capture.py --bundle-directory "$F/scene_bundle" --project-root "$PWD" --feasibility-root /root/autodl-tmp/m710-official-dynamics-20260910/repo-feasibility-core-648e177 --usd-directory "$F/usd" --output "$NEW" --appearance-ab
"$GPU_PY" tools/run_carton_appearance_ab.py --capture "$NEW" --historical "$F" --vision /root/vision-fixed --models /root/autodl-tmp/v05-acceptance/model-manifest.json
"$GPU_PY" tools/evaluate_carton_appearance_ab.py --capture "$NEW" --historical "$F" --output "$NEW/evaluation"
"$GPU_PY" tools/diagnose_carton_seams.py --capture "$NEW" --historical "$F" --evaluation "$NEW/evaluation"
```

**暂停点：等用户查看外观与第一轮结果后，再决定是否运行 C/D。当前没有真实照片对照，也没有将理想深度结果冒称真实相机性能。**
