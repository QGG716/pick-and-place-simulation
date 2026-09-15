# 完整纸箱 USD 接入：原图待确认

本轮替换完整 40 箱的视觉资产，保留原 8 层 × 5 列、0.6 × 0.4 × 0.3 m 名义尺寸、实体 ID、根位姿、相机标定和机械布局。**只进行原图交付，外观尚待用户确认；没有运行 SAM、训练、规划或 ROS 全量验收。`planning_admissible=false`。**

| 正式传感器输入 | 同场景与原像素局部 |
|---|---|
| [上模组原始 RGB，2592×1944](evidence/carton-usd-20260915/upper.png) | [整体视图](evidence/carton-usd-20260915/overview.png) |
| [下模组原始 RGB，2592×1944](evidence/carton-usd-20260915/lower.png) | [上部接触](evidence/carton-usd-20260915/upper_contact.png) · [下部接触](evidence/carton-usd-20260915/lower_contact.png) · [扫描箱接触](evidence/carton-usd-20260915/scan_contact.png) |

38 箱引用 NVIDIA Isaac Assets 5.0 的 `SM_CardBoxD_04.usd`，保留完整倒角网格、原 UV、纸板/胶带贴图、法线/ORM 和 MDL 材质关系。2 箱引用 1057822006-svg v0.1 摄影测量 `BOX_30X17X22.usdc`，固定在 `carton_l02_c02` 和 `carton_l07_c04`，纸板、胶带、标签仍属于原来的一个逻辑实体。来源、许可原文措辞、型号取舍和修改见 [资产说明](../../assets/materials/industrial/CARTON_USD_SOURCES.md)，14 个完整依赖的 URL/归档成员/哈希见 [固定配置](../../configs/isaac/carton_assets.json)。大型资产不在 Git 中分发。

NVIDIA 文档的 WarehousePile_A04 实际入口为 `WarehousePile_A4.usd`。已检查其可分离单箱，但较方的型号不适合冻结尺寸，采用独立官方 D04。扫描 01/04 未纳入。扫描 05 的局部坐标轴按原网格主面方向校正；拟合使用**变换后的实际可见顶点**，避免旋转 AABB 的保守外扩把箱体整体缩小。原箱位和间距没有改变；局部凹陷、折边和倒角来自资产本身。

照明、曝光及正式相机保持原配置，仅将独立展示相机放到车厢开口前方，便于同时查看箱堆、机器人和输送机。局部图是 PNG 原像素裁切，没有缩放、描边、锐化或生成式处理；[裁切坐标](evidence/carton-usd-20260915/crops.json) 可复核。原有机械遮挡如实保留。扫描箱的纸板/胶带颜色和轻微表面不规则与官方箱不同，最终观感由用户确认。

检查覆盖每个箱体的根位姿、名义尺寸、实际顶点包络、隐藏碰撞代理及多 mesh 实例 mask 并集。RGB、光学 Z 深度和实例标注来自同次新采集，哈希绑定到独立采集 epoch。名义八角点/六平面仅为长方体近似，详细网格的真实物理角点/面未评价。[基本检查](evidence/carton-usd-20260915/basic_checks.json) · [采集配置](evidence/carton-usd-20260915/capture_configuration.json) · [新 manifest](evidence/carton-usd-20260915/manifest.json)。

起始提交为 `2f2d6286b58566b38bad50c6669963db1f2de5d5`，精确被测/采集提交为 `08748ffeeaa559ad4de206dd13de2b7da18f93b1`，采集时工作树干净。GPU 上 `pytest -q`：**393 passed、1 deselected（既有默认筛选），5.98 s**；基本一致性检查通过，渲染日志无 `[Error]`。见 [运行凭据](evidence/carton-usd-20260915/run_receipt.json) 和 [测试日志](evidence/carton-usd-20260915/pytest.log)；最终交付提交及 CI 实际状态在交付回复列出。历史 A/B 和本轮接入调试目录均保留，不覆盖。

复现（指定 GPU 的已有环境，`NEW` 必须是新目录）：

```bash
cd /root/autodl-tmp/pick-and-place-simulation-v0.5-perception-ros2
export OPENBLAS_NUM_THREADS=1 PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PWD/tools:."
ISAAC_PY=/root/autodl-tmp/envs/isaacsim-clean/bin/python
CACHE=/root/autodl-tmp/v05-assets/cartons/mirror
F=/root/autodl-tmp/v05-acceptance/dual-rgbd-v4-221b296
NEW=/root/autodl-tmp/v05-acceptance/carton-usd-NEW
"$ISAAC_PY" tools/fetch_carton_assets.py --cache "$CACHE" --config configs/isaac/carton_assets.json
OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1 "$ISAAC_PY" scripts/isaacsim_perception_capture.py --bundle-directory "$F/scene_bundle" --project-root "$PWD" --feasibility-root /root/autodl-tmp/m710-official-dynamics-20260910/repo-feasibility-core-648e177 --usd-directory "$F/usd" --output "$NEW" --carton-assets configs/isaac/carton_assets.json --asset-cache "$CACHE"
"$ISAAC_PY" tools/verify_carton_capture.py --capture "$NEW" --source-manifest "$F/scene_bundle/FULL_STACK_NOMINAL.manifest.json"
/root/autodl-tmp/v05-acceptance/cpu/venv/bin/python -m pytest -q
```

等待用户明确确认这组原始画面后，再单独运行现有 SAM＋深度约束求解器。
