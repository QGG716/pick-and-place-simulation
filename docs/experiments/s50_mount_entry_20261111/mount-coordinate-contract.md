# 安装接触坐标契约修正

基线：`b7bb327`。本轮将安装参考、模型基座与 CAD 测量支承面分开；历史 v1 报告保持原样。

## 三个坐标系

- **安装参考 R**：冻结布局给出的世界安装位置和朝向，沿用 `[-1.85, 0.15, 0.90] m`。
- **模型基座 B**：由当前机器人配置显式声明 `R_from_B = binding.mounting_from_base`。运行时严格锁定 `W_from_B = W_from_R @ R_from_B`。
- **CAD 支承面 C**：由实际源模型测量得到 `B_from_C`。它的原点是支承面与安装轴的交点，Z 轴是支承面法向；其 X/Y 切向轴不代表制造商安装 clocking。

旧规则比较 `W_from_B @ B_from_C == W_from_R`，会把 CAD 支承面的切向坐标选择也强加为安装条件。v2 改为锁定声明的实际 `W_from_B`，同时独立检查 CAD 支承面原点、安装轴与安装参考一致，以及法向、支座顶面、接触区域和非穿透条件。允许切向坐标不同不等于允许机器人自由转动：任何未在绑定中声明的实际 yaw、平移或下沉仍被拒绝。

契约使用 `source_bound_fixed_base_plane_v2`，包括：

| 字段 | 含义 |
|---|---|
| `expected_world_from_installation_reference` | 冻结的 `W_from_R` |
| `declared_installation_reference_from_model_base` | 配置声明的 `R_from_B` |
| `expected_world_from_model_base` | 二者乘积，且必须与实际模型基座一致 |
| `model_base_from_cad_support_frame` | 来源模型的 `B_from_C` |

声明的映射必须与不可变 model binding identity 一致。CAD 支承 frame 的原点/Z 轴必须与其测量平面字段一致；只放开切向 X/Y 比较。旧 v1 契约不能被新代码静默重新解释。

## 保留的检查

底座必须固定到世界，只可能允许固定 base 与原 `mount_plinth` 的接触。原支座几何、接触区域覆盖、全底座半空间非穿透及同哈希源点的既有离散偏差规则不变。所有活动连杆仍没有安装接触许可。Python/Coal 与 MoveIt 实现同一 v2 判定。

真实 S50 安装 clocking 仍未知。本轮没有填写制造商参数，没有把接触证明改成正式任务准入。非零 yaw 只存在于输出目录中的 `synthetic-yaw90-only.yaml`，明确标为 `SYNTHETIC_INSTALLATION_MAPPING_PROTOCOL_ONLY`、`manufacturer_mapping=false`、`task_qualified=false`；其使用真实 CAD 网格来验证坐标传输，不构成 S50 安装证据。

## 已运行与复现

本地定向回归：`39 passed`。覆盖声明非零 yaw、未声明 yaw/平移/下沉、模型基座与测量支承面原点不同的解析 fixture、声明倾斜仍违反法向/平面条件、绑定映射篡改、原源点规则、缓存几何变化撤销及 FANUC 原支座规则。

本地 probe 输入生成已运行：原正常接触通过，下沉负例拒绝；合成声明 yaw90 通过，合成未声明额外 yaw、平移、下沉全部拒绝。活动连杆案例的安装结果只表示固定 base pair 可用，不能将其解读为整个姿态合法。

```bash
PYTHONPATH=src python -m pytest tests/test_s50_mount_contact.py tests/test_joint_search_layout.py -q
python tools/build_s50_mount_contact_probe.py --coal --synthetic-mapping-cases --output outputs/s50_mount_entry_20261111/mount
source /opt/ros/humble/setup.bash
cmake --build build/moveit --target m710_moveit_worker s50_mount_contact_probe -j2
build/moveit/s50_mount_contact_probe outputs/s50_mount_entry_20261111/mount/native-input.json > outputs/s50_mount_entry_20261111/mount/native-report.json
```

原生输入已在服务器重新生成，新 worker 和 probe 已重建并实际运行。8 个案例的 Python/Coal 与原生接触许可完全一致，56 对 link/plinth 原始相交结果全部一致：正常接触和声明 yaw90 通过，真实下沉及未声明 yaw/平移/下沉全部拒绝。

只读复核原姿态与合成映射下的两个活动连杆负例，均记录 `s50_link_2 / mount_plinth` 的 `raw_intersects=true`、`permission_applies=false`；原生均保留 8 个采样接触。固定 base 局部许可未掩盖活动连杆碰撞。这些记录没有将合成映射认定为制造商参数，也不代表整个姿态、正式任务或动力学合格。详见 [mount-comparison.json](mount-comparison.json) 与 [native-mount-report.json](native-mount-report.json)。
