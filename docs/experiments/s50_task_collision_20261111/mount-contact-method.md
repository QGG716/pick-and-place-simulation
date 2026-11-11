# S50 固定底座安装接触规则

本轮只判断原支座与固定底座的几何接触。预览模型的安装转角仍属于 CAD 仿真约定；`installation_verified=false`，通过此规则不代表正式任务模型、安装资格或动力学已经通过。

## 输入与不变量

- 原冻结安装面坐标为 `[-1.85, 0.15, 0.90] m`。原 `mount_plinth` 的世界范围是 X `[-2.1885, -1.625]`、Y `[-0.125, 0.425]`、Z `[0.60, 0.90] m`，不移动或改形。
- S50 支承面来自原 STEP 的六个平面面元 17、18、19、20、21、28；安装轴由首关节实测轴线与支承面求交。`base_from_measured_mount` 明确保留 URDF 原点与实测安装面之间的关系。
- 验证源文件、实际底座 collision mesh 的哈希及 URDF collision 变换。读取实际三角形支承面区域，要求支承面法向相对、完整接触区域位于原支座顶面内、接触面积大于零、底座任何其它部分不进入支座。
- 固定底座必须沿 URDF 父链固定到世界。唯一可返回的 pair 为 `s50_base / mount_plinth`；所有活动连杆、工具和其它环境对均保留原规则。

## 原始离散偏差

原始 OBJ 中只有第 14085 个点比支承面低 `0.54 µm`。原转换参数重新生成该离散点后，坐标误差为 `2.1566 nm`；它到真实 STEP 支承面 19 的最近点保持相同 X/Z，距离 `0.540000000027 µm`。原 CAD 完整底座的精确边界下界相对支承面只存在约 `0.1 nm` 数值偏差。

该点不满足“最近边距离不超过该边自身容差”的证明，因此没有沿用先前的最大 CAD 边容差解释。来源记录明确保留 `local_tolerance_qualification_passed=false`，归因为可复现的源网格离散偏差。

规则只识别同一源网格哈希中的这个点。许可深度由该实际源点相对支承面的数值直接算出，不采用 `100 µm` 网格化弦高、不增加一般穿透容差、不改网格。整个安装变换仍必须等于冻结安装变换；额外低点、底座真实下移、支座改变和活动连杆碰撞都不能使用此许可。原始 mesh 的相交结果仍写入诊断记录。

## Python、Coal 与 MoveIt

`mount_contact.py` 绑定来源证明和固定场景，`qualified_mount_pairs` 只在证明通过时返回上述固定 pair。精确后端加载后，半空间证明使用 Coal 实际加载的网格顶点；源网格三角形用于支承面覆盖证明。

Coal 模型 revision、实际固定底座 BVH 序列化内容（含网格顶点及拓扑）、collision placement 或实际缓存顶点对象改变时，撤回缓存的安装许可并报告 `MOUNT_COLLISION_GEOMETRY_CHANGED`。不使用可能临时重建的 Python/Boost wrapper 地址作为 shape identity。数值接口也明确拒绝空网格、维数错误与 NaN/Inf 输入。

`model_request` 将同一份安装接触契约纳入不可变 worker identity。原生 `mount_contact.h` 使用 MoveIt 实际加载的底座 collision 顶点及三角形重新计算证明，`ProcessPolicy` 拒绝未通过证明的 ACM 许可。FANUC 历史安装分支保持不变。

`s50_mount_contact_probe` 独立读取真实模型，只测接触规则及原始 FCL link/plinth 对，不求 IK、不规划、不执行。测量用局部 ACM 仅隔离待测 pair，从不写回任务场景。负例姿态 J2=`2.4 rad` 来自本轮 Coal 的有限碰撞诊断选取，不是机械限位或合法任务姿态。

## 复现

```bash
python tools/diagnose_s50_runtime_collision.py --mount-cases-only --output outputs/s50_task_collision_20261111/mount-cases
python tools/build_s50_mount_contact_probe.py --coal
source /opt/ros/humble/setup.bash
cmake -S ros2/m710_moveit_backend -B build/moveit -DCMAKE_BUILD_TYPE=Release
cmake --build build/moveit --target m710_moveit_worker s50_mount_contact_probe -j2
build/moveit/s50_mount_contact_probe outputs/s50_task_collision_20261111/mount/native-input.json > outputs/s50_task_collision_20261111/mount/native-report.json
python -m pytest tests/test_s50_mount_contact.py tests/test_joint_search_layout.py tests/test_configured_native_entry.py tests/test_robot_model_binding.py tests/test_moveit2_backend.py -q
```

2026-11-11 已实际取得：本地最后一条定向回归 `82 passed, 1 skipped`（本地旧解释器缺 Pinocchio）。原生 worker 和接触 probe 已在服务器实际构建；原生 process-policy 24 例及使用正确输入文件的 FANUC gap 合同回归通过。

真实 Coal 与 MoveIt/FCL 对三个安装情形中的 21 个 link/plinth pair 的相交结果及许可范围完全一致：正常底座接触局部通过；独立负例下移 1 mm 被拒绝；J2=2.4 rad 出现 `s50_link_2 / mount_plinth` 碰撞且两端都没有给予活动连杆许可。原始 mesh 相交结果完整保留。两端底座最小支座距离一致，支承面面积差 `6.5544e-10 m²`；这不声称不同 mesh reader 产生的所有顶点逐位相等。详细结果和来源哈希见 [mount-runtime-comparison.json](mount-runtime-comparison.json)。

缓存签名曾使用临时 wrapper 地址而把未改变的实际 Coal 网格误拒，相关失败输出保留。改为实际 BVH 内容 SHA 后，最终服务器复测正常接触、下移负例和两个有限 J2 探针全部完成，结果文件 SHA256 为 `46592e9c2ccc4038e860361d1142942de9096fe274fc3950ef24dcbba065230f`，与之前真实几何成功记录完全相同。此修复不改变源网格、物理场景或任何碰撞许可范围。
