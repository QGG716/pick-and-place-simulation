# M-710iD/70 真实 CAD、运动学与工程动力学轮次

> 历史记录（2026-09-09）：本页描述切换官方 FANUC 模型之前的代理/CAD 门控。
> 当前实现与结论已由
> [官方模型与独立吸盘验证轮次](m710id70_official_model_independent_cups_round.md)
> 取代；本页的“缺少14个网格”、60杯门限和 `NOT_RUN_PER_USER_REQUEST` 不再是当前状态。

日期：2026-09-09
分支：`feat/v0.5-feasibility-core`

## 结果

本轮完成了真实资产来源审计、冻结布局下的单箱严格搜索、独立工程动力学合同、
动态场景预检、内容绑定的 M-710 Isaac 回放适配和纯函数物理策略。结果保持
**fail-closed**：当前没有满足执行门限的真实分连杆碰撞网格，也没有完整单箱轨迹，
所以不启动物理执行。按用户本轮
指令，Isaac 状态明确为 `NOT_RUN_PER_USER_REQUEST`，没有生成 MP4、关键帧或实际
状态日志。上一轮静态布局视频不能作为本轮动作结果。

| 层次 | 本轮结果 | 资格边界 |
| --- | --- | --- |
| FANUC 原始 CAD | `PASS`：17 文件、29,450,224 bytes，格式/哈希/ZIP CRC/展开副本一致 | 仍缺 7 个 link visual 和 7 个 CAD-derived collision mesh |
| 皖泰吸具资产 | STEP、GLB、STL、质量分析和转换脚本哈希一致 | 当前整形 STL 含柔性吸盘且非最终刚体碰撞表示 |
| 新布局单箱运动学 | 顶层 5 箱，完整轨迹 `0/5` | 严格覆盖和 IK 不放宽；`NO_IK` 仅表示预算耗尽 |
| 工程动力学输入 | CPU 严格校验通过 | `ENGINEERING_ESTIMATE_NOT_MACHINE_QUALIFIED` |
| Isaac / 视频 | `NOT_RUN_PER_USER_REQUEST` / 未生成 | 不用代理碰撞、瞬移或无限约束制造录像 |

因此，本轮没有完成或声称完成：Isaac 中 40 箱落稳、非零关节 CPU/Isaac FK/TCP
对照、逆动力学/驱动反力与跟踪误差测量、真实接触吸附/释放、带上传送、实际状态
日志、关键帧、单箱 MP4 或 2–3 箱串行执行。这些是后端实跑验收项，不能由配置
校验代替。

## 真实资产与未解决的机构语义

机器人源文件保留在 `res/M-710iD_70/`。审计确认核心文件是真实 Parasolid v29
`x_t`、JT 9.5、DXF 和 ROBCAD/Process Simulate 数据，不是 Git LFS 指针，也没有
通过改扩展名伪造转换。ROBCAD 的 21 个刚体已映射为 `k1..k7 -> base_link,
J1_link..J6_link`，关节轴与当前链一致；但 `J3 follows J2 by 1` 的控制器到运动学
解释和 tool0 绕公共工具 Z 的 180°差异仍未解决。本机没有经许可且确实支持
Parasolid/JT 的分连杆转换器，因此 14 个预期执行网格保持缺失，真实基座前缘也
不能由代理半径替代重算。

吸具权威几何仍是 `res/上海皖泰真空吸盘三分区.STEP`。新的完整 SE(3) 帧合同区分
三个共轴帧：法兰到未压缩吸盘极面 0.2275 m，压缩 15 mm 后名义接触面 0.2125 m，
虚拟任务 TCP 为 0.2500 m；因此虚拟 TCP 比未压缩工作面外伸 22.5 mm、比名义压缩
接触面外伸 37.5 mm。子帧局部 `+Z` 通过 `R_y(+90 deg)` 映射到法兰 `+X`。任务候选
先在箱体物理表面生成，再转换为虚拟 TCP 做严格 IK/FK 残差；覆盖、接触例外、实际
附着与证据则转换回物理接触面。附着不能因为虚拟 TCP 到达箱面而隔空建立，tool0
绕公共工具 Z 的 180 度差异在解决前仍使执行资格失败。

## 严格运动学实跑

新任务集合只来自冻结场景 `SupportRelationGraph.removable_cartons`：

`carton_l07_c02, carton_l07_c01, carton_l07_c03, carton_l07_c00, carton_l07_c04`

全部 40 箱、底盘和两条传送带均保留；接收面固定为 `conveyor_transverse`，未启用
base scan、升降、伸缩或 conveyor Z 优化。本轮没有运行旧 104/129 集合，它们不是
此布局的验收分母。

| 统计 | 值 |
| --- | ---: |
| task / candidate pose | 5 / 48 |
| 吸盘覆盖拒绝 | 34 |
| IK 调用 / 实际 seeds | 14 / 322 |
| IK 迭代 | 57,960 |
| 收敛 / 有效 / 去重解 | 0 / 0 / 0 |
| 路径连接尝试 / 完整轨迹 | 0 / 0 |

front 面最多覆盖 48 杯，低于既定 60 杯门限；其余 14 个通过覆盖门的 top/side
候选耗尽确定性 IK 种子流后仍无严格解。该状态是
`BUDGET_EXHAUSTED_NOT_INFEASIBILITY_PROOF`，不是数学不可达证明。即使未来获得
IK，完整轨迹仍须通过真实 CAD 碰撞资格门。

任务级失败为 `5 x NO_STRICT_GRASP_IK`；候选级失败为
`34 x INSUFFICIENT_SEALED_CUPS` 和 `14 x NO_IK`。候选数量不会重复计入任务数。
此外，layout-bound 的 CAD-qualified 完整路径连接器本轮尚未接入；若 CAD 和严格
grasp 门未来先行通过，当前代码会显式返回
`CAD_QUALIFIED_PATH_CONNECTOR_NOT_IMPLEMENTED`，不会把代理路径误报为完整轨迹。

## 工程动力学与回放适配

独立配置显式给出：机器人机械单元总质量 580 kg、固定工具贡献 20 kg、40 个箱各
42.5 kg（配置总质量 2300 kg）；工具质量和惯量虽然有独立来源，但按
`FIXED_TOOL_COMBINED_INTO_J6_RIGID_BODY_EXACTLY_ONCE` 合并进 `J6_link`，动态机器人
装配总质量为 600 kg，不再创建第二个工具动态刚体。所有刚体质量为正、惯量
有限/对称/SPD 并满足刚体三角不等式。580 kg 锚定 FANUC 官方机械重量，但分连杆
质量/惯量由当前 URDF 代理
体积按总重缩放，驱动力和 PD 增益、摩擦、阻尼、纸箱惯量以及真空阈值均为可追溯
工程估计，不是厂家驱动或真机安全资格。官方参考：
[FANUC M-710iD/70 产品页](https://www.fanucamerica.com/products/robot/m-710id-70)、
[数据表](https://www.fanucamerica.com/docs/default-source/robotics-files/m-710id-70-data-sheet.pdf)。

动态场景合同包含 3 个固定装配件、40 个独立动态箱，以及物理地板和两侧壁的有限
求解器 patch。只声明已知 `Z=0` 地板和 `Y=±1.15 m` 侧壁；车厢长度/高度仍为
`null / NOT_DEFINED`。横带表面方向为 `-Y`，纵带为 `-X`。回放适配器读取合同时间
步、有限驱动力、质量/惯量、材料、阻尼和求解器迭代，避免重复地板，并将传送带
延迟到释放撤离后启动。时间映射把预抓取稳定、真空建立和释放保持时长计入对应
事件，且强制 `grasp <= release <= release_retreat`。

SurfaceGripper 的附着点位于名义压缩后的物理杯面。调用后端 `close_gripper` 前，
纯函数审计会检查所有活动吸盘射线、signed gap、2 mm 最大接近距离、0.2 mm 最大
数值穿透和 5 度法向偏差；任一项失败都拒绝附着。SurfaceGripper 只执行其实际支持
的有限轴向/剪切门限；未被后端施加的扭矩脱落阈值不得报告为完整 6D wrench
qualification。

传送带控制为逐物理步、基于箱体中心和 OBB 足迹的单表面选择。转角重叠时保留当前
表面，切换时先全部关闭再只启用一个表面；未知或不在任何带面时全部关闭。日志只按
实际激活表面的方向统计输送进度，并记录最大同时激活表面数，不能再从所有配置方向
中选择最大位移制造输送证据。

M-710 replay 在导出和执行两端均要求一个内容指纹正确且状态为 READY 的完整 preflight。
执行脚本在导入或实例化 `SimulationApp` 之前核验 bundle 整体 payload、内嵌 preflight、
冻结 plan/config/scene/唯一 trajectory、当前源码和当前资产审计；BLOCKED preflight
无法导出。SHA-256 的作用仅是内容完整性与陈旧性检测，不是签名、授权或来源认证。

`simulation_execution_ready` 与厂家资格分开：缺少厂家连杆惯量、认证驱动力和
实测真空包络只产生 `machine_qualification_warnings`，不会永久禁止未来工程仿真；
真实分连杆 CAD、机构坐标、工具动态碰撞、基座定位或完整轨迹缺失仍是
`simulation_readiness_blockers`。本轮完整 blocker 是：

- `DYNAMIC_COLLISION_REPRESENTATION_NOT_QUALIFIED`
- `EXECUTION_COLLISION_GEOMETRY_NOT_QUALIFIED`
- `FLANGE_CLOCKING_PROVISIONAL`
- `J3_FOLLOWS_J2_CONTROLLER_TO_KINEMATIC_MAPPING`
- `NO_STRICT_GRASP_IK`
- `PER_LINK_COLLISION_MESHES_MISSING`
- `PER_LINK_VISUAL_MESHES_MISSING`
- `REAL_BASE_MOUNTING_EXTENT_NOT_AVAILABLE`
- `TOOL0_CLOCKING_180_DEG_ABOUT_COMMON_TOOL_Z`

当前仿真执行资格和厂家资格均未通过。

## 复现

本轮实际执行的 CPU 命令：

```powershell
.\.venv\Scripts\python.exe tools\run_m710id70_layout_single_carton.py `
  --config configs\validation\m710id70_layout_v1_single_carton.yaml `
  --output docs\validation\evidence\m710id70_dynamic_execution_v1\single_carton_motion_audit.json

.\.venv\Scripts\python.exe tools\prepare_m710id70_dynamic_execution.py `
  --config configs\simulation\m710id70_dynamic_execution_v1.yaml `
  --motion-result docs\validation\evidence\m710id70_dynamic_execution_v1\single_carton_motion_audit.json `
  --output docs\validation\evidence\m710id70_dynamic_execution_v1\dynamic_execution_preflight.json

.\.venv\Scripts\python.exe tools\archive_m710id70_dynamic_execution_evidence.py `
  --motion-result docs\validation\evidence\m710id70_dynamic_execution_v1\single_carton_motion_audit.json `
  --preflight docs\validation\evidence\m710id70_dynamic_execution_v1\dynamic_execution_preflight.json `
  --output docs\validation\evidence\m710id70_dynamic_execution_v1\run_summary.json

.\.venv\Scripts\python.exe -m pytest -q --basetemp .tmp\pytest-full-20260909b
```

最终完整回归结果为 `386 passed, 4 deselected in 199.42s`。4 项 deselected 来自仓库
既有的显式选择策略，不是本轮隐藏或放宽失败。

证据目录中的 motion/preflight 都有内容指纹；`run_summary.json` 另记录生成命令、
运行环境、生成时 Git 状态、实际参与源码哈希及文件 SHA-256。三份证据的互相身份
核验为 `PASS`：

| 对象 | 内容身份 / SHA-256 |
| --- | --- |
| motion evidence fingerprint | `98f963f752712b77f740ec58beab426e51b420a4c4fce33e46d51ec0c1a42a74` |
| motion file | `cbae68fe9118515ac1b7a01b8c7b1ba334176889ee482f2a90f35ebb796e8ae5` |
| preflight fingerprint | `b1c3bc946051e4d693b3d3c51de53f39b02177c44c3965818a25acf99ec685d8` |
| preflight file | `22b1d1a55e98a70aeb49a1e9f92c759991eab2c5f14fb1218f87249802ce5a9f` |
| execution asset fingerprint | `d44bb8cdae825b8999e1ec8073bcb4404436233ec4e2734aaf67efede50af33b` |
| aggregate evidence identity | `798c8eee00b61b4b24900d36208a7fb2caff94d3c418a6bfa3f7aa0a61a1ed8f` |
| run summary fingerprint | `2d01d3338e3c105522c88b7aa905bf2e00913feff0b59214e7c268d1f1eb5996` |
| run summary file | `fd141285e80f7035ba59e80cad2a4dc2a298374790fe68e8885b515c45f640c4` |

只有预检变为 READY 后，才可由它导出内容绑定的 bundle：

```powershell
.\.venv\Scripts\python.exe scripts\export_isaac_fanuc_replay.py `
  --preflight docs\validation\evidence\m710id70_dynamic_execution_v1\dynamic_execution_preflight.json `
  --output outputs\m710id70_layout_v1\isaacsim\qualified_replay_bundle.json
```

当前 BLOCKED preflight 执行该导出会失败且不产生文件，这是已验证的预期行为。仅在
导出成功后，才允许在已接受 NVIDIA EULA 的隔离 Isaac 环境中执行：

```bash
OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH="$PWD/src" \
python scripts/isaacsim_fanuc_replay.py \
  --bundle <qualified-m710-replay-bundle.json> \
  --project-root "$PWD" \
  --usd-directory outputs/m710id70_layout_v1/isaacsim/usd \
  --output outputs/m710id70_layout_v1/isaacsim/single_carton \
  --record-video
```

当前运行该命令会在启动 `SimulationApp` 前被资格门拒绝，这是预期结果。本轮没有
执行该命令。
