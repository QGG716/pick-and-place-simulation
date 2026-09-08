# M-710iD/70 V3 可行性核心加固记录

日期：2026-09-08

工作分支：`feat/v0.5-feasibility-core`

起始提交：`c6fa45595833088319e342c7ef61cc94afef3f58`

本轮只处理四个已核实的正确性问题：模型资产内容变化未进入任务缓存身份、配置入口缺少完整数值/语义校验、接触路径终点未经重新闭合验证就创建附着、escape 搜索只看首个 station 且受方向/旋转顺序和“短于直退”条件系统性剪枝。本轮没有改变原始场景、分母、安装位姿、碰撞余量、IK/FK 容差或全局搜索预算，也没有启动动态传送带、升降轴或连续 129 箱验收。

## 基线与复现条件

修改前在干净的起始提交上重新生成了固定传送带原始 104 任务基线，没有沿用历史输出：

```powershell
.venv\Scripts\python.exe tools\run_m710id70_v3.py `
  --phase grid-fixed --workers 12 `
  --output-dir .tmp\review_c6fa455_baseline
```

环境为 Windows 11、CPython 3.14.7、NumPy 2.5.2、PyYAML 6.0.3、pytest 9.1.1。任务生成使用配置中的 `seed=71070`，固定传送带状态、104 个有效 grid 样本和每任务派生种子均保持不变。关键输入内容摘要如下：

| 输入 | SHA-256 |
| --- | --- |
| `configs/validation/m710id70_v3.yaml` | `b6b5dcfb70c71a47cc59fd50fcd189b635140892664aeda416cdb2d70e4763dc` |
| `configs/robots/fanuc_m710id_70.yaml` | `ad5c8771d5324073bb66fc5e9cf176eb0b2fb6e8c39540f7b73b7ddca387b12e` |
| `assets/robots/fanuc_m710id_70/m710id_70.urdf` | `7f1ec9ac66d520b5210533a6fcfaf68b5f3dd12e59972964358192459d0048b3` |
| `configs/tools/unloading_gripper_20kg.yaml` | `724a68677f768817465ea8391e24385d62a7523258ac2a1ec435567baf91b64a` |

基线测试为 `259 passed, 2 deselected`。基线任务缓存只有代码/配置等摘要，没有 cache schema 或模型资产内容指纹，因此不能作为本轮实现的有效缓存输入；基线结果仅用于同输入的前后对比。

## A. 模型资产指纹和缓存失效

`ValidationConfig` 现在生成 `m710_model_assets_v1` 清单。参与语义指纹的内容包括实际加载的机器人模型 YAML、URDF、解析到的物理工具 YAML、负载证据 YAML、实际工具碰撞 OBB、实际 self-collision 排除索引以及碰撞后端标识。当前 CPU 后端实际使用 URDF collision primitives、确定性 link capsules 和工具 OBB；SRDF 路径会作为“声明但未加载”的诊断信息记录，不会错误地要求未启用的 SRDF 或 mesh 存在。

文件内容采用 SHA-256；YAML 同时采用键排序后的规范 JSON 语义摘要。绝对路径只作诊断，不进入资产语义指纹，mtime 和字典枚举顺序也不参与身份。必需资产缺失、不可读或不是预期映射时会在加载阶段按资产角色和路径明确失败。

任务缓存升级为 `m710_task_cache_v2_model_assets`，验证策略为 `strict_contact_escape_scheduler_v2`。缓存键保留原有代码、有效配置、场景箱体及位姿、机器人状态、传送带状态、种子、模式和抓取筛选维度，并新增模型资产指纹与数值运行时身份。结果证据同步写入这些字段。缺少新 schema 或资产指纹的旧缓存会自然 miss，不删除历史文件。

运行时规则是：CPython 实现与 major/minor、NumPy 版本和碰撞后端影响缓存有效性；完整 Python、NumPy、PyYAML 版本均写入证据。PyYAML 版本本身不单独使缓存失效，因为其解析结果已经由有效配置和 YAML 语义摘要覆盖。

本轮默认资产语义指纹为：

```text
99fb9af6519ebfa4e4f4a09e7b9e84945d6ebe6fac4bc2bf64495e3a2a9a2dff
```

## B. 配置数值与语义校验

现有 `load_validation_config()` 仍是唯一入口，但在创建机器人或进入规划前完成校验：

- 拒绝 NaN、Inf、bool 冒充整数/数值、错误维度和非法上下界；
- 对几何尺寸、插值步长、采样周期、速度和求解参数分别执行严格正值或非负值约束；
- 校验概率、箱体 COM fraction、传送带 Z/extension 跨字段关系、吸盘数量和整数预算；
- 校验机器人六关节向量、模型关节限制/速度/加速度/jerk 维度与有限性；
- 校验已构造的 chassis、mount、TCP 齐次变换和 SO(3)；
- 保留抓取面内偏移、姿态微调和 escape 旋转的合法有符号值，也保留 `escape_path_attempt_limit=0` 的显式关闭语义。

非法配置以字段名和原因抛出 `ValueError`，不再伪装成任务不可达。

## C. 实际接触终点闭合

候选阶段仍保留快速严格预筛选，但不再创建 `RigidAttachment`。接触路径完成后，`Cell.validate_contact_endpoint()` 以 `contact_path[-1]` 重新计算实际 FK，并用同一实际 TCP 和当时目标箱世界位姿重新验证吸盘覆盖、接触位置/法向/姿态以及带 target-contact 语义的完整碰撞状态。

只有这些检查全部通过后才捕获 `tcp_from_box`。新的不可变 `ContactState` 保存最终关节状态、实际 TCP/箱体位姿、覆盖与碰撞证据、资产指纹和实际附着变换；support release、extraction 与载荷接触计算复用该状态。附着瞬间箱体世界位姿连续性也作为数值证据写入，阈值为 `1e-12`。失败终点不会生成或遗留 attached 状态。

## D. escape 搜索覆盖与预算

几何 proposal 现在是惰性确定性迭代器，实际机器人验证仍使用原有全局 `escape_path_attempt_limit=4`：

1. 在多个 constrained-straight station 和 lift/left/right 等合法方向间轮询；
2. 小预算先覆盖新的 station/方向，并及时回填新方向在较近 station 的候选；
3. 所有 station/方向的零旋转层优先于同一 pair 的旋转细化；
4. 几何路径长度保留为排序/诊断证据，不再用“必须短于 pure straight”作为可行性门槛；
5. 每个 proposal 仍经过实际 IK、机器人/工具/附着箱体完整路径碰撞、support-release 和 initial-proximity 恢复检查；
6. pure straight 是 escape 预算之外的必选回退，但仍走原有严格笛卡尔路径验证。

每个实际尝试记录 station/direction/rotation、schedule index、几何查询数、路径长度、验证阶段、拒绝原因和预算。终止值区分 `SUCCESS`、`SEARCH_BUDGET_EXHAUSTED`、`CANDIDATES_EXHAUSTED` 与 `NO_GEOMETRIC_CANDIDATES`。数值 IK 或路径搜索耗尽仍是“未证明不可行”，不能解释成几何不可行性证明。

## 回归和真实链路验证

```powershell
.venv\Scripts\python.exe -m pytest -q `
  --basetemp .tmp\pytest-hardening-final `
  tests\test_v3_validation_hardening.py
# 23 passed in 2.53s

.venv\Scripts\python.exe -m pytest -q `
  --basetemp .tmp\pytest-full-final
# 282 passed, 2 deselected in 18.40s

.venv\Scripts\python.exe -m pytest -q `
  --basetemp .tmp\pytest-slow-final `
  tests\test_v3_motion_contract.py -m slow
# 1 passed, 21 deselected in 21.26s
```

定向测试覆盖同路径 URDF collision 内容变化、实际工具碰撞资产变化、仅 mtime 变化、规范序列化、缺失资产、旧缓存失效、参数化非法配置、合法有符号值/零预算、实际接触终点拒绝与连续附着、确定性 escape 调度、跨 station/方向继续搜索、保留较长绕行和真实笛卡尔路径检查。slow witness 是真实 `grid_022` 机器人链路，不是 FakeRun：首个 escape 候选被严格约束拒绝，后续候选成功，并完整通过抓取、接触闭合、附着、support release（如需）、extraction、transit、place 和 withdrawal。

pytest 首次使用系统默认 `%TEMP%` 时因现存 `pytest-of-liukai_xd` 目录 ACL 返回 18 个 setup `PermissionError`；这是临时目录权限问题而非测试断言失败。以上最终命令显式使用仓库内 `.tmp`，已全部通过。

## 固定传送带 104 任务前后对比

修改后以相同任务和预算全新运行，输出到 `.tmp/review_hardened_post`：

```powershell
.venv\Scripts\python.exe tools\run_m710id70_v3.py `
  --phase grid-fixed --workers 12 `
  --output-dir .tmp\review_hardened_post
```

| 指标 | 起始提交基线 | 本轮后测 | 变化 |
| --- | ---: | ---: | ---: |
| 原始任务分母 | 104 | 104 | 0 |
| grasp reachable | 52 | 52 | 0 |
| extraction feasible | 26 | 26 | 0 |
| 完整几何成功 | 10 | 10 | 0 |
| 新增成功 | - | 0 | - |
| 丢失成功 | - | 0 | - |
| 共同成功 | - | 10 | - |

共同成功任务为 `grid_022, grid_023, grid_025, grid_029, grid_031, grid_040, grid_041, grid_043, grid_047, grid_049`。104 行逐任务比较中，`GRASP_REACHABLE`、`EXTRACTION_FEASIBLE`、`GEOMETRICALLY_REACHABLE`、最终 failure stage/reason、成功 face/roll 均无变化；这些列规范化后的共同 SHA-256 为 `b603b3de505fa000f66bd691a662b1d32615538461961066d4ce5768837614e8`。因此没有需要解释的历史成功丢失。

后测 10 个成功任务都带有通过的实际终点 coverage/collision `ContactState`；逐任务复核的最大附着瞬时世界位姿误差为 `2.220446049250313e-16`。

修改后最终失败分布为：

| 最终原因 | 数量 |
| --- | ---: |
| `NO_IK` | 35 |
| `GRASP_CONSTRAINT_FAILED` | 17 |
| `ROBOT_COLLISION` | 16 |
| `PAYLOAD_PROXIMITY_NOT_RELEASED` | 8 |
| `TOOL_SELF_COLLISION` | 8 |
| `PATH_SEARCH_EXHAUSTED` | 8 |
| `SINGULARITY` | 1 |
| `PAYLOAD_PROXIMITY_WORSENED` | 1 |

其中最终为 `NO_IK` 的 35 个任务和 `PATH_SEARCH_EXHAUSTED` 的 8 个任务属于有限数值搜索未确认项，不是不可能性证明。任务候选历史中有 20 个任务至少一次以 `SEARCH_BUDGET_EXHAUSTED` 结束 escape 搜索：`grid_022, grid_024, grid_026, grid_027, grid_028, grid_030, grid_031, grid_042, grid_044, grid_045, grid_046, grid_099, grid_115, grid_117, grid_119, grid_133, grid_135, grid_137, grid_151, grid_153`；这可与同任务的其他候选成功并存，不能当成任务最终结论。

以 104 个 task JSON 的首末写入时间估算，新鲜运行跨度由基线 1302.21 s 降至 1053.62 s（约 -19.1%）。这不是严格 wall-clock benchmark，只用于说明没有因跨 station 调度造成预算级别的膨胀。对后测目录原命令复跑耗时 7.12 s，104 个 task JSON 的最新写入时间未变化，证明新 schema 下缓存可命中。

## 尚未完成与边界

- 本轮没有重跑连续 129 箱场景，也没有开展 dynamic conveyor A/B、base Z/lift range 或节拍优化；它们继续按 P1 暂缓。
- 10/104 仅表示严格单箱完整几何链路，不表示连续卸货、负载资格、整机动力学或厂家安全认证通过。
- 43 个搜索耗尽任务仍需在保持全局预算与数值标准的前提下，通过更好的候选排序、解析种子或等价的确定性搜索提高确认率；不能把预算整体放大后直接宣称恢复。
- 当前后端未实际加载 SRDF/mesh。若后续启用，必须先把其实际解析内容和生效的碰撞排除规则加入同一资产清单，再允许缓存复用。
