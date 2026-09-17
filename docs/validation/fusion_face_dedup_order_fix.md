# 面片去重与冲突检测的顺序不变性（2026-09-17）

仅第五步，分支 `feat/v0.5-perception-ros2`。开始时读取 AGENTS.md、fetch，
工作区干净，本地与远端均为 `94f26f9d4962bab50bd2529ddb1e6421f54af0f6`。
生产修改限于 `fusion.py` 和 `algorithm_handoff.py`，没有修改前四步实现、ROS 消息或第六步退出码。

## 修复前实际反例

新增 `tests/test_fusion_face_reduction.py` 后，先在未修改的生产 `_merge_faces()` 上运行：

```powershell
.venv310/Scripts/python.exe -m pytest -q -s tests/test_fusion_face_reduction.py
```

结果为 **2 failed**。真实 ObservedFace/ObservedFaceSet 的世界坐标 CPU fixture：
A、B 均为 0.4 m × 0.4 m，Z=2 m，X 起点分别为 -1 m、0 m；
B′ 与 B 同范围，支持点数 120，A/B 支持点数 100。
A/B 是一个已关联实例里的两个分离面片，不能解释为真实多箱检测。

| 输入 | 旧代码实际代表身份 | 旧冲突标志 | 新代码代表身份 | 新冲突标志 |
|---|---|---|---|---|
| `[A,B]+[B′]` | A, B, B′（3 个） | false | A, B′（2 个） | false |
| `[B,A]+[B′]` | B′, A（2 个） | false | A, B′（2 个） | false |
| `[A,B]+[B′]`，B′ 的 Z=2.03 m | A, B, B′（3 个） | false | A, B, B′（3 个） | true |
| `[B,A]+[B′]`，B′ 的 Z=2.03 m | B, A, B′（3 个） | true | A, B, B′（3 个） | true |

这两类旧缺陷均实际复现：首个无关共面候选触发 break，后面的重复/冲突候选未比较。
此外，旧实现支持点数相等时保留先到记录。

## 归约规则与适用范围

- 对象关联仍为原来的世界面片评分、歧义判定和互选配对。只对该算法产生的组归约；
  空间分离的共面对象不会放进全局面片池。关联中心的点求和改为固定坐标顺序，
  消除浮点累加顺序对诊断的影响，不改变评分公式。
- `_reduce_faces()` 是公开融合入口和兼容 `_merge_faces()` 包装器共用的实现。
  先按完整来源身份枚举所有原始面片对，独立保存重复关系和冲突证据，再选择代表。
- 数值基线不变：法向对齐 >=0.94、近似共面距离 <=0.06 m、重复平面距离 <=0.01 m、
  默认角点集合双向距离 <=0.04 m；冲突要求投影交集/较小面片面积 >=0.20 且平面距离 >0.01 m。
  关联阈值、评分、同步和来源检查均未放宽。
- 重复条件补上 **正面积重叠**。两个宽 0.02 m 的窄片，X 起点相差 0.02/0.03 m 时，
  虽角点距离仍在 0.04 m 内，但只是边界接触/空间分离，必须保留两面。
  完全重复和相差 0.02 m 且有面积重叠的正常正例仍归并。
- 部分重叠但角点范围差异超过原有容差、包含窄条但范围不同，均保留。
  不增加新的精度门槛，也不把共面、包含或重叠本身当成重复。
- 代表优先级为支持点数降序，其次
  `(module_id, capture_id, source_instance_id, face_id)` 字典序。
  face_id 单独不足以识别来源；不同模组可重复使用同一 face_id。
  完整来源键重复时明确报 `DUPLICATE_FACE_SOURCE_IDENTITY`，不任意取先到者。
- 按固定优先级只向仍保留的代表归并；若同时匹配多个代表，选优先级最高者。
  被删除成员不会继续充当桥梁。例如 X=0/0.03/0.06 m 的 A~B~C，
  A 支持最多时保留 A/C，B→A，不把不满足直接重复条件的 C 压到 A。
- 返回代表是实际输入对象，不平均、不重拟合、不拼接几何、支持数或嵌套诊断；调用不修改输入。
  代表列表、来源对应关系和冲突对均使用稳定顺序。

顺序不变性适用于**采集身份和观测记录不变**的同一集合：batches、face_sets、faces 的排列改变。
不保证重新采集或重新生成来源身份后得到相同 fusion_id，也不提供跨帧 track_id。

## 代表、原始证据与冲突交接

`FusedObservedObject.observed_faces` 是融合代表集合。现有 runner 导出融合结果，
`render_fused_observed_faces.py` 消费该集合显示面片；`run_isaac_rgbd_geometry.py` 也用它统计发布面数。
`diagnostics.face_reduction` 包括输入数、代表数、代表来源、重复成员→代表对应关系和冲突记录。

每个冲突包含两个完整来源键、原因、法向对齐、平面距离、角点距离、重叠比例，
以及投影到第一个来源面平面的交集多边形（世界坐标米）。它是诊断区域，不是新生成的代表面。
冲突依据原始记录比较，之后不会因代表被替换或成员被删除而消失。

特别测试 Z=2.000/2.009/2.018 m 三面：中间面的支持最高、分别与两端重复，
但两端相差 0.018 m 构成冲突。代表集合可只剩实际中间面，
原始两端仍由来源证据和冲突对保留，状态仍为 `CONFLICT_RETAINED_NO_AVERAGE`。
因此代表集合本身不承诺包含每个冲突端点。

正式 `fused_algorithm_observation()` 的 `cargo.observed_surfaces` 继续保留全部原始观测，
包括几何、支持、采集/标定绑定和边界诊断。它的数量有意不等于代表数。
增加 `raw_result.fusion_diagnostics` 传递上述对应关系和冲突证据，沿用现有 JSON/ROS 字段，
不新增消息协议。交接按稳定来源顺序输出面片，并固定输入 observation 排序，避免 observation_id
和未匹配记录顺序随调用顺序改变。

冲突状态原样穿透；未知体积保持未知，candidate_eligible=false，pose/full_dimensions_m/
corners_3d_m/axes_3d_rows 为 None，fusion_id 不充当 track_id。未知区域仍阻断规划。

## 验证结果

新增 19 项 CPU 回归：原始两类反例、不同/相同支持数、空间分离、边界接触、部分重叠、
正常重复、窄片反例、来源键冲突、非传递相似链、原始冲突端点被删除、输入不变性。
正常/冲突两组公开 fixture 各枚举 32 种 batches/face_sets/faces 排列，比较完整 asdict，
无字段剔除或测试入口预排序。关联组由真实公开算法产生，另有两个空间分离对象作为对照。

相同排列还经过真实 `fuse_module_face_batches()` → `fused_algorithm_observation()` →
正式 `dumps()/loads()`，逐项比较完整序列化结果、原始面片和诊断。
正常组为 2 个代表/4 个来源面，冲突组为 3 个代表/4 个来源面，两个冲突对均保留。

| 检查 | 实际结果 |
|---|---|
| Windows：融合、面片、handoff、显示相关测试 | 46 passed，6.94 s |
| Windows：默认轻量回归，默认 GBK 环境 | 505 passed，1 failed，1 deselected；既有 scene_snapshot.json 的 read_text 未指定 UTF-8 |
| Windows：PYTHONUTF8=1 默认轻量回归 | 506 passed，1 deselected，44.16 s |
| 授权服务器：默认轻量回归 | 506 passed，1 deselected，8.51 s |
| 授权服务器：全部 tests_metric | 31 passed，1.10 s；含第四步正常/冲突完整体对照 |
| 授权服务器：现有 Humble test_humble_imports.py | 4 passed，0.11 s |
| 授权服务器：两组公开 fixture → handoff → 真实 ROS observation_to_msg/from_msg | 完整指纹、来源面和冲突状态均一致 |

没有增加 skip 或放宽断言。默认命令的 1 deselected 是项目原有 marker 排除项。
Windows 解码问题仅通过进程 UTF-8 模式验证，未扩大本轮修改范围。
服务器使用基线提交的隔离归档目录并上传三个改动文件，未覆盖服务器原仓库。
本地/服务器三个源码文件 SHA-256 一致：

```text
e6f5129a7fe3dd1f97217aedd6ede8e95aec96f569c02c3aef474054bdb31a62  src/unloading_perception/fusion.py
114d43c385d57f8b0a87f3c8ca637a06663cdbd914d4111d07d7306b75b8cf56  src/unloading_perception/algorithm_handoff.py
19edee7c8046ef5c986e0209a5d6a9d2dd1d122908bc31f46cda66c6f814ad30  tests/test_fusion_face_reduction.py
```

主要复现命令：

```powershell
.venv310/Scripts/python.exe -m pytest -q tests/test_fusion_face_reduction.py tests/test_multimodule_fusion.py tests/test_observed_faces.py tests/test_algorithm_surface_contract.py tests/test_render_fused_observed_faces.py --basetemp .test-tmp/fusion-step5-targeted
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q --basetemp .test-tmp/fusion-step5-utf8
```

```bash
cd /root/autodl-tmp/v05-acceptance/fusion-order-20260917/repo
env -u PYTHONPATH /root/autodl-tmp/v05-acceptance/cpu/venv/bin/python -m pytest -q
PYTHONPATH=src:packages/unloading_contracts/src:tools:. \
  /root/autodl-tmp/v05-acceptance/boundary-evidence-20260917/venv/bin/python -m pytest -q tests_metric
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/v05-acceptance/joint-contract-20260917/install/setup.bash
export PYTHONPATH="$PWD/src:$PWD/packages/unloading_contracts/src:$PWD/tests:$PWD/ros2_ws/src/unloading_ros_bridge:$PYTHONPATH"
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_humble_imports.py
# 公开 ROS 往返使用 test_fusion_face_reduction.batch_fixture(False/True) 和
# module_observations()，经上述公开入口后验证映射前后 canonical_fingerprint 相等。
```

没有启动 Isaac、SAM/MoGe、训练、重采集、视频、运动规划或任务扫描。
未新增大型依赖；上述合成几何通过不代表真实满垛融合验收。
没有新建完整节点验收框架；当前提交的现有 Humble CI 查询结果在交付回复报告，
本地映射检查不替代完整 CI。
