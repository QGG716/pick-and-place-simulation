# 深度连通域与单次滤波修复

## 基线、范围和失败复现

实际基线 `15b78519bb4232eecc905a154d5bf0c0d27d1a09`；工作分支始终为
`feat/v0.5-perception-ros2`。开始时工作区干净，fetch 后 origin 与本地一致。
阅读根目录 AGENTS.md，相关目录没有额外 AGENTS.md。未 reset、clean、切分支或合并其他功能。

修改生产代码前，新增用例直接调用仓库 `_depth_continuous_components()`，实际执行：

```text
.venv310/Scripts/python.exe -m pytest -q tests/test_depth_components_single_pass.py --basetemp=tmp/pytest-depth-before
2 failed in 0.48s
```

| 10×10 全 true mask | 修复前分量数 | 修复前像素归属总数 | 修复后分量大小 | 修复后归属总数 |
|---|---:|---:|---|---:|
| A：深度全部为 1 m | 100 | 199 | `[100]` | 100 |
| B：左右各 50 点，深度 1/2 m，阈值 0.02 m | 100 | 198 | `[50,50]` | 100 |

这不是转述评审隔离结果；[counterexamples.json](evidence/depth-single-pass-20260922/counterexamples.json)
记录本轮生产函数的实际失败和修复后结果。

## 最小生产修改

`np.where(mask & ~seen)` 在外层循环开始时已生成全部起点，BFS 更新 `seen` 不会删除
该列表的后续起点。本轮只在开始 BFS 前增加 `if seen[start_y,start_x]: continue`。

保持原来的行优先起点顺序、上/下/左/右扩展顺序、稳定的按大小降序排序、四邻域、
mask 限制，以及**相邻两像素** `abs(depth[q]-depth[p]) <= threshold` 条件。
不按起点深度/分量均值判断，不引入二值 connectedComponents、新后端或新阈值。
`retain_multiple_depth_components`、`minimum_component_points`、`minimum_points` 的语义不变。

新增组合入口 `masked_metric_pointmap_with_filter()`，在同一次调用内返回
`(MetricPointMap, InstanceDepthFilterResult)`。原 `masked_metric_pointmap()` 签名和返回类型不变，
通过组合入口返回点图。`_build_pointmap()` 改为直接消费组合返回值，点坐标与审计数组来自同一次滤波。
没有外部旧结果参数、全局缓存、磁盘缓存或跨帧/mask_id 复用。
错误仍按既有规则产生实例 REJECTED；全部拒绝仍抛出原有 RuntimeError。

`_run_secondary_module()` 的旧 `elapsed_seconds` 仍只覆盖外部几何恢复与后续米制拟合区间，
**不改变历史字段含义**。新增 `timing_seconds` 返回值、summary 分项和独立
`rgbd_stage_timing.json`：

- `pointmap_build_including_filter_audits`：完整 `_build_pointmap`，含逐实例审计 NPZ 写入。
- `pointmap_write`：合并点图 NPZ 写入。
- `legacy_geometry`：等于旧 `elapsed_seconds`。
- `module_total_before_timing_record`：从模块入口到原有结果/审计写完，包括验证、点图、几何、评价；
  截止点为新增计时小文件写入之前。

用实际模块入口和可控时钟测试：点图 7 s、点图写入 3 s、旧几何区间 24 s、其他阶段 7 s，
旧字段仍为 24 s，新整体字段为 41 s；测试中的外部算法为明确的桩，不是性能证据。

## 固定真实输入与隔离方式

复用上轮使用的完整绑定采集组与原有 SAM 产物；数据是实际运行算法处理 Isaac 渲染 RGB-D 的结果，
不是硬件实拍，也不是本轮合成性能样本。没有重跑 SAM、MoGe、采集或启动 Isaac。

```text
capture: /root/autodl-tmp/v05-acceptance/roof-mast-20260916/capture-f24f63a
algorithm: /root/autodl-tmp/v05-acceptance/single-handoff-20260918/algorithm-run-01
epoch: carton-assets-capture-f24f63a
sequence: 100
capture_time: 1.6666666666666667, ros_sim_time
```

**运行前**冻结 [fixed_plan.json](evidence/depth-single-pass-20260922/fixed_plan.json)：
上模组 IDs 1–16、下模组 IDs 1–31，所有 box/cardboard_box mask，保持归档顺序；
两者原分辨率均为 `2592×1944`。记录 manifest、RGB、深度、binding、metadata、标定、
SAM mask/instances/2D faces、worker response/metrics、oracle proposals 的路径及 SHA-256。
先用正式 artifact loader、capture payload loader、worker artifact/lineage 校验再处理。
保留 ORACLE-PROMPTED / `raw_image_automatic=false` 的事实。

使用既有 `configs/isaac/perception_validation.yaml:vision.pointcloud_filter`：
erosion=2、percentiles=1/99、MAD scale=6、floor=0.02 m、local selection=true、
retain multiple=true、minimum component=8、minimum total=50。无调参、降采样或删实例。

基线通过 `git archive` 独立导出并设为只读；修改版在另一个代码目录。
Windows archive 导出使用 CRLF：已确认服务器基线文件与本地 archive 字节完全一致，
且仅规范化 CRLF→LF 后与指定 Git 提交逐字节相同。没有回退工作区或在一个进程混用两版包。

服务器新工作目录：`/root/autodl-tmp/v05-acceptance/depth-single-pass-20260922/`。
固定运行顺序：基线完整一遍（上→下），然后修改版完整一遍（上→下）；**各一次，无挑最快值或追加微基准**。
独立输出为 `baseline-run-01/`、`candidate-run-01/`。每版先复制已验证输入到新目录，
全部逐实例全分辨率 NPZ、组件审计 JSON 和合并点图均实际写出，未关闭压缩或审计。
所有原输入哈希在前后再次核查不变。

两版使用同一服务器 `/usr/bin/python3`，Python 3.10.12、NumPy 1.21.5。
这是既有服务器解释器，不等同于本地测试环境或依赖版本完整验收。
主机可见 208 CPU；基线前/后 1 分钟 load 为 18.65/17.88，修改版前/后为 17.50/14.74。
容器内观测的主要 CPU 消耗是测量进程，期间没有并行运行另一版或推理；
共享主机仍有外部负载，且运行次序可能影响页缓存，不据单次结果推导稳定加速倍数。

## 实测分项（秒）

测量脚本 `tools/benchmark_depth_preprocessing.py` 包装真实生产入口计数和计时，
不复制/替换 BFS 或滤波实现。分项定义在两版相同：

| 分项 | 上：基线 | 上：修复后 | 下：基线 | 下：修复后 |
|---|---:|---:|---:|---:|
| 深度连通域，全部实际调用累计 | 6.497 | 1.714 | 11.588 | 2.690 |
| 完整实例滤波，全部实际调用累计 | 49.520 | 2.850 | 80.204 | 4.692 |
| 其中：连通域后掩码/分量审计生成 | 42.332 | 0.797 | 67.398 | 1.421 |
| 点图提升/构造，扣除内部滤波 | 1.995 | 1.976 | 3.634 | 3.597 |
| 逐实例原始点审计数组生成 | 1.197 | 1.245 | 2.151 | 2.262 |
| 逐实例审计 NPZ 压缩写入 | 8.243 | 8.038 | 15.310 | 15.170 |
| **`_build_pointmap` 整体墙钟** | **61.836** | **14.882** | **102.795** | **27.015** |
| 合并点图 NPZ 写入（在上述入口外） | 7.326 | 0.786 | 10.742 | 0.998 |
| 完整组件审计 JSON 写入（入口外） | 7.724 | 0.000777 | 12.324 | 0.001179 |

分项存在包含关系，不能把所有行相加。完整滤波包含连通域、百分位处理、保留 mask 和
组件统计；“连通域后”还包含拒绝码生成，并非单独 JSON 序列化时间。
逐实例审计数组计时从该生产入口的 `np.indices` 开始，到 `np.savez_compressed` 调用前结束；
目录创建、字典追加等其余开销包含在整体墙钟中。测量 wrapper/计数开销没有从墙钟扣除，
两版使用同一脚本。输入哈希核查/加载快照在计时入口之外。

两个模组 `_build_pointmap` 合计 **164.631 → 41.898 s**（本次下降约 74.6%）。
点坐标提升及全分辨率审计 NPZ 写入基本没有变快；主要收益来自删除重复分量的统计工作和第二次滤波。
这不是完整视觉流水线实时性结论。逐箱数组分配/压缩仍是显著开销，本轮不再重构。

## 调用次数、输出与正确性比较

| 指标 | 上：基线→修复后 | 下：基线→修复后 |
|---|---|---|
| 进入处理的 mask / PASS 数 | 16 / 16 → 16 / 16 | 31 / 31 → 31 / 31 |
| 实际滤波调用次数 | 32 → 16 | 62 → 31 |
| 每实例滤波次数 | 2 → 1 | 2 → 1 |
| 输出组件审计记录数 | 1,039,709 → 21 | 1,676,635 → 38 |
| 输出组件像素归属总数 | 2,079,397 → 1,039,709 | 3,353,232 → 1,676,635 |
| 实际全部调用的像素归属累计 | 4,158,794 → 1,039,709 | 6,706,464 → 1,676,635 |
| 逐实例审计 NPZ 总字节 | 10,630,648 → 10,630,648 | 19,106,499 → 19,106,499 |
| 组件审计 JSON 字节 | 232,597,235 → 18,037 | 374,047,641 → 34,355 |
| 合并点图 NPZ 字节 | 14,039,855 → 5,145,918 | 23,506,505 → 9,511,377 |
| 上述输出总字节 | 257,267,738 → 15,794,603 | 416,660,645 → 28,652,231 |

合计输出分量 **2,716,344 → 59**，像素归属 **5,432,629 → 2,716,344**，
修复后归属数等于各实例连通域入口 mask 的像素数之和。
各实例审计记录仍为 47 条；只消除了内部错误组件条目。
全量实际调用产生的组件数另为 **5,432,688 → 59**，基线翻倍来自重复滤波，不能与单份输出审计混淆。

`tools/compare_depth_preprocessing.py` 对全部 47 个实例和两个合并点图逐数组比较，
[comparison.json](evidence/depth-single-pass-20260922/comparison.json) 的
`all_preserved_outputs_equal=true`：

- raw-valid mask、retained mask、逐像素 rejected reason、原始点/过滤后点坐标全部一致；
  包括 dtype、形状和 NaN 分布。47 个实例均 PASS，实例拒绝原因也一致。
- 合并点图的 XYZ、有效 mask、深度、K、归一化内参全部一致；
  去除本应变化的 component 列表和新输出路径后，完整 metadata 相同。
- 组件数量、像素计数、审计列表、包含审计的元数据/文件哈希允许并实际发生变化，
  不补回错误单点或旧计数；每个新文件的正常 SHA-256 都已记录。

完整逐次计时、调用数量、各 mask 状态和所有输出文件体积/哈希见
[baseline_measurements.json](evidence/depth-single-pass-20260922/baseline_measurements.json)、
[candidate_measurements.json](evidence/depth-single-pass-20260922/candidate_measurements.json)。
大体积原始 NPZ/JSON 保留于服务器两个新目录，本次只提交验证清单和结果，不重复拷贝数百 MB 审计。
[run_manifest.json](evidence/depth-single-pass-20260922/run_manifest.json) 核查基线/修改版代码身份、
输入未变、证据哈希和合计数据。

实际服务器运行命令（工作目录为上述新目录）：

```bash
python3 benchmark.py --code-root baseline-code --plan fixed_plan.json --prepare \
  --capture /root/autodl-tmp/v05-acceptance/roof-mast-20260916/capture-f24f63a \
  --algorithm-root /root/autodl-tmp/v05-acceptance/single-handoff-20260918/algorithm-run-01 \
  --baseline-sha 15b78519bb4232eecc905a154d5bf0c0d27d1a09
PYTHONDONTWRITEBYTECODE=1 python3 benchmark.py --code-root baseline-code --plan fixed_plan.json --output baseline-run-01 --label baseline
PYTHONDONTWRITEBYTECODE=1 python3 benchmark.py --code-root candidate-code --plan fixed_plan.json --output candidate-run-01 --label candidate
python3 compare.py --before baseline-run-01 --after candidate-run-01 --output comparison.json
```

服务器 `benchmark.py` / `compare.py` 分别是仓库两个测量/比较工具的原样上传。
计划、运行目录和比较结果均要求不存在才写入，复现需使用新的输出名称。

## 测试与未验证范围

本地 `PYTHONUTF8=1`，`.venv310/Scripts/python.exe`：

```text
python -m pytest -q tests/test_depth_components_single_pass.py tests/test_rgbd_pipeline.py tests/test_offline_payload_binding.py tests/test_instance_lineage.py tests/test_algorithm_artifact.py --basetemp=tmp/pytest-depth-targeted
# 70 passed in 10.58s
python -m pytest -q --basetemp=tmp/pytest-depth-full
# 792 passed, 1 deselected in 125.30s
python -m pytest -q tests/test_depth_components_single_pass.py --basetemp=tmp/pytest-depth-timing
# 15 passed in 1.08s
```

全量运行收集后增加了一个模块计时边界测试，随后单独运行该文件全部 15 项验证。
包括空/单/孤立像素、空间分离、图像边缘/孔洞、对角不连接、阈值相等/超阈、逐边深度链、
多分量/最大分量/同大小排序、最小分量/总点数、非法深度排除、接口兼容、拒绝传播与数组身份。
固定种子 71923 的 80 个小图，与独立并查集逐边参考比较：互斥、覆盖、连通性及跨分量无合法边。
实际 `_build_pointmap` 调用计数还覆盖失败实例、非箱体跳过、同 ID 再调用不缓存。

前三轮源时间/接管、取消/停止、尺寸先验适配实现与测试未修改，均包含在 CPU 全量回归中。
没有跑 Humble 集成、SAM/MoGe/米制拟合、Isaac、运动规划、完整卸货或机器人执行。
没有改 tracker、融合/面片阈值、planning_geometry 假设、状态机、场景或资产。

沿用分支开发版本 `0.5.0.dev0`，contracts `1.1.0`、ROS interfaces `1.2.0`、
execution bridge `1.2.1` 不变；本轮是局部 Python 修复和新增独立审计计时字段。
