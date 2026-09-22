# 米制面片与可选旧恢复诊断

## 基线与依赖核查

基线 `dbc70a843618a47a6424b7311a039012a4e64498`，分支始终为
`feat/v0.5-perception-ros2`。根 AGENTS.md 已阅读，相关目录没有额外指令；
开始时工作区干净，fetch 后远端与本地一致。保留前四轮修复，不回退/清理/合并其他分支。

| 路径 | 真实依赖与本轮处理 |
|---|---|
| `_run_secondary_module` → `metric_depth_runner.run_metric_depth` | 原先强制启动旧恢复并解析 raw；最终米制策略没有读取 raw。只将这里的旧恢复改为显式可选诊断。 |
| `run_metric_depth` | 实际读取绑定点图的来源/采集身份/K、SAM instances/mask、原深度及 K；用 `extract_observation_labels`、`fit_metric_faces` 生成结果。`raw=None` 仅兼容旧调用，不读取它，不制造空 raw 文件。 |
| `load_extractor` | 每次仍校验上游 Git SHA，再导入固定 `recover_box_cuboids_3d.py` 的 `fit_planes`。服务器源码已读取核查：顶层只有导入、常量和函数定义，文件 IO 位于 `main()`；`fit_planes` 仅用传入点集和 pyransac3d，没有旧输出或磁盘缓存依赖。 |
| 随机状态 | `extract_observation_labels` 每次按既有配置 seed=17 设置 Python/NumPy 随机状态；旧恢复原本在独立子进程中。未改变种子、阈值、采样或数学逻辑。 |
| `_observation` / lineage | 谱系依赖 SAM mask/instances、oracle proposals 及最终 geometry records；面转换/校验只读最终面片、深度、捕获变换。未读取旧 raw/图片。 |
| fusion / `algorithm_artifact` / handoff | 消费最终观测和面集，不消费旧恢复文件。发布后仍调用原正式 loader，原哈希、来源、采集、模块和候选门控全部保留。 |
| `run_workcell_perception_once`、视频 worker | 一次处理入口继续传既有配置给 `_run_secondary_module`；视频 worker 间接复用此入口。新增开关透传、状态/计时汇报和正常的新 config identity。未运行视频 worker。 |
| `validate_rgbd_without_moge` / `run_carton_appearance_ab` | 最终面片消费者，不读取旧 raw；继承新默认。前者修正报告中的固定 baseline/V4 计时描述，汇报实际诊断状态和分项。未重跑这些 SAM/外观流程。 |
| `metric_v4_runner.run_metric_v4` | 确实使用 raw 构造 joint/fallback 输入；保持不变。 |
| `run_metric_small_matrix` | 真实的 legacy/metric 配对模式，显式强制开启诊断，并从本次返回的新目录读取旧结果，避免默认切换或误读历史目录破坏配对。算法不变。 |
| `diagnose_metric_calibration` 主程序、`complete_metric_stage_evidence`、历史证据合成/汇总 | 确实读取旧 raw/图片作阶段/历史比较；保持不变。新运行若需要这些诊断文件须显式启用。 |
| `regress_metric_frozen_stack` | 其历史配对报告仍读取 raw 和 legacy 目录；保持原行为。 |
| resident/one-shot vision worker 的 MoGe/cuboid/assembly 路径 | 旧恢复结果确实传入 assembly；保持不变。SAM-only 提前返回路径不受影响。 |

旧流程的绑定/谱系检查位于旧恢复之前，点图校验与上游版本检查仍位于最终面片函数内部，
未因禁用诊断被跳过。没有修改第三方上游、SAM/MoGe、拟合/融合算法、尺寸先验、执行门控或场景资产。

## 开关、状态及失败规则

配置为 `vision.legacy_cuboid_diagnostic`。当前米制入口及一次处理 CLI 支持
`--legacy-cuboid-diagnostic` / `--no-legacy-cuboid-diagnostic`，通过现有配置字典传递。

关闭时不执行旧子进程，不读取旧 raw/图片或回填历史输出。新 provenance 的
`external_geometry_command=null`，诊断状态 `DISABLED`。
开启时按原命令/参数执行，记录 `COMPLETED`、真实 returncode、秒数、输出哈希及日志。
非零退出、超时、缺失输出或非法 JSON 会保存 `FAILED` 和错误后抛出，不能以主结果存在掩盖失败。
一次处理主入口据此返回非零，不发布成功 artifact；A/B 驱动保存失败 summary 并非零退出。

最终观测模型描述明确为 pinned plane extractor + depth-constrained metric faces。
新观测和 artifact 使用有效配置的正常指纹；旧 artifact 的发布/加载规则没有放宽。
`legacy_comparison` 字段仍是历史比较工具的路径提示，不表示执行过该工具。

默认值先保留开启；只有定向测试及固定真实输入 A/B 精确比较通过后，才提交默认关闭。
本轮两项均通过后，已将 YAML 及 `_run_secondary_module` 缺省值改为 `false`。
显式 `--legacy-cuboid-diagnostic` 仍可恢复旧诊断；真正消费旧 raw 的 small-matrix 配对入口强制开启。
固定 A/B 使用显式 true/false，运行时入口的缺省仍为 true；两组生产代码哈希相同。
验证后仅切换该缺省分支，不改本轮已验证的算法。缺配置键的关闭路径另由生产入口 CPU 用例验证。

## 严格 A/B 比较

`compare_depth_preprocessing.py` 现在必须传 `--plan`，检查非空固定计划、模块顺序/数量/ID、
实例顺序/数量/ID、重复项、审计及点图必需数组；不再只比较 zip 的共同前缀。
正常拒绝单独计数，双方均拒绝不作为成功几何；缺文件/结构错误/数组不一致明确失败。
CLI 捕获异常写失败报告并返回 1，数值比较 false 也返回 1，验收不依赖可被 `-O` 删除的 assert。

新 `compare_metric_faces_ab.py` 同样核对完整清单，重新加载正式 artifact、校验配置及运行归属，
逐实例比较完整最终记录、观测，再比较完整融合结果、融合观测和索引。
没有浮点容差；不是只比较面数。允许差异的白名单限于：

- 当前运行目录的精确前缀替换；不忽略其他路径或字符串。
- 观测 `processed_time`，同时核查它等于 capture_time + 原计时区间。
- 观测/索引的 config identity，事先核查等于有效配置的规范指纹。
- 索引 run_id、诊断 bool；配置其他字段完全比较。
- 已通过正式 loader 哈希校验且全文另行比较的模块观测、融合和总观测引用哈希。

采集时间、绑定、mask 顺序、支持像素、残差、法向、平面位置、三维点、面身份、
接受/拒绝原因、unknown、融合对象和关联关系均不在忽略清单中。

## 固定真实输入与运行

复用上一轮 `capture-f24f63a`，原 2592×1944，上模组 IDs 1–16、下模组 IDs 1–31。
运行前通过原 capture/worker/lineage/artifact 验证器重新核对全部绑定和哈希，并冻结新计划。
输入是原来实际运行 SAM 处理 Isaac 渲染 RGB-D 的结果，仍为 ORACLE-PROMPTED、
`raw_image_automatic=false`。不重新推理、采集或启动 Isaac/ROS/RViz/执行桥。

使用同一修改版单次滤波代码、配置、seed 和 `/root/v05-gpu-venv/bin/python`，
分别在独立进程/目录运行 A（诊断开）和 B（诊断关），各一次，无换样本或重试到通过。
不是与上一轮有重复滤波缺陷的基线比较。

服务器新根目录：`/root/autodl-tmp/v05-acceptance/optional-legacy-20260922/`。
完整输入快照、点图/审计、旧诊断（仅 A）、最终面片、观测、融合及 artifact 保留在
`A-diagnostic-on/`、`B-diagnostic-off/`，历史输入和产物不变。

执行命令：

```bash
/root/v05-gpu-venv/bin/python code/tools/validate_metric_faces_ab.py prepare \
  --plan fixed_plan.json \
  --input-plan /root/autodl-tmp/v05-acceptance/depth-single-pass-20260922/fixed_plan.json \
  --vision /root/vision-fixed
/root/v05-gpu-venv/bin/python code/tools/validate_metric_faces_ab.py run \
  --plan fixed_plan.json --output A-diagnostic-on --diagnostic on
/root/v05-gpu-venv/bin/python code/tools/validate_metric_faces_ab.py run \
  --plan fixed_plan.json --output B-diagnostic-off --diagnostic off
/root/v05-gpu-venv/bin/python code/tools/compare_metric_faces_ab.py \
  --before A-diagnostic-on --after B-diagnostic-off \
  --plan fixed_plan.json --output comparison.json
```

## 计时语义

原 `elapsed_seconds` / `legacy_geometry` 保持“所选旧恢复诊断 + 最终米制面片”的区间，
不扩展成含预处理的总时长。关闭诊断后该区间的旧恢复部分为零。
新增分别记录旧诊断、最终面片、最终校验/观测转换、评价/观测写入、审计 JSON 写入。
模块总时长包含输入快照、点图和结果写出；A/B 驱动另计输入加载/绑定、融合发布、
artifact 加载、最终输入哈希复核、整体墙钟和未归类开销。

`pointmap_build_including_filter_audits` **包含逐箱全分辨率审计 NPZ 压缩写入**，
本次不将该内部写入再单独分摊；合并点图 NPZ 和最终组件审计 JSON 写入另计。
最终面片函数计时包含它原有的点图来源检查、提取器加载、拟合、结果 JSON 和叠图写出。
“旧几何区间”“模块总时长”“整体墙钟”均为包含时间，不能与其子项重复求和。

## 实际结果与收益

两次真实运行及严格比较均退出 0，无失败实例被删除，无重试。
计划完整、结果相同是两个独立字段，均为 true；全部 47 条最终记录精确一致，差异列表为空。
模块观测、融合关联、融合观测、原始采集和 unknown 均通过完整字段比较；没有使用数值容差。

| 模组 | 固定 mask ID | A/B 面片数 | 有面片 / 无面片实例 | 完整箱体 accepted |
|---|---|---:|---:|---:|
| upper | 1–16 | 23 / 23 | 16 / 0 | 0 / 0 |
| lower | 1–31 | 48 / 48 | 30 / 1 | 0 / 0 |

lower mask 1 的 670 个选中像素在最终标签选择中排除边界 320、深度间断 350，保留 0；
仍为 `UNRESOLVED_PHYSICAL_BOUNDARIES`，没有伪造面片或完整箱体。
这与点图滤波阶段的 PASS 不矛盾：两组点图审计均完整包含 16+31 条 PASS，最终几何还有独立检查。
融合均为 37 个对象、84 个 unknown；完整融合 JSON 字节哈希也相同。
capture_time 保持 1.6666666666666667、frame_sequence=100，仍为历史采集、ORACLE-PROMPTED，
`raw_image_automatic=false`、`planning_admissible=false`，未解除在线门控。

旧恢复子进程：A 每模块 1 次、共 2 次，真实返回码均为 0；B 为 0 次且无旧 raw/图片。
最终 `run_metric_depth` 两组均每模块 1 次、共 2 次，处理全部 47 个 mask。
两组均保留完整点图和逐箱审计；模块目录 NPZ 数为 18+33（含复制的 GT 评价 mask 文件），
总大小 A=44,718,585 bytes，B=44,718,590 bytes；路径元数据产生正常的新字节和哈希。

以下以秒计，分项为上下模组合计；包含项单列，不能重复加和：

| 计时范围 | A：诊断开 | B：诊断关 |
|---|---:|---:|
| 初始输入哈希/环境验证 | 0.0611 | 0.0549 |
| 输入加载、绑定及 worker 产物验证 | 4.0702 | 4.5623 |
| 模块输入校验与快照 | 0.0540 | 0.0594 |
| 点图构建（含逐箱审计 NPZ 压缩写入） | 43.0150 | 45.2717 |
| 合并点图 NPZ 写入 | 1.8317 | 1.8717 |
| 旧恢复诊断 | 44.3636 | 0 |
| 最终面片计算（含内部校验与结果写出） | 167.7510 | 187.4429 |
| 最终校验 / 观测转换 | 45.6048 | 48.7841 |
| 评价 / 观测写入 | 1.7064 | 1.8625 |
| 滤波审计 JSON 写入 | 0.0017 | 0.0018 |
| 模块内未归类开销 | 0.0691 | 0.0665 |
| 双模组融合 / artifact 发布 | 2.4949 | 2.6845 |
| artifact 正式加载校验 | 0.2977 | 0.2689 |
| 最终输入哈希复核 | 0.0393 | 0.1930 |
| 驱动未归类开销 | 0.0992 | 0.0705 |
| **整体墙钟（包含上述项目，summary 写入前）** | **311.4595** | **293.1949** |
| 原 elapsed/legacy_geometry 区间（包含诊断与最终面片） | 212.1362 | 187.4583 |
| 上模组整体（包含项） | 101.9932 | 95.0605 |
| 下模组整体（包含项） | 202.4040 | 190.3002 |

模块内未归类 = module_total 减去该模块的互不重叠分项；驱动未归类 = 整体墙钟减去模块总计、
加载/验证、融合发布与加载校验。包括小型 JSON 读取/输出和包装调用开销；未把嵌套区间相加。

实际整体下降 **18.2647 秒（5.86%）**；原几何区间下降 **24.6780 秒（11.63%）**。
省去的诊断为 44.3636 秒，但不能把它直接宣称为整体收益：B 的最终面片阶段增加了 19.6919 秒。
两组均在同一服务器、Python 3.10.12 / NumPy 1.26.4、同一解释器运行，各一次，没有挑最快样本。
系统 load average（1/5/15 分钟）A 起止为 (17.14,18.18,17.84) → (27.00,26.76,21.86)，
B 为 (20.73,25.14,21.55) → (38.40,39.26,29.04)。这是整机负载，含本任务多线程数值计算；
没有持续采样或隔离其他租户资源，不能将数值拟合耗时波动准确归因于某个进程。
仅据本次观测报告收益，不承诺稳定加速倍数。

最大剩余分项是最终面片计算 B=187.44 秒，其次观测转换 48.78 秒、点图与逐箱审计 45.27 秒。
本轮不继续优化拟合、数组或存储。这是复用 SAM 的几何处理，不是 RGB 到 ROS 的完整实时测试。

## 证据、测试与边界

轻量原始证据保存在 [evidence/optional-legacy-20260922](evidence/optional-legacy-20260922/)：
`fixed_plan.json` 固定输入哈希/配置/47 个 ID，`A-summary.json`、`B-summary.json` 保存完整分项，
`comparison.json` 保存全部逐实例结果，`evidence_manifest.json` 保存输出文件哈希、审计数量和无面片完整记录。
文件保留原始字节；完整大产物保留在服务器本轮独立目录。
计划 SHA256 为 `768695f6ad0a8ce5baaeaace4a76ea0a7babe0e4a3380c6ec6f1fd6d40cebda6`；
运行时代码归档 SHA256 为 `c2b4f07272c64c834dd1ec7cf21770348369ec5c497d0bbe8482852055ead994`。
两组代码文件哈希见 summary；上游仍固定 `1d208f2ed380a207e6e46b4a62d2ac640edfe477`。

定向 CPU 命令（`.venv310/Scripts/python.exe`）：

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q tests/test_preprocessing_comparison.py tests/test_metric_faces_ab_comparison.py tests/test_optional_legacy_diagnostic.py tests/test_depth_components_single_pass.py tests/test_upstream_v4_adapter.py --basetemp=tmp/pytest-legacy-all-targeted
.venv310/Scripts/python.exe -m pytest -q tests/test_optional_legacy_diagnostic.py tests/test_depth_components_single_pass.py --basetemp=tmp/pytest-final-default
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/pytest-optional-legacy-full
```

第一组 43 passed / 11.46 秒；默认切换后补充缺配置键及真实旧策略入口测试，第二组 27 passed / 5.25 秒。
全量默认 CPU 回归：**819 passed，1 deselected，139.07 秒**；前三轮执行/尺寸先验及上一轮深度修复测试保留并通过。
测试覆盖零/一次旧子进程、正常最终函数次数、无旧文件、raw 参数兼容不读取、固定 SHA 校验，
真实 one-shot 配置传递及 artifact 发布/加载、显式诊断失败退出，
实际 small-matrix 入口强制启用且消费当前目录的 raw，原 V4 适配器保留旧输入语义。
严格比较器覆盖缺模块、双空、缺实例、重复 ID、缺文件、数组/几何坐标差异与产物哈希不符；
预处理比较的 CLI 故障用例实际使用 `python -O`，均应非零退出，不依赖 assert。
注入的超时、非零返回和缺诊断文件如实记录 FAILED；这些是合成故障测试，真实 A/B 无失败。

未运行 SAM/MoGe 推理、Isaac 采集、视频 worker、Humble/ROS/RViz、运动规划或机器人执行，
未把真实 A/B 的输入等价性扩大为所有未来场景的算法精度或实时保证。
按本分支规则保持核心开发版 `0.5.0.dev0`、contracts `1.1.0`、ROS interfaces `1.2.0`、
bridge `1.2.1`；未修改的包、消息和执行契约不升版。
