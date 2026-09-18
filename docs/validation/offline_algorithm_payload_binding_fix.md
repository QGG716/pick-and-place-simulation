# 离线算法原始载荷绑定

基线 `5a0952da7b4165f0622f0697ce099eec4055884d`，仅分支 `feat/v0.5-perception-ros2`。
开始时读取 AGENTS.md、fetch，确认本地/远端一致、工作区干净。前三步时间、回放和执行授权代码未改。

## 修复前反例

先用原始 binding 完整的合成采集 fixture 调用生产 `_build_pointmap()`，
只把 float32 光轴深度一个有效像素从 1 改为 1.125，保留 dtype、shape、可解析性和原 binding。
旧代码正常生成点图，并将当前被修改文件的新哈希写为 depth_identity；预期拒绝断言实际为
`DID NOT RAISE CapturePayloadError`，**1 failed，0.73 s**。
OpenCV 显示导入使用未被调用的测试替身；载荷读取、点图入口及过滤代码均为生产代码，未涉及模型或 GPU。
修复后，同一反例在点图输入边界以 `HASH_MISMATCH file=metric_depth_m.npy` 拒绝。

## 本轮消费表

| 原始文件/数据 | 原始绑定字段/身份 | 共同校验位置 | 本轮消费者 |
|---|---|---|---|
| sensor_rgb.png | rgb_sha256 | load_capture_payload，同一字节缓冲区哈希与 PNG 解码 | SAM 独立输入文件；注册 RGB-D、算法和评价显示数组 |
| metric_depth_m.npy | metric_depth_sha256 | 同一 NPY 缓冲区哈希与 allow_pickle=False 解析 | 点图、metric_depth_runner、最终几何检查 |
| camera_info.json | camera_calibration_identity，规范化标定内容哈希 | 同一 JSON 缓冲区；再比对 manifest 相机 K、分辨率、注册、T_W_C | 注册、拟合、观测与评价 |
| capture_metadata.json | capture_metadata_sha256；module/capture/epoch/frame/time/clock | 同一缓冲区；原绑定、相机与 manifest 一致性 | 请求身份、点图、观测、融合、评价 |
| gt_annotations.json | gt_snapshot_sha256；manifest/module/时间身份 | 同一缓冲区哈希和结构 | 明示 oracle proposals 与 GT 评价 |
| gt_instance_masks.npz | instance_masks_sha256 | 仅 with_instance_masks=True 时同缓冲区校验/解析；对象关联、数组形状/二值检查 | mask 与观测评价；不进入本轮实际分割或几何恢复 |
| capture_binding.json | 原采集完成记录 | 读取原文；缺少必需 expected hash 明确拒绝 | 上述 expected 身份及 provenance；绝不读取后重算背书 |

PNG 是唯一 RGB 权威；原始 sensor_rgb.npy 不再被这些算法路径读取。
该文件缺失、损坏或与 PNG 不同均不影响它们，原文件不删除、不改写，也不列入已验证输入。
现有外部接口使用 PNG/点图，无需生成派生 RGB NPY。RGB→BGR 只在 OpenCV 绘图接口发生。
点云仍只有 with_pointcloud=True 才要求文件和绑定；正式算法路径不启用它。
ROS 默认路径不消费 GT masks，也不会因新增评价选项强制要求 masks。

## 校验、隔离与实际消费

共同边界仍为 `load_capture_payload()`。扩展的 VerifiedCapturePayload 保存已验证原始字节、只读数组、
只读相机/注释映射、元数据和可选评价 masks。`require_capture_payload()` 检查类型、目录、manifest 和预期模组匹配；
不接受裸字典加 verified=true。所有原始消费文件由同一次读取返回的缓冲区完成哈希检查和解析。

`run_workcell_perception_once.py` 先检查全部模组并准备输入，之后才读取模型配置、导入/初始化重型运行时。
坏模组记录 payload_validation 技术失败、sam_attempts=metric_attempts=0；其他合法模组仍按原策略各调用一次。
全部无效时没有模型初始化。保留不可覆盖运行目录、原子 UTF-8 summary、中断非零和部分失败非零规则。
合法空分割、零完整体仍是技术完成；proposal_source 始终明示 ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL，raw_image_automatic=false。

`write_capture_snapshot()` 在新目录以排他写入保存已验证字节，包括原 binding；PNG 不重编码。
生产消费端不调用 finalize_capture_binding，也不使用 binding_payload 参数构造 expected hash。
`input_provenance.json` 和单次 summary 区分原来源、原 binding、隔离目录、实际文件哈希和引用。
几何的 `geometry_input_provenance.json` 另记录送入子进程的命令、派生点图哈希及其原始 RGB/深度父身份。
这不构成对恶意并发写入者的安全保证。

明确覆盖：

- 单次 main → 真实 oracle_proposals/infer 请求准备 → worker 产物检查 → mask 评价 → 几何链。
- 独立 `run_isaac_rgbd_geometry.py` main，以及 `_build_pointmap()`、`_run_secondary_module()`、`_observation()`。
  未传 payload 时，几何入口自己加载验证；传入时要求来源匹配。独立 main 默认写入
  `<capture-directory>/rgbd-geometry-run`，也可用 `--output-directory` 指定**不存在**的独立目录。
  单独调用 secondary helper 时默认使用该模组下的新 `geometry-run`，返回 module_directory。
- 几何链将同一深度、RGB、K、元数据、注释和评价 masks 传到后续消费者。
  主模组复用同一几何编排，消除原先再次直接读取深度/标定/评价数据的重复路径。

已缓存 SAM 仍检查输入/提案/产物哈希、模型 revision、上游提交和 instance lineage；
原始缓存的 lineage 路径仍按其原身份检查，不通过重写缓存路径来通过校验。
独立几何会将缓存 oracle proposals 与已绑定注释的确定性派生结果比较。
单次请求 ID、产物归属和次数检查不放宽。点图的 depth_identity 取原 binding 的 expected hash，
其数组正是同一缓冲区验证后的深度，而非后续又读取的文件。

## 验证与未覆盖边界

新增测试使用小型合成采集、合成 SAM 产物和明确外部模型替身。
正式载荷加载、输入准备、提案生成、元数据传递、汇总和 CLI/SystemExit 都使用生产代码。
真实几何衔接测试只替换外部可执行程序与不会在六像素 fixture 上被调用的 extractor，
实际点图、metric 策略、最终几何检查、lineage、观测和评价代码继续运行。
可信加载后替换原目录所有原始消费文件，仍使用验证时快照；输出深度内容指纹与原数组一致。
既有 analytic metric 回归保持正常完整体和正确拒绝行为；没有改阈值、种子、拟合或融合策略。

保留 float32、米、optical_z_m 与原注册/时间约束；不新增 JointState 资格要求。
NaN/Inf、无效深度及全无效深度载荷保留原值；几何支持不足仍由已有算法规则判断。
GT mask key=None 的不可见对象和合法空评价集合不被新增文件规则拒绝。

未迁移为上述正式入口的研究/诊断流程包括 `run_metric_small_matrix.py` 的 oracle-mask/legacy 比较 main、
`diagnose_*`、`metric_v4_runner.py`、`regress_metric_frozen_stack.py`、`reevaluate_frozen_lineage.py`、
`run_carton_appearance_ab.py` 和 `validate_rgbd_without_moge.py`。
本轮只迁移 small_matrix 中被正式入口使用的 proposal/infer 函数；旧研究调用需显式传递载荷、
遵守新输出目录约定，历史目录缺绑定时不能当作已验证来源继续运行。
`_worker_artifacts()` 单独作为旧诊断工具的 worker 校验不等于原始载荷校验；正式链始终传入载荷。
通用 metric_depth_runner 的其他研究调用亦不在本轮完整来源保证内。

## 命令和实际结果

服务器复用既有 Ubuntu 22.04 / Humble / Python 3.10 CPU 环境，无依赖升级。
在 `/root/autodl-tmp/v05-acceptance/offline-payload-20260918/repo` 使用已有父提交归档加本地已提交差异还原基线，
再上传本轮变更；服务器原工作区与历史采集未修改。

```bash
python -m pytest -q tests/test_offline_payload_binding.py tests/test_isaac_payload_binding.py tests/test_workcell_perception_once.py
python -m pytest -q
# 既有可选 metric CPU 环境
PYTHONPATH=src:packages/unloading_contracts/src:tools:. python -m pytest -q tests_metric
# Humble 环境
ARTIFACT_ROOT=/tmp/offline-payload RUN_ID=payload bash tools/run_humble_acceptance.sh

# 正式单次入口；有效 capture 需原始绑定，结果写入新 perception-once 目录
python tools/run_workcell_perception_once.py --capture CAPTURE --vision VISION --models MODELS_JSON
# 独立几何入口；使用已有可核验 SAM 产物及新的输出目录
python tools/run_isaac_rgbd_geometry.py --capture-directory CAPTURE --bundle-directory BUNDLE \
  --vision-root VISION --upstream-python UPSTREAM_PYTHON --output-directory NEW_RUN
```

| 检查 | 结果 |
|---|---|
| 原始未校验深度反例 | 1 failed，0.73 s；修复后 HASH_MISMATCH |
| 定向输入/退出语义 | 74 passed，43.92 s；独立几何真实子进程另行最终复核通过 |
| 既有载荷、退出、几何相关 CPU | 132 passed，57.33 s；后续补充断言随全量复跑 |
| 前三步及 Isaac 真实 ROS | 43 passed，27.23 s |
| 默认轻量 pytest | 619 passed，1 deselected，15.47 s |
| 独立 metric CPU | 32 passed，1.08 s |
| 完整 Humble build/test | 3 packages，64 tests，0 errors/failures/skipped，退出 0 |
| Windows 默认编码 | 617 passed，1 failed，1 deselected，93.69 s；仅既有 GBK 问题，后补一项跨 capture 测试 |
| Windows PYTHONUTF8=1 | 619 passed，1 deselected，70.71 s |

Windows 失败仍是 `tests/test_effective_scene.py:29` 未显式指定编码的历史 JSON 读取；不删除测试或扩大编码整改。
服务器日志在隔离目录上一层 related-ros.log、default-final.log、metric-final.log、humble.log，Humble 完整产物在 humble/payload。
没有运行 SAM、MoGe、Isaac、渲染、任务扫描或真实硬件。未进行实际模型推理验收。
本轮证明输入与声明的原始采集记录一致，不证明算法准确率、传感器真实性、完整卸货能力，
也不抵御文件与 binding 同时恶意修改。当前提交 CI 按实际查询在交付中报告。
