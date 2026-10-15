# Grounding DINO → 现有 SAM（2026-10-15）

本轮只替换提示框来源，保持 SAM、五组扰动、输出选择、后处理、实例去重和同图编码复用；几何数学、接触、融合、IK、碰撞、动力学和执行门禁未改。

**结论：接通完成，视觉质量未达标。** 四张原始图像均执行真实 Grounding DINO + 真实 SAM，无 GT 推理验证通过。frame602 上模组出现严重整片并箱，不能把运行成功当作分割通过。

本机交付入口：[`outputs/dino-sam-20261015/index.html`](../../outputs/dino-sam-20261015/index.html)。完整包为同目录 `grounding-dino-sam-review.zip`，包含首轮、同参数复跑、补充帧、历史 ORACLE 对照、原始 RGB、全部框/mask/逐实例图片和评价。大产物不提交 Git；精简的绑定、候选、评价和对照证据提交在 [`evidence/grounding-dino-sam-20261015`](evidence/grounding-dino-sam-20261015/delivery-summary.json)，其中四张图的完整 detector/evaluation JSON 以无损 `.json.gz` 保存。服务器完整产物位于 `/root/autodl-tmp/v05-acceptance/dino-sam-20261015`。

## 代码与调用关系

起始本地分支与 fetch 后的 GitHub 远端均为 `feat/v0.5-perception-ros2@48c2f8de8c9837e40eaa2825b60e18666fc02bbe`，工作区干净。服务器已有 checkout 是另一提交，本轮没有覆盖它；测试代码放在独立目录 `/root/autodl-tmp/v05-acceptance/dino-sam-20261015/code`。

默认链：`run_workcell_perception_once.main` → `perception_sam_pipeline.prepare_proposals` → `ResidentRuntime.detect` → `GroundingDinoRuntime.detect(RGB)` → 过滤/近重复框抑制 → `detector_proposals.json` → `infer_sam` → 原 `ResidentRuntime.infer/_sam` → 原 pinned upstream SAM。

视频的 `workcell_video_worker.RuntimeCache` 复用同一 runtime、检测模型及 SAM。`run_finite_workcell_sequence` 和 `run_perception_stage_capture` 也默认走新入口；显式 `--proposal-mode oracle` 才走旧链。`run_metric_small_matrix` 仍是明确的 ORACLE 实验工具，不是默认生产入口。

原接线追踪：ORACLE 文档来自 `run_metric_small_matrix.oracle_proposal_document`；`infer` 使用本地文件 URI + SHA-256 的 `proposal_reference` 交给 `vision_resident_worker.infer`。原几何进程和 viewer 硬编码文件名的消费者改为 `proposal_path`，新文件由 `proposal_binding.json` 显式绑定。`lineage.load_instance_lineage` 原本允许无 `simulation_object_id`，因此没有改动它；旧 viewer 使用该字段查 GT 的位置改为可选，新的蓝线使用冻结后的独立匹配。`isaac_payload` 的完整归档验证保留，新 `rgb_input` 只读传感器必要文件；`prompt_audit` 和 `sam_image_session` 的算法与提示构造未修改。

新增文件：`proposals.py`、`rgb_input.py`、`grounding_dino_runtime.py`、`perception_sam_pipeline.py`、`run_detector_sam_review.py`、`evaluate_detector_sam.py`、`build_sam_review.py`、`sam_review.html`、固定 detector 配置和 CPU 接口测试。原 worker、单次/视频/有限批次入口、模型准备、几何产物读取和旧评审页只适配来源、引用和绑定。

`cargo_instances.json.bbox` 与 NPZ `boxes` 仍然是原提示框；`mask_bbox` 单独计算。`proposal_id`、SAM `instance_id`、NPZ `mask_id` 按真实输出关联，支持拒绝和合并，检测 proposal 不包含 `simulation_object_id`。全部过滤前候选及拒绝理由保存在 detector JSON；原始 logits、归一化 cxcywh、token IDs 单独保存 NPZ。

## 模型和固定配置

- 检测：`IDEA-Research/grounding-dino-base@12bdfa3120f3e7ec7b434d90674b3396eccf88eb`。
- 权重 SHA-256：`5548f844c928c4b6f411fa8cbcc2bfa8dbbba437cb1d513975519f93c2a9ed21`。来源为 Hugging Face 官方模型仓库，通过显式指定的 `hf-mirror.com` 传输；常规下载停滞后分段恢复，并验证全部 933400872 字节的整文件 hash。
- SAM 不变：`facebook/sam-vit-base@70c1a07f894ebb5b307fd9eaaee97b9dfc16068f`；upstream `1d208f2ed380a207e6e46b4a62d2ac640edfe477`。
- 实际环境：Torch `2.7.1+cu128`、Transformers `4.53.2`、tokenizers `0.21.2`、huggingface-hub `0.33.4`、safetensors `0.5.3`；RTX 4090。没有升级原 SAM 环境。
- 文本 `cardboard box.`，box threshold `0.25`，text threshold `0.25`，近重复 IoU `0.9`，shortest/longest edge `800/1333`，候选上限 `300`，seed `17`。未使用已知对象数量。
- SAM 外部工作分辨率保持原始 2592×1944，divisor=1。

官方版本源码确认使用 `threshold` 和 `text_labels`，`target_sizes=[(height,width)]`；禁止按更新版文档盲用另一套参数。[Transformers v4.53.2 官方实现](https://github.com/huggingface/transformers/blob/v4.53.2/src/transformers/models/grounding_dino/processing_grounding_dino.py)、[官方模型](https://huggingface.co/IDEA-Research/grounding-dino-base)。

`raw_image_automatic=true` 只表示无需人工/GT 框；仍使用预配置类别文本，RGB 仍来自仿真采集。页面分别显示 detector score、SAM predicted IoU、SAM stability，不构造总置信度。

## 直接运行

使用现有锁定 GPU 环境，不执行 `pip install -U`。下面的输出路径必须尚不存在。

```sh
PY=/root/v05-gpu-venv/bin/python
VISION=/root/vision-fixed
CACHE=/root/autodl-tmp/huggingface
OLD_MODELS=/root/autodl-tmp/v05-acceptance/model-manifest.json
MODELS=/path/to/new-models.json
CAPTURE=/path/to/original/capture

# 显式准备，复制既有 SAM manifest，仅下载并增加固定 Grounding DINO。
$PY tools/prepare_vision_models.py --cache-dir "$CACHE" --output "$MODELS" \
  --add-grounding-dino "$OLD_MODELS" --endpoint https://hf-mirror.com

# 新默认链，仅检测与 SAM；不运行几何、接触、ROS、规划。
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 $PY tools/run_workcell_perception_once.py \
  --capture "$CAPTURE" --vision "$VISION" --models "$MODELS" \
  --stop-after sam --output-directory /path/to/new-detector-run

# 独立 ORACLE 对照；不成为失败恢复路径。
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 $PY tools/run_workcell_perception_once.py \
  --capture "$CAPTURE" --vision "$VISION" --models "$MODELS" \
  --proposal-mode oracle --stop-after sam --output-directory /path/to/new-oracle-control

# 固定多图、无 GT 访问拦截，全部预测完成后才显式评价。
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 $PY tools/run_detector_sam_review.py \
  --plan /path/to/primary-plan.json --models "$MODELS" --vision "$VISION" \
  --output /path/to/new-review --evaluate-gt
```

配置参数统一放在 `configs/perception/grounding_dino.json`，可用 `--detector-config` 指向单独的新实验配置。无 GT 路径只需要模块目录里的 `sensor_rgb.png`、`capture_binding.json`、`capture_metadata.json`、`camera_info.json`，并保留图像 hash、尺寸、捕获时间、模组、帧和标定检查。

本次服务器可直接复用的值：`MODELS=/root/autodl-tmp/v05-acceptance/dino-sam-20261015/models.json`；`--plan /root/autodl-tmp/v05-acceptance/dino-sam-20261015/primary-plan.json`。原始 capture 为 `/root/autodl-tmp/v05-acceptance/video-demo-20260920/recording-01/frames/000000` 与 `000095`。运行命令应在本次代码 checkout 根目录执行。`run_perception_stage_capture` 是完整阶段归档工具，不提供 `--stop-after`；轻量批量评审使用上面的 `run_detector_sam_review`。

**边界：检测/SAM 已可独立于 GT 运行；整个历史 RGB-D 归档加载器并未去除 GT。** 完整几何入口仍沿用原归档的 GT 完整性校验和历史评价流程，本轮没有关闭其校验。`--stop-after sam` 完全绕过该加载器，后续阶段写 `NOT_RUN`，执行保持阻断；空检测不意味着场景无障碍。

## 实验与评价规则

主样本固定为 frame32、frame602 的上下模组四张原始 PNG，路径分别为已有录制的 `frames/000000`、`frames/000095`。使用既有验证清单里的 RGB 哈希，未使用截图或替代图。全部输入、模型和配置在模型初始化前落盘。

补充样本规则在首次模型运行前固定：选择录制中 frame32 之后、排除两个主样本帧的第一帧，即 frame38 的两个模组。没有将临近录制帧当作独立场景泛化证据。

检测匹配：按 detector score 降序，在未占用 GT 中选择最大 bbox IoU，阈值 ≥0.5，稳定 ID 处理并列；每个 GT 最多匹配一次。mask 评价沿这份预测冻结后的关系进行，合并的 SAM mask 不能重复占用多个 GT 的平均分。同时列出未匹配预测、未匹配 GT、无 mask、合并、拆分/重复以及全部 GT 分母的 mask IoU。

边界误差以原始像素计算，参考为可见渲染 mask（包含遮挡/裁剪边），不声称是实体物理箱角。独立匹配只写评价产物，不回填 `simulation_object_id`。疑似并箱的检测诊断规则为：框内覆盖至少两个 GT 各 ≥50% 的可见像素；这是评价规则，绝不参与推理筛选。

## 四张图完整结果

首轮 `round1` 原样保留。`final-verification` 为同模型、同配置、同 SAM 的复跑，完善了计时/展示记录；逐一比较四份 detector 文档和 NPZ 中全部数组，**完全相同**。本轮文本/阈值调整次数为 **0**，没有从不同配置挑选结果。

| 原图 | 可见 GT | 检测框 | bbox 匹配 | Precision@.5 | Recall@.5 | 最终 SAM mask | 匹配集合 mask IoU 均值* | IoU 总和 / 全部 GT |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| frame32 上模组 | 16 | 16 | 15 | 0.9375 | 0.9375 | 15 | 0.9537 | 0.8941 |
| frame32 下模组 | 31 | 31 | 29 | 0.9355 | 0.9355 | 30 | 0.9596 | 0.8977 |
| frame602 上模组 | 12 | 13 | 12 | 0.9231 | 1.0000 | **1** | **0.0042** | **0.0042** |
| frame602 下模组 | 24 | 25 | 24 | 0.9600 | 1.0000 | 24 | 0.9593 | 0.9593 |

\* 分母是 bbox 匹配集合，合并 mask 已占用后的其他匹配计 0，不只取成功 mask。合计 85 个检测框、80 个 bbox 匹配、83 个可见 GT，检测 precision 0.9412、recall 0.9639；最终共 70 个 mask，不能按数量暗示一一对应或质量合格。

| 原图 | 边界可评价匹配 / bbox 匹配 | GT→预测平均边界距离（px） | 2 px 边界 precision | 2 px 边界 recall |
|---|---:|---:|---:|---:|
| frame32 上 | 15 / 15 | 4.884 | 0.496 | 0.497 |
| frame32 下 | 29 / 29 | 3.387 | 0.604 | 0.570 |
| frame602 上 | **1 / 12** | **43.296** | 0.110 | 0.442 |
| frame602 下 | 24 / 24 | 2.437 | 0.604 | 0.587 |

边界列只对有唯一 mask 的匹配计算，不对其余 11 个合并目标伪造边界分数；它们已在 mask IoU 汇总中计 0。逐实例距离、未匹配项和合并原因保存在每张图的 `detector_sam_evaluation.json`，并可从页面选择 proposal 追踪。

检测问题：四张图的 P1 都框住整片可见箱墙，是实际的多箱框/额外检测，不是补框。IoU≥0.9 的近重复抑制没有删框；没有候选上限截断。frame32 上漏匹配 `carton_l04_c03`；frame32 下漏匹配 `carton_l00_c03`、`carton_l01_c03`，未匹配预测 P1、P27；frame602 两图各多出 P1。没有用 GT 排除这些框。

SAM 问题：frame602 上模组的 P1 产生箱墙 mask，原 SAM 嵌套 mask 去重把其余 12 个 proposal 合并到该结果。高 bbox recall 在这里不能代表实例分割有效。其余三图中，P1 被原去重合并进单箱结果。frame32 的细小遮挡区域仍漏分；在既有显著重叠诊断规则下，未观察到额外拆箱/重复 mask，但边界误差仍存在。SAM 预测 IoU 可能超过 1（箱墙结果约 1.003），页面展示原值，不将其转换成检测概率。

frame32 下模组重点：P24、P27 的原生三栏放大图均保留。P24 看起来是单个正面框，评价规则却因邻居 GT 极小条带而标记多箱，这是诊断规则的局限。`carton_l00_c03` 的可见 GT bbox `[1345,1525,1350,1602]` 只有约 5 px 宽；`carton_l01_c03` bbox `[1346,1323,1542,1602]` 包含向上延伸的窄条，使 P27 的正面框无法达到 bbox IoU .5。已保存原 RGB/蓝色单 GT 对照。这是**待核查的遮挡/标注疑点**，不是断言 GT 错误；GT 没有被 SAM 修改，指标未重算挑优。

系统结果：四张输入 hash、尺寸、捕获/模组身份和产物 SHA-256 均通过；没有技术失败、空检测或候选截断。真实推理在仅有四个必要文件的隔离目录运行，同时拦截 `builtins.open` / `io.open` 对 GT/ORACLE 文件名的访问；四张均通过，推理结束后才加载 GT。未运行真实 OOM、超时或空场景试验；空检测、模型异常及禁止回退由 CPU 接口测试覆盖。

## 独立对照与补充样本

ORACLE + 同 SAM 的历史对照来自 `/root/autodl-tmp/v05-acceptance/stage-repair-20261002/final-run`，已核对原图、模型/upstream、proposal 和 SAM 产物绑定。它们是 **HISTORICAL_CONTROL_NOT_NEW_INFERENCE**，本轮只重新进行独立评价和绘图，历史文件未改写。

| 原图 | 历史 ORACLE SAM masks | 历史 mask IoU / 全部 GT | 新 DINO SAM masks | 新 IoU / 全部 GT |
|---|---:|---:|---:|---:|
| frame32 上 | 16 | 0.9079 | 15 | 0.8941 |
| frame32 下 | 31 | 0.9370 | 30 | 0.8977 |
| frame602 上 | 12 | 0.9605 | 1 | 0.0042 |
| frame602 下 | 24 | 0.9605 | 24 | 0.9593 |

ORACLE 自带 GT 框，其检测 precision/recall=1 是定义结果，不能拿来论证检测器精度。对照完整 masks、提示记录、框和叠图位于 `historical-oracle-control`。

预先选定的 frame38 上/下模组分别得到 16/31 个检测框、15/30 个最终 mask，bbox 匹配分别为 15/16 和 29/31。两张完整结果位于 `supplemental`，依旧有箱墙额外框和细小目标问题。它是相邻录制帧的补充检查，不能据此宣称泛化。

## 实测耗时（秒）

下表使用同参数最终复跑。检测器 GPU 阶段均显式同步；SAM 沿用原图编码会话的 GPU 同步计时。SAM 阶段包含原来的五组扰动、选择、后处理、去重和写出，并非只报模型 forward。

| 原图 | detector 预处理 | detector 推理 | detector 后处理 | SAM 阶段 | RGB→mask 可用墙钟 | 无 GT 评审读写/绘图 | 预测冻结后的评价+重绘 |
|---|---:|---:|---:|---:|---:|---:|---:|
| frame32 上 | .083 | .483 | .040 | 7.714 | 10.577 | 3.683 | 8.382 |
| frame32 下 | .063 | .100 | .045 | 15.558 | 16.151 | 5.689 | 20.667 |
| frame602 上 | .066 | .101 | .038 | 5.140 | 5.602 | 1.658 | 3.545 |
| frame602 下 | .067 | .100 | .044 | 12.497 | 13.007 | 3.904 | 13.664 |

冷启动另记：SAM runtime 初始化 2.810 s（其中模型加载 .290 s），DINO 模型初始化 1.241 s。第一张 RGB→mask 墙钟包含延迟加载 DINO、其权重 hash 校验、读写及请求处理；不包含此前 SAM runtime 初始化，冷启动首图约 13.387 s。后续换帧的 runtime 复用获取 .00277 s，**不是冷启动也不是新结果推理时间**。四图共享一次 resident runtime 和两个模型对象，每图 SAM 图像编码复用未变。全批（输入准备、冷启动、推理、两次绘图及评价）110.974 s。

`proposal_stage_including_io`、`request_total` 以及 SAM 图像预处理/编码/提示 forward 的细分均保留在 JSON。这里没有把写盘与计算全部拆成独立 kernel，也没有用预览帧率或缓存读取速度宣称推理 FPS。首轮耗时也完整保留，没有以最快一次替换。

## 测试记录

修改前 `.venv310/Scripts/python.exe -m pytest -q`：1039 passed、3 failed、6 skipped、1 deselected。已有失败：`test_depth_components_single_pass.py` 的两项计时断言；`test_effective_scene.py` 的 Windows 默认 GBK 解码。未删除或放松这些断言。

CPU 测试仅用明确的接口替身，不下载模型、不初始化 GPU、不联网，不作为模型精度证据。历史 ORACLE 测试改为显式 `--proposal-mode oracle`，保留原失败/未知/阻断断言；新增测试覆盖默认链进入原几何边界前的真实接线。

最终 `.venv310/Scripts/python.exe -m pytest -q`：**1059 passed、3 failed、6 skipped、1 deselected**，151.25 s。3 项失败与基线相同，没有新增失败。新增检测/SAM 接口测试单独运行 **20 passed**。测试日志随完整包交付。

覆盖内容包括非正方形 H/W 顺序、连续坐标裁剪/NaN/Inf/退化框/空结果、相邻框保留、候选截断标记、无仿真 ID、proposal 与 mask ID 不同、拒绝/合并追踪、错帧/hash/尺寸/config 拒绝、实际默认入口无 GT 访问、失败无 ORACLE 回退、常驻模型复用、无 GT 评审和显式历史 ORACLE 模式。完整默认入口到几何边界的接线通过 CPU 测试；真实 GPU 验证止于 SAM。

两份 viewer 内联 JS 的 `node --check` 通过；最后的 viewer/评审适配回归为 **23 passed、2 skipped**；检查了四图全分辨率产物与问题实例 PNG。可用浏览器清单为空，因此**浏览器交互验收 NOT_RUN**。本轮真实几何、接触、融合、Isaac 重采集、ROS、规划和执行均 **NOT_RUN**，没有伪造后续结果。原 SAM 不存储被去重丢弃候选的原始 mask 像素，故页面保留其真实记录、分数、提示及最终归属，但不伪造这些已丢弃像素。

复跑后最终审阅还修正了完整几何入口的显式评价路由及旧完整阶段工具的 CLI 参数声明；没有改变 detector/SAM 推理函数或配置，新增分支由上述 CPU 测试覆盖。`final-verification/frozen_plan.json` 保存运行时源文件 hash，提交版与实跑版的差异可据此核对。
