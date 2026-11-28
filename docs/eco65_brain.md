# ECO65 智能助手（Qwen VLM 大脑 + 本地 ASR/TTS）

用户 2026-11-28：“给当前工程加个QWEN的VLM作为大脑，一是要看懂周围的环境，主要是判断是否可以开始抓放以及箱堆是否存在坍塌风险，二是可以和操作员实时沟通，要有ASR+VLM+TTS。”

用户对四个问题的决定：

| 问题 | 决定 |
|---|---|
| VLM 跑在哪 | 远程 GPU 服务器（与远程检测同一台，SSH 密钥登录） |
| 模型 | **Qwen3.5-9B**（用户 2026-11-28“用Qwen3.5-9B”；另两个候选 Qwen2.5-VL-7B、Qwen3-VL-8B 没有实测） |
| VLM 判断的权限 | **只做提示**：显示并播报，不进入任何执行条件、计划或作业 |
| ASR / TTS | 现场 PC 本地 CPU |
| 语音能做什么 | 问答 + 语音停止；**不能用语音启动作业或移动机械臂** |

## 组成

```
D435i 实时彩色帧 ──JPEG──► SSH ──► GPU 服务器 vlm_server.py（Qwen3.5-9B，bf16，贪心解码，关闭思考模式）
                                      │
麦克风 arecord ─► silero VAD ─► SenseVoice ASR ─┬─ 含停止词 ─► 软件停止（与“结束作业”按钮相同）
                                                └─ 其它 ─► VLM 对话（当前帧 + 系统状态）─► MeloTTS ─► aplay
```

- 代码 `src/unloading_sim/workbench/brain/`：`vlm_server.py` / `vlm_protocol.py`（服务器端，每次连接上传并核对 SHA-256）、`vlm_client.py`、`assessment.py`（提示词与严格解析）、`assistant.py`（调度与记录）、`voice.py`（本地语音）、`panel.py`（界面）、`settings.py`。
- 界面（用户 2026-11-28 调整）：连续作业页右栏“智能助手”面板：**XD-Bot** 按钮（立即评估）、自动评估（默认每 15 s）、评估结论（绿/黄/红）；“观测与检测”页右侧一列“XD-Bot 对话”：对话记录与输入框（字号比默认大两号）、🎤 语音对话（麦克风）、🔊 语音播报（用本机扬声器朗读回复，默认关）。
- 配置 `configs/brain/eco65_brain.json`；连接参数在被忽略的 `outputs/workbench/remote_vlm.local.json`（样例 `configs/brain/remote_vlm.example.json`），只用密钥，不存任何密码。`--brain-config` 可换配置文件。

## 现场评估

- 输入：本会话最新一帧实时彩色图（与实时检测同一帧源）+ 系统状态（设备、作业状态、执行条件阻塞项、是否持箱）。
- 输出必须是一个 JSON：`can_start` YES/NO/UNCERTAIN，`collapse_risk` LOW/MEDIUM/HIGH/UNCERTAIN，`people_in_area`，`hazards`、`reasons`、`summary`。解析不出或字段不合规记为 `INVALID_OUTPUT`，显示为黄色“评估无效”，**从不当作“安全”**；没有相机帧或连接失败记为 `ERROR`。
- 级别：不建议开始 / 坍塌风险高 / 有人 → 红色 ALERT；风险中、无法判断 → 黄色 CAUTION；其余绿色 OK。级别变化时语音播报（风险高时以“注意，箱堆有坍塌风险”开头）。
- 每条结论带 `authority=ADVISORY_ONLY_NO_GATE`。它不参与执行按钮阻塞项、连续作业的开轮条件、规划或任何检查；所有原有阈值、检查和门控不变。

## 对话

- 键入或语音提问 → VLM 看当前帧 + 系统状态 + 最近一次评估 + 最近 8 轮对话，回答简短口语（约 60 字内）。
- 语音播报是独立开关：打开后回复、评估级别变化和语音停止确认都经默认放音设备（PulseAudio → 本机扬声器）朗读；关闭时立即打断正在播的句子。只开麦克风不会出声。
- 提示词明确：助手只能提供信息，不能启动、移动、开关真空或改参数。

## 语音停止

- 识别结果含 `停止 / 停下 / 停车 / 急停 / 快停 / 别动 / stop` 之一即为语音停止：不经过 VLM，在识别线程里立即调用 `ContinuousJob.stop()`（作业进行时）和 `TaskManager.stop()`——与“结束作业”按钮相同的软件停止；随后界面线程刷新按钮。停止**不释放**吸住的箱子，**不替代**硬件急停；ASR 在说完后约 0.2–0.5 s 才出结果，也可能听错。
- 助手说话时麦克风仍在听，但只响应停止词（可以打断播报）；助手自己的话里停止词会被替换（“停止”→“终止”，“急停”→“紧急按钮”等），不会因为听到自己而停机。

## 记录

`outputs/brain/<YYYYMMDD>/brain-<pid>.jsonl`：连接、每次评估（提示词 SHA-256、上下文、原始回复、解析结果、耗时）、每轮对话、每次语音停止；评估与对话用到的 JPEG 存在 `images/<sha256>.jpg`。

## 部署

服务器（一次性，已于 2026-11-28 完成，见下方证据）：

```bash
MODEL=Qwen/Qwen3.5-9B DEST=/root/autodl-tmp/eco65-vlm/models/Qwen3.5-9B bash tools/brain/fetch_qwen_vl.sh   # 在服务器上运行
```

```bash
bash tools/brain/install_vlm_venv.sh   # 在服务器上运行：独立 venv /root/eco65-vlm-venv（transformers 5.20.2 + flash-linear-attention），不动检测用的 venv
```

现场 PC：

```bash
.venv-workbench/bin/pip install sherpa-onnx==2.0.3
```

```bash
bash tools/brain/fetch_voice_models.sh
```

离线检验（保存的现场冻结帧，不开界面、不连设备）：

```bash
.venv-workbench/bin/python tools/brain/assess_records.py 482503205ac74410a2a2d501e028b5c2 --ask "箱堆稳吗"
```

模型哈希清单：`configs/brain/qwen3.5-9b.sha256`（当前模型；`qwen2.5-vl-7b-instruct.sha256` 保留作对照）（ModelScope 文件 API 列出的 SHA-256，服务器加载前逐文件核对，按 设备/inode/大小/mtime/ctime 复用缓存）、`configs/brain/voice_models.sha256`（语音启动时核对）。注意 sherpa-onnx 的 `vits-melo-tts-zh_en` 发布包里 `model.int8.onnx` 只是 133 字节的 LFS 指针，加载的是 `model.onnx`。
