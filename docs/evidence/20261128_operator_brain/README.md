# 2026-11-28 智能助手（Qwen3.5-9B + 本地 ASR/TTS）实测

环境：GPU 服务器 RTX 4080 32 GB（AutoDL，密钥登录；主机信息只在被忽略的本地文件里），VLM venv：torch 2.8.0、transformers 5.20.2、flash-linear-attention 0.4.0（`causal_conv1d` 未安装，用参考实现）；现场 PC CPU：sherpa-onnx 2.0.3。没有连接任何设备，没有运动、没有阀门命令，现场 PC 不出声（TTS 只合成不播放）。

## 保存的现场冻结帧（`qwen35_9b_saved_frames.json`，`tools/brain/assess_records.py`）

| 记录 | 结论 | 评估耗时 |
|---|---|---|
| 482503205ac7 | 可以开始 · 风险低 · 无人 | 10.4 s |
| a36db921bfd9 | 可以开始 · 风险低 · 无人；指出地面拖把 | 9.2 s |
| c6da145eb913 | 可以开始 · 风险低 · 无人 | 7.0 s |
| 01fc3f5fdfbe | 可以开始 · 风险低 · 无人；指出地面拖把 | 8.4 s |
| 38a5ba54ea58 | 无法判断（离线回放无实时状态） | 8.8 s |

- 5/5 回复都是合规 JSON（650 输入 token、150–180 输出 token）。
- **局限**：a36db921 帧右上纸箱明显外伸出下方大箱右缘，模型仍判“坍塌风险低”——对外伸、悬空这类细节不敏感。结论只作提示（ADVISORY_ONLY_NO_GATE），所有原有检查不变。
- 对话（“箱堆稳吗，现在能开始抓吗”）：首次 16.4 s（新的序列长度触发 Triton 自动调优），之后 1.9–2.7 s。

## 整链路（`records/20261128/brain-151731.jsonl`）

真实 OperatorBrain + 远程 Qwen3.5-9B + 本地 VoiceLoop（假麦克风、静音扬声器）。问题用 TTS 合成后经 VAD/ASR 识别送入助手：

- 连接（哈希校验 + 加载 + 640×480 预热）25.5 s；评估 10.6 s；ASR 0.11 s；首次回答 16.6 s，第一句语音 18.4 s 后可播。
- ASR 把“箱堆”识别成“相对”（SenseVoice 无热词），VLM 仍正确回答。
- “快停下” → ASR “快停下。” → stop_callback 在 0.3 ms 内被调用，没有 VLM 往返。

## 问题与修正（本轮已改）

- 首个请求编译 Triton 内核超过 30 s 请求超时 → 服务端加载后先做一次 640×480 预热（计入 600 s 连接超时）。
- 客户端超时后服务端继续生成并占用显存 → 每个请求带时间预算（请求超时 −3 s），超时即停并回 BUDGET_EXCEEDED。
- transformers 5.x 需要 torch ≥ 2.6；最新 flash-linear-attention 需要比 Triton 3.4 更新的版本 → 固定 torch 2.8.0 / fla 0.4.0。

未做：现场麦克风/扬声器实测、实时 UI 中的端到端测试、`causal_conv1d` 加速。物理验收 NOT_RUN。
