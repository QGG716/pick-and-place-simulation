# 远程 GPU 检测服务器部署

现场工作台通过 SSH 在 GPU 服务器上运行 Grounding DINO + SAM（`detector.backend=remote_ssh`）。
服务器不开放端口；每次连接由现场电脑上传 `hf_detector_core.py` 与 `remote_detector_server.py`，
并校验两者及全部模型文件的 SHA-256 与现场电脑一致，否则拒绝推理。

服务器目录 `/root/autodl-tmp/eco65-gpu-inference`：

1. 上传模型：`rsync -a --relative models/huggingface/<dino> models/huggingface/<sam> root@<host>:<目录>/`
2. 下载依赖：上传 `closure.txt`、`fetch_wheels.py`，在服务器运行
   `python3 -I fetch_wheels.py closure.txt wheels > fetch.log`。每个 wheel 都按索引 SHA-256 校验，
   torch/torchvision 按 download.pytorch.org 官方索引的哈希校验；并行分段下载用于绕过单连接限速。
3. 安装：`bash install_venv.sh`，离线 `--no-index --require-hashes` 安装到 `/root/eco65-gpu-infer-venv`，
   版本与现场 `.venv-workbench-cuda` 完全一致（torch 2.5.1+cu124、transformers 4.44.2）。
4. 自检：现场电脑运行 `.venv/bin/python tools/remote_detector/selftest.py <task_id>`，
   用保存的真实帧比较远程 GPU 与现场 CPU 的检测框和掩码。这不是现场验收。

连接参数（主机、端口、用户、密钥）只写在被忽略的 `outputs/workbench/remote_detector.local.json`。
