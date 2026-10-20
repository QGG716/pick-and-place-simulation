# 横带剪叉缩宽证据

300 mm、260 mm 两档中心距均未消除原有 J2／剪叉净空失败。260 mm 新原生规划新增 **0/1，37→37 箱**。详见 [中文报告](../../m710_transverse_scissor_20261020.md)。

完整解压 `offline-viewers.zip` 后打开 `index.html`。四个前缀页分别显示四个独立候选的已检查初始降带；`candidates.html` 显示 144 个被拒绝的真实请求，失败目标不作为执行轨迹。视频 `scissor026-prefix-side.mp4` 是 640×360、5 fps 的候选展示，无物理执行。

主要文件：

- `paired-j2-probe.json`：300 mm 冻结源码上的成对诊断。
- `paired-j2-probe-026.json`、`frozen-paired-j2-probe-026.json`：260 mm 开发目录及冻结目录上的同输入诊断，均保留。
- `formal026-raw.tar.gz`：完整新原生规划、所有请求、失败和日志。SHA256 为 `af8766e025e5c560a5210c18e09f014eb634aa167e37232bf947e6d78ad04284`。
- `summary.json`、`verification.json`：原生规划指标和提交根未变的保存后核验。`PASS_FAILURE_RETENTION_UNCHANGED_COMMITTED_ROOT` 不是任务成功。
- `source-manifest.json`、`026-source-manifest.json`：分别对应 `501ee61ddd2a63b779187b901e5e2eb580f894d3`、`99569e7b7e4eaa2f499b3c1fde9fcaa0a606ca23`。
- `native-reuse*.json`：47 个原生源文件相同及 worker 哈希证据。
- `focused-tests*.log`、`browser-checks.json`、`ffprobe.json`：定向测试、页面检查及视频参数。
- `artifact-integrity.json`：本目录交付文件的字节数和 SHA256；`.gitattributes` 禁止换行转换。

复现使用已隔离的 ROS Humble / MoveIt 2.5.10 / MTC 0.1.3 环境和 `/opt/planning-opt-20261003-venv/bin/python`。官方模型资源保持同一冻结资源根；不放宽模型／工具身份检查。先查看上一轮 [环境与 worker 配置](../m710_junction_hmin_20261020/README.md)。

```bash
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-seam-approach-formal-v2-20261020
python -m pytest -q tests/test_transverse_scissor_width.py \
  tests/test_longitudinal_width_variant.py tests/test_conveyor_lift.py \
  tests/test_junction_reception.py

# 只比较已失败目标，不作为规划输入。
python tools/probe_m710_transverse_scissor_width.py /previous/width060 \
  --variant 026 --output /new/paired-j2-probe-026.json

# 在 99569e7 冻结源执行；输出目录必须尚不存在。
python tools/plan_m710_junction_single.py --transverse-scissor-026 \
  --archive /previous/v2-on.tar.gz --output /new/formal026 \
  --worker-command /existing/worker-fixed.sh \
  --worker-binary /existing/formal-build/m710_moveit_worker \
  --source-commit 99569e7b7e4eaa2f499b3c1fde9fcaa0a606ca23 \
  --source-manifest 026-source-manifest.json

# 原始结果核验，不再规划。
mkdir -p /new/evidence
tar xzf formal026-raw.tar.gz -C /new/evidence
python tools/verify_m710_top_layer_batch.py /new/evidence/formal026
```

服务器路径为 chroot 中 `/work-transverse-scissor-formal-20261020`（300 mm）、`/work-transverse-scissor026-formal-20261020`（260 mm）；结果目录 `/tmp/transverse-scissor-20261020`。固定起点与原上轮 width060 完全一致，OMPL seed=71070；KDL 内部随机源不由此接口控制，不保证逐次相同构型或时间。未进行额外物理试验或全行搜索。
