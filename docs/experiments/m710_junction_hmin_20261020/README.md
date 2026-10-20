# c00 Hmin 与纵带缩宽证据

两轮均 **0/1、37→37 箱**，没有完整抓放。0.60 m 纵带轮次返回数值 IK，但没有通过碰撞检查的接近轨迹。查看 [中文报告](../../m710_junction_hmin_20261020.md)。

下载 `offline-viewers.zip`，完整解压后打开 `index.html`。全部 HTML 内嵌脚本和模型，不依赖网络。各失败候选独立展示，只播放已经检查通过的初始降带；没有接近、接触、抽离、下落或清空动画。候选查看器允许检查被拒绝的真实 q，明确不是执行轨迹。

`candidate-prefix-side.mp4` 为 0.70 m 第一候选；`width060-prefix-side.mp4` 为追加 0.60 m 第一候选。均为 640×360、5 fps，左下状态面板，6.2 s；每段降带 4 s 只是几何展示，机器人没有接受运动。真实机构速度和升降耗时未知。

## 固定源码与原始结果

- `7be1f05c2c3d3d0af53198199e5ec7d446253bd4`：Hmin 策略及 0.70 m 轮次；`source-manifest.json` 覆盖 281 文件。
- `3f177b11d3c17f9efee9d4b4e32fee7f09583879`：用户追加的 0.60 m 纵带试验；`width-source-manifest.json` 覆盖 282 文件。
- `raw-runs.tar.gz`：`formal` 初始化错误、`formal-envfixed` 正式 0.70 m 失败及全部请求/搜索/结果，未删除失败。
- `width060-raw.tar.gz`：追加试验全部记录，包含 365 个未搜索选项、原生失败见证、实际初态和新布局身份。
- `predecessor-preflight.json`：只读恢复历史 c02/c03/c01 的白名单、哈希和 37 箱起点。旧 c00 成功数据未被读取。
- `official-asset-hashes.json`：22 个官方模型资产逐文件一致；`width-native-reuse.json`：47 个原生源码精确一致和 worker 哈希。
- `artifact-integrity.json`：交付文件 SHA256。原始 tar 直接无损压缩 JSON，没有需要再展开的内部 JSON.gz。

```bash
mkdir -p /new/hmin-evidence
tar xzf raw-runs.tar.gz -C /new/hmin-evidence
tar xzf width060-raw.tar.gz -C /new/hmin-evidence
python tools/verify_m710_top_layer_batch.py /new/hmin-evidence/formal-envfixed
python tools/verify_m710_top_layer_batch.py /new/hmin-evidence/width060
```

核验状态 `PASS_FAILURE_RETENTION_UNCHANGED_COMMITTED_ROOT` 只表示失败保留和提交根未变；不是任务完成。

## 复现规划

在新的隔离目录检出对应冻结源码，使用既有 Humble / MoveIt 2.5.10 / MTC 0.1.3 及项目 Python 环境。官方资源目录取已核验的历史冻结目录，使 URDF 资源 URI 与归档模型身份一致。不能将其他路径下生成的不同模型 fingerprint 静默忽略。

```bash
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-seam-approach-formal-v2-20261020
source /opt/ros/humble/setup.bash
cmake -S ros2/m710_moveit_backend -B /new/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/new/install
cmake --build /new/build --target m710_moveit_worker m710_coupled_receiver_test -j2
cmake --install /new/build
```

新 worker 启动脚本必须显式包含插件目录；CMake install 不提供顶层 setup.bash：

```bash
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/new/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/new/install/lib:$LD_LIBRARY_PATH
exec /new/build/m710_moveit_worker
```

前序归档来自基线证据 `m710_seam_approach_20261020/v2-on.tar.gz.part-*`；按该目录 archive-index 验证后拼接。程序还会核对归档 SHA256，仅读取允许的 8 个成员：

```bash
python tools/plan_m710_junction_single.py \
  --archive /new/v2-on.tar.gz --output /new/c00-hmin \
  --worker-command /new/worker.sh --worker-binary /new/build/m710_moveit_worker \
  --source-commit 7be1f05c2c3d3d0af53198199e5ec7d446253bd4
```

对于用户追加缩宽试验，另建对应源码目录和输出目录：

```bash
python tools/plan_m710_junction_single.py --longitudinal-060 \
  --archive /new/v2-on.tar.gz --output /new/c00-width060 \
  --worker-command /new/worker.sh --worker-binary /new/build/m710_moveit_worker \
  --source-commit 3f177b11d3c17f9efee9d4b4e32fee7f09583879
```

服务器实际使用 `/opt/planning-opt-20261003-venv/bin/python`。若使用 git archive 而非 checkout，再提供匹配的 `--source-manifest`。入口固定 seed=71070、任务 300 s、共享阶段 90 s；无次数配额。不要覆盖旧输出目录。OMPL seed 固定；KDL 内部重启随机源不由该接口控制，共享服务器负载也影响墙钟，不保证逐次相同时间或构型。

## 复现展示

使用当前交付源码读取结果，不再规划。0.60 m 页会从 protocol 的 active_layout_file 读取同一配置，并核对原生外部轴末态的完整固定几何。

```bash
python tools/export_m710_top_layer_replay.py /new/hmin-evidence/width060 \
  --seed 71070 --attempt attempt-001 --output /new/view/prefix-001.html
python tools/export_m710_approach_comparison.py /new/hmin-evidence/width060 \
  --seed 71070 --output /new/view/candidates.html
node tools/record_m710_top_layer_replay.mjs /new/view/prefix-001.html \
  /new/frames /path/to/chrome --hardware-rendering --side-closeup
ffmpeg -nostdin -framerate 5 -i /new/frames/frames/frame-%05d.png \
  -c:v libx264 -crf 20 -pix_fmt yuv420p -movflags +faststart /new/prefix.mp4
```

```text
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
```
