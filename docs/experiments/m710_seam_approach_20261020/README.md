# 接缝与箱间接近原始证据

最终修正版规划源码：`feba0c5491ffe4958e61e30ee3baa3d4ee954e5e`，清单为 `frozen-source-v2-manifest.json`。第一版源码为 `666235d74a55b0b8e745b8e890572649f8ec01f3`，清单为 `frozen-source-manifest.json`。最终交付的查看器、统计和报告修改不参与这些批次的求解。两版原生 47 个源文件逐字节一致，复用同一原生二进制，见 `native-binary-reuse-v2.json`。

第一版 `comparison-off` / `final-on`，以及修正版 `comparison-v2-off` / `final-v2-on`，都从各自完整 40 箱初态运行预先固定的 71070、71071。每版 OFF/ON 使用同一源码、原生二进制、几何、预算和恢复模式；只有空载接近的直接方法偏好不同。修正版开启组同时是最终回归，不补跑单箱或选择最快批次。第一版因整批运动时间退化而触发一次定向修正，其负结果完整保留在 `comparison-summary-v1.json` 和 `v1-*.tar.gz` 归档中。`dev-01` 是凸实体包围检查补强之前的开发记录，不计正式成绩。

最终四批均为 5/5、下层 35 箱不变；ON 的机器人轨迹时间为 161.881 / 180.932 s，OFF 为 172.631 / 190.495 s。ON 整批计算为 347.427 / 334.496 s，OFF 为 330.674 / 343.711 s。计算成本和个别路径仍有退化，详见 [报告](../../m710_seam_approach_20261020.md) 及 `comparison-summary.json`，不是统计成功率保证。

归档保留每个过程尝试的原生模型、请求、返回、搜索记录、计划、失败及检查点。大 JSON 使用逐文件无损 gzip；`lossless-compression-manifest.json` 给出原始 SHA256 及压缩前后字节数，压缩后已解码比对原始哈希。传输分片按文件名顺序拼接；`archive-index.json` 给出分片与整体 SHA256。验证哈希后解压 tar，再按清单解压 `.json.gz` 即恢复原始 JSON，便可运行保存后核对工具。

`old-direct.tar.gz` 为修改前旧几何的固定输入诊断，包含两种子 c02→c03、c03→c04 的原端点、世界、权限、旧轨迹、PTP 请求及真实失败见证。没有将其 q 或路径输入新正式批次。

| 归档逻辑名称 | 内容 |
|---|---|
| `development.tar.gz` | 开发批次、当时未提交源码快照和 worker；不计正式成绩 |
| `v1-off.tar.gz` / `v1-on.tar.gz` | 第一版四批、全部失败、协议和保存后验证 |
| `v2-off.tar.gz` / `v2-on.tar.gz` | 最终修正版四批、全部失败、协议、完整阶段和检查点 |
| `validation.tar.gz` | 定向测试日志、593 状态机构检查、1190 个第一版实际 q 行程样本、原生 worker/插件及源码清单 |

上述大归档按最多 40 MiB 分片提交，文件名为 `.part-000`、`.part-001` 等；它们不是缺失或裁剪的原始记录。`old-direct.tar.gz` 单文件保留。第一版和修正版视频分别为 `v1-seed-*.mp4`、`seed-*.mp4`。`*-ffprobe.json` 记录实际编码尺寸、帧率、时长和帧数；`browser-checks.json` 保存查看器无脚本错误、实际 q/FK 和箱数核对。

`offline-viewers.zip` 解压后打开 `index.html`。布局、旧诊断、OFF、ON 各自使用对应世界；完整回放中的关节来自本轮原生轨迹，机构来自同源函数。路径查看器显示实际原生候选和最终接受状态、端点、方法、逐轴累计及 FK 指标。大型搜索树保留在原始归档，展示页明确标记未嵌入，不伪造树。USD 是独立可移植碰撞几何，未启用动力学。

视频为 640×360、5 fps、1× 轨迹时间的展示派生物。机器人轨迹时间、机构演示时间、名义下落和事件展示分别记录；机构演示不是硬件时间。释放后的名义下落与 `IDEAL_RECEIVER_DRAIN` 不是实测物理接收或出料。

共同边界：`qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`、`execution_ready=false`。

## 复现

在独立的上述源码树及既有 ROS Humble / MoveIt 2.5.10 / Pilz 2.5.10 / MTC 0.1.3 环境内，沿用仓库官方机器人及工具资产；不需要 Isaac 启动：

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=src M710_MOVEIT_ASSET_ROOT="$PWD"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
cmake -S ros2/m710_moveit_backend -B /new/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/new/install
cmake --build /new/build --target m710_moveit_worker -j2
cmake --install /new/build
export AMENT_PREFIX_PATH="/new/install:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="/new/install/lib:$LD_LIBRARY_PATH"
python tools/plan_m710_top_layer_lifted.py \
  --source-commit feba0c5491ffe4958e61e30ee3baa3d4ee954e5e \
  --seeds 71070 71071 --no-direct-first \
  --worker-command /new/build/m710_moveit_worker \
  --worker-binary /new/build/m710_moveit_worker --output /new/off
python tools/verify_m710_top_layer_batch.py /new/off
# 同一源码和预算，再运行 --direct-first，输出到全新的 /new/on。
```

Git 源码树使用上述实测 SHA；若从没有 `.git` 的源码包运行，增加 `--source-manifest /path/to/frozen-source-v2-manifest.json`。清单必须匹配实测源码，不能把交付阶段修改过的展示工具伪装为原实测文件。

恢复归档示例（在证据目录运行，目标目录需预先新建）：

```bash
cat v2-on.tar.gz.part-* > /new/v2-on.tar.gz
python - <<'PY'
import hashlib, json
from pathlib import Path
index = json.loads(Path("archive-index.json").read_text())
archive = Path("/new/v2-on.tar.gz")
assert hashlib.file_digest(archive.open("rb"), "sha256").hexdigest() == index[archive.name]["sha256"]
PY
tar xzf /new/v2-on.tar.gz -C /new
python - <<'PY'
import gzip, hashlib, json
from pathlib import Path
root = Path("/new/final-v2-on")
for row in json.loads((root/"lossless-compression-manifest.json").read_text()):
    data = gzip.decompress((root/row["gzip"]).read_bytes())
    assert len(data) == row["bytes"] and hashlib.sha256(data).hexdigest() == row["sha256"]
    with (root/row["file"]).open("xb") as out:
        out.write(data)
PY
python tools/verify_m710_top_layer_batch.py /new/final-v2-on
```

展示工具使用最终交付源码，读取保存的数据，不求解新路径：

```bash
python tools/export_m710_top_layer_replay.py /new/on \
  --seed 71070 --output /new/view/on-full-71070.html
python tools/export_m710_approach_comparison.py /new/on \
  --seed 71070 --output /new/view/on-71070.html
python tools/preview_m710_conveyor_lift.py --usd \
  --source-commit feba0c5491ffe4958e61e30ee3baa3d4ee954e5e \
  --output /new/view/layout
node tools/record_m710_top_layer_replay.mjs /new/view/on-full-71070.html \
  /new/recording /path/to/chrome --hardware-rendering
ffmpeg -nostdin -framerate 5 -i /new/recording/frames/frame-%05d.png \
  -c:v libx264 -crf 20 -pix_fmt yuv420p -movflags +faststart /new/seed-71070.mp4
```

`--usd` 只需要已有 pxr 环境；不启动 SimulationApp。旧输入诊断使用历史配置 `m710id70_unloading_layout_v1_rear_fixed_20261019.yaml` 及归档内对应旧 worker/source，不能用新几何替代。OMPL 种子已固定；KDL 内部数值重启随机源不由该接口控制，且共享服务器负载会影响墙钟。两种子结果不构成统计成功率保证。
