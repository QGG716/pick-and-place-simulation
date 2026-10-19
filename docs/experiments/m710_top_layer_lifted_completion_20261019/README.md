# 本轮证据

结论和边界见 [正式报告](../../m710_top_layer_lifted_completion_20261019.md)。本目录记录连续原生规划、同源联动几何、名义重力下落与 `IDEAL_RECEIVER_DRAIN`，没有物理执行。

- `seed-71070.html`、`seed-71071.html`：各自完整批次的离线查看器。下载后用浏览器打开，不依赖 CDN 或联网服务。
- `seed-71070.mp4`：预定第一种子的连续回放，640×360、5 fps、机器人轨迹正常时间；升降演示时间独立标记。
- `checks/`：测试、源码清单、原生父状态链/箱间连续性、浏览器 FK/计数与视频编码检查。
- `raw/`：所有开发失败、首次冻结运行磁盘中断、最终两个完整批次及事后行程诊断的原始记录。归档摘要在 `raw/archive-index.json`。

归档内的 `.json.gz` 是原 JSON 的无损压缩版本，`lossless-compression-manifest.json` 提供原文件名、字节大小及 SHA-256。解压 gzip 后再运行需要 `.json` 的查看/核对工具；不得将磁盘中断的零字节或截断 JSON 当作完整证据。所有实际文件哈希见 `checks/artifact-sha256.json`。

最终完整批次归档为两个有序分片 `formal-complete.tar.gz.part-000` 和 `part-001`，拼接后 SHA-256 必须匹配 `raw/archive-index.json`。在本目录运行：

```python
from pathlib import Path
import gzip, tarfile
raw = Path('raw')
archive = raw / 'formal-complete.tar.gz'
archive.write_bytes(b''.join(p.read_bytes() for p in sorted(raw.glob('formal-complete.tar.gz.part-*'))))
with tarfile.open(archive) as source:
    source.extractall('unpacked', filter='data')
for p in Path('unpacked').rglob('*.json.gz'):
    p.with_suffix('').write_bytes(gzip.decompress(p.read_bytes()))
```

还原后的 `unpacked/run-complete` 可交给 `tools/verify_m710_top_layer_batch.py`。先保留 `.json.gz`，核对原字节哈希后再决定是否移走压缩副本。上面的解包示例使用支持安全 `filter='data'` 的 Python 版本。

`formal-disk-abort.tar.gz` 中的第五箱 plan 写入不完整，明确不计入已完成批次。开发运行可能含 `pending-C` 等开发标记，不能混入冻结源码结果。事后诊断只使用本轮实际 q 进行静态样本查询，不生成轨迹、提供先验或增加完成数。

录屏器的旧软件渲染尝试仅用于画面检查，后续以硬件渲染重新输出同一时间轴；这不是重新规划或物理试验。软件渲染产生的本地中间帧保留在工作区 artifacts 中，交付视频对应已通过全段浏览器检查的硬件渲染记录。

机器人网格来自 `FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`；附 `FANUC-LICENSE.txt`。离线查看器内嵌 Three.js，附 `LICENSE.three`。

`qualification_status=NOT_EVALUATED`；`continuous_swept_proof=false`；`physical_execution=NOT_RUN`；`execution_ready=false`。
