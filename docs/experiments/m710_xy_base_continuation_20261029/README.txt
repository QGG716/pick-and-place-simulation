2026-10-29 允许 XY 基座移动的 40 箱完整卸箱规划

打开 index.html 播放完整 4 倍速视频（unload-40-4x.mp4，640×360，5 fps）。
replay.html 提供交互回放；report.html / report.txt 为逐箱与搜索报告。
这是原生轨迹规划、名义接收和理想出料的完整接受链；physical_execution=NOT_RUN。

从完整 40 箱场景的原 31 箱接受前缀继续，新增 9 箱，剩余 0 箱。
最终采用两次 +X 平移：400 mm + 200 mm。只移动机械臂及原实体台座，
底盘、输送带和箱堆固定；安装高度 Z=0.900 m 不变。
每次空载平移保持实际关节末态，吸盘关闭；检查完整机器人、工具、台座支撑及环境。
未合并不同布局的独立成功数，没有假定移除箱体，没有新增碰撞豁免。

本次还修复接收面高度的浮点舍入误拒绝：已通过 1e-9 高度一致性检查后，
释放公式使用冻结的声明高度。释放公式变化 1e-12 仍拒绝，碰撞间隙未放宽。
27 项相关回归和 3 项真实原生证据篡改回归通过；独立校验覆盖完整 40 箱父链、
2 段平移、266 个回放阶段，全部 40 个目标及 0..40 出料计数已在视频采样中出现。

服务器实际运行源码由 planning-source-manifest.json（v6）记录，
独立验证/回放导出由 export-source-manifest.json（v3）记录。
source_commit 是历史参考提交，不足以描述开发期间未提交修改；以清单中逐文件
SHA-256 和本 Git 提交为准。视频生成代码单独记录于 presentation-manifest.json。
依赖的原 31 箱和 29 箱历史报告保留在相邻目录，均不修改。

服务器证据目录：
/root/autodl-tmp/m710-l05-x500-20261022/rootfs/xy-base-continuation-20261029
原始档案位置及哈希见 server-evidence.json；原始证据包仅保留在服务器。

重制视频（安装可选依赖 playwright、imageio-ffmpeg，及 Chromium）：
python tools/render_m710_batch_video.py <本目录/replay.html> <新输出.mp4> --speed 4 --fps 5
本机 Edge 可通过 --browser 指定。视频生成只采样回放时间，不更改规划/物理时间步。
