# 连续 RGB-D → 当前算法 → ROS/RViz

观看 [原速录屏（14:05）](evidence/rgbd-video-20260920/demo-original-speed.mp4)、
[16 倍速摘要（52.8 秒，明确标注加速）](evidence/rgbd-video-20260920/demo-summary-16x.mp4)、
[真实 RViz 截图](evidence/rgbd-video-20260920/rviz-final.png)。
原速文件完整保留模型与几何处理的等待；摘要不用于证明实时性能。

本次输入实测 **8.64 FPS**，实际新完成组频率 **0.002414 Hz**，覆盖等待帧组 **94** 次。
同一 SAM 仅加载一次，下面每组包含上、下两个模组，各一次 SAM 和几何调用。

| 处理帧 | SAM 合计 | 几何合计 | 播放该帧至 ROS 确认（墙钟） | 源时间滞后 | 融合代表面 |
|---|---:|---:|---:|---:|---:|
| 32 | 28.95 s | 408.06 s | 467.82 s | 9.50 s | 67 |
| 602 | 21.35 s | 313.00 s | 817.62 s（含等待槽 458.18 s） | 0 s | 60 |

两组都完成真实算法与 ROS 交付，完整箱体接受数均为 0。第二组上模组保留 mask 10，明确拒绝 label 2 的不可投影候选。
源时间滞后 0 不代表墙钟低延迟。完整数据与计时口径见 [performance.json](evidence/rgbd-video-20260920/performance.json)。

## 启动

已验证环境为服务器 Ubuntu 22.04 / ROS 2 Humble / Python 3.10。
ROS 与 RViz 用系统 Python；算法使用既有 GPU 虚拟环境。
不要将虚拟环境解释器的符号链接解析为系统 Python。

在已有服务器数据与本次 colcon 安装上，一条命令启动界面和两组实际算法：

```bash
cd /root/autodl-tmp/v05-acceptance/video-demo-20260920/repo
DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 \
HUMBLE_INSTALL=/root/autodl-tmp/v05-acceptance/video-demo-20260920/install \
bash tools/run_rgbd_video_demo.sh \
  --recording ../recording-01/sequence-prefix.json --output ../my-new-demo \
  --models /root/autodl-tmp/v05-acceptance/model-manifest.json \
  --vision /root/vision-fixed --algorithm-python /root/v05-gpu-venv/bin/python \
  --record --auto-start
```

输出目录必须不存在。默认完成后保留界面，Ctrl+C 关闭；加 `--auto-close` 则完成后保留 15 秒再退出。
省略 `--auto-start` 时先打开 READY 界面，`touch <output>/START` 才开始；录屏模式要求 `--auto-start`，保证先收到屏幕帧再放行输入。

其他 Humble 机器先构建 `ros2_ws`，将 `HUMBLE_INSTALL` 指向其 install，并提供原始 RGB-D 数据和模型路径。
桌面会话直接使用已有 DISPLAY；无桌面时，本次使用 `Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp` 和 Openbox。
最小图形依赖为 `ros-humble-rviz2 xvfb openbox ffmpeg python3-pyqt5 fonts-dejavu-core`；实际 RViz OpenGL 4.5 渲染器为 Mesa llvmpipe。

服务器另有仅监听 loopback 的 x11vnc（5901）和 noVNC/websockify（6081）。本机连接：

```powershell
ssh -N -L 127.0.0.1:18777:127.0.0.1:6081 -p 41855 root@connect.westb.seetacloud.com
```

浏览器打开 `http://127.0.0.1:18777/vnc.html?autoconnect=true&resize=scale&view_only=true`。
显示服务未绑定公网地址，访问经过 SSH 身份认证；仓库不保存密码。

## 数据与操作

- 本次重新采集同一 Isaac 会话的连续双 RGB-D 数据；沿用已有箱堆、模型、材质、灯光。
  虚拟双相机采集架沿世界 +Y 平移，未让机械臂或真实机器人运动。不是 A/B/C 图片插帧。
- 计划采集 120 组，SSH 断开导致采集中断；保留全部原文件，通过原载荷校验的连续前缀为 96 组。
  帧 32→602，源时间 0.533→10.033 s。第 97 组不完整，索引在此停止，没有跨过坏帧拼接后续数据或修写 binding。
  `tools/index_rgbd_recording_prefix.py` 只建立新索引。界面明确显示采集中断与 `RGB-D RECORDING`。
- 算法输入保持 2592×1944 RGB、米制深度、K、采集时 T_W_C、原时间和帧身份；预览缩至 640×480。
  点云按像素步长 10 降采样用于显示，深度预览步长 4，均不改变算法输入。
- 左侧为当前输入；中间为实际处理的原帧及 SAM/面片，首组允许阶段性显示，之后保留最近完成组。
  右侧点云、相机 TF、融合面片来自同一选中组，切换时显式删除旧层。
  在 RViz `Panels → Displays` 中启用 `Upper depth (toggle)` 或 `Lower depth (toggle)` 查看深度，源结束后仍可查看末帧。
- 一次一个双模组算法任务，一个最新配对等待槽；新输入覆盖等待组并累计丢弃数。
  同一个 resident SAM 服务两组输入；每模组一次 SAM、一次几何，无自动重试。
- 输入结束后显示 `SOURCE ENDED / PROCESSING`，继续保留真实等待。源时间差与墙钟处理耗时分开。
  新结果频率只计实际完成并经 ROS 确认的组，不计 world 心跳或旧结果重复发布。

## 边界与已知限制

- ORACLE-PROMPTED SAM：GT 框仅作既有提示；最终分割、米制面片与融合仍运行当前算法。
  现有 `isaac_sensor_adapter_node` 会向 `/unloading/perception` 发布 GT observation，因此本启动器不启动它。
  本演示仅由已验证算法产物的 finite replay 节点发布该话题，预览与状态独立在 `/demo/*`。
- 历史只读展示不刷新原 capture time，不修改 /clock 或 freshness，不启动执行桥。
  `planning_admissible=false`；未知体积仍未知，观测面不被冒充为完整箱体或六维抓取位姿。
- 第一轮实际录制的第二组上模组发生 `FACE_BEHIND_CAMERA`，失败产物和原速录屏保留在服务器 `demo-03`。
  定向修复沿用最终校验已有的几何拒绝语义，在投影前记录 `FACE_BEHIND_CAMERA` 候选拒绝：
  保留实例、支持区域、测量平面、失败坐标和拒绝原因，并在界面显示拒绝数量；不发布无效面片，也不据此补全箱体。
  没有删除困难实例、修改阈值、缩小算法分辨率或重新选择输入。推理/worker/文件与产物校验故障仍是技术失败。
  曾尝试已有内接矩形分支，但同一输入仍无效，其失败录屏保留在 `demo-04`；该尝试不在最终实现中。
- 当前是短录制输入的异步展示，不是墙钟实时感知；GPU 服务器同期有其他任务，耗时不是隔离性能基准。
  完整原始数据留在服务器 `recording-01`，未将约 5 GB 的深度/原图复制进 Git。
- CPU 编排与 ROS 故障测试使用明确标注的替身；录屏中的 SAM、几何、融合和 RViz 都是真实运行。
  默认 Windows pytest 仍有既有 GBK 读取失败；`PYTHONUTF8=1` 下 646 项通过，1 项 deselected。
  相关编排 68 项、米制几何 34 项、真实 Humble 72 项通过；没有扩大 C 组专项研究。
