# S50 固定 100 mm 布局：整面 40/40 箱完成

2026-11-12。新安装从完整 40 箱开始，按原有高层优先、中心向两侧展开的顺序完成全部抓放与输送，剩余 0 箱；末态机械臂返回初态，横带回到 600 mm。没有跳过失败目标、移动装置或移动剩余箱堆。原 105 mm 安装完成的 3 箱属于另一份历史结果，不计入本次 40 箱。

[打开完整三维回放](viewer.html) · [逐箱结果与姿态数据](result-summary.json) · [完整计算记录（无损 gzip）](cycle.json.gz) · [临时参数与 TODO](TODO.md)

查看器支持播放、暂停、时间轴和逐箱选择。总动画时间约 54.3 分钟，采用保守仿真时序；可选择 8×，也可直接点击阶段查看。工具、每个箱体的附着/释放、独立出料、横带与纵带联动均来自本次实际关节序列和事件。

## 本次改动

- 上臂前倾采用实际 J2→J3 连线在世界 XZ 平面相对竖直的倾角。暂以朝箱堆 30° 内不加罚、超过后连续加罚排序；这是软偏好，未改关节限位。
- 在原 105 mm 短支架前增加 80 mm 同轴法兰隔距段，直径 118 mm。它把宽安装板整体移离腕部。背板距法兰 185 mm，压缩吸盘接触面距法兰 252.5 mm；原 200 mm 延长段仍移除。
- 隔距段有实际可视和碰撞网格，始终随法兰运动。原吸具、72 个吸盘、支架前部 CAD 开口和器件相对位置保留；FANUC 的原连接配置不变。
- S50 默认配置已切换为此安装。原 105 mm 配置保留在 `configs/robots/huayan_s50_short105_kinematic_sim.yaml`。

固定安装轴心为 [−1.25, 0.15, 0.90] m；横带前沿至箱堆 100 mm。横带行程 550–1500 mm，按当前箱底减 100 mm 并裁剪到行程范围。底盘、支座、两条传送带、剪叉和车厢保持原相对安装关系。

## 安装恢复的实际结果

以下比较针对此前停止的 `carton_l07_c00`，诊断候选不计作执行完成数。详见[对比记录](mount-comparison.json)。

| 安装或吸取方式 | 实际结果 |
|---|---|
| 原 105 mm 短支架、正吸 | 抽离约 394 mm 时宽安装板碰到小臂 |
| 单纯把短支架加长至 150 mm | 宽板仍贴近腕部，约 345 mm 处仍碰小臂；未采用 |
| 原 105 mm 支架 + 50 mm 同轴隔距段 | 安装几何检查通过；首次干涉推迟到约 541 mm，仍未完成抽离 |
| 原 105 mm 支架 + 80 mm 同轴隔距段 | 两个候选通过完整约 615 mm 直线抽离；随后用于本次完整 40 箱运行 |
| 原 105 mm 安装，内侧面吸取 | 157 次 IK 记录、82 次收敛；接触构型均被上臂/横带或小臂/下层邻箱碰撞拒绝 |

新隔距段的固定安装检查通过；全任务另按每个实际采样状态检查机器人、工具、箱体和环境。此次采用正吸 21 箱、顶吸 19 箱；带载转运采用直线 38 箱、短弧 2 箱。带载 TCP 路长约 0.787–1.503 m，名义释放落差为 49.995–50.000 mm。

## 任务与偏好结果

40 箱均完成预抓取、接近、吸取、分离抬升、直线抽离、转运、释放、名义落带、退出、避让返回和出料；每箱的阶段状态均在 `result-summary.json` 中列出。横带依次使用 1500、1400、1100、800、550 mm 接收高度，结束后回到 600 mm。

此前受阻的顶层 `carton_l07_c00` 已完整完成。其抽离 J1 转幅约 23.76°，上臂朝箱堆倾角约 38°→21°，全带载过程小臂相对水平面的夹角绝对值最大约 29.80°，J4→J5 基本朝 −Z；释放落差约 50 mm。

**软偏好没有全部满足。** 本次已通过的底层路径仍包含较大前倾：最大约 86.82°，发生在 `carton_l00_c00`。顶层和第二层的 `c03` 使用腕朝上的分支；顶吸时 J4→J5 多为近水平。部分箱子的 J1 抽离转幅和小臂水平夹角也超过偏好。它们均是受检查的实际求解结果，未隐藏或手动改动画，也不据此宣称其他构型不可能。

[顶层 c00 吸取姿态](keyframes/c00-grasp.png) · [完整抽离姿态](keyframes/c00-withdraw.png) · [80 mm 隔距段近景](keyframes/c00-connector.png) · [释放姿态](keyframes/c00-release.png) · [最后一箱释放](keyframes/last-release.png) · [整面卸完后](keyframes/last-state.png)

## 关键姿态的横带距离和水平占用

以下距离来自真实机器人网格与横带几何。水平占用为安装参考系 X×Y 外包范围；本布局该参考系的水平轴与世界轴平行。活动连杆为 1–6，肘部为 2–3，腕部为 4–6。这里只测量记录的关键帧，不作为连续路径最小净空证明。

| 箱体/阶段 | 部件 | 横带表面距离 mm | 水平占用 X×Y mm |
|---|---|---:|---:|
| 顶层 c00 抽离结束 | 活动连杆 | 374.9 | 956.0 × 1180.6 |
| 同上 | 肘部 | 616.2 | 956.0 × 1030.8 |
| 同上 | 腕部 | 715.2 | 217.0 × 146.7 |
| 顶层 c00 释放 | 活动连杆 | 250.5 | 1164.5 × 746.6 |
| 同上 | 肘部 | 369.8 | 1164.5 × 746.6 |
| 同上 | 腕部 | 250.5 | 217.0 × 151.9 |
| 最底层 c04 释放 | 活动连杆 | 511.7 | 1005.0 × 1121.7 |
| 同上 | 肘部 | 511.7 | 602.9 × 1110.7 |
| 同上 | 腕部 | 602.5 | 260.0 × 282.7 |

全部关键帧的边界坐标及距离见 [keyframe-compactness.json](keyframe-compactness.json)。

## 启动与记录

在已安装 Pinocchio/Coal 的现有 Linux 规划环境运行：

```bash
python tools/run_s50_face_kinematic_sim.py --config configs/robots/huayan_s50_spacer80_kinematic_sim.yaml --gaps .1 --allow-side-suction --output outputs/s50_spacer80_fixed_face
```

授权服务器上的完整命令：

```bash
ssh -p 30794 root@connect.westd.seetacloud.com "chroot /root/m710-gap-search-20261107/rootfs /bin/bash -lc 'cd /work-s50-step1-kinematic-20261112 && OPENBLAS_NUM_THREADS=1 /opt/planning-venv/bin/python tools/run_s50_face_kinematic_sim.py --config configs/robots/huayan_s50_spacer80_kinematic_sim.yaml --gaps .1 --allow-side-suction --output outputs/s50_spacer80_fixed_face'"
```

现有 `viewer.html` 可直接离线打开，无需重新计算。最终计算发生在 `outputs/s50_spacer80_full_face02_20261112/gap_100`。

GitHub 中约 220 MB 的原始 `cycle.json` 以 `cycle.json.gz` 无损保存；`attempt02/cycle.json.gz` 是相同原始记录的归档副本。解压后的 SHA-256 与本次已验证原件完全一致，详见 [archive-manifest.json](archive-manifest.json)。查看器自带完整播放数据，无需解压。需要重跑轨迹检查或导出时，在仓库根目录执行：

```bash
python -c "import gzip,pathlib; p=pathlib.Path('docs/experiments/s50_spacer80_fixed_face_20261112/cycle.json.gz'); p.with_suffix('').write_bytes(gzip.decompress(p.read_bytes()))"
```

记录包含 12,083 次 IK 尝试（11,080 次收敛）、117,238 次已保留的碰撞状态检查和 51,446 个回放状态。完整场景累计记录墙钟约 75.2 分钟，包含失败搜索与已记录中断成本；不作为实机节拍。

初次运行完成两箱后，在空载边界提高了近半圈 J1 换位的软成本，随后继续同一几何场景。前 2,205 个状态逐项一致；保留了 [attempt01](attempt01/cycle.json) 的取消记录与 [attempt02](attempt02/run.log) 的日志。数值搜索明确重启，未声称保留 RRT 树；中断时尚未写入检查点的碰撞查询次数不可得，没有把这部分成本记为零。

必要服务器回归 12 项通过，见 [focused-tests.txt](focused-tests.txt)。全部状态的独立 JS FK、关节范围、附着变换、箱体计数和返回检查通过，见 [replay-checks.json](replay-checks.json)。浏览器检查与截图保存在 [keyframes](keyframes/browser-checks.json)，核对播放、暂停、拖动、逐箱状态、4 个连接零件和前倾角显示。

原始结果 SHA-256：`f8c3aa6df540f3a2b9053f4b07668667e8aad38409c3d3883f38adb6419d4f33`。这是运动学场景完成；控制器零位映射、制造配合与动力学仍按 [TODO](TODO.md) 后续校准。
