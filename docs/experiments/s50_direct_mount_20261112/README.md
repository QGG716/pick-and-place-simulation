# S50 短连接与小 J1 抽离：同一中间箱完成

2026-11-12，`feat/v0.5-s50-moveit2-native-single-carton`，在完整 40 箱初始场景、固定 100 mm 布局中重新求解 `carton_l07_c02`。最终结果为 **COMPLETED_KINEMATIC_SINGLE_CARTON，1 箱完成、39 箱保留**。没有先取邻箱，没有移动箱堆或单独调整机器人与传送带的安装关系。

[直接打开三维回放](viewer.html) · [原始周期](cycle.json) · [短连接近景](keyframes/connector.png) · [抽离姿态](keyframes/withdraw.png) · [释放姿态](keyframes/release.png) · [临时参数与 TODO](TODO.md)

## 本次改变

- S50 使用 **105 mm 短刚性安装间距**：法兰至原背板后平面 105 mm，至实际压缩吸盘接触面 172.5 mm。原长支架及 200 mm 延长段移除，合计比原接触距离缩短 245 mm，没有叠加虚拟工具长度。FANUC 的原连接件和共享工具源资产保持不变。
- 保留原吸具全部其他器件及 72 个独立吸盘。短安装件由原 CAD 的两块侧板后端裁短、安装板前移得到；保留前端开口和原器件接口。三件实际网格同时用于碰撞和查看器，并显示为不透明橙色。
- 对实际求得的直线抽离分支按 J1 转幅、J2 更负及原有姿态软偏好排序。初态也独立求 IK 并优先相近的 J1 构型，避免空载返回被迫跨到另一支。
- 箱体完全抽出后直接向横带的接收点下降，无带载大绕圈。目标保持直立，释放中心约为 [−0.45, −0.10, 1.70] m，横带 1.50 m，释放落差 **50.0 mm**。

## 真实姿态与偏好的取舍

| 项目 | 实际结果 |
|---|---|
| 抽离 J1 | 6.39° → 20.20°，转幅 13.81° |
| 转运末 J1 | 1.20° |
| 抽离 J2 | −92.36° → −53.90° |
| 释放 J2 | −34.72° |
| 带载 TCP 路长 | 约 1.230 m |
| 带载小臂相对水平夹角 | 9.59°～32.07°，软偏好 30° 有约 2.07° 超出 |
| 带载 J4→J5 偏离 −Z | 最大约 0.074° |

**J2 向更大负角度运动的偏好没有同时实现。** 已求得的另一类抽离解可让 J2 到 −126°或 −135°，但 J1 抽离转幅约 44.9°。本次选取小 J1 的分支，在这一类解中优先更负的 J2，并如实保留取舍。接触 IK 中另有大负 J2 构型被上臂与横带／剪叉碰撞拒绝，不能靠扩大限位或忽略碰撞采用。六轴固定 TCP 姿态的直线运动不是冗余问题，不能把 J1 任意锁死后仍假设整条直线可解；这些结果也不证明所有其他姿态都不可行。

## 阶段结果与检查

| 阶段 | 实际状态 |
|---|---|
| 独立初态、横带 600→1500 mm | PASS；初态覆盖完整升降行程 |
| 预抓取、定向接近、48 杯吸取 | PASS；现有关节路径 + 上一点种子逐点 IK + 完整密封环 |
| 分离、直线 −X 抽离 | PASS；未先取邻箱、未横向绕开箱堆 |
| 直接下移转运、50 mm 释放 | PASS；逐点 IK，带载整段连续时间律 |
| 名义落带、定向退出、空载回位 | PASS |
| 原路由出料、横带降回 600 mm、返回 | PASS；首末关节角相同 |

共 **345 次 IK 尝试、1648 次碰撞状态检查、1042 个回放状态**；计算约 51.83 s，保守动画时长约 74.10 s。计算时间不含此前失败重试，也不用于节拍排名。失败的抓取构型与中心 Y=0 接收点 IK 尝试均保留在报告中。

实际结构网格的固定装配检查见 [short-mount-fit-final.json](short-mount-fit-final.json)：原背部器件与 S50 第六连杆、三件新安装件与保留器件共 505 对，51 对进入近距离窄相检查。保留两处原器件与未改变侧板安装面的设计接触，仅在指定侧面及器件范围内接受；其余无相交。法兰至背板的 105 mm 间距和短连接的实际 FK 在浏览器中另外检查。

服务器必要回归 **10/10 通过**，见 [focused-tests.log](focused-tests.log)。Windows 本地不具备 Pinocchio/Coal，相应两项跳过，其余 8 项通过。独立 JavaScript 对全部 1042 状态的 FK、附着相对变换、限位、速度、释放、出料及返回检查通过，矩阵最大误差约 1.8e−15，见 [replay-checks.json](replay-checks.json)。Edge 独立隐藏测试实例的 **15 个阶段、短连接 FK/105 mm 间距、播放、暂停、时间轴全部通过**，关键截图已查看；见 [keyframes/browser-checks.json](keyframes/browser-checks.json)。

## 关键构型的距离与占用

以下是实际 CAD 网格在指定构型的测量，**不是连续路径最小净空证明**。活动连杆为 1～6，肘部为 2～3，腕部为 4～6；水平跨度为安装参考系的 X×Y 包围范围。原始各轴上下界见 [keyframe-compactness.json](keyframe-compactness.json)。

| 构型 | 集合 | 到横带距离 mm | 水平跨度 X×Y mm |
|---|---|---:|---:|
| 抽离末 | 活动连杆 | 294.5 | 998.9 × 773.7 |
| 抽离末 | 肘部 | 294.5 | 895.1 × 735.1 |
| 抽离末 | 腕部 | 711.4 | 217.1 × 148.4 |
| 释放 | 活动连杆 | 191.6 | 1167.0 × 615.8 |
| 释放 | 肘部 | 383.0 | 1089.9 × 478.0 |
| 释放 | 腕部 | 191.6 | 217.0 × 133.7 |

## 启动与失败保留

在该分支、已有 Pinocchio/Coal 的 Python 环境执行：

```sh
python tools/run_s50_kinematic_sim.py --config configs/robots/huayan_s50_short105_kinematic_sim.yaml --output outputs/s50_short_mount
```

授权服务器的明确入口：

```powershell
ssh -p 30794 root@connect.westd.seetacloud.com "chroot /root/m710-gap-search-20261107/rootfs /bin/bash -lc 'cd /work-s50-step1-kinematic-20261112 && OPENBLAS_NUM_THREADS=1 /opt/planning-venv/bin/python tools/run_s50_kinematic_sim.py --config configs/robots/huayan_s50_short105_kinematic_sim.yaml --output outputs/s50_short_mount'"
```

这会重新求解并生成 `outputs/s50_short_mount/viewer.html`。已有 [viewer.html](viewer.html) 是本次实际结果的单文件离线回放，可直接打开，无需启动服务器，支持播放、暂停、时间轴、阶段跳转和短连接近景。吸具跟随真实法兰 FK，吸取后箱子按固定相对变换跟随，释放后独立落带、输送并退出。

记录保留：零间距直连的实际器件／腕部干涉在 `attempt01.json`、`attempt02.json` 和两份 `probe-roll-*.json`；首次短连接运行的另一支初态造成空载 RRT 失败，见 `short-attempt01.log`、`short-attempt01-cancelled.json`；第二次短连接已完成出料，但旧碰撞器把已出料箱体保留为碰撞对象，降带失败见 `short-attempt02.json`。最终修正只在完整箱体越过原 X=−3.2 m 出料界面后移交该箱体，其他 39 箱和环境检查仍启用。

求解端的 [frozen-scene.json](frozen-scene.json) 与周期环境指纹一致。回放使用此原始快照，避免跨 Windows/Linux 重新计算三角函数时末位差异导致指纹变化。厂家、控制器、动力学资格布尔值均未借用或改为已验证；本次仅证明这个单箱周期，未宣称同一布局的整面卸垛已完成。
