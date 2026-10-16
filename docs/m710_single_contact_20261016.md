# 无先验单目标接近与接触（2026-10-16）

新入口实际完成 c02、c04 的 **pregrasp＋contact**，均在第一个正面 `x-` 成功。没有执行、附着或卸箱；不是新的单箱完整取放或五箱成绩。

- 实测源码：`91e6d2caf1a2decb548992a0c38520d08a932379`，基线 `af35e42d05240676600a311fe9ce6fbfb8f9aae3`。实测期间没有修改源码，两案例各运行一次。
- worker SHA256：`4a5197a1de8213065c7cadf20719d9742c3d8ad675cf18dd203e7d0780a244bd`。
- 状态：`APPROACH_CONTACT_PLANNED_NOT_EXECUTABLE`；`qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`。
- [机器汇总](experiments/m710_single_contact_20261016/summary.json)、[环境/预算冻结](experiments/m710_single_contact_20261016/freeze.json)、[实际运行脚本](experiments/m710_single_contact_20261016/run-cases.sh)。

## 实现与边界

`contact_target.py` 提供小型目标接口及仿真目标提供层。正面由**机器人基座**相对箱体的方向确定，优先级固定为正面、顶面、两侧；通过原有暴露/可移除筛选，每个物理面只取一个中心点。角色与局部物理面 ID 分开，旋转箱体不会把局部侧面改名后误用 front 几何；旧几何函数不能表示的 `x+`、`z-` 显式拒绝。

工具 X 为 6 行短轴，Y 为 12 列长轴（中心跨度 0.240 / 0.528 m）；Z 指向箱内。接触面使用既有名义压缩物理坐标系，满足 `R[:,2]=-n_out`，接近点为 `p_contact+0.1*n_out`，姿态相同。保持物理接触坐标系与虚拟任务 TCP/法兰转换；没有 roll、倾角、面内偏移或退距枚举。

`single_contact.py` 消费一个目标：一次以当前 q 为种子的 KDL `getPositionIK` → 一次 OMPL RRTConnect → 一次 Pilz LIN。显式关闭 PTP、先验、多次 OMPL 及旧候选循环，并在从初始化开始的协议入口阻止先验与旧生成请求。worker 新增受 planning-only/cold 模式限制的 `check_start`，复用现有单状态 Clearance/ProcessPolicy，非法起点停止整个任务。其他现有求解路径未改。

保持原模型、完整工具/箱体、5 mm 外部净空、ACM、阶段权限及优化。目标始终是世界对象，未附着/删除；LIN 使用 pregrasp 实际末样本与原生父阶段。只对已返回 LIN 点做 FK 核查，并按实际终点 FK 进行原有杯面几何检查；无追加密集碰撞采样。普通有界面失败可尝试下一面，取消/截止/身份或起点问题停止整项；共享预算不随换面重置。

## 两个真实案例

| 案例 | 初始世界 | 实际尝试/选中 | 两段结果 | 目标生成(s) | IK阶段(s) | pregrasp(s) | contact(s) | 规划合计(s) |
|---|---|---|---|---:|---:|---:|---:|---:|
| c02 | 配置初始 q，40 箱 | FRONT / x-，1 面 | 完成 | 0.013636 | 0.061249 | 1.350317 | 0.249154 | **1.830751** |
| c04 | 归档撤离末态，移除 c02/c01/c03/c00，36 箱 | FRONT / x-，1 面 | 完成 | 0.015788 | 0.056552 | 9.690194 | 0.241535 | **10.148112** |

这是离线几何状态，不是本次物理实测；c04 只读取归档的起点及已移除列表，未输入旧目标 q、路径、mask 或先验图。c02 两侧因邻箱遮挡筛掉，顶面可用；c04 顶面/两侧可用。正面成功后均未尝试其他面。

初始化分别 **1.729029 / 1.682494 s**，其中场景准备 **0.013113 / 0.012810 s**，不计入规划合计。规划合计含目标生成、预期杯面检查、起点检查、请求构造/往返/处理、两段拼接与终点检查；终点 FK＋杯面检查为 0.008805 / 0.009012 s。IK 阶段内的原生 IK 内核仅 0.000247 / 0.000354 s。上述子区间均不再与父区间相加。

两案各有 **IK=1、RRTConnect=1、LIN=1、PTP=0、先验构建/加载/查询=0**；随机重启、历史解读取、旧运动生成、禁止入口尝试均为 0。初始化只做模型加载与 FK 对齐。实际请求顺序为 init、3 次模型 FK、check_start、IK、OMPL、LIN。两个独立 Task 内各生成两个阶段，父 ID 和关节末/首样本精确衔接，原生时间参数化保留，各段首末速度为零。

接触目标分别为 `[0,0,2.25]`、`[0,0.84,2.25]` m，接近目标 X=-0.1 m，法向均为 `[-1,0,0]`；X/Y/Z 轴分别为世界 +Z/-Y/+X。完整矩阵、起终 q、原生时间化轨迹和 receipt 在：
[c02 原始结果](experiments/m710_single_contact_20261016/c02/result.json.gz)、
[c04 原始结果](experiments/m710_single_contact_20261016/c04/result.json.gz)。

LIN 返回 31 / 32 点；最大法向线偏差 **9.649 / 9.300 μm**，姿态偏差 **3.124 / 5.623 μrad**，均在原 0.1 mm / 0.2 mrad 容差内。两个终点均有 48 个既定命令杯通过实际返回 q 的几何检查。预期 mask 的 actual 字段保持未测量；终点 mask 明确为规划几何，`measured=false, attached=false`，没有 ATTACH 事件。

## 实际检查与计时

| 请求 | c02 / c04 Clearance 状态数 | c02 / c04 AABB 检查数 | c02 / c04 FCL 窄查询数 | c02 / c04 Clearance累计(s) |
|---|---:|---:|---:|---:|
| 起点检查 | 1 / 1 | 8,248 / 7,700 | 15 / 36 | 0.000355 / 0.000423 |
| pregrasp | 768 / 9,481 | 6,334,464 / 55,063,829 | 9,458 / 277,738 | 0.219795 / 2.329473 |
| contact | 32 / 33 | 261,632 / 251,724 | 471 / 494 | 0.007482 / 0.007590 |

只汇总去重后的当前请求聚合记录，不累加 Task 累计快照。ProcessSealInput 每检查请求编译一次；`native_check_reuse=true`、窄阶段资格开关开启。本轮无附着箱体，因此固定工具—持物 box 对资格/复用计数为 0，不声称产生了此类缓存收益。实际 `pair_profile_enabled=false`，四类对象对耗时均为 null；工艺细粒度计时同样关闭。

c04 主要耗时是 pregrasp：阶段 9.690194 s，其中请求往返 9.563918 s、MTC 求解 9.454833 s，Clearance 2.329473 s 嵌套其中。搜索中保留了墙体、刚性工具/邻箱等拒绝见证。没有通过放宽碰撞得到成功；未归因余额不称为 Python 或 IPC。两个案例不用于宣称稳定时延或推算五箱收益，也不与不同机器/任务范围的历史先验数据作因果加速比较。

## 验证、环境与复现

156 项相关 Python 测试通过（其中新增 38 项）；3 个原生测试通过：`m710_native_semantic_event_test`、`m710_request_evidence_test`、`m710_clearance_test`。覆盖坐标/旋转面映射、队列顺序、单次派发、非法/取消/过期起点、碰撞/LIN失败、终点几何、实际末态及禁止先验/旧入口。运行后的 `analyze.py` 只读取现有记录核对连续性和计数，不重新求解或碰撞采样。

环境：现有 Ubuntu 22.04.5 / ROS Humble，MoveIt/Pilz 2.5.10、MTC 0.1.3、FCL 0.7.0、g++ 11.4，Release -O3 -DNDEBUG；Xeon Platinum 8352V，固定 CPU 0–21，OMP/OpenBLAS 各 1 线程。99 个资产文件与上一轮冻结摘要逐一匹配，未升级依赖。seed=71070，整任务 300 s、阶段 90 s、IPC watchdog 100 s，退距 0.1 m。IK 是单次有限迭代 getPositionIK；请求现有 0.25 s 字段不被该 API 当作独立求解墙钟期限，外层仍受共享截止/IPC 约束。

在已有隔离环境、源码根目录运行（wrapper 先 source ROS，再 exec 本轮 worker）：

```bash
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
  --target carton_l07_c02 --output /tmp/new-c02 \
  --worker-command /tmp/single-contact-20261016/worker.sh \
  --worker-binary /tmp/single-contact-20261016/build/m710_moveit_worker \
  --source-commit 91e6d2caf1a2decb548992a0c38520d08a932379 \
  --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100
# c04: target 改为 carton_l07_c04，使用独立新 output，
# 增加 --initial-state docs/experiments/m710_single_contact_20261016/c04-input.json
```

构建/测试、实际命令、依赖与原始响应均归档于同目录。启动脚本最初在 ROS setup 因 `set -u` 提前退出，未进入模型/规划；保留该环境启动记录，修正 shell 加载顺序后才运行上述两个案例。worker 原有世界目标/附着体查询仍产生 “Attached body not found” 日志，保留原文；请求及 ProcessPolicy 证据均确认目标在世界、未附着，并且求解成功，没有抹去日志。

旧完整取放入口、候选调度及先验相关功能仍保留；**无先验保证仅适用于新入口**。本轮未接视觉，未迁移抽离/搬运/放置，未运行 Isaac、真机、执行包、preflight、五箱/40箱任务或重型资格验收。真实案例仅覆盖正面成功；其他面与失败分支由定向测试覆盖，未宣称全部真实可达。到此停止。
