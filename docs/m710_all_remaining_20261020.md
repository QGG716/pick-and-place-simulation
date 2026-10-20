**未能全部卸完。本轮完整抓放 8/36，完成率 22.2%；36→28 箱。**

从已接受的 c00 末态继续，历史 4 箱单列。原始 40 箱台账累计为 12/40（30.0%），其中只有新增 8 箱属于本次实测。这里统计的是完整原生规划及名义接收、理想清空，不是物理执行成功率。

| 本轮 36 箱的去向 | 数量 |
|---|---:|
| 完整抓放、名义首次触带、工具分离及 IDEAL_RECEIVER_DRAIN 全部接受 | 8 |
| 实际进入原生规划，预算内未完成 | 3 |
| 既定接收高度超出现有机构范围，未派发原生规划 | 2 |
| 仍支撑上方保留箱体，未派发规划 | 23 |

实际进入原生规划的 11 箱中成功 8 箱，条件完成率为 72.7%。按全部进入工艺调度的 13 箱计算为 61.5%。主指标保留 **8/36**，不排除受阻箱来抬高整批完成率。这是单一场景的一次续跑观测，不是多次试验估计的普遍成功概率。

本轮沿用 seed=71070、每箱 300 s 任务预算、90 s 共享前缀预算、0.60 m 纵带和 0.26 m 横带剪叉中心距。保留上臂软偏好、原碰撞规则、Hmin/Hmax 几何分类和实际父链。没有新增偏心权限：原始顶层 c00 已在历史台账中，以下新增流程的 delta 均为 0。右端/左端身份由原始各层几何冻结，不能随剩余箱体重新授权。

| 箱体 | 本轮结果 | 接收高度 / 同步 | 原生候选派发数 | 任务耗时 s |
|---|---|---|---:|---:|
| carton_l07_c04 | 完整成功 | Hmax 2.05 / PREGRASP | 1 | 12.28 |
| carton_l06_c02 | 抽离接收区域搜索未完成 | Hmax 1.75；尝试原带高及临时 0.60 | 2 | 142.84 |
| carton_l06_c03 | 抽离接收区域搜索未完成 | Hmax 1.75；尝试原带高及临时 0.60 | 2 | 150.91 |
| carton_l06_c01 | 完整成功 | Hmin 1.45 / ALREADY_AT_RECEIVE | 1 | 20.50 |
| carton_l06_c04 | 完整成功 | Hmax 1.75 / PREGRASP | 1 | 14.42 |
| carton_l06_c00 | 接触/升回接收高度同步未闭合 | Hmin 1.45；临时 1.45、1.40、1.35 | 4 | 93.19 |
| carton_l05_c01 | 完整成功 | Hmin 1.15 / ALREADY_AT_RECEIVE | 1 | 25.09 |
| carton_l05_c04 | 完整成功 | Hmax 1.45 / PREGRASP | 1 | 10.55 |
| carton_l04_c01 | 完整成功 | Hmin 0.85 / ALREADY_AT_RECEIVE | 1 | 17.47 |
| carton_l04_c04 | 完整成功 | Hmax 1.15 / PREGRASP | 1 | 10.01 |
| carton_l03_c01 | 工艺高度越界 | 要求 Hmin 0.55，机构最低 0.60 | 0 | 0.14 |
| carton_l03_c04 | 完整成功 | Hmax 0.85 / PREGRASP，成功候选先降至 0.60 | 2 | 35.56 |
| carton_l02_c04 | 工艺高度越界 | Hmax 0.55、Hmin 0.25 均低于 0.60 | 0 | 0.13 |

ALREADY_AT_RECEIVE 表示必要降带后，在实际 pregrasp/contact 末态核查时已经处于所需接收高度，并非跳过核查。全部 8 条成功流程的预测有效杯、命令杯及实际接触几何有效杯均为 **48/72**，全体 72 个物理杯和工具实体仍保留。沿用 `ideal_independent_cups` 的理想承载假设，未增加杯数承载门槛或声称已验证真空力。

3 箱规划未完成的具体证据如下，均来自本轮：

- **l06_c02**：完成接近和 contact，同步后在接收区域尝试了 152 个抽离端点。原生抽离检查记录了 `J2_link / trailer_left_wall` 的 5 mm 净空拒绝；结束原因为接收区域阶段预算耗尽。另有 2 个工艺/高度选项未访问，不能称为全局不可达。
- **l06_c03**：完成接近和 contact，在接收区域尝试了 320 个端点。最终原生抽离请求返回 `NO_IK_SOLUTION`，接收区域阶段预算耗尽；没有把接近阶段的墙体见证冒充最终抽离碰撞。另有 2 个工艺/高度选项未访问。
- **l06_c00**：派发了 4 个中心目标/高度/同步组合：1.45 已到接收高度、1.40 PREGRASP、1.40 CONTACT、1.35 PREGRASP。局部接近和部分低带高 contact 可通过，但未形成合格的 contact＋Hmin 同步前缀；本轮记录含 `J2_link / conveyor_transverse` 的 contact 净空拒绝。共享前缀到期，剩余 271 个已定义工艺/高度/同步选项未访问。未把中途成功的 contact 当作完整成功，也未重置预算重跑。

失败箱保留原位，不修改其下方箱体的支撑关系。23 个受阻箱的直接支撑依赖已重新从最终几何核验，完整逐箱清单见 [all-targets.csv](experiments/m710_all_remaining_20261020/all-targets.csv)。低层未到达的构型不记为 IK 失败。未改尺寸、规划器、初态、seed、碰撞权限或接收高度来追求更高完成率。

本轮派发 **17 个候选**，其中 8 个完整接受、9 个未接受；共 1,178 个原生请求。保存的请求中有 113 次数值 IK 成功、38 次显式端点检查通过、27 次接近运动请求成功、14 次 contact 请求成功。这些是包含丢弃候选的**请求次数**，不等于箱体成功数。完整验收仍只有 8 箱。每个已派发候选的 delta、临时高度、同步位置、q2、结构 FK 上臂仰角、杯数和碰撞见证保存在 [verification.json](experiments/m710_all_remaining_20261020/verification.json) 及原始请求中。

全批调度与归档耗时 **558.885 s（9.31 min）**；各箱任务耗时合计 **533.097 s**；候选访问耗时合计 **482.791 s**。三者是嵌套口径，不能相加。预算限制作用于规划，共享前缀之后进入合法后缀的任务可超过 90 s；记录、序列化和关闭 worker 的成本也保留。详见 [costs.json](experiments/m710_all_remaining_20261020/costs.json)。未进行第二次正式搜索或更换 seed；后端自身声明该 seed 控制 OMPL，KDL 内部随机数不由它控制，故不承诺跨运行逐点相同。

定向回归 24 项通过；回放缓存专项回归 1 项通过。保存的原生链路核验通过 **49 个接受阶段、395 个联动升降采样检查、658 个名义下落采样检查**。全部 attachment 在实际 contact 后捕获，末态姿态连续，释放同位姿，箱体下落期间工具 q 保持不变；没有 late sync 或抽离后持箱升带。

生产规划没有集成异常。本轮开发检查曾因同步目录漏传既有 `scripts/expand_official_m710id70_urdf.py` 而失败，补传后通过，模型和依赖未变。回放首次导出曾因小数缓存键把两个相邻浮点带高合并而触发严格几何断言；最大差异为 4.44e-16 m，按实际高度重新生成与原生场景完全相同。已改为精确浮点缓存键并复验，未放宽比较；失败及修复单列在原始包的 `checks/`。历史 c00 的 `joint_axis_frames` AttributeError 仍属于上一轮历史记录，本轮没有重标为同一失败。

实测规划源码：`66c34d4e4464589e6ea865125ccf4e71854d5be8`。验证/回放交付代码：`2ccaf4e6f34117c02b7d5c5596c0803261ad0b8d`。本轮起点为 `a41f86a279ecf85aa192ef3cd2288ad6b4753e4a`，分支保持 `feat/v0.5-moveit2-native-single-carton`。所有规划核心、原生代码、配置及规划入口在实测与交付代码间逐文件哈希相同；最终证据提交 SHA 与推送状态在交付答复给出。

原生 worker SHA256：`5aaff6025df88c7c931e25e5bc2d2d7ff11e22a102c19c8dc2c3b5d6a92b0f29`。完整原始包 SHA256：`406ae27eaa7b5f7b93bb18c0838ff393a45acdcd8022c0229855b7535f335c1c`。

在授权服务器既有隔离环境中的复现命令如下，输出目录必须全新；该命令会重新搜索，不读取本次成功 q 或轨迹作为解提示：

```bash
chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash
cd /work-all-remaining-formal-v1-20261020
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-seam-approach-formal-v2-20261020
/opt/planning-opt-20261003-venv/bin/python tools/plan_m710_all_remaining.py \
  --archive /tmp/seam-approach-20261020/archives/v2-on.tar.gz \
  --c00-result /tmp/c00-resume-20261020/formal-v1/seed-71070/result.json \
  --output /tmp/all-remaining-20261020/reproduction-new \
  --worker-command /tmp/eccentric-20261020/worker.sh \
  --worker-binary /tmp/eccentric-20261020/formal-build/m710_moveit_worker \
  --source-commit 66c34d4e4464589e6ea865125ccf4e71854d5be8 \
  --source-manifest /work-all-remaining-formal-v1-20261020/source-manifest.json
```

下载 [离线三维回放包](experiments/m710_all_remaining_20261020/offline-replay.zip) 后双击 `accepted-cycles.html`，可看本轮 8 条连续接受流程。也可直接打开 [HTML](experiments/m710_all_remaining_20261020/accepted-cycles.html)。浏览器逐段验证本轮原生 q、FK、箱数及下落时固定工具，未拼入丢弃候选。原生机器人运动约 408.23 s；机构升降每次 4 s 只是展示时间；总回放 457.74 s 不是硬件实测周期。

[完整原始记录](experiments/m710_all_remaining_20261020/formal-raw.tar.gz) 包含 13 个工艺任务的输入、失败、成功计划、请求、trace、日志、逐文件归档校验，以及开发/导出失败记录。历史前序的档案哈希及读取用途另见 [predecessor.json](experiments/m710_all_remaining_20261020/predecessor.json)，所有交付文件校验见 [manifest.json](experiments/m710_all_remaining_20261020/manifest.json)。

```text
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
```
