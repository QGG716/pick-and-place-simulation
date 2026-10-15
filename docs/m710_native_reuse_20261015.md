# 原生检查固定关系复用与热循环去重（2026-10-15）

本轮已实现、Release 构建并完成定向测试及唯一一次 3 组交替 A/B。真实持箱 transit **前三条边**的检查中位数由 **0.185368 s → 0.174714 s（减少 5.75%）**；工艺检查分项由 0.016928 s → 0.005725 s，约 2.96×。FCL 工作区总时间未显著改善；空载检查增加约 0.096 ms（0.88%）。没有新五箱成绩，也不按片段倍数推算五箱耗时。

基线：3a84888f85fd490b538e5ce643aa135c2e8cabdb；实测源码：eaac48743d5c446524ed7d758659f608c94bf244。开发、构建、测试及计时均在新授权 Linux 服务器独立目录 /work-native-reuse2-20261015 完成。Windows 仅收取与发布。旧证据及其他项目未修改。

## 改动与不变量

- ProcessSealInput 在候选边界校验并编译 72 杯中心、半径、mask、接触变换、面和阈值；每状态只做原数值运算，失败或取证时生成 JSON。压缩检查预解析杯体描述及阈值，保持原 SAT 和顺序。
- ClearanceWorkspace 仅复用**工具 box—附着 box 的具名净空性质**。从真实 Link/JointModel 向 flange 遍历，整段必须为固定关节；工具身份还必须属于请求绑定的 tool 集合。首次实际访问执行原 AABB/FCL 查询，仅成功且距阈值超过 1 mm 时保存成功性质。1 mm 仅决定是否重查，不放宽 5 mm 接受阈值；边界、活动关节、mesh 和不能证明的关系继续原查询。
- 上下文由 worker 的已验证模型及私有请求副本控制；工作区持有模型、场景、ACM、附着快照，逐状态仍核对模型、附着 link/shape/局部变换/touch links 及有限变换。不是凭对象名、外部 fingerprint 或裸指针判断可复用。新候选/阶段重建上下文，没有跨 Task 缓存。
- **密封环不缓存 verdict**，压缩 SAT 也不缓存。浮点边界及有序接触继续逐状态算；仅其输入和分配去重。环境/邻箱、工具—活动连杆、支撑分离、抽离再进入、父阶段与实际末态、attach/release、零运动、采样网格、碰撞权限、规划与执行合同均保留。状态复制/变换更新和已有锁未删除。
- 请求诊断开关 native_check_reuse=false 只关闭本轮优化，仍使用此前已优化的 FCL 工作区。默认开启前已通过定向检查。native_process_profile 默认为 false；正式 A/B 的日志、计时级别一致。

## 固定输入与实测

查看优化成绩前，已冻结旧正式单箱选中的 transit :10、pregrasp :1 各 **[0,3) 边**及完整请求。采用原 Clearance 严格插值网格、相同访问顺序和重复端点，分别 **359 / 27 状态**；两段均无有序 tracker 初始历史。完整 40 箱身份中，持箱请求为 39 世界箱 + 1 附着箱，空载为 40 世界箱。归档只作为检查输入，不作为新规划答案。

同机 Xeon Gold 6459C，Ubuntu 22.04 / ROS 2 Humble，Release -O3 -DNDEBUG；固定 CPU 0、单线程环境，串行 **A/B、B/A、A/B**，每次独立进程/候选上下文，遍历开始前没有判定缓存。模型加载单列，候选场景及检查器准备包含在 setup；首次固定关系计算包含在 check。没有 Isaac、控制器或轨迹导出。

| 片段 | reference 准备(s) | optimized 准备(s) | reference 检查(s) | optimized 检查(s) | reference 合计(s) | optimized 合计(s) |
|---|---:|---:|---:|---:|---:|---:|
| 持箱 transit | 0.061288 | 0.060807 | 0.185368 | 0.174714 | 0.246563 | 0.235659 |
| 空载 pregrasp | 0.013645 | 0.013451 | 0.010865 | 0.010961 | 0.024510 | 0.024405 |

均为 3 次中位数；合计先逐次相加再取中位数，不能直接相加表内中位数。inclusive 分项不重复计入合计。各次原数：[CSV](experiments/m710_native_reuse_20261015/timing.csv)、[机器结果](experiments/m710_native_reuse_20261015/formal/comparison.json)；完整逐状态判定保存在同目录 6 份 gzip JSON。汇总 JSON 只去掉与输入完全相同的重复 policy；原始完整汇总另存 comparison.full.json.gz，未改变任何计时或计数。

transit 固定关系首次访问均为 130 对；实际性质计算 **46,670 → 43,637**，复用 **0 → 3,033**。实际 FCL 窄查询仅 **52,964 → 52,505**，不能把 3,033 全部称为 FCL 减少。动态对象对检查均 **2,905,387**，world 窄查询均 **3,231**；密封环几何求值均 **359**，压缩 SAT 均 **25,848**。空载固定复用为 **0**。所有访问状态有效，双方完整判定、首个失败见证及最终 seal mask 完全一致。

单独短剖析：原 transit 的 seal 约 0.009513 s、compression 约 0.007367 s；工作区约 0.166137 s，包含约 295 万次对象对宽阶段与计时/循环开销。当前主要剩余成本仍在工作区遍历；本轮没有继续改造宽阶段、计时器或规划策略。小样本不是 p99 或硬实时结论。

## 验证与复现

已运行 9 条原生定向测试命令，包括 32 个净空边界差分、24 个 seal 边界差分、7 个非法输入、活动连杆/邻箱/环境碰撞、可动工具不复用、附着变化拒绝、mask/阶段/释放、压缩阈值、抽离再进入、零运动和真实 MTC 语义传播；[原生日志](experiments/m710_native_reuse_20261015/native-tests.txt)。另运行 tests/test_planning_only.py 和 tests/test_moveit2_native_cold.py，**67 passed**。不是完整回归或执行资格验收。

实际命令及身份：
[构建](experiments/m710_native_reuse_20261015/build-command.sh)、
[原生测试](experiments/m710_native_reuse_20261015/test-command.sh)、
[基准](experiments/m710_native_reuse_20261015/benchmark-command.sh)、
[源码/worker/benchmark 二进制与输入 SHA256](experiments/m710_native_reuse_20261015/freeze.json)、
[输入冻结范围](experiments/m710_native_reuse_20261015/input-freeze.json)、
[环境和 14 个实际模型 mesh 哈希](experiments/m710_native_reuse_20261015/environment.json)。

model.json.gz 和 fixed-input.json.gz 分别为完整模型参数、固定请求的原字节压缩副本；复现分别执行 gzip -dc model.json.gz > model.json 和 gzip -dc fixed-input.json.gz > fixed-input.json；其 mesh URI 指向既有隔离环境 /work-native-cold-20261002/assets/...。在该 rootfs 内执行所列脚本；换工作目录时调整脚本路径，模型几何及输入不得替换。每个基准子进程硬上限 120 s，总共最多 3 对，不自动重试。

未运行新五箱/40箱/104或129任务、全量 pytest、历史重型验收、Isaac 或真机。规划来源、候选顺序、先验预算及执行入口不变；planning-only 仍是 PLANNING_ONLY_NOT_EXECUTABLE、qualification_status=NOT_EVALUATED、continuous_swept_proof=false。几何复用不代表实际吸附或执行成功。本轮已停止，剩余工作区遍历成本及空载微小退化按实测保留。
