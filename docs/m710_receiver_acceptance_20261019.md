# Hmax 接收区域：完整候选验收后再选择（2026-10-19）

已修复原生 LIN `SUCCESS` 后提前退出区域搜索的问题。c02、c04 各两个预先固定种子的真实原生复核均完成到释放准备，目标仍附着。四次均首样本通过；后置拒绝恢复由明确标注的定向测试覆盖，未在这四次真实运行中触发，不能据此证明成功率统计不退化。

全部结果保持 `LIFTED_RECEIVER_RELEASE_READY_PLANNED_NOT_EXECUTABLE`、`qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`。

## 接受与回退语义

原流程在区域 LIN 成功时写入 `selected=true`，之后父类的返回点历史、终点分离或接收区域检查失败便结束该前缀。现在仅在当前候选依次通过以下检查后写入 `FINAL_SELECTED`：原生 LIN；实际起点及 Task/父阶段/路径绑定；既有返回点 FK 和负载接触历史；实际末态分离与正常自由空间；完整箱底投影、间隔和姿态；最终取消、共享截止与上下文核对。

通过小型 `_plan_extraction_suffix` 钩子复用 `_complete_extraction_candidate` 中的原检查和计划组装。旧 contact-only、through-extraction、through-preplace 保留原流程；没有全局搬移验收职责，也没有新碰撞算法、采样降密或新增求解次数硬上限。

有明确见证的候选路径/终点/接收条件拒绝会继续原区域采样。方法求解超时表示该尝试未完成，不能证明端点或连续区域不可达。取消、任务截止、非法起点、场景/模型/目标/附着/父链不一致、非有限值、未知检查结果等请求级问题立即停止。阶段期限或离散探索耗尽记录为本次搜索未完成，不宣称区域全局无解。

所有候选从同一本次已验收 contact 实际末态及其 receipt 开始，保持 Hmax 场景与实际捕获的固定 attachment。求解与返回点检查分别克隆初始 tracker；候选行、计时和前缀计划独立。原生记录可保留诊断，但下一候选显式回到原 contact 父阶段，不能继承失败候选末态。上下文和路径摘要用于本候选最终核对，不是跨任务缓存。

每个候选原始 `native_result` 与 `sample_check`、`terminal_separation`、`release_ready_check`、拒绝作用范围及最终选择独立记录。原生成功后被项目检查拒绝时，仍保留原生 `SUCCESS`。查看器分别显示“原生生成的候选”“后置验收被拒绝”“本次任务最终选择”。通过的计划直接返回，父类不再重复整段验收；最终方案只有一次规划 ATTACH。

## 实测结果与全部尝试成本

运行前固定顺序 c02/71070、c04/71070、c02/71071、c04/71071；没有根据结果增加试验或挑选最快记录。每次是独立新规划，从配置静止初态、完整 40 箱场景开始，不是上一箱物理反馈，也未输入旧成功轨迹。

| 案例 / seed | 完成 / 区域尝试 | 总规划(s) | 初始化(s) | 区域循环含验收(s) | LIN 候选(s) | 返回点核查(s) | 终点分离(s) |
|---|---|---:|---:|---:|---:|---:|---:|
| c02 / 71070 | 是 / 1 | 2.372825 | 0.798130 | 1.084365 | 0.363574 | 0.611653 | 0.046356 |
| c04 / 71070 | 是 / 1 | 3.021089 | 0.805995 | 1.087044 | 0.367547 | 0.612167 | 0.045546 |
| c02 / 71071 | 是 / 1 | 2.369245 | 0.794273 | 1.077212 | 0.357714 | 0.609626 | 0.045092 |
| c04 / 71071 | 是 / 1 | 2.872567 | 0.787157 | 1.065998 | 0.362208 | 0.597947 | 0.043949 |

四次总规划耗时合计 **10.635726 s**，所有尝试均计入；本批没有候选求解或后置验收失败。接收投影/间隔/姿态末检分别为 0.000212、0.000218、0.000218、0.000224 s。初始化与场景准备单列于 [timing.csv](experiments/m710_receiver_acceptance_20261019/timing.csv)。总规划包含区域循环，区域循环包含 LIN、返回点核查和末检；请求往返又包含原生求解与检查，**这些嵌套列不可相加**。不是机器人运动时间或完整作业周期。

每次真实调用为 IK API 1、OMPL 1、LIN 2、PTP 0、先验 0，另有一次外部升带请求；这些是实际计数，不是业务次数限制或插件内部尝试数。每条抽离返回 209 点，仅进行一次既有返回点核查、一次原生 `check_extraction_end`，最终 ATTACH 数为 1。全部选正面，无换面。实际 contact→extraction q 及真实父阶段一致；`pair_profile_enabled=false`，未测量对象对细项为 null。

本次仍真实拒绝配置 q 下的 Hmax 初态：`tool_compliant_bellows_0 / conveyor_transverse`，`INTERSECTION_OR_CONTACT`，5 mm 规则；相同 q 的低位场景通过。四次均在本次接近实际末态停臂，通过原生纯 Z 扫掠后升带，既有装配接触分离规则和场景身份不变。候选中心 X=−0.5102 m、−X 抽离 0.8102 m；来自原接收区域采样而非固定带中心。实际箱底间隔 c02 约 0.049999996 m、c04 约 0.049999891 m，完整投影与姿态检查通过。目标仍 attached，未释放。

## 回归与证据边界

- 旧基线 a2076a6 上，新增“首个 LIN 成功但分离被拒，第二个候选通过”的回归实际失败：[旧版失败日志](experiments/m710_receiver_acceptance_20261019/baseline-regression.log)。修复后相关 Python 测试 **171 passed / 1.56 s**，涵盖分离/投影/间隔/姿态拒绝、tracker 隔离、父链与状态损坏、取消及迟到成功、未知结果、无重复核查与 ATTACH、原入口兼容。[最终日志](experiments/m710_receiver_acceptance_20261019/tests-final.log)
- 人为后置拒绝均标注 **TEST INJECTION**，只属于定向测试。实际协调函数和原验收函数参与测试；不能把注入失败写成真实机器人失败。原生四次全部首样本通过，因此未真实触发后置失败续搜。
- 已有 `m710_lifted_receiver_test` 原生测试通过，正常 worker 启动通过；本轮 C++ 未改，复用同一已编译 Release 二进制，没有必要重建。原生日志收于原始包。
- 查看器标签 6 项测试通过；两份实际回放各 16 项检查通过，包含实际 q、升带中间状态、负载 FK、唯一身份及最终选择标签。[c02 检查](experiments/m710_receiver_acceptance_20261019/c02-browser-checks.json) / [c04 检查](experiments/m710_receiver_acceptance_20261019/c04-browser-checks.json)。回放固定选 seed 71070，不是挑选最快结果。

## 版本、环境与复现

实测源码 **`5dd484e59cac51fd3203cf52e83802408feec10a`**，基线 `a2076a6ce83ba18884f3d9ca3a52382aa5110bc9`。冻结后没有改规划代码；后续交付提交仅含报告、原始记录、摘要字段与 README。代码提交和报告提交是不同 Git 对象，并非字符编码不同；文本采用 UTF-8。

worker SHA256：`cf6c18ba575b55a011eebf470b169d51d499e3575d3a38410b09d23712f41364`。Ubuntu 22.04 / ROS Humble，MoveIt 2.5.10、MTC 0.1.3、FCL 0.7.0、KDL 1.5.1、OMPL 1.7.0；既有 Release `-O3 -DNDEBUG`。CPU **Xeon Gold 6459C**，亲和性 0–21，OMP/OPENBLAS 均 1；任务/阶段/IPC 为 **300/90/100 s**。上轮为 Platinum 8352V，不能用历史耗时宣称本补修的同机因果加速。模型、配置、二进制与运行身份见 [identity.json](experiments/m710_receiver_acceptance_20261019/identity.json)、[环境记录](experiments/m710_receiver_acceptance_20261019/environment.txt) 和摘要中的 model_identity。

在既有 chroot `/work-single-contact-20261016` 内执行（另一目标及 seed 按上述固定顺序替换，输出目录须新建）：

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT="$PWD"
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python \
  tools/plan_m710_approach_contact.py --through-lifted-release-ready \
  --target carton_l07_c02 --seed 71070 \
  --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100 \
  --worker-command /tmp/lift-native-20261017/worker.sh \
  --worker-binary /tmp/lift-native-20261017/build/m710_moveit_worker \
  --source-commit 5dd484e59cac51fd3203cf52e83802408feec10a \
  --output /tmp/receiver-acceptance-20261019/c02-s71070
```

实际四次命令：[run.sh](experiments/m710_receiver_acceptance_20261019/run.sh)。[机器可读摘要](experiments/m710_receiver_acceptance_20261019/summary.json)、[只读身份/计数核对](experiments/m710_receiver_acceptance_20261019/consistency.json)、[全部必要原始记录](experiments/m710_receiver_acceptance_20261019/raw-records.zip)、[原始文件摘要](experiments/m710_receiver_acceptance_20261019/raw-identities.json)、[实际关节离线回放](experiments/m710_receiver_acceptance_20261019/offline-viewers.zip)。原始包包含四次 result、requests、search-trace、worker/console 日志与原生检查日志；旧证据未覆盖。

未运行 Isaac、真机、释放、执行包、preflight、五箱/40 箱卸货或重型资格验收；40 箱仅作为每例初始障碍场景。未接侧抓、Hmin、纵带衔接或物理接收；机构行程/动态参数、真实吸附承载与连续扫掠资格仍未评估。本轮到释放准备候选接受后停止。
