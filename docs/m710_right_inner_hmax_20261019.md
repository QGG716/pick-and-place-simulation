# 右端内侧 +Y → −X / Hmax 接收后缀（2026-10-19）

**本轮完成代码、定向测试和固定案例运行，但右端侧抓未到达释放准备。** 两个开放通道案例均停在接近目标检查/IK恢复；两段原生 LIN 尚未被真实案例触发。正面 c02/c04 回归均完成。没有以测试注入、失败构型或正面轨迹冒充侧抓成功。

实测源码：`fff5e93a981bd239ad42840c41d853cbcd9fbb50`；基线：`af8a50cab537f7d67dbe7d2f4349ad9b7eff66aa`。运行后只补报告与查看器显示（目标 q 查看、方向文字及失败状态），没有修改规划源码。交付提交与实测源码提交分开记录；两者文件编码均为 UTF-8。

## 实现与边界

新增显式 `--receiver-template right_inner`，只用于 through-lifted-release-ready。普通正面优先、contact/extraction/preplace 入口和先验禁用不变。右端由原始层几何与 Y 排序确定，实际面为 `y+`、外法向 +Y、工具工作轴 −Y；不靠 c00 字符串选角色。侧面暴露筛查从无限远射线改为该显式模板下完整工具外包络加既有退距的有限范围（本例 0.317500003 m），远端障碍仍完整进入原生场景与检查。

复用原 Hmax 升带及完整区域候选验收循环。区域由完整箱底投影得出；转折 Y 和最终 X 以既有 FK 位置容差作分辨率、公平二维细分。完整工具保守筛查仅决定尝试区域，原始区域保留并交替探索；不固定展示中点，不增加小次数上限。第一段从本次实际 contact/attachment 发出 +Y Pilz LIN；第二段从第一段实际末 q/receipt 发出 −X LIN，保持空间姿态。没有转折点 IK、第二次 ATTACH 或任意绕行替代。

新增仅此模板使用的具名同平面支撑历史：允许合法支撑交接，禁止穿入支撑、非支撑侧碰及已分离对象再进入。原生内部检查与返回点顺序核查、Python tracker 使用同一终端历史，第二段要求原生父记录的历史完全一致；最终仍恢复正常自由空间、分离、接收投影/间隔/姿态检查。没有扩大 ACM。第一段不套用最终全分离判据。

只有完整两段及后置验收、最终身份/截止检查通过才 FINAL_SELECTED。相同 contact 父状态、attachment、场景/策略和 Y 下，可复用本候选作用域内已验收第一段；第二段失败不能污染下次历史。原有单段父链断言保留；多段必须显式提供两段身份与已验收前缀，原始点/速度不重写。

## 固定原生案例结果

运行前固定顺序与种子见 [protocol.json](experiments/m710_right_inner_hmax_20261019/protocol.json)。每案新进程，从配置初始 q 重新规划；task 300 s / stage 90 s / IPC 100 s。开放通道明确预先移除顶层 c01/c02/c03，保留 c04 与所有下层箱，共37箱；这是输入前提，新增卸箱数为0。其余案例完整40箱。这是五个独立单目标案例，**不是连续五箱卸货**。

| 案例 | seed | 释放准备 | 总规划(s) | 初始化(s) | 场景准备(s) | IK请求/API | OMPL / LIN / PTP |
|---|---:|---|---:|---:|---:|---:|---:|
| 右端开放通道 c00 | 71070 | 未完成 | 90.062953 | 0.922184 | 0.004545 | 477 / 477 | 0 / 0 / 0 |
| 右端开放通道 c00 | 71071 | 未完成 | 90.074096 | 0.922165 | 0.004641 | 469 / 469 | 0 / 0 / 0 |
| 右端未开放通道 c00 | 71070 | 模板被阻挡 | 0.038410 | 1.264304 | 0.004642 | 0 / 0 | 0 / 0 / 0 |
| 中间正面 c02 | 71070 | 完成，仍attached | 2.653265 | 0.943984 | 0.004643 | 1 / 1 | 1 / 2 / 0 |
| 左端正面 c04 | 71070 | 完成，仍attached | 3.316967 | 0.938314 | 0.004668 | 1 / 1 | 1 / 2 / 0 |

两次右端各16次原生运动请求均先被 `INVALID_GOAL_CLEARANCE` 拒绝，首个对象对全部为 **J4_link / carton_l06_c02**，见证 `INTERSECTION_OR_CONTACT`。FCL 的 −1 返回是相交标记，不能解释为穿透1米。IK数值候选合法不等于目标碰撞合法；没有进入 RRT、contact、升带、附着、区域生成或两段抽离。最终 `SINGLE_TARGET_STAGE_BUDGET_EXHAUSTED`，末次原生返回 `NATIVE_IK_REQUEST_DEADLINE`；不能解释为连续区域无解或箱体不可卸。

两例接近目标均为本次上游中心点/朝向：物理接触 `[0.30,-0.64,2.25] m`、接近 `[0.30,-0.54,2.25] m`；没有换 roll/面内点、删下层箱、扩大预算或追加 seed。各16个数值候选不等于16个独立物理构型（含关节等价绕转）。KDL内部恢复次数不可观察，记null；seed实际控制OMPL，未宣称KDL逐位确定性。

未开放通道反例由仍在场的 c01 遮挡 `y+` 面，0个IK/运动调用，未删除障碍；只是当前显式模板不可用。

所有已进行的初态检查均是 LOW 通过、同 q 的 HMAX 因 `tool_compliant_bellows_0 / conveyor_transverse` 相交被拒绝。两例正面在本次实际 pregrasp 末态同步升带到2.05 m，通过纯Z几何扫掠及父链更新。各1个区域候选完整通过、1次规划ATTACH；原生起末 q 与父链完全连续。实际箱中心分别约 `[-0.510199994,0.000000022,2.250000000]`、`[-0.510199997,0.839999980,2.249999980] m`，实际间隔0.049999996 / 0.049999891 m，完整投影在接收区内。右端没有第一/第二段末态与历史，必须记 NOT_REACHED，不补造。

| 已有计时区间(s) | 右端71070 | 右端71071 | 正面c02 | 正面c04 |
|---|---:|---:|---:|---:|
| 上游选面/目标生成 | 0.039455 | 0.047808 | 0.025639 | 0.010961 |
| 显式IK处理（包含请求往返） | 60.991735 | 60.274021 | 0.030589 | 0.046566 |
| 接近连接处理累计 | 1.439784 | 1.490409 | 0.782854 | 1.426531 |
| 升带几何请求往返 | 未到达 | 未到达 | 0.090275 | 0.091434 |
| contact处理 | 未到达 | 未到达 | 0.142581 | 0.145753 |
| attachment上下文 | 未到达 | 未到达 | 0.004924 | 0.003789 |
| 抽离生成处理 | 未到达 | 未到达 | 0.394156 | 0.403599 |
| 返回点FK/历史 | 未到达 | 未到达 | 0.679073 | 0.700154 |
| 末态分离/正常净空 | 未到达 | 未到达 | 0.060908 | 0.044438 |
| 接收投影/间隔/姿态 | 未到达 | 未到达 | 0.000237 | 0.000253 |

上表以机器记录为准；初始化/场景准备单列。右端原生IK API累计46.735418 / 46.012688 s，是IK处理时间的子区间；请求构造13.047222 / 14.155719 s是既有边界计数，不能与父区间重复相加。总规划包含全部失败恢复与其他组织成本；未归因部分不叫IPC或Python耗时。前缀/区域总计同样包含子阶段，不把列相加当总时间。正面机械臂原生时间律约31.390 / 31.214 s，与规划计算耗时不同；升带无硬件速度、加速度参数，不报告完整作业周期。

[request-costs.json](experiments/m710_right_inner_hmax_20261019/request-costs.json) 按本运行唯一 response.request_id 汇总当前请求证据，未累加父链快照。右端各35次Clearance状态检查、162201次AABB、2940次窄查询，累计0.008828 / 0.008537 s；此结果不支持把本次90秒阻塞归因于碰撞遍历。正面 c02/c04 分别990/1852次状态检查、15471/26231次窄查询；固定AABB与窄阶段资格各134/126，复用26334/836。所有实际检查响应保留优化开关，`pair_profile_enabled=false`，逐对耗时null。先验、PTP、历史生成器调用均0。未到达的侧移专用检查没有资格通过结论。

## 验证、回放与复现

190项受影响Python测试通过，2项原生测试通过，8项查看器标签检查通过。新增协调测试明确标为 TEST INJECTION：合法但未全分离的第一段、跨段父链/历史、第二段后置拒绝继续区域搜索、第一段复用、独立tracker、一次ATTACH、取消/过期/身份错误停止。原生侧移测试使用真实FCL验证同平面支持、穿透、非平面障碍与分离后再进入；它不是机器人完整两段规划成功。真实案例没有触发侧抓两段与后置恢复分支，小样本不能证明成功率不退化。

[下载离线查看器](experiments/m710_right_inner_hmax_20261019/offline-viewers.zip)：两份右端查看器包含各16个真实拒绝目标，点击“查看请求目标 q（非轨迹）”突出J4与下层箱，**没有合法运动路径或两段抽离动画**；两份正面查看器可切换实际接近、升带、contact及抽离原生点。4份HTML浏览器加载检查通过，无脚本异常。未记录搜索树，不伪造树。示意截图：[右端被拒目标](experiments/m710_right_inner_hmax_20261019/right-rejected-goal.png)、[正面实际释放准备规划末态](experiments/m710_right_inner_hmax_20261019/front-c02-release-ready.png)。

[原始结果ZIP](experiments/m710_right_inner_hmax_20261019/raw-results.zip) 保留5案 result/requests/search-trace/worker日志；[汇总](experiments/m710_right_inner_hmax_20261019/summary.json)、[逐案CSV](experiments/m710_right_inner_hmax_20261019/timing.csv)、[环境](experiments/m710_right_inner_hmax_20261019/environment.txt)、[实际批次命令](experiments/m710_right_inner_hmax_20261019/run.sh)。Linux既有隔离环境Ubuntu22.04.5/Humble，MoveIt2.5.10、MTC0.1.3、FCL0.7.0、KDL1.5.1、OMPL1.7.0；Xeon Gold6459C，CPU0–21，OMP/OPENBLAS各1线程，Release -O3 -DNDEBUG。依赖未升级。官方模型/工具/配置摘要在每案result与汇总中；worker SHA256 `0c5c94fef7f01a460c1a96f15c9be6cc2960b1da44171c1b01b0de19a6771472`。

在原隔离环境 `/work-single-contact-20261016` 复现单案（输出必须换新目录；本轮不再运行）：

```sh
source /opt/ros/humble/setup.bash
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=$PWD
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
  --through-lifted-release-ready --receiver-template right_inner --open-middle-channel \
  --target carton_l07_c00 --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100 \
  --worker-command /tmp/right-inner-20261019/worker.sh \
  --worker-binary /tmp/right-inner-20261019/build/m710_moveit_worker \
  --source-commit fff5e93a981bd239ad42840c41d853cbcd9fbb50 --output /tmp/NEW_OUTPUT
```

本轮停止于保留的接近阻塞：**右端新后缀已实现但完整原生贯通未验证**。没有更改上游目标来绕过下层箱。仍保留 `LIFTED_RECEIVER_RELEASE_READY_PLANNED_NOT_EXECUTABLE` 分类标记，但是否完成必须读取 `complete`；全部 `qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`。未接左端Hmin、真实释放、载箱降带、横纵对接、右侧出料、冲击/稳定/承载；未运行Isaac、真机、五箱/40箱任务或重型执行资格流程。
