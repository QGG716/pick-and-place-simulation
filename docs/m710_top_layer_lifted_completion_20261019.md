# 连续顶层抓放与联动机构（2026-10-19）

**本轮两个预定独立批次均完成顶层连续 5/5。** 完成包含抓取、脱垛、同位姿释放、名义首次触带、必要工具分离和箱间实际末态衔接；依赖声明的简化重力下落和 `IDEAL_RECEIVER_DRAIN`。横带、旋转纵带、剪叉及支座真实接入同源碰撞世界和原生场景。没有物理执行或机器资格结论。

[连续视频：种子 71070](experiments/m710_top_layer_lifted_completion_20261019/seed-71070.mp4) · [离线查看器 71070](experiments/m710_top_layer_lifted_completion_20261019/seed-71070.html) · [离线查看器 71071](experiments/m710_top_layer_lifted_completion_20261019/seed-71071.html) · [离线 ZIP](experiments/m710_top_layer_lifted_completion_20261019/offline-viewers.zip) · [独立核对结果](experiments/m710_top_layer_lifted_completion_20261019/checks/verification.json)。

| 预定种子 | 连续完成 | 释放 / 名义触带 / 分离 / 理想清空 | 批次墙钟 | 原生机器人轨迹时间 | 保留失败分支 |
|---|---:|---|---:|---:|---:|
| 71070 | 5/5 | 各 5 | 345.642 s | 214.720 s | 2 |
| 71071 | 5/5 | 各 5 | 354.417 s | 215.454 s | 2 |

两批均由当前几何和通道状态得到 c02→c03→c04→c01→c00。前三箱正抓直出/Hmax；后两箱正抓 +Y→−X/Hmin。c00 的高位接近和“先降带接近、再升带”的分支失败均保留；最终使用“当前 q 检查降带→低位接近和两段抽离→实际持箱 q 检查升至 Hmin→释放→撤离”。前序已接受箱体没有回滚，没有从 home 重启，也没有拼接独立单箱试验。

保存后的独立验证逐个核对原生 Task/父阶段、每段实际起末 q、场景、attachment、零速度边界、三组 72 位吸盘掩码以及事件顺序。每批 **33 个原生阶段、413 个联动样本、431 个名义下落样本**通过；下方 **35 箱身份及位姿保持不变**。下一箱根状态包含上一箱 final receipt，所有非零机器人运动来自该批当前原生规划。

视频固定使用第一种子，没有挑选较快结果：640×360、5 fps、1205 帧、241.0 s。原生轨迹按正常时间播放；升降演示共 17.2 s，名义下落共约 0.837 s，事件展示共 8.0 s，最终帧按 5 fps 量化。硬件升降时间和完整硬件节拍未知。两个查看器均通过 44 个阶段边界的 q、渲染 FK、箱体数量核对，浏览器错误为 0；视频已进行解码画面检查。

## 实现与建模范围

`tools/plan_m710_top_layer_lifted.py` 从配置初态和完整 40 箱开始，按几何冻结原顶层 5 箱。批次持久保存实际关节 q、零速度边界、横带高度、同源联动世界、剩余箱体、支撑图和交付台账。通道优先级从剩余几何重新计算；未按箱号规定唯一顺序，未预删中间箱，未加入下一层。

新增后支座向后偏置 120 mm，保持与连续轴承体的连接；后转轴位置、纵带刚性长度、横带 XY/尺寸均不变。轴承体与底盘实际间隙为 10 mm，纵向支脚与底盘为 20 mm。水平安装头降低 20 mm，并增加 20×500×50 mm 的连接颈，将水平头与随纵带倾斜的安装头连接。没有缩小机器人、工具或箱体，没有移动基座、底盘或箱堆。显示与碰撞使用同一个 `coupled_conveyor_state()`。

机构内部仅声明具体装配对象对。后支座与整条纵带没有豁免，底盘与所有 `lift_*` 没有通配放行。历史模型 `historical_rejected_v1` 保留，两个 40×80×70 mm 实体相交反例仍被拒绝；新模型标识为 `rear_offset_neck_v2`。

`COUPLED_LIFT` 原生外部轴逐个导入同步高度状态，保持实际 q 不变，并检查官方机器人全部碰撞网格、完整工具、实际附着负载以及机构/环境对象。纵带旋转、剪叉、销、导轨、支座与横带一起更新。最大高度步长 10 mm，失败样本和具体对象对保留。旧纯 Z 单盒检查没有覆盖或替代新机构检查；离散样本不是连续扫掠证明。

正面可直入接收区的箱使用 Hmax；超出横带 −Y 接收范围的箱有正抓 +Y→−X / Hmin 选项，当前 c00、c01 都由几何得到该选项。两段运动沿用第一段实际末 q、同一 attachment 和历史 tracker。转折与释放位置来自完整接收区域。原内侧抓取选项继续存在，但不作为正抓成功的前置条件。

每个目标共享 300 s 任务预算，阶段预算 90 s，IPC 上限 100 s。既有有限候选、数值恢复和检查保留，`pair_profile_enabled=false`。在当前高位接近失败后，可以检查“当前 q 降到低位后接近，再升带”或“低位接近和抽离完成后，持箱停稳升带”。这不是每箱强制回低位；接受前必须完成整个释放与分离后缀。失败分支不修改已提交批次根。

## 释放、接收与计数的含义

原生 `NOMINAL_DROP_RELEASE` 在本次实际末 q 和同一箱体世界位姿解除附着，目标从 attached 转为 world 独立箱体，保持原尺寸及身份。没有 `static_prior_fast`、历史解输入、零长度 place LIN、瞬移或提前删除。

下落采用初线速度/角速度为零、姿态恒定、g=9.81 m/s² 的名义模型。Hmax 面高 2.05 m，落差 0.05 m；Hmin 面高 1.75 m，落差 0.35 m。Hmin 工具停在原抓持高度，不随箱体下落。每个下落位移步长不超过 2 mm；检查机器人、刚性工具、邻箱与机构，首次触带要求完整箱底投影且保留 10 mm 边界余量。

仅释放时已经接触的具名柔性吸盘可以在分离过程中暂时小于 5 mm；不得增加压缩，且恢复 5 mm 后不得重新进入。刚性插入件、刚性工具、机器人均没有下落豁免。箱体落带后仍作为 world 对象参加原生撤离，最终普通净空检查通过后才记录 `SAFE_TOOL_SEPARATION`。

`IDEAL_RECEIVER_DRAIN` 随后将箱体移入台账，保留落带位姿与来源。它是用户允许的理想清空假设，不是横纵带物理交接、实际出料或接收稳定性证明。释放数、名义首次触带数、分离数、理想清空数和最终完成数分别记录。物理接收没有合成观测。

所有结果保持：`qualification_status=NOT_EVALUATED`、`continuous_swept_proof=false`、`physical_execution=NOT_RUN`、`execution_ready=false`。未运行 Isaac、下一层、整车、真实吸附承载、冲击/回弹/稳定、液压或制造资格。

## 失败保留与修复范围

| 运行 | 完成 | 批次墙钟时间 | 保留的主要问题 |
|---|---:|---:|---|
| 开发 dev-01，71070 | 3/5 | 170.959 s | c00 高位接近未完成；c01 第二段丢失 `native_receipt` |
| 开发 dev-02，71070 | 4/5 | 268.288 s | c00 高位正抓/侧抓接近预算耗尽 |
| 开发 dev-03，71070 | 4/5 | 322.413 s | 降带后的空载 IK 错用旧根身份 |
| 开发 dev-04，71071 | 4/5 | 365.453 s | 同一空载父状态问题；多种同步时机失败均保留 |
| 初次冻结运行，71070 | 可完整核对的提交为 4/5 | 见原始日志 | 第五箱候选进入提交时磁盘写满，plan 文件不完整；不计 5/5，第二种子未启动 |

开发记录是迭代探索，运行中源码有调整，不能作为固定版本的性能对照。`native_receipt` 修复仅在返回第一点与实际起点逐值相等时保留父状态；不接受改动后的数值。空载 IK 修复验证实际外部轴父阶段、末 q、接收世界与 target 身份，不回退为配置 home 或重建伪起点。

磁盘中断后对旧大文件逐个 gzip 无损压缩并核对解压 SHA-256，保存 `lossless-compression-manifest.json`。新版本采用紧凑 JSON 和原子检查点写入；失败写入留下 `.partial`，不覆盖上一个完整检查点。正式批次从完整初态重跑，没有用开发成功单箱补齐。

该磁盘中断运行的第五候选完整请求和最终耗时未能落盘，保留原截断文件、零字节请求文件及异常，不补造缺失证据。最终两批的全部候选记录以完整落盘及核对为准。

高位 c00 的局部构型曾被 J2/剪叉、J2/纵带、J2/横带或 J3/横带净空拒绝。这些是具体构型证据；预算耗尽仍表示搜索未完成，不能推断箱体全局不可卸。原始请求、失败返回、搜索轨迹和候选成本见 [原始记录目录](experiments/m710_top_layer_lifted_completion_20261019/raw/)。

## 定向验证

冻结规划核心的 166 项 Python 定向测试通过（42.33 s），覆盖机构旧反例、新几何、双向同步、冻结集合、动态通道、两段原生历史、完整候选回退、释放同位姿、Hmin 下落、提前清空拒绝和箱间连续性。原生测试通过 587 个同步机构/世界状态，以及旧几何拒绝、释放位姿/初速/投影、未知刚性障碍、有限 IK API/预算和原有侧移/纯 Z 兼容性检查。没有运行全量 pytest 或旧 104/129 任务人口。

补充独立只读诊断取第一箱本轮实际空载升降 q 和实际持箱释放 q，分别检查低位↔Hmax、低位↔Hmin、Hmax↔Hmin，以及三个高度保持。共 **1190/1190** 个原生机器人/完整工具/负载状态通过，最大高度步长不超过 10 mm。诊断只调用 init/validate，不向求解器提供历史解、不生成运动、不增加批次完成数，也不授予执行资格。它覆盖这两个明确构型，不能推广到任意 q。

诊断首次尝试因复用原场景指纹但改变世界内容被 `SCENE_FINGERPRINT_REUSED_WITH_DIFFERENT_CONTENT` 正确拒绝，日志保留；随后为每个实际世界计算独立内容指纹重跑。该修复只影响事后诊断工具，正式规划源码和会话没有改变。原子落盘定向检查也确认 NaN 等失败写入不会覆盖已有完整检查点。

[Python 日志](experiments/m710_top_layer_lifted_completion_20261019/checks/python-tests.log) · [原生日志](experiments/m710_top_layer_lifted_completion_20261019/checks/native-tests.log) · [全机器人双向行程汇总](experiments/m710_top_layer_lifted_completion_20261019/checks/interval-summary.json)。

## 源码与复现

正式规划源码：`bf3766514fb22b6cc6613e90e6a0ffd039d26f2a`。原生 worker 编译源码为 `7c760da1839106a0747f082db922c873f3b85226`；两版本 `ros2/` 所有源文件哈希完全一致，复用同一二进制，二进制 SHA-256 记录在 protocol 中。源码清单检查覆盖 src/tools/ros2/configs；部署脚本来自同一提交。查看器修改不参与规划。

| 阶段 | 提交 |
|---|---|
| A：概念机构几何与装配 | `f57e380` |
| B：原生 coupled 与批次状态 | `52a1059` |
| C：正抓两段、释放分离与同步恢复 | `c61ff61`、`7c760da` |
| D：连续回放与台账验证 | `4618844`、`e1b7b9b` |
| 证据落盘修复，正式重跑源码 | `bf37665` |
| 补充只读全机器人行程检查 | `c374be3`、`06d00e4` |
| 离线录屏硬件渲染选项 | `10e7114` |

在独立检出 `bf3766514fb22b6cc6613e90e6a0ffd039d26f2a`、具备既有模型资产的目录内，使用原有 ROS Humble/MoveIt 和规划 venv；不升级依赖。设置 `PYTHONPATH=src`、`PYTHONNOUSERSITE=1`、`OMP_NUM_THREADS=1`、`OPENBLAS_NUM_THREADS=1`、`M710_MOVEIT_ASSET_ROOT=$PWD`，source ROS 环境后：

```bash
python tools/plan_m710_top_layer_lifted.py \
  --source-commit bf3766514fb22b6cc6613e90e6a0ffd039d26f2a \
  --seeds 71070 71071 --worker-command /path/to/worker.sh \
  --worker-binary /path/to/m710_moveit_worker --output /new/output/run
python tools/verify_m710_top_layer_batch.py /new/output/run
python tools/export_m710_top_layer_replay.py /new/output/run \
  --seed 71070 --output /new/output/seed-71070.html
node tools/record_m710_top_layer_replay.mjs /new/output/seed-71070.html \
  /new/output/recording /path/to/chrome --hardware-rendering
ffmpeg -nostdin -framerate 5 -i /new/output/recording/frames/frame-%05d.png \
  -c:v libx264 -pix_fmt yuv420p -movflags +faststart /new/output/seed-71070.mp4
```

记录种子控制 OMPL；现有 KDL 内部随机重启不可直接控制，原生返回如实标注，不能承诺逐位重现或以两个批次证明成功率不退化。墙钟含初始化、搜索、检查及证据写入；机构演示时间不是硬件升降时间，不推算箱/小时。

墙钟来自共享 GPU 服务器上的 CPU 原生规划，期间存在其他任务及本轮只读诊断，不是隔离性能基准。本轮先交付完整检查通过的基线，没有追加性能扫参、下一层或物理试验。

硬件渲染选项属于后续展示工具提交，使用最终分支的录屏器；规划复现仍使用上面的冻结源码。未提供硬件渲染选项时保留软件渲染。两者都逐阶段验证实际 q、渲染 FK 和箱体计数。

原始 tar.gz 内的大 JSON 可能已逐文件 gzip；按压缩清单解压可还原原始字节。归档哈希见 `raw/archive-index.json`。所有历史失败与原先 0/5 报告保留，README 当前结论与历史记录分开。
