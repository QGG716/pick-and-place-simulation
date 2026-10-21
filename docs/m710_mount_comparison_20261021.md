**四种安装位置、五个代表箱的完整抓放对照已完成：原位 2/5，仅 −Y 100 mm 为 4/5，仅 −X 100 mm 为 5/5，同时 −X/−Y 各 100 mm 为 5/5。建议优先选仅后移 −X 100 mm。**

仅后移让此前失败的 `l06_c02 / l06_c03 / l06_c00` 全部完成，并保留两个顶层端箱的成功。额外右移没有增加这个样本的成功数，两组总耗时也基本相同。这里是五箱配对样本，每格是独立新规划世界，不是连续卸完 20 箱，也不能把原来的全批 **8/36** 改写成全批 100%。尚未把试验配置提升为默认布局。

| 目标 | 原位 | 仅 −Y 100 mm | 仅 −X 100 mm | 同时 −X/−Y 各 100 mm |
|---|---:|---:|---:|---:|
| l06_c02 | 未完成，142.95 s | 完成，55.28 s | 完成，60.04 s | 完成，57.02 s |
| l06_c03 | 未完成，152.83 s | 未完成，144.58 s | 完成，59.07 s | 完成，58.03 s |
| l06_c00 | 未完成，94.07 s | 完成，15.31 s | 完成，15.35 s | 完成，15.59 s |
| l07_c00 | 完成，102.02 s | 完成，32.38 s | 完成，30.88 s | 完成，32.39 s |
| l07_c04 | 完成，16.68 s | 完成，16.92 s | 完成，12.25 s | 完成，15.18 s |
| 完整成功 / 样本数 | **2/5（40%）** | **4/5（80%）** | **5/5（100%）** | **5/5（100%）** |
| 含失败的五箱任务耗时合计 | 508.55 s | 264.47 s | 177.59 s | 178.21 s |

成功要求整条原生规划、实际 contact 末态吸盘几何与附着、接收高度同步、−X 抽离、完整接收区域验收、同位姿解除附着、名义下落、名义首次触带、工具分离及显式 `IDEAL_RECEIVER_DRAIN` 全部接受。数值 IK、目标检查或局部前缀通过不计完整成功。

世界约定仍为 +X 朝车厢内、+Y 向左、+Z 向上。只改变机器人在底盘上的安装平移，旋转和 Z 不变；底盘、输送带、剪叉、箱堆、车厢墙和工具都保持原几何。

| 布局 | 基座世界原点 XYZ / m | 底座前缘 X / m |
|---|---|---:|
| 原位 | [-1.325, 0.350, 0.600] | -1.100 |
| 仅右移 | [-1.325, 0.250, 0.600] | -1.100 |
| 仅后移 | [-1.425, 0.350, 0.600] | -1.200 |
| 同时移动 | [-1.425, 0.250, 0.600] | -1.200 |

建议配置为 [m710id70_unloading_layout_v1_mount_x_minus_100.yaml](../configs/workcells/m710id70_unloading_layout_v1_mount_x_minus_100.yaml)。另外两种试验配置也保留。布局审计仅允许这三个明确命名、数值精确匹配的变体；原布局检查仍保留。正式 worker 的 URDF `world → base_link` 已逐格核对上述坐标，20 格碰撞策略相同。

每个目标读取其历史派发前的合法根状态，同一目标四组的 q、零关节速度、带高、剩余箱体和历史完成台账完全相同。历史来源及 SHA 在 [paired-inputs.json](experiments/m710_mount_comparison_20261021/paired-inputs.json) 中。旧原生 receipt 清空；每格重新建立 native root，不输入该目标旧成功 q、轨迹或附着关系。20 个初态均由正式 `PinocchioHppFclBackend` 检查通过。各目标初态来自不同历史位置，所以五个成功结果不能直接拼接为同一世界的连续卸货。

固定 seed=71070、每箱 300 s 任务预算、90 s 共享前缀预算、0.60 m 纵带宽度、0.26 m 横带剪叉中心距和上臂软偏好。规划器、关节限位、碰撞规则及接收规则未改变；原生 C++ 与前轮实测源码一致。保留原策略每次候选访问的前缀时间片（原始顶层右端偏心入口 10 s，其他入口 20 s），没有新增 IK、RRT 或固定尝试次数上限。

只有冻结原始顶层右端 `l07_c00` 沿用已有正面 +Y 偏心权限，其他四个目标没有扩权。实际成功参数如下；每行的预测有效杯、命令杯、实际 contact 几何有效杯数量相同，完整 72 位掩码保存在原始记录中。

| 目标 / 成功布局 | 实际 δ / m | 接近时带高 / m | 接收高度 / m | 实际同步位置 | 有效杯 / 72 |
|---|---:|---:|---:|---|---:|
| l06_c02：Y、X、XY | 0 | 0.60 | Hmax 1.75 | PREGRASP | 48 |
| l06_c03：X、XY | 0 | 0.60 | Hmax 1.75 | PREGRASP | 48 |
| l06_c00：Y、X、XY | 0 | 1.45 | Hmin 1.45 | ALREADY_AT_RECEIVE | 48 |
| l07_c00：原位 | +0.331875 | 1.75 | Hmin 1.75 | ALREADY_AT_RECEIVE | 18 |
| l07_c00：Y、X、XY | +0.221250 | 1.75 | Hmin 1.75 | ALREADY_AT_RECEIVE | 30 |
| l07_c04：全部四组 | 0 | 1.75 | Hmax 2.05 | PREGRASP | 48 |

`ALREADY_AT_RECEIVE` 表示必要降带后在实际端态核查时已经位于接收高度。Hmin 右端流程没有因工具偏心改为 Hmax；箱体 Y 保持，抽离至释放维持所需高度。没有 late receiver sync、抽离后持箱升带或释放时工具随箱体下落。采用 `ideal_independent_cups`：所选几何有效杯的吸力足够，全部物理杯和刚性工具仍在场；没有新加承载杯数门槛，也没有验证真空力、剥离或实际接收动力学。

四个未完成格的本次证据如下，全部保留初始提交状态，没有删除失败目标或继续其下方箱体：

| 布局 / 目标 | 最后达到的阶段和本次失败见证 | 已访问与未搜索范围 |
|---|---|---|
| 原位 l06_c02 | 接近、contact 和同步已通过；抽离搜索记录 `J2_link / trailer_left_wall` 净空 **4.359 mm < 5 mm**，最后原生请求为 `INVALID_MOTION_PLAN`；接收区域阶段到期 | 2 个派发候选；抽离区域 150 个端点；另 2 个工艺选项未访问 |
| 原位 l06_c03 | 接近、contact 和同步已通过；最后抽离请求为 `NO_IK_SOLUTION`，接收区域阶段到期；本次没有抽离阶段碰撞见证，不把先前接近阶段的墙体或剪叉见证当作抽离原因 | 2 个派发候选；抽离区域 345 个端点；另 2 个工艺选项未访问 |
| 仅右移 l06_c03 | 接近、contact 和同步已通过；抽离搜索记录 `J3_link / conveyor_longitudinal` 净空 **3.623 mm < 5 mm**，最后原生请求为 `INVALID_MOTION_PLAN`；接收区域阶段到期 | 2 个派发候选；抽离区域 194 个端点；另 2 个工艺选项未访问 |
| 原位 l06_c00 | 部分接近和低带高 contact 通过，但 contact＋Hmin 同步前缀未闭合；记录 `J2_link / conveyor_transverse` 净空 **4.351 mm < 5 mm**，同步请求也有拒绝或超时；共享前缀到期 | 4 个派发候选：1.45 m 已到接收高、1.40 m PREGRASP、1.40 m CONTACT、1.35 m PREGRASP；另 271 个选项未访问 |

前三个失败格均先访问原带高，再访问临时 0.60 m 候选。所有失败仅说明本次预算内未完成，不能称为全局不可达。`MoveItUnavailable` 包装的 sweep 拒绝/阶段超时属于已记录规划请求失败；本次没有未处理的生产接口异常。历史 `joint_axis_frames()` AttributeError 仍属于旧轮次，不归入本次。见证的重复记录数不等于独立候选数；距离字段 −1 是交叠/接触标志，不能解释为 1 m 穿透深度。

本轮共派发 **40 个候选**，完整接受 **16 个**；累计 2,667 个原生请求，其中数值 IK 成功 226 次、显式目标检查通过 65 次、接近运动请求通过 44 次、contact 请求通过 23 次、受检升降请求通过 68 次。这些含弃用候选和局部端态的请求数不能当作成功箱数。每个实际候选的 δ、临时带高、同步位置、q2、J2/J3 结构 FK 上臂仰角、杯几何及碰撞见证保存在 [verification.json](experiments/m710_mount_comparison_20261021/verification.json) 与原始请求中。未派发的选项没有算作已搜索候选。

全实验调度、规划及运行记录写出耗时 **1,178.248 s（19.64 min）**；20 格任务耗时合计 **1,128.816 s**；40 次候选访问耗时合计 **1,028.958 s**。三种口径嵌套，不能相加；环境恢复和后续校验/导出不包含在实验规划耗时中。逐格及逐候选成本见 [costs.json](experiments/m710_mount_comparison_20261021/costs.json)。同一布局没有重复正式搜索以挑选成功。seed 控制既有 OMPL 随机源，KDL 内部随机重启仍受既有可观测性限制，不能保证跨运行逐点相同；时间预算也可能改变完成的数值尝试数。

本轮 24 项定向测试在本地和服务器通过，20 个生产后端初态通过；原生联动 fixture 的 32 个采样和实体包含/身份测试通过。该 fixture 没有 release 样例，不能把它声称为 release 单测覆盖。完整实测记录复核了 **101 个接受阶段、1,667 个联动升降采样和 1,466 个名义下落采样**，父链、末态、箱体保持、附着连续性、同位姿释放及事件顺序均通过。16 个成功三维回放的浏览器检查记录另存于 `browser-checks/`。

新服务器原有运行目录为空，故在 `/root/autodl-tmp/m710-mount-20261021/rootfs` 建立独立 Ubuntu 22.04 环境。ROS 使用已有七项精确版本锁和 `humble/2026-09-14` 历史快照，Python 24 项版本锁沿用仓库已有锁文件，没有升级规划依赖或修改宿主环境。根文件系统是新恢复的，不声称与旧服务器 OS 逐字节相同；四组对照均在这个相同恢复环境重跑。部署时的快照版本/签名键问题、遗漏 STEP 和 pyproject 上传、native fixture 首次缺少命令行参数的失败均单独保留在 [setup-records.tar.gz](experiments/m710_mount_comparison_20261021/setup-records.tar.gz)，修正后才正式运行。没有以这些部署失败冒充规划不可行。

实测源码为 **`cf40c1dda5a3fdda9001053289aca29bb5b88785`**，本轮开始时本地/远端均为 `693e4e2dc8a14c856961584bd81b971438ae47c4`。后续交付只增强只读验证、保留证据和回放，不改变本次规划源码。最终交付 SHA 和实际推送状态在交付答复中给出。

原生 worker SHA256：`0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a`。trace 插件 SHA256：`1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892`。新服务器重编译的二进制哈希与旧服务器不同，C++ 源码相同；没有混用旧二进制身份。

Windows Git archive 导出的输入 JSON 是 CRLF，提交 fixture 是 LF，两者 JSON 值完全相同。正式使用的字节保存在 [runtime-paired-inputs.json](experiments/m710_mount_comparison_20261021/runtime-paired-inputs.json)，其 SHA256 为 `f68f6e8ae9067384de55d2a8b7e82c49273000e86940318620ce0d45b664373f`。源码 manifest、输入差异说明见 [source-check.json](experiments/m710_mount_comparison_20261021/source-check.json)。[实测完整源码包](experiments/m710_mount_comparison_20261021/measured-source.tar.gz) 保留实际导出字节、配置、模型资产和原始 STEP；不要用工作区后续验证脚本替代实测源码身份。

在本次服务器保存的隔离环境中，以下命令重新生成 20 格完整流程；输出目录必须全新。它不读取本轮成功轨迹作为解提示：

```bash
chroot /root/autodl-tmp/m710-mount-20261021/rootfs /bin/bash
cd /work-mount-cf40c1d
export PYTHONPATH=src PYTHONNOUSERSITE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-mount-cf40c1d
/opt/planning-venv/bin/python -u tools/plan_m710_mount_comparison.py \
  --inputs docs/experiments/m710_mount_comparison_20261021/paired-inputs.json \
  --output /mount-reproduction-new \
  --worker-command '/bin/bash /worker.sh' \
  --worker-binary /mount-build/m710_moveit_worker \
  --source-commit cf40c1dda5a3fdda9001053289aca29bb5b88785 \
  --source-manifest source-manifest.json
```

下载 [离线三维回放包](experiments/m710_mount_comparison_20261021/offline-replay.zip)，解压后打开 `index.html`，即可比较四组并播放 16 条完整成功路线。机器人来自每格本次 native q；机构升降使用展示时长，整体视频/回放时长不代表硬件周期。名义下落、理想接收清空与未进行的物理执行均单独标记。失败格提供实际失败摘要，不生成成功动画。

[完整原始记录](experiments/m710_mount_comparison_20261021/formal-raw.tar.xz) 包含全部 20 格输入、尝试、失败、原生请求、trace 和结果；包内逐文件哈希与本地复核结果保留。其由原始 gzip 包无损重压缩，内容未删减。[manifest.json](experiments/m710_mount_comparison_20261021/manifest.json) 绑定交付文件字节。[protocol.json](experiments/m710_mount_comparison_20261021/protocol.json) 和 [summary.json](experiments/m710_mount_comparison_20261021/summary.json) 保存完整运行协议及汇总。

后移缓解了本样本的可达与碰撞问题，但不能解决先前 `l03_c01` 要求 Hmin 0.55 m、`l02_c04` 接收高度低于机构最小 0.60 m 的工艺限制，也没有覆盖其余未探索低层构型。这一轮在完成四布局对照和证据交付后停止，没有扩展为新一轮 36 箱清空或 Isaac 物理试验。

```text
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
```
