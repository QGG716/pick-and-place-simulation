# ECO65 探针尖端 TCP 新标定、RGB-D 与原生循环，2026-10-21

基于工作分支 `feat/v0.6-eco65-moveit2-native-single-carton` 的
`5981f96f5800f5f9b9e55dd092e7e38840d35996` 增量实现。
标定、视觉绑定、原生方向策略和查看器代码提交：
`cafc069cba82c4b16de4541853e7a222f4ce629d`。
机器人、箱垛、两带及最新桅杆安装布局不再调整。

## 标定对象和数据

这里使用用户指定的**此前标定探针尖端 TCP**，不是吸盘 TCP。
已知尖端在 `link_6` 坐标系的位置是
`[0.10212736613382173, 0, 0.22775036000000004] m`。
仅沿用探针及标定板定义；关节观测、板放置、图像和相机外参均重新生成。
这次求解相机外参，不重新辨识探针尖端自身的偏置。

相机跟随 J1，因此求解的是 `T_link1_optical`：
探针实际 FK 触点配准板坐标，再结合新图像的 Charuco PnP 和该图像时刻的
`T_D_link1(q)` 求解相机安装变换。
使用时 `T_D_optical(q) = T_D_link1(q) @ T_link1_optical`，不是固定的世界外参。
板的世界真值、CAD 名义相机外参不进入估计器；真值只进入单独评估。

新观测为 6 个训练触点、3 个留出触点、两个板放置及 6 张实际渲染图像。
每个放置的三张图像具有不同 J1。结果：

| 指标 | 本次结果 |
|---|---:|
| 留出触点最大三维误差 | 0.804182 mm |
| 训练触点最大配准残差 | 0.0000387 mm |
| 训练图像最大重投影 RMS | 0.151904 px |
| 外参对独立仿真真值的平移误差 | 2.091300 mm |
| 外参对独立仿真真值的角度误差 | 0.211347° |

原有阈值未放宽。内参沿用经实际渲染射线核对的仿真投影参数，
没有声称重新标定实物相机内参。触点经过本次原生 IK/FK 及完整静态几何检查，
属于离散仿真观测，没有执行连续探针接触运动或实物力接触。
标定板使用锁定的虚拟夹具；不声称验证了实物支架。

此前误用已知板世界位姿的 `perception_20261021_cycle01` 已中止，标记
`ABORTED_CALIBRATION_METHOD_MISMATCH`，不能用于本任务。
新增门禁拒绝没有 TCP 触点和独立验证绑定的标定记录。

## 视觉与规划绑定

新拍摄的 RGB 和 Z 深度通过估计外参转换到 D 坐标系，依据可见前面和顶面识别
三列顶层箱体；没有读取实例真值掩码或名义相机外参。
150 × 100 × 100 mm、轴对齐、三列堆叠是明确输入先验。
被遮挡的下层箱体按上层测量和尺寸先验补全，不称为独立可见检测。

目标 `carton_r00_l03_c01` 的估计中心为
`[0.20498234559748585, 0.0013598178278424328, 0.5307527832829739] m`。
物理布局快照指纹：
`dd393241e34d092cde91ca24b5a02ce6bb7022419bdb1b5fd92b1fd42ce8bce3`。
估计规划快照指纹：
`87e8664608ed1c6068597aafd4ef8da24bd4ad1866c333b3dcb9b2d19e09b89e`。
规划起点严格继承实际 RGB-D 采集姿态 `[-pi/2,0,0,0,0,0]`，不重新挑选初态。
采集姿态是独立静态仿真设置，没有宣称从之前标定姿态运动到了它。

首次接近单独选用逆时针；附着到释放仍为原始 J1 非增，释放至下一箱接触
仍为非减，整个区间反向数值误差累计上限仍为 `1e-6 rad`。
策略版本 `RAW_J1_FIRST_CCW_OUTBOUND_CW_RETURN_CCW_TOTAL_REVERSE_1E6_V3`
同时进入 Python、原生 worker、请求、结果校验及回放。

## 原生运行与交付

使用既有 `isaacsim-clean` 环境和 Humble chroot，未安装或升级依赖。
原生安装前缀为 `/work-eco65-20261015/install-coupled-20261020`。
`tcp_20261021_build03/build.log` 记录 worker 编译及 25 项方向/合同检查通过。
Python 定向检查通过；本地缺少 USD 的一项测试在现有服务器环境单独通过。

`tcp_20261021_cycle01` 的首次接近顺时针版本未完成。
`tcp_20261021_cycle02` 完成首箱抓放，但回程在右侧箱候选耗尽任务预算，
最终 `complete=false`、墙钟 918.142776 s（900 s 搜索截止，包含收尾和 IPC）。
其首箱采用顶抓 `part_022_seal`、`part_024_seal`，横带从 0.18 m 升至
0.3307527833 m，实际名义下落 0.05 m，退出后才理想清空。
回程改为轮换目标，并保留接近当前工具姿态的杯组候选；
`tcp_20261021_cycle03` 从同一新采集姿态重新原生求解，未输入任何历史轨迹。

**cycle03 完成一箱完整抓放及下一箱实际接触/附着，整体校验 PASS。**
首箱 `carton_r00_l03_c01` 已理想清空；下一箱为 `carton_r00_l03_c00`，
到达本次原生轨迹的真实接触态并附着后结束，没有把第二箱计为已完成。
最终 world 10 箱、attached 1 箱、completed 1 箱。
规划墙钟 792.989484 s；接受轨迹机器人运动时间 157.961892 s；
机构展示时间 2.512546 s；名义下落时间 0.100964 s。
J1 累计反向数值量：首次接近 `2.17343636e-7 rad`、带载去程 `0`、
回程（含下一箱 pregrasp/contact）`2.08302801e-7 rad`，均满足区间总容差。
同一附件保持至释放；外部轴段 q 不变；释放箱体在工具退出前仍处于碰撞世界。

完整结果、逐阶段 q/带高、墙钟/机器人/机构时间及对象统计由交付目录的
`report.json` 记录。离线 `replay.html` 内嵌实际模型、Three.js、七张输入图像、
当前运行的请求/返回和真实搜索数据；`delivery.zip` 同时包含完整证据与
`replay.zip`。失败的 PTP/LIN/IK 不生成装饰性 RRT 树。
前一轮不完整回放独立保留为 `tcp_20261021_delivery02`，不得当作完整结果。
最终完整交付为 `outputs/eco65_native/tcp_20261021_delivery03/`。

所有结果保持 `planning_only=True`、`physical_execution=NOT_RUN`、
`execution_ready=false`。名义下落和 `IDEAL_RECEIVER_DRAIN` 是运动学工艺假设，
尚未模拟真实输送或执行硬件。

实际命令见交付包 `evidence/tcp_20261021_build04/commands_executed.txt`；
使用相同命令重跑时必须选择新的输出目录。

## 三维失败诊断补充

`tools/refresh_eco65_coupled_viewer.py` 可将既有完整回放更新为诊断界面，
先验证源 HTML 的 SHA256，再确认压缩数据与该 HTML 内嵌数据完全一致。
输出的 `.data.json.gz` 与源文件逐字节相同；此次没有重新规划或改变接受轨迹。

cycle03 的 373 个请求中，250 个返回失败：206 次 IK 未求得解、19 次原生规划失败、
16 次原生求解截止、9 次状态检查拒绝。这些候选失败与最终完整任务成功分别显示。
例如界面请求 #22 的首个拒绝样本为 `part_006_rigid` 对 `carton_r00_l03_c00`，
实测间隙 1.516137 mm，要求 2.400 mm。柔性唇的邻箱许可不覆盖刚性内件。

页面提供失败类型筛选、前后请求切换、逐个拒绝样本选择、对象聚焦及完整流程回放。
红色 A、黄色 B 仅高亮记录中的精确碰撞对象，其他视觉几何变灰；即使关闭碰撞图层，
这两个对象仍可见。姿态和场景使用各请求或升降拒绝样本自己的记录。
负距离标记不解释为穿透深度；IK/超时无几何见证时显示种子/请求起点，
不虚构碰撞位置，也不把预算耗尽解释为不可达证明。

本地离线诊断包生成命令（输出路径须为新的路径）：

```powershell
.venv/Scripts/python.exe tools/refresh_eco65_coupled_viewer.py --source outputs/eco65_native/tcp_20261021_delivery03/replay.html --output outputs/eco65_native/tcp_20261021_failure_viewer02/failures.html --failures
.venv/Scripts/python.exe tools/check_eco65_coupled_replay_browser.py --html outputs/eco65_native/tcp_20261021_failure_viewer02/failures.html --browser 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe' --output outputs/eco65_native/tcp_20261021_failure_viewer02/browser01
```

离线 Edge 定向浏览器检查 PASS（3148 项，含已有场景与回放检查），
实际截图为 `failure_viewer02/browser01/checked.png`（上述完整输出路径下）。
最终 world / attached / completed 仍为 10 / 1 / 1。
