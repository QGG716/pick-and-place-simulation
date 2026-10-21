# ECO65 底座和桅杆安装改动，2026-10-21

在 `feat/v0.6-eco65-moveit2-native-single-carton` 的
`7c178ffee56ca6a9bc4e48d5cc9014012ae07bd5` 上增量修改。
本轮只改变安装方向，并用同一新场景重新运行原生连续任务。
参考分支、官方机器人关节定义、原始关节角和有限关节范围均未修改。
**两项安装修改已完成；完整“一箱抓放＋下一箱抓取”仍未跑通。**
本轮保存了真实抓取、附着、带载升降和抽离前缀，没有伪造后续转运或释放。

## 安装关系

- 官方 `base_link.STL` 的开关/接口面法向是 base +X。
  固定安装 `base_rpy_rad=[0,0,π]`，使该面朝场景 −X；底座位置仍为
  `[-0.34,0,0.24] m`。
- 桅杆安装底座绕 J1 的 Z 轴，在相同原始 J1 下相对旧场景逆时针旋转 90°。
  因机械臂固定底座同时转了 180°，`T_link1_mast` 的局部旋转变化为 −90°。
  这是安装变换，不通过修改 q1 或加减 2π 实现。
- 为保持相同原始 J1 下的相机世界朝向，圆杆及其一体上端安装槽
  `mast_part_019`、相机及俯仰转接件绕真实杆轴反转 90°；下端圆形夹持关系保留。
  相机下俯仍为 30°，CAD→optical 关系不变，世界朝向矩阵最大误差 0。
  相机随 J1 的原有运动关系保留，没有新增世界锁定轴。
- 桅杆底部贴合座按官方 link_1 表面和新方向的原始支架重新生成，保持安装高度；
  替换旧的 251 个凸体，采用 180 个新的局部保守凸体。
  没有缩放原厂零件，也没有保留参与碰撞的旧贴合座。
- 相机名义外参和安装指纹已更新；旧安装外参被拒绝。输入仍来自冻结场景，
  未进行实机标定或视觉识别闭环。

配置入口是 `configs/workcells/eco65_base_mast_remount_v1.yaml`，统一实现是
`src/unloading_sim/eco65/layout_installation.py`。Python FK、原生 URDF、附件碰撞、
三维回放及 USD 使用同一安装变换。原始 12 箱身份/尺寸、12 杯几何、
`robot_mount_plate`、输送机尺寸和联动关系保留。

有效碰撞策略仍为 `eco65_pair_collision_policy_v4`。吸盘与具名邻箱的许可、
脱垛阶段负载与邻箱的许可，以及机械臂本体与桅杆总成的精确对象对豁免保留；
仅随新贴合座清单更新其相应对象对。工具/负载与桅杆、机器人与输送机等检查保留。

## 本次原生运行

新场景：`outputs/eco65_native/layout_remount_20261021_scene01`。
快照指纹：
`2a24cc4229d042020af6e6f90c0406c87f84c4ae733e3cc785755008ea6a9ef4`。

在用户指定服务器的既有隔离环境构建、安装并启动仓库原生 worker。
每轮均为新生成的 ECO65 原生试算，没有加载历史成功轨迹。
预算保持 task 900 s / connection 100 s / solver 8 s / RPC 100 s，8 个 IK seed、
最多 24 个抓取候选。未增加总预算或放宽碰撞规则。

| 运行 | 规划墙钟 | 终止结果 |
|---|---:|---|
| `layout_remount_20261021_run01` | 763.507076 s | 状态检查候选期限到达，被误当作任务终止 |
| `layout_remount_20261021_run02` | 431.032034 s | 尝试恢复后，客户端已关闭 worker，报 Broken pipe |
| `layout_remount_20261021_run03` | 902.209189 s | 总任务预算耗尽，`INCOMPLETE / NATIVE_TASK_BUDGET_EXHAUSTED` |

三次原生运行规划墙钟合计 `2096.748299 s`。第三轮共 998 个原生请求，
包含 443 个 IK 失败、92 个 IK 成功、14 个状态拒绝、338 个运动规划失败。
各候选的成功结果不拼成完整任务；最终退出发生在尝试另一首抓预抓取分支时。
没有再次出现 Broken pipe。原生请求及 IPC 往返耗时按业务区间保存在 `summary.json`；
902.209189 s 总墙钟也包含 Python 场景/候选准备与归档收尾。

修复范围限于完整且 request_id 匹配的 `check_state_offline` 超时响应：
仍拒绝该候选，保留响应和耗时，但允许同一 resident worker 处理下一请求。
实际 IPC 超时、取消、错配及其他错误仍关闭连接；不重置任务绝对截止时间。
接收高度候选按距当前真实带高的距离排序，先尝试 0.33 m，再保留 0.43 m。

第三轮保存了一条含真实升降的连续前缀
`first_carton_extracted_lifted_candidate.json`，原生 task ID 为
`eco65-3ef6452016034843a05ba747af11f69f-initial-0-0`。
它来自同一次运行、同一初态试算，后续候选回退不会将其与其他路径拼接。

| 已接受前缀阶段 | 原生记录 | 结果 | 机器人运动时间 |
|---|---|---|---:|
| 首箱预抓取 | pregrasp-27 | SUCCESS | 19.395173 s |
| 实际接触 | contact-28 | SUCCESS | 5.847236 s |
| 实际附着 | attach-29 | SUCCESS | 事件 |
| 固定 q 带载升降 0.18→0.33 m | coupled_lift-150 | SUCCESS | 机构展示 2.5 s |
| 带载抽离 | extraction-155 | SUCCESS | 19.123081 s |

目标 `carton_r00_l03_c01`，front 抓取，启用 `part_002_seal` 和
`part_004_seal`。实际 FK 密封通过；工具板边最大角误差
`2.98023224e-8 rad`。初态为本次原生 stand-off IK 得到并检查的新仿真初态，
没有隐藏的初态迁移轨迹。

```text
initial = [-2.137542496, 1.315695154,-1.244430395, 3.070342615, 2.574842830,-1.568650696]
grasp   = [-2.812068806, 1.263325254,-0.923952292, 2.802222406, 1.900316502,-1.568661147]
extract = [-2.956152946, 0.817176829,-0.559666575, 2.884083833, 1.756232361,-1.568662007]
```

该前缀首次接近累计顺时针 `0.674526310 rad`，附着后抽离顺时针
`0.144084141 rad`，累计反向量均为 0；尚未产生回程，不能报告回程方向通过。
升降时 q 与附件不变，联动世界随 0.18→0.33 m 采样更新。
机器人运动时间合计 `44.365491 s`，机构展示 `2.5 s`；查看器的附着事件停留
另列，不能计作机器人运动或规划墙钟。
该前缀抽离后的 J1 距官方下限 −3.106 rad 仅 `0.149847054 rad`（约 8.586°）。
这是这条前缀的剩余顺时针行程，不是其他抓取分支全局不可行的证明。

所展示前缀末态为 world 11 箱、attached 1 箱、completed 0 箱，
其余箱体身份和尺寸保持。`next_target_id=null`；没有第二箱完成记录。
完整运行的失败回退末态另保存在 `partial_plan.json` 和 summary 中，
不把展示前缀当作全局最终采用计划。

验收边界：前缀的原生父状态、q、附件和机构状态连续，方向检查及固定 q 升降成立；
完整首箱转运/释放/接收/退出/清空、释放后箱体留在碰撞世界中的退出、
回程方向及下一箱实际接触均未到达，不能标为通过。
当前证据仅说明本轮有限候选/预算没有找到完整解，不构成全局不可行证明。

第三轮带载终态检查的具体证据（原始归档从 0 开始的 call_index）：

| 调用 | 带高 | 状态检查碰撞对 | 原生结果 |
|---|---:|---|---|
| 51 | 0.18 m | link_3 / part_026 | 相交或接触 |
| 56 | 0.18 m | link_2 / eco65_transverse_relief_frame | 相交或接触 |
| 187 | 0.33 m | link_2 / conveyor_transverse__mesh002_component000 | 相交或接触 |
| 192 | 0.33 m | link_3 / eco65_transverse_relief_frame | 相交或接触 |
| 526 | 0.33 m | link_3 / part_039 | 间隙 0.001112694 m，小于要求 0.0024 m |

完整起始 q、对象对和失败世界保存在 `diagnostics.json` 及原始请求/返回中。
另有预抓取候选被刚性吸盘插芯 `part_002_rigid` 与目标箱间隙拒绝；
吸盘邻箱接触许可没有扩大到刚性插芯、目标或机器人。
第三轮运动请求均为 Pilz（PTP 7 成功/2 失败，LIN 11 成功/336 失败），
没有满足终态筛选后进入 OMPL 的请求，实际搜索 trace 数为 0。
查看器不会为这些 PTP/LIN/IK 请求补造 RRT 树。

最终离线浏览器 2931 项检查通过，FK 最大矩阵误差 `2.22e-16`；
明确显示 `PARTIAL_ONLY`、world 11 / attached 1 / completed 0。
页面展示 5 段连续记录和 993 个请求（不含 init 与 4 次纯 FK 调用）。
最终 HTML SHA256：
`352caab6e3a0362abe87e3e73f610fe9dba418d377c865efedc8d0e346cbc04f`。
机器人 44.365491 s、机构 2.5 s、附着展示停留 0.35 s、名义下落 0 s，
与完整运行规划墙钟 902.209189 s 分别显示。

## 三维交付及检查

生成目录：`outputs/eco65_native/layout_remount_20261021_result/`。

- `layout_comparison.png`：相同原始 J1=0 下的旧/新安装俯视图。
- `layout_preview.html` / `.zip`：新安装及三种机构高度预览；预览不是执行升降证据。
- `run01.html`：第一轮同一试算的抓取、附着和抽离成功前缀及其真实失败请求。
- `run01_lift_failure.png`：0.43 m 升降拒绝时的实际世界，碰撞对为
  `base_link` / `conveyor_longitudinal__mesh002_component000`。
  记录的 −1 距离值是相交/接触哨兵值，不是穿透深度。
- 最终 `cycle.html` / `.zip`：第三轮同一试算的含升降前缀及全部实际请求。
- `run01_records.tar.gz` / `run02_records.tar.gz`：前两轮失败请求、结果与日志，
  不作为第三轮轨迹输入。

离线 HTML 内嵌 Three.js、真实几何和运行数据，打开时不需要本机资产目录或网络。
ZIP 包含快照、同一试算前缀、请求/返回、可用搜索 trace 和运行报告。
失败升降使用实际失败的 after/sample 世界和带高，不再套用请求起点的低位场景。
没有记录的搜索树不补造；PTP/LIN/IK/外部轴明确标记不适用。

已做的定向检查：安装方向、原始 q 不变、同 q 相机方向保持、旧外参拒绝、
FK/原生模型绑定、碰撞对象清单、失败升降世界，以及客户端超时恢复。
已有相机、连续状态和导出共享数据回归通过。
本机缺少 pxr 的一项 USD 测试跳过，服务器实际导出 USD 并核对变换：
base 和 camera 误差 0，mast_part_019 最大误差 `2.78e-17`。
首轮浏览器 2917 项页面检查通过，FK 最大误差 `2.22e-16`；这不代表完整循环通过。
新加的客户端传输定向检查共 6 项通过，覆盖可恢复只读响应、不可恢复规划错误、
未知错误、真实 IPC 超时、取消和响应错配。

## 实际构建、运行及导出命令

在服务器工程 `/root/autodl-tmp/eco65-native-20261015/repo` 下运行。
源文件先以逐文件 SHA256 和既有基线核对后部署；记录位于各 `build*/source_deployment.json`。
没有升级软件环境。

```bash
chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c \
 'source /opt/ros/humble/setup.bash; \
  cmake --build /work-eco65-20261015/build-coupled-20261020 --target m710_moveit_worker -j2 && \
  cmake --install /work-eco65-20261015/build-coupled-20261020'

export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
S=outputs/eco65_native/layout_remount_20261021_scene01
"$P" tools/build_eco65_coupled_carton.py \
 --source outputs/eco65_native/cup_contact_20261020_scene01/snapshot.json \
 --home outputs/eco65_native/cup_contact_20261020_scene01/home_candidate.json \
 --layout-config configs/workcells/eco65_base_mast_remount_v1.yaml \
 --output "$S" --native-asset-root /work-eco65-20261015 --usd

# model 和 layout_attachments 按相同相对路径部署到 rootfs /work-eco65-20261015。
WORKER='chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /usr/bin/env M710_MOVEIT_INSTALL_PREFIX=/work-eco65-20261015/install-coupled-20261020 /bin/bash /work-eco65-20261015/ros2/m710_moveit_backend/worker.sh'
RUN=outputs/eco65_native/layout_remount_20261021_run03
"$P" tools/run_eco65_coupled_cycle.py --snapshot "$S/snapshot.json" \
 --home-candidate "$S/home_candidate.json" --worker-command "$WORKER" \
 --output "$RUN" --task-seconds 900 --connection-seconds 100 \
 --solver-seconds 8 --rpc-seconds 100 --ik-seeds 8 --max-grasps 24 --search-trace

# 失败运行选取同一原生试算保存的真实前缀；完整成功时使用 plan.json。
PLAN="$RUN/first_carton_extracted_lifted_candidate.json"
OUT=outputs/eco65_native/layout_remount_20261021_result03
"$P" tools/summarize_eco65_coupled_run.py --run "$RUN" \
 --snapshot "$S/snapshot.json" --prefix-plan "$PLAN"
"$P" tools/export_eco65_coupled_replay.py --snapshot "$S/snapshot.json" \
 --plan "$PLAN" --run "$RUN" --output "$OUT/cycle.html"

"$P" tools/export_eco65_warehouse_workstation.py --snapshot "$S/snapshot.json" \
 --home "$S/home_candidate.json" --output outputs/eco65_native/layout_remount_20261021_usd01
```

本地离线浏览器检查入口：

```powershell
$env:PYTHONPATH='src;tools'
.venv/Scripts/python.exe tools/check_eco65_coupled_replay_browser.py `
 --html outputs/eco65_native/layout_remount_20261021_result/cycle.html `
 --browser 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe' `
 --output outputs/eco65_native/layout_remount_20261021_result/browser_final
```

输出目录采用新名称，构建入口拒绝覆盖已有结果。原始 CAD、生成快照、USD、
HTML、图片及记录包保持忽略，随本地交付包提供，不写入 Git。
`cycle.zip` 还附带本报告、安装对比图、浏览器检查、构建/运行日志和静态 USD。
部署清单中 81 个运行文件与本地源文件哈希一致，见 `source_match.json`。

所有结果均为 `planning_only=True` / `KINEMATIC_REPLAY`，
`physical_execution=NOT_RUN`、`execution_ready=false`。
