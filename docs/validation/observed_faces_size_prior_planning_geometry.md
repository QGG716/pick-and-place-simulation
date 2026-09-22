# 观测面与显式尺寸先验的离线规划几何适配

## 基线与边界

分支 `feat/v0.5-perception-ros2`；开始时本地与 fetch 后 origin 均为
`0fc381b0e5c5affb6f69c775092ab9eee7e48d34`，工作区干净。已阅读根目录
AGENTS.md。仓库是独立 worktree，没有回退、清理、合并或复制其他分支算法。

本轮仅新增 `unloading_perception/planning_geometry.py`、默认关闭的先验配置、
CPU 测试、独立事后评价工具和本说明/证据。没有改动观测生成、融合、跟踪、执行上下文、
取消/停止状态机、授权、时间、ROS 消息、场景资产或任何运动规划/碰撞算法。
前两轮实现及测试保留。

沿用分支开发版本：根 Python 包 `0.5.0.dev0`、Python contracts `1.1.0`；
未修改的 ROS interfaces `1.2.0`、execution bridge `1.2.1` 不升版。
新增的离线配置和产物分别使用 `observed_size_prior_v1`、`offline_planning_geometry_v1`，
不改变既有消息契约或在线接纳规则。

## 查明的几何语义

- `metric_faces.py` 在真实深度支持区域上拟合平面，投影像素射线，构造内接四边形；
  包含收缩支持检查及最大观测矩形策略。`frozen_support_regions` 是像素支持区域，
  `boundary_2d_px`、`corners_3d_m` 是该局部面片的边界，不能直接当作完整箱面边界。
- `observed_faces.py` 仅保留直接测量、最终支持验证通过的面，记录采集身份、支持量、
  平面残差、边界诊断；世界变换必须显式使用捕获时 `T_W_C`。平面法向朝向相机。
  边缘局部具有 physical-edge 支持，不等于四个物理角点全部认证。
- 这组 `rgbd_cuboids.json` 保持 `accepted=false`、
  `complete_observability=UNRESOLVED_PHYSICAL_BOUNDARIES`。
  契约中的面保持 `volume_status=UNKNOWN`；面片没有提供完整体积证据。
- `fusion.py` 以世界面几何关联；不同模块/多张面片不保证不同几何方向。
  `algorithm_handoff.py` 保留全部 contributor 表面与未知体积，清空完整位姿、尺寸、
  角点及目标资格；关联冲突/歧义不能被本适配层当成一个可靠对象。
- `algorithm_artifact.py` 校验索引、模块观测、融合及总观测的哈希和采集绑定，继续要求
  `raw_image_automatic=false`、oracle proposal 标记和无抓取资格。
  `scene.py` 仍把未知几何/历史回放作为阻塞项。
- 规划侧已有 `unloading_sim.geometry.OBB`，尺寸语义是半长、旋转列为局部轴在世界的方向。
  本轮提供 `envelope_to_obb()` 并实际构造、校验其角点；没有新建平行场景模型。

## 观测、先验和派生结果

原始观测在输出目录按原始字节复制；逐对象报告同时完整保留原始表面，包括边界、
法向、残差、支持、捕获时间、标定、变换和证据引用。原文件和历史验收文件不变。

先验来自 `configs/perception/nominal_size_prior_offline.json`，默认 `enabled=false`。
CLI 必须显式指定 `--enable-size-prior`，才在新目录记录启用后的配置；源配置不改。
Python API 还要求 `enabled=True` 和配置开关均启用。

本次固定实验配置如下，均在适配前确定：

- 规格：仓库 `configs/validation/m710id70_v3.yaml:scene.box_sizes_m` 记录的
  A `[0.6,0.4,0.3]` / B `[0.4,0.6,0.3]` 米具有相同无序尺寸。
  对全部选中对象使用同一无序规格，枚举六种轴对应，不按 GT 身份选规格。
- 尺寸区间：精确名义 `[0.6,0.6]`、`[0.4,0.4]`、`[0.3,0.3]` 米。
  零尺寸公差是**名义长方体实验假设**，不是实际外观网格经过测量的尺寸误差界。
  API 支持非零区间，但本次没有编造实际箱体公差。
- 独立环境先验：固定世界轴向、理想长方体。这个先验由配置声明，未读取任何仿真位姿。
  不支持连续方向不确定性；缺少固定轴向时返回信息不足。
- 独立测量假设：面片每个顶点在每条配置箱轴上的误差绝对值不超过 2 mm，
  法向与对应轴夹角不超过 0.01 rad。它们是尚未标定的实验误差上界，
  **不是**平面残差的替代解释、covariance 或置信概率。
  法向阈值只核查轴对应，不把箱体真实方向扩展到该夹角范围。

配置包含规格 ID、来源、适用范围、误差依据和配置身份；输出另存配置指纹。
生产适配函数只接收表面和配置，不接受 GT 中心、姿态、角点、USD 变换或匹配答案。

## 有限范围的解析算法

将世界点投影到显式固定的箱轴（只旋转坐标，不移动原始面片）。法向必须为有限单位向量，
朝向捕获相机、与唯一配置轴相容；校验有限坐标、凸非退化面片、平面位置、正尺寸区间、
采集身份、标定及右手正交变换。所有输入面共同约束，不能删掉困难面或放宽阈值。

对每种尺寸轴对应，逐轴求箱体负/正面坐标 `L,U` 的可行区间。测量点坐标为 `p`，
声明误差为 `e`、尺寸区间为 `[dmin,dmax]`：

```text
所有面片顶点： L <= min(p)+e，U >= max(p)-e
负向观测面：   max(p)-e <= L <= min(p)+e
正向观测面：   max(p)-e <= U <= min(p)+e
尺寸先验：     dmin <= U-L <= dmax
```

这是每轴两个变量的差分约束。可行端点可由相对面的上下界和尺寸界解析传播得到；
`[min L,max U]` 覆盖全部可行的连续中心位置和尺寸。
对所有可行轴对应取这些区间的并集外包络，仍为配置轴向的一个 OBB。
没有离散采样连续位姿并宣称全面覆盖。输出区间忽略跨轴关联只会扩大包络，不会缩小。
1e-12 相对量级的向外浮点保护不是拟合阈值。

一个局部单面在固定轴向、有限尺寸上界下可以限定平移的有限范围，
但通常不能唯一确定中心。包络可能很宽，不是选了某个唯一姿态。
三个独立轴的面配合已知轴对应、精确尺寸和零误差，CPU 用例可恢复一致体积；
这个特殊情形也不是视觉独立重建。

状态分别为 `DISABLED`、`INVALID_INPUT`、`INSUFFICIENT_INFORMATION`、
`PRIOR_OBSERVATION_CONFLICT`、`MULTIPLE_HYPOTHESES`、`CONSERVATIVE_VOLUME`。
多假设状态仍可具有条件保守外包络；每个轴对应候选保留自己的连续区间，
拒绝的对应也逐个记录。关联歧义/冲突阻止导出对象规划几何。

`conditional_conservative_volume` 仅指相对上述全部假设的保守性。
`unique_reconstruction` 只描述该条件模型内的解，不代表视觉完整重建。
`planning_admissible`、`candidate_eligible` 始终为 false；没有抓取、可达性、
碰撞、场景完整性或执行安全结论。

## 固定真实算法样本和结果

复用已存在的真实 SAM + RGBD 算法产物（输入为 Isaac 渲染图像，不是硬件采集，也不是本轮合成 fixture）：

```text
/root/autodl-tmp/v05-acceptance/single-handoff-20260918/algorithm-run-01/algorithm_artifact.json
sha256: c91e80018c7b46b98bcf7b701da5faa2869e5b22f4e97993bc8721bd5ebc7b58
epoch: carton-assets-capture-f24f63a
sequence: 100
capture_time: 1.6666666666666667, ros_sim_time
```

它仍是 **ORACLE-PROMPTED**，`raw_image_automatic=false`。
适配前固定“全部融合 source_instance_id 按字典排序取前六个，不按结果或 GT 筛选”，
见 [selection.json](evidence/size-prior-20260922/selection.json)。没有换样本或丢弃失败候选。
每个对象使用上面的同一尺寸/环境/误差先验。

下表 ID 省略共同 `fusion-` 前缀；来源格式为“模块 / SAM 实例末尾编号”，
完整源身份和原始表面均在 [derived_geometry.json](evidence/size-prior-20260922/derived_geometry.json)。

| 融合对象 ID | 输入来源 | 面数 / 独立轴数 | 原本缺少的约束 | 结果 / 候选数 | 可转换 OBB |
|---|---|---:|---|---|---|
| 0983d381278cea8cfac4 | lower / 20 | 1 / 1 | 完整物理边界、另两轴面位置、尺寸轴对应 | 多假设 / 4 | 是，条件包络 |
| 1148b5c2ee6ec2917b37 | lower / 11 | 1 / 1 | 同上 | 多假设 / 4 | 是，条件包络 |
| 195907e7f03fb6450efe | lower / 18 | 2 / 2 | 第三轴面位置、完整边界、尺寸轴对应 | 多假设 / 4 | 是，条件包络 |
| 1c8800b84990d4d84a27 | upper / 10；lower / 30 | 2 / 1 | 两模块仍同向，缺另两轴面位置及尺寸轴对应 | 多假设 / 4 | 是，条件包络 |
| 23d2ea899ff42fa46fe9 | lower / 10 | 1 / 1 | 完整物理边界、另两轴面位置、尺寸轴对应 | 多假设 / 4 | 是，条件包络 |
| 23e1093828c52b5cfd0e | lower / 14 | 2 / 1 | 两张局部面仍同向，缺另两轴面位置及尺寸轴对应 | 多假设 / 4 | 是，条件包络 |

六个对象各有两种轴对应因面位置/支持包含与尺寸联合不相容而拒绝，四种保留；
无唯一解、无完整视觉重建成功。所有包络在 Python 中实际转换成既有 OBB，并生成角点。
典型包络约 `0.604 × 0.834 × 0.935 m`，明显宽于名义箱体，展示了缺失约束的实际代价。

保留原始 84 个 unknown region、31 个未选融合对象、全部模块 coverage 和原始采样时间。
输出始终保留 `HISTORICAL_REPLAY_DISPLAY_ONLY`、未知区域和离线执行阻塞，
没有修改任何 observation、binding、summary 或历史验收文件。

## 独立事后真值评价及限制

冻结派生文件后才运行 `tools/evaluate_size_prior_nominal.py`，读取已有 mask evaluation 的
唯一显著匹配，并校验 GT 快照与 capture binding。多模块必须指向相同对象和相同名义几何，
否则评价歧义；不从候选中按 GT 挑选最佳结果，不调用适配器或修改配置。

| 融合对象短 ID | 事后匹配的名义 GT | 包络覆盖名义 8 角点 | 实际外观网格 |
|---|---|---|---|
| 0983d381278cea8cfac4 | carton_l04_c03 | 全覆盖 | 未评价 |
| 1148b5c2ee6ec2917b37 | carton_l02_c04 | 全覆盖 | 未评价 |
| 195907e7f03fb6450efe | carton_l04_c01 | 全覆盖 | 未评价 |
| 1c8800b84990d4d84a27 | carton_l06_c03 | 全覆盖 | 未评价 |
| 23d2ea899ff42fa46fe9 | carton_l02_c03 | 全覆盖 | 未评价 |
| 23e1093828c52b5cfd0e | carton_l03_c02 | 全覆盖 | 未评价 |

完整 [nominal_evaluation.json](evidence/size-prior-20260922/nominal_evaluation.json)
记录输入哈希、匹配、名义角点和每轴覆盖余量。GT 自身明确标记
`NOMINAL_CUBOID_ONLY; EXACT_SURFACE_CORNERS_AND_PLANES_NOT_EVALUATED`。
因此 **6/6 名义覆盖不等于实际外观网格保守性得到证明**。原场景使用替换的外观资产，
本轮没有导入 USD 网格或重新运行 Isaac，也没有将 USD 变换传给生产适配器。

## 复现与测试

服务器新目录 `/root/autodl-tmp/v05-acceptance/size-prior-20260922/`；
新输出 `derived-run-01/`，目录必须不存在且不能位于原算法产物目录内。
完整复制的原观测留在服务器新目录；仓库已有原始观测归档，本轮没有再重复提交该大文件。
启用后的 [prior.json](evidence/size-prior-20260922/prior.json) 与所有候选/拒绝记录另存于新证据目录。

```bash
export PYTHONPATH=src:packages/unloading_contracts/src
python3 -m unloading_perception.planning_geometry \
  --artifact /root/autodl-tmp/v05-acceptance/single-handoff-20260918/algorithm-run-01/algorithm_artifact.json \
  --artifact-sha256 c91e80018c7b46b98bcf7b701da5faa2869e5b22f4e97993bc8721bd5ebc7b58 \
  --prior configs/perception/nominal_size_prior_offline.json \
  --selection docs/validation/evidence/size-prior-20260922/selection.json \
  --output-directory derived-run-01 --enable-size-prior
# objects=6, conditional_envelopes=6, planning_admissible=false
python3 evaluate_size_prior_nominal.py \
  --derived derived-run-01/derived_geometry.json \
  --output derived-run-01/nominal_evaluation.json
# 6/6 COVERED (nominal-only)
```

服务器本次将仓库 `tools/evaluate_size_prior_nominal.py` 上传在工作目录根目录执行；
在仓库内复现时使用 `python3 tools/evaluate_size_prior_nominal.py`。

本地使用 `.venv310/Scripts/python.exe`，`PYTHONUTF8=1`；实际测试命令：

```text
python -m pytest -q tests/test_planning_geometry.py --basetemp=tmp/pytest-size-prior-second
# 27 passed
python -m pytest -q --basetemp=tmp/pytest-size-prior-full
# 778 passed, 1 deselected in 121.90s
```

定向测试覆盖独立多面、单面与连续平移歧义、尺寸轴多解、面/尺寸冲突、
内接边界不作物理边界、遮挡及不完整支持、错误法向、缺失标定、非法坐标/尺寸/变换、
默认关闭、旋转和全长/半长语义、尺寸与位置连续极值覆盖、输入文件不变、原始回放门控、
哈希拒绝和拒绝覆盖输出目录。

未运行 SAM/米制重建/Isaac、Humble 集成、抓取/IK/RRT/碰撞搜索、轨迹或机器人动作。
没有启动执行桥。仍缺实际规格公差和测量误差标定、真实外观网格评价、缺失方向/物理边界证据、
未知箱体与遮挡区的完整场景观测。本轮到离线输入准备为止。
