# S50 安装坐标、任务入口与运行时核对

本轮基于 `b7bb327c0127881d52c24fb474f8dea64e119501`，提交到 `feat/v0.5-s50-moveit2-native-single-carton`。本次是模型接入进展，不是“机械臂替换完成”：实际控制器映射、安装绕轴定向及 S50 法兰与原工具的物理配合仍缺依据。正式初态 IK、单箱原生路径及动态执行均未运行，没有结构不可达结论。

## 冻结范围

| 参数 | 保持值 |
|---|---|
| 横带前沿至箱堆 | 700 mm |
| 真实安装面世界位置 | [-1.85, 0.15, 0.90] m |
| 安装面高度 | 900 mm |
| 安装参考朝向 | 沿用冻结基线 |
| 横带范围 / 初始高度 | 550–1500 / 600 mm |
| 工具原延长段 | 200 mm |
| 初始箱数 | 40，未导入已卸检查点 |
| 工具 / 箱体质量 | 20 / 42.5 kg，未修改 |

非机器人实体、尺寸、位姿、机构行程和工具固有几何比较通过，见 [入口与布局核对](entry-routing-validation.json)。真实安装参考与 CAD 模型基座分别记录；没有通过移动布局或修改工具取得结果。FANUC 分支保持 `a3f08cc`，其工作树与实验未改动。

## 已实现与验证

- 安装接触 v2 锁定 `W_model_base = W_installation_reference @ binding.mounting_from_base`。CAD 支承面的切向轴不再被误当作世界安装方向；安装轴、支承面、原支座覆盖及非穿透仍独立检查。详见 [坐标契约](mount-coordinate-contract.md)。
- 服务器重新编译实际原生 worker 和安装 probe。8 个案例的 Python/Coal 与 C++ 安装许可一致，56 个 link–支座相交布尔结果一致，见 [比较报告](mount-comparison.json) 和 [原生记录](native-mount-report.json)。已声明的非零绕轴映射是明确标记的协议回归，不是实机安装参数。未声明旋转、平移、下沉被拒绝；活动连杆撞支座没有得到豁免。
- 生成器输入配置与生成的 `task/model.yaml` 分离。准入、初态/单箱入口、查看器共同解析已生成且哈希绑定的模型；文件缺失或过期时保留缺项，不回退为预览范围。路由测试中的人工映射仅是测试夹具，不写入正式配置。
- 正式成功计划的指标入口读取 `batch_root.height_m` 及外轴阶段逐条 `checks[].height_m`，保留原始关节路点；不会默认整个轨迹处于 600 mm。缺少记录时报告指标未测量，保留原全机器人安全验证。此连接目前通过协议测试，尚无真实 S50 成功计划可供测量。
- 在完整 148 个环境实体、40 箱及真实横带凸实体上，重新执行 15 个 CAD 诊断姿态，共 20,520 次查询，未知查询数为 0。原始相交全部保留，未放宽相邻机构或腕部/工具排除。见 [运行摘要](collision-summary.json) 和 [接触规则复核](adjacent-contact-review.json)。

### 加载器三角形差异尚未全部解释

两次实际审计均保留原始失败：[首轮](loader-first-run.json) 按顶点行顺序比较，发现加载器顺序变化；随后加入精确坐标多重集置换及保留绕序的角点循环映射，[第二轮](loader-permutation-run.json) 仅 J1 局部补片获得完整对应，其余 8 个几何仍返回 `UNEXPLAINED_VERTEX_CONVERSION`。没有用近邻配准或扩大误差阈值制造通过。

因此，加载器少掉的 56 个三角形与源零面积面 **尚未完成逐项证明**。源顶点与实际加载数组已经在服务器保存，需继续解释坐标转换后才能建立完整面索引对应。该审计缺口与真实距离查询完成、J1 残余 BRep 问题分别记录；任何一项都没有被单独当作整个碰撞模型合格的依据。

### 分部件指标

下表仅是这 15 个 CAD 诊断姿态、600 mm 横带下的样本最小值；姿态未获得真实限位及安装资格，不能作为卸箱能力或运动路径净空结论。所有机器人安全检查仍保留。

| 测量集合 | 与横带最小样本距离 | 对应构型 |
|---|---:|---|
| 全机器人（含固定底座） | 413.911 mm | cad_q0 |
| 活动连杆 1–6 | 543.989 mm | axis_1_negative |
| 肘部：连杆 2、3 | 763.116 mm | coupled_b |
| 腕部：连杆 4、5、6 | 2160.277 mm | coupled_a |

[运行摘要](collision-summary.json) 同时记录每个极值的六关节角、最近对象对、安装轴坐标系下 X/Y 占用边界与最大水平半径。腕部仍处于较高位置；较大的横带距离不代表初态合格，也不证明能折叠进入目标作业空间。

## 真实资料仍缺什么

[来源复核](source-review.md) 与 [机械字段清单](mechanical-source-review.json) 记录已确定的支承面、六条轴线、法兰尺寸、手册限位及工具内在变换。最低补充资料是：

1. 与这份 STEP 对应的参考姿态六个控制器读数，以及各轴正转方向；或同版本厂家零位/符号表。用于确定每轴 `q_controller = sign × q_sim + offset`，尤其 J2/J3，进而转换真实限位。
2. 安装俯视图：将一个唯一基座接插件/定位孔方向与世界 +X 联系起来，给出绕安装法向的角度与正方向。支承面高度和轴心已经确定，缺的是安装绕轴定向。
3. S50 法兰至原吸具安装参考的装配/剖面图：轴向与侧向尺寸、法向、定向基准、孔位对应，以及实际连接件实体。不能用虚拟 TCP 偏移代替真实连接。

速度版本冲突只影响时序资料资格；质量惯量、力矩与额定负载资格单独记录，不阻断 CAD 预览或独立几何诊断。没有把这些缺项合并成“所有工作都不能做”。

实际运行 [任务生成](task-generation-local.json)、[准入](admission-local.json) 和 [单箱入口](single-carton-local.json) 均按真实缺项退出（码 2）。代表箱为 `carton_l07_c02`，IK 尝试 0、规划尝试 0，未启动全 40 箱或 Isaac。

## 可复现命令

在项目根目录使用现有规划 Python 环境；前两步在资料缺失时应停止，不应以人工测试参数补齐。

```bash
python tools/build_s50_task_model.py --config configs/robots/huayan_s50_2000.yaml \
  --output outputs/s50_mount_entry_20261111/task-generation.json
python tools/check_s50_integration.py --config assets/robots/huayan_s50_2000/task/model.yaml \
  --output outputs/s50_mount_entry_20261111/admission.json
source /opt/ros/humble/setup.bash
cmake -S ros2/m710_moveit_backend -B build/moveit -DCMAKE_BUILD_TYPE=Release
cmake --build build/moveit --target m710_moveit_worker s50_mount_contact_probe -j2
python tools/plan_s50_single_carton.py --mode single-carton \
  --config assets/robots/huayan_s50_2000/task/model.yaml \
  --worker-command "$PWD/build/moveit/m710_moveit_worker" \
  --output outputs/s50_mount_entry_20261111/single-carton
# 仅当上一步真实生成轨迹后执行：
python tools/export_s50_cad_motion_viewer.py \
  --task-config assets/robots/huayan_s50_2000/task/model.yaml \
  --trajectory-report outputs/s50_mount_entry_20261111/single-carton/native-joint-trajectory.json \
  --output outputs/s50_mount_entry_20261111/task-trajectory-viewer.html
```

独立 CAD 诊断仍可运行：

```bash
python tools/audit_s50_loader_triangles.py --output outputs/s50_mount_entry_20261111/loader-permutation
python tools/diagnose_s50_runtime_collision.py --output outputs/s50_mount_entry_20261111/collision
python tools/build_s50_mount_contact_probe.py --coal --synthetic-mapping-cases \
  --output outputs/s50_mount_entry_20261111/mount
build/moveit/s50_mount_contact_probe outputs/s50_mount_entry_20261111/mount/native-input.json
```

已有 [离线六关节查看器](../s50_task_collision_20261111/viewer.html) 保留上轮实际诊断记录，双击即可打开；它不包含本轮新增分部件指标。浏览器组件未能可靠识别 Edge 地址并强制停止，见 [浏览器验收记录](browser-validation.json)，因此没有宣称本轮浏览器验收通过。

完整原始碰撞记录和加载器 NPZ 数组留在服务器 `/root/m710-gap-search-20261107/rootfs/work-s50-mount-entry-20261111-v1/outputs/s50_mount_entry_20261111`。仓库只收录代码、配置和摘要报告，未下载完整证据包。本地回归清单见 [local-validation.json](local-validation.json)；有交集的测试批次不累加成独立测试数量。
