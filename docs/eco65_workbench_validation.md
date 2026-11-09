# 阶段一验收记录（2026-11-09）

验收范围是可运行的 Ubuntu 工作台、回放和假设备边界，没有连接任何实物。
**硬件验证 NOT_RUN**。实际源版本见 `configs/workbench/source_versions.json`。

## 已完成检查

| 检查 | 结果与证据 |
|---|---|
| Ubuntu 安装 | Ubuntu 22.04.5，Python 3.10.12，Tk 8.6；独立 venv 安装 editable workbench/scene/dev extras |
| 无硬件启动 | 实际 Tk 窗口打开，三个设备均 DISCONNECTED；不导入可选硬件 SDK/模型权重 |
| 回放显示 | RGB、对齐深度、检测列表、原分割、三组关节和四路压力；来源 REPLAY/original=SIMULATED |
| 外参与配对 | 非正交外参拒绝并清空旧有效外参；训练状态、缺失会话保留；人工静止单独标记 |
| 过期/断连 | STALE 后撤销配对；停止回放清空图像/检测；无自动模拟替代 |
| 只读边界 | 假 SDK/串口检查构造、启动、连接、查询、旧配置、报警与退出；只出现连接/查询/关闭，无设备控制命令 |
| SDK 相机接口 | 假 D435i 实测 profile 筛选、非法组合拒绝、对齐、SDK depth_scale、两路帧号及时间域保留 |
| 压力 | 分帧/校验、四通道独立接收年龄、ASCII 不等同二进制、0x71/0x72 映射保持 |
| 定向 Python 检查 | 本地 Windows 与 Ubuntu 同一组 68 项通过；23 项为新增阶段一检查，其余为既有机器人/MoveIt/相机定向检查 |
| 两工程 FK | 三组姿态全部 link 对比，最大分量差 2.5263e-16；记录模型哈希和限位差异 |
| 原生 MoveIt | 既有 Humble worker 实际启动 READY，冻结模型身份匹配；三姿态 FK 最大矩阵差 7.7716e-16 |
| 原仿真入口 | `tools/run_eco65_moveit2_single_carton.py --help` 正常；未改规划/IK/抓取算法，未重跑整箱运动 |
| 真实 GUI 布局 | Ubuntu Xvfb 上实际 Tk 窗口与场景窗口；检查无设备、回放、坐标、过期、完整几何显示和正常关闭 |

运行环境：Linux 5.15.0-94，numpy 2.2.6、scipy 1.15.3、Pillow 12.3.0、
opencv-python-headless 5.0.0.93、pyserial 3.5、MuJoCo 3.17.0、trimesh 5.2.0。
Ubuntu 工作目录 `/root/autodl-tmp/eco65-real-integration-20261109/repo`，独立环境在其同级 `venv`。
原 ROS rootfs 和工作区未修改。这里只做软件模型读取和 FK 回归，没有输出机械臂执行任务。

## 可复跑命令

```bash
python -m pytest \
  tests/test_workbench_readonly.py tests/test_workbench_coordinates.py \
  tests/test_robot_load.py tests/test_moveit2_backend.py \
  tests/test_moveit2_tcp.py tests/test_eco65_tilted_camera.py -q

# 无头测试需额外 apt install xvfb；本脚本打开真实 Tk 窗口，不是生成 UI mockup。
xvfb-run -a -s '-screen 0 1600x1100x24' python tools/check_eco65_workbench_ui.py \
  --output outputs/workbench/ui \
  --scene-snapshot outputs/workbench/scene_snapshot.json \
  --scene-input-root /path/to/archived-source-repo

python tools/check_eco65_workbench_moveit.py \
  --snapshot outputs/workbench/scene_snapshot.json \
  --scene-input-root /path/to/archived-source-repo \
  --worker-command '/path/to/existing/humble/worker.sh' \
  --output outputs/workbench/moveit_acceptance.json
```

没有私有官方模型资产时，两项官方模型测试会明确跳过，不能计为已验证。本次资产已恢复，以上 68 项没有跳过。
场景验收保留旧快照的配置来源；当前源码有三份配置与历史快照不同，使用显式归档输入根逐项验证，详见安装说明。
首次 GUI 实测发现日志区域被挤出窗口，已固定日志区域并缩小预览；初次场景加载的配置/基本形状问题已在显示适配层处理。

原 `tests/test_eco65_transfer_policy.py` 另有两个既有失败：
`test_plan_recomputes_fk_instead_of_trusting_saved_zero_errors`、
`test_segmented_synthetic_contract_preserves_each_native_parent_and_attachment`。
都在旧 synthetic_plan_fixture 缺少 `init['collision_policy']` 时抛 KeyError。
已在完全未修改的 `269ab67` detached worktree 复现相同两项失败（该文件 8 通过、2 失败）。
未为本轮验收改动规划算法或隐去失败，也未声称全量历史测试全绿。

## 截图与证据

实际 GUI 截图保存在本地交付目录 `outputs/workbench/ui_final/`：
`ubuntu_no_devices.png`、`ubuntu_replay.png`、`ubuntu_coordinates.png`、
`ubuntu_stale.png`、`ubuntu_complete_scene.png`。生成图片遵循工程规则不入 Git。
机器可读的 FK、GUI、MoveIt 结果保存于 `docs/evidence/eco65_workbench/`。

## 待现场验证

* D435i 的设备枚举、USB 带宽、支持流组合、RGB/深度对齐、真实 SDK 时间域、深度尺度和畸变；不以假 SDK 结果替代。
* ECO65-B SDK 与固件版本、控制器关节顺序/符号/零位、反馈延迟及断网行为；现场抓包确认仅连接/状态查询/关闭。
* 实际 RS485/RS232 接线、电气层、串口权限、被动心跳是否启用、各表量程、校验/报警与断线；确认打开串口的电气行为无控制副作用。
* 独立标定验证数据采集、当前相机真实安装与仿真冲突定位、world/base 实测安装、70 mm 探针记录、140 mm 实际吸具位置与朝向。
* 原始标定会话缺失仍待补齐；未生成替代采样或虚构验证哈希。
* GroundingDINO/SAM 真实权重、torch 环境、实拍箱面分割与深度几何质量。当前验证了已有输出接入、适配合同和离线分割显示，未运行神经网络权重推理验收。

下一阶段的轨迹提交/反馈/取消接口已经声明但全部禁用；实际运动、气路状态机、输送带、自动卸箱和在线闭环均不在本轮验收内。
