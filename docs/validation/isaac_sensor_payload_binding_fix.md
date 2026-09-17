# Isaac 传感器载荷绑定修复（2026-09-17）

仅第三步载荷完整性；分支 `feat/v0.5-perception-ros2`。读取 AGENTS.md 并 fetch 后，
本地/远端基线均为 `43ed5eb893ffe6f63fda7f0fc1ae6632663b2e8e`，工作区干净；提交前再次 fetch 未变化。
前两步的采样机器人来源、非零速度、历史时间、世界桥时间准入和执行门控均保留。

## 原漏洞与消费边界

原适配器只校验 PNG/GT 文件，却读取另一份 RGB NPY，并无校验地读取深度、点云和元数据；
还在校验完成前逐个更新节点属性。模组采集已有深度/元数据/掩码哈希，兼容主目录缺深度/元数据哈希；
两条采集路径都会提前暴露中间 binding。`IsaacCaptureBinding` 往返会丢掉扩展字段和 robot_state。

修改生产代码前，以内部一致的最小合成 fixture 启动真实适配器，只改深度 NPY 的一个有效值，
保留 shape/dtype、PNG、GT 和 binding。旧 `_load_capture()` 没有拒绝，回归按预期失败：
[修复前真实节点日志](evidence/payload-binding-20260917/red-ros.log)。新路径拒绝为 `HASH_MISMATCH`。

| 采集端写入文件 | 正式绑定与检查 | 适配器实际消费/发布 |
|---|---|---|
| `sensor_rgb.png` | 既有 `rgb_sha256`，实际文件字节 SHA-256；RGB PNG 模式/尺寸 | **唯一 RGB 权威来源**，同一缓冲区解码为 RGB，发布 rgb8 |
| `sensor_rgb.npy` | 不属于本适配器的已验证输入 | 不读取、不使用；采集端仍保存供其他离线工具使用，不称其已通过本适配器验证 |
| `metric_depth_m.npy` | 接通既有 `metric_depth_sha256`；原始 float32、标定尺寸、光轴 Z 米制语义 | 同一已校验缓冲区加载，发布 32FC1 |
| `pointcloud_world_m.npz` | 新增 `pointcloud_sha256`，以及 array_key/dtype/unit/frame_id 字段 | 仅启用时读取 `xyz_m`，要求 Nx3 float32、米、world，发布 PointCloud2 |
| `capture_metadata.json` | 接通既有 `capture_metadata_sha256`，复用 CaptureMetadata 契约 | 发布采集 ID、时间、时钟域、帧和标定元数据 |
| `camera_info.json` | 既有 `camera_calibration_identity` 对规范化 JSON **内容身份**的校验；再核对 manifest 实际 K、T_W_C、尺寸、畸变等 | CameraInfo 使用已核对的 manifest 相机参数，TF 使用同一采集外参 |
| `gt_annotations.json` | 既有 `gt_snapshot_sha256`，核对 manifest/epoch/frame/time/module | 同帧 GT 检测和明确模拟真值 observation |
| manifest / binding.robot_state | 复用 manifest 内容指纹及第一步机器人身份/采样来源验证 | 仅有效 robot_state 发布 JointState，缺失/错误记录独立抑制 |
| `gt_instance_masks.npz` | 既有 `instance_masks_sha256` 往返保留 | 本适配器不读取，不新增掩码算法验收 |

PNG 由 Pillow 解码，要求 RGB 模式；没有 OpenCV BGR 数组直接发布的问题。被替换甚至不可解析的
RGB NPY 不影响经过验证的 PNG 发布，也不会把该 NPY 宣称为已验证缓存。
文件 SHA-256 一律针对实际字节；相机 calibration identity 和 manifest fingerprint 是已有的内容身份，
并非数组哈希或冒称文件字节哈希。CameraInfo 不再仅因两个 JSON 写了相同身份字符串就通过。

`src/unloading_perception/isaac_payload.py` 提供小型 CPU 文件边界；每个消费文件只读一次，
对同一字节缓冲区校验和解析，避免校验路径后再次打开文件。没有 ROS/Isaac/GPU 依赖；PNG 解码依赖
Pillow，已在项目依赖与 ROS 包运行依赖中声明。binding 沿用 v1 加性扩展，正式 from_dict/to_dict
深拷贝保留扩展字段，包括本轮载荷字段、已有 instance_masks_sha256 和第一步 robot_state。

## 身份、语义和写入

RGB/深度/元数据/相机/TF 按同一 manifest、module/camera、epoch、frame、capture_id、原始采集时间、
ros_sim_time 和 RGB/depth frame 校验；metadata 的 T_W_C、q1 与 manifest 相机记录一致。
robot_state 继续独立验证机器人资产、manifest、epoch/frame/sample_time 和关节顺序，
不把相机配置 q1 或 manifest 目标 q 冒充关节读回。此处不新增运动学或几何精度验收。

深度按既有 float32 optical_z_m 约定验证，先查原始 dtype 再允许发布端字节序转换。
NaN/+Inf、其他原约定的无效深度值原样保留；全无效深度帧也不被新增几何阈值拒绝，不填零或背景距离。
复用 RegisteredRgbdFrame 的参数契约；本 Isaac 路径要求既有同轴理想注册模式。
RGB/depth 两个光学帧均发布同一已验证外参。点云关闭时既不读取文件，也不构造 PointCloud2；
启用时任何缺失、哈希或格式错误都会阻断**整组样本**，包括 /clock。

采集脚本的上模组、下模组和兼容主目录共用 begin/finalize 辅助函数：覆盖文件前移除旧完成记录，
先写完 RGB、深度、点云、相机、GT 和 metadata，再生成实际文件哈希、验证完整绑定，最后通过临时文件、
flush/fsync 和原子 replace 发布 binding。新增点云写入复用原有 stride=2 的 optical-Z→world 算法。
不再提前写中间 binding；PNG 写入失败会显式报错；没有对 binding 自身做自引用哈希。

## 历史数据与失败策略

已有可信 PNG、深度、metadata 哈希及一致相机/采集身份的历史记录可验证；旧兼容主目录的
`module_0_main` capture_id 别名只在缺少显式 module_id 的既有格式下接受，实际相机仍由内容校验定位。
旧深度的 float32/米/optical-Z 语义沿用相机契约；不要求重新生成 RGB NPY 哈希。
关键深度或 metadata 缺少既存哈希时返回 `BINDING_MISSING`，不能进入已验证 RGB-D 发布；
不会计算当前哈希填回 expected 字段，不改历史证据、不重渲染。点云启用但缺其绑定同样拒绝。

节点先在局部变量完成全部载荷加载与校验，再整体提交当前样本；旧样本可留作诊断，
但 `sample_ready=false` 时不会发布旧数组、旧 JointState 或新时钟。稳定原因包括
`HASH_MISMATCH`、`FILE_MISSING`、`BINDING_MISSING`、`IDENTITY_MISMATCH`、`FORMAT_INVALID`，
注明文件类别及可用的 module/capture/epoch/frame，不记录大数组。

初始样本坏则启动失败。序列中 A 成功后若 B 损坏，B 不发任何载荷或 /clock；下一周期尝试后续样本，
合法 C 加载完成后再于后续周期发布。若末样本坏则保持不发布。合法静态样本重复回放只用已验证缓存，
不每周期读文件/哈希，也不刷新采样时间。传感器有效但 robot_state 缺失/非法仍可回放已验证图像，
仅抑制 JointState；图像可回放不代表世界快照可规划。

## 验证

所有运行在授权服务器 CPU、Ubuntu 22.04.5、ROS 2 Humble/Python 3.10；合成文件仅 3×2 像素。

| 测试 | 实际结果 |
|---|---|
| 原漏洞真实适配器回归，旧代码 | 1 failed：DID NOT RAISE，确认漏检 |
| 新载荷 CPU（含上下模组 finalizer、历史别名及同缓冲区解析） | 43 passed |
| 新真实适配器 ROS 载荷及 A→坏 B→C | 4 passed |
| 前两步及相关 CPU 回归 | 74 passed |
| 前两步真实 ROS 接线、时间及映射回归 | 9 passed |
| 默认轻量 pytest | 487 passed，1 deselected |
| 采集脚本、载荷模块和适配器编译检查 | exit 0 |

真实 ROS 测试核对 Image 字节/编码/尺寸/frame/time、CameraInfo、metadata、TF、JointState 和启用的
PointCloud2；坏 B 的整组发布被阻断，C 用不同图像、深度、点云、外参和时间恢复。
删除 C 的源文件后重复发布仍使用已验证缓存。没有替代发布器或 monkeypatch 被测 ROS 加载路径；
CPU 的受控文件读测试仅用于确认文件在读取后被替换也不会改变已验证的同一缓冲区。

日志及 LF 规范化源码 SHA-256 见 [证据目录](evidence/payload-binding-20260917/tested_sources.json)。
新测试沿用默认 pytest 和现有 Humble CI colcon test 自动收集；未新增 skip 或放宽断言。
当前提交的实际 CI 查询状态在交付回复报告。

```bash
# 服务器仓库根目录；复用已构建的当前 ROS 消息接口。
CPU=/root/autodl-tmp/v05-acceptance/cpu/venv/bin/python
env -u PYTHONPATH "$CPU" -m pytest -q tests/test_isaac_payload_binding.py
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/v05-acceptance/joint-contract-20260917/install/setup.bash
export PATH="/usr/bin:/bin:$PATH"
export PYTHONPATH="$PWD/ros2_ws/src/unloading_ros_bridge:$PWD/src:$PWD/packages/unloading_contracts/src:$PYTHONPATH"
export ROS_DOMAIN_ID=175
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_isaac_payload_transport.py
env -u PYTHONPATH "$CPU" -m pytest -q tests/test_isaac_joint_capture.py tests/test_perception_time_admission.py tests/test_perception_integration.py
/usr/bin/python3.10 -m pytest -q ros2_ws/src/unloading_ros_bridge/test/test_isaac_joint_state_transport.py ros2_ws/src/unloading_ros_bridge/test/test_perception_time_transport.py ros2_ws/src/unloading_ros_bridge/test/test_humble_imports.py
env -u PYTHONPATH "$CPU" -m pytest -q
/usr/bin/python3.10 -m py_compile scripts/isaacsim_perception_capture.py src/unloading_perception/isaac_payload.py ros2_ws/src/unloading_ros_bridge/unloading_ros_bridge/isaac_sensor_adapter_node.py
```

未启动 Isaac、SAM/MoGe、GPU 推理、渲染或真机；新增测试不启动执行节点，不生成授权或轨迹，
`enable_hardware=false` 未改。真实 Isaac 写入与传感器物理正确性尚未验收，fixture 不能替代该证据。
哈希只检出意外替换/混帧/未完成写入，不提供恶意篡改签名保证，也不证明深度精度、算法或可执行性。
其他离线算法消费者未在本轮扩展为统一验证路径；未开始第四步几何边界修复。
