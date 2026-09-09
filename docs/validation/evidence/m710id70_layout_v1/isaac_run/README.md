# Isaac Sim 单箱聚焦初始化回放证据

本目录是 2026-09-09 在 NVIDIA Isaac Sim 6.0.1.0 上完成的后端实跑输出。
结果为 **PASS**，但资格范围仅为“完整 40 箱静态初始化并聚焦一个目标箱”。目标箱
`carton_l07_c02` 用亮橙色显示；它没有被移出货垛。

- [`m710id70_layout_one_carton_focus.mp4`](m710id70_layout_one_carton_focus.mp4)：
  1280×720、30 FPS、180 帧、6 秒，三个固定视角。
- [`01_overview.png`](01_overview.png)、[`02_opposite.png`](02_opposite.png)、
  [`03_target_focus.png`](03_target_focus.png)：视频对应的三张无损关键帧。
- [`run_status.json`](run_status.json)：后端版本、合同/资产/适配器身份和资格边界。
- [`backend_scene_dump.json`](backend_scene_dump.json)：从实际 USD stage 与 articulation
  读回的 44 个 primitive、安装位姿和关节状态。
- [`backend_scene_audit.json`](backend_scene_audit.json)：读回值对冻结合同的反审计。
- [`isaac_run_manifest.json`](isaac_run_manifest.json)：环境隔离、计算身份、媒体属性与
  本目录文件哈希。

反审计没有缺失或额外 primitive。最大物体位姿误差为
`7.483020125764739e-08 m`，最大尺寸误差为 `9.536743172944284e-08 m`，机器人
安装位姿误差为 0，最大关节读回误差为 `1.1177019798580545e-07 rad`。适配器使用
显式 USD/float32 序列化审计阈值 `1e-6 m` 与 `1e-5 rad`；这不是碰撞 margin、IK/FK
容差或物理接触放宽。

本证据没有评估物理抓取、吸附、负载动力学、输送或完整单箱周期。车厢长度/高度和
完整制造 CAD 仍未知，因此也没有声明完整工作单元净空通过。
