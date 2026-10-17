# ECO65 J1 相机视角演示

此入口独立于触碰标定与卸货验收：只复用已绑定的当前组合场景，
读取官方 FK，逐状态渲染。它不调用 IK、规划器、碰撞验收、控制器或真实硬件，
也不修改这些入口的门禁。既有探针支架/工具间隙不足保留。

输出标记为 `KINEMATIC_VISUALIZATION`、`MOTION_SAFETY_NOT_EVALUATED`、
`Dynamics: NOT_EVALUATED`。物理时间轴停止，视频使用独立的 `replay_time_s`。

## 入口与来源

`scripts/isaacsim_visualize_eco65_j1.py` 接受显式的配置、当前相机派生 USD、
该 USD 的 SHA256、源码基线提交及全新输出目录。它复用阶段 1.1 的场景来源绑定，
再核对相机 USD 的真实引用链、XD500 顶点、安装矩阵、替换件停用状态，
以及实际使用的 ECO65-B URDF/网格哈希。其他型号、未使用 CAD 和制造图纸不属于本入口输入。

`configuration.display_q` 提供固定 J2～J6。工具只对 J1 做 73 点有限区间括定和
标量求根，以实际光心绕轴轨迹求水平朝向箱堆的角度。优先选择限位内的 +90°
转开端点，否则使用 −90°；第二次转开复用同一端点。

默认 5 fps，四次停留各 2 秒，三次转动各 6 秒。五次 smoothstep 的峰值速度
与已有 J1 限速核对，必要时延长转动时间。逐帧更新所有同级 link 根节点、
工具根位姿及诊断坐标系；桅杆/相机和探针分别自然继承 link_1/link_6。
不会再次给 Visual/Collision 子节点施加 FK。

传感器 render product 绑定当前 `camera_optical/RenderCamera`，
1024×1024、64°×64°真实渲染经同射线重采样得到 640×480、64°H×50°V。
外部参考录像单独渲染，1280×720，与传感器使用相同的 130 个关节状态和回放时刻。
只有外部诊断相机可使用 look-at。

## 复现

在已授权、现有隔离 Isaac 环境中执行（路径和哈希必须绑定实际输入）：

```bash
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y OMNI_KIT_ALLOW_ROOT=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
"$P" scripts/isaacsim_visualize_eco65_j1.py \
  --configuration "$BOUND_CONFIGURATION" \
  --scene "$CURRENT_CAMERA_USD" --scene-sha256 "$CAMERA_USD_SHA256" \
  --commit "$SOURCE_COMMIT" --output "$NEW_OUTPUT"
"$P" tools/report_eco65_j1_visualization.py "$NEW_OUTPUT"
```

`--layout-only` 只生成五张布局原图及 USD 读回，不依赖录像或完整轨迹。
历史结果不覆盖。标注脚本投影实际读回坐标，原图保留。
结果目录包含布局 JSON/Markdown、相机参数、原始 PNG、四个停留关键帧、
逐帧 CSV/矩阵、MP4 解码检查和文件哈希。

## 本轮实测范围

来源基线 `8a5327352a3624fbde9afd03006b07d92d2bd968`，当前三列四层共 12 箱，
顶板关闭，保留当前连接座、实际相机及临时探针/标定板。
计算得到 q_face = −86.597127906°，q_away = +3.402872094°。
相机保持水平，其光心高于箱堆顶部，实际首帧只覆盖箱堆上部。
这不代表完整箱堆视觉覆盖，也不代表运动安全通过。

检查覆盖 J2～J6 恒定、相机相对 link_1 恒定、所有连杆 FK 只应用一次、
固定物体位姿恒定、水平光轴与正立图像、返回姿态、重复停留图像相似性、
转动中真实图像更新、视频完整解码/分辨率/帧数/时长。
不把 AABB 重叠当成穿透，不运行新的完整碰撞资格检查。

私有场景与图像留在忽略目录，实际命令、分项耗时和人工画面检查记录随交付保存。
两个针对性回归验证偏心光心求角及限速下的四停留时序，
并不替代真实 Isaac 采图与 MP4 解码。

Isaac Sim 6.0.1.0 实际完成录像，两个 MP4 均完整解码 130 帧，时长 26 秒。
录像运行含布局渲染和检查约 202.55 秒，含编码/解码共 206.58 秒。
相机/link_1 最大矩阵误差 2.22e-16；FK、固定物体、局部安装变换和 J2～J6
变化读回均为零。两次 face/away 图像的平均 RGB 差分别为 2.01/0.98 个灰度值。
安装近景首次裁切了相机顶部，随后仅改变外部观察相机，另行静态渲染；
修正版布局与原录像的场景身份、固定物体和 q_face 逐字段一致。原录像未重录，
初版近景和实际录制源码保留在私有诊断目录。


## 2026-10-17 follow-up

The explicit 30? installation, supported board, bracket repair and fresh calibration
are recorded in [eco65_tilted_camera_touch.md](eco65_tilted_camera_touch.md).
The measurements and failures above remain historical evidence for the older installation.
