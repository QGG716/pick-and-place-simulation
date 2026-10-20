# J2／剪叉／铰销三维失败对比

打开 **[j2-scissor-comparison.html](j2-scissor-comparison.html)**，或解压 `offline-viewer.zip` 后双击该文件。脚本、官方机器人网格、工具及场景均内嵌，不需要联网或重新规划。

- 默认显示真实构型 #11（带面 1.70 m）的左排上层交叉铰销，固定同一 q 并排比较 460、300、260 mm 支撑中心距。
- 绿色、黄色、红色分别表示原成对诊断的表面距离通过、5 mm 间隙不足、接触／相交；颜色不是测得的接触点。
- 浅紫线是官方 J2 三角网格在所选部件中心高度处的水平截面。默认俯视；拖动可旋转三维场景，或切换“碰撞局部”“装配全景”。
- “聚焦部件”可选左上层臂 A/B、交叉铰销、右上层臂 B。可显示所有失败部件、装配背景，或单独放大一档宽度。
- 包含全部 40 个 J2／剪叉失败请求；“仅看 300 mm 新增铰销接触”筛选出其中 30 个。切换构型是静态展示，不插值为运动。

默认例子的基线记录为 **网格表面距 43.298 mm**，300、260 mm 记录为接触／相交。截面中，基线铰销投影也位于 J2 轮廓内；表面距离通过不能解释成实体内含已经排除。页面保留原始测量和真实网格，截面只画边界，不填造实心、不反推接触点，也不把负距离当作穿透深度。实体内含与碰撞网格内部的物理解释需要独立核查；本次没有修改任何既有判定或规划门禁。

来源为 [上一轮缩宽诊断](../m710_transverse_scissor_20261020/README.md) 和此前的 width060 失败目标。新导出器逐文件验证哈希，按 stage ID 绑定原失败 q；三档机构由同一 `coupled_conveyor_state` 生成。基线世界及 260 mm 世界还分别与实际原生请求中的完整场景核对，容差 1e-12。300 mm 场景绑定其已冻结布局 fingerprint。

40 个 q 的 J2、法兰位姿由 CPU URDF 独立计算并与浏览器 FK 核对；浏览器检查覆盖 40 构型 × 4 类部件 = 160 项，包括三档实体位姿、原始距离/判定、截面高度、筛选和单独显示。`browser-checks.json` 绑定最终 HTML SHA256。

本次只增加查看器和显示检查，**未重新求 IK、规划、Isaac 或物理执行**。源数据原有 0/1、37 箱状态保持。

```bash
python tools/export_m710_j2_scissor_comparison.py \
  --output docs/experiments/m710_j2_scissor_viewer_20261020/j2-scissor-comparison.html
node tools/check_m710_j2_scissor_viewer.mjs \
  docs/experiments/m710_j2_scissor_viewer_20261020/j2-scissor-comparison.html \
  /new/browser-checks /path/to/chrome --hardware-rendering
```

导出只需要核心 NumPy/PyYAML 和已保存证据，不需要 ROS、GPU 或 Pinocchio。`j2-scissor-comparison.checks.json` 保留输入哈希和显示几何核对结果；`artifact-integrity.json` 保留本目录文件哈希。FANUC 与 Three.js 许可证随 ZIP 提供。
