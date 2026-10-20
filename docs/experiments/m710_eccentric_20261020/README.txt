本轮结果：0/1；37 箱保持不变，历史三箱另列。未物理执行。

打开 eccentric-inspection.html：
  可查看本次原生 IK 数值解、官方 J2/J3 上臂方向、面中心和全 72 杯。
  该 IK 返回后程序接口异常，尚未检查目标碰撞或接近路径。
  下拉列表中的正偏移只是未派发目标几何；不会移动机器人。

打开 checked-prefix.html 或 checked-prefix.mp4：
  仅回放已检查的 2.05→1.75 m 降带前缀，机器人保持前序 q。
  不是完成抓放的动画；没有接触、附着、抽离、释放或清空。
  视频 640x360 / 5 fps / 6.2 s；其中升降 4 s 是演示时间。
  原生机器人运动时间 0；真实硬件升降和物理周期未知。

唯一正式实测源码：
  eed289495d46b8ea88c7a7c0dc8d3a9c8ecd6dec
修复/查看器源码（126 项定向测试通过，未重跑正式任务）：
  ae08fbbf866615cc5841d494129739c26dbcba26

正式失败：
  AttributeError: 'PinocchioHppFclBackend' object has no attribute 'joint_axis_frames'
修复：
  改用两种后端共同支持、官方结构点对应的 J2_link/J3_link 世界 FK。
  补生产后端对照测试；一次正式运行限制下，没有第二次 c00 任务。

实测 delta=0，预测有效/命令启用各 48 杯；实际接触未到达。
有效杯承载足够为理想规划假设，不是实测能力。
运行预算 300 s / 共享前缀 90 s，seed=71070，0.60 m 纵带 / 0.26 m 剪叉。
全部失败及阶段耗时在 formal-raw.tar.gz 和 formal-result.json。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
