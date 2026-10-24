l00_c00、c01、c02、c03顶吸：完整抓放0/4，成功率0%。四箱均停在接近目标IK，数值成功0；没有本次碰撞拒绝见证。
delta=0；横带固定250 mm（当前最低带高，不是l00 Hmax）；没有升降同步动作。预测有效杯/命令杯均48，实际接触杯未知，因为没有到达contact。继续采用ideal_independent_cups吸力足够假设，未加承载杯数门槛。

l00_c00：447次原生IK请求，90.519988秒；预测/命令48/48杯，实际contact未到达。
l00_c01：464次原生IK请求，90.628388秒；预测/命令48/48杯，实际contact未到达。
l00_c02：456次原生IK请求，90.640919秒；预测/命令48/48杯，实际contact未到达。
l00_c03：445次原生IK请求，90.602988秒；预测/命令48/48杯，实际contact未到达。

沿用总加长200 mm连接件（两连接板140→340 mm）、固定法兰、202实体/72物理杯。机械臂基座[-1.825,.25,.6] m（累计-X500 mm、-Y100 mm）、0.60 m纵带、0.26 m横带剪叉中心距、low_profile_250_v1几何不变。请求路线先+Z再-X，但四箱均未到达任何机器人运动。
保留现有机器人/安装底座权限；工具/chassis豁免仍仅限原l01，未扩展到l00。四箱的正式起点检查均通过；不能把本次IK失败归因于工具/chassis、剪叉、左壁或上一轮碰撞。
每箱一个原始顶面业务目标，按c00→c01→c02→c03派发，seed=71070，任务300秒/共享前缀90秒。每次从当时真实已接受根状态新建原生worker；只读取已核验历史末态，不导入旧c04/l00成功q、轨迹、附着或树。四箱都失败，未改变已提交状态，历史34箱单列，现场仍6箱。l00_c04及其上方l01_c04未搜索、未移除。
有限预算内没有得到合法数值构型，不是全局不可达证明。目标碰撞、接近轨迹、contact、+Z提起、-X转运、接收区、同位姿解除附着、名义下落、首次触带、工具分离和IDEAL_RECEIVER_DRAIN均未到达。本轮没有物理执行。
全部正常任务成本合计362.392283秒；调度循环363.017531秒；1812次IK请求，四个实际派发业务候选。数值求解、候选、任务和循环时间相互嵌套，不可相加。KDL内部迭代数不可观测，seed控制OMPL，不控制KDL内部随机重启；不将IK请求数当业务候选数或内部迭代数。未搜索不同抓法、偏移、高度或更改机构。

实测源码SHA：0398dc4756df9b97ecbc855fb3fe7f89e26051c9。
正式运行无集成异常。GPU针对性44项通过（7.30秒）；本地11通过、3因缺生产依赖跳过，GPU已补齐。原生接收及两种过程策略检查通过。复用与本轮ros2源码逐项散列相同的48dcea934c7eb909673561ba7a3a2fc62e471261原生构建。source-check、原始文件散列及保存的世界/父链/失败保留验证通过。
数值候选q2/上臂仰角及实际contact构型不存在；result.json中的root_configuration仅为真实起点，不能冒充求得的候选。原始记录保留每次初值及响应。启动q2和上臂仰角分别由正式PinocchioHppFclBackend及J2/J3结构点记录。
三维analysis-v1/top-ik.html复用原查看器：机器人显示本次真实起点，青色目标框显示请求TCP，静态目标未连成虚假轨迹。没有机器人运动、名义下落或理想清空回放。可离线直接打开。完整请求、失败、输入、杯掩码与日志在formal-v1-raw.tar.xz；逐文件散列已验证，共55份。

原GPU保留环境复现（输出目录必须不存在）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /bin/bash /l00-top-20261024/formal-v1.sh /l00-top-20261024/reproduction-new
完整CLI见reproduce/formal-v1.sh；只读取/l01-hmax-v2-20261024/formal-v1的固定34箱已接受末态。源码/资产包本地artifacts/m710_l00_top_20261024/measured-source.tar.gz、GPU父目录l00-top-source.tar.gz，散列见reproduce/source-receipt.json。
最终交付提交：git log -1 --format=%H -- docs/experiments/m710_l00_top_20261024。它与实测源码分列；最终推送状态见交付答复。
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
