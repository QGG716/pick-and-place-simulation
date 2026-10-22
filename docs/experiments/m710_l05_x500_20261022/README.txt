l05_c03：额外沿 -X 后退100 mm，完整抓放规划 1/1。
累计安装偏移 -X 500 mm、-Y 100 mm；本轮Y不变。
基座世界坐标 [-1.825, 0.25, 0.60] m。
打开 index.html 看报告，replay.html 看本次完整原生轨迹。

实际源码提交：e0c33a6b599eff6493e320b9f7242e927b1e7834
原生worker SHA256：0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
源码快照：measured-source.tar.gz；精确文件哈希：source-manifest.json。
所有原始记录（包括启动错误）：raw-records.tar.xz。
环境恢复、部署遗漏和修复：setup-records.tar.gz、costs.json。
复现命令（在已恢复并校验的这台GPU上，须使用尚不存在的输出目录）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /bin/bash /formal.sh /reproduce-l05-x500
底层入口及参数见 formal.sh，seed=71070，任务/共享前缀预算300/90秒。

本次1个业务候选，3次数值IK成功，5个原生阶段请求成功。
接触几何有效/命令/接触杯均为48/72；ideal_independent_cups承载假设。
delta=0；接收高度1.45 m，ALREADY_AT_RECEIVE；无需升降带。
完整路线：PTP接近→LIN接触→实际末态附着→LIN抽离→接收区域验收→同位姿解除附着→50 mm名义下落→名义首次触带→LIN工具分离→IDEAL_RECEIVER_DRAIN。
机器人在名义下落期间保持释放q；历史11箱单列，本次现场29→28箱。
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
未推送远端；结果提交SHA见交付消息。
