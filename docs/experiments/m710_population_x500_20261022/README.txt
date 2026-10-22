累计−X500 mm、−Y100 mm布局，全新40箱连续规划：20/40，50.0%，尚未卸完。
实际搜索25箱，规划未完成5箱，承重未搜索15箱；现场40→20箱。
实测源码：66604f4fd4a346be8d1947f3d3d44c544a5ad407
完整关节限制保留；历史单箱成功轨迹/附着不注入。
打开index.html查看报告，accepted-cycles.html查看本轮原生轨迹。
formal-raw.tar.xz保留全部原始证据字节与SHA256；measured-source.tar.gz包括本轮源码与同源模型。
在本次既有隔离环境中复现（输出目录必须尚不存在）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /bin/bash /population-x500-20261022/formal-v1.sh /reproduce-population-x500
入口与全部参数见formal-v1.sh；普通worker.sh和ROS环境沿用前一轮已恢复的固定环境，无依赖升级。
浏览器核验记录：browser-checks.json；运行源码/环境核验：source-check.json。
本轮只是规划。名义下落、名义接收与理想清空不属于物理执行。
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
最终交付提交SHA及推送状态见交付答复；此文件不自引用其提交SHA。
