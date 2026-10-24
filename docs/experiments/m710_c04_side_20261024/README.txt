c04 侧吸重试：0/1，未完成。
实测源码：94ff46486f4c2a929a9c46323699198a14d18af4。
本次从已卸34箱、剩余6箱的接受末态重建原生根状态。仅试c04的y-侧面，要求先-Y再-X；Hmax固定250 mm，delta=0；连接件总加长100 mm。底座X-500/Y-100 mm和其他检查不变。
面可见性通过，预测有效杯72、命令杯72；实际contact未到达。ideal_independent_cups承载假设不变。
一个业务目标实际派发443次原生IK，全部NATIVE_IK_FAILED，数值IK成功0；90秒共享前缀预算耗尽。初态检查通过，但目标碰撞、接近、contact、抽离、释放、工具分离均未通过。无本次碰撞见证；不能引用旧正吸的J2/左壁碰撞，也不是全局不可达证明。
任务90.634016879秒，循环90.807157673秒，候选访问89.967174560秒，原生IK求解合计44.465449482秒；这些时间嵌套，不能相加。448次原生请求=初始化1+FK3+初态检查1+IK443。KDL内部迭代数不可观测；seed71070控制OMPL，KDL内部RNG未受该seed控制。任务预算300秒、共享前缀90秒不变。
当前仍为历史34箱、剩余6箱：l01_c04及l00五箱。没有箱体移动、接收、理想清空或物理执行。
GPU针对性测试39项通过（5.40秒），生产PinocchioHppFclBackend初态通过，原生策略测试、保存的父链及失败保留检查通过。344份源码散列核验通过；复用48dcea934c7eb909673561ba7a3a2fc62e471261的原生二进制，ros2源码逐项相同，详见原始包build.log。
side-ik.html是静态3D记录，显示真实起点q和请求虚拟TCP；无目标解q或运动回放。查看器导出是运行后的只读处理，不是规划源码变动；side-ik-checks.json记录其散列。浏览器核验同源FK、6箱几何、无运动/附件/伪造碰撞通过。
formal-v1-raw.tar.xz包含全部原始请求、失败、实际根状态、验证和测试日志，25文件散列已核验。source-manifest.json和source-receipt.json绑定实测源码/资产。实测源码包保存在GPU本轮目录及本地artifacts/m710_c04_side_20261024/measured-source.tar.gz。
在原GPU环境复现（新输出目录必须不存在）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /bin/bash /c04-side-20261024/formal-v1.sh /c04-side-20261024/reproduction-new
固定输入为/l01-hmax-v2-20261024/formal-v1的已接受末态；不注入旧c04解或树。完整命令见formal-v1.sh。
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
最终交付提交可用 git log -1 --format=%H -- docs/experiments/m710_c04_side_20261024 定位；它与上述实测源码提交分开记录。
