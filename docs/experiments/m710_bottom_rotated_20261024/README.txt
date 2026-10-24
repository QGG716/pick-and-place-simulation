本轮三箱完整抓放：3/3。累计已接受 39/40，现场只剩 l00_c00（本轮未重试）。
delta=0；横带全程固定 250 mm，进入任务时已在接收高度，无升降同步；l01 使用冻结 Hmax=250 mm。
l00_c03 顶吸绕世界 +Z 旋转 +90°，实际有效/命令/几何合格杯均为 72；l01_c04 与 l00_c04 顶吸均为 48。
承载采用 ideal_independent_cups 理想假设；没有增加杯数承载门槛，也没有删除物理杯或刚性工具实体。

实际路线及结果
1. l00_c03：+90° 居中顶吸 → +Z 300 mm → −X 810.20 mm → 完整接收区域 → 同位姿解除附着 → 名义下落约 50 mm → 首次名义触带 → 工具分离 → IDEAL_RECEIVER_DRAIN。完整通过，7.132 s 规划任务耗时。−90° 未搜索，因为 +90° 已完整成功。
2. l01_c04：原居中顶吸 → +Z 50 mm → −X 810.20 mm → 同位姿释放与完整后缀；名义下落约 100 mm。完整通过，7.575 s。
3. l00_c04：仅在 l01_c04 真正完成并移除后派发；居中顶吸 → +Z 300 mm → −X 810.20 mm → 同位姿释放与完整后缀；名义下落约 50 mm。完整通过，7.491 s。

l01_c04 三个吸面均已实际尝试
顶吸：上述连续任务中通过，48 杯。
侧吸：从与顶吸相同的已完成 37 箱状态独立重建原生根；y− 面，先 −Y 再 −X，72 杯，完整通过，19.314 s。其结果不追加到主连续账本。
正吸：从同一 37 箱状态独立重建原生根；x− 面，直接 −X，48 杯。接近/contact 通过，附着后的抽离失败，101.452 s。
正吸实际派发 156 个不同接收端点、156 个原生 LIN 抽离请求，均失败；抽离阶段保存 1248 条 J2_link / trailer_left_wall 见证。首个净空 4.659950 mm，要求 5 mm；后续构型出现 INTERSECTION_OR_CONTACT。值 −1 是标记，不是 1 m 穿透深度。
搜索抽离距离范围 0.810200–0.889800 m，原箱 Y 不变。90 秒后缀阶段预算用尽，RECEIVER_REGION_SEARCH_INCOMPLETE；不能据此宣称整个连续接收区域或所有构型全局不可行。接近早期另有 8 条工具柔性杯/横带见证，后续接近已通过，不把它们当成最终抽离原因。

状态、输入和边界
当前布局继续是 base=[−1.625,0.250,0.600] m，较上一轮 x500 布局 +X200 mm；本轮不再移动底座。
连接件总延长 200 mm；纵带宽 0.60 m；横带剪叉中心距 0.26 m；low_profile_250_v1。
seed=71070；每箱任务 300 s；前缀/阶段共享预算 90 s。±90° 和吸面机会沿用同一个箱任务预算，非最后候选沿用 20 s 访问时限，不新增固定尝试次数上限。
三箱主流程从已提交 c01/c02 的 36 箱完成末态开始；失败方案不提交世界变化。侧吸/正吸只恢复本轮已成功的 c03 前序，不读取 c04 已成功的 q、轨迹或附着关系作为提示，也不保留旧原生树。
所有 IK、位置、关节速度/加速度/减速度、完整碰撞、密封环和实际 attachment 检查保留。l05 的速率例外不作用于本轮。机器人/安装底座例外保留，工具/chassis 例外仍只授权冻结 l01，未给 l00 扩权。

实际成本
主连续任务合计 22.198193 s，循环墙钟 23.560401 s；两项独立对比 120.765758 s。
全部五个实际任务合计 142.963951 s；249 次原生接口请求，23 次数值 IK 请求，20 次数值 IK 成功。IK 重试不是业务候选数量；所有失败请求成本已包含，不重复相加嵌套计时。
主流程 3 个完整抓放接受；独立侧吸 1 个完整对比接受；正吸 0 个。独立对比不把累计箱数从 39 再增加。
GPU 定向测试：主流程 50 passed / 8.79 s；对比启动旧校验版本 53 passed / 10.07 s，但生产启动检查失败；修复后正式入口覆盖 55 passed / 11.64 s。
唯一生产集成错误是旧剩余箱数校验，在真实启动检查阶段、规划派发之前发生。保留 integration-errors/c04-comparison-v1-preflight.tar.gz 及当时源码清单；修复后实际运行侧吸和正吸，未停留在单元测试。

源码与复现
主连续流程实测源码：a4552c363c0e2cce56be4d4b28958dd44ebbe3e8
侧吸/正吸实测源码：c3b815d498684ce75e22717389bf9507205b90dd
保留的启动错误源码：dea9b2bce8b5fc770efc48db703b8d0cf6ba0a3e
最终交付 SHA 与推送验证在本轮答复中给出；它不同于实测源码 SHA。
GPU 已存在隔离 rootfs、固定环境与 native worker，未升级依赖、未重新构建未变化的 native 代码。
复现主流程（新输出目录）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /bin/bash /bottom-rotated-20261024/formal-v1.sh /bottom-rotated-reproduction
复现侧吸/正吸（独立新输出目录）：
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /usr/bin/env C04_FACE=side /bin/bash /current-c04-comparison-v2-20261024/formal-v1.sh /current-c04-side-reproduction
chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs /usr/bin/env C04_FACE=front /bin/bash /current-c04-comparison-v2-20261024/formal-v1.sh /current-c04-front-reproduction
每条脚本固定对应源码清单、原生二进制、先前输入，输出已存在时会拒绝覆盖。源码打包资产散列见 reproduce/*/source-receipt.json。

证据与离线查看
index.html 为入口。analysis-v1/accepted-cycles.html 回放主连续 3 箱；comparison/analysis-side/accepted-cycle.html 是独立侧吸；comparison/analysis-front/failure.html 是本轮真实被拒绝的抽离搜索构型，无虚构运动。
formal-v1-raw.tar.xz、comparison/side-raw.tar.xz、comparison/front-raw.tar.xz 各自带逐文件 SHA256 清单；分析中逐一验证。完整候选 q、q2、上臂仰角、预测/命令/实际杯、原生请求和失败见证分别保留在各 analysis JSON、verification JSON 和原始归档。
主回放 126.406082 s 原生机器人轨迹 + 0.344712 s 名义下落 + 5.2 s 事件展示，机构演示 0 s。显示时间不是硬件周期，所有机器人运动由本次原生 q 驱动。
offline-replay.zip 可解压离线打开 index.html。名义下落和 IDEAL_RECEIVER_DRAIN 是显式理想假设，实测物理接收为 0。
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
