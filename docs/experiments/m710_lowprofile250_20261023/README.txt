250 mm薄型剪叉与安装底座碰撞权限：本轮完成 2/12；历史28箱单列，累计 30/40，剩余 10 箱。
成功箱l02_c01、l02_c00：均为正吸，delta=0，实际接收高度250 mm；接触前已在接收高度，ALREADY_AT_RECEIVE。预测/命令/实际contact均48杯，采用理想独立杯吸力足够假设。
两箱都走完降带/接近/contact/实际附着/-X抽离/完整接收区域/同姿态解附着/名义下落/首次触带/工具分离/IDEAL_RECEIVER_DRAIN。
实测源码：f17d636ed547a86389150f268445d44bae6f58c4。最终交付提交与推送状态见聊天/git记录。

策略与边界
从原先28箱已接受末态重新建立原生任务根，首箱l02_c01。历史末态q、实际550 mm带高和剩余12箱是合法输入；不注入历史成功候选、轨迹或attachment，不复用旧原生父链。
上一轮选层顶吸的0/12失败保留于 ../m710_layer_top_20261023/，没有覆盖；它没有改变上述已接受末态。
取消按l02层开放顶吸。只按冻结原始箱顶 z_bottom+height < 0.250 m 判定；等于250 mm也不允许。现有最低层箱顶300 mm，本轮没有任何箱体获得顶吸权限。
保留原有正面/侧面、Hmin/Hmax规则；允许顶吸时仍是先+Z再-X。未新增接触偏移权限，原始右端授权及中心delta=0保留。
机械臂累计安装偏移-X500 mm、-Y100 mm，世界基座[-1.825,.25,.6]；纵带宽600 mm，横带剪叉中心距260 mm。
新增且仅新增官方物理连杆base_link、J1_link..J6_link与精确世界对象chassis的碰撞接受权限。工具、杯、箱体、其他环境、剪叉及机器人自碰撞仍检查。Python精确校验和原生ACM使用同一策略；Isaac适配器按实际碰撞体归属接受此对象对并保留物理响应，本轮没有运行Isaac。
所有关节位置、速度、加速度、减速度检查保留；seed71070，单箱300 s，90 s共享前缀；未更换规划器或增加固定尝试次数上限。
理想独立杯吸力足够假设保留，无新增最低承载杯数门槛。实际contact重新检查命令杯全环封闭，保留真实偏心附着。

机构调整
新增独立配置 min250_lowprofile，旧min250配置不覆盖。横带两级1.25 m杆长、纵带四级0.60 m杆长保留。
横带：下铰点40 mm，杆截面垂向20 mm、导轨厚20 mm、上铰点距带底36 mm；250 mm带高时上铰点94 mm，单级升高27 mm，脚板实体高度10 mm，脚底仍在原地板。
纵带：下铰点40 mm，杆垂向20 mm、导轨20 mm；后端固定滑轨中心由400降到340 mm，滑座高度由80增至150 mm，使向下倾斜的纵带保持净空且连接连续。
没有删除任何实体、扩大装配接触豁免或降低5 mm规则；CPU、原生世界和回放使用同源几何。
第一次几何草案保留的20 mm横带杆配40 mm上端间距只有4.797 mm最小非配合销/杆间隙，纵带原25 mm杆和55 mm下铰点也不足5 mm；原后滑轨还与下倾纵带相交。随后调整为上述最终尺寸，保留全部检查。
250..2100 mm每10 mm共186个样本，机构与固定环境离散检查全部通过，耗时49.446573 s；这不证明连续扫掠，不替代正式运行中的机器人和实际箱体检查。

结果与成本
新增完整接受2/12（16.7%）；历史28箱单列，累计30/40（75.0%）。失败箱及其支撑依赖仍在世界中。
累计值包含以前布局下已接受的28箱，不是把新剪叉策略从40箱初态重新测得的独立成功率。
数值IK成功12；显式目标检查通过7。逐阶段、每个真实候选及全部原生尝试见all-targets.json和records/verification.json，数值IK次数不是业务候选数。
任务环路349.703321 s，嵌套单箱任务合计343.280188 s，二者不相加；原生请求301，IK请求15。
实际进入目标搜索的业务候选5，初始化worker的候选6；七箱得到终结记录，另五箱因上方支撑依赖未搜索。
l01_c02/c03/c04均已完成接近、实际contact几何48杯和attachment检查，分别尝试64/66/102个接收区域端点；抽离LIN/区域搜索在阶段预算内未完成，不是程序异常或全局不可达结论。
l01_c02/c03的本次见证为tool_rigid_13/11与chassis；l01_c04为tool_rigid_13与chassis以及J2_link与trailer_left_wall。这些对象对都没有获得新增豁免。
l01_c01的Hmin=-50 mm低于250 mm；l01_c00正吸同样越界，250 mm右内侧吸的业务面被保留箱体遮挡。l00_c00..c04分别仍支撑l01_c00..c04，不强行移除。
候选的delta、临时高度、同步、q2、上臂角、杯掩码及碰撞见证保留在verification/原始记录。不能把未搜索目标或旧静态预览算作本次规划结果。
仅完整接收区域、同姿态解除附着、名义下落、首次触带、工具分离、显式IDEAL_RECEIVER_DRAIN全链通过才计数；回放中名义下落和理想清空不是物理执行。

验证与复现
本地35项针对性测试通过，另38项原有剪叉/顶吸/续跑测试通过。GPU测试及新原生策略测试结果见records/日志。初次新测试把StartHeight写成dict导致单个测试失败，已修正测试数据类型，不影响正式运行。
首次生产贯通检查源码ab491d2314e5b08eff6a2f3f84b469bb25a215ae因新approval_basis没有加入布局数值审计映射而中断；未开始正式规划。保留production-check.log和integration-v1源码/清单，补入明确安装位置和600 mm纵带的既有数值后再次检查并完整运行。没有放宽审计阈值，新增了10 mm错误安装必须被拒绝的回归。
新worker在ab491d2编译；修复后的全部原生源码逐文件哈希与它一致，因此复用该二进制，见records/native-build-reuse.json。历史旧worker没有覆盖。
正式入口实际PinocchioHppFclBackend检查保留初态；源码及native worker/trace库哈希见source-check.json、source-receipt.json、records/native-binary-sha256.txt。
固定源码打包：python docs/experiments/m710_lowprofile250_20261023/reproduce/prepare-source.py（复用已保留且哈希核验的min250资产源包；不升级依赖）。
GPU现有rootfs：/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
chroot内复现：bash /lowprofile250-20261023/formal-after-audit-repair.sh /lowprofile250-20261023/reproduction-new
完整参数和新隔离编译过程见reproduce/formal-v1.sh与deploy-and-run.sh。输出目录必须不存在。
输入：/min250-20261023/formal-v2；逐份前序摘要、协议、验证和任务result固定哈希校验，仅恢复接受末态。
formal-v1-raw.tar.xz含所有失败与成功的原始请求、模型、关节轨迹、父链、检查、日志和文件哈希，不混入历史失败。
只读统计脚本初版对缺失的杯掩码求和报错，修为缺失仍标null；统计版本单列analyze-v2。原生验收在统计前已通过，未因此重跑规划或改变轨迹。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
