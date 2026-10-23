本轮完整抓放：0/10；实际派发5箱，另5箱因支撑关系未搜索。累计保持30/40。
只对冻结原层l01、l00开放顶吸；横带全程250 mm，delta=0。
5个已派发候选预测有效杯/命令杯均48；实际contact未到达，因此实际有效杯、contact q2及上臂仰角均未取得。
承载采用ideal_independent_cups：所选有效杯吸力足够，不添加最低杯数门槛。
最后阶段：顶吸pregrasp数值IK；未生成任何机器人运动，没有到达先+Z再−X路线。

实测源码：4337f9115d193645bfaf6122c1ca5673ebf3e7ac
继承前序：f17d636ed547a86389150f268445d44bae6f58c4的已验收30箱末态；输入 /lowprofile250-20261023/formal-v1。
终态q、10箱位置、250 mm带高及历史30箱台账逐文件哈希和逐次提交重建验证；清除所有旧原生父链。
未注入历史成功IK、轨迹或attachment。本轮新建原生根；5箱失败均未改变世界。
前序记录与本轮分别存放，历史30箱不是本轮完成数，也不是当前策略从40箱初态测得的独立成功率。

实际结果
l01_c02: 416次原生IK请求，任务90.743132554 s；1个顶吸业务候选。
l01_c01: 415次原生IK请求，任务90.860033177 s；1个顶吸业务候选。
l01_c03: 417次原生IK请求，任务90.833375059 s；1个顶吸业务候选。
l01_c00: 419次原生IK请求，任务90.847821169 s；1个顶吸业务候选。
l01_c04: 419次原生IK请求，任务90.817163333 s；1个顶吸业务候选。
l00_c00..c04各自仍支撑同列l01箱；支撑关系验证通过，未强行移除或计作IK失败。
总计原生请求2111次，其中IK2086次：2082次NATIVE_IK_FAILED、4次NATIVE_IK_REQUEST_DEADLINE。
这属于5个业务候选的数值求解恢复尝试，不是2086个业务抓法，也不是固定尝试次数上限。
IK成功0；目标碰撞检查通过0（没有进入目标检查）；接近轨迹0；contact及同步0；完整抓放0。
无本轮碰撞见证。原生结果ik_collision_checked=false，不能把旧剪叉/底座对象对当作本轮失败原因。
当前预算内未求得解不等于证明全局不可达。未搜索范围包括l00五箱、接触/抽离/释放后缀，以及预算外的其他数值构型。
本轮用户指定两层顶吸，因此不再派发旧正吸/侧吸业务面。其他层未扩权。

配置和成本
保持机器人安装位置累计−X500 mm、−Y100 mm；纵带0.60 m、横带剪叉中心距0.26 m。
沿用low_profile_250_v1同源实体；只豁免base_link及J1..J6与chassis的7组对象对，工具/载荷/环境/自碰撞原规则保留。
顶吸权限由冻结原层箱顶几何<=0.60 m建立，恰好覆盖这10箱；不按名称解析层号授权。
顶吸路线：l01先+Z 50 mm，l00先+Z 300 mm，再−X；均为尚未到达的业务路线，不是生成轨迹。
seed=71070，单箱预算300 s、共享前缀90 s、IPC100 s；全部关节位置、速度、加速度和减速度规则保留。
原生记录注明seed作用于OMPL；KDL内部RNG不受该seed控制，不虚称跨次耗时/搜索次数逐位确定。
任务循环455.057592608 s；嵌套单箱任务合计454.101525292 s，两者不能相加。
完整原始记录含各次初值、native响应和时间成本。没有程序异常，也没有失败后换seed重复搜索。

验证与查看
本地23项针对性回归通过；GPU同23项通过（records/focused-tests.log）。
正式PinocchioHppFclBackend初态检查通过，5个worker原生check_start通过。
47份原生源码与ab491d2构建版本一致，复用已校验worker/trace库；记录见native-build-reuse.json与source-check.json。
保存记录验证覆盖40箱身份、剩余支撑、父链、世界不变、统计与失败保留；见records/verification.json。
top-ik-failures.html复用原查看器与官方模型，显示5箱实际起点q和派发TCP，无虚构目标q/运动/碰撞点。
5个查看器用例均通过浏览器检查，渲染FK与生产FK一致，零运动点、零运行错误；见qa/。
没有成功周期，因此没有降带视频冒充抓放回放，也没有生成名义下落或理想清空事件。
名义下落、首次触带、工具分离和IDEAL_RECEIVER_DRAIN均未执行；物理执行NOT_RUN。
导出工具仅修复历史硬编码带高/历史计数并匹配当前底座权限，未修改实测冻结源码；导出器哈希见top-ik-failures-checks.json。

复现
现有GPU rootfs：/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
进入chroot后：bash /bottom-two-top-20261023/formal-v1.sh /bottom-two-top-20261023/reproduction-new
输出目录必须不存在；完整命令见reproduce/formal-v1.sh。需保留已验证前序/lowprofile250-20261023/formal-v1。
固定源码打包：python docs/experiments/m710_bottom_two_top_20261023/reproduce/prepare-source.py
打包脚本复用并哈希验证已有min250资产包，环境与模型不升级。部署脚本对已有目标目录拒绝覆盖。
原始记录：formal-v1-raw.tar.xz，SHA256=94731f52ac5ad5ed1d824d266fdfc6b04f72f3fa5aace4a433f8affa31ed6ba9，62份文件逐份校验通过。
source-manifest.json绑定所有实测src/tools/ros2/configs；source-receipt.json记录资产和源码包。
最终交付提交SHA与推送状态见本轮交付消息及git记录；实测源码SHA与最终证据提交分开标记。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
