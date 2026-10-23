本轮尚未实际续跑：GPU端口拒绝连接。代码、20箱续跑初态和传输包已准备。
本轮成功率：未测；原生请求0次。历史20/40单列，不计为本轮完成。
准备源码：a300d234633e122c6d8328dddb259f3c9c38ad91
最终交付提交见聊天回复；本轮未推送。

已实现
1. 新配置明确请求横带最低表面世界高度0.250 m，保留最高2.100 m。
2. 本轮仅在冻结箱顶世界Z严格小于0.250 m时使用顶吸；等于阈值不触发。
   历史Hmin判定显式保留供旧实验复现，新入口不会使用该旧判定。
3. 保留累计−X500 mm、−Y100 mm安装位置，纵带0.60 m宽、剪叉0.26 m间距。
   底盘、机器人安装高度、纵带后端0.60 m支撑、剪叉尺寸及碰撞体均保留。
4. 验证前序全部25箱结果的根状态、哈希、20次完整抓放提交和5次失败保留。
   新worker根状态只恢复合法末态q、带高1.15 m、剩余20箱及历史台账；旧原生父链清空。
   首箱明确为carton_l03_c02；全关节速度/加速度/减速度检查保留。
5. 已知剪叉高度构造失败单列为原生搜索前的工艺几何拒绝，不能计为IK失败。
   其他真实异常仍中断并保留；碰撞、实际接触、附着、完整释放后缀检查不绕过。

实际几何发现
当前箱高300 mm，最底层箱顶也是300 mm，所以剩余20箱均不触发250 mm顶吸规则。
l03_c02箱顶1200 mm；Hmin=550 mm、Hmax=850 mm，恢复既有正面/侧面候选。
550、850、1150 mm处的全机构几何检查通过；这些是静态机构检查，不是机器人规划通过。
250 mm处无法构造现有剪叉：上铰点Z=250−120−70=60 mm，下铰点Z=100 mm。
两级剪叉每级升高−20 mm，而实体杆截面要求每级升高严格大于35 mm。
仅此几何条件就要求横带表面高于360 mm；不代表360 mm以上已完成全行程或机器人碰撞验证。
配置250 mm并不使机构自动具备该行程。程序保留这个拒绝，不移动支架、缩杆或删碰撞体制造通过。

本地验证
31 passed, 9 deselected in 3.75 s；9项是历史全行程扫掠，不是本轮单点条件。
此前22项针对性测试也通过；额外启动的历史全行程测试被停止，未作为整组通过计数。
冻结源包的文件集合与全部源码哈希已核验；前序末态用最终源码再次恢复并对比一致。
这些检查未替代正式PinocchioHppFclBackend初态检查或MoveIt原生完整抓放；两者均待GPU恢复。
2026-10-23 02:52:40 UTC，对connect.weste.seetacloud.com:37642的TCP连接被拒绝，尚未进入SSH认证。
见connectivity.json、delivery-status.json、preparation.json及predecessor-check.json。
没有生成新的卸箱回放；上一轮顶吸失败查看器仍为历史结果。

复现/续跑
在仓库根目录，使用现有含numpy的Python；不升级依赖：
  PYTHONPATH=src:tools python tools/prepare_m710_min250_continuation.py --previous docs/experiments/m710_population_x500_20261022 --output artifacts/m710_min250_20261023/preparation-v1
  python docs/experiments/m710_min250_20261023/prepare-source.py
Windows PYTHONPATH分隔符用分号。输出目录必须不存在；当前准备目录已存在，不需重新生成。
源包与输入包目前位于artifacts/m710_min250_20261023，哈希见source-receipt.json。
输入包可从仓库已保留、哈希固定的前序formal-raw.tar.xz重新生成，未复制到本交付目录。
正式入口及全部参数见formal-v1.sh；部署入口见deploy-and-run.sh（均尚未在GPU执行）。
恢复原有隔离环境后，将三个文件上传到/root/autodl-tmp/m710-l05-x500-20261022：
  prepared-source.tar.gz -> min250-prepared-source.tar.gz
  previous-inputs.tar.gz -> min250-previous-inputs.tar.gz
  formal-v1.sh -> min250-formal-v1.sh
运行deploy-and-run.sh。它检查既有worker和trace库哈希，使用新目录，保留旧运行。
seed=71070；任务300 s，共享前缀90 s，IPC100 s；无历史候选/轨迹提示。
正式运行完成后再用verify_m710_population.py检查及导出既有回放，不能仅凭准备检查宣布完成。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
