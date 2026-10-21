**两箱重跑均未完成，完整抓放为 0/2。已确认：本次固定正面朝向、保持箱体 Y/Z、沿 −X 到接收区的所有已派发终点，都没有满足官方 J2/J3 限位的肩肘解支。** 接近与 contact 均通过；停在抽离终点 IK，尚未生成抽离轨迹。两箱均为 delta=0、48 个命令杯及 contact 几何有效杯，采用既有 `ideal_independent_cups` 承载假设；没有物理接触测量或物理执行。

| 箱号 | 本次接收带高 / 同步 | 抽离终点请求 | 完整抓放 | 任务耗时 |
|---|---|---:|---:|---:|
| l05_c03 | Hmax=1.45 m，`ALREADY_AT_RECEIVE` | 349 | 0/1 | 107.279567 s |
| l04_c02 | Hmax=1.15 m，`ALREADY_AT_RECEIVE` | 354 | 0/1 | 108.463449 s |

每箱从原实验该任务的合法历史根状态独立重启，保持根 q、场景、带高和台账；清除旧 native 父链凭据，创建新 native Task。旧 contact q、轨迹和 attachment 没有作为求解输入。本次独立生成的 contact q 恰好与旧记录逐位相同。两次重跑不能拼接成新的连续卸箱结果，原实验 17/40 计数不变。

保持安装平移 −X/−Y 各 100 mm、0.60 m 纵带、0.26 m 横带剪叉中心距、seed=71070、300 s 任务预算及 90 s 阶段预算。沿用正式 `PinocchioHppFclBackend`、原生 MoveIt/Pilz/KDL 和全机构碰撞规则。未改限位、抓法、工具姿态、箱体初态或依赖。正式运行没有生产接口异常。

最短抽离请求需要箱体中心由 X=0.30 m 到 **X=−0.5102 m**，距离 **0.8102 m**。对应实际 TCP 的腕心 X 约为 **−1.2027 m**；基座世界原点为 [−1.425, 0.250, 0.600] m。保持前吸朝向时，腕部需要回收到靠近肩部的位置，所需折臂超出关节范围。以下是该终点的一个精确逆解分支；官方 **J2∈[−75°,150°]、J3∈[−80.5°,205°]**：

| 箱号 | 所需 J2 | 所需 J3 | 具体限位冲突 |
|---|---:|---:|---|
| l05_c03 | −79.33874° | −71.01879° | J2 比下限低 4.33874° |
| l04_c02 | −77.61567° | −87.23163° | J2 比下限低 2.61567°；J3 比下限低 6.73163° |

结论不依赖这一支数值解。对 **703 次实际终点请求**，检查两种肩部方向和两种肘部方向，并穷举在官方有限范围内可能合法的 2π 等价角；**每个终点的四个肩肘分支都不合法**。最短终点的完整分支如下，角度采用原始诊断记录中的周期代表值：

| 箱号 | 肩 / 肘分支 | J1 / J2 / J3（度） | 无合法周期等价的关节 |
|---|---|---|---|
| l05_c03 | + / + | 37.40627 / −79.33874 / −71.01879 | J2 |
| l05_c03 | + / − | 37.40627 / 109.31597 / 232.53898 | J3 |
| l05_c03 | − / + | −142.59373 / −126.60027 / −61.66231 | J2 |
| l05_c03 | − / − | −142.59373 / 43.49959 / 223.18251 | J3 |
| l04_c02 | + / + | −48.35651 / −77.61567 / −87.23163 | J2、J3 |
| l04_c02 | + / − | −48.35651 / 167.47314 / −111.24817 | J2、J3 |
| l04_c02 | − / + | 131.64349 / 197.75453 / −69.95548 | J2 |
| l04_c02 | − / − | 131.64349 / 24.03970 / 231.47568 | J3 |

逆几何使用正式模型的关节轴、结构点及当前 native 请求中的 TCP 变换：肩偏置 0.150 m、上臂 0.895 m、肘偏置 0.170 m、前臂 1.045 m、法兰至腕心偏移 0.175 m。没有复制或替换 FK。每一支都回送到同一个生产 Pinocchio FK 核对腕心；最大误差分别为 **1.09×10⁻¹⁵ m、1.31×10⁻¹⁵ m**。额外以本次已通过的两个 contact 姿态作正对照，均恢复其合法实际肩肘分支。两个 contact 的 native FK 与 Pinocchio FK 最大元素差分别为 2.22×10⁻¹⁶、4.44×10⁻¹⁶，未发现坐标系或结构绑定不一致。

独立数值诊断也使用现有生产 FK/Jacobian：每箱同一最短终点，以 contact、限位中心和 8 个固定 seed 数值初值调用既有核心 DLS，均未得到限位内精确解。这些有限诊断次数不是业务规划的新上限。仅用于定位边界的无界反事实计算在 23/22 次迭代后达到约 1.25×10⁻¹⁰ / 2.37×10⁻¹¹ m 位置误差，并给出上表第一分支的越界角；这些 q 没有被接受、碰撞放行或派发为运动。其 Jacobian 均为满秩，因此这两个反事实解也没有显示必须落在运动学奇异点的迹象。生产模型限位始终未修改。

本次 native 日志直接记录 `Failed to compute inverse kinematics ... of goal pose`。349/354 次抽离请求均返回 `NO_IK_SOLUTION`，轨迹点数为 0；没有可供通过抽离目标碰撞检查的目标 q，也没有本次抽离碰撞见证。不能据此说 J2 碰剪叉或工具撞箱。抽离区域搜索在 90.153 / 90.020 s 后以 `STAGE_BUDGET_EXHAUSTED` 停止；预算耗尽是程序停止条件，关节限位是已派发终点失败的具体原因。

证据的范围是本次已派发的固定朝向终点。两箱各有 3 个业务工艺选项未访问，未证明所有抓法、朝向或整个连续接收区域都不可行，也没有连续扫掠证明。先前“可能只是接触解支不佳、换初值就能通过”的推测应据此收窄：**只更换求解初值或肩肘分支，无法让本次这些相同终点合法**；需要另行验证能改变终点腕心或姿态的合法工艺。本轮没有扩权或重设计工艺。

正式两次任务墙钟合计 **215.743016 s**。每箱原生请求、各阶段时间及求解次数见 [verification.json](experiments/m710_extraction_ik_diagnosis_20261021/verification.json)；任务、候选、区域搜索和 IPC 时间互相包含，不能叠加。只读探针记录的 DLS 计算时间合计 2.260377 s，带正对照的全部肩肘分支检查耗时 6.410809 s，均不计为规划成功或新业务候选；探针初始化、归档及初版分支检查未单独计时。初版诊断辅助脚本因缺少 SciPy 在入口失败；保留该脚本和日志，改用已有 NumPy/核心 DLS 后完成诊断，未安装依赖。它与两次正常完成搜索的生产运行分开存放。

实测核心源码为 **`f37448f998d0e82401c096a72c198b28e58fd2c2`**，重跑入口提交为 **`20bd4ebc173c09427d531d9160afe199e6d1134b`**。运行前逐文件核对核心 source manifest。worker SHA256 为 `0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a`。本轮提交只增加重跑、只读诊断及交付记录；诊断脚本精确字节哈希见 verification.json，最终提交 SHA 和推送状态见交付答复。

[全部原始记录](experiments/m710_extraction_ik_diagnosis_20261021/raw-records.tar.xz)（SHA256 `3165f0cc09cf5092945c2213ef2a7e1658e979811d7d4cc893efa0efbcceef10`）保留两次输入、native 请求/响应、失败日志、实际源码副本、初版诊断失败及修复版本、全部 703 个终点分支和数值探针。下载后核验归档及内部 44 个文件的 SHA256/长度。可直接阅读 [分支摘要](experiments/m710_extraction_ik_diagnosis_20261021/arm-branches-summary.json) 和 [原始文件清单](experiments/m710_extraction_ik_diagnosis_20261021/raw-manifest.json)。

复现使用同一授权服务器的既有隔离环境；以下 `--output` 必须是未存在的新目录。`/xy-top-formal-v1` 为原实验输入，可由上一轮 `formal-raw.tar.xz` 中的 `formal-v1` 目录恢复；本轮归档解压得到 `/extraction-diagnostic-20261021`。核心和模型快照沿用上一轮 `measured-runtime.tar.gz`。未将旧接触解或轨迹作为解算输入。

```bash
chroot /root/autodl-tmp/m710-mount-20261021/rootfs /bin/bash
export PYTHONPATH=/work-xy-top-f37448f/src PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export M710_MOVEIT_ASSET_ROOT=/work-xy-top-f37448f
/opt/planning-venv/bin/python /extraction-diagnostic-20261021/rerun.py \
  --core-root /work-xy-top-f37448f --previous-run /xy-top-formal-v1 \
  --output /extraction-rerun-reproduction \
  --wrapper-commit 20bd4ebc173c09427d531d9160afe199e6d1134b
/opt/planning-venv/bin/python /extraction-diagnostic-20261021/probe.py \
  --core-root /work-xy-top-f37448f --previous-run /extraction-rerun-reproduction \
  --output /extraction-probe-reproduction
/opt/planning-venv/bin/python /extraction-diagnostic-20261021/branches.py \
  --core-root /work-xy-top-f37448f --run /extraction-rerun-reproduction \
  --output /extraction-branches-reproduction
```

已有[五箱三维查看器](experiments/m710_five_failures_20261021/index.html)继续保留旧实测记录（每箱 346 个请求），没有混写成此次 349/354 次，也没有把越界反事实 q 伪装成执行轨迹。此次没有成功抽离运动或释放后缀可回放。实际 contact 杯几何仅为规划姿态下的几何复核，理想承载、名义下落和理想清空均不代表物理测量；此次没有到达下落或清空阶段。

```text
qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
```
