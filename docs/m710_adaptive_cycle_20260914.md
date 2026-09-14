# M-710 自适应接近、释放与连续取放（2026-09-14）

本轮仅修改 `feat/v0.5-feasibility-core`，基于 `cf329a9`。固定布局、官方模型、
20 kg 工具、42.5 kg 箱体和 72 杯理想独立吸附假设保持不变。
GPU 恢复后，规划、针对性检查和 Isaac 均在服务器独立目录执行。
几何规划使用服务器 CPU，渲染使用 GPU；保留已验证的 PhysX CPU 后端设置。

## 实现

- `TRANSVERSE_SIDE` 的工作法向统一为世界 +X；新动作语义和源码指纹阻止旧计划直接复用。
- 接近可直接连接或使用自适应预抓取，两者共用实际 FK、刚性工具净空和完整杯环检查。
  IK 找到首个有效解即尝试下游，失败再继续取解。
- 脱垛保留快速直线，也搜索有限平移、转动及中间位姿；放置失败可以换脱垛出口。
  原箱垛接触许可按实际脱离状态锁存，已接收箱体始终使用普通碰撞规则。
- 支撑释放和 0–50 mm 内的短落释放使用不同门控。短落预测使用质心速度与角速度、
  重力和完整足迹，释放不修改箱体速度、质量、惯量或碰撞器。
- 两带接缝仍按真实联合支撑处理；接收点保留 10 mm 外缘余量，并检查沿选定带面的输送支撑。
- 释放后比较保守移动占用下的脱离路线和下一箱真实接近连接。
  一步前瞻最多取当前行两个下一目标、各两个接触候选；每个安全脱离终点最多搜索 8 秒。
  前瞻失败只表示该小预算未找到连接，当前合法等待位仍可保留。
- 续箱从实际 q 和全部保留箱体状态开始。上一箱占用覆盖至出口前的有界停带位置，
  接收监测持续运行；出口停带有明确事件，不重置物体速度。
- 导出保留已检查的关节路径拐点，不向几何采样点附加停顿。视频使用 1920×1080、1×物理时间。
  默认 30 fps；本轮整排尝试使用独立的 `m710id70_first_row_recording_v1.yaml`，以 15 fps 减少渲染墙钟时间。
- 续箱按任务箱身份集合及数量验证，允许同一行的末尾候选随连接成本重新排序。
  工具局部碰撞盒比较仅允许坐标往返产生的 `1e-12` 绝对数值舍入误差，身份、结构与所有其他物理输入仍严格一致；
  实测两次变换中心最大差 `7.21645e-16 m`，旋转元素最大差 `2.22045e-16`，尺寸无差异。

## 本轮物理发现

最新连续尝试使用服务器 `isaac_first_row`，首段成功，实际动作 90.433 s、Isaac 墙钟 1114.165 s，
1080p/15 fps 原始视频 1356 帧。第一箱仍是 `carton_l07_c02`；历次独立世界里的同一箱成功不累加为卸箱数量。
本次首段复用已验证的首箱几何计划（原规划 120.632 s），只重新生成与当前源码及录像配置绑定的执行文件。
此前 `isaac_continuous_verified` 也完成首箱，但第二箱尚未物理启动就被工具盒浮点比较误拒，仿真退出。
修复后，本次在独立新目录启动；同一连续运行内部保留所有 40 个刚体。

首个开发执行保存于服务器 `isaac_cycle01`：实际接触、释放及首次接收成立，
但工具脱离期间箱体偏转，最终足迹超出纵带边界，`physical_cycle_completed=false`。
该结果保留为失败证据，没有启动续箱。后续实现加入接收外缘余量、输送足迹检查、
实际接收时刻起算的输送采样窗口，以及短落候选的已验证搬运前缀复用。

随后 `isaac_final` 首箱完整通过：`physical_cycle_completed=true`，没有运行时停止或非预期机器人接触。
实际释放最低角点高度为 24.135 mm，89.2417 s 观察到真实接收，结束时全足迹支撑成立，
箱体沿纵带速度为 0.3000 m/s。释放后静止等待为 0，杯接触正常碰撞规则在释放请求后 70.8 ms 恢复。
此过程保留真实重力、摩擦、42.5 kg 箱体和原速度，没有重置载荷状态。

| 首箱结果 | 数值 |
| --- | ---: |
| 实际动作时间 | 90.433 s |
| Isaac 运行、监测和渲染墙钟 | 1230.709 s |
| 实际吸附时完整密封杯数 | 40 / 72 |
| 最大关节跟踪误差 | 0.002276 rad |
| 视频 | 1920×1080，30 fps，2713 帧，1× |
| 已交接出工作单元 | 0 |

首箱录像保存后，续箱入口暴露 `KeyError: carton_l07_c02`：暴露面查询错误地把已接收箱体传入只含剩余箱垛的支撑图。
修复后支撑图查询只使用图内身份，已接收箱体仍保留在全部碰撞物体与输送占用中。
续箱规划进程异常也不再直接关闭仍可重试的物理世界。相关 22 项针对性检查通过。
首箱成功输出及当时源码补丁独立保留，后续连续执行使用新目录。

`qualification_passed=false` 的唯一剩余项目是实际驱动输出力矩通道不可用（`NOT_EVALUATED`）。
官方有限 effort/velocity 配置及后端读回成立，但不把驱动输入或模型逆动力学值冒充实测输出力矩。
此字段与实际几何/动力学循环完成、以及真机资格分别报告。

## 同一输入的代表性比较

首箱比较使用相同源码、场景指纹 `3672163e…f68644ff`、初始 q、
`carton_l07_c02`、随机种子和 240 s 规划预算。只改变接近模式。

| 项目 | 直接接近 | 自适应预抓取 |
| --- | ---: | ---: |
| 接近连接规划墙钟 | 1.829 s | 1.853 s |
| 完整规划墙钟 | 119.542 s | 120.506 s |
| 计划动作时间 | 90.386 s | 90.386 s |
| 关节路径长度（逐段二范数和） | 5.003715 rad | 5.003715 rad |
| 控制指令数 | 4640 | 4640 |

本例两种模式找到相同几何路径。35.001 mm 末段由工具几何决定；直接模式将接近合并，
自适应模式显式记录预抓取阶段，均没有给该边界加停顿。默认选择先找到完整路径的直接模式，
不能由这一次比较声称直接模式本身更快。

同一放置搜索中，支撑释放分支返回 `PAYLOAD_COLLISION`；25 mm 和 49.6 mm 短落分支均有完整路径，
分别为 5.003715 rad 和 5.031551 rad，选中较短的 25 mm 分支。高度以实际箱体最低角点到接收面的距离复核，
使用 `release_prediction` 的几何证据；最终导出也将 `free_fall_height_m` 更新为这一复核高度。
侧向交接比较找到约 212.6 mm 的斜向离开和 30.2 mm 的近竖直离开，选中后者。
两者的一步前瞻均在各 8 s 预算内未找到下一接近连接；这不等于无 IK 解，实际续箱使用较大预算重新规划。

针对性检查在服务器执行：相关 8 个文件 84 项通过，补充的已接收箱体碰撞作用域和保留输送占用检查 2 项通过。
续箱人口排序修复的 5 项数值检查通过；真实浮点误拒的直接回归通过，并用此前两个实际导出文件确认衔接验证通过。
未运行全量测试、104/129 历史任务或 40 箱清空。

## 复现入口

在保留的服务器目录 `/root/autodl-tmp/m710-adaptive-20260914/final-repo` 内运行：

```bash
export PYTHONPATH="$PWD/src"
M710_CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
M710_ISAAC=/root/autodl-tmp/envs/isaacsim-clean/bin/python
M710_RUN="$PWD/outputs/m710_adaptive_reproduction_$(date +%Y%m%d_%H%M%S)"
"$M710_CPU" tools/run_m710_contact_unloading.py --planning-wall-time-s 240 \
  --execution-config configs/simulation/m710id70_first_row_recording_v1.yaml \
  --output "$M710_RUN/plan01" \
  --execution-bundle "$M710_RUN/plan01/replay_bundle.json"
M710_ROW_COUNT=$("$M710_CPU" -c 'import json,sys; print(json.load(open(sys.argv[1]))["statistics"]["task_count"])' "$M710_RUN/plan01/motion.json")
```

复用已有官方 USD，并验证原资产来源。复现使用新的输出目录，保留本轮原始录像。

```bash
export OMNI_KIT_ACCEPT_EULA=YES OMNI_KIT_ALLOW_ROOT=1
export XDG_RUNTIME_DIR=/root/autodl-tmp/runtime-root
export XDG_CACHE_HOME=/root/autodl-tmp/cache/xdg
export XDG_CONFIG_HOME=/root/autodl-tmp/config/xdg
export XDG_DATA_HOME=/root/autodl-tmp/data/xdg
"$M710_ISAAC" scripts/isaacsim_fanuc_replay.py \
  --bundle "$M710_RUN/plan01/replay_bundle.json" --project-root "$PWD" \
  --usd-directory "$M710_RUN/usd" \
  --reuse-usd-entrypoint /root/autodl-tmp/m710-official-dynamics-20260910/isaac_usd/m710id_70_official_8/m710id_70_official.usda \
  --reuse-usd-run-evidence /root/autodl-tmp/m710-official-dynamics-20260910/isaac_outputs/initialization_render_sync_logged_final/run_status.json \
  --reuse-usd-source-contract /root/autodl-tmp/m710-official-dynamics-20260910/repo/outputs/m710_official_dynamics_20260910_final/initialization_contract.json \
  --output "$M710_RUN/isaac" --record-video --video-preview-speed 1 \
  --continuation-dir "$M710_RUN/continuation" --maximum-segments "$M710_ROW_COUNT" --continuation-wait-seconds 3600
```

在第二个终端使用同一工作目录和变量，取得上述 Isaac 进程的 PID 并设置 `M710_ROW_PID`，
等每段实际接收通过后提交下一箱计划；物理进程退出或规划失败则停止：

```bash
for ((i=2; i<=M710_ROW_COUNT; i++)); do
  printf -v n '%03d' "$i"
  ready="$M710_RUN/continuation/segment_${n}_ready.json"
  while ! test -f "$ready"; do
    kill -0 "$M710_ROW_PID" 2>/dev/null || exit 4
    sleep 2
  done
  "$M710_CPU" tools/continue_m710_contact_unloading.py \
    --ready-file "$ready" --output "$M710_RUN/plan_${n}" \
    --execution-config configs/simulation/m710id70_first_row_recording_v1.yaml || break
done
```

离线规划期间暂停物理时间，保留同一个 Isaac World；恢复前验证场景和起点指纹。
录像的 1× 倍率以物理时间计，规划墙钟时间另列。释放、实际接收与出工作单元分别记录，
不将停在带上的箱体标为已出料。真机资格和缺失的驱动输出测量渠道保持独立记录。
