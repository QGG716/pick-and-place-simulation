# ECO65-B — NVIDIA Warehouse 静态外观场景

本轮只组合场景并渲染外观。基于 `compact_20260918_03` 的冻结初始快照与既有 HOME，未重新规划、未执行轨迹、未开展新的物理抓放；硬件禁用。物理结果为 **NOT_EVALUATED**。九箱配置、四箱历史执行证据和三来源锁均保留。

## 官方资产与实际运行环境

- Isaac Sim **6.0.1.0**，Kit **110.1.2+production.326809.f9bf0dda.gl**；使用服务器原有环境，没有升级。
- 资产根目录由运行时 `isaacsim.storage.native.get_assets_root_path()` 返回，并与已安装 `isaacsim.storage.native/config/extension.toml` 配置核对。
- [NVIDIA 官方 Warehouse USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/warehouse.usd)，SHA-256 `0b833b2c59c18832a1fb421292c256e7ca21977961f62a2b9b4e2feee86b9903`。
- [NVIDIA 官方 SM_CardBoxD_04 USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd)，SHA-256 `e35bac376af806f52c5413fbe397e587450c942201525ada5bbfca58989c52ff`。
- 共 **158 个文件、299,133,603 字节**，递归包含 USD 引用、材质、MDL 相对模块及纹理。逐文件来源和 SHA-256 保存在本地 `assets/official/manifest.json`。服务器部分官方纹理请求超时，缺失文件通过本地从相同官方 URL 获取后传入；原件字节不改动。
- 两个主资产均为 Z-up、米制。仓库原有内部对象变换保持不变，未整体缩放。

## 布局与坐标

桌面局部坐标 D 沿用紧凑四箱配置，+X 向厢内、+Y 向左、+Z 向上。机器人、真实吸具、转接架、完整厢体、两条输送带和四箱的相对位置、尺寸及初始姿态保持不变。

桌板范围 D：X `[-1.300, 0.700]`、Y `[-1.000, 1.000]` m，厚 30 mm，顶面 D-Z=0。桌子包含四条 60 mm 方形支腿和上下横向支撑，脚底落在实际仓库地面，未增加移动底盘。桌架为本轮场景设计，并非实测桌子或承载认证。

官方地面网格顶面约为原始 Z=`1.4305114426e-7` m，仅在组合层平移该微小偏差，使实际地面 W-Z=0。工作站以单一刚体平移放入空旷区，不另加偏航：

```text
T_W_D = Translation(0.320, -0.100, 0.800) m
```

| 检查对象 | 仓库世界坐标高度（m） |
|---|---:|
| 地面 | 0.000 |
| 桌面上表面 | 0.800 |
| 180 mm 固定安装座上表面 | 0.980 |
| 横向传送带上表面 | 0.980 |
| 右侧纵向传送带上表面 | 0.980 |
| 下层纸箱中心 | 0.880 |
| 上层纸箱中心 | 1.040 |
| 模拟厢体顶板内表面 | 1.450 |

实际 USD 读回误差小于 2 µm。全部设备只随工作站根变换抬高一次。桌面扩大不改变机器人、吸具或设备比例。

## 纸箱和碰撞

四箱分别引用完整官方 `SM_CardBoxD_04.usd`，保留几何拓扑、UV、材质和纹理。由实际网格包围盒测得原始尺寸约 `0.380 × 0.250 × 0.14875` m；在独立 Fit/Pivot 层中按轴适配为 `0.220 × 0.220 × 0.160` m，并以测得的包围盒中心对齐原布局中心。此项形状适配仅作用于纸箱可视资产，完整吸附轮廓及物理吸附仍未重新评估。

稳定 ID：`carton_r00_l00_c00`、`carton_r00_l00_c01`、`carton_r00_l01_c00`、`carton_r00_l01_c01`。每箱保留一个独立、精确对应现有 layout 的长方体碰撞代理；在组合层关闭该纸箱可视引用自带的重复碰撞表示。仓库原有碰撞不受此处理影响。

保留官方地面、墙体、货架、托盘与灯光；没有移除背景对象。逐个检查仓库实际网格/碰撞体与工作站实际几何的世界包围盒，包含实例内部网格。选位预留 0.40 m，保留 **781 个环境碰撞 API**。实际比较 **148 个工作站几何表示与 745 个非地面环境网格/碰撞表示**，包围盒相交数为 0；四腿脚底与地面一致。该检查是保守静态几何检查，不是动力学接触验收。

完整厢体各板保留碰撞，采用低反射、无磨砂折射的半透明展示材质；这不是实物板材光学参数的认定。补充柔和工作台拍摄补光，官方照明保留；所有增补仅在独立组合层中。

## 复现与本地证据

公开入口：

```powershell
$env:PYTHONPATH='src'
.venv/Scripts/python.exe tools/export_eco65_warehouse_workstation.py `
  --snapshot outputs/eco65_desktop_layout/compact_20260918_03/scene_snapshot.json `
  --home outputs/eco65_desktop_layout/compact_20260918_03/backend/manifest.json `
  --output outputs/eco65_desktop_warehouse/NEW_RUN
```

在既有 Isaac Python 环境中运行 `scripts/isaacsim_warehouse_assets.py --output PRIVATE_RUN/assets`，再运行：

```bash
python scripts/isaacsim_render_eco65_warehouse.py \
  --assets PRIVATE_RUN/assets \
  --workstation PRIVATE_RUN/workstation_D.usdc \
  --output PRIVATE_RUN/render_01
```

`--compose-only` 可用普通 OpenUSD Python 环境独立检查几何组合。输出目录必须是新目录。资源检查、真实 USD 读回、各高度、四箱完整尺寸及 ID、四腿落地和环境干涉为本轮针对性验证；未运行全量 pytest、整堆或性能扫描。

私有 CAD、派生几何、详细快照、USD 和图像仅在忽略目录 `outputs/eco65_desktop_warehouse/warehouse_20260918_01/` 及用户指定的 GPU 服务器任务目录中保存，不提交公开仓库。USD 使用相对引用，复制同一目录结构即可离线复现。配置与脚本均使用 warehouse 命名。

## 渲染交付

最终交付目录为本地 `outputs/eco65_desktop_warehouse/warehouse_20260918_01/render_09/`。五个视角均为服务器 Isaac RTX 实际渲染，原图 **1920×1080 PNG**：

1. `01_warehouse_overall.png`：官方西侧货架、托盘、墙体与完整工作桌同时入镜。
2. `02_workstation_oblique.png`：真实 ECO65-B、吸具、四箱、完整厢体与两条传送带。
3. `03_layout_top.png`：完整 2 m 桌面与原紧凑 layout 俯视图。
4. `04_nvidia_cartons.png`：官方纸箱的封箱胶带、标签、纹理和完整几何近景。
5. `05_table_dimensions.png`：用实际 USD 相机矩阵投影标注两个 2000 mm 边长和地面至桌面 800 mm 高度；另存未经标注的 `05_table_dimensions_raw.png`。

`render_report.json` 记录精确相机、变换、碰撞结果和图像 SHA-256。下载后逐张核对哈希与分辨率。渲染使用停止的时间轴、零时间步，最终时间 **0.0 s**；未执行新的机器人动作、吸附、释放或出料。原图包为同一 run 根目录下的 `warehouse_original_images.zip`。

既有四箱布局数值和冻结初始完整几何两项 pytest 回归 **2 passed**。最终 USD 使用本地相对依赖重新加载通过，官方文件哈希和五张交付图哈希均核对一致。

试渲染和失败日志保留：早期运行的现有库加载顺序、相机 API 参数、外侧铺地区域取景、透明板材质及尺寸标注构图问题均在独立输出中记录；旧目录不覆盖。最终采用明确的 USD 相机，确保渲染与尺寸标注使用相同矩阵。

本轮至此停止，等待用户确认外观，再另行开展物理抓放。
