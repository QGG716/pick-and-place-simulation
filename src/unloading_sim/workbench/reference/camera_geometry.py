"""Projection metadata and explanations of the coordinates actually computed."""

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class ProjectionInfo:
    mode: str
    width: int
    height: int
    fx: float | None = None
    fy: float | None = None
    cx: float | None = None
    cy: float | None = None
    scale_m: float | None = None
    baseline_m: float | None = None
    q: tuple[tuple[float, ...], ...] | None = None
    depth_type: int | None = None
    engine: str = ""
    # RGB calibration projection, when its distortion convention is known.
    distortion: tuple[float, ...] | None = None
    distortion_model: str = ""


def opencv_brown_coefficients(info):
    """Return compatible RGB coefficients without treating missing data as zero."""
    original=info.distortion_model or '未知'
    model=original.strip().lower().removeprefix('distortion.').removeprefix('rs2_distortion_')
    direct=('opencv_brown','brown_conrady')
    zero_only=('inverse_brown_conrady','modified_brown_conrady')
    if model not in (*direct,*zero_only,'none'):
        raise ValueError(f'SDK 畸变模型 {original} 暂不支持用于 OpenCV Brown；请提供已校验的 RGB 标定参数')
    if info.distortion is None:
        if model=='none':return (0.,)*5
        raise ValueError(f'当前 RGB 的畸变系数未知（模型 {original}），不能使用默认零畸变')
    try:
        coefficients=tuple(float(value) for value in info.distortion)
    except (TypeError,ValueError) as exc:
        raise ValueError(f'SDK 畸变系数无效（模型 {original}）') from exc
    counts=(4,5,8) if model in direct else (5,)
    if len(coefficients) not in counts or not all(isfinite(value) for value in coefficients):
        raise ValueError(f'SDK 畸变系数数量或数值无效（模型 {original}）')
    if model in (*zero_only,'none') and any(value!=0 for value in coefficients):
        raise ValueError(f'SDK 畸变模型 {original} 返回非零系数 {coefficients}，不能直接用于 OpenCV Brown；请提供对应 RGB 的 OpenCV 标定参数')
    # The inverse/modified Brown models are identical to a pinhole model only
    # in the exact all-zero case. Retain the values actually supplied by SDK.
    return coefficients


def xyz_formula(mode, min_depth=0.1, max_depth=3.0, info=None, *, layout="wide"):
    """Short equations for the camera connection page's XYZ calculation box."""
    if mode == "realsense":
        lines = ["RealSense XYZ · m", "Z = d × s", "X = (u − cx) Z / fx",
                 "Y = (v − cy) Z / fy", "需: u,v,d,s + 内参"]
    elif mode == "stereo":
        lines = ["双目 XYZ · m", "h = Q [u,v,d,1]ᵀ", "(X,Y,Z)=h₁:₃/(1000 h₄)",
                 "d = u左 − u右（px）", "参数: u,v,d,Q（标定）"]
    elif mode == "xd500":
        distance = "R" if info and info.depth_type == 0 else "Ztof" if info and info.depth_type == 1 else "D"
        if layout == "wide":
            return "\n".join([f"X-D500 · 原始 TOF · {distance} = d × s（m）",
                              "RGB/TOF 未配准 · 目标 XYZ 暂不可用", "需: 内参、类型、配准"])
        if layout == "medium":
            return "\n".join([f"X-D500 · {distance} = d × s（m）", "RGB/TOF 未配准",
                              "目标 XYZ 暂不可用", "需: 内参、类型、配准"])
        return "\n".join(["X-D500 · 原始 TOF", f"{distance} = d × s（m）", "RGB/TOF 未配准",
                          "目标 XYZ 暂不可用", "需: 内参、类型、配准"])
    else:
        if layout != "narrow":
            return "\n".join(["单目 RGB · XYZ 不可用", "X=(u−cx)Z/fx  Y=(v−cy)Z/fy",
                              "需: 深度 Z 与相机内参"])
        return "\n".join(["单目 RGB · XYZ 不可用", "仅有目标像素 (u,v)", "需: 深度 Z 与相机内参",
                          "X = (u − cx) Z / fx", "Y = (v − cy) Z / fy"])
    lines.append(f"深度: {min_depth:.2f}–{max_depth:.2f} m")
    if layout == "wide":
        return "\n".join([lines[0] + " · " + lines[1], lines[2] + "； " + lines[3],
                          lines[4] + " · " + lines[5]])
    if layout == "medium":
        if mode == "realsense":
            return "\n".join(["RealSense XYZ/m · Z = d × s", "X=(u−cx)Z/fx  Y=(v−cy)Z/fy",
                              lines[4], lines[5]])
        return "\n".join(["双目 XYZ/m · d=u左−u右(px)", lines[1], lines[2],
                          f"Q标定 · 深度: {min_depth:.2f}–{max_depth:.2f}m"])
    return "\n".join(lines)


def xyz_details(mode, info=None):
    """Full parameter definitions; absent live values are explicitly unavailable."""
    if mode == "realsense":
        text = ["RealSense：深度先对齐至 RGB，使用对齐深度帧的内参。",
                "Z = d × s；X = (u − cx) × Z / fx；Y = (v − cy) × Z / fy。",
                "u,v：选定目标像素，单位 px；d：该像素原始深度值。",
                "s：SDK 深度单位（m/原始值）；fx,fy：像素焦距；cx,cy：主点（px）。",
                "所需参数由 RealSense SDK 读取，不使用机械臂手眼标定矩阵。"]
    elif mode == "stereo":
        text = ["双目：先用 K左、K右、畸变 D左/D右及外参 R,T 做双目校正，再计算视差。",
                "d = u左 − u右（px）；h = Q × [u,v,d,1]ᵀ；XYZ = h[:3] / h[3] / 1000（m）。",
                "当前调用 OpenCV reprojectImageTo3D，除以 1000 是因为标定平移量 T 使用 mm。",
                "零视差校正下可写成 Z = f × B / d，X = (u − cx) × Z / f，Y = (v − cy) × Z / f。",
                "f、cx、cy 来自校正投影矩阵；B 是校正后的基线长度，单位 m。",
                "参数来源：程序内置的双目标定常量（参考分辨率 1280×720），内参按实际尺寸缩放，Q 重新计算。",
                "这些参数必须对应实际双目相机；它们不是在线自动标定结果。",
                "输出坐标属于校正后的左相机坐标系。SGBM 的原始视差除以 16，IGEV 视差单位为 px。"]
    elif mode == "xd500":
        text = ["X-D500 当前只解码 RGB 与原始 TOF 深度：D = d × fUnitOfDepth / 1000（m）。",
                "深度类型 0 为斜距 R，1 为光轴深度 Z；斜距不能直接当作 Z。",
                "尚未将 TOF 配准到 RGB，因此不能在 RGB 检测像素上直接取 TOF 值计算目标 XYZ。",
                "还需要 TOF/RGB 内参、畸变、两相机外参和正确的深度类型，完成反投影与配准。"]
    else:
        text = ["单目 USB 目前只提供 RGB 与目标像素，没有度量深度或已校验内参，不计算目标 XYZ。",
                "若将来具备 Z 与有效内参，可用 X = (u − cx) × Z / fx，Y = (v − cy) × Z / fy。",
                "仅靠检测框或像素位置，无法唯一确定三维坐标。"]
    if info is None or info.mode != mode:
        text.append("\n当前相机尚无采集参数，未填入示例内参或深度单位。")
    else:
        text.append(f"\n实际投影图像尺寸：{info.width} × {info.height} px")
        for name, value in (("fx / px", info.fx), ("fy / px", info.fy), ("cx / px", info.cx),
                            ("cy / px", info.cy), ("s / m", info.scale_m), ("B / m", info.baseline_m)):
            if value is not None:
                text.append(f"{name} = {value:.9g}")
        if info.engine:
            text.append(f"视差算法：{info.engine}")
        if info.depth_type is not None:
            text.append(f"深度类型：{info.depth_type}（{'斜距 R' if info.depth_type == 0 else '光轴 Z'}）")
        if info.q is not None:
            text.append("Q（当前分辨率）：\n" + "\n".join("  ".join(f"{value:.8g}" for value in row) for row in info.q))
    if mode in {"realsense", "stereo"}:
        text.extend(["\n目标像素的实际选取过程：",
                     "1. 取推理同一帧的 SAM/分割掩码内有效深度（有限且 > 0），计算深度中位数。",
                     "2. 优先保留与中位数相差不超过 0.03 m 的像素；若为空则保留原有效区域。",
                     "3. 在这些像素中选最接近其像素质心的真实像素 (u,v)，而非直接使用检测框中心。",
                     "4. 列表 XYZ 从该像素对应的 X/Y/Z 图读取；没有有效分割/深度就不产生抓取坐标。",
                     "相机坐标：X 向右、Y 向下、Z 向前，单位 m。",
                     "画面框上的 Z 使用当前帧区域深度中位数，可能与同帧测得并保留在列表中的 XYZ 略有差异。",
                     "手眼矩阵只参与之后的相机坐标到机械臂基坐标转换。"])
    return "\n".join(text)
