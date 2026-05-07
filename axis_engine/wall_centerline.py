"""
墙体中心线提取算法 (Refactored)
=============================
针对正交（水平/竖直）墙体，从外轮廓 Polygon 反向提取 CAD 风格的中心线网络。

主要功能：
1. 自动推断墙体厚度。
2. 基于边缘配对提取骨架线。
3. 自动延伸端点以修复拐角连接。

依赖: shapely, numpy, matplotlib (仅测试用)
"""

from collections import Counter
from typing import List, Optional, Tuple, Union

import numpy as np
from matplotlib import pyplot as plt
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Polygon
from shapely.ops import linemerge, unary_union

# 类型别名
Geometry = Union[Polygon, MultiPolygon]
Segment = Tuple[float, float, float, bool]  # (const_coord, min_coord, max_coord, is_exterior)


# =============================================================================
# 1. 公共 API
# =============================================================================
def extract_mixed_thickness_walls(
    wall_geom_input,
    min_thickness_threshold=50.0,  # 最小墙厚阈值，低于此不再提取
    max_iterations=3,  # 最大剥离次数，防止死循环
):
    """
    处理混合厚度墙体的中心线提取（剥离法）。
    """
    if wall_geom_input is None or wall_geom_input.is_empty:
        return []

    # 预处理
    current_geom = wall_geom_input.buffer(0)
    all_centerlines = []

    # 记录已处理的厚度，避免重复死循环
    processed_thicknesses = set()

    print(f"开始混合厚度提取流程...")

    for i in range(max_iterations):
        if current_geom.is_empty or current_geom.area < 1e-3:
            print(f"  [Pass {i+1}] 几何体已空，停止提取。")
            break

        # 1. 估算当前剩余几何体的主导厚度
        current_thickness = estimate_wall_thickness(current_geom)
        current_thickness = int(round(current_thickness / 50) * 50)  # 四舍五入到最近的50mm

        # 终止条件：厚度太小或无法检测
        if current_thickness < min_thickness_threshold:
            print(f"  [Pass {i+1}] 检测厚度 {current_thickness:.2f} < 阈值，停止剥离。")
            break

        print(f"  [Pass {i+1}] 识别主导厚度: {current_thickness:.2f}")
        processed_thicknesses.add(current_thickness)

        # 清洗碎片：去掉面积极小的噪点
        if isinstance(current_geom, MultiPolygon):
            valid_polys = [p for p in current_geom.geoms if p.area > (current_thickness * current_thickness)]
            current_geom = unary_union(valid_polys)
        elif isinstance(current_geom, Polygon):
            if current_geom.area < (current_thickness * current_thickness):
                current_geom = Polygon()

        # 2. 提取当前厚度的中心线
        # 注意：这里我们只提取符合当前厚度的部分，tolerance设紧一点
        lines = extract_wall_centerline(
            current_geom,
            thickness=current_thickness,
            tolerance_ratio=0.2,  # 容差稍微给大一点点，适应施工误差
            merge_lines=False,  # 先不合并，方便后续处理
        )

        if lines.is_empty:
            print(f"  [Pass {i+1}] 未提取到有效线段，跳过。")
            continue

        # 收集线段
        if isinstance(lines, LineString):
            all_centerlines.append((lines, current_thickness))
        elif isinstance(lines, MultiLineString):
            for line in lines.geoms:
                all_centerlines.append((line, current_thickness))

        # 3. 构造遮罩并剥离 (Peeling)
        # 用提取出的线段，按当前厚度生成Buffer，从原图中挖掉
        # 技巧：buffer稍微大一点点(比如+0.1mm)，确保切断连接处，防止残留细丝
        lines = linemerge(lines)
        mask = lines.buffer(current_thickness / 2.0 + 0.1, cap_style=2, join_style=2)
        current_geom = current_geom.difference(mask)
        print(f"  [Pass {i+1}] 剥离后剩余面积: {current_geom.area:.2f}")
        # visualize current geometry for debug
        # visualize_wall_extraction(current_geom, lines, title=f"Current Geometry after {i+1} passes")

    return all_centerlines


def extract_wall_centerline(
    wall_geom: Geometry,
    thickness: Optional[float] = None,
    tolerance_ratio: float = 0.15,
    merge_lines: bool = True,
) -> MultiLineString:
    """
    提取墙体中心线的主入口函数。

    Parameters
    ----------
    wall_geom : Polygon | MultiPolygon
        墙体的几何轮廓（输入前会自动进行 validity 修复）。
    thickness : float, optional
        墙体厚度。如果为 None，则自动根据几何特征推断。
    tolerance_ratio : float
        容差系数（相对于墙厚）。用于判定平行边是否配对。默认 0.15。
    merge_lines : bool
        是否将连接的线段合并为长线段（LineMerge）。默认 True。

    Returns
    -------
    MultiLineString
        提取出的中心线网络。
    """
    if wall_geom is None or wall_geom.is_empty:
        return MultiLineString([])

    # 预处理：标准化几何
    wall_geom = wall_geom.buffer(0)

    # 1. 自动推断墙厚
    if thickness is None or thickness <= 0:
        thickness = estimate_wall_thickness(wall_geom)
        if thickness <= 0:
            # 无法推断且未指定，返回空或尝试极小值（此处直接返回空以防误判）
            return MultiLineString([])

    tolerance = thickness * tolerance_ratio

    # 处理输入可能是 MultiPolygon 的情况
    polys = list(wall_geom.geoms) if isinstance(wall_geom, MultiPolygon) else [wall_geom]

    raw_lines = []
    for poly in polys:
        # 2. 提取原始中心线段
        lines = _extract_segments_from_poly(poly, thickness, tolerance)
        raw_lines.extend(lines)

    if not raw_lines:
        return MultiLineString([])

    # 3. 修复拐角连接（延伸线段至交点）
    healed_lines = _heal_corner_connections(raw_lines, thickness, tolerance)

    # 4. 合并线段
    result = linemerge(healed_lines) if merge_lines else MultiLineString(healed_lines)

    # 确保返回 MultiLineString 类型
    if result.geom_type == "LineString":
        return MultiLineString([result])
    return result.simplify(0)


def estimate_wall_thickness(wall_geom, num_samples: int = 50, max_wall_thickness: float = None) -> float:
    """
    通过扫描线切割法估算墙厚（鲁棒性最强的方法）。
    适用于：田字形、回字形、复杂多孔结构。

    原理：
    生成水平和竖直的扫描线穿过几何体，测量扫描线在几何体内部的截断长度。
    统计这些截断长度的众数。

    Parameters
    ----------
    wall_geom : Geometry
        墙体几何
    num_samples : int
        每个方向扫描线的数量，默认50条（数量越多越准，但越慢）
    max_wall_thickness : float, optional
        预期的最大墙厚（用于过滤掉纵向切过墙体的长线段）。
        如果为None，取几何体边界框短边的 1/5。
    """
    if wall_geom is None or wall_geom.is_empty:
        return 0.0

    # 确保几何有效
    if not wall_geom.is_valid:
        wall_geom = wall_geom.buffer(0)

    minx, miny, maxx, maxy = wall_geom.bounds
    width = maxx - minx
    height = maxy - miny

    # 自动设定过滤阈值：如果截取长度太长，说明是顺着墙切的，不是切断面
    if max_wall_thickness is None:
        max_wall_thickness = min(width, height) * 0.2
        # 兜底：如果几何体很小，至少允许一定厚度
        max_wall_thickness = max(max_wall_thickness, 5.0)

    intersect_lengths = []

    # 定义扫描函数
    def scan_direction(axis_min, axis_max, other_axis_min, other_axis_max, is_vertical):
        # 在范围内生成均匀分布的扫描位置
        positions = np.linspace(axis_min, axis_max, num=num_samples)

        for pos in positions:
            # 构造扫描线
            if is_vertical:
                line = LineString([(pos, other_axis_min - 1), (pos, other_axis_max + 1)])
            else:
                line = LineString([(other_axis_min - 1, pos), (other_axis_max + 1, pos)])

            # 计算交集
            intersection = wall_geom.intersection(line)

            if intersection.is_empty:
                continue

            # 处理交集结果 (可能是 LineString 或 MultiLineString)
            segments = []
            if intersection.geom_type == "LineString":
                segments = [intersection]
            elif intersection.geom_type == "MultiLineString":
                segments = list(intersection.geoms)
            elif intersection.geom_type == "GeometryCollection":
                # 有时会包含点，需过滤
                segments = [g for g in intersection.geoms if g.geom_type in ["LineString"]]

            # 收集长度
            for seg in segments:
                length = seg.length
                # 过滤掉噪点（太短）和纵向切面（太长）
                if 0.01 < length < max_wall_thickness:
                    intersect_lengths.append(length)

    # 1. 竖直扫描 (测量水平墙段的厚度)
    scan_direction(minx, maxx, miny, maxy, is_vertical=True)

    # 2. 水平扫描 (测量竖直墙段的厚度)
    scan_direction(miny, maxy, minx, maxx, is_vertical=False)

    if not intersect_lengths:
        # 如果扫描失败（极罕见），回退到旧方法或最小宽度法
        return 0.0

    # 统计众数 (自适应分箱)
    return _get_mode_value(intersect_lengths)


def visualize_wall_extraction(
    wall_geom,
    centerline_geom,
    title: str = "Wall Centerline Extraction",
    show: bool = True,
    save_path: str = None,
    ax=None,
):
    """
    可视化墙体及其中心线提取结果。

    Parameters
    ----------
    wall_geom : Polygon | MultiPolygon
        原始墙体几何。
    centerline_geom : LineString | MultiLineString
        提取出的中心线几何。
    title : str
        图表标题。
    show : bool
        是否立即调用 plt.show()。
    save_path : str, optional
        如果提供路径，将图片保存到该位置。
    ax : matplotlib.axes.Axes, optional
        如果提供 ax，则在指定的 ax 上绘制（用于子图）；否则创建新图。
    """

    # 1. 准备绘图环境
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 10))

    # 2. 绘制墙体 (Wall)
    # 辅助函数：绘制单个多边形（含空洞）
    def plot_single_polygon(poly: Polygon):
        # 绘制外轮廓填充
        x, y = poly.exterior.xy
        ax.fill(x, y, color="#cccccc", alpha=0.5, label="Wall Body")  # 浅灰色填充
        ax.plot(x, y, color="black", linewidth=1.0, label="Wall Border")  # 黑色边框

        # 绘制内孔 (Interiors)
        for interior in poly.interiors:
            ix, iy = interior.xy
            ax.plot(ix, iy, color="black", linewidth=1.0)
            # 在填充层之上用白色填充孔洞，制造"镂空"视觉效果
            # 注意：如果背景不是白色，这里需要改为背景色
            ax.fill(ix, iy, color="white")

    if isinstance(wall_geom, Polygon):
        plot_single_polygon(wall_geom)
    elif isinstance(wall_geom, MultiPolygon):
        for poly in wall_geom.geoms:
            plot_single_polygon(poly)

    # 3. 绘制中心线 (Centerline)
    if centerline_geom and not centerline_geom.is_empty:
        if isinstance(centerline_geom, LineString):
            lx, ly = centerline_geom.xy
            ax.plot(lx, ly, color="red", linewidth=2, zorder=10)
        elif isinstance(centerline_geom, MultiLineString):
            for i, line in enumerate(centerline_geom.geoms):
                lx, ly = line.xy
                ax.plot(lx, ly, color="red", linewidth=2, zorder=10)

    # 4. 设置图表样式
    ax.set_title(title, fontsize=14)
    ax.set_aspect("equal", adjustable="box")  # 保持几何比例
    ax.grid(True, linestyle="--", alpha=0.3)
    # 去掉坐标轴刻度
    ax.set_xticks([])
    ax.set_yticks([])

    # 5. 输出
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        print(f"Visualization saved to: {save_path}")

    if show:  # 如果是作为独立图表运行
        plt.show()

    return ax


# =============================================================================
# 2. 核心逻辑：线段提取
# =============================================================================


def _extract_segments_from_poly(poly: Polygon, thickness: float, tolerance: float) -> List[LineString]:
    """
    从单个 Polygon 中提取中心线，并增加实体验证防止误判。
    """
    h_edges, v_edges = _decompose_polygon_edges(poly, tolerance)
    centerlines = []

    # 定义验证函数：检查生成的线是否大部分位于墙体内
    def is_line_valid_inside(line: LineString, geometry: Polygon, threshold: float = 0.7) -> bool:
        """
        验证线段有效性。
        原理：计算线段与墙体多边形的交集长度。
        如果 (交集长度 / 线段总长) > 阈值，则认为该线段有效。
        阈值设为 0.95 而不是 1.0 是为了容忍浮点数计算误差。
        """
        if line.length < 1e-6:
            return False

        # 快速预判：如果中点都不在多边形内，直接排除（性能优化）
        midpoint = line.interpolate(0.5, normalized=True)
        if not geometry.contains(midpoint) and not geometry.touches(midpoint):
            # 注意：contains 对边界点返回 False，所以加上 touches 兜底，
            # 但更严谨的是直接做 intersection，这里仅作为快速过滤
            # 为保险起见，如果中点检测失败，我们还是走一下完整 intersection 流程，
            # 除非明确确定中点在完全外部。
            # 简单起见，这里直接进行 intersection 计算，它是最准的。
            pass

        try:
            intersection = line.intersection(geometry)
            return (intersection.length / line.length) > threshold
        except Exception:
            # 极少数拓扑错误情况
            return False

    def process_pairs(edges: List[Segment], is_horizontal: bool):
        n = len(edges)
        for i in range(n):
            coord1, min1, max1, _ = edges[i]
            for j in range(i + 1, n):
                coord2, min2, max2, _ = edges[j]

                # 1. 距离判定
                dist = abs(coord1 - coord2)
                if abs(dist - thickness) > tolerance:
                    continue

                # 2. 投影重叠判定
                overlap_start = max(min1, min2)
                overlap_end = min(max1, max2)

                if overlap_end - overlap_start > tolerance:
                    mid_coord = (coord1 + coord2) / 2

                    # 3. 生成候选线
                    if is_horizontal:
                        candidate = LineString([(overlap_start, mid_coord), (overlap_end, mid_coord)])
                    else:
                        candidate = LineString([(mid_coord, overlap_start), (mid_coord, overlap_end)])

                    # 4. [新增] 实体空间验证
                    # 只有当线段确实“踩”在墙体实体上时才保留
                    if is_line_valid_inside(candidate, poly):
                        centerlines.append(candidate)

    process_pairs(h_edges, is_horizontal=True)
    process_pairs(v_edges, is_horizontal=False)

    return centerlines


def _decompose_polygon_edges(poly: Polygon, tolerance: float) -> Tuple[List[Segment], List[Segment]]:
    """分解多边形的所有环（外环+内环）为水平和竖直边"""
    h_edges = []
    v_edges = []

    def process_ring(coords, is_exterior):
        if len(coords) < 2:
            return
        # 关闭环的处理已经在Shapely coords中包含(首点=尾点)
        pts = coords
        if pts[0] == pts[-1]:
            pts = pts[:-1]

        n = len(pts)
        for i in range(n):
            p1 = pts[i]
            p2 = pts[(i + 1) % n]

            dx = abs(p2[0] - p1[0])
            dy = abs(p2[1] - p1[1])

            # 判断方向
            if dy < tolerance and dx > tolerance:  # 水平
                y = (p1[1] + p2[1]) / 2
                x_min, x_max = sorted((p1[0], p2[0]))
                h_edges.append((y, x_min, x_max, is_exterior))
            elif dx < tolerance and dy > tolerance:  # 竖直
                x = (p1[0] + p2[0]) / 2
                y_min, y_max = sorted((p1[1], p2[1]))
                v_edges.append((x, y_min, y_max, is_exterior))

    process_ring(poly.exterior.coords, True)
    for interior in poly.interiors:
        process_ring(interior.coords, False)

    return h_edges, v_edges


# =============================================================================
# 3. 后处理：拐角修复 (Heal Corners)
# =============================================================================


def _heal_corner_connections(lines: List[LineString], thickness: float, tolerance: float) -> List[LineString]:
    """
    延长相邻且正交的线段以形成闭合拐角。
    """
    if len(lines) < 2:
        return lines

    half_t = thickness / 2.0
    # 搜索半径：略大于半墙厚，用于捕捉L型拐角的断开
    search_radius = half_t * 1.5

    # 转换为可变列表
    # 结构: [coords_list, ...]
    multi_lines = linemerge(lines).simplify(0)
    lines_coords = [list(line.coords) for line in multi_lines.geoms]

    # 记录每个端点的信息: (line_index, is_start_point, point_coord)
    endpoints = []
    for idx, coords in enumerate(lines_coords):
        endpoints.append((idx, True, coords[0]))  # 起点
        endpoints.append((idx, False, coords[-1]))  # 终点

    # 查找并应用延伸
    # 注意：简单的两两比较是O(N^2)，对于建筑图纸通常N较小，可接受
    for i in range(len(endpoints)):
        idx1, is_start1, pt1 = endpoints[i]

        for j in range(i + 1, len(endpoints)):
            idx2, is_start2, pt2 = endpoints[j]

            if idx1 == idx2:
                continue  # 同一条线

            # 计算距离
            dist_sq = (pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2

            # 如果端点并未连接，但在容差范围内
            if tolerance**2 < dist_sq < search_radius**2:
                # 检查正交性
                coords1 = lines_coords[idx1]
                coords2 = lines_coords[idx2]

                is_horz1 = abs(coords1[0][1] - coords1[-1][1]) < abs(coords1[0][0] - coords1[-1][0])
                is_horz2 = abs(coords2[0][1] - coords2[-1][1]) < abs(coords2[0][0] - coords2[-1][0])

                # if is_horz1 != is_horz2:
                # 计算理想的交点 (Orthogonal intersection)
                if is_horz1:
                    target_pt = (coords2[0][0], coords1[0][1])  # (x from vert, y from horz)
                else:
                    target_pt = (coords1[0][0], coords2[0][1])

                # 更新线段坐标
                if is_start1:
                    lines_coords[idx1][0] = target_pt
                else:
                    lines_coords[idx1][-1] = target_pt

                if is_start2:
                    lines_coords[idx2][0] = target_pt
                else:
                    lines_coords[idx2][-1] = target_pt

    # 重建 LineString 对象并过滤掉退化的线段
    result = []
    for coords in lines_coords:
        ls = LineString(coords)
        if ls.length > tolerance:
            result.append(ls)

    return result


def _get_mode_value(values: List[float]) -> float:
    """自适应分箱求众数"""
    if not values:
        return 0.0
    arr = np.array(values)
    # 动态步长：取范围的5%作为bin大小，防止浮点误差
    bin_step = max((np.max(arr) - np.min(arr)) * 0.05, 0.01)

    binned = np.round(arr / bin_step).astype(int)
    most_common_bin = Counter(binned).most_common(1)[0][0]

    # 取该bin内所有原始值的均值
    mask = binned == most_common_bin
    return float(np.mean(arr[mask]))
