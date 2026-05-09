import json
from pathlib import Path
from typing import List

import matplotlib.cm as cm
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Polygon
from shapely.geometry.polygon import orient
from shapely.ops import linemerge, polygonize, unary_union
from shapely.strtree import STRtree

from axis_engine.linework_axis_extractor import (
    extract_wall_axes_from_linework,
    infer_wall_thicknesses,
    repair_wall_linework,
)
from axis_engine.opening_clustering import cluster_opening_segments
from axis_engine.opening_embedment import (
    OPENING_BALCONY,
    OPENING_DOOR,
    infer_opening_embedments,
)
from axis_engine.raw_wall_polygon_builder import build_wall_polygon_from_raw_lines
from cad_tests.dxf_utils import (
    pick_dxf_wall_layers,
    read_dxf,
    read_dxf_line_segments_from_layers,
    read_dxf_segments_from_layers,
)

from .line_network_calibrator import LineNetworkCalibrator, NetworkSegment, SegmentType
from .rect_decomposer import RectangularDecomposer
from .wall_centerline import extract_mixed_thickness_walls, visualize_wall_extraction

# from preprocess.wall_centerline import extract_wall_centerline


def _iter_lines(geometry):
    if isinstance(geometry, LineString):
        return [geometry]
    if isinstance(geometry, MultiLineString):
        return list(geometry.geoms)
    if hasattr(geometry, "geoms"):
        return [item for geom in geometry.geoms for item in _iter_lines(geom)]
    return []


def _iter_straight_segments(geometry, min_length=1.0):
    for line in _iter_lines(geometry):
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        for start, end in zip(coords, coords[1:]):
            segment = LineString([start, end])
            if segment.length >= min_length:
                yield segment


def _remove_collinear_vertices(coords):
    """
    从一个坐标列表中移除所有共线的顶点。
    例如，对于 A-B-C，如果B在AC线段上，则B将被移除。

    Args:
        coords: 一个代表多边形环的坐标点列表 (第一个点等于最后一个点)。

    Returns:
        清理后的坐标点列表。
    """
    if len(coords) < 4:
        return coords  # 无法简化三角形或更小的图形

    cleaned_coords = []
    # 我们需要处理环形列表，所以使用(i-1, i, i+1)的模式并处理环绕
    input_points = coords[:-1]  # 使用不含重复收尾点的列表
    n = len(input_points)

    for i in range(n):
        p_prev = input_points[(i - 1 + n) % n]
        p_curr = input_points[i]
        p_next = input_points[(i + 1) % n]

        # 使用向量叉积来判断共线性
        # (p_curr[1] - p_prev[1]) * (p_next[0] - p_curr[0]) - (p_curr[0] - p_prev[0]) * (p_next[1] - p_curr[1])
        # 如果叉积约等于0，则三点共线
        cross_product = (p_curr[1] - p_prev[1]) * (p_next[0] - p_curr[0]) - (p_curr[0] - p_prev[0]) * (
            p_next[1] - p_curr[1]
        )

        # 只有当点不共线时才保留 (允许一个小的容差)
        if abs(cross_product) > 1e-9:
            cleaned_coords.append(p_curr)

    if not cleaned_coords:
        return coords  # 如果所有点都共线，返回原始坐标

    # 重新添加闭合点并返回
    cleaned_coords.append(cleaned_coords[0])
    return cleaned_coords


def form_polygons_from_cluster(line_cluster):
    """
    将一个线段集群转换为一个闭合多边形。

    此函数利用Shapely的强大功能，首先合并所有线段以清理拓扑关系
    （如移除内部重叠线），然后从结果中构建多边形。

    Args:
        line_cluster (List[Line]): 代表单个墙体集群的线段列表。

    Returns:
        Tuple[Polygon, List[Point]]:
            - 第一个元素是Shapely的Polygon对象。
            - 第二个元素是顶点列表，代表一个多边形的有序顶点。
    """
    if not line_cluster:
        return [], []

    # 1. 将原始线段转换为Shapely的LineString对象
    line_strings = [LineString(line) for line in line_cluster]
    line_strings = [line for line in line_strings if line.length > 5]

    # 2. 合并所有线段，这个操作会处理所有内部线条，只留下轮廓
    # 这对于处理T型连接处的冗余线非常有效
    union_lines = unary_union(line_strings)

    # 3. 从融合后的线网络中构建多边形
    # polygonize会为所有闭合环创建多边形，包括内部的
    initial_polygons = list(polygonize(union_lines))

    if not initial_polygons:
        return None, []

    # print(f"从一个集群中初步生成了 {len(initial_polygons)} 个多边形。")

    # 4. 合并所有初步多边形，以溶解内边界，获得最外围轮廓
    merged_geometry = unary_union(initial_polygons)

    polygon_to_process: Polygon = None
    if merged_geometry.geom_type == "Polygon":
        polygon_to_process = merged_geometry
    elif merged_geometry.geom_type == "MultiPolygon":
        # 如果有多个不相交的区域，选择面积最大的作为主轮廓
        polygon_to_process = max(merged_geometry.geoms, key=lambda p: p.area)
        print(f"合并后生成了多区域多边形，已选择其中面积最大的一个。")

    if not polygon_to_process:
        print("警告: 没能从集群生成有效的外轮廓多边形。")
        return None, []

    # 5. 提取有序顶点并确保方向
    # --- 新增：简化多边形，移除共线顶点 ---
    # simplify(0) 可以移除不改变几何形状的冗余顶点
    # buffer(0) 是一个常用的技巧，用于修复可能由其他操作导致的无效几何
    simplified_poly = polygon_to_process.simplify(0.1).buffer(0)

    # 检查简化后是否还是有效的多边形
    if not isinstance(simplified_poly, Polygon) or simplified_poly.is_empty:
        # 如果简化导致多边形退化，则使用原始多边形
        simplified_poly = polygon_to_process

    # 强制将多边形的外环设置为逆时针方向
    oriented_polygon = orient(simplified_poly, sign=1.0)

    # 移除共线顶点，只保留真正的角点
    cleaned_coords = _remove_collinear_vertices(list(oriented_polygon.exterior.coords))

    # 从清理后的顶点创建最终的多边形
    # 确保清理后仍然有足够顶点形成一个有效的多边形
    if len(cleaned_coords) >= 4:
        final_polygon = Polygon(cleaned_coords)
    else:
        # 如果清理过度，则退回使用原始方向的多边形
        final_polygon = oriented_polygon

    return final_polygon


class CADLayoutProcessor:
    def __init__(
        self,
        source_path,
        wall_layers=None,
        opening_layers=None,
        wall_thicknesses=None,
        visible_only: bool = False,
        wall_axis_options=None,
        opening_cluster_options=None,
        embedment_options=None,
    ):
        self.source_path = Path(source_path)
        self.json_path = str(self.source_path)
        self.is_dxf = self.source_path.suffix.lower() == ".dxf"
        self.wall_layers = wall_layers
        self.opening_layers = list(opening_layers or ["WINDOW"])
        self.wall_thicknesses = wall_thicknesses
        self.visible_only = visible_only
        self.wall_axis_options = dict(wall_axis_options or {})
        self.opening_cluster_options = dict(opening_cluster_options or {})
        self.embedment_options = dict(embedment_options or {})
        self.raw_data = {} if self.is_dxf else self._load_json()

        # 几何数据容器
        self.wall_polygon = MultiPolygon()  # 重构后的墙体多边形
        self.wall_centerlines = []  # 计算出的墙体中轴线，改为列表存储 (LineString, thickness)
        self.components = {}  # 其他构件 (门窗梁) 的 LineStrings
        self.opening_clusters = []
        self.opening_embedments = []
        self.wall_lines = []
        self.opening_lines = []

    def _load_json(self):
        with open(self.source_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def _parse_segments_to_lines(self, component_list, precision=0):
        """解析 JSON 中的 StartPoint/EndPoint 格式为 Shapely LineString 列表"""
        lines = []
        for element in component_list:
            p1_x = round(element["StartPoint"]["X"], precision)
            p1_y = round(element["StartPoint"]["Y"], precision)
            p2_x = round(element["EndPoint"]["X"], precision)
            p2_y = round(element["EndPoint"]["Y"], precision)

            p1 = (p1_x, p1_y)
            p2 = (p2_x, p2_y)
            lines.append(LineString([p1, p2]))
        return lines

    def _cluster_lines_with_tolerance(
        self, lines: List[List[float]], tolerance: float = 5.0
    ) -> List[List[LineString]]:
        """
        基于距离容差聚类线段。
        如果两条线段之间的最短距离小于 tolerance，则认为它们属于同一个集群。
        """
        if not lines:
            return []

        # 转换为 Shapely 对象
        shapely_lines = []
        for line in lines:
            # 兼容输入可能是坐标列表或LineString对象
            if isinstance(line, LineString):
                shapely_lines.append(line)
            else:
                # 假设输入是 [[x1, y1], [x2, y2]]
                shapely_lines.append(LineString(line))

        graph = nx.Graph()
        graph.add_nodes_from(range(len(shapely_lines)))

        # 创建空间索引
        tree = STRtree(shapely_lines)

        for i, line_i in enumerate(shapely_lines):
            # 技巧：将线段 i 膨胀 tolerance，然后去查相交
            # 这样能快速筛选出距离在 tolerance 范围内的候选线段
            search_geom = line_i.buffer(tolerance)
            possible_indices = tree.query(search_geom)

            for j in possible_indices:
                if i >= j:
                    continue

                line_j = shapely_lines[j]

                # 精确距离判断
                if line_i.distance(line_j) <= tolerance:
                    graph.add_edge(i, j)

        # 提取连通分量
        clusters = []
        for component_indices in nx.connected_components(graph):
            cluster = [shapely_lines[idx] for idx in component_indices]
            clusters.append(cluster)

        return clusters

    def build_robust_wall_polygon(self, line_clusters, gap_tolerance=5.0):
        """
        将线段集群转换为实体墙多边形，自动修复微小间隙。

        原理：
        不直接使用 polygonize（因为它要求完美闭合），而是将线段
        稍微膨胀一点点，变成"香肠"形状，然后融合在一起。
        这能自动粘合断开的端点。
        """
        valid_polys = []

        for cluster in line_clusters:
            if not cluster:
                continue

            # 1. 融合线段
            merged_lines = unary_union(cluster)

            poly_res = list(polygonize(merged_lines))

            for p in poly_res:

                # 3. 整理结果
                if p.is_empty:
                    continue

                if isinstance(p, Polygon):
                    valid_polys.append(p)
                elif isinstance(p, MultiPolygon):
                    valid_polys.extend(p.geoms)

        # 最后合并所有集群生成的块
        final_wall_geom = unary_union(valid_polys)

        # 转换为 MultiPolygon 统一格式
        if isinstance(final_wall_geom, Polygon):
            return MultiPolygon([final_wall_geom])
        return final_wall_geom

    def build_geometry(self):
        """步骤 1: 将散乱的线段构建为几何对象"""
        if self.is_dxf:
            self._build_geometry_from_dxf()
            return

        print("正在构建几何对象...")

        # 1. 处理墙体 (Walls) -> 重构为闭合多边形
        if "walls" in self.raw_data:
            wall_segments = self._parse_segments_to_lines(self.raw_data["walls"])
            if wall_segments:
                # clusters = self._cluster_lines_with_tolerance(wall_segments, tolerance=5.0)
                # # print(f"  - 墙体聚类完成: 发现 {len(clusters)} 个独立墙体组")

                # self.wall_polygon = self.build_robust_wall_polygon(clusters, gap_tolerance=5.0)

                merged_lines = unary_union(wall_segments)
                # polygonize 会寻找所有闭合环，不会只取最大，而是保留所有独立墙体
                polys = list(polygonize(merged_lines))
                self.wall_polygon = MultiPolygon(polys)
                print(f"  - 墙体重构完成: 生成了 {len(polys)} 个闭合区域")

                # 统计面积非零的区域
                count = len(self.wall_polygon.geoms) if not self.wall_polygon.is_empty else 0
                print(f"  - 几何重构完成: 生成了 {count} 个闭合墙体块")

        # 2. 处理其他构件 (直接转为 LineString)
        for comp_type in ["doors", "windows", "beams"]:
            if comp_type in self.raw_data:
                lines = self._parse_segments_to_lines(self.raw_data[comp_type])
                self.components[comp_type] = MultiLineString(lines) if lines else MultiLineString()
                print(f"  - {comp_type}: 加载了 {len(lines)} 条线段")

    def _build_geometry_from_dxf(self):
        """从 DXF 读取墙体和门窗图层，并直接构建单线语义网络。"""
        print("正在从 DXF 构建单线几何对象...")
        doc = read_dxf(self.source_path)

        wall_layers = pick_dxf_wall_layers(doc, self.wall_layers)
        self.wall_layers = wall_layers
        self.wall_lines = read_dxf_line_segments_from_layers(
            doc,
            wall_layers,
            visible_only=self.visible_only,
        )
        print(f"  - 墙体图层: {', '.join(wall_layers)}")
        print(f"  - 原始墙体线段: {len(self.wall_lines)}")

        repair_options = {
            key: self.wall_axis_options[key]
            for key in ("axis_tolerance", "snap_tolerance")
            if key in self.wall_axis_options
        }
        if "min_feature_len" in self.wall_axis_options:
            repair_options["min_segment_length"] = self.wall_axis_options["min_feature_len"]
        repaired_edges = repair_wall_linework(self.wall_lines, **repair_options)
        thicknesses = self.wall_thicknesses or infer_wall_thicknesses(repaired_edges)
        self.wall_thicknesses = thicknesses
        self.wall_centerlines = extract_wall_axes_from_linework(
            self.wall_lines,
            thickness_candidates=thicknesses,
            **self.wall_axis_options,
        )
        print(f"  - 墙轴线: {len(self.wall_centerlines)}")

        self.wall_polygon = build_wall_polygon_from_raw_lines(
            self.wall_lines,
            wall_thicknesses=thicknesses,
        )
        polygon_count = len(self.wall_polygon.geoms) if not self.wall_polygon.is_empty else 0
        print(f"  - 辅助墙体面: {polygon_count}")

        self.components["doors"] = MultiLineString()
        self.components["windows"] = MultiLineString()
        self.components["balconies"] = MultiLineString()

        if not self.opening_layers:
            return

        opening_segments = read_dxf_segments_from_layers(
            doc,
            self.opening_layers,
            visible_only=self.visible_only,
        )
        self.opening_lines = [segment.line for segment in opening_segments]
        self.opening_clusters = cluster_opening_segments(
            opening_segments,
            **self.opening_cluster_options,
        )
        self.opening_embedments = infer_opening_embedments(
            self.wall_centerlines,
            self.opening_clusters,
            **self.embedment_options,
        )
        self._load_opening_embedments_as_components()

        print(f"  - 门窗图层: {', '.join(self.opening_layers)}")
        print(f"  - 门窗原始线段: {len(self.opening_lines)}")
        print(f"  - 门窗基元组: {len(self.opening_clusters)}")
        print(f"  - 门窗嵌入线: {len(self.opening_embedments)}")

    def _load_opening_embedments_as_components(self):
        by_component = {
            "doors": [],
            "windows": [],
            "balconies": [],
        }
        for embedment in self.opening_embedments:
            lines = _iter_lines(embedment.embed_line)
            if embedment.opening_type == OPENING_DOOR:
                by_component["doors"].extend(lines)
            elif embedment.opening_type == OPENING_BALCONY:
                by_component["balconies"].extend(lines)
            else:
                by_component["windows"].extend(lines)

        for component_name, lines in by_component.items():
            self.components[component_name] = MultiLineString(lines) if lines else MultiLineString()

    def extract_centerlines(self):
        """步骤 2: 提取墙体中轴线"""
        if self.is_dxf:
            if self.wall_centerlines:
                print(f"墙轴线已由 DXF 线网流程提取: {len(self.wall_centerlines)} 条")
            else:
                print("警告: DXF 流程尚未生成墙轴线，请先执行 build_geometry()。")
            return

        if self.wall_polygon.is_empty:
            print("警告: 没有有效的墙体多边形，无法提取中轴线。")
            return

        print("正在提取墙体中轴线...")
        # 调用核心算法 (自动估算厚度 + 提取 + 验证)
        self.wall_centerlines = extract_mixed_thickness_walls(self.wall_polygon, max_iterations=3)
        print(f"  - 提取完成: 包含 {len(self.wall_centerlines)} 条中轴线段")

    def visualize(self, save_path=None):
        """步骤 3: 综合可视化"""
        print("正在生成可视化图表...")
        fig, ax = plt.subplots(figsize=(14, 10))

        # --- A. 绘制墙体 (底图) ---
        # 使用灰色填充墙体实心区域
        if not self.wall_polygon.is_empty:
            for poly in self.wall_polygon.geoms:
                # 绘制外轮廓
                x, y = poly.exterior.xy
                ax.fill(x, y, color="gray", alpha=0.3, zorder=1)  # 浅灰填充
                ax.plot(x, y, color="black", linewidth=1.0, zorder=2, label="Wall Border")

                # 绘制内孔 (如有)
                for interior in poly.interiors:
                    ix, iy = interior.xy
                    ax.fill(ix, iy, color="white", zorder=2)  # 挖空
                    ax.plot(ix, iy, color="black", linewidth=1.0, zorder=2)

        # 手动去重图例的辅助集合
        legend_handles = {}

        def add_legend(handle, label):
            if label not in legend_handles:
                legend_handles[label] = handle

        # --- B. 绘制计算出的墙体中轴线 ---
        if self.wall_centerlines:
            for line, thickness in self.wall_centerlines:
                x, y = line.xy
                (l,) = ax.plot(x, y, color="red", linewidth=2.0, linestyle="-", zorder=10)
                add_legend(l, "Extracted Centerline")
                # endpoints
                # s = ax.scatter(x, y, color="black", s=4, zorder=11)
                # add_legend(s, "Centerline Endpoints")

        # --- C. 绘制其他 CAD 构件 (门/窗/梁) ---
        # 定义样式配置
        style_map = {
            "doors": {"color": "blue", "lw": 2.5, "label": "Doors"},
            "windows": {"color": "green", "lw": 2.0, "label": "Windows"},  # 改为绿色区分
            "balconies": {"color": "#f57c00", "lw": 2.0, "label": "Balconies"},
            "beams": {"color": "orange", "lw": 1.5, "linestyle": "--", "label": "Beams"},
        }

        for comp_name, geom in self.components.items():
            if geom.is_empty:
                continue

            style = style_map.get(comp_name, {"color": "purple", "lw": 1})
            c = style["color"]
            lw = style["lw"]
            ls = style.get("linestyle", "-")
            lbl = f"{style['label']} ({len(geom.geoms)})"

            # 绘制所有线段
            lines_collection = []
            for line in geom.geoms:
                x, y = line.xy
                (l,) = ax.plot(x, y, color=c, linewidth=lw, linestyle=ls, zorder=5)
                lines_collection.append(l)

            if lines_collection:
                add_legend(lines_collection[0], lbl)

        # --- 设置图表属性 ---
        ax.set_aspect("equal")
        ax.set_title("Architectural Layout Analysis & Centerline Extraction", fontsize=15, fontweight="bold")
        ax.grid(True, alpha=0.2, linestyle="--")

        # # 生成图例
        # ax.legend(
        #     legend_handles.values(), legend_handles.keys(), loc="upper right", frameon=True, shadow=True
        # )

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches="tight")
            print(f"图表已保存至: {save_path}")

        plt.show()

    def generate_rooms(self):
        """步骤 4: 整合墙、门、窗，生成封闭房间区域"""
        print("正在生成房间区域...")

        # 1. 准备数据容器
        self.all_segments: List[NetworkSegment] = []

        for line, thickness in self.wall_centerlines:
            self._append_network_segments(
                line,
                thickness=thickness,
                seg_type=SegmentType.WALL,
                is_structural=(True if thickness >= 180.0 else False),
            )

        # B. 添加门 (Doors)
        if "doors" in self.components:
            for line in self.components["doors"].geoms:
                self._append_network_segments(
                    line,
                    thickness=100.0,  # 门通常依附于墙，厚度不重要，重要的是位置
                    seg_type=SegmentType.DOOR,
                    is_structural=False,
                )

        # C. 添加窗 (Windows)
        if "windows" in self.components:
            for line in self.components["windows"].geoms:
                self._append_network_segments(
                    line,
                    thickness=100.0,
                    seg_type=SegmentType.WINDOW,
                    is_structural=False,
                )

        # D. 添加阳台/飘窗轴线，当前房间生成阶段按非结构开口处理
        if "balconies" in self.components:
            for line in self.components["balconies"].geoms:
                self._append_network_segments(
                    line,
                    thickness=100.0,
                    seg_type=SegmentType.WINDOW,
                    is_structural=False,
                )

        # 2. 初始化校准器
        calibrator = LineNetworkCalibrator(structural_thickness_threshold=300.0)

        # 3. 执行校准与缝合
        # 这一步会返回一个完全连通的 MultiLineString (包含墙门窗)
        self.unified_network = calibrator.calibrate(self.all_segments)

        # 4. 生成房间多边形 (Polygonize)
        # polygonize 会寻找所有最小闭合环
        polys = list(polygonize(self.unified_network))
        # visualize_wall_extraction(
        #     MultiPolygon(polys),
        #     None,
        #     title="Initial Polygons from Unified Network",
        #     show=False,
        # )  # for debug

        # 5. 过滤无效区域 (如面积过小的碎块)
        valid_rooms = []
        decomposer = RectangularDecomposer()
        self.room_groups = []  # 结构: List[List[Polygon]]
        for p in polys:
            sub_rects = decomposer.decompose(p)
            valid_rooms.extend(sub_rects)
            self.room_groups.append(sub_rects)

        self.rooms = valid_rooms
        print(f"  - 房间生成完成: 识别到 {len(self.rooms)} 个房间")

    def _append_network_segments(self, geometry, thickness, seg_type, is_structural=False):
        for line in _iter_straight_segments(geometry):
            self.all_segments.append(
                NetworkSegment(
                    geometry=line,
                    thickness=thickness,
                    seg_type=seg_type,
                    is_structural=is_structural,
                )
            )

    def visualize_rooms(self, save_path=None, show=True):
        """可视化生成的房间"""
        fig, ax = plt.subplots(figsize=(12, 12))

        # 绘制房间填充
        import matplotlib.cm as cm

        groups = (
            self.room_groups
            if hasattr(self, "room_groups") and self.room_groups
            else [[room] for room in self.rooms]
        )
        colors = cm.rainbow(np.linspace(0, 1, len(groups)))

        for i, group in enumerate(groups):
            color = colors[i]
            group_geom = unary_union(group)
            for room in group:
                x, y = room.exterior.xy
                ax.fill(x, y, color=color, alpha=0.5)
                ax.plot(x, y, color="black", linewidth=1.5)

            cx, cy = group_geom.centroid.x, group_geom.centroid.y
            ax.text(cx, cy, str(i + 1), fontsize=12, ha="center", fontweight="bold", color="black")

        # 绘制原始网络作为参考 (灰色虚线)
        if hasattr(self, "unified_network"):
            for line in self.unified_network.geoms:
                x, y = line.xy
                ax.plot(x, y, color="gray", linewidth=1, linestyle="--", alpha=0.5)

        ax.set_title("Generated Room Layout", fontsize=15)
        ax.set_aspect("equal")
        if save_path:
            plt.savefig(save_path, bbox_inches="tight", dpi=300)
            print(f"Room visualization saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def visualize_final_layout(self, save_path=None):
        """
        最终可视化：
        1. 房间：同源同色，虚线边界。
        2. 构件：不同类型不同颜色，粗实线覆盖。
        """
        if not hasattr(self, "room_groups"):
            print("请先执行 generate_rooms()")
            return

        fig, ax = plt.subplots(figsize=(16, 12), dpi=100)
        ax.set_facecolor("#f8f9fa")  # 极淡的背景色

        # ==========================================
        # Layer 1: 房间填充 (同源同色)
        # ==========================================
        # 使用柔和的调色板 (Pastel1, Set3 等)
        # cmap = cm.get_cmap("Pastel1")
        # 或者自定义一组柔和颜色
        soft_colors = ["#FFB3BA", "#FFDFBA", "#FFFFBA", "#BAFFC9", "#BAE1FF", "#E2F0CB", "#FFDAC1"]

        for i, group in enumerate(self.room_groups):
            # 为这一组分配一个颜色
            color = soft_colors[i % len(soft_colors)]

            for rect in group:
                x, y = rect.exterior.xy
                # 填充
                ax.fill(x, y, color=color, alpha=0.6, zorder=1)
                # 虚线边界 (细)
                ax.plot(x, y, color="green", linestyle="--", linewidth=0.8, zorder=2)

        # ==========================================
        # Layer 2: 建筑构件 (粗实线)
        # ==========================================
        # 定义样式字典
        style_map = {
            "STRUCTURAL_WALL": {
                "color": "red",
                "lw": 2,
                "label": "Structural Wall",
                "z": 10,
            },  # 深蓝灰，最粗
            "INFILL_WALL": {"color": "#7F8C8D", "lw": 2.0, "label": "Infill Wall", "z": 9},  # 灰色
            "DOOR": {"color": "blue", "lw": 2, "label": "Door", "z": 11},  # 亮蓝
            "WINDOW": {"color": "green", "lw": 2, "label": "Window", "z": 11},  # 亮绿
        }

        # 辅助集合用于生成图例
        legend_patches = {}

        for seg in self.all_segments:
            # 确定类型 key
            if seg.seg_type == SegmentType.WALL:
                key = "STRUCTURAL_WALL" if seg.is_structural else "INFILL_WALL"
            elif seg.seg_type == SegmentType.DOOR:
                key = "DOOR"
            elif seg.seg_type == SegmentType.WINDOW:
                key = "WINDOW"
            else:
                continue

            s = style_map[key]
            x, y = seg.geometry.xy

            # 绘制
            ax.plot(x, y, color=s["color"], linewidth=s["lw"], solid_capstyle="butt", zorder=s["z"])

            # 记录图例
            if key not in legend_patches:
                legend_patches[key] = mpatches.Patch(color=s["color"], label=s["label"])

        # ==========================================
        # 图表修饰
        # ==========================================
        ax.set_aspect("equal")
        ax.set_title("Semantic Layout Reconstruction", fontsize=16, fontweight="bold", pad=20)

        # 去除坐标轴刻度，只保留网格
        ax.set_xticks([])
        ax.set_yticks([])
        # ax.grid(True, linestyle=':', alpha=0.3, color='gray') # 网格可选

        # 生成自定义图例
        handles = list(legend_patches.values())
        # 添加一个代表"Room Area"的图例
        handles.append(
            mpatches.Patch(facecolor="#FFB3BA", edgecolor="black", linestyle="--", label="Room Zone (Split)")
        )

        ax.legend(handles=handles, loc="upper right", framealpha=0.9, shadow=True, fontsize=10)

        if save_path:
            plt.savefig(save_path, bbox_inches="tight", dpi=300)
            print(f"Visualization saved to {save_path}")

        plt.show()


# =============================================================================
# 运行示例
# =============================================================================

if __name__ == "__main__":
    # DXF 文件路径；传入 JSON 时仍保留旧流程。
    cad_file = r"E:\Common\Desktop\test\ai-structures\case3\test.dxf"

    # 实例化处理流程
    processor = CADLayoutProcessor(cad_file, opening_layers=["WINDOW"])

    # 1. 构建几何 (将墙体线段转为面)
    processor.build_geometry()

    # 2. 提取中轴线 (需确保依赖函数存在)
    processor.extract_centerlines()

    # 3. 可视化结果
    # processor.visualize()
    processor.generate_rooms()
    # processor.visualize_final_layout()
    processor.visualize_rooms()
