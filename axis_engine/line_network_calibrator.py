from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
from shapely.affinity import translate
from shapely.geometry import LineString, MultiLineString, Point
from shapely.ops import linemerge, split, unary_union


class SegmentType(Enum):
    WALL = 0
    DOOR = 1
    WINDOW = 2
    BEAM = 3


@dataclass
class NetworkSegment:
    """带有语义信息的线段结构"""

    geometry: LineString
    thickness: float
    seg_type: SegmentType
    is_structural: bool = False


class LineNetworkCalibrator:
    def __init__(self, structural_thickness_threshold: float = 180.0):
        """
        Args:
            structural_thickness_threshold: 判定为承重墙的最小厚度 (如 200mm)
        """
        self.struct_thresh = structural_thickness_threshold
        self.x_grid: List[float] = []
        self.y_grid: List[float] = []

    def calibrate(self, raw_segments: List[NetworkSegment]) -> MultiLineString:
        """
        主流程：分级校准 + 拓扑缝合
        Args:
            raw_segments: [(LineString, thickness), ...] 的列表
        """

        # 2. 建立主导网格 (仅使用承重墙)
        self._build_grid_from_structural(raw_segments)

        # 3. 吸附对齐 (将所有构件吸附到网格)
        aligned_lines = []
        for seg in raw_segments:
            # 动态计算吸附阈值：承重墙厚度的一半
            # 也可以设定固定值，这里按你的需求逻辑：
            # 如果是填充墙，它离轴线的允许误差应该是承重墙的一半（保证能对齐到皮或中）
            snap_dist = self.struct_thresh

            new_geom = self._snap_segment(seg.geometry, snap_dist)
            seg.geometry = new_geom  # 更新几何体
            aligned_lines.append(new_geom)

        # 4. 拓扑缝合 (延伸相交)
        # 先合并一下共线的线段，减少计算量
        merged_lines = linemerge(aligned_lines)
        if isinstance(merged_lines, LineString):
            line_list = [merged_lines]
        else:
            line_list = list(merged_lines.geoms)

        final_network = self._stitch_connectivity(line_list, extension_dist=self.struct_thresh)

        return final_network

    def _build_grid_from_structural(self, segments: List[NetworkSegment]):
        """
        仅提取承重墙的坐标来构建“高置信度”网格
        """
        xs = []
        ys = []

        for seg in segments:
            if not seg.is_structural:
                continue

            p1, p2 = seg.geometry.coords[0], seg.geometry.coords[-1]
            dx = abs(p1[0] - p2[0])
            dy = abs(p1[1] - p2[1])

            # 提取竖直线段的 X
            if dx < 1.0 and dy > 10.0:
                xs.append(p1[0])
                xs.append(p2[0])  # 双保险
            # 提取水平线段的 Y
            elif dy < 1.0 and dx > 10.0:
                ys.append(p1[1])
                ys.append(p2[1])

        # 使用聚类去重 (复用你 room_calibrator 中的逻辑简化版)
        self.x_grid = self._cluster_coords(xs)
        self.y_grid = self._cluster_coords(ys)
        print(f"网格构建完成: X轴 {len(self.x_grid)} 条, Y轴 {len(self.y_grid)} 条")

    def _snap_segment(self, line: LineString, threshold: float) -> LineString:
        """
        将线段吸附到最近的网格线
        """
        p1, p2 = line.coords[0], line.coords[-1]
        x1, y1 = p1
        x2, y2 = p2

        is_vert = abs(x1 - x2) < abs(y1 - y2)

        new_x1, new_y1 = x1, y1
        new_x2, new_y2 = x2, y2

        # 简单策略：如果是竖直线，只修 X；如果是水平线，只修 Y
        # 斜线不处理，防止形状破坏

        if is_vert:
            # 寻找最近的 X 网格
            nearest_x = self._find_nearest(x1, self.x_grid, threshold)
            if nearest_x is not None:
                new_x1 = nearest_x
                new_x2 = nearest_x
        else:
            # 寻找最近的 Y 网格
            nearest_y = self._find_nearest(y1, self.y_grid, threshold)
            if nearest_y is not None:
                new_y1 = nearest_y
                new_y2 = nearest_y

        return LineString([(new_x1, new_y1), (new_x2, new_y2)])

    def _stitch_connectivity(self, lines: List[LineString], extension_dist: float) -> MultiLineString:
        """
        延伸线段以消除 T 型或 L 型断点
        原理：
        1. 找出所有端点。
        2. 如果端点没有和其他线连接（度数=1），则沿线段方向延伸 extension_dist。
        3. 如果延伸线与某条线相交，则截断在交点处。
        """

        # --- 最佳实践方案：基于 unary_union 的自动打断 ---
        # 我们手动延伸所有线段的两头
        candidates = []
        delta = extension_dist * 1.5

        for line in lines:
            if line.length < extension_dist * 2:
                continue
            # 使用辅助函数生成新的延伸线，而不是自己算向量
            extended_line = self.extend_linestring(line, delta)
            candidates.append(extended_line)

        # 3. 核心：由 Shapely 处理交点
        u = unary_union(candidates)

        return u  # 返回 MultiLineString

    def extend_linestring(self, line: LineString, distance: float) -> LineString:
        """
        仅延伸 LineString 的起点和终点，保持中间形状不变。
        """
        coords = list(line.coords)
        if len(coords) < 2:
            return line

        # --- 1. 处理起点 (Start) ---
        # 向量方向：从第二个点指向第一个点 (p1 -> p0)，即向外延伸
        p0 = np.array(coords[0])
        p1 = np.array(coords[1])
        vec_start = p0 - p1
        len_start = np.linalg.norm(vec_start)

        if len_start > 1e-6:  # 防止重合点导致除零
            u_start = vec_start / len_start
            new_p0 = p0 + u_start * distance
            coords[0] = tuple(new_p0)

        # --- 2. 处理终点 (End) ---
        # 向量方向：从倒数第二个点指向倒数第一个点 (p_n-1 -> p_n)，即向外延伸
        pend = np.array(coords[-1])
        pend_prev = np.array(coords[-2])
        vec_end = pend - pend_prev
        len_end = np.linalg.norm(vec_end)

        if len_end > 1e-6:
            u_end = vec_end / len_end
            new_pend = pend + u_end * distance
            coords[-1] = tuple(new_pend)

        return LineString(coords)

    def _cluster_coords(self, coords: List[float], tol: float = 50.0) -> List[float]:
        """一维坐标聚类 (K-Means simplified)"""
        if not coords:
            return []
        coords = sorted(coords)
        clusters = []
        if not coords:
            return []

        current_cluster = [coords[0]]
        for x in coords[1:]:
            if x - current_cluster[-1] <= tol:  # 这里的tol要严格，比如 50mm
                current_cluster.append(x)
            else:
                clusters.append(current_cluster)
                current_cluster = [x]
        clusters.append(current_cluster)

        return [sum(c) / len(c) for c in clusters]

    def _find_nearest(self, val: float, grid: List[float], threshold: float) -> Optional[float]:
        if not grid:
            return None
        # 简单遍历，网格线不会太多
        best_g = None
        min_d = float("inf")
        for g in grid:
            d = abs(val - g)
            if d < min_d:
                min_d = d
                best_g = g

        if min_d <= threshold:
            return best_g
        return None

    def _get_touching_lines(self, point, lines):
        return [l for l in lines if l.distance(point) < 1e-3]
