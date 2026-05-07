from typing import List, Set, Tuple

import numpy as np
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Point, Polygon
from shapely.ops import split


class RectangularDecomposer:
    """
    正交多边形矩形分解器
    目标：使用最少的切割线将多边形分解为矩形集合。
    """

    def __init__(self, tolerance=1.0, max_depth=100):
        self.tol = tolerance
        self.max_depth = max_depth

    def decompose(self, polygon: Polygon) -> List[Polygon]:
        """
        主入口：递归分解多边形
        """
        return self._decompose_recursive(polygon, depth=0)

    def _decompose_recursive(self, polygon: Polygon, depth: int) -> List[Polygon]:
        """
        递归分解多边形，带深度限制
        """
        # 防止无限递归
        if depth > self.max_depth:
            print(f"Warning: max recursion depth {self.max_depth} reached, returning polygon as-is")
            return [polygon] if not polygon.is_empty else []

        if polygon.is_empty:
            return []

        # 面积过小的多边形直接返回
        if polygon.area < 1e-6:
            return []

        # 0. 预处理：简化与修正
        poly = polygon.simplify(0.1).buffer(0)
        if poly.is_empty:
            return []

        if isinstance(poly, MultiPolygon):
            results = []
            for p in poly.geoms:
                results.extend(self._decompose_recursive(p, depth + 1))
            return results

        if isinstance(poly, GeometryCollection):
            results = []
            for geom in poly.geoms:
                if isinstance(geom, Polygon):
                    results.extend(self._decompose_recursive(geom, depth + 1))
            return results

        if not isinstance(poly, Polygon):
            return []

        # 1. 终止条件：如果已经是矩形，直接返回
        if self._is_rectangle(poly):
            return [poly]

        # 2. 寻找凹点 (Reflex Vertices)
        reflex_verts = self._get_reflex_vertices(poly)
        if not reflex_verts:
            # 如果没有凹点但不是矩形，返回原多边形
            return [poly]

        # 3. 寻找最佳切割线
        cut_line = self._find_best_cut(poly, reflex_verts)

        if cut_line is None:
            return [poly]

        # 4. 执行切割
        extended_cut = self._extend_line(cut_line)
        try:
            split_result = split(poly, extended_cut)

            # 获取有效的子多边形
            sub_polys = []
            for part in split_result.geoms:
                if isinstance(part, Polygon) and part.area > 1e-6:
                    sub_polys.append(part)

            # 检查切割是否有效（必须产生至少2个有效子多边形）
            if len(sub_polys) < 2:
                # 切割无效，尝试其他凹点
                alternative_cut = self._find_alternative_cut(poly, reflex_verts, cut_line)
                if alternative_cut is not None:
                    extended_alt = self._extend_line(alternative_cut)
                    split_result = split(poly, extended_alt)
                    sub_polys = []
                    for part in split_result.geoms:
                        if isinstance(part, Polygon) and part.area > 1e-6:
                            sub_polys.append(part)

            # 如果仍然无法有效切割，返回原多边形
            if len(sub_polys) < 2:
                return [poly]

            # 检查是否有子多边形与原多边形过于相似（防止无限递归）
            original_area = poly.area
            for sp in sub_polys:
                if abs(sp.area - original_area) / original_area < 0.01:
                    # 子多边形面积与原多边形几乎相同，说明切割无效
                    return [poly]

            # 5. 递归处理子区域
            rects = []
            for part in sub_polys:
                rects.extend(self._decompose_recursive(part, depth + 1))
            return rects

        except Exception as e:
            print(f"Split failed at depth {depth}: {e}")
            return [poly]

    def _is_rectangle(self, poly: Polygon) -> bool:
        """判断是否为矩形（容差范围内）"""
        # 矩形必须有4个点（不含闭合点5个坐标）且面积接近包围盒
        # 但 shapely 的 coords 可能包含共线点，所以用面积判定最稳
        return np.isclose(poly.area, poly.envelope.area, atol=1e-3)

    def _get_reflex_vertices(self, poly: Polygon) -> List[Tuple[float, float]]:
        """
        识别所有凹点（reflex vertices）
        凹点是内角大于180度的顶点
        """
        coords = list(poly.exterior.coords)[:-1]  # 去掉重复尾点
        n = len(coords)
        reflex_points = []

        # 检查多边形方向
        is_ccw = poly.exterior.is_ccw

        for i in range(n):
            p_prev = np.array(coords[(i - 1 + n) % n])
            p_curr = np.array(coords[i])
            p_next = np.array(coords[(i + 1) % n])

            v1 = p_curr - p_prev
            v2 = p_next - p_curr

            # 计算叉积 (2D Cross Product)
            cross = v1[0] * v2[1] - v1[1] * v2[0]

            # 对于逆时针(CCW)多边形，凹点的叉积 < 0
            # 对于顺时针(CW)多边形，凹点的叉积 > 0
            if is_ccw:
                is_reflex = cross < -1e-6
            else:
                is_reflex = cross > 1e-6

            if is_reflex:
                reflex_points.append(tuple(p_curr))

        return reflex_points

    def _find_best_cut(self, poly: Polygon, reflex_verts: List[Tuple]) -> LineString:
        """
        策略核心：
        Priority 1: 连接两个凹点的切割线 (Chord)
        Priority 2: 最短的切割线
        """
        candidates = self._generate_cut_candidates(poly, reflex_verts)

        if not candidates:
            return None

        # 评分系统
        best_cut = None
        best_score = float("inf")

        for seg in candidates:
            if seg.length < 1e-6:
                continue

            p1 = seg.coords[0]
            p2 = seg.coords[-1]
            length = seg.length

            # 检查两端是否都是凹点
            hits_reflex = 0
            for rv in reflex_verts:
                d1 = (p1[0] - rv[0]) ** 2 + (p1[1] - rv[1]) ** 2
                d2 = (p2[0] - rv[0]) ** 2 + (p2[1] - rv[1]) ** 2
                if d1 < 1.0 or d2 < 1.0:
                    hits_reflex += 1

            # 优先选择连接两个凹点的切割线
            score = length - (100000.0 if hits_reflex >= 2 else 0)

            if score < best_score:
                best_score = score
                best_cut = seg

        return best_cut

    def _find_alternative_cut(
        self, poly: Polygon, reflex_verts: List[Tuple], exclude_line: LineString
    ) -> LineString:
        """
        寻找替代切割线（排除已尝试过的）
        """
        candidates = self._generate_cut_candidates(poly, reflex_verts)

        if not candidates:
            return None

        best_cut = None
        best_score = float("inf")

        for seg in candidates:
            if seg.length < 1e-6:
                continue

            # 跳过与排除线相似的切割线
            if exclude_line is not None:
                if seg.distance(exclude_line) < 1.0 and abs(seg.length - exclude_line.length) < 1.0:
                    continue

            p1 = seg.coords[0]
            p2 = seg.coords[-1]
            length = seg.length

            hits_reflex = 0
            for rv in reflex_verts:
                d1 = (p1[0] - rv[0]) ** 2 + (p1[1] - rv[1]) ** 2
                d2 = (p2[0] - rv[0]) ** 2 + (p2[1] - rv[1]) ** 2
                if d1 < 1.0 or d2 < 1.0:
                    hits_reflex += 1

            score = length - (100000.0 if hits_reflex >= 2 else 0)

            if score < best_score:
                best_score = score
                best_cut = seg

        return best_cut

    def _generate_cut_candidates(self, poly: Polygon, reflex_verts: List[Tuple]) -> List[LineString]:
        """
        生成所有可能的切割线候选
        切割线必须穿过多边形内部，而不是沿着边界
        """
        candidates = []
        bounds = poly.bounds  # (minx, miny, maxx, maxy)
        boundary = poly.exterior

        for v in reflex_verts:
            vx, vy = v
            # 尝试生成水平和垂直的射线
            h_line = LineString([(bounds[0] - 10, vy), (bounds[2] + 10, vy)])
            v_line = LineString([(vx, bounds[1] - 10), (vx, bounds[3] + 10)])

            for ray in [h_line, v_line]:
                inter = poly.intersection(ray)

                if inter.is_empty:
                    continue

                segs = inter.geoms if hasattr(inter, "geoms") else [inter]
                for seg in segs:
                    if not isinstance(seg, LineString):
                        continue

                    # 检查这条线段是否包含凹点
                    if seg.distance(Point(v)) > 1e-3:
                        continue

                    # 关键修改：检查这条线段是否在多边形内部
                    # 如果线段完全在边界上，则跳过
                    seg_midpoint = seg.interpolate(0.5, normalized=True)

                    # 检查中点是否严格在多边形内部（不在边界上）
                    if boundary.distance(seg_midpoint) < 1e-6:
                        # 线段中点在边界上，说明这是沿着边界的线段
                        # 但我们需要进一步检查：线段是否部分在内部
                        # 尝试从凹点延伸到线段的另一端
                        seg_coords = list(seg.coords)
                        p1, p2 = seg_coords[0], seg_coords[-1]

                        # 判断哪个端点是凹点
                        d1 = (p1[0] - vx) ** 2 + (p1[1] - vy) ** 2
                        d2 = (p2[0] - vx) ** 2 + (p2[1] - vy) ** 2

                        if d1 < 1e-6:
                            other_end = p2
                        elif d2 < 1e-6:
                            other_end = p1
                        else:
                            continue

                        # 检查从凹点到另一端的中点是否在多边形内部
                        mid_x = (vx + other_end[0]) / 2
                        mid_y = (vy + other_end[1]) / 2
                        mid_point = Point(mid_x, mid_y)

                        # 如果中点在多边形内部（不仅仅是在边界上），这是有效的切割线
                        if poly.contains(mid_point) and boundary.distance(mid_point) > 1e-6:
                            candidates.append(seg)
                    else:
                        # 线段中点不在边界上，说明是穿过内部的线段
                        candidates.append(seg)

        return candidates

    def _extend_line(self, line):
        """稍微延长切割线以确保 split 成功"""
        p1 = np.array(line.coords[0])
        p2 = np.array(line.coords[-1])
        vec = p2 - p1
        vec = vec / np.linalg.norm(vec) * 0.1  # 延长 0.1 单位
        return LineString([p1 - vec, p2 + vec])


# =======================
# 集成示例
# =======================
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # 创建一个 L 型多边形
    L_shape = Polygon([(0, 0), (10, 0), (10, 4), (4, 4), (4, 10), (0, 10)])

    print(f"L形多边形顶点: {list(L_shape.exterior.coords)}")
    print(f"多边形是否逆时针: {L_shape.exterior.is_ccw}")
    print(f"多边形面积: {L_shape.area}")
    print(f"包围盒面积: {L_shape.envelope.area}")

    decomposer = RectangularDecomposer()

    # 调试：检测凹点
    reflex = decomposer._get_reflex_vertices(L_shape)
    print(f"检测到的凹点: {reflex}")

    # 调试：检查切割候选
    if reflex:
        candidates = decomposer._generate_cut_candidates(L_shape, reflex)
        print(f"切割候选数量: {len(candidates)}")
        for i, c in enumerate(candidates):
            print(f"  候选 {i}: {list(c.coords)}, 长度: {c.length:.2f}")

    rects = decomposer.decompose(L_shape)

    print(f"分解为 {len(rects)} 个矩形")

    # 可视化
    fig, ax = plt.subplots()
    for i, rect in enumerate(rects):
        x, y = rect.exterior.xy
        ax.plot(x, y, color="black")
        ax.fill(x, y, alpha=0.3, label=f"Rect {i}")
        cx, cy = rect.centroid.x, rect.centroid.y
        ax.text(cx, cy, f"{i}", ha="center")

    ax.set_aspect("equal")
    plt.show()
