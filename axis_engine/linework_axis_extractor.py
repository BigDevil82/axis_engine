from __future__ import annotations

from collections import Counter
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from numbers import Integral

from shapely.geometry import LineString, MultiLineString, box
from shapely.ops import linemerge, unary_union
from shapely.strtree import STRtree


@dataclass(frozen=True)
class AxisAlignedEdge:
    axis: str
    const: float
    start: float
    end: float

    @property
    def length(self) -> float:
        return self.end - self.start


@dataclass(frozen=True)
class OppositeEdgePair:
    first: AxisAlignedEdge
    second: AxisAlignedEdge
    thickness: float
    overlap_start: float
    overlap_end: float
    score: float


def repair_wall_linework(
    wall_segments: Iterable[LineString],
    axis_tolerance: float = 5.0,
    snap_tolerance: float = 3.0,
    min_segment_length: float = 20.0,
) -> list[AxisAlignedEdge]:
    """
    Normalize primitive CAD wall linework into merged horizontal/vertical edges.

    This intentionally keeps the representation as linework. It does not require
    closed polygons.
    """
    edges: list[AxisAlignedEdge] = []
    for line in wall_segments:
        coords = list(line.coords)
        for start, end in zip(coords, coords[1:]):
            x1, y1 = start[:2]
            x2, y2 = end[:2]
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            length = max(dx, dy)
            if length < min_segment_length:
                continue

            if dy <= axis_tolerance and dx >= dy:
                y = _snap((y1 + y2) / 2.0, snap_tolerance)
                a, b = sorted((_snap(x1, snap_tolerance), _snap(x2, snap_tolerance)))
                if b - a >= min_segment_length:
                    edges.append(AxisAlignedEdge("h", y, a, b))
            elif dx <= axis_tolerance and dy >= dx:
                x = _snap((x1 + x2) / 2.0, snap_tolerance)
                a, b = sorted((_snap(y1, snap_tolerance), _snap(y2, snap_tolerance)))
                if b - a >= min_segment_length:
                    edges.append(AxisAlignedEdge("v", x, a, b))

    return _merge_collinear_edges(edges, snap_tolerance)


def infer_wall_thicknesses(
    edges: Sequence[AxisAlignedEdge],
    min_thickness: float = 80.0,
    max_thickness: float = 400.0,
    bin_size: float = 50.0,
    min_overlap: float = 80.0,
    max_count: int = 5,
) -> list[float]:
    """Infer common wall thicknesses from distances between overlapping parallel edges."""
    weights: Counter[float] = Counter()
    for axis in ("h", "v"):
        axis_edges = sorted([edge for edge in edges if edge.axis == axis], key=lambda item: item.const)
        for i, first in enumerate(axis_edges):
            for second in axis_edges[i + 1 :]:
                gap = second.const - first.const
                if gap > max_thickness:
                    break
                if gap < min_thickness:
                    continue
                overlap = _overlap_length(first, second)
                if overlap < min_overlap:
                    continue
                bucket = round(gap / bin_size) * bin_size
                if min_thickness <= bucket <= max_thickness:
                    weights[bucket] += max(1, int(overlap / 100.0))

    if not weights:
        return [200.0]

    return [thickness for thickness, _count in weights.most_common(max_count)]


def extract_wall_axes_from_linework(
    wall_segments: Iterable[LineString],
    thickness_candidates: Sequence[float] | None = None,
    axis_tolerance: float = 5.0,
    snap_tolerance: float = 3.0,
    thickness_tolerance_ratio: float = 0.10,
    thickness_tolerance_abs: float = 12.0,
    min_overlap: float = 30.0,
    min_feature_len: float = 20.0,
    min_score: float = 1.35,
    alignment_tolerance: float = 80.0,
    connection_tolerance: float = 160.0,
    merge_axes: bool = True,
) -> list[tuple[LineString, float]]:
    """
    Extract wall axes directly from raw CAD wall boundary segments.

    The algorithm pairs opposite horizontal/vertical boundary edges and emits
    centerlines for their overlapping span. Polygon reconstruction is not a
    prerequisite.
    """
    edges = repair_wall_linework(
        wall_segments,
        axis_tolerance=axis_tolerance,
        snap_tolerance=snap_tolerance,
        min_segment_length=min_feature_len,
    )
    if not edges:
        return []

    thicknesses = list(
        thickness_candidates or infer_wall_thicknesses(edges, min_overlap=max(80.0, min_overlap))
    )
    pairs = find_opposite_edge_pairs(
        edges,
        thicknesses,
        thickness_tolerance_ratio=thickness_tolerance_ratio,
        thickness_tolerance_abs=thickness_tolerance_abs,
        min_overlap=min_overlap,
        min_score=min_score,
    )

    axes = [_axis_from_pair(pair) for pair in pairs]
    axes = heal_axis_connections(
        axes,
        alignment_tolerance=alignment_tolerance,
        connection_tolerance=connection_tolerance,
    )
    if merge_axes:
        axes = _merge_axis_lines(axes, snap_tolerance)
    return axes


def heal_axis_connections(
    axes: Sequence[tuple[LineString, float]],
    alignment_tolerance: float = 80.0,
    connection_tolerance: float = 160.0,
) -> list[tuple[LineString, float]]:
    if not axes:
        return []

    normalized = [_normalize_axis_line(line, thickness) for line, thickness in axes if line.length > 0]
    groups = _group_axes_by_influence(normalized, connection_tolerance)
    healed: list[tuple[LineString, float, str]] = []
    for group in groups:
        aligned = _align_parallel_axes(group, alignment_tolerance)
        healed.extend(_connect_orthogonal_axes(aligned, connection_tolerance))
    return [(item[0], item[1]) for item in healed if item[0].length > 0]


def find_opposite_edge_pairs(
    edges: Sequence[AxisAlignedEdge],
    thickness_candidates: Sequence[float],
    thickness_tolerance_ratio: float = 0.10,
    thickness_tolerance_abs: float = 12.0,
    min_overlap: float = 30.0,
    min_score: float = 1.35,
) -> list[OppositeEdgePair]:
    candidates: list[OppositeEdgePair] = []
    for axis in ("h", "v"):
        axis_edges = sorted([edge for edge in edges if edge.axis == axis], key=lambda item: item.const)
        max_thickness = max(thickness_candidates) + max(
            thickness_tolerance_abs,
            max(thickness_candidates) * thickness_tolerance_ratio,
        )
        for index, first in enumerate(axis_edges):
            for second in axis_edges[index + 1 :]:
                gap = second.const - first.const
                if gap > max_thickness:
                    break

                thickness = _matched_thickness(
                    gap,
                    thickness_candidates,
                    thickness_tolerance_ratio,
                    thickness_tolerance_abs,
                )
                if thickness is None:
                    continue

                overlap_start = max(first.start, second.start)
                overlap_end = min(first.end, second.end)
                overlap = overlap_end - overlap_start
                if overlap < min_overlap:
                    continue

                if _has_blocking_edge(first, second, axis_edges, overlap_start, overlap_end, min_overlap):
                    continue

                score = _pair_score(first, second, gap, thickness, overlap)
                if score < min_score:
                    continue

                candidates.append(
                    OppositeEdgePair(
                        first=first,
                        second=second,
                        thickness=thickness,
                        overlap_start=overlap_start,
                        overlap_end=overlap_end,
                        score=score,
                    )
                )

    return _select_non_conflicting_pairs(candidates)


def _select_non_conflicting_pairs(candidates: Sequence[OppositeEdgePair]) -> list[OppositeEdgePair]:
    selected: list[OppositeEdgePair] = []
    used_intervals: dict[AxisAlignedEdge, list[tuple[float, float]]] = {}

    for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
        interval = (candidate.overlap_start, candidate.overlap_end)
        if _edge_interval_conflicts(candidate.first, interval, used_intervals):
            continue
        if _edge_interval_conflicts(candidate.second, interval, used_intervals):
            continue

        selected.append(candidate)
        used_intervals.setdefault(candidate.first, []).append(interval)
        used_intervals.setdefault(candidate.second, []).append(interval)

    return selected


def _edge_interval_conflicts(
    edge: AxisAlignedEdge,
    interval: tuple[float, float],
    used_intervals: dict[AxisAlignedEdge, list[tuple[float, float]]],
) -> bool:
    length = interval[1] - interval[0]
    for used_start, used_end in used_intervals.get(edge, []):
        overlap = min(interval[1], used_end) - max(interval[0], used_start)
        if overlap > max(10.0, length * 0.35):
            return True
    return False


def _has_blocking_edge(
    first: AxisAlignedEdge,
    second: AxisAlignedEdge,
    edges: Sequence[AxisAlignedEdge],
    overlap_start: float,
    overlap_end: float,
    min_overlap: float,
) -> bool:
    for edge in edges:
        if edge == first or edge == second:
            continue
        if not first.const < edge.const < second.const:
            continue
        overlap = min(overlap_end, edge.end) - max(overlap_start, edge.start)
        if overlap >= min_overlap:
            return True
    return False


def _pair_score(
    edge: AxisAlignedEdge, other: AxisAlignedEdge, gap: float, thickness: float, overlap: float
) -> float:
    thickness_error = abs(gap - thickness) / max(thickness, 1.0)
    overlap_ratio = overlap / max(min(edge.length, other.length), 1.0)
    length_support = min(overlap / max(thickness, 1.0), 3.0) / 3.0
    return (1.0 - thickness_error) + overlap_ratio + length_support


def _axis_from_pair(pair: OppositeEdgePair) -> tuple[LineString, float]:
    const = (pair.first.const + pair.second.const) / 2.0
    if pair.first.axis == "h":
        line = LineString([(pair.overlap_start, const), (pair.overlap_end, const)])
    else:
        line = LineString([(const, pair.overlap_start), (const, pair.overlap_end)])
    return line, pair.thickness


def _merge_axis_lines(
    axes: Sequence[tuple[LineString, float]],
    snap_tolerance: float,
) -> list[tuple[LineString, float]]:
    by_thickness: dict[float, list[LineString]] = {}
    for line, thickness in axes:
        if line.length <= 0:
            continue
        by_thickness.setdefault(thickness, []).append(_snap_line(line, snap_tolerance))

    merged_axes: list[tuple[LineString, float]] = []
    for thickness, lines in by_thickness.items():
        unified = unary_union(lines)
        if isinstance(unified, LineString):
            merged_axes.append((unified, thickness))
            continue
        merged = linemerge(unary_union(lines))
        if isinstance(merged, LineString):
            merged_axes.append((merged, thickness))
        elif isinstance(merged, MultiLineString):
            merged_axes.extend((line, thickness) for line in merged.geoms if line.length > 0)

    return merged_axes


def _normalize_axis_line(line: LineString, thickness: float) -> tuple[LineString, float, str]:
    coords = list(line.coords)
    start = coords[0]
    end = coords[-1]
    if abs(start[1] - end[1]) <= abs(start[0] - end[0]):
        y = (start[1] + end[1]) / 2.0
        x1, x2 = sorted((start[0], end[0]))
        return (LineString([(x1, y), (x2, y)]), thickness, "h")

    x = (start[0] + end[0]) / 2.0
    y1, y2 = sorted((start[1], end[1]))
    return (LineString([(x, y1), (x, y2)]), thickness, "v")


def _group_axes_by_influence(
    axes: Sequence[tuple[LineString, float, str]],
    connection_tolerance: float,
) -> list[list[tuple[LineString, float, str]]]:
    if not axes:
        return []

    influence_geoms = []
    for line, thickness, direction in axes:
        influence_geoms.append(_axis_influence_geometry(line, thickness, direction, connection_tolerance))

    merged = unary_union(influence_geoms)
    if merged.is_empty:
        return [list(axes)]
    components = list(merged.geoms) if hasattr(merged, "geoms") else [merged]

    groups: list[list[tuple[LineString, float, str]]] = [[] for _ in components]
    for axis, influence in zip(axes, influence_geoms):
        best_index = None
        best_area = -1.0
        for index, component in enumerate(components):
            if not influence.intersects(component):
                continue
            area = influence.intersection(component).area
            if area > best_area:
                best_area = area
                best_index = index
        if best_index is None:
            groups.append([axis])
        else:
            groups[best_index].append(axis)

    return [group for group in groups if group]


def _axis_influence_geometry(
    line: LineString,
    thickness: float,
    direction: str,
    connection_tolerance: float,
):
    extended = _extend_axis_line(line, direction, max(thickness, connection_tolerance * 0.5))
    return extended.buffer(thickness / 2.0 + connection_tolerance * 0.15, cap_style=2, join_style=2)


def _extend_axis_line(line: LineString, direction: str, distance: float) -> LineString:
    coords = list(line.coords)
    if direction == "h":
        x1, x2 = sorted((coords[0][0], coords[-1][0]))
        y = _axis_const(line, direction)
        return LineString([(x1 - distance, y), (x2 + distance, y)])
    y1, y2 = sorted((coords[0][1], coords[-1][1]))
    x = _axis_const(line, direction)
    return LineString([(x, y1 - distance), (x, y2 + distance)])


def _align_parallel_axes(
    axes: Sequence[tuple[LineString, float, str]],
    alignment_tolerance: float,
) -> list[tuple[LineString, float, str]]:
    aligned: list[tuple[LineString, float, str]] = []
    for direction in ("h", "v"):
        direction_axes = [axis for axis in axes if axis[2] == direction]
        groups = _group_parallel_axes(direction_axes, alignment_tolerance)
        for group in groups:
            reference_line, _reference_thickness, _direction = max(group, key=lambda item: item[0].length)
            reference_const = _axis_const(reference_line, direction)
            for line, thickness, axis_direction in group:
                aligned.append((_set_axis_const(line, direction, reference_const), thickness, axis_direction))

    return aligned


def _group_parallel_axes(
    axes: Sequence[tuple[LineString, float, str]],
    tolerance: float,
) -> list[list[tuple[LineString, float, str]]]:
    groups: list[list[tuple[LineString, float, str]]] = []
    sorted_axes = sorted(axes, key=lambda item: _axis_const(item[0], item[2]))
    for axis in sorted_axes:
        axis_const = _axis_const(axis[0], axis[2])
        target_group = None
        for group in groups:
            group_const = _weighted_group_const(group)
            if abs(axis_const - group_const) <= tolerance and _has_projection_support(axis, group):
                target_group = group
                break
        if target_group is None:
            groups.append([axis])
        else:
            target_group.append(axis)
    return groups


def _connect_orthogonal_axes(
    axes: Sequence[tuple[LineString, float, str]],
    connection_tolerance: float,
) -> list[tuple[LineString, float, str]]:
    mutable = [[list(line.coords), thickness, direction] for line, thickness, direction in axes]
    vertical_items = [(index, item) for index, item in enumerate(mutable) if item[2] == "v"]
    if not vertical_items:
        return [(LineString(coords), thickness, direction) for coords, thickness, direction in mutable]

    vertical_geoms = [LineString(item[0]) for _index, item in vertical_items]
    tree = STRtree(vertical_geoms)

    for h_item in mutable:
        if h_item[2] != "h":
            continue

        hx1, hy = h_item[0][0]
        hx2, _ = h_item[0][-1]
        h_min, h_max = sorted((hx1, hx2))
        search = box(
            h_min - connection_tolerance,
            hy - connection_tolerance,
            h_max + connection_tolerance,
            hy + connection_tolerance,
        )

        for query_result in tree.query(search):
            local_index = (
                int(query_result)
                if isinstance(query_result, Integral)
                else vertical_geoms.index(query_result)
            )
            _global_index, v_item = vertical_items[local_index]
            vx, vy1 = v_item[0][0]
            _, vy2 = v_item[0][-1]
            v_min, v_max = sorted((vy1, vy2))

            if not _near_orthogonal_cross(h_min, h_max, hy, vx, v_min, v_max, connection_tolerance):
                continue

            h_min = min(h_min, vx)
            h_max = max(h_max, vx)
            v_min = min(v_min, hy)
            v_max = max(v_max, hy)
            h_item[0] = [(h_min, hy), (h_max, hy)]
            v_item[0] = [(vx, v_min), (vx, v_max)]

    healed = []
    for coords, thickness, direction in mutable:
        line = LineString(coords)
        if line.length > 0:
            healed.append((line, thickness, direction))
    return healed


def _near_orthogonal_cross(
    h_min: float,
    h_max: float,
    hy: float,
    vx: float,
    v_min: float,
    v_max: float,
    tolerance: float,
) -> bool:
    horizontal_gap = 0.0
    if vx < h_min:
        horizontal_gap = h_min - vx
    elif vx > h_max:
        horizontal_gap = vx - h_max

    vertical_gap = 0.0
    if hy < v_min:
        vertical_gap = v_min - hy
    elif hy > v_max:
        vertical_gap = hy - v_max

    return (
        horizontal_gap <= tolerance
        and vertical_gap <= tolerance
        and max(horizontal_gap, vertical_gap) <= tolerance
    )


def _weighted_group_const(group: Sequence[tuple[LineString, float, str]]) -> float:
    total_length = sum(item[0].length for item in group)
    if total_length <= 0:
        return _axis_const(group[0][0], group[0][2])
    return sum(_axis_const(item[0], item[2]) * item[0].length for item in group) / total_length


def _has_projection_support(
    axis: tuple[LineString, float, str],
    group: Sequence[tuple[LineString, float, str]],
) -> bool:
    direction = axis[2]
    axis_start, axis_end = _axis_interval(axis[0], direction)
    for item in group:
        item_start, item_end = _axis_interval(item[0], direction)
        overlap = min(axis_end, item_end) - max(axis_start, item_start)
        gap = max(item_start - axis_end, axis_start - item_end, 0.0)
        if overlap > 0 or gap <= max(axis[1], item[1]) * 0.75:
            return True
    return False


def _axis_const(line: LineString, direction: str) -> float:
    coords = list(line.coords)
    if direction == "h":
        return (coords[0][1] + coords[-1][1]) / 2.0
    return (coords[0][0] + coords[-1][0]) / 2.0


def _set_axis_const(line: LineString, direction: str, const: float) -> LineString:
    coords = list(line.coords)
    if direction == "h":
        x1, x2 = sorted((coords[0][0], coords[-1][0]))
        return LineString([(x1, const), (x2, const)])
    y1, y2 = sorted((coords[0][1], coords[-1][1]))
    return LineString([(const, y1), (const, y2)])


def _axis_interval(line: LineString, direction: str) -> tuple[float, float]:
    coords = list(line.coords)
    if direction == "h":
        return tuple(sorted((coords[0][0], coords[-1][0])))
    return tuple(sorted((coords[0][1], coords[-1][1])))


def _snap_line(line: LineString, tolerance: float) -> LineString:
    return LineString([(_snap(x, tolerance), _snap(y, tolerance)) for x, y in line.coords])


def _matched_thickness(
    distance: float,
    thickness_candidates: Sequence[float],
    tolerance_ratio: float,
    tolerance_abs: float,
) -> float | None:
    best = None
    best_error = float("inf")
    for thickness in thickness_candidates:
        tolerance = max(tolerance_abs, thickness * tolerance_ratio)
        error = abs(distance - thickness)
        if error <= tolerance and error < best_error:
            best = thickness
            best_error = error
    return best


def _overlap_length(first: AxisAlignedEdge, second: AxisAlignedEdge) -> float:
    return min(first.end, second.end) - max(first.start, second.start)


def _snap(value: float, tolerance: float) -> float:
    if tolerance <= 0:
        return float(value)
    return round(float(value) / tolerance) * tolerance


def _merge_collinear_edges(
    edges: Iterable[AxisAlignedEdge],
    gap_tolerance: float,
) -> list[AxisAlignedEdge]:
    intervals_by_line: dict[tuple[str, float], list[tuple[float, float]]] = {}
    for edge in edges:
        intervals_by_line.setdefault((edge.axis, edge.const), []).append((edge.start, edge.end))

    merged: list[AxisAlignedEdge] = []
    for (axis, const), intervals in intervals_by_line.items():
        intervals.sort()
        current_start = None
        current_end = None
        for start, end in intervals:
            if current_start is None:
                current_start = start
                current_end = end
                continue
            if start <= current_end + gap_tolerance:
                current_end = max(current_end, end)
            else:
                merged.append(AxisAlignedEdge(axis, const, current_start, current_end))
                current_start = start
                current_end = end
        if current_start is not None:
            merged.append(AxisAlignedEdge(axis, const, current_start, current_end))

    return merged
