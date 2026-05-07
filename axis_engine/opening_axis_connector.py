from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from numbers import Integral

from shapely.geometry import LineString, MultiLineString
from shapely.ops import unary_union
from shapely.strtree import STRtree


@dataclass(frozen=True)
class AxisSegment:
    line: LineString
    thickness: float
    direction: str
    const: float
    start: float
    end: float


def infer_opening_axis_connectors(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_segments: Iterable[LineString],
    opening_buffer: float = 120.0,
    axis_snap_tolerance: float = 120.0,
    max_gap: float = 3000.0,
    min_gap: float = 80.0,
) -> list[tuple[LineString, float]]:
    """
    Infer opening connectors from door/window linework and nearby wall axes.

    The algorithm does not classify opening shapes. Opening primitives are used
    only as spatial evidence that a gap between two collinear wall axes should
    be bridged.
    """
    axes = [_normalize_axis(line, thickness) for line, thickness in wall_axes if line.length > 0]
    opening_lines = _filter_opening_lines_near_axes(
        [line for line in opening_segments if line.length > 0],
        axes,
        max_distance=max(opening_buffer * 4.0, axis_snap_tolerance * 4.0),
    )
    if not axes or not opening_lines:
        return []

    opening_geom = unary_union(opening_lines).buffer(opening_buffer, cap_style=2, join_style=2)
    opening_components = list(opening_geom.geoms) if hasattr(opening_geom, "geoms") else [opening_geom]

    opening_axes: list[tuple[LineString, float]] = []
    for direction in ("h", "v"):
        direction_axes = [axis for axis in axes if axis.direction == direction]
        opening_axes.extend(
            _connect_axis_gaps_for_direction(
                direction_axes,
                opening_components,
                direction,
                axis_snap_tolerance,
                min_gap,
                max_gap,
            )
        )

    return opening_axes


def infer_opening_axes_from_groups(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_groups: Sequence[Sequence[LineString]],
    opening_buffer: float = 180.0,
    endpoint_tolerance: float = 260.0,
    axis_const_tolerance: float = 140.0,
    max_gap: float = 3200.0,
    min_gap: float = 80.0,
    wall_overlap_ratio: float = 0.25,
) -> list[tuple[LineString, float]]:
    axes = [_normalize_axis(line, thickness) for line, thickness in wall_axes if line.length > 0]
    if not axes or not opening_groups:
        return []

    wall_axis_lines = [axis.line for axis in axes]
    wall_tree = STRtree(wall_axis_lines)
    result: list[tuple[LineString, float]] = []
    seen = set()

    for group in opening_groups:
        group_lines = [line for line in group if line.length > 0]
        if not group_lines:
            continue

        group_geom = unary_union(group_lines)
        search_geom = group_geom.buffer(opening_buffer, cap_style=2, join_style=2)
        nearby_axes = [axes[index] for index in _query_tree_indices(wall_tree, wall_axis_lines, search_geom)]
        if len(nearby_axes) < 2:
            continue

        candidates = _opening_candidates_from_axis_endpoints(
            nearby_axes,
            group_geom,
            endpoint_tolerance,
            axis_const_tolerance,
            min_gap,
            max_gap,
        )
        candidates.extend(
            _opening_candidates_from_axis_overlap(
                nearby_axes,
                group_geom,
                endpoint_tolerance,
                min_gap,
                max_gap,
            )
        )
        for line, thickness, score in sorted(candidates, key=lambda item: item[2], reverse=True):
            if _overlaps_existing_wall_axis(line, wall_axis_lines, wall_overlap_ratio) and line.length > max_gap * 0.8:
                continue
            key = _line_key(line)
            if key in seen:
                continue
            seen.add(key)
            result.append((line, thickness))
            break

    return result


def merge_wall_axes_with_connectors(
    wall_axes: Sequence[tuple[LineString, float]],
    connectors: Sequence[tuple[LineString, float]],
) -> list[tuple[LineString, float]]:
    by_thickness: dict[float, list[LineString]] = {}
    for line, thickness in list(wall_axes) + list(connectors):
        by_thickness.setdefault(thickness, []).append(line)

    merged_axes: list[tuple[LineString, float]] = []
    for thickness, lines in by_thickness.items():
        merged = unary_union(lines)
        merged = merged if isinstance(merged, LineString) else MultiLineString(list(merged.geoms))
        merged = merged if isinstance(merged, LineString) else merged
        merged = _linemerge_safe(merged)
        if isinstance(merged, LineString):
            merged_axes.append((merged, thickness))
        elif isinstance(merged, MultiLineString):
            merged_axes.extend((line, thickness) for line in merged.geoms if line.length > 0)
    return merged_axes


def _connect_axis_gaps_for_direction(
    axes: Sequence[AxisSegment],
    opening_components,
    direction: str,
    axis_snap_tolerance: float,
    min_gap: float,
    max_gap: float,
) -> list[tuple[LineString, float]]:
    result: list[tuple[LineString, float]] = []
    if len(axes) < 2:
        return result

    opening_tree = STRtree(opening_components)
    groups = _group_axes_by_const(axes, axis_snap_tolerance)
    for group in groups:
        reference_const = _reference_const(group)
        sorted_group = sorted(group, key=lambda item: item.start)
        for first, second in zip(sorted_group, sorted_group[1:]):
            gap_start = first.end
            gap_end = second.start
            gap = gap_end - gap_start
            if gap < min_gap or gap > max_gap:
                continue

            endpoint_line = _line_from_interval(direction, reference_const, gap_start, gap_end)
            search = endpoint_line.buffer(
                max(axis_snap_tolerance, max(first.thickness, second.thickness)), cap_style=2
            )
            if not any(
                component.intersects(search)
                for component in _query_tree(opening_tree, opening_components, search)
            ):
                continue

            result.append((endpoint_line, max(first.thickness, second.thickness)))

    return result


def _group_axes_by_const(axes: Sequence[AxisSegment], tolerance: float) -> list[list[AxisSegment]]:
    groups: list[list[AxisSegment]] = []
    for axis in sorted(axes, key=lambda item: item.const):
        target = None
        for group in groups:
            if abs(axis.const - _reference_const(group)) <= tolerance:
                target = group
                break
        if target is None:
            groups.append([axis])
        else:
            target.append(axis)
    return groups


def _reference_const(group: Sequence[AxisSegment]) -> float:
    longest = max(group, key=lambda item: item.line.length)
    return longest.const


def _normalize_axis(line: LineString, thickness: float) -> AxisSegment:
    coords = list(line.coords)
    start = coords[0]
    end = coords[-1]
    if abs(start[1] - end[1]) <= abs(start[0] - end[0]):
        x1, x2 = sorted((start[0], end[0]))
        y = (start[1] + end[1]) / 2.0
        normalized = LineString([(x1, y), (x2, y)])
        return AxisSegment(normalized, thickness, "h", y, x1, x2)

    y1, y2 = sorted((start[1], end[1]))
    x = (start[0] + end[0]) / 2.0
    normalized = LineString([(x, y1), (x, y2)])
    return AxisSegment(normalized, thickness, "v", x, y1, y2)


def _opening_candidates_from_axis_endpoints(
    axes: Sequence[AxisSegment],
    group_geom,
    endpoint_tolerance: float,
    axis_const_tolerance: float,
    min_gap: float,
    max_gap: float,
) -> list[tuple[LineString, float, float]]:
    candidates: list[tuple[LineString, float, float]] = []

    for direction in ("h", "v"):
        direction_axes = [axis for axis in axes if axis.direction == direction]
        endpoints = []
        for axis in direction_axes:
            endpoints.append((axis, axis.start))
            endpoints.append((axis, axis.end))

        for index, (first_axis, first_pos) in enumerate(endpoints):
            for second_axis, second_pos in endpoints[index + 1 :]:
                if first_axis == second_axis:
                    continue
                if abs(first_axis.const - second_axis.const) > axis_const_tolerance:
                    continue

                gap = abs(second_pos - first_pos)
                if gap < min_gap or gap > max_gap:
                    continue

                const = _weighted_const(first_axis, second_axis)
                start, end = sorted((first_pos, second_pos))
                line = _line_from_interval(direction, const, start, end)
                if group_geom.distance(line) > endpoint_tolerance:
                    continue

                endpoint_score = max(0.0, endpoint_tolerance - group_geom.distance(line)) / endpoint_tolerance
                const_score = 1.0 - abs(first_axis.const - second_axis.const) / max(axis_const_tolerance, 1.0)
                length_score = min(gap / 1000.0, 1.0)
                score = endpoint_score + const_score + length_score
                candidates.append((line, max(first_axis.thickness, second_axis.thickness), score))

    return candidates


def _opening_candidates_from_axis_overlap(
    axes: Sequence[AxisSegment],
    group_geom,
    endpoint_tolerance: float,
    min_gap: float,
    max_gap: float,
) -> list[tuple[LineString, float, float]]:
    candidates: list[tuple[LineString, float, float]] = []
    minx, miny, maxx, maxy = group_geom.bounds

    for axis in axes:
        if group_geom.distance(axis.line) > endpoint_tolerance:
            continue

        if axis.direction == "h":
            start = max(axis.start, minx)
            end = min(axis.end, maxx)
        else:
            start = max(axis.start, miny)
            end = min(axis.end, maxy)

        if end - start < min_gap or end - start > max_gap:
            continue

        line = _line_from_interval(axis.direction, axis.const, start, end)
        score = 0.8 + min((end - start) / 1200.0, 1.0)
        candidates.append((line, axis.thickness, score))

    return candidates


def _weighted_const(first: AxisSegment, second: AxisSegment) -> float:
    return (first.const * first.line.length + second.const * second.line.length) / max(
        first.line.length + second.line.length,
        1.0,
    )


def _overlaps_existing_wall_axis(
    line: LineString,
    wall_axis_lines: Sequence[LineString],
    max_overlap_ratio: float,
) -> bool:
    if line.length <= 0:
        return True
    for wall_line in wall_axis_lines:
        if line.distance(wall_line) > 1e-6:
            continue
        overlap = line.intersection(wall_line).length
        if overlap / line.length > max_overlap_ratio:
            return True
    return False


def _line_key(line: LineString) -> tuple[tuple[float, float], tuple[float, float]]:
    coords = list(line.coords)
    start = (round(coords[0][0], 3), round(coords[0][1], 3))
    end = (round(coords[-1][0], 3), round(coords[-1][1], 3))
    return tuple(sorted((start, end)))


def _line_from_interval(direction: str, const: float, start: float, end: float) -> LineString:
    if direction == "h":
        return LineString([(start, const), (end, const)])
    return LineString([(const, start), (const, end)])


def _filter_opening_lines_near_axes(
    opening_lines: Sequence[LineString],
    axes: Sequence[AxisSegment],
    max_distance: float,
) -> list[LineString]:
    if not opening_lines or not axes:
        return []

    axis_lines = [axis.line for axis in axes]
    axis_tree = STRtree(axis_lines)
    filtered = []
    for line in opening_lines:
        search = line.buffer(max_distance, cap_style=2)
        candidates = _query_tree(axis_tree, axis_lines, search)
        if any(line.distance(axis_line) <= max_distance for axis_line in candidates):
            filtered.append(line)
    return filtered


def _query_tree(tree: STRtree, geoms, geometry):
    result = []
    for item in tree.query(geometry):
        if isinstance(item, Integral):
            result.append(geoms[item])
        else:
            result.append(item)
    return result


def _query_tree_indices(tree: STRtree, geoms, geometry) -> list[int]:
    result = []
    for item in tree.query(geometry):
        if isinstance(item, Integral):
            result.append(int(item))
        else:
            result.append(geoms.index(item))
    return result


def _linemerge_safe(geometry):
    from shapely.ops import linemerge

    try:
        return linemerge(geometry)
    except ValueError:
        return geometry
