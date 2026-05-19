from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, Point
from shapely.ops import linemerge, unary_union

from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.structural_design.models import ShearWall


@dataclass(frozen=True)
class ShearWallLayoutOptions:
    connection_tolerance: float = 20.0
    collinear_tolerance: float = 20.0
    long_wall_length: float = 6000.0
    max_split_segment_length: float = 3000.0
    min_segment_length: float = 1.0


def dominant_wall_thickness(wall_axes: Sequence[tuple[LineString, float]]) -> float | None:
    thickness_lengths: dict[float, float] = {}
    for line, thickness in wall_axes:
        thickness_lengths[thickness] = thickness_lengths.get(thickness, 0.0) + line.length
    if not thickness_lengths:
        return None
    return max(thickness_lengths.items(), key=lambda item: item[1])[0]


def select_shear_walls(
    wall_axes: Sequence[tuple[LineString, float]],
    dominant_thickness: float | None,
    options: ShearWallLayoutOptions | None = None,
) -> list[ShearWall]:
    if dominant_thickness is None:
        return []
    options = options or ShearWallLayoutOptions()
    dominant_lines = [
        line
        for line, thickness in wall_axes
        if thickness == dominant_thickness and line.length > 0
    ]
    merged_lines = _merged_lines(dominant_lines, options.min_segment_length)
    connected_lines = _remove_isolated_lines(merged_lines, options.connection_tolerance)
    adjusted_lines = _shorten_long_collinear_runs(connected_lines, options)
    return [
        ShearWall(line, dominant_thickness, "dominant_wall_thickness|layout_adjusted")
        for line in adjusted_lines
        if line.length >= options.min_segment_length
    ]


def _merged_lines(
    lines: Sequence[LineString],
    min_segment_length: float,
) -> list[LineString]:
    if not lines:
        return []
    unified = unary_union(lines)
    merged = linemerge(unified) if hasattr(unified, "geoms") else unified
    return [
        segment
        for line in iter_lines(merged)
        for segment in iter_straight_segments(line, min_length=min_segment_length)
    ]


def _remove_isolated_lines(
    lines: Sequence[LineString],
    tolerance: float,
) -> list[LineString]:
    if len(lines) <= 1:
        return []
    return [
        line
        for index, line in enumerate(lines)
        if _connected_at_endpoint(line.coords[0], index, lines, tolerance)
        or _connected_at_endpoint(line.coords[-1], index, lines, tolerance)
    ]


def _connected_at_endpoint(
    point,
    line_index: int,
    lines: Sequence[LineString],
    tolerance: float,
) -> bool:
    for index, line in enumerate(lines):
        if index == line_index:
            continue
        if line.distance(Point(point)) <= tolerance:
            return True
    return False


def _shorten_long_collinear_runs(
    lines: Sequence[LineString],
    options: ShearWallLayoutOptions,
) -> list[LineString]:
    adjusted: list[LineString] = []
    consumed: set[int] = set()
    groups = _collinear_groups(lines, options.collinear_tolerance)
    for group in groups:
        if len(group) == 1:
            index = group[0]
            if index in consumed:
                continue
            consumed.add(index)
            line = lines[index]
            if line.length > options.long_wall_length:
                adjusted.extend(_split_long_line(line, options))
            else:
                adjusted.append(line)
            continue

        run_lines = _continuous_run_lines([lines[index] for index in group], options)
        for run in run_lines:
            if run.length > options.long_wall_length:
                adjusted.extend(_split_long_line(run, options))
            else:
                adjusted.append(run)
        consumed.update(group)
    return adjusted


def _collinear_groups(lines: Sequence[LineString], tolerance: float) -> list[list[int]]:
    groups: dict[tuple[str, int], list[int]] = {}
    for index, line in enumerate(lines):
        feature = _line_feature(line, tolerance)
        if feature is None:
            groups.setdefault(("free", index), []).append(index)
            continue
        axis, const, _start, _end = feature
        groups.setdefault((axis, round(const / max(tolerance, 1e-6))), []).append(index)
    return list(groups.values())


def _continuous_run_lines(lines: Sequence[LineString], options: ShearWallLayoutOptions) -> list[LineString]:
    if not lines:
        return []
    feature = _line_feature(lines[0], options.collinear_tolerance)
    if feature is None:
        return list(lines)

    axis, const, _start, _end = feature
    intervals = []
    for line in lines:
        line_feature = _line_feature(line, options.collinear_tolerance)
        if line_feature is None:
            continue
        intervals.append((line_feature[2], line_feature[3]))
    if not intervals:
        return []

    intervals.sort()
    runs: list[list[float]] = []
    for start, end in intervals:
        if not runs or start > runs[-1][1] + options.connection_tolerance:
            runs.append([start, end])
        else:
            runs[-1][1] = max(runs[-1][1], end)

    if axis == "h":
        return [LineString([(start, const), (end, const)]) for start, end in runs]
    return [LineString([(const, start), (const, end)]) for start, end in runs]


def _split_long_line(line: LineString, options: ShearWallLayoutOptions) -> list[LineString]:
    length = line.length
    keep = min(options.max_split_segment_length, length / 2.0)
    if keep <= options.min_segment_length:
        return []

    return [_subline(line, 0.0, keep), _subline(line, length - keep, length)]


def _line_feature(line: LineString, tolerance: float) -> tuple[str, float, float, float] | None:
    coords = list(line.coords)
    if len(coords) < 2:
        return None
    start = coords[0]
    end = coords[-1]
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    if dy <= tolerance and dx > tolerance:
        x1, x2 = sorted((start[0], end[0]))
        return "h", (start[1] + end[1]) / 2.0, x1, x2
    if dx <= tolerance and dy > tolerance:
        y1, y2 = sorted((start[1], end[1]))
        return "v", (start[0] + end[0]) / 2.0, y1, y2
    return None


def _subline(line: LineString, start_distance: float, end_distance: float) -> LineString:
    start = line.interpolate(start_distance)
    end = line.interpolate(end_distance)
    return LineString([(start.x, start.y), (end.x, end.y)])
