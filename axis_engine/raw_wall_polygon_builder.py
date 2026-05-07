from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass

from shapely.geometry import LineString, MultiPolygon, Polygon, box
from shapely.ops import unary_union


@dataclass(frozen=True)
class AxisAlignedSegment:
    axis: str
    const: float
    start: float
    end: float

    @property
    def length(self) -> float:
        return self.end - self.start


def build_wall_polygon_from_raw_lines(
    lines: Iterable[LineString],
    wall_thicknesses: Sequence[float] = (100.0, 200.0),
    angle_tolerance: float = 5.0,
    snap_tolerance: float = 3.0,
    thickness_tolerance: float = 15.0,
    min_overlap: float = 150.0,
    min_line_length: float = 80.0,
) -> MultiPolygon:
    """
    Reconstruct wall solid polygons from raw orthogonal CAD wall lines.

    This is intended for layers like WALL where entities are primitive linework
    instead of ready-made wall boundary polylines.
    """
    segments = _collect_axis_aligned_segments(lines, angle_tolerance, snap_tolerance, min_line_length)
    horizontal = [segment for segment in segments if segment.axis == "h"]
    vertical = [segment for segment in segments if segment.axis == "v"]

    wall_boxes = []
    wall_boxes.extend(
        _pair_parallel_segments(horizontal, "h", wall_thicknesses, thickness_tolerance, min_overlap)
    )
    wall_boxes.extend(
        _pair_parallel_segments(vertical, "v", wall_thicknesses, thickness_tolerance, min_overlap)
    )

    if not wall_boxes:
        return MultiPolygon([])

    merged = unary_union(wall_boxes).buffer(0)
    if merged.is_empty:
        return MultiPolygon([])
    if isinstance(merged, Polygon):
        return MultiPolygon([merged])
    return MultiPolygon([poly for poly in merged.geoms if isinstance(poly, Polygon) and poly.area > 0])


def _collect_axis_aligned_segments(
    lines: Iterable[LineString],
    angle_tolerance: float,
    snap_tolerance: float,
    min_line_length: float,
) -> list[AxisAlignedSegment]:
    segments: list[AxisAlignedSegment] = []
    for line in lines:
        coords = list(line.coords)
        for start, end in zip(coords, coords[1:]):
            x1, y1 = start[:2]
            x2, y2 = end[:2]
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            length = max(dx, dy)
            if length < min_line_length:
                continue

            if dy <= angle_tolerance and dx >= dy:
                y = _snap((y1 + y2) / 2.0, snap_tolerance)
                a, b = sorted((_snap(x1, snap_tolerance), _snap(x2, snap_tolerance)))
                if b - a >= min_line_length:
                    segments.append(AxisAlignedSegment("h", y, a, b))
            elif dx <= angle_tolerance and dy >= dx:
                x = _snap((x1 + x2) / 2.0, snap_tolerance)
                a, b = sorted((_snap(y1, snap_tolerance), _snap(y2, snap_tolerance)))
                if b - a >= min_line_length:
                    segments.append(AxisAlignedSegment("v", x, a, b))

    return _dedupe_segments(segments, snap_tolerance)


def _pair_parallel_segments(
    segments: list[AxisAlignedSegment],
    axis: str,
    wall_thicknesses: Sequence[float],
    thickness_tolerance: float,
    min_overlap: float,
) -> list[Polygon]:
    result = []
    sorted_segments = sorted(segments, key=lambda item: (item.const, item.start, item.end))

    for index, first in enumerate(sorted_segments):
        for second in sorted_segments[index + 1 :]:
            gap = second.const - first.const
            if gap <= 0:
                continue
            max_thickness = max(wall_thicknesses) + thickness_tolerance
            if gap > max_thickness:
                break
            if not _is_wall_thickness(gap, wall_thicknesses, thickness_tolerance):
                continue

            start = max(first.start, second.start)
            end = min(first.end, second.end)
            if end - start < min_overlap:
                continue

            if axis == "h":
                result.append(box(start, first.const, end, second.const))
            else:
                result.append(box(first.const, start, second.const, end))

    return result


def _is_wall_thickness(value: float, wall_thicknesses: Sequence[float], tolerance: float) -> bool:
    return any(abs(value - thickness) <= tolerance for thickness in wall_thicknesses)


def _snap(value: float, tolerance: float) -> float:
    if tolerance <= 0:
        return float(value)
    return round(value / tolerance) * tolerance


def _dedupe_segments(
    segments: Iterable[AxisAlignedSegment], snap_tolerance: float
) -> list[AxisAlignedSegment]:
    merged_by_line: dict[tuple[str, float], list[tuple[float, float]]] = {}
    for segment in segments:
        key = (segment.axis, segment.const)
        merged_by_line.setdefault(key, []).append((segment.start, segment.end))

    result = []
    for (axis, const), intervals in merged_by_line.items():
        intervals.sort()
        merged_intervals: list[list[float]] = []
        for start, end in intervals:
            if not merged_intervals or start > merged_intervals[-1][1] + snap_tolerance:
                merged_intervals.append([start, end])
            else:
                merged_intervals[-1][1] = max(merged_intervals[-1][1], end)

        for start, end in merged_intervals:
            result.append(AxisAlignedSegment(axis, const, start, end))

    return result
