from __future__ import annotations

import numbers
from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, Point, box
from shapely.strtree import STRtree

from axis_engine.geometry_utils import iter_straight_segments
from axis_engine.opening_embedment import OpeningEmbedment

Point2D = tuple[float, float]


@dataclass(frozen=True)
class SkeletonCalibrationResult:
    wall_axes: list[tuple[LineString, float]]
    opening_embedments: list[OpeningEmbedment]


@dataclass(frozen=True)
class _SegmentRecord:
    line: LineString
    source: str
    source_index: int
    thickness: float | None = None
    opening: OpeningEmbedment | None = None


@dataclass(frozen=True)
class _AxisSegment:
    start: Point2D
    end: Point2D
    axis: str
    source: str
    source_index: int
    thickness: float | None = None
    opening: OpeningEmbedment | None = None

    @property
    def line(self) -> LineString:
        return LineString([self.start, self.end])

    @property
    def length(self) -> float:
        return self.line.length

    @property
    def const(self) -> float:
        if self.axis == "h":
            return (self.start[1] + self.end[1]) / 2.0
        return (self.start[0] + self.end[0]) / 2.0

    @property
    def interval(self) -> tuple[float, float]:
        if self.axis == "h":
            return tuple(sorted((self.start[0], self.end[0])))
        return tuple(sorted((self.start[1], self.end[1])))


def calibrate_skeleton_topology(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    endpoint_snap_tolerance: float = 120.0,
    axis_tolerance: float = 8.0,
    min_segment_length: float = 1.0,
) -> SkeletonCalibrationResult:
    """Snap near endpoints together and split orthogonal crossings into nodes."""
    records = _collect_records(wall_axes, opening_embedments, min_segment_length)
    segments = [
        segment
        for record in records
        for segment in [_axis_segment_from_record(record, axis_tolerance)]
        if segment is not None and segment.length >= min_segment_length
    ]
    snapped = _snap_segment_endpoints(segments, endpoint_snap_tolerance)
    split_segments = _split_at_orthogonal_intersections(snapped, min_segment_length)
    return _restore_semantic_segments(split_segments)


def _collect_records(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    min_segment_length: float,
) -> list[_SegmentRecord]:
    records: list[_SegmentRecord] = []
    for index, (line, thickness) in enumerate(wall_axes):
        for segment in iter_straight_segments(line, min_length=min_segment_length):
            records.append(_SegmentRecord(segment, "wall", index, thickness=thickness))

    for index, embedment in enumerate(opening_embedments):
        for segment in iter_straight_segments(embedment.embed_line, min_length=min_segment_length):
            records.append(_SegmentRecord(segment, "opening", index, opening=embedment))
    return records


def _axis_segment_from_record(record: _SegmentRecord, axis_tolerance: float) -> _AxisSegment | None:
    coords = list(record.line.coords)
    start = (float(coords[0][0]), float(coords[0][1]))
    end = (float(coords[-1][0]), float(coords[-1][1]))
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    if dy <= axis_tolerance and dx >= dy:
        y = (start[1] + end[1]) / 2.0
        x1, x2 = sorted((start[0], end[0]))
        return _AxisSegment((x1, y), (x2, y), "h", record.source, record.source_index, record.thickness, record.opening)
    if dx <= axis_tolerance and dy >= dx:
        x = (start[0] + end[0]) / 2.0
        y1, y2 = sorted((start[1], end[1]))
        return _AxisSegment((x, y1), (x, y2), "v", record.source, record.source_index, record.thickness, record.opening)
    return None


def _snap_segment_endpoints(
    segments: Sequence[_AxisSegment],
    tolerance: float,
) -> list[_AxisSegment]:
    if not segments:
        return []

    line_geoms = [segment.line for segment in segments]
    line_tree = STRtree(line_geoms)
    endpoints = []
    endpoint_owner = []
    for index, segment in enumerate(segments):
        endpoints.append(Point(segment.start))
        endpoint_owner.append((index, 0))
        endpoints.append(Point(segment.end))
        endpoint_owner.append((index, 1))
    endpoint_tree = STRtree(endpoints)

    snapped: list[_AxisSegment] = []
    for index, segment in enumerate(segments):
        start, start_snapped = _snap_endpoint(
            segment.start,
            index,
            line_geoms,
            line_tree,
            endpoints,
            endpoint_owner,
            endpoint_tree,
            tolerance,
        )
        end, end_snapped = _snap_endpoint(
            segment.end,
            index,
            line_geoms,
            line_tree,
            endpoints,
            endpoint_owner,
            endpoint_tree,
            tolerance,
        )
        snapped.append(_rebuild_axis_segment(segment, start, end, start_snapped, end_snapped))
    return [segment for segment in snapped if segment.length > 0]


def _snap_endpoint(
    point: Point2D,
    segment_index: int,
    line_geoms: Sequence[LineString],
    line_tree: STRtree,
    endpoints: Sequence[Point],
    endpoint_owner: Sequence[tuple[int, int]],
    endpoint_tree: STRtree,
    tolerance: float,
) -> tuple[Point2D, bool]:
    point_geom = Point(point)
    candidates: list[tuple[float, Point2D]] = []

    for result in line_tree.query(point_geom.buffer(tolerance)):
        candidate_index = _tree_result_index(result, line_geoms)
        if candidate_index == segment_index:
            continue
        candidate_line = line_geoms[candidate_index]
        projected = candidate_line.interpolate(candidate_line.project(point_geom))
        distance = point_geom.distance(projected)
        if distance <= tolerance:
            candidates.append((distance, (float(projected.x), float(projected.y))))

    for result in endpoint_tree.query(point_geom.buffer(tolerance)):
        endpoint_index = _tree_result_index(result, endpoints)
        owner_index, _endpoint_number = endpoint_owner[endpoint_index]
        if owner_index == segment_index:
            continue
        endpoint = endpoints[endpoint_index]
        distance = point_geom.distance(endpoint)
        if distance <= tolerance:
            candidates.append((distance, (float(endpoint.x), float(endpoint.y))))

    if not candidates:
        return point, False

    _distance, snapped = min(candidates, key=lambda item: item[0])
    return snapped, True


def _rebuild_axis_segment(
    original: _AxisSegment,
    start: Point2D,
    end: Point2D,
    start_snapped: bool,
    end_snapped: bool,
) -> _AxisSegment:
    if original.axis == "h":
        y_values = []
        if start_snapped:
            y_values.append(start[1])
        if end_snapped:
            y_values.append(end[1])
        y = sum(y_values) / len(y_values) if y_values else original.const
        x1, x2 = sorted((start[0], end[0]))
        return _copy_segment(original, (x1, y), (x2, y))

    x_values = []
    if start_snapped:
        x_values.append(start[0])
    if end_snapped:
        x_values.append(end[0])
    x = sum(x_values) / len(x_values) if x_values else original.const
    y1, y2 = sorted((start[1], end[1]))
    return _copy_segment(original, (x, y1), (x, y2))


def _split_at_orthogonal_intersections(
    segments: Sequence[_AxisSegment],
    min_segment_length: float,
) -> list[_AxisSegment]:
    horizontal = [(index, segment) for index, segment in enumerate(segments) if segment.axis == "h"]
    vertical = [(index, segment) for index, segment in enumerate(segments) if segment.axis == "v"]
    split_values: dict[int, set[float]] = {index: set(segment.interval) for index, segment in enumerate(segments)}

    vertical_geoms = [segment.line for _index, segment in vertical]
    vertical_tree = STRtree(vertical_geoms) if vertical_geoms else None

    if vertical_tree is not None:
        for h_index, h_segment in horizontal:
            h_start, h_end = h_segment.interval
            search = box(h_start, h_segment.const, h_end, h_segment.const)
            for result in vertical_tree.query(search):
                local_index = _tree_result_index(result, vertical_geoms)
                v_index, v_segment = vertical[local_index]
                v_start, v_end = v_segment.interval
                x = v_segment.const
                y = h_segment.const
                if h_start <= x <= h_end and v_start <= y <= v_end:
                    split_values[h_index].add(x)
                    split_values[v_index].add(y)

    result_segments: list[_AxisSegment] = []
    for index, segment in enumerate(segments):
        values = sorted(split_values[index])
        for start_value, end_value in zip(values, values[1:]):
            if end_value - start_value < min_segment_length:
                continue
            if segment.axis == "h":
                result_segments.append(_copy_segment(segment, (start_value, segment.const), (end_value, segment.const)))
            else:
                result_segments.append(_copy_segment(segment, (segment.const, start_value), (segment.const, end_value)))
    return result_segments


def _restore_semantic_segments(segments: Sequence[_AxisSegment]) -> SkeletonCalibrationResult:
    wall_axes: list[tuple[LineString, float]] = []
    opening_embedments: list[OpeningEmbedment] = []

    for segment in segments:
        if segment.source == "wall":
            wall_axes.append((segment.line, float(segment.thickness or 0.0)))
            continue

        if segment.opening is None:
            continue
        opening_embedments.append(
            OpeningEmbedment(
                opening_type=segment.opening.opening_type,
                embed_line=segment.line,
                cluster_index=segment.opening.cluster_index,
                confidence=segment.opening.confidence,
                reason=f"{segment.opening.reason}|topology_calibrated",
            )
        )

    return SkeletonCalibrationResult(wall_axes, opening_embedments)


def _copy_segment(segment: _AxisSegment, start: Point2D, end: Point2D) -> _AxisSegment:
    return _AxisSegment(
        start=start,
        end=end,
        axis=segment.axis,
        source=segment.source,
        source_index=segment.source_index,
        thickness=segment.thickness,
        opening=segment.opening,
    )


def _tree_result_index(result, geoms: Sequence) -> int:
    if isinstance(result, numbers.Integral):
        return int(result)
    geom_to_index = {id(geom): index for index, geom in enumerate(geoms)}
    return geom_to_index[id(result)]
