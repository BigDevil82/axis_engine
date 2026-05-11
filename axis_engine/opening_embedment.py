from __future__ import annotations

import math
import numbers
from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

from axis_engine.geometry_utils import iter_lines
from axis_engine.opening_clustering import OpeningCluster

OPENING_DOOR = "door"
OPENING_WINDOW = "window"
OPENING_BALCONY = "balcony"

Point2D = tuple[float, float]


@dataclass(frozen=True)
class OpeningEmbedment:
    opening_type: str
    embed_line: LineString
    cluster_index: int
    confidence: float
    reason: str


@dataclass(frozen=True)
class EmbedmentCandidate:
    opening_type: str
    line: LineString
    reason: str


@dataclass(frozen=True)
class AxisLineFeature:
    axis: str
    const: float
    start: float
    end: float

    @property
    def length(self) -> float:
        return self.end - self.start


class WallAxisEndpointIndex:
    def __init__(self, points: Sequence[Point2D]):
        self.points = list(points)
        self.geometries = [Point(point) for point in self.points]
        self.tree = STRtree(self.geometries) if self.geometries else None
        self._geom_to_index = {id(geometry): index for index, geometry in enumerate(self.geometries)}

    @classmethod
    def from_wall_axes(cls, wall_axes: Sequence[tuple[LineString, float]]) -> WallAxisEndpointIndex:
        axis_lines = [line for line, _thickness in wall_axes if line.length > 0]
        if not axis_lines:
            return cls(())

        network = unary_union(axis_lines)
        endpoints: list[Point2D] = []
        seen: set[tuple[int, int]] = set()
        for line in iter_lines(network):
            coords = list(line.coords)
            if len(coords) < 2:
                continue
            for coord in (coords[0], coords[-1]):
                point = (float(coord[0]), float(coord[1]))
                key = (round(point[0]), round(point[1]))
                if key in seen:
                    continue
                seen.add(key)
                endpoints.append(point)
        return cls(endpoints)

    def nearest(self, point: Point2D, max_distance: float) -> Point2D | None:
        if self.tree is None:
            return None

        query_point = Point(point)
        nearest = self.tree.nearest(query_point)
        if nearest is None:
            return None

        index = self._nearest_result_index(nearest)
        nearest_point = self.points[index]
        if query_point.distance(self.geometries[index]) > max_distance:
            return None
        return nearest_point

    def _nearest_result_index(self, nearest) -> int:
        if isinstance(nearest, numbers.Integral):
            return int(nearest)
        return self._geom_to_index[id(nearest)]


def infer_opening_embedments(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_clusters: Sequence[OpeningCluster],
    snap_tolerance: float = 400.0,
    parallel_line_min_length: float = 400.0,
    parallel_line_distance: float = 200.0,
    parallel_line_min_count: int = 3,
    axis_tolerance: float = 8.0,
) -> list[OpeningEmbedment]:
    embedments: list[OpeningEmbedment] = []
    endpoint_index = WallAxisEndpointIndex.from_wall_axes(wall_axes)

    for index, cluster in enumerate(opening_clusters):
        candidates = infer_opening_candidates(
            cluster,
            parallel_line_min_length=parallel_line_min_length,
            parallel_line_distance=parallel_line_distance,
            parallel_line_min_count=parallel_line_min_count,
            axis_tolerance=axis_tolerance,
        )
        cluster_embedments: list[OpeningEmbedment] = []
        for candidate in candidates:
            if candidate.line.length <= 1.0:
                continue

            snapped = snap_embed_line_to_wall_endpoints(
                candidate.line,
                endpoint_index,
                snap_tolerance=snap_tolerance,
            )
            confidence = _confidence(candidate.line, snapped, snap_tolerance)
            cluster_embedments.append(
                OpeningEmbedment(candidate.opening_type, snapped, index, confidence, candidate.reason)
            )

        embedments.extend(_dedupe_embedments(cluster_embedments))

    return embedments


def infer_opening_candidates(
    cluster: OpeningCluster,
    parallel_line_min_length: float = 400.0,
    parallel_line_distance: float = 200.0,
    parallel_line_min_count: int = 3,
    axis_tolerance: float = 8.0,
) -> list[EmbedmentCandidate]:
    candidates: list[EmbedmentCandidate] = []
    door_line, reason = _door_embed_line(cluster)
    if door_line is not None:
        candidates.append(EmbedmentCandidate(OPENING_DOOR, door_line, reason))

    parallel_lines = _parallel_line_embed_lines(
        cluster,
        min_length=parallel_line_min_length,
        distance=parallel_line_distance,
        min_count=parallel_line_min_count,
        axis_tolerance=axis_tolerance,
    )
    candidates.extend(
        EmbedmentCandidate(OPENING_WINDOW, line, "parallel_long_line_group")
        for line in parallel_lines
        if door_line is None or not _looks_like_door_leaf_line(line, door_line)
    )
    return candidates


def snap_embed_line_to_wall_endpoints(
    line: LineString,
    endpoint_index: WallAxisEndpointIndex,
    snap_tolerance: float = 400.0,
) -> LineString:
    coords = list(line.coords)
    if len(coords) < 2:
        return line

    start = (float(coords[0][0]), float(coords[0][1]))
    end = (float(coords[-1][0]), float(coords[-1][1]))
    snapped_start = endpoint_index.nearest(start, snap_tolerance) or start
    snapped_end = endpoint_index.nearest(end, snap_tolerance) or end

    if Point(snapped_start).distance(Point(snapped_end)) <= 1.0:
        return _orthogonalize(line)

    return LineString([snapped_start, snapped_end])


def unmatched_opening_indices(cluster_count: int, embedments: Sequence[OpeningEmbedment]) -> list[int]:
    matched = {embedment.cluster_index for embedment in embedments}
    return [index for index in range(cluster_count) if index not in matched]


def _dedupe_embedments(
    embedments: Sequence[OpeningEmbedment], tolerance: float = 50.0
) -> list[OpeningEmbedment]:
    selected: list[OpeningEmbedment] = []
    for embedment in sorted(
        embedments,
        key=lambda item: (
            -item.embed_line.length,
            -item.confidence,
            item.opening_type != OPENING_DOOR,
        ),
    ):
        if any(
            _embed_lines_are_duplicates(embedment.embed_line, existing.embed_line, tolerance)
            for existing in selected
        ):
            continue
        selected.append(embedment)
    return selected


def _looks_like_door_leaf_line(
    line: LineString, door_line: LineString, hinge_tolerance: float = 100.0
) -> bool:
    door_coords = list(door_line.coords)
    line_coords = list(line.coords)
    if len(door_coords) < 2 or len(line_coords) < 2:
        return False

    hinge = Point(door_coords[0])
    touches_hinge = any(Point(coord).distance(hinge) <= hinge_tolerance for coord in line_coords)
    if not touches_hinge:
        return False

    angle_delta = _angle_delta(_line_angle(line), _line_angle(door_line))
    return abs(angle_delta - math.pi / 2.0) <= math.radians(15.0)


def _embed_lines_are_duplicates(first: LineString, second: LineString, tolerance: float) -> bool:
    if first.hausdorff_distance(second) <= tolerance:
        return True

    first_feature = _axis_line_feature(first, axis_tolerance=tolerance)
    second_feature = _axis_line_feature(second, axis_tolerance=tolerance)
    if first_feature is None or second_feature is None:
        return False
    if first_feature.axis != second_feature.axis:
        return False
    if abs(first_feature.const - second_feature.const) > tolerance:
        return False

    overlap = min(first_feature.end, second_feature.end) - max(first_feature.start, second_feature.start)
    if overlap <= 0:
        return False

    shorter = min(first_feature.length, second_feature.length)
    return overlap / max(shorter, 1.0) >= 0.6


def _door_embed_line(cluster: OpeningCluster) -> tuple[LineString | None, str]:
    arcs = [arc for arc in cluster.arcs if arc.center and arc.start and arc.end]
    if len(arcs) >= 2:
        centers = [arc.center for arc in arcs if arc.center is not None]
        first, second = _farthest_pair(centers)
        if first and second and Point(first).distance(Point(second)) > 100.0:
            return LineString([first, second]), "double_door_arc_centers"

    if not arcs:
        return None, "door_without_arc"

    arc = max(arcs, key=lambda item: float(item.params.get("radius", 0.0)))
    center = arc.center
    start = arc.start
    end = arc.end
    if center is None or start is None or end is None:
        return None, "invalid_arc"

    opened_endpoint = _endpoint_supported_by_leaf(cluster, center, (start, end))
    closed_endpoint = end if opened_endpoint == start else start
    return LineString([center, closed_endpoint]), "single_door_arc_hinge_to_closed_endpoint"


def _parallel_line_embed_lines(
    cluster: OpeningCluster,
    min_length: float,
    distance: float,
    min_count: int,
    axis_tolerance: float,
) -> list[LineString]:
    features = [
        feature
        for geometry in cluster.geometries
        if geometry.geom_type == "LINE"
        for feature in [_axis_line_feature(geometry.representative_line(), axis_tolerance)]
        if feature is not None and feature.length >= min_length
    ]
    embed_lines: list[LineString] = []
    for axis in ("h", "v"):
        axis_features = [feature for feature in features if feature.axis == axis]
        for group in _cluster_parallel_features(axis_features, distance):
            if len(group) < min_count:
                continue
            const = sum(feature.const for feature in group) / len(group)
            start = min(feature.start for feature in group)
            end = max(feature.end for feature in group)
            if end - start < min_length:
                continue
            if axis == "h":
                embed_lines.append(LineString([(start, const), (end, const)]))
            else:
                embed_lines.append(LineString([(const, start), (const, end)]))
    return embed_lines


def _axis_line_feature(line: LineString, axis_tolerance: float) -> AxisLineFeature | None:
    coords = list(line.coords)
    if len(coords) < 2:
        return None

    start = coords[0]
    end = coords[-1]
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    if dy <= axis_tolerance and dx >= dy:
        x1, x2 = sorted((start[0], end[0]))
        return AxisLineFeature("h", (start[1] + end[1]) / 2.0, x1, x2)
    if dx <= axis_tolerance and dy >= dx:
        y1, y2 = sorted((start[1], end[1]))
        return AxisLineFeature("v", (start[0] + end[0]) / 2.0, y1, y2)
    return None


def _cluster_parallel_features(
    features: Sequence[AxisLineFeature],
    distance: float,
) -> list[list[AxisLineFeature]]:
    sorted_features = sorted(features, key=lambda item: item.const)
    if not sorted_features:
        return []

    groups: list[list[AxisLineFeature]] = [[sorted_features[0]]]
    for feature in sorted_features[1:]:
        if abs(feature.const - groups[-1][-1].const) <= distance:
            groups[-1].append(feature)
        else:
            groups.append([feature])
    return groups


def _endpoint_supported_by_leaf(
    cluster: OpeningCluster,
    center: Point2D,
    endpoints: tuple[Point2D, Point2D],
    tolerance: float = 25.0,
) -> Point2D:
    candidates = []
    for endpoint in endpoints:
        chord = LineString([center, endpoint])
        support = 0.0
        for geometry in cluster.geometries:
            if geometry.geom_type != "LINE":
                continue
            line = geometry.representative_line()
            if line.length <= 1.0:
                continue
            if line.distance(chord) <= tolerance:
                overlap_hint = min(line.distance(Point(center)), line.distance(Point(endpoint)))
                support += max(0.0, tolerance - overlap_hint) + min(line.length, chord.length)
        candidates.append((support, endpoint))

    candidates.sort(reverse=True, key=lambda item: item[0])
    if candidates[0][0] > candidates[1][0]:
        return candidates[0][1]
    return endpoints[0]


def _farthest_pair(points: Sequence[Point2D]) -> tuple[Point2D | None, Point2D | None]:
    if len(points) < 2:
        return None, None

    best_pair = (points[0], points[1])
    best_distance = -1.0
    for i, first in enumerate(points):
        for second in points[i + 1 :]:
            distance = Point(first).distance(Point(second))
            if distance > best_distance:
                best_distance = distance
                best_pair = (first, second)
    return best_pair


def _orthogonalize(line: LineString) -> LineString:
    start, end = list(line.coords)[0], list(line.coords)[-1]
    if abs(end[0] - start[0]) >= abs(end[1] - start[1]):
        y = (start[1] + end[1]) / 2.0
        return LineString([(start[0], y), (end[0], y)])
    x = (start[0] + end[0]) / 2.0
    return LineString([(x, start[1]), (x, end[1])])


def _confidence(raw: LineString, snapped: LineString, snap_tolerance: float) -> float:
    movement = raw.hausdorff_distance(snapped)
    return max(0.0, min(1.0, 1.0 - movement / max(snap_tolerance, 1.0)))


def _line_angle(line: LineString) -> float:
    coords = list(line.coords)
    start = coords[0]
    end = coords[-1]
    return math.atan2(end[1] - start[1], end[0] - start[0])


def _angle_delta(first: float, second: float) -> float:
    delta = abs((first - second + math.pi) % math.pi - math.pi / 2.0)
    return abs(delta - math.pi / 2.0)
