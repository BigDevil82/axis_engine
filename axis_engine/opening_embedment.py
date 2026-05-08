from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot, pi, sqrt
from typing import Sequence

from shapely.geometry import LineString, MultiLineString, Point, box
from shapely.ops import unary_union

from axis_engine.linework_axis_extractor import extract_wall_axes_from_linework

Point2D = tuple[float, float]

OPENING_DOOR = "door"
OPENING_WINDOW = "window"
OPENING_BALCONY = "balcony"


@dataclass(frozen=True)
class WallAxisEndpoint:
    axis_index: int
    point: Point2D
    at_start: bool
    direction: str
    thickness: float


@dataclass(frozen=True)
class OpeningEmbedment:
    opening_index: int
    embed_line: LineString | MultiLineString
    first_endpoint: WallAxisEndpoint
    second_endpoint: WallAxisEndpoint
    score: float
    endpoint_distance: float
    opening_distance: float
    opening_bounds: tuple[float, float, float, float]
    opening_type: str = OPENING_WINDOW
    candidate_points: tuple[Point2D, ...] = ()


@dataclass(frozen=True)
class ArcFeature:
    center: Point2D
    points: tuple[Point2D, ...]
    radius: float
    residual: float


def infer_opening_embedments(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_groups: Sequence[Sequence[LineString]],
    search_distance: float = 700.0,
    bbox_padding: float = 0.0,
    min_embed_length: float = 80.0,
    max_embed_length: float | None = None,
    axis_alignment_tolerance: float = 160.0,
) -> list[OpeningEmbedment]:
    """Infer door/window embedment lines from pairs of nearby wall-axis endpoints.

    This intentionally does not project opening bounding boxes onto wall axes.  It
    searches wall-axis endpoints around each opening group and connects the best
    pair of facing endpoints from two different wall axis segments.
    """

    endpoints = _axis_endpoints(wall_axes)
    if not endpoints or not opening_groups:
        return []

    max_length = max_embed_length or max(search_distance * 4.0, min_embed_length)
    result: list[OpeningEmbedment] = []
    used_endpoint_pairs: set[tuple[tuple[int, bool], tuple[int, bool]]] = set()
    wall_axis_lines = [axis for axis, _thickness in wall_axes]

    for opening_index, group in enumerate(opening_groups):
        opening_lines = [line for line in group if line.length > 0]
        if not opening_lines:
            continue

        opening_geom = unary_union(opening_lines)
        opening_bounds = _bounds(opening_lines, bbox_padding)
        bbox_geom = box(*opening_bounds)
        search_geom = bbox_geom.buffer(search_distance, cap_style=2, join_style=2)
        opening_type, candidate_points = classify_opening_group(opening_lines, wall_axis_lines)
        candidates = [
            endpoint
            for endpoint in endpoints
            if search_geom.contains(_point_as_geometry(endpoint.point))
            and bbox_geom.distance(_point_as_geometry(endpoint.point)) <= search_distance
        ]

        if opening_type == OPENING_DOOR:
            match = _best_candidate_point_pair(
                opening_index,
                opening_type,
                candidate_points,
                bbox_geom,
                opening_bounds,
                candidates,
                min_embed_length=min_embed_length,
                max_embed_length=max_length,
                axis_alignment_tolerance=axis_alignment_tolerance,
                used_endpoint_pairs=used_endpoint_pairs,
            )
        elif opening_type == OPENING_BALCONY:
            match = _best_balcony_embedment(
                opening_index,
                opening_type,
                opening_lines,
                bbox_geom,
                opening_bounds,
                candidates,
                min_embed_length=min_embed_length,
                max_embed_length=max_length,
                axis_alignment_tolerance=axis_alignment_tolerance,
                used_endpoint_pairs=used_endpoint_pairs,
            )
        else:
            match = None

        if match is None:
            match = _best_endpoint_pair(
                opening_index,
                opening_type,
                opening_geom,
                bbox_geom,
                opening_bounds,
                candidates,
                min_embed_length=min_embed_length,
                max_embed_length=max_length,
                axis_alignment_tolerance=axis_alignment_tolerance,
                used_endpoint_pairs=used_endpoint_pairs,
            )
        if match is None:
            continue

        used_endpoint_pairs.add(_endpoint_pair_key(match.first_endpoint, match.second_endpoint))
        result.append(match)

    return result


def unmatched_opening_indices(opening_count: int, embedments: Sequence[OpeningEmbedment]) -> list[int]:
    matched = {embedment.opening_index for embedment in embedments}
    return [index for index in range(opening_count) if index not in matched]


def classify_opening_group(
    opening_lines: Sequence[LineString],
    wall_axis_lines: Sequence[LineString] = (),
) -> tuple[str, tuple[Point2D, ...]]:
    bounds = _bounds(opening_lines)
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    ratio = max(width, height) / max(min(width, height), 1.0)
    arcs = _detect_arc_features(opening_lines)

    if arcs:
        points = [arc.center for arc in arcs[:2]]
        if len(points) == 1:
            points = _single_arc_door_candidate_points(arcs[0], bounds, wall_axis_lines)
        return OPENING_DOOR, tuple(points[:2])

    if ratio > 3.0:
        return OPENING_WINDOW, ()

    if _looks_like_u_shape(opening_lines, bounds):
        return OPENING_BALCONY, ()

    return OPENING_WINDOW, ()


def _best_endpoint_pair(
    opening_index: int,
    opening_type: str,
    opening_geom,
    bbox_geom,
    opening_bounds: tuple[float, float, float, float],
    endpoints: Sequence[WallAxisEndpoint],
    min_embed_length: float,
    max_embed_length: float,
    axis_alignment_tolerance: float,
    used_endpoint_pairs: set[tuple[tuple[int, bool], tuple[int, bool]]],
) -> OpeningEmbedment | None:
    best: OpeningEmbedment | None = None

    for first_index, first in enumerate(endpoints):
        for second in endpoints[first_index + 1 :]:
            if first.axis_index == second.axis_index:
                continue
            if first.direction != second.direction:
                continue
            if _endpoint_pair_key(first, second) in used_endpoint_pairs:
                continue
            if not _endpoints_are_aligned(first, second, axis_alignment_tolerance):
                continue

            embed_line = LineString([first.point, second.point])
            embed_length = embed_line.length
            if embed_length < min_embed_length or embed_length > max_embed_length:
                continue

            opening_distance = bbox_geom.distance(embed_line)
            if opening_distance > axis_alignment_tolerance:
                continue
            if not embed_line.buffer(axis_alignment_tolerance, cap_style=2).intersects(bbox_geom):
                continue

            endpoint_distance = max(
                bbox_geom.distance(_point_as_geometry(first.point)),
                bbox_geom.distance(_point_as_geometry(second.point)),
            )
            score = _score_pair(
                embed_line,
                opening_bounds,
                endpoint_distance,
                opening_distance,
                axis_alignment_tolerance,
            )
            candidate = OpeningEmbedment(
                opening_index=opening_index,
                embed_line=embed_line,
                first_endpoint=first,
                second_endpoint=second,
                score=score,
                endpoint_distance=endpoint_distance,
                opening_distance=opening_distance,
                opening_bounds=opening_bounds,
                opening_type=opening_type,
            )
            if best is None or candidate.score > best.score:
                best = candidate

    return best


def _best_candidate_point_pair(
    opening_index: int,
    opening_type: str,
    candidate_points: Sequence[Point2D],
    bbox_geom,
    opening_bounds: tuple[float, float, float, float],
    endpoints: Sequence[WallAxisEndpoint],
    min_embed_length: float,
    max_embed_length: float,
    axis_alignment_tolerance: float,
    used_endpoint_pairs: set[tuple[tuple[int, bool], tuple[int, bool]]],
) -> OpeningEmbedment | None:
    if len(candidate_points) < 2:
        return None

    best: OpeningEmbedment | None = None
    for first_index, first in enumerate(endpoints):
        for second in endpoints[first_index + 1 :]:
            if first.axis_index == second.axis_index:
                continue
            if _endpoint_pair_key(first, second) in used_endpoint_pairs:
                continue
            if not _endpoint_line_matches_candidate_direction(
                first,
                second,
                candidate_points,
                axis_alignment_tolerance,
            ):
                continue

            embed_line = LineString([first.point, second.point])
            embed_length = embed_line.length
            if embed_length < min_embed_length or embed_length > max_embed_length:
                continue
            if bbox_geom.distance(embed_line) > axis_alignment_tolerance:
                continue

            direct = _candidate_endpoint_distance(candidate_points, first, second)
            swapped = _candidate_endpoint_distance(candidate_points, second, first)
            candidate_distance = min(direct, swapped)
            if candidate_distance > axis_alignment_tolerance * 2.5:
                continue

            embed_line = _orthogonal_line_from_candidate_direction(first.point, second.point, candidate_points)
            opening_distance = bbox_geom.distance(embed_line)
            score = (
                5.0
                - candidate_distance / max(axis_alignment_tolerance, 1.0)
                - opening_distance / max(axis_alignment_tolerance, 1.0)
            )
            candidate = OpeningEmbedment(
                opening_index=opening_index,
                embed_line=embed_line,
                first_endpoint=first,
                second_endpoint=second,
                score=score,
                endpoint_distance=candidate_distance,
                opening_distance=opening_distance,
                opening_bounds=opening_bounds,
                opening_type=opening_type,
                candidate_points=tuple(candidate_points[:2]),
            )
            if best is None or candidate.score > best.score:
                best = candidate

    return best


def _best_balcony_embedment(
    opening_index: int,
    opening_type: str,
    opening_lines: Sequence[LineString],
    bbox_geom,
    opening_bounds: tuple[float, float, float, float],
    endpoints: Sequence[WallAxisEndpoint],
    min_embed_length: float,
    max_embed_length: float,
    axis_alignment_tolerance: float,
    used_endpoint_pairs: set[tuple[tuple[int, bool], tuple[int, bool]]],
) -> OpeningEmbedment | None:
    axis_geometry = _balcony_axis_geometry(opening_lines, opening_bounds)
    if axis_geometry is None:
        return None

    if axis_geometry.length < min_embed_length or axis_geometry.length > max_embed_length * 3.0:
        return None

    candidate_points = _geometry_endpoints(axis_geometry)
    first_point = candidate_points[0] if candidate_points else (opening_bounds[0], opening_bounds[1])
    second_point = candidate_points[-1] if len(candidate_points) > 1 else first_point
    dummy_first = WallAxisEndpoint(
        axis_index=-(opening_index + 1),
        point=first_point,
        at_start=True,
        direction=_line_direction(LineString([first_point, second_point])) if first_point != second_point else "h",
        thickness=0.0,
    )
    dummy_second = WallAxisEndpoint(
        axis_index=-(opening_index + 1),
        point=second_point,
        at_start=False,
        direction=dummy_first.direction,
        thickness=0.0,
    )

    return OpeningEmbedment(
        opening_index=opening_index,
        embed_line=axis_geometry,
        first_endpoint=dummy_first,
        second_endpoint=dummy_second,
        score=3.5,
        endpoint_distance=0.0,
        opening_distance=bbox_geom.distance(axis_geometry),
        opening_bounds=opening_bounds,
        opening_type=opening_type,
        candidate_points=tuple(candidate_points),
    )


def _score_pair(
    embed_line: LineString,
    opening_bounds: tuple[float, float, float, float],
    endpoint_distance: float,
    opening_distance: float,
    tolerance: float,
) -> float:
    minx, miny, maxx, maxy = opening_bounds
    bbox_center = ((minx + maxx) / 2.0, (miny + maxy) / 2.0)
    midpoint = embed_line.interpolate(0.5, normalized=True)
    center_distance = hypot(midpoint.x - bbox_center[0], midpoint.y - bbox_center[1])
    bbox_size = max(maxx - minx, maxy - miny, 1.0)
    return (
        3.0
        - endpoint_distance / max(tolerance, 1.0)
        - opening_distance / max(tolerance, 1.0)
        - center_distance / bbox_size
    )


def _detect_arc_features(opening_lines: Sequence[LineString]) -> list[ArcFeature]:
    non_axis_lines = [line for line in opening_lines if not _is_axis_aligned(line, tolerance=0.08)]
    components = _connected_line_components(non_axis_lines, tolerance=25.0)
    arcs: list[ArcFeature] = []

    for component in components:
        points = _unique_component_points(component, tolerance=5.0)
        if len(points) < 4:
            continue

        center_radius = _fit_circle(points)
        if center_radius is None:
            continue

        center, radius, residual = center_radius
        if radius < 100.0:
            continue
        if residual > max(35.0, radius * 0.08):
            continue

        span = _angular_span(center, points)
        if span < pi / 9.0 or span > pi * 1.25:
            continue
        arcs.append(ArcFeature(center=center, points=tuple(points), radius=radius, residual=residual))

    if not arcs:
        arcs = _detect_arc_features_by_circle_inliers(non_axis_lines)

    return sorted(arcs, key=lambda item: item.radius, reverse=True)


def _detect_arc_features_by_circle_inliers(lines: Sequence[LineString]) -> list[ArcFeature]:
    remaining = _unique_component_points(lines, tolerance=5.0)
    arcs: list[ArcFeature] = []

    for _ in range(2):
        best: ArcFeature | None = None
        for i in range(len(remaining)):
            for j in range(i + 1, len(remaining)):
                for k in range(j + 1, len(remaining)):
                    circle = _circle_from_three_points(remaining[i], remaining[j], remaining[k])
                    if circle is None:
                        continue
                    center, radius = circle
                    if radius < 100.0:
                        continue
                    tolerance = max(35.0, radius * 0.08)
                    inliers = [
                        point
                        for point in remaining
                        if abs(_distance(point, center) - radius) <= tolerance
                    ]
                    if len(inliers) < 4:
                        continue
                    refined = _fit_circle(inliers)
                    if refined is None:
                        continue
                    refined_center, refined_radius, residual = refined
                    span = _angular_span(refined_center, inliers)
                    if span < pi / 10.0 or span > pi * 1.35:
                        continue
                    if residual > max(60.0, refined_radius * 0.12):
                        continue
                    candidate = ArcFeature(
                        center=refined_center,
                        points=tuple(inliers),
                        radius=refined_radius,
                        residual=residual,
                    )
                    if best is None or len(candidate.points) > len(best.points) or candidate.residual < best.residual:
                        best = candidate
        if best is None:
            break
        arcs.append(best)
        remaining = [point for point in remaining if point not in set(best.points)]
        if len(remaining) < 4:
            break

    return arcs


def _arc_point_near_wall_axis(arc: ArcFeature, wall_axis_lines: Sequence[LineString]) -> Point2D:
    if not wall_axis_lines:
        return arc.points[-1]

    best_point = arc.points[0]
    best_distance = float("inf")
    for point in arc.points:
        point_geom = Point(point)
        distance = min(axis.distance(point_geom) for axis in wall_axis_lines)
        if distance < best_distance:
            best_distance = distance
            best_point = point
    return best_point


def _single_arc_door_candidate_points(
    arc: ArcFeature,
    bounds: tuple[float, float, float, float],
    wall_axis_lines: Sequence[LineString],
) -> list[Point2D]:
    minx, miny, maxx, maxy = bounds
    width = maxx - minx
    height = maxy - miny
    center = arc.center

    if width >= height:
        edge_y = miny if abs(center[1] - miny) <= abs(center[1] - maxy) else maxy
        return [(minx, edge_y), (maxx, edge_y)]

    edge_x = minx if abs(center[0] - minx) <= abs(center[0] - maxx) else maxx
    return [(edge_x, miny), (edge_x, maxy)]


def _looks_like_u_shape(opening_lines: Sequence[LineString], bounds: tuple[float, float, float, float]) -> bool:
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    if max(width, height) < 1200.0:
        return False

    long_axis_lines = [line for line in opening_lines if _is_axis_aligned(line) and line.length >= min(width, height) * 0.35]
    if len(long_axis_lines) < 3:
        return False

    horizontal = [line for line in long_axis_lines if _line_direction(line) == "h"]
    vertical = [line for line in long_axis_lines if _line_direction(line) == "v"]
    return bool(horizontal) and bool(vertical)


def _u_shape_axis(
    opening_lines: Sequence[LineString],
    bounds: tuple[float, float, float, float],
) -> LineString | None:
    minx, miny, maxx, maxy = bounds
    width = maxx - minx
    height = maxy - miny
    long_axis_lines = [line for line in opening_lines if _is_axis_aligned(line) and line.length >= min(width, height) * 0.35]
    if not long_axis_lines:
        return None

    if width >= height:
        y_values = [
            (line.bounds[1] + line.bounds[3]) / 2.0
            for line in long_axis_lines
            if _line_direction(line) == "h"
        ]
        y = _nearest_edge_coordinate(y_values, miny, maxy)
        return LineString([(minx, y), (maxx, y)])

    x_values = [
        (line.bounds[0] + line.bounds[2]) / 2.0
        for line in long_axis_lines
        if _line_direction(line) == "v"
    ]
    x = _nearest_edge_coordinate(x_values, minx, maxx)
    return LineString([(x, miny), (x, maxy)])


def _balcony_axis_geometry(
    opening_lines: Sequence[LineString],
    bounds: tuple[float, float, float, float],
) -> LineString | MultiLineString | None:
    axis_aligned_lines = [line for line in opening_lines if _is_axis_aligned(line, tolerance=0.08)]
    if len(axis_aligned_lines) < 2:
        return _u_shape_axis(opening_lines, bounds)

    axes = extract_wall_axes_from_linework(
        axis_aligned_lines,
        thickness_candidates=None,
        axis_tolerance=8.0,
        snap_tolerance=5.0,
        thickness_tolerance_ratio=0.25,
        thickness_tolerance_abs=30.0,
        min_overlap=30.0,
        min_feature_len=20.0,
        min_score=0.8,
        alignment_tolerance=80.0,
        connection_tolerance=120.0,
        merge_axes=False,
    )
    axis_lines = [axis for axis, _thickness in axes if axis.length > 0]
    if axis_lines:
        return axis_lines[0] if len(axis_lines) == 1 else MultiLineString(axis_lines)

    return _u_shape_axis(opening_lines, bounds)


def _axis_endpoints(wall_axes: Sequence[tuple[LineString, float]]) -> list[WallAxisEndpoint]:
    endpoints: list[WallAxisEndpoint] = []
    for axis_index, (axis, thickness) in enumerate(wall_axes):
        if axis.length <= 0 or len(axis.coords) < 2:
            continue
        coords = list(axis.coords)
        endpoints.append(
            WallAxisEndpoint(
                axis_index=axis_index,
                point=(float(coords[0][0]), float(coords[0][1])),
                at_start=True,
                direction=_endpoint_direction(coords, at_start=True),
                thickness=float(thickness),
            )
        )
        endpoints.append(
            WallAxisEndpoint(
                axis_index=axis_index,
                point=(float(coords[-1][0]), float(coords[-1][1])),
                at_start=False,
                direction=_endpoint_direction(coords, at_start=False),
                thickness=float(thickness),
            )
        )
    return endpoints


def _endpoint_direction(coords, at_start: bool) -> str:
    start = coords[0] if at_start else coords[-1]
    end = coords[1] if at_start else coords[-2]
    return "h" if abs(float(end[0]) - float(start[0])) >= abs(float(end[1]) - float(start[1])) else "v"


def _line_direction(line: LineString) -> str:
    start = line.coords[0]
    end = line.coords[-1]
    return "h" if abs(float(end[0]) - float(start[0])) >= abs(float(end[1]) - float(start[1])) else "v"


def _is_axis_aligned(line: LineString, tolerance: float = 0.04) -> bool:
    start = line.coords[0]
    end = line.coords[-1]
    dx = abs(float(end[0]) - float(start[0]))
    dy = abs(float(end[1]) - float(start[1]))
    length = max(line.length, 1.0)
    return min(dx, dy) / length <= tolerance


def _endpoints_are_aligned(
    first: WallAxisEndpoint,
    second: WallAxisEndpoint,
    tolerance: float,
) -> bool:
    if first.direction == "h":
        return abs(first.point[1] - second.point[1]) <= tolerance
    return abs(first.point[0] - second.point[0]) <= tolerance


def _bounds(lines: Sequence[LineString], padding: float = 0.0) -> tuple[float, float, float, float]:
    minx = min(line.bounds[0] for line in lines) - padding
    miny = min(line.bounds[1] for line in lines) - padding
    maxx = max(line.bounds[2] for line in lines) + padding
    maxy = max(line.bounds[3] for line in lines) + padding
    return minx, miny, maxx, maxy


def _endpoint_pair_key(
    first: WallAxisEndpoint,
    second: WallAxisEndpoint,
) -> tuple[tuple[int, bool], tuple[int, bool]]:
    a = (first.axis_index, first.at_start)
    b = (second.axis_index, second.at_start)
    return (a, b) if a <= b else (b, a)


def _point_as_geometry(point: Point2D) -> Point:
    return Point(point)


def _geometry_endpoints(geometry: LineString | MultiLineString) -> list[Point2D]:
    if isinstance(geometry, LineString):
        coords = list(geometry.coords)
        return [(float(coords[0][0]), float(coords[0][1])), (float(coords[-1][0]), float(coords[-1][1]))]

    points: list[Point2D] = []
    for line in geometry.geoms:
        coords = list(line.coords)
        points.append((float(coords[0][0]), float(coords[0][1])))
        points.append((float(coords[-1][0]), float(coords[-1][1])))
    return points


def _candidate_endpoint_distance(
    candidate_points: Sequence[Point2D],
    first: WallAxisEndpoint,
    second: WallAxisEndpoint,
) -> float:
    return max(
        _distance(candidate_points[0], first.point),
        _distance(candidate_points[1], second.point),
    )


def _endpoint_line_matches_candidate_direction(
    first: WallAxisEndpoint,
    second: WallAxisEndpoint,
    candidate_points: Sequence[Point2D],
    tolerance: float,
) -> bool:
    if len(candidate_points) < 2:
        return True

    dx = abs(candidate_points[1][0] - candidate_points[0][0])
    dy = abs(candidate_points[1][1] - candidate_points[0][1])
    if dx >= dy:
        return abs(first.point[1] - second.point[1]) <= tolerance
    return abs(first.point[0] - second.point[0]) <= tolerance


def _orthogonal_line_from_candidate_direction(
    first: Point2D,
    second: Point2D,
    candidate_points: Sequence[Point2D],
) -> LineString:
    dx = abs(candidate_points[1][0] - candidate_points[0][0])
    dy = abs(candidate_points[1][1] - candidate_points[0][1])
    if dx >= dy:
        y = (first[1] + second[1]) / 2.0
        return LineString([(first[0], y), (second[0], y)])
    x = (first[0] + second[0]) / 2.0
    return LineString([(x, first[1]), (x, second[1])])


def _nearest_endpoint(
    point: Point2D,
    endpoints: Sequence[WallAxisEndpoint],
    max_distance: float,
) -> WallAxisEndpoint | None:
    best: tuple[float, WallAxisEndpoint] | None = None
    for endpoint in endpoints:
        distance = _distance(point, endpoint.point)
        if distance > max_distance:
            continue
        if best is None or distance < best[0]:
            best = (distance, endpoint)
    return None if best is None else best[1]


def _orthogonal_line(first: Point2D, second: Point2D, direction: str) -> LineString:
    if direction == "h":
        y = (first[1] + second[1]) / 2.0
        return LineString([(first[0], y), (second[0], y)])
    x = (first[0] + second[0]) / 2.0
    return LineString([(x, first[1]), (x, second[1])])


def _distance(first: Point2D, second: Point2D) -> float:
    return hypot(first[0] - second[0], first[1] - second[1])


def _connected_line_components(lines: Sequence[LineString], tolerance: float) -> list[list[LineString]]:
    components: list[list[LineString]] = []
    visited: set[int] = set()
    for start_index in range(len(lines)):
        if start_index in visited:
            continue
        stack = [start_index]
        visited.add(start_index)
        component = []
        while stack:
            index = stack.pop()
            line = lines[index]
            component.append(line)
            for neighbor_index, neighbor in enumerate(lines):
                if neighbor_index in visited:
                    continue
                if line.distance(neighbor) > tolerance:
                    continue
                visited.add(neighbor_index)
                stack.append(neighbor_index)
        components.append(component)
    return components


def _unique_component_points(lines: Sequence[LineString], tolerance: float) -> list[Point2D]:
    points: list[Point2D] = []
    for line in lines:
        for x, y in line.coords:
            point = (float(x), float(y))
            if any(_distance(point, existing) <= tolerance for existing in points):
                continue
            points.append(point)
    return points


def _fit_circle(points: Sequence[Point2D]) -> tuple[Point2D, float, float] | None:
    if len(points) < 3:
        return None

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    u = [x - mean_x for x in xs]
    v = [y - mean_y for y in ys]

    suu = sum(item * item for item in u)
    svv = sum(item * item for item in v)
    suv = sum(a * b for a, b in zip(u, v))
    suuu = sum(item**3 for item in u)
    svvv = sum(item**3 for item in v)
    suvv = sum(a * b * b for a, b in zip(u, v))
    svuu = sum(b * a * a for a, b in zip(u, v))
    det = suu * svv - suv * suv
    if abs(det) < 1e-9:
        return None

    rhs_u = 0.5 * (suuu + suvv)
    rhs_v = 0.5 * (svvv + svuu)
    uc = (rhs_u * svv - rhs_v * suv) / det
    vc = (suu * rhs_v - suv * rhs_u) / det
    center = (mean_x + uc, mean_y + vc)
    distances = [_distance(point, center) for point in points]
    radius = sum(distances) / len(distances)
    residual = sqrt(sum((distance - radius) ** 2 for distance in distances) / len(distances))
    return center, radius, residual


def _circle_from_three_points(
    first: Point2D,
    second: Point2D,
    third: Point2D,
) -> tuple[Point2D, float] | None:
    x1, y1 = first
    x2, y2 = second
    x3, y3 = third
    det = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(det) < 1e-9:
        return None

    ux = (
        (x1 * x1 + y1 * y1) * (y2 - y3)
        + (x2 * x2 + y2 * y2) * (y3 - y1)
        + (x3 * x3 + y3 * y3) * (y1 - y2)
    ) / det
    uy = (
        (x1 * x1 + y1 * y1) * (x3 - x2)
        + (x2 * x2 + y2 * y2) * (x1 - x3)
        + (x3 * x3 + y3 * y3) * (x2 - x1)
    ) / det
    center = (ux, uy)
    return center, _distance(center, first)


def _angular_span(center: Point2D, points: Sequence[Point2D]) -> float:
    angles = sorted(atan2(point[1] - center[1], point[0] - center[0]) for point in points)
    if len(angles) < 2:
        return 0.0
    gaps = [angles[index + 1] - angles[index] for index in range(len(angles) - 1)]
    gaps.append(angles[0] + 2.0 * pi - angles[-1])
    return 2.0 * pi - max(gaps)


def _nearest_edge_coordinate(values: Sequence[float], low: float, high: float) -> float:
    if not values:
        return (low + high) / 2.0
    return min(values, key=lambda value: min(abs(value - low), abs(value - high)))
