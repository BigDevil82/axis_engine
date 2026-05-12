from __future__ import annotations

import numbers
from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.opening_embedment import OpeningEmbedment


@dataclass(frozen=True)
class SkeletonPruneResult:
    wall_axes: list[tuple[LineString, float]]
    opening_embedments: list[OpeningEmbedment]


@dataclass(frozen=True)
class _GraphEdge:
    line: LineString
    start_key: tuple[float, float]
    end_key: tuple[float, float]


@dataclass(frozen=True)
class _SemanticEdge:
    line: LineString
    kind: str
    thickness: float | None = None
    opening: OpeningEmbedment | None = None

    @property
    def start(self) -> tuple[float, float]:
        coord = self.line.coords[0]
        return (float(coord[0]), float(coord[1]))

    @property
    def end(self) -> tuple[float, float]:
        coord = self.line.coords[-1]
        return (float(coord[0]), float(coord[1]))

    @property
    def start_key(self) -> tuple[float, float]:
        return _node_key(self.start)

    @property
    def end_key(self) -> tuple[float, float]:
        return _node_key(self.end)


def prune_skeleton_spurs(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    spur_length: float = 300.0,
    stitch_gap_distance: float = 250.0,
    stitch_probe_width: float = 5.0,
    stitch_min_incident_length: float = 400.0,
    min_segment_length: float = 1.0,
) -> SkeletonPruneResult:
    skeleton = _line_union(
        [line for line, _thickness in wall_axes] + [item.embed_line for item in opening_embedments]
    )
    pruned_skeleton = _prune_degree_one_edges(skeleton, spur_length, min_segment_length)
    if pruned_skeleton.is_empty:
        return SkeletonPruneResult([], [])

    clean_walls = _assign_clean_walls_by_thickness(wall_axes, pruned_skeleton, min_segment_length)
    clean_wall_union = _line_union([line for line, _thickness in clean_walls])
    clean_openings = _assign_clean_openings(
        opening_embedments, pruned_skeleton, clean_wall_union, min_segment_length
    )
    clean_walls, clean_openings = _stitch_dangling_semantic_edges(
        clean_walls,
        clean_openings,
        gap_distance=stitch_gap_distance,
        probe_width=stitch_probe_width,
        min_incident_length=stitch_min_incident_length,
        min_segment_length=min_segment_length,
    )
    return SkeletonPruneResult(clean_walls, clean_openings)


def _prune_degree_one_edges(
    skeleton,
    spur_length: float,
    min_segment_length: float,
):
    edges = _graph_edges_from_skeleton(skeleton, min_segment_length)
    if not edges:
        return unary_union([])

    active = [True] * len(edges)
    changed = True
    while changed:
        changed = False
        degrees = _node_degrees(edge for index, edge in enumerate(edges) if active[index])
        for index, edge in enumerate(edges):
            if not active[index]:
                continue
            if edge.line.length >= spur_length:
                continue
            if degrees.get(edge.start_key, 0) == 1 or degrees.get(edge.end_key, 0) == 1:
                active[index] = False
                changed = True

    remaining = [
        edge.line
        for index, edge in enumerate(edges)
        if active[index] and edge.line.length >= min_segment_length
    ]
    return _line_union(remaining)


def _graph_edges_from_skeleton(skeleton, min_segment_length: float) -> list[_GraphEdge]:
    noded = unary_union(list(iter_straight_segments(skeleton, min_length=min_segment_length)))
    edges: list[_GraphEdge] = []
    for segment in iter_straight_segments(noded, min_length=min_segment_length):
        coords = list(segment.coords)
        start = (float(coords[0][0]), float(coords[0][1]))
        end = (float(coords[-1][0]), float(coords[-1][1]))
        edges.append(_GraphEdge(segment, _node_key(start), _node_key(end)))
    return edges


def _node_degrees(edges) -> dict[tuple[float, float], int]:
    degrees: dict[tuple[float, float], int] = {}
    for edge in edges:
        degrees[edge.start_key] = degrees.get(edge.start_key, 0) + 1
        degrees[edge.end_key] = degrees.get(edge.end_key, 0) + 1
    return degrees


def _assign_clean_walls_by_thickness(
    wall_axes: Sequence[tuple[LineString, float]],
    pruned_skeleton,
    min_segment_length: float,
) -> list[tuple[LineString, float]]:
    wall_lines_by_thickness: dict[float, list[LineString]] = {}
    for line, thickness in wall_axes:
        wall_lines_by_thickness.setdefault(thickness, []).append(line)

    thickness_order = sorted(
        wall_lines_by_thickness,
        key=lambda thickness: sum(line.length for line in wall_lines_by_thickness[thickness]),
        reverse=True,
    )

    remaining = pruned_skeleton
    clean_walls: list[tuple[LineString, float]] = []
    for thickness in thickness_order:
        if remaining.is_empty:
            break
        thickness_union = _line_union(wall_lines_by_thickness[thickness])
        clean_geometry = remaining.intersection(thickness_union)
        for line in _clean_lines(clean_geometry, min_segment_length):
            clean_walls.append((line, thickness))
        if not clean_geometry.is_empty:
            remaining = remaining.difference(clean_geometry)
    return clean_walls


def _assign_clean_openings(
    opening_embedments: Sequence[OpeningEmbedment],
    pruned_skeleton,
    clean_wall_union,
    min_segment_length: float,
) -> list[OpeningEmbedment]:
    opening_space = pruned_skeleton.difference(clean_wall_union)
    if opening_space.is_empty:
        return []

    clean_openings: list[OpeningEmbedment] = []
    for embedment in opening_embedments:
        clean_geometry = opening_space.intersection(embedment.embed_line)
        for line in _clean_lines(clean_geometry, min_segment_length):
            clean_openings.append(
                OpeningEmbedment(
                    opening_type=embedment.opening_type,
                    embed_line=line,
                    cluster_index=embedment.cluster_index,
                    confidence=embedment.confidence,
                    reason=f"{embedment.reason}|spur_pruned",
                )
            )
    return clean_openings


def _stitch_dangling_semantic_edges(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    gap_distance: float,
    probe_width: float,
    min_incident_length: float,
    min_segment_length: float,
) -> tuple[list[tuple[LineString, float]], list[OpeningEmbedment]]:
    edges = _semantic_edges(wall_axes, opening_embedments, min_segment_length)
    if not edges:
        return list(wall_axes), list(opening_embedments)

    degrees = _semantic_node_degrees(edges)
    line_geoms = [edge.line for edge in edges]
    tree = STRtree(line_geoms)
    bridges: list[_SemanticEdge] = []
    bridge_keys: set[tuple[tuple[float, float], tuple[float, float], str]] = set()

    for index, edge in enumerate(edges):
        if edge.line.length < min_incident_length:
            continue
        for endpoint, endpoint_key, other_point in (
            (edge.start, edge.start_key, edge.end),
            (edge.end, edge.end_key, edge.start),
        ):
            if degrees.get(endpoint_key, 0) != 1:
                continue

            probe = _extension_probe(endpoint, other_point, gap_distance)
            if probe is None:
                continue

            bridge = _nearest_bridge(edge, index, endpoint, probe, tree, line_geoms, gap_distance, probe_width)
            if bridge is None or bridge.line.length < min_segment_length:
                continue

            key = _bridge_key(bridge)
            if key in bridge_keys:
                continue
            bridge_keys.add(key)
            bridges.append(bridge)

    if not bridges:
        return list(wall_axes), list(opening_embedments)

    stitched_walls = list(wall_axes)
    stitched_openings = list(opening_embedments)
    for bridge in bridges:
        if bridge.kind == "wall" and bridge.thickness is not None:
            stitched_walls.append((bridge.line, bridge.thickness))
        elif bridge.kind == "opening" and bridge.opening is not None:
            stitched_openings.append(
                OpeningEmbedment(
                    opening_type=bridge.opening.opening_type,
                    embed_line=bridge.line,
                    cluster_index=bridge.opening.cluster_index,
                    confidence=bridge.opening.confidence,
                    reason=f"{bridge.opening.reason}|gap_stitched",
                )
            )

    return stitched_walls, stitched_openings


def _semantic_edges(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    min_segment_length: float,
) -> list[_SemanticEdge]:
    edges: list[_SemanticEdge] = []
    for line, thickness in wall_axes:
        for segment in iter_straight_segments(line, min_length=min_segment_length):
            edges.append(_SemanticEdge(segment, "wall", thickness=thickness))
    for embedment in opening_embedments:
        for segment in iter_straight_segments(embedment.embed_line, min_length=min_segment_length):
            edges.append(_SemanticEdge(segment, "opening", opening=embedment))
    return edges


def _semantic_node_degrees(edges: Sequence[_SemanticEdge]) -> dict[tuple[float, float], int]:
    degrees: dict[tuple[float, float], int] = {}
    for edge in edges:
        degrees[edge.start_key] = degrees.get(edge.start_key, 0) + 1
        degrees[edge.end_key] = degrees.get(edge.end_key, 0) + 1
    return degrees


def _extension_probe(
    endpoint: tuple[float, float],
    other_point: tuple[float, float],
    distance: float,
) -> LineString | None:
    dx = endpoint[0] - other_point[0]
    dy = endpoint[1] - other_point[1]
    if abs(dx) >= abs(dy):
        if abs(dx) <= 1e-9:
            return None
        direction = 1.0 if dx > 0 else -1.0
        return LineString([endpoint, (endpoint[0] + direction * distance, endpoint[1])])

    if abs(dy) <= 1e-9:
        return None
    direction = 1.0 if dy > 0 else -1.0
    return LineString([endpoint, (endpoint[0], endpoint[1] + direction * distance)])


def _nearest_bridge(
    edge: _SemanticEdge,
    edge_index: int,
    endpoint: tuple[float, float],
    probe: LineString,
    tree: STRtree,
    line_geoms: Sequence[LineString],
    gap_distance: float,
    probe_width: float,
) -> _SemanticEdge | None:
    endpoint_point = Point(endpoint)
    best_point = None
    best_distance = float("inf")
    search_geometry = probe.buffer(probe_width, cap_style=2, join_style=2)
    for result in tree.query(search_geometry):
        candidate_index = _tree_result_index(result, line_geoms)
        if candidate_index == edge_index:
            continue

        candidate = line_geoms[candidate_index]
        intersection = probe.intersection(candidate)
        for point in _intersection_points(intersection):
            distance = endpoint_point.distance(point)
            if distance <= 1e-6 or distance > gap_distance:
                continue
            if distance < best_distance:
                best_distance = distance
                best_point = (float(point.x), float(point.y))

    if best_point is None:
        return None
    return _SemanticEdge(LineString([endpoint, best_point]), edge.kind, edge.thickness, edge.opening)


def _intersection_points(geometry) -> list[Point]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, Point):
        return [geometry]
    if isinstance(geometry, LineString):
        coords = list(geometry.coords)
        if not coords:
            return []
        return [Point(coords[0]), Point(coords[-1])]
    if hasattr(geometry, "geoms"):
        points: list[Point] = []
        for geom in geometry.geoms:
            points.extend(_intersection_points(geom))
        return points
    return []


def _bridge_key(edge: _SemanticEdge) -> tuple[tuple[float, float], tuple[float, float], str]:
    first = _node_key(edge.start)
    second = _node_key(edge.end)
    ordered = (first, second) if first <= second else (second, first)
    return (ordered[0], ordered[1], edge.kind)


def _clean_lines(geometry, min_segment_length: float) -> list[LineString]:
    return [line for line in iter_lines(geometry) if line.length >= min_segment_length]


def _line_union(lines: Sequence[LineString]):
    non_empty = [line for line in lines if line.length > 0]
    if not non_empty:
        return unary_union([])
    return unary_union(non_empty)


def _node_key(point: tuple[float, float], precision: int = 2) -> tuple[float, float]:
    return (round(point[0], precision), round(point[1], precision))


def _tree_result_index(result, geoms: Sequence[LineString]) -> int:
    if isinstance(result, numbers.Integral):
        return int(result)
    geom_to_index = {id(geom): index for index, geom in enumerate(geoms)}
    return geom_to_index[id(result)]
