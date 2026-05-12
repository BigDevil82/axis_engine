from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString
from shapely.ops import unary_union

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


def prune_skeleton_spurs(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    spur_length: float = 300.0,
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


def _clean_lines(geometry, min_segment_length: float) -> list[LineString]:
    return [line for line in iter_lines(geometry) if line.length >= min_segment_length]


def _line_union(lines: Sequence[LineString]):
    non_empty = [line for line in lines if line.length > 0]
    if not non_empty:
        return unary_union([])
    return unary_union(non_empty)


def _node_key(point: tuple[float, float], precision: int = 6) -> tuple[float, float]:
    return (round(point[0], precision), round(point[1], precision))
