from __future__ import annotations

import numbers
from collections import deque
from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString
from shapely.strtree import STRtree

from cad_tests.dxf_utils import DxfLineSegment


@dataclass(frozen=True)
class OpeningCluster:
    segments: tuple[DxfLineSegment, ...]

    @property
    def lines(self) -> list[LineString]:
        return [segment.line for segment in self.segments]

    @property
    def has_arc(self) -> bool:
        return any(segment.is_arc for segment in self.segments)

    @property
    def arc_lines(self) -> list[LineString]:
        return [segment.line for segment in self.segments if segment.is_arc]

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return cluster_bounds(self.lines)


def cluster_opening_segments(
    segments: Sequence[DxfLineSegment],
    distance: float = 120.0,
    min_segments: int = 2,
    max_bbox_size: float = 5000.0,
) -> list[OpeningCluster]:
    if not segments:
        return []

    lines = [segment.line for segment in segments]
    tree = STRtree(lines)
    visited: set[int] = set()
    clusters: list[OpeningCluster] = []

    for start_index in range(len(lines)):
        if start_index in visited:
            continue

        queue = deque([start_index])
        visited.add(start_index)
        component_indices = []

        while queue:
            index = queue.popleft()
            component_indices.append(index)
            search_geom = lines[index].buffer(distance, cap_style=2, join_style=2)

            for neighbor_index in _query_tree_indices(tree, lines, search_geom):
                if neighbor_index in visited:
                    continue
                if lines[index].distance(lines[neighbor_index]) > distance:
                    continue
                visited.add(neighbor_index)
                queue.append(neighbor_index)

        cluster = OpeningCluster(tuple(segments[index] for index in component_indices))
        if not _is_valid_cluster(cluster, min_segments, max_bbox_size):
            continue
        clusters.append(cluster)

    return sorted(clusters, key=lambda item: (item.bounds[0], item.bounds[1]))


def cluster_line_segments(
    lines: Sequence[LineString],
    distance: float = 120.0,
    min_lines: int = 2,
    max_bbox_size: float = 5000.0,
) -> list[list[LineString]]:
    segments = [DxfLineSegment(line=line, source_type="UNKNOWN", is_arc=False) for line in lines]
    clusters = cluster_opening_segments(
        segments,
        distance=distance,
        min_segments=min_lines,
        max_bbox_size=max_bbox_size,
    )
    return [cluster.lines for cluster in clusters]


def cluster_bounds(lines: Sequence[LineString], padding: float = 0.0) -> tuple[float, float, float, float]:
    minx = min(line.bounds[0] for line in lines) - padding
    miny = min(line.bounds[1] for line in lines) - padding
    maxx = max(line.bounds[2] for line in lines) + padding
    maxy = max(line.bounds[3] for line in lines) + padding
    return minx, miny, maxx, maxy


def _is_valid_cluster(cluster: OpeningCluster, min_segments: int, max_bbox_size: float) -> bool:
    if len(cluster.segments) < min_segments:
        return False

    minx, miny, maxx, maxy = cluster.bounds
    width = maxx - minx
    height = maxy - miny
    if width <= 1.0 and height <= 1.0:
        return False
    return width <= max_bbox_size and height <= max_bbox_size


def _query_tree_indices(tree: STRtree, geoms, geometry) -> list[int]:
    result = tree.query(geometry)
    if len(result) == 0:
        return []

    first = result[0]
    if isinstance(first, numbers.Integral):
        return [int(index) for index in result]

    geom_to_index = {id(geom): index for index, geom in enumerate(geoms)}
    return [geom_to_index[id(geom)] for geom in result]
