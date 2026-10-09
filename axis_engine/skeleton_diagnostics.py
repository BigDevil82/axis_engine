from __future__ import annotations

from typing import Sequence

from shapely.geometry import LineString
from shapely.ops import unary_union

from axis_engine.geometry_utils import iter_straight_segments
from axis_engine.opening_embedment import OpeningEmbedment


def find_skeleton_isolated_points(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments: Sequence[OpeningEmbedment],
    min_segment_length: float = 1.0,
) -> list[tuple[float, float]]:
    """Return dangling endpoints in the calibrated skeleton network."""
    lines = [line for line, _thickness in wall_axes]
    lines.extend(embedment.embed_line for embedment in opening_embedments)
    network = unary_union([line for line in lines if line.length >= min_segment_length])

    degrees: dict[tuple[float, float], int] = {}
    points: dict[tuple[float, float], tuple[float, float]] = {}
    for segment in iter_straight_segments(network, min_length=min_segment_length):
        start = _point(segment.coords[0])
        end = _point(segment.coords[-1])
        for point in (start, end):
            key = _point_key(point)
            degrees[key] = degrees.get(key, 0) + 1
            points.setdefault(key, point)

    return [points[key] for key, degree in sorted(degrees.items()) if degree == 1]


def _point(coord) -> tuple[float, float]:
    return (float(coord[0]), float(coord[1]))


def _point_key(point: tuple[float, float], precision: int = 2) -> tuple[float, float]:
    return (round(point[0], precision), round(point[1], precision))
