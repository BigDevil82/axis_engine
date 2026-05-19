from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, Point

from axis_engine.geometry_utils import iter_straight_segments
from axis_engine.structural_design.models import ShearWall


@dataclass(frozen=True)
class ShearWallLayoutOptions:
    connection_tolerance: float = 20.0
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
    raw_walls = [
        ShearWall(axis=segment, thickness=thickness)
        for line, thickness in wall_axes
        if thickness == dominant_thickness and line.length > 0
        for segment in iter_straight_segments(line, min_length=options.min_segment_length)
    ]
    connected_walls = _remove_isolated_walls(raw_walls, options.connection_tolerance)
    return _shorten_long_walls(connected_walls, options)


def _remove_isolated_walls(
    walls: Sequence[ShearWall],
    tolerance: float,
) -> list[ShearWall]:
    if len(walls) <= 1:
        return []
    return [
        wall
        for index, wall in enumerate(walls)
        if _connected_at_endpoint(wall.axis.coords[0], index, walls, tolerance)
        or _connected_at_endpoint(wall.axis.coords[-1], index, walls, tolerance)
    ]


def _connected_at_endpoint(
    point,
    wall_index: int,
    walls: Sequence[ShearWall],
    tolerance: float,
) -> bool:
    for index, wall in enumerate(walls):
        if index == wall_index:
            continue
        if wall.axis.distance(Point(point)) <= tolerance:
            return True
    return False


def _shorten_long_walls(
    walls: Sequence[ShearWall],
    options: ShearWallLayoutOptions,
) -> list[ShearWall]:
    adjusted: list[ShearWall] = []
    for wall in walls:
        if wall.axis.length <= options.long_wall_length:
            adjusted.append(wall)
            continue
        adjusted.extend(_split_long_wall(wall, options))
    return adjusted


def _split_long_wall(wall: ShearWall, options: ShearWallLayoutOptions) -> list[ShearWall]:
    length = wall.axis.length
    keep = min(options.max_split_segment_length, length / 2.0)
    if keep <= options.min_segment_length:
        return []

    start_part = _subline(wall.axis, 0.0, keep)
    end_part = _subline(wall.axis, length - keep, length)
    return [
        ShearWall(start_part, wall.thickness, f"{wall.source}|shortened"),
        ShearWall(end_part, wall.thickness, f"{wall.source}|shortened"),
    ]


def _subline(line: LineString, start_distance: float, end_distance: float) -> LineString:
    start = line.interpolate(start_distance)
    end = line.interpolate(end_distance)
    return LineString([(start.x, start.y), (end.x, end.y)])
