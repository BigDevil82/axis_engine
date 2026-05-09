from __future__ import annotations

from collections.abc import Iterable

from shapely.geometry import LineString, MultiLineString


def iter_lines(geometry) -> Iterable[LineString]:
    if isinstance(geometry, LineString):
        yield geometry
        return
    if isinstance(geometry, MultiLineString):
        for line in geometry.geoms:
            yield line
        return
    if hasattr(geometry, "geoms"):
        for geom in geometry.geoms:
            yield from iter_lines(geom)


def iter_straight_segments(geometry, min_length: float = 1.0) -> Iterable[LineString]:
    for line in iter_lines(geometry):
        coords = list(line.coords)
        if len(coords) < 2:
            continue
        for start, end in zip(coords, coords[1:]):
            segment = LineString([start, end])
            if segment.length >= min_length:
                yield segment

