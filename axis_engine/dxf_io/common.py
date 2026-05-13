from __future__ import annotations

from collections.abc import Iterable

from shapely.geometry import LineString

from axis_engine.geometry_utils import iter_lines
from axis_engine.opening_embedment import OpeningEmbedment

APPID = "AXIS_ENGINE"


def opening_lines(items: Iterable[OpeningEmbedment | LineString]) -> Iterable[LineString]:
    for item in items:
        geometry = item.embed_line if isinstance(item, OpeningEmbedment) else item
        yield from iter_lines(geometry)


def thickness_to_lineweight(thickness: float) -> int:
    # DXF lineweight is stored in 1/100 mm and has a finite standard range.
    return max(13, min(211, int(round(thickness))))


def read_thickness_from_entity(entity, default: float) -> float:
    try:
        xdata = entity.get_xdata(APPID)
    except Exception:
        xdata = ()
    for code, value in xdata:
        if code == 1040:
            return float(value)

    lineweight = int(entity.dxf.get("lineweight", -1))
    if lineweight > 0:
        return float(lineweight)
    return default


def read_thickness_from_com_entity(entity, default: float) -> float:
    try:
        lineweight = int(entity.Lineweight)
    except Exception:
        lineweight = -1
    if lineweight > 0:
        return float(lineweight)
    return default


def line_endpoints_3d(line: LineString):
    coords = list(line.coords)
    start = coords[0]
    end = coords[-1]
    return (float(start[0]), float(start[1]), 0.0), (float(end[0]), float(end[1]), 0.0)
