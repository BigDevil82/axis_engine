from __future__ import annotations

from collections.abc import Iterable, Sequence
from typing import Any

from shapely.geometry import LineString, Polygon

from axis_engine.dxf_utils import DxfGeometry
from axis_engine.opening_embedment import OpeningEmbedment
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall, SlabRegion


def geometry_from_payload(item: dict[str, Any]) -> DxfGeometry:
    return DxfGeometry(
        geom_type=str(item["geom_type"]).upper(),
        params=_tuplify_params(dict(item.get("params", {}))),
        layer=str(item.get("layer", "")),
        source_type=str(item.get("source_type", item.get("geom_type", ""))).upper(),
        block_path=tuple(item.get("block_path", ())),
    )


def geometries_from_payload(items: Iterable[dict[str, Any]]) -> list[DxfGeometry]:
    return [geometry_from_payload(item) for item in items]


def wall_axis_from_payload(item: dict[str, Any]) -> tuple[LineString, float]:
    return line_from_payload(item["line"]), float(item.get("thickness", 200.0))


def wall_axes_from_payload(items: Iterable[dict[str, Any]]) -> list[tuple[LineString, float]]:
    return [wall_axis_from_payload(item) for item in items]


def opening_from_payload(item: dict[str, Any], index: int = 0) -> OpeningEmbedment:
    return OpeningEmbedment(
        opening_type=str(item.get("opening_type", "opening")),
        embed_line=line_from_payload(item["line"]),
        cluster_index=int(item.get("cluster_index", index)),
        confidence=float(item.get("confidence", 1.0)),
        reason=str(item.get("reason", "manual")),
    )


def openings_from_payload(items: Iterable[dict[str, Any]]) -> list[OpeningEmbedment]:
    return [opening_from_payload(item, index) for index, item in enumerate(items)]


def shear_wall_from_payload(item: dict[str, Any]) -> ShearWall:
    return ShearWall(
        axis=line_from_payload(item["line"]),
        thickness=float(item.get("thickness", 200.0)),
        source=str(item.get("source", "manual")),
    )


def shear_walls_from_payload(items: Iterable[dict[str, Any]]) -> list[ShearWall]:
    return [shear_wall_from_payload(item) for item in items]


def beam_from_payload(item: dict[str, Any]) -> Beam:
    return Beam(
        axis=line_from_payload(item["line"]),
        kind=BeamKind(str(item.get("kind", BeamKind.PERIMETER.value))),
        reason=str(item.get("reason", "manual")),
        related_ids=tuple(int(value) for value in item.get("related_ids", ())),
    )


def beams_from_payload(items: Iterable[dict[str, Any]]) -> list[Beam]:
    return [beam_from_payload(item) for item in items]


def lines_from_payload(items: Iterable[Sequence[Sequence[float]] | dict[str, Any]]) -> list[LineString]:
    return [line_from_payload(item["line"] if isinstance(item, dict) else item) for item in items]


def line_from_payload(points: Sequence[Sequence[float]]) -> LineString:
    return LineString([(float(point[0]), float(point[1])) for point in points])


def wall_axis_to_payload(line: LineString, thickness: float, index: int) -> dict[str, Any]:
    return {
        "id": f"wall_axis_{index}",
        "line": line_to_payload(line),
        "thickness": thickness,
    }


def opening_to_payload(opening: OpeningEmbedment, index: int) -> dict[str, Any]:
    return {
        "id": f"opening_embedment_{index}",
        "line": line_to_payload(opening.embed_line),
        "opening_type": opening.opening_type,
        "cluster_index": opening.cluster_index,
        "confidence": opening.confidence,
        "reason": opening.reason,
    }


def shear_wall_to_payload(wall: ShearWall, index: int) -> dict[str, Any]:
    return {
        "id": f"shear_wall_{index}",
        "line": line_to_payload(wall.axis),
        "thickness": wall.thickness,
        "source": wall.source,
    }


def beam_to_payload(beam: Beam, index: int) -> dict[str, Any]:
    return {
        "id": f"beam_{index}",
        "line": line_to_payload(beam.axis),
        "kind": beam.kind.value,
        "reason": beam.reason,
        "related_ids": list(beam.related_ids),
    }


def slab_region_to_payload(region: SlabRegion, index: int) -> dict[str, Any]:
    return {
        "id": f"slab_region_{index}",
        "polygon": polygon_to_payload(region.recovered_polygon),
        "inner_boundary": line_to_payload(region.inner_boundary),
        "source_polygon_index": region.source_polygon_index,
        "hole_index": region.hole_index,
        "area": region.recovered_polygon.area,
    }


def line_to_payload(line: LineString) -> list[list[float]]:
    return [[float(x), float(y)] for x, y in line.coords]


def polygon_to_payload(polygon: Polygon) -> dict[str, Any]:
    return {
        "exterior": [[float(x), float(y)] for x, y in polygon.exterior.coords],
        "interiors": [
            [[float(x), float(y)] for x, y in interior.coords]
            for interior in polygon.interiors
        ],
    }


def _tuplify_params(params: dict[str, Any]) -> dict[str, Any]:
    return {key: _tuplify_value(value) for key, value in params.items()}


def _tuplify_value(value):
    if isinstance(value, list):
        if value and all(isinstance(item, (int, float)) for item in value):
            return tuple(float(item) for item in value)
        return [_tuplify_value(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_tuplify_value(item) for item in value)
    return value
