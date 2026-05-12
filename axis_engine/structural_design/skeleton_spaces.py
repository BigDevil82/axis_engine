from __future__ import annotations

from collections.abc import Iterable

from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import linemerge, unary_union

from axis_engine.geometry_utils import iter_lines
from axis_engine.opening_embedment import OpeningEmbedment
from axis_engine.structural_design.models import SlabRegion


def skeleton_lines(
    wall_axes: Iterable[tuple[LineString, float]],
    opening_embedments: Iterable[OpeningEmbedment],
) -> list[LineString]:
    lines = [line for line, _thickness in wall_axes if line.length > 0]
    for embedment in opening_embedments:
        lines.extend(line for line in iter_lines(embedment.embed_line) if line.length > 0)
    return lines


def build_buffered_network(
    wall_axes: Iterable[tuple[LineString, float]],
    opening_embedments: Iterable[OpeningEmbedment],
    buffer_distance: float,
):
    lines = skeleton_lines(wall_axes, opening_embedments)
    if not lines:
        return MultiLineString().buffer(0)

    network = linemerge(unary_union(lines))
    return network.buffer(buffer_distance, cap_style="square", join_style="mitre")


def buffered_network_polygons(buffered_network) -> list[Polygon]:
    if buffered_network.is_empty:
        return []
    geoms = list(buffered_network.geoms) if hasattr(buffered_network, "geoms") else [buffered_network]
    return [geom for geom in geoms if isinstance(geom, Polygon) and not geom.is_empty]


def extract_slab_regions(buffered_network, recovery_buffer: float) -> list[SlabRegion]:
    regions: list[SlabRegion] = []
    for polygon_index, polygon in enumerate(buffered_network_polygons(buffered_network)):
        for hole_index, interior in enumerate(polygon.interiors):
            inner_boundary = LineString(interior.coords)
            recovered_polygon = Polygon(interior).buffer(
                recovery_buffer,
                cap_style="square",
                join_style="mitre",
            )
            if recovered_polygon.is_empty:
                continue
            regions.append(
                SlabRegion(
                    inner_boundary=inner_boundary,
                    recovered_polygon=recovered_polygon,
                    source_polygon_index=polygon_index,
                    hole_index=hole_index,
                )
            )
    return regions
