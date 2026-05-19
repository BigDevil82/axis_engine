from __future__ import annotations

from typing import Sequence

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from axis_engine.opening_embedment import OpeningEmbedment
from axis_engine.structural_design.models import Beam, BeamKind, SlabRegion


def infer_balcony_beams(
    slab_regions: Sequence[SlabRegion],
    opening_embedments: Sequence[OpeningEmbedment],
    existing_beams: Sequence[Beam],
    edge_tolerance: float = 120.0,
    min_edge_length: float = 800.0,
    min_opening_overlap_ratio: float = 0.55,
) -> list[Beam]:
    opening_union = unary_union(
        [embedment.embed_line for embedment in opening_embedments if embedment.embed_line.length > 0]
    )
    if opening_union.is_empty:
        return []

    outer_contour = _slab_footprint_outer_contour(slab_regions)
    if outer_contour.is_empty:
        return []

    existing_union = unary_union([beam.axis for beam in existing_beams if beam.axis.length > 0])
    beams: list[Beam] = []
    seen: set[tuple[tuple[float, float], tuple[float, float]]] = set()
    for index, region in enumerate(slab_regions):
        polygon = region.recovered_polygon
        if not _is_rectangular_region(polygon, edge_tolerance):
            continue
        for line in _balcony_inner_edges(
            polygon,
            opening_union,
            outer_contour,
            edge_tolerance,
            min_edge_length,
            min_opening_overlap_ratio,
        ):
            if _covered_by_existing(line, existing_union):
                continue
            key = _line_key(line)
            if key in seen:
                continue
            seen.add(key)
            beams.append(
                Beam(
                    axis=line,
                    kind=BeamKind.BALCONY,
                    reason="balcony_opposite_opening_edge",
                    related_ids=(index,),
                )
            )
    return beams


def _balcony_inner_edges(
    polygon: Polygon,
    opening_union,
    outer_contour,
    tolerance: float,
    min_length: float,
    min_opening_overlap_ratio: float,
) -> list[LineString]:
    opening_edges = [
        edge
        for edge in _polygon_edges(polygon)
        if edge.length >= min_length
        and _opening_overlap_ratio(edge, opening_union, tolerance) >= min_opening_overlap_ratio
    ]
    if len(opening_edges) < 2:
        return []

    beams: list[LineString] = []
    for first in opening_edges:
        for second in opening_edges:
            if first.equals(second):
                continue
            if not _opposite_parallel_edges(first, second, tolerance):
                continue
            first_outer = first.distance(outer_contour) <= tolerance
            second_outer = second.distance(outer_contour) <= tolerance
            if first_outer == second_outer:
                continue
            beams.append(second if first_outer else first)
    return beams


def _opening_overlap_ratio(edge: LineString, opening_union, tolerance: float) -> float:
    if opening_union.is_empty or edge.length <= 0:
        return 0.0
    covered = edge.intersection(
        opening_union.buffer(tolerance, cap_style="square", join_style="mitre")
    )
    return covered.length / max(edge.length, 1.0)


def _opposite_parallel_edges(first: LineString, second: LineString, tolerance: float) -> bool:
    first_feature = _line_feature(first, tolerance)
    second_feature = _line_feature(second, tolerance)
    if first_feature is None or second_feature is None:
        return False
    if first_feature[0] != second_feature[0]:
        return False
    if abs(first_feature[1] - second_feature[1]) <= tolerance:
        return False
    overlap = min(first_feature[3], second_feature[3]) - max(first_feature[2], second_feature[2])
    return overlap / max(min(first_feature[4], second_feature[4]), 1.0) >= 0.75


def _is_rectangular_region(polygon: Polygon, tolerance: float) -> bool:
    if polygon.is_empty or polygon.area <= 0:
        return False
    minx, miny, maxx, maxy = polygon.bounds
    bbox_area = max((maxx - minx) * (maxy - miny), 1.0)
    if polygon.area / bbox_area < 0.93:
        return False
    return all(_line_feature(edge, tolerance) is not None for edge in _polygon_edges(polygon))


def _line_feature(line: LineString, tolerance: float):
    start, end = line.coords[0], line.coords[-1]
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    if dy <= tolerance and dx > tolerance:
        x1, x2 = sorted((start[0], end[0]))
        return "h", (start[1] + end[1]) / 2.0, x1, x2, x2 - x1
    if dx <= tolerance and dy > tolerance:
        y1, y2 = sorted((start[1], end[1]))
        return "v", (start[0] + end[0]) / 2.0, y1, y2, y2 - y1
    return None


def _polygon_edges(polygon: Polygon) -> list[LineString]:
    coords = list(polygon.exterior.coords)
    edges: list[LineString] = []
    for start, end in zip(coords, coords[1:]):
        line = LineString([start, end])
        if line.length > 0:
            edges.append(line)
    return edges


def _slab_footprint_outer_contour(slab_regions: Sequence[SlabRegion]):
    footprint_parts = [
        region.recovered_polygon
        for region in slab_regions
        if not region.recovered_polygon.is_empty
    ]
    if not footprint_parts:
        return LineString()

    footprint = unary_union(footprint_parts)
    contours = [
        LineString(polygon.exterior.coords)
        for polygon in _iter_polygons(footprint)
        if not polygon.is_empty
    ]
    if not contours:
        return LineString()
    return unary_union(contours)


def _iter_polygons(geometry):
    if isinstance(geometry, Polygon):
        yield geometry
        return
    if hasattr(geometry, "geoms"):
        for geom in geometry.geoms:
            yield from _iter_polygons(geom)


def _covered_by_existing(line: LineString, existing_union) -> bool:
    if existing_union.is_empty:
        return False
    return line.difference(existing_union.buffer(1.0, cap_style="square", join_style="mitre")).length <= 1.0


def _line_key(line: LineString, precision: int = 2) -> tuple[tuple[float, float], tuple[float, float]]:
    start = tuple(round(value, precision) for value in line.coords[0])
    end = tuple(round(value, precision) for value in line.coords[-1])
    return tuple(sorted((start, end)))
