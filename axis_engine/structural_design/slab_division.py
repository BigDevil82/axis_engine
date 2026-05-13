from __future__ import annotations

from dataclasses import dataclass
from math import inf
from typing import Sequence

from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import split, unary_union

from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall, SlabRegion
from axis_engine.structural_design.skeleton_spaces import buffered_network_polygons, extract_slab_regions


@dataclass(frozen=True)
class SlabDivisionOptions:
    max_edge_length: float = 6000.0
    min_split_length: float = 1000.0
    min_area_ratio: float = 0.25
    axis_snap_tolerance: float = 300.0
    edge_axis_tolerance: float = 8.0
    max_divisions_per_region: int = 1


def slab_regions_from_structural_lines(
    shear_walls: Sequence[ShearWall],
    beams: Sequence[Beam],
    buffer_distance: float,
) -> list[SlabRegion]:
    lines = [wall.axis for wall in shear_walls if wall.axis.length > 0]
    lines.extend(beam.axis for beam in beams if beam.axis.length > 0)
    if not lines:
        return []

    network = unary_union(lines)
    buffered = network.buffer(buffer_distance, cap_style="square", join_style="mitre")
    return extract_slab_regions(buffered, buffer_distance)


def infer_slab_divider_beams(
    slab_regions: Sequence[SlabRegion],
    existing_beams: Sequence[Beam],
    axis_linework: Sequence[LineString],
    options: SlabDivisionOptions,
) -> list[Beam]:
    axis_x, axis_y = _reference_axis_values(axis_linework)
    existing_union = unary_union([beam.axis for beam in existing_beams if beam.axis.length > 0])
    divider_beams: list[Beam] = []

    for region_index, region in enumerate(slab_regions):
        region_beams = _divider_beams_for_region(
            region.recovered_polygon,
            region_index,
            axis_x,
            axis_y,
            existing_union,
            options,
        )
        divider_beams.extend(region_beams[: options.max_divisions_per_region])

    return divider_beams


def _divider_beams_for_region(
    polygon: Polygon,
    region_index: int,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    existing_union,
    options: SlabDivisionOptions,
) -> list[Beam]:
    if polygon.is_empty or polygon.area <= 0:
        return []

    candidates: list[tuple[float, LineString]] = []
    for edge in _axis_aligned_edges(polygon, options.edge_axis_tolerance):
        if edge.length <= options.max_edge_length:
            continue
        direction = _edge_direction(edge, options.edge_axis_tolerance)
        if direction is None:
            continue

        for const in _candidate_split_constants(edge, polygon, direction, axis_x, axis_y, options):
            split_line = _through_polygon_line(polygon, direction, const)
            beam_line = _best_inside_segment(polygon, split_line, options.min_split_length)
            if beam_line is None or _covered_by_existing(beam_line, existing_union):
                continue
            score = _split_score(polygon, split_line, beam_line, const, direction, axis_x, axis_y, options)
            if score < inf:
                candidates.append((score, beam_line))

    candidates.sort(key=lambda item: item[0])
    beams: list[Beam] = []
    seen: set[tuple[tuple[float, float], tuple[float, float]]] = set()
    for _score, line in candidates:
        key = _line_key(line)
        if key in seen:
            continue
        seen.add(key)
        beams.append(
            Beam(
                axis=line,
                kind=BeamKind.SLAB_DIVIDER,
                reason="large_slab_balanced_split",
                related_ids=(region_index,),
            )
        )
    return beams


def _axis_aligned_edges(polygon: Polygon, tolerance: float) -> list[LineString]:
    coords = list(polygon.exterior.coords)
    edges: list[LineString] = []
    for start, end in zip(coords, coords[1:]):
        line = LineString([start, end])
        if line.length <= 0:
            continue
        if _edge_direction(line, tolerance) is not None:
            edges.append(line)
    return edges


def _edge_direction(edge: LineString, tolerance: float) -> str | None:
    start, end = edge.coords[0], edge.coords[-1]
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    if dy <= tolerance and dx > tolerance:
        return "h"
    if dx <= tolerance and dy > tolerance:
        return "v"
    return None


def _candidate_split_constants(
    long_edge: LineString,
    polygon: Polygon,
    edge_direction: str,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    options: SlabDivisionOptions,
) -> list[float]:
    coords = list(long_edge.coords)
    start = coords[0]
    end = coords[-1]
    if edge_direction == "h":
        a, b = sorted((start[0], end[0]))
        vertex_values = [point[0] for point in polygon.exterior.coords if a < point[0] < b]
        axis_values = [value for value in axis_x if a < value < b]
    else:
        a, b = sorted((start[1], end[1]))
        vertex_values = [point[1] for point in polygon.exterior.coords if a < point[1] < b]
        axis_values = [value for value in axis_y if a < value < b]

    midpoint = (a + b) / 2.0
    values = _dedupe_values(vertex_values + axis_values + [midpoint], options.axis_snap_tolerance * 0.25)
    margin = max(options.min_split_length * 0.25, 300.0)
    return [value for value in values if a + margin < value < b - margin]


def _through_polygon_line(polygon: Polygon, edge_direction: str, const: float) -> LineString:
    minx, miny, maxx, maxy = polygon.bounds
    pad = max(maxx - minx, maxy - miny, 1.0)
    if edge_direction == "h":
        return LineString([(const, miny - pad), (const, maxy + pad)])
    return LineString([(minx - pad, const), (maxx + pad, const)])


def _best_inside_segment(polygon: Polygon, split_line: LineString, min_length: float) -> LineString | None:
    inside = polygon.intersection(split_line)
    segments = [line for line in iter_lines(inside) if line.length >= min_length]
    if not segments:
        return None
    return max(segments, key=lambda line: line.length)


def _split_score(
    polygon: Polygon,
    split_line: LineString,
    beam_line: LineString,
    const: float,
    edge_direction: str,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    options: SlabDivisionOptions,
) -> float:
    pieces = [
        geom for geom in split(polygon, split_line).geoms if isinstance(geom, Polygon) and geom.area > 1.0
    ]
    if len(pieces) < 2:
        return inf

    pieces = sorted(pieces, key=lambda item: item.area, reverse=True)
    area_a, area_b = pieces[0].area, pieces[1].area
    area_ratio = min(area_a, area_b) / max(area_a, area_b)
    if area_ratio < options.min_area_ratio:
        return inf

    axis_values = axis_x if edge_direction == "h" else axis_y
    axis_distance = min((abs(const - value) for value in axis_values), default=options.axis_snap_tolerance)
    axis_bonus = min(axis_distance, options.axis_snap_tolerance) / max(options.axis_snap_tolerance, 1.0)
    balance_penalty = 1.0 - area_ratio
    length_penalty = 1.0 / max(beam_line.length, 1.0)
    return balance_penalty * 10.0 + axis_bonus + length_penalty


def _covered_by_existing(line: LineString, existing_union) -> bool:
    if existing_union.is_empty:
        return False
    return line.difference(existing_union.buffer(1.0, cap_style="square", join_style="mitre")).length <= 1.0


def _reference_axis_values(axis_linework: Sequence[LineString], tolerance: float = 8.0):
    x_values: list[float] = []
    y_values: list[float] = []
    for line in axis_linework:
        for segment in iter_straight_segments(line):
            start, end = segment.coords[0], segment.coords[-1]
            dx = abs(end[0] - start[0])
            dy = abs(end[1] - start[1])
            if dx <= tolerance and dy > tolerance:
                x_values.append((start[0] + end[0]) / 2.0)
            elif dy <= tolerance and dx > tolerance:
                y_values.append((start[1] + end[1]) / 2.0)
    return _dedupe_values(x_values, tolerance), _dedupe_values(y_values, tolerance)


def _dedupe_values(values: Sequence[float], tolerance: float) -> list[float]:
    groups: list[list[float]] = []
    for value in sorted(float(value) for value in values):
        if not groups or abs(value - groups[-1][-1]) > tolerance:
            groups.append([value])
        else:
            groups[-1].append(value)
    return [sum(group) / len(group) for group in groups]


def _line_key(line: LineString, precision: int = 2) -> tuple[tuple[float, float], tuple[float, float]]:
    start = tuple(round(value, precision) for value in line.coords[0])
    end = tuple(round(value, precision) for value in line.coords[-1])
    return tuple(sorted((start, end)))
