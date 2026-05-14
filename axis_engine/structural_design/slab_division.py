from __future__ import annotations

from dataclasses import dataclass
from math import ceil, inf
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
    room_boundary_clearance: float = 80.0
    min_room_boundary_length: float = 1200.0
    irregular_area_factor: float = 0.9
    rectangular_area_factor: float = 0.95
    min_parallel_spacing: float = 1500.0
    max_axis_divisions_per_region: int = 2


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
    initial_slab_regions: Sequence[SlabRegion],
    shear_walls: Sequence[ShearWall],
    existing_beams: Sequence[Beam],
    axis_linework: Sequence[LineString],
    buffer_distance: float,
    options: SlabDivisionOptions,
) -> list[Beam]:
    axis_x, axis_y = _reference_axis_values(axis_linework)
    existing_lines = [wall.axis for wall in shear_walls if wall.axis.length > 0]
    existing_lines.extend(beam.axis for beam in existing_beams if beam.axis.length > 0)
    existing_union = unary_union(existing_lines)

    divider_beams = _room_boundary_divider_beams(
        slab_regions,
        initial_slab_regions,
        axis_x,
        axis_y,
        existing_lines,
        existing_union,
        options,
    )

    if divider_beams:
        all_beams = list(existing_beams) + divider_beams
        slab_regions = slab_regions_from_structural_lines(shear_walls, all_beams, buffer_distance)
        existing_lines = existing_lines + [beam.axis for beam in divider_beams if beam.axis.length > 0]
        existing_union = unary_union(
            existing_lines
        )

    divider_beams.extend(
        _axis_grid_divider_beams(
            slab_regions,
            axis_x,
            axis_y,
            existing_lines,
            existing_union,
            options,
        )
    )
    return divider_beams


def _room_boundary_divider_beams(
    slab_regions: Sequence[SlabRegion],
    initial_slab_regions: Sequence[SlabRegion],
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    existing_lines: Sequence[LineString],
    existing_union,
    options: SlabDivisionOptions,
) -> list[Beam]:
    divider_beams: list[Beam] = []

    for region_index, region in enumerate(slab_regions):
        polygon = region.recovered_polygon
        if not _needs_division(polygon, options):
            continue
        region_beams = _room_boundary_beams_for_region(
            polygon,
            initial_slab_regions,
            region_index,
            axis_x,
            axis_y,
            existing_lines + [beam.axis for beam in divider_beams],
            existing_union,
            options,
        )
        divider_beams.extend(region_beams)

    return divider_beams


def _axis_grid_divider_beams(
    slab_regions: Sequence[SlabRegion],
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    existing_lines: Sequence[LineString],
    existing_union,
    options: SlabDivisionOptions,
) -> list[Beam]:
    divider_beams: list[Beam] = []
    for region_index, region in enumerate(slab_regions):
        polygon = region.recovered_polygon
        if not _needs_axis_grid_division(polygon, options):
            continue
        region_beams = _divider_beams_for_region(
            polygon,
            region_index,
            axis_x,
            axis_y,
            existing_lines + [beam.axis for beam in divider_beams],
            existing_union,
            options,
        )
        divider_beams.extend(region_beams[: options.max_axis_divisions_per_region])
    return divider_beams


def _room_boundary_beams_for_region(
    slab_polygon: Polygon,
    initial_slab_regions: Sequence[SlabRegion],
    region_index: int,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    existing_lines: Sequence[LineString],
    existing_union,
    options: SlabDivisionOptions,
) -> list[Beam]:
    room_polygons: list[Polygon] = []
    for room in initial_slab_regions:
        if not _room_belongs_to_slab(room.recovered_polygon, slab_polygon):
            continue
        room_polygons.extend(
            polygon
            for polygon in _iter_polygons(room.recovered_polygon.intersection(slab_polygon))
            if polygon.area > 1.0
        )
    if len(room_polygons) < 2:
        return []

    slab_edge = LineString(slab_polygon.exterior.coords).buffer(
        options.room_boundary_clearance,
        cap_style="square",
        join_style="mitre",
    )
    room_edges = unary_union([LineString(room.exterior.coords) for room in room_polygons])
    internal_edges = room_edges.difference(slab_edge)

    beams: list[Beam] = []
    seen: set[tuple[tuple[float, float], tuple[float, float]]] = set()
    for line in iter_straight_segments(internal_edges, min_length=options.min_room_boundary_length):
        if not _on_reference_axis(line, axis_x, axis_y, options):
            continue
        if _too_close_to_parallel(line, existing_lines + [beam.axis for beam in beams], options):
            continue
        if _covered_by_existing(line, existing_union):
            continue
        key = _line_key(line)
        if key in seen:
            continue
        seen.add(key)
        beams.append(
            Beam(
                axis=line,
                kind=BeamKind.SLAB_DIVIDER,
                reason="large_slab_room_boundary_split",
                related_ids=(region_index,),
            )
        )
    return beams


def _room_belongs_to_slab(room_polygon: Polygon, slab_polygon: Polygon) -> bool:
    if room_polygon.is_empty:
        return False
    point = room_polygon.representative_point()
    if slab_polygon.buffer(1.0, cap_style="square", join_style="mitre").covers(point):
        return True
    overlap = room_polygon.intersection(slab_polygon).area
    return overlap / max(room_polygon.area, 1.0) >= 0.50


def _needs_division(polygon: Polygon, options: SlabDivisionOptions) -> bool:
    if polygon.is_empty or polygon.area <= 0:
        return False
    if _max_axis_edge_length(polygon, options.edge_axis_tolerance) > options.max_edge_length:
        return True
    minx, miny, maxx, maxy = polygon.bounds
    return max(maxx - minx, maxy - miny) > options.max_edge_length


def _needs_axis_grid_division(polygon: Polygon, options: SlabDivisionOptions) -> bool:
    if not _needs_division(polygon, options):
        return False
    minx, miny, maxx, maxy = polygon.bounds
    bbox_area = max((maxx - minx) * (maxy - miny), 1.0)
    fill_ratio = polygon.area / bbox_area
    return fill_ratio < options.irregular_area_factor or polygon.area > options.max_edge_length**2


def _max_axis_edge_length(polygon: Polygon, tolerance: float) -> float:
    lengths = [edge.length for edge in _axis_aligned_edges(polygon, tolerance)]
    return max(lengths, default=0.0)


def _is_regular_rectangular(polygon: Polygon, options: SlabDivisionOptions) -> bool:
    if polygon.is_empty or polygon.area <= 0:
        return False
    minx, miny, maxx, maxy = polygon.bounds
    bbox_area = max((maxx - minx) * (maxy - miny), 1.0)
    if polygon.area / bbox_area < options.rectangular_area_factor:
        return False

    coords = list(polygon.exterior.coords)
    return all(
        _edge_direction(LineString([start, end]), options.edge_axis_tolerance) is not None
        for start, end in zip(coords, coords[1:])
        if LineString([start, end]).length > options.edge_axis_tolerance
    )


def _on_reference_axis(
    line: LineString,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    options: SlabDivisionOptions,
) -> bool:
    direction = _edge_direction(line, options.edge_axis_tolerance)
    if direction is None:
        return False
    start, end = line.coords[0], line.coords[-1]
    if direction == "h":
        const = (start[1] + end[1]) / 2.0
        values = axis_y
    else:
        const = (start[0] + end[0]) / 2.0
        values = axis_x
    return any(abs(const - value) <= options.axis_snap_tolerance for value in values)


def _too_close_to_parallel(
    line: LineString,
    reference_lines: Sequence[LineString],
    options: SlabDivisionOptions,
) -> bool:
    feature = _line_feature(line, options.edge_axis_tolerance)
    if feature is None:
        return False

    axis, const, start, end = feature
    for ref_line in reference_lines:
        for ref_segment in iter_straight_segments(ref_line):
            ref_feature = _line_feature(ref_segment, options.edge_axis_tolerance)
            if ref_feature is None or ref_feature[0] != axis:
                continue
            _ref_axis, ref_const, ref_start, ref_end = ref_feature
            if abs(ref_const - const) >= options.min_parallel_spacing:
                continue
            overlap = min(end, ref_end) - max(start, ref_start)
            if overlap >= options.min_split_length * 0.5:
                return True
    return False


def _line_feature(
    line: LineString,
    tolerance: float,
) -> tuple[str, float, float, float] | None:
    direction = _edge_direction(line, tolerance)
    if direction is None:
        return None
    start, end = line.coords[0], line.coords[-1]
    if direction == "h":
        x1, x2 = sorted((start[0], end[0]))
        return "h", (start[1] + end[1]) / 2.0, x1, x2
    y1, y2 = sorted((start[1], end[1]))
    return "v", (start[0] + end[0]) / 2.0, y1, y2


def _divider_beams_for_region(
    polygon: Polygon,
    region_index: int,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    existing_lines: Sequence[LineString],
    existing_union,
    options: SlabDivisionOptions,
) -> list[Beam]:
    if polygon.is_empty or polygon.area <= 0:
        return []

    candidates: list[tuple[float, LineString]] = []
    is_rectangular = _is_regular_rectangular(polygon, options)
    for direction, const in _region_grid_split_constants(polygon, axis_x, axis_y, is_rectangular, options):
        _add_split_candidate(
            candidates,
            polygon,
            direction,
            const,
            axis_x,
            axis_y,
            existing_lines,
            existing_union,
            options,
        )

    for edge in _axis_aligned_edges(polygon, options.edge_axis_tolerance):
        if edge.length <= options.max_edge_length:
            continue
        direction = _edge_direction(edge, options.edge_axis_tolerance)
        if direction is None:
            continue

        for const in _candidate_split_constants(edge, polygon, direction, axis_x, axis_y, is_rectangular, options):
            _add_split_candidate(
                candidates,
                polygon,
                direction,
                const,
                axis_x,
                axis_y,
                existing_lines,
                existing_union,
                options,
            )

    candidates.sort(key=lambda item: item[0])
    beams: list[Beam] = []
    seen: set[tuple[tuple[float, float], tuple[float, float]]] = set()
    for _score, line in candidates:
        key = _line_key(line)
        if key in seen:
            continue
        if _too_close_to_parallel(line, existing_lines + [beam.axis for beam in beams], options):
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


def _add_split_candidate(
    candidates: list[tuple[float, LineString]],
    polygon: Polygon,
    direction: str,
    const: float,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    existing_lines: Sequence[LineString],
    existing_union,
    options: SlabDivisionOptions,
) -> None:
    split_line = _through_polygon_line(polygon, direction, const)
    beam_line = _best_inside_segment(polygon, split_line, options.min_split_length)
    if beam_line is None or _covered_by_existing(beam_line, existing_union):
        return
    if _too_close_to_parallel(beam_line, existing_lines, options):
        return
    score = _split_score(polygon, split_line, beam_line, const, direction, axis_x, axis_y, options)
    if score < inf:
        candidates.append((score, beam_line))


def _region_grid_split_constants(
    polygon: Polygon,
    axis_x: Sequence[float],
    axis_y: Sequence[float],
    allow_midpoint: bool,
    options: SlabDivisionOptions,
) -> list[tuple[str, float]]:
    minx, miny, maxx, maxy = polygon.bounds
    candidates: list[tuple[str, float]] = []
    if maxx - minx > options.max_edge_length:
        candidates.extend(
            ("h", value) for value in _span_split_values(minx, maxx, axis_x, allow_midpoint, options)
        )
    if maxy - miny > options.max_edge_length:
        candidates.extend(
            ("v", value) for value in _span_split_values(miny, maxy, axis_y, allow_midpoint, options)
        )
    return candidates


def _span_split_values(
    start: float,
    end: float,
    axis_values: Sequence[float],
    allow_midpoint: bool,
    options: SlabDivisionOptions,
) -> list[float]:
    margin = max(options.min_split_length * 0.25, 300.0)
    usable_start = start + margin
    usable_end = end - margin
    if usable_start >= usable_end:
        return []

    span = end - start
    split_count = max(1, ceil(span / options.max_edge_length) - 1)
    targets = [start + span * index / (split_count + 1) for index in range(1, split_count + 1)]
    axis_candidates = [value for value in axis_values if usable_start < value < usable_end]
    values = []
    for target in targets:
        nearest_axis = min(axis_candidates, key=lambda value: abs(value - target), default=None)
        if nearest_axis is not None:
            values.append(nearest_axis)
        elif allow_midpoint:
            values.append(target)
    return _dedupe_values(values, options.axis_snap_tolerance * 0.25)


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
    allow_midpoint: bool,
    options: SlabDivisionOptions,
) -> list[float]:
    coords = list(long_edge.coords)
    start = coords[0]
    end = coords[-1]
    if edge_direction == "h":
        a, b = sorted((start[0], end[0]))
        axis_values = [value for value in axis_x if a < value < b]
    else:
        a, b = sorted((start[1], end[1]))
        axis_values = [value for value in axis_y if a < value < b]

    midpoint = (a + b) / 2.0
    values = list(axis_values)
    if allow_midpoint and not values:
        values.append(midpoint)
    values = _dedupe_values(values, options.axis_snap_tolerance * 0.25)
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


def _iter_polygons(geometry):
    if isinstance(geometry, Polygon):
        yield geometry
        return
    if hasattr(geometry, "geoms"):
        for geom in geometry.geoms:
            yield from _iter_polygons(geom)
