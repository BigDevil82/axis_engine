from __future__ import annotations

import numbers
from dataclasses import dataclass
from typing import Sequence

from shapely.geometry import LineString, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall


@dataclass(frozen=True)
class StructuralPruneResult:
    shear_walls: list[ShearWall]
    beams: list[Beam]


@dataclass(frozen=True)
class _GraphEdge:
    line: LineString
    start_key: tuple[float, float]
    end_key: tuple[float, float]


@dataclass(frozen=True)
class _BeamSource:
    line: LineString
    kind: BeamKind
    reason: str
    related_ids: tuple[int, ...]

    @property
    def start(self) -> tuple[float, float]:
        point = self.line.coords[0]
        return (float(point[0]), float(point[1]))

    @property
    def end(self) -> tuple[float, float]:
        point = self.line.coords[-1]
        return (float(point[0]), float(point[1]))

    @property
    def start_key(self) -> tuple[float, float]:
        return _node_key(self.start)

    @property
    def end_key(self) -> tuple[float, float]:
        return _node_key(self.end)


def prune_structural_spurs(
    shear_walls: Sequence[ShearWall],
    beams: Sequence[Beam],
    spur_length: float = 300.0,
    stitch_gap_distance: float = 300.0,
    stitch_probe_width: float = 5.0,
    stitch_min_beam_length: float = 400.0,
    min_segment_length: float = 1.0,
) -> StructuralPruneResult:
    structure = _line_union([wall.axis for wall in shear_walls] + [beam.axis for beam in beams])
    pruned = _prune_degree_one_edges(structure, spur_length, min_segment_length)
    if pruned.is_empty:
        return StructuralPruneResult([], [])

    clean_walls = _restore_shear_walls(shear_walls, pruned, min_segment_length)
    wall_union = _line_union([wall.axis for wall in clean_walls])
    beam_sources = _restore_beam_sources(beams, pruned.difference(wall_union), min_segment_length)
    beam_sources = _stitch_dangling_beams(
        clean_walls,
        beam_sources,
        stitch_gap_distance,
        stitch_probe_width,
        stitch_min_beam_length,
        min_segment_length,
    )
    clean_beams = _restore_beams_by_kind(beam_sources, clean_walls, min_segment_length)
    return StructuralPruneResult(clean_walls, clean_beams)


def _prune_degree_one_edges(skeleton, spur_length: float, min_segment_length: float):
    edges = _graph_edges_from_skeleton(skeleton, min_segment_length)
    if not edges:
        return unary_union([])

    active = [True] * len(edges)
    changed = True
    while changed:
        changed = False
        degrees = _node_degrees(edge for index, edge in enumerate(edges) if active[index])
        for index, edge in enumerate(edges):
            if not active[index]:
                continue
            if edge.line.length >= spur_length:
                continue
            if degrees.get(edge.start_key, 0) == 1 or degrees.get(edge.end_key, 0) == 1:
                active[index] = False
                changed = True

    return _line_union(
        [
            edge.line
            for index, edge in enumerate(edges)
            if active[index] and edge.line.length >= min_segment_length
        ]
    )


def _graph_edges_from_skeleton(skeleton, min_segment_length: float) -> list[_GraphEdge]:
    noded = unary_union(list(iter_straight_segments(skeleton, min_length=min_segment_length)))
    edges: list[_GraphEdge] = []
    for line in iter_straight_segments(noded, min_length=min_segment_length):
        start = line.coords[0]
        end = line.coords[-1]
        edges.append(
            _GraphEdge(
                line=line,
                start_key=_node_key((float(start[0]), float(start[1]))),
                end_key=_node_key((float(end[0]), float(end[1]))),
            )
        )
    return edges


def _node_degrees(edges) -> dict[tuple[float, float], int]:
    degrees: dict[tuple[float, float], int] = {}
    for edge in edges:
        degrees[edge.start_key] = degrees.get(edge.start_key, 0) + 1
        degrees[edge.end_key] = degrees.get(edge.end_key, 0) + 1
    return degrees


def _restore_shear_walls(
    shear_walls: Sequence[ShearWall],
    pruned,
    min_segment_length: float,
) -> list[ShearWall]:
    clean_walls: list[ShearWall] = []
    for wall in shear_walls:
        clean_geometry = pruned.intersection(wall.axis)
        for line in _clean_lines(clean_geometry, min_segment_length):
            clean_walls.append(ShearWall(line, wall.thickness, f"{wall.source}|spur_pruned"))
    return clean_walls


def _restore_beam_sources(
    beams: Sequence[Beam],
    available_space,
    min_segment_length: float,
) -> list[_BeamSource]:
    sources: list[_BeamSource] = []
    for beam in beams:
        clean_geometry = available_space.intersection(beam.axis)
        for line in _clean_lines(clean_geometry, min_segment_length):
            sources.append(_BeamSource(line, beam.kind, f"{beam.reason}|spur_pruned", beam.related_ids))
    return sources


def _stitch_dangling_beams(
    shear_walls: Sequence[ShearWall],
    beam_sources: Sequence[_BeamSource],
    _gap_distance: float,
    probe_width: float,
    min_beam_length: float,
    min_segment_length: float,
) -> list[_BeamSource]:
    beam_edges = [
        _BeamSource(segment, beam.kind, beam.reason, beam.related_ids)
        for beam in beam_sources
        for segment in iter_straight_segments(beam.line, min_length=min_segment_length)
    ]
    if not beam_edges:
        return list(beam_sources)

    structure_lines = [wall.axis for wall in shear_walls]
    structure_lines.extend(beam.line for beam in beam_sources)
    extension_distance = _extension_distance(structure_lines)
    degrees = _node_degrees(
        _GraphEdge(line, _node_key(line.coords[0]), _node_key(line.coords[-1]))
        for line in structure_lines
    )
    tree = STRtree(structure_lines)

    extended: list[_BeamSource] = []
    for edge in beam_edges:
        extended.append(
            _extend_dangling_beam(
                edge,
                degrees,
                tree,
                structure_lines,
                extension_distance,
                probe_width,
                min_beam_length,
            )
        )

    return extended


def _restore_beams_by_kind(
    beam_sources: Sequence[_BeamSource],
    shear_walls: Sequence[ShearWall],
    min_segment_length: float,
) -> list[Beam]:
    remaining = _line_union([source.line for source in beam_sources]).difference(
        _line_union([wall.axis for wall in shear_walls])
    )
    clean_beams: list[Beam] = []
    for kind in (BeamKind.PERIMETER, BeamKind.COUPLING, BeamKind.SLAB_DIVIDER):
        if remaining.is_empty:
            break
        kind_sources = [source for source in beam_sources if source.kind == kind]
        kind_union = _line_union([source.line for source in kind_sources])
        clean_geometry = remaining.intersection(kind_union)
        source_reason = _beam_reason(kind_sources, kind)
        for line in _clean_lines(clean_geometry, min_segment_length):
            clean_beams.append(Beam(line, kind, source_reason))
        if not clean_geometry.is_empty:
            remaining = remaining.difference(clean_geometry)
    return clean_beams


def _beam_reason(sources: Sequence[_BeamSource], kind: BeamKind) -> str:
    if not sources:
        return f"{kind.value}|spur_pruned"
    reason = sources[0].reason
    return reason if reason.endswith("spur_pruned") else f"{reason}|spur_pruned"


def _extension_probe(
    endpoint: tuple[float, float],
    other_point: tuple[float, float],
    distance: float,
) -> LineString | None:
    dx = endpoint[0] - other_point[0]
    dy = endpoint[1] - other_point[1]
    if abs(dx) >= abs(dy):
        if abs(dx) <= 1e-9:
            return None
        direction = 1.0 if dx > 0 else -1.0
        return LineString([endpoint, (endpoint[0] + direction * distance, endpoint[1])])

    if abs(dy) <= 1e-9:
        return None
    direction = 1.0 if dy > 0 else -1.0
    return LineString([endpoint, (endpoint[0], endpoint[1] + direction * distance)])


def _extension_distance(lines: Sequence[LineString]) -> float:
    if not lines:
        return 1.0
    union = _line_union(lines)
    minx, miny, maxx, maxy = union.bounds
    span = max(maxx - minx, maxy - miny, 1.0)
    return span * 2.0


def _extend_dangling_beam(
    edge: _BeamSource,
    degrees: dict[tuple[float, float], int],
    tree: STRtree,
    line_geoms: Sequence[LineString],
    extension_distance: float,
    probe_width: float,
    min_beam_length: float,
) -> _BeamSource:
    if edge.line.length < min_beam_length:
        return edge

    start = edge.start
    end = edge.end
    for side in ("start", "end"):
        if side == "start":
            endpoint, endpoint_key, other_point = start, _node_key(start), end
        else:
            endpoint, endpoint_key, other_point = end, _node_key(end), start
        if degrees.get(endpoint_key, 0) != 1:
            continue

        probe = _extension_probe(endpoint, other_point, extension_distance)
        if probe is None:
            continue
        target = _nearest_extension_target(edge, endpoint, probe, tree, line_geoms, probe_width)
        if target is None:
            continue
        if side == "start":
            start = target
        else:
            end = target

    if start == edge.start and end == edge.end:
        return edge
    return _BeamSource(LineString([start, end]), edge.kind, f"{edge.reason}|extended", edge.related_ids)


def _nearest_extension_target(
    edge: _BeamSource,
    endpoint: tuple[float, float],
    probe: LineString,
    tree: STRtree,
    line_geoms: Sequence[LineString],
    probe_width: float,
) -> tuple[float, float] | None:
    endpoint_point = Point(endpoint)
    best_point = None
    best_distance = float("inf")
    search_geometry = probe.buffer(probe_width, cap_style=2, join_style=2)
    for result in tree.query(search_geometry):
        index = _tree_result_index(result, line_geoms)
        candidate = line_geoms[index]
        if candidate.equals(edge.line):
            continue

        intersection = probe.intersection(candidate)
        for point in _intersection_points(intersection):
            distance = endpoint_point.distance(point)
            if distance <= 1e-6:
                continue
            if distance < best_distance:
                best_distance = distance
                best_point = (float(point.x), float(point.y))

    if best_point is None:
        return None
    return best_point


def _intersection_points(geometry) -> list[Point]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, Point):
        return [geometry]
    if isinstance(geometry, LineString):
        coords = list(geometry.coords)
        if not coords:
            return []
        return [Point(coords[0]), Point(coords[-1])]
    if hasattr(geometry, "geoms"):
        points: list[Point] = []
        for geom in geometry.geoms:
            points.extend(_intersection_points(geom))
        return points
    return []


def _beam_source_key(source: _BeamSource) -> tuple[tuple[float, float], tuple[float, float], BeamKind]:
    start = _node_key(source.start)
    end = _node_key(source.end)
    first, second = (start, end) if start <= end else (end, start)
    return first, second, source.kind


def _clean_lines(geometry, min_segment_length: float) -> list[LineString]:
    return [line for line in iter_lines(geometry) if line.length >= min_segment_length]


def _line_union(lines: Sequence[LineString]):
    non_empty = [line for line in lines if line.length > 0]
    if not non_empty:
        return unary_union([])
    return unary_union(non_empty)


def _node_key(point, precision: int = 2) -> tuple[float, float]:
    return (round(float(point[0]), precision), round(float(point[1]), precision))


def _tree_result_index(result, geoms: Sequence[LineString]) -> int:
    if isinstance(result, numbers.Integral):
        return int(result)
    geom_to_index = {id(geom): index for index, geom in enumerate(geoms)}
    return geom_to_index[id(result)]
