from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from axis_engine.cad_processor import LayoutArtifacts
from axis_engine.geometry_utils import iter_straight_segments
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall, StructuralDesignResult
from axis_engine.structural_design.slab_division import (
    SlabDivisionOptions,
    infer_slab_divider_beams,
    slab_regions_from_structural_lines,
)
from axis_engine.structural_design.skeleton_spaces import (
    build_buffered_network,
    buffered_network_polygons,
    extract_slab_regions,
)


@dataclass(frozen=True)
class StructuralDesignOptions:
    buffer_distance: float = 150.0
    coupling_const_tolerance: float = 10.0
    coupling_min_gap: float = 300.0
    coupling_max_gap: float = 5000.0
    coupling_near_parallel_wall_distance: float = 600.0
    coupling_near_parallel_wall_overlap_ratio: float = 0.30
    min_element_length: float = 1.0
    slab_division_options: SlabDivisionOptions = field(default_factory=SlabDivisionOptions)


@dataclass(frozen=True)
class AxisFeature:
    line: LineString
    axis: str
    const: float
    start: float
    end: float
    source_index: int

    @property
    def length(self) -> float:
        return self.end - self.start


class StructuralDesigner:
    def __init__(self, options: StructuralDesignOptions | None = None):
        self.options = options or StructuralDesignOptions()
        if isinstance(self.options.slab_division_options, dict):
            self.options = StructuralDesignOptions(
                buffer_distance=self.options.buffer_distance,
                coupling_const_tolerance=self.options.coupling_const_tolerance,
                coupling_min_gap=self.options.coupling_min_gap,
                coupling_max_gap=self.options.coupling_max_gap,
                coupling_near_parallel_wall_distance=self.options.coupling_near_parallel_wall_distance,
                coupling_near_parallel_wall_overlap_ratio=self.options.coupling_near_parallel_wall_overlap_ratio,
                min_element_length=self.options.min_element_length,
                slab_division_options=SlabDivisionOptions(**self.options.slab_division_options),
            )

    def design(self, artifacts: LayoutArtifacts) -> StructuralDesignResult:
        dominant_thickness = dominant_wall_thickness(artifacts.wall_axes)
        shear_walls = select_shear_walls(artifacts.wall_axes, dominant_thickness)
        buffered_network = build_buffered_network(
            artifacts.wall_axes,
            artifacts.opening_embedments,
            self.options.buffer_distance,
        )
        slab_regions = extract_slab_regions(buffered_network, self.options.buffer_distance)
        exterior_shell = _exterior_shell_union(buffered_network)
        beams: list[Beam] = []
        beams.extend(self._perimeter_beams_from_slab_footprint(slab_regions, buffered_network, shear_walls))
        beams.extend(self._coupling_beams(shear_walls, exterior_shell))
        beams = _dedupe_beams(beams)
        slab_regions = slab_regions_from_structural_lines(shear_walls, beams, self.options.buffer_distance)
        beams.extend(self._slab_divider_beams(artifacts, slab_regions, shear_walls, beams))
        beams = _dedupe_beams(beams)
        slab_regions = slab_regions_from_structural_lines(shear_walls, beams, self.options.buffer_distance)
        return StructuralDesignResult(shear_walls, beams, slab_regions, dominant_thickness)

    def _perimeter_beams_from_slab_footprint(
        self,
        slab_regions,
        buffered_network,
        shear_walls: Sequence[ShearWall],
    ) -> list[Beam]:
        """Use the merged slab footprint as perimeter beams.

        Slab regions are recovered from the stable buffered-network holes, so
        their union tends to remove small corner jitter from the raw skeleton.
        The merged footprint therefore gives a cleaner closed outer contour than
        tracing the buffered skeleton shell directly.
        """
        outer_contour = _slab_footprint_outer_contour(slab_regions)
        if outer_contour.is_empty:
            outer_contour = _outer_contour_centerline(buffered_network, self.options.buffer_distance)
        if outer_contour.is_empty:
            return []

        shear_wall_union = unary_union([wall.axis for wall in shear_walls if wall.axis.length > 0])
        beam_geometry = outer_contour.difference(shear_wall_union)
        beams: list[Beam] = []
        seen: set[tuple[tuple[float, float], tuple[float, float]]] = set()
        for line in iter_straight_segments(beam_geometry, self.options.min_element_length):
            key = _line_key(line)
            if key in seen:
                continue
            seen.add(key)
            beams.append(
                Beam(
                    axis=line,
                    kind=BeamKind.PERIMETER,
                    reason="slab_footprint_outer_contour_minus_shear_wall",
                )
            )
        return beams

    def _coupling_beams(self, shear_walls: Sequence[ShearWall], exterior_shell) -> list[Beam]:
        features = [
            feature
            for index, wall in enumerate(shear_walls)
            for segment in iter_straight_segments(wall.axis, min_length=self.options.min_element_length)
            for feature in [_axis_feature(segment, index)]
            if feature is not None
        ]

        beams: list[Beam] = []
        for axis in ("h", "v"):
            axis_features = sorted(
                [feature for feature in features if feature.axis == axis],
                key=lambda item: (item.const, item.start, item.end),
            )
            groups = _group_collinear_features(axis_features, self.options.coupling_const_tolerance)
            for group in groups:
                group = sorted(group, key=lambda item: item.start)
                for first, second in zip(group, group[1:]):
                    gap = second.start - first.end
                    if not self.options.coupling_min_gap <= gap <= self.options.coupling_max_gap:
                        continue
                    if axis == "h":
                        line = LineString([(first.end, first.const), (second.start, first.const)])
                    else:
                        line = LineString([(first.const, first.end), (first.const, second.start)])
                    if exterior_shell is not None and not exterior_shell.covers(line):
                        continue
                    if _has_near_parallel_shear_wall(
                        line,
                        axis,
                        first.source_index,
                        second.source_index,
                        features,
                        self.options.coupling_near_parallel_wall_distance,
                        self.options.coupling_near_parallel_wall_overlap_ratio,
                    ):
                        continue
                    beams.append(
                        Beam(
                            axis=line,
                            kind=BeamKind.COUPLING,
                            reason="gap_between_collinear_shear_walls",
                            related_ids=(first.source_index, second.source_index),
                        )
                    )
        return beams

    def _slab_divider_beams(
        self,
        artifacts: LayoutArtifacts,
        slab_regions,
        shear_walls: Sequence[ShearWall],
        beams: Sequence[Beam],
    ) -> list[Beam]:
        return infer_slab_divider_beams(
            slab_regions,
            beams,
            artifacts.axis_linework,
            self.options.slab_division_options,
        )


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
) -> list[ShearWall]:
    if dominant_thickness is None:
        return []
    return [
        ShearWall(axis=line, thickness=thickness)
        for line, thickness in wall_axes
        if thickness == dominant_thickness and line.length > 0
    ]


def _exterior_shell_union(buffered_network):
    shells = [Polygon(polygon.exterior) for polygon in buffered_network_polygons(buffered_network)]
    if not shells:
        return None
    return unary_union(shells)


def _outer_contour_centerline(buffered_network, buffer_distance: float):
    contours: list[LineString] = []
    for polygon in buffered_network_polygons(buffered_network):
        shell = Polygon(polygon.exterior)
        recovered = shell.buffer(
            -buffer_distance,
            cap_style="square",
            join_style="mitre",
        )
        for recovered_polygon in _iter_polygons(recovered):
            if recovered_polygon.is_empty:
                continue
            contours.append(LineString(recovered_polygon.exterior.coords))
    if not contours:
        return LineString()
    return unary_union(contours)


def _slab_footprint_outer_contour(slab_regions):
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


def _line_key(line: LineString, precision: int = 2) -> tuple[tuple[float, float], tuple[float, float]]:
    start = tuple(round(value, precision) for value in line.coords[0])
    end = tuple(round(value, precision) for value in line.coords[-1])
    return tuple(sorted((start, end)))


def _dedupe_beams(beams: Sequence[Beam]) -> list[Beam]:
    priority = {
        BeamKind.PERIMETER: 0,
        BeamKind.COUPLING: 1,
        BeamKind.SLAB_DIVIDER: 2,
    }
    selected: dict[tuple[tuple[float, float], tuple[float, float]], Beam] = {}
    for beam in beams:
        key = _line_key(beam.axis)
        current = selected.get(key)
        if current is None or priority[beam.kind] < priority[current.kind]:
            selected[key] = beam
    return sorted(
        selected.values(),
        key=lambda beam: (priority[beam.kind], _line_key(beam.axis)),
    )


def _axis_feature(line: LineString, source_index: int, axis_tolerance: float = 8.0) -> AxisFeature | None:
    coords = list(line.coords)
    if len(coords) < 2:
        return None
    start = coords[0]
    end = coords[-1]
    dx = abs(end[0] - start[0])
    dy = abs(end[1] - start[1])
    if dy <= axis_tolerance and dx >= dy:
        x1, x2 = sorted((start[0], end[0]))
        return AxisFeature(line, "h", (start[1] + end[1]) / 2.0, x1, x2, source_index)
    if dx <= axis_tolerance and dy >= dx:
        y1, y2 = sorted((start[1], end[1]))
        return AxisFeature(line, "v", (start[0] + end[0]) / 2.0, y1, y2, source_index)
    return None


def _group_collinear_features(
    features: Sequence[AxisFeature],
    const_tolerance: float,
) -> list[list[AxisFeature]]:
    groups: list[list[AxisFeature]] = []
    for feature in features:
        target = None
        for group in groups:
            group_const = sum(item.const * item.length for item in group) / max(
                sum(item.length for item in group),
                1.0,
            )
            if abs(feature.const - group_const) <= const_tolerance:
                target = group
                break
        if target is None:
            groups.append([feature])
        else:
            target.append(feature)
    return groups


def _has_near_parallel_shear_wall(
    beam_line: LineString,
    axis: str,
    first_wall_index: int,
    second_wall_index: int,
    wall_features: Sequence[AxisFeature],
    distance_tolerance: float,
    overlap_ratio_threshold: float,
) -> bool:
    beam_feature = _axis_feature(beam_line, -1)
    if beam_feature is None:
        return False

    for feature in wall_features:
        if feature.axis != axis:
            continue
        if feature.source_index in {first_wall_index, second_wall_index}:
            continue
        if abs(feature.const - beam_feature.const) > distance_tolerance:
            continue
        overlap = min(feature.end, beam_feature.end) - max(feature.start, beam_feature.start)
        if overlap <= 0:
            continue
        overlap_ratio = overlap / max(beam_feature.length, 1.0)
        if overlap_ratio >= overlap_ratio_threshold:
            return True
    return False
