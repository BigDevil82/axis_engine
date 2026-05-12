from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

from shapely.geometry import LineString
from shapely.ops import unary_union

from axis_engine.cad_processor import LayoutArtifacts
from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall, StructuralDesignResult
from axis_engine.structural_design.skeleton_spaces import (
    build_buffered_network,
    buffered_network_polygons,
    extract_slab_regions,
)


@dataclass(frozen=True)
class StructuralDesignOptions:
    buffer_distance: float = 150.0
    perimeter_opening_distance_tolerance: float = 30.0
    coupling_const_tolerance: float = 10.0
    coupling_min_gap: float = 300.0
    coupling_max_gap: float = 5000.0
    min_element_length: float = 1.0
    slab_division_options: dict = field(default_factory=dict)


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

    def design(self, artifacts: LayoutArtifacts) -> StructuralDesignResult:
        dominant_thickness = dominant_wall_thickness(artifacts.wall_axes)
        shear_walls = select_shear_walls(artifacts.wall_axes, dominant_thickness)
        buffered_network = build_buffered_network(
            artifacts.wall_axes,
            artifacts.opening_embedments,
            self.options.buffer_distance,
        )
        slab_regions = extract_slab_regions(buffered_network, self.options.buffer_distance)
        beams: list[Beam] = []
        beams.extend(self._perimeter_beams(artifacts, buffered_network))
        beams.extend(self._coupling_beams(shear_walls))
        beams.extend(self._slab_divider_beams(slab_regions, shear_walls, beams))
        return StructuralDesignResult(shear_walls, beams, slab_regions, dominant_thickness)

    def _perimeter_beams(self, artifacts: LayoutArtifacts, buffered_network) -> list[Beam]:
        exterior_boundaries = [polygon.exterior for polygon in buffered_network_polygons(buffered_network)]
        if not exterior_boundaries:
            return []

        beams: list[Beam] = []
        target_distance = self.options.buffer_distance
        tolerance = self.options.perimeter_opening_distance_tolerance
        for index, embedment in enumerate(artifacts.opening_embedments):
            for line in iter_lines(embedment.embed_line):
                if line.length < self.options.min_element_length:
                    continue
                if not _line_near_any_exterior(line, exterior_boundaries, target_distance, tolerance):
                    continue
                beams.append(
                    Beam(
                        axis=line,
                        kind=BeamKind.PERIMETER,
                        reason="opening_on_buffered_exterior",
                        related_ids=(index,),
                    )
                )
        return beams

    def _coupling_beams(self, shear_walls: Sequence[ShearWall]) -> list[Beam]:
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
        slab_regions,
        shear_walls: Sequence[ShearWall],
        beams: Sequence[Beam],
    ) -> list[Beam]:
        # Reserved for the next rule set. The slab regions are already available
        # as recovered polygons, so future rules can split large regions along
        # nearby walls, axes, or existing beam directions.
        return []


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


def _line_near_any_exterior(
    line: LineString,
    exterior_boundaries: Sequence[LineString],
    target_distance: float,
    tolerance: float,
) -> bool:
    return any(abs(line.distance(boundary) - target_distance) <= tolerance for boundary in exterior_boundaries)


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
