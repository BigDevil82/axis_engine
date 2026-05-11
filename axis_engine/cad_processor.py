from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from shapely.geometry import LineString, MultiLineString, MultiPolygon, Polygon

from axis_engine.dxf_utils import (
    DxfGeometry,
    geometries_to_linework,
    pick_dxf_axis_layers,
    pick_dxf_wall_layers,
    read_dxf,
    read_dxf_geometries_from_layers,
)
from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.line_network_calibrator import NetworkSegment, SegmentType
from axis_engine.linework_axis_extractor import (
    extract_wall_axes_from_linework,
    infer_wall_thicknesses,
    repair_wall_linework,
)
from axis_engine.opening_clustering import OpeningCluster, cluster_opening_geometries
from axis_engine.opening_embedment import (
    OPENING_BALCONY,
    OPENING_DOOR,
    OpeningEmbedment,
    infer_opening_embedments,
)
from axis_engine.raw_wall_polygon_builder import build_wall_polygon_from_raw_lines
from axis_engine.room_generation import generate_rooms_from_segments
from axis_engine.skeleton_topology_calibrator import calibrate_skeleton_topology


@dataclass(frozen=True)
class LayoutArtifacts:
    axis_geometries: list[DxfGeometry]
    axis_linework: list[LineString]
    wall_geometries: list[DxfGeometry]
    wall_linework: list[LineString]
    wall_axes: list[tuple[LineString, float]]
    wall_polygon: MultiPolygon
    opening_geometries: list[DxfGeometry]
    opening_clusters: list[OpeningCluster]
    opening_embedments: list[OpeningEmbedment]
    components: dict[str, MultiLineString]


class CADLayoutProcessor:
    """Explicit DXF-to-skeleton workflow.

    The processor keeps DXF parsing, wall-axis extraction, opening inference and
    room generation as separate stages with stable data flowing between them.
    """

    def __init__(
        self,
        source_path: str | Path,
        wall_layers: Sequence[str] | None = None,
        axis_layers: Sequence[str] | None = None,
        opening_layers: Sequence[str] | None = None,
        wall_thicknesses: Sequence[float] | None = None,
        visible_only: bool = False,
        wall_axis_options: dict | None = None,
        opening_cluster_options: dict | None = None,
        embedment_options: dict | None = None,
        topology_calibration_options: dict | None = None,
        reference_axis_snap_tolerance: float = 100.0,
    ):
        self.source_path = Path(source_path)
        if self.source_path.suffix.lower() != ".dxf":
            raise ValueError("CADLayoutProcessor only supports DXF input.")

        self.wall_layers = list(set(wall_layers)) if wall_layers else None
        self.axis_layers = list(set(axis_layers)) if axis_layers else None
        self.opening_layers = list(set(opening_layers or ["WINDOW"]))
        self.wall_thicknesses = list(wall_thicknesses) if wall_thicknesses else None
        self.visible_only = visible_only
        self.wall_axis_options = dict(wall_axis_options or {})
        self.opening_cluster_options = dict(opening_cluster_options or {})
        self.embedment_options = dict(embedment_options or {})
        self.topology_calibration_options = dict(topology_calibration_options or {})
        self.reference_axis_snap_tolerance = float(reference_axis_snap_tolerance)

        self.axis_geometries: list[DxfGeometry] = []
        self.axis_linework: list[LineString] = []
        self.reference_axes = ReferenceAxisGrid()
        self.wall_geometries: list[DxfGeometry] = []
        self.wall_linework: list[LineString] = []
        self.wall_axes: list[tuple[LineString, float]] = []
        self.wall_polygon = MultiPolygon()
        self.opening_geometries: list[DxfGeometry] = []
        self.opening_clusters: list[OpeningCluster] = []
        self.opening_embedments: list[OpeningEmbedment] = []
        self.components: dict[str, MultiLineString] = {}
        self.all_segments: list[NetworkSegment] = []
        self.unified_network = MultiLineString()
        self.rooms: list[Polygon] = []
        self.room_groups: list[list[Polygon]] = []
        self._geometry_built = False

    def build_geometry(self) -> LayoutArtifacts:
        doc = read_dxf(self.source_path)
        self.components = _empty_components()
        self._load_reference_axes(doc)
        self._load_wall_geometries(doc)
        self._extract_wall_axes()
        self._align_wall_axes_to_reference_axes()
        self._build_wall_polygon()
        self._load_openings(doc)
        self._align_openings_to_reference_axes()
        self._calibrate_skeleton_topology()
        self._geometry_built = True

        return LayoutArtifacts(
            axis_geometries=self.axis_geometries,
            axis_linework=self.axis_linework,
            wall_geometries=self.wall_geometries,
            wall_linework=self.wall_linework,
            wall_axes=self.wall_axes,
            wall_polygon=self.wall_polygon,
            opening_geometries=self.opening_geometries,
            opening_clusters=self.opening_clusters,
            opening_embedments=self.opening_embedments,
            components=self.components,
        )

    def extract_centerlines(self) -> list[tuple[LineString, float]]:
        self._require_geometry()
        return self.wall_axes

    def collect_network_segments(self) -> list[NetworkSegment]:
        self._require_geometry()
        segments: list[NetworkSegment] = []
        for line, thickness in self.wall_axes:
            segments.extend(
                NetworkSegment(segment, thickness, SegmentType.WALL, thickness >= 180.0)
                for segment in iter_straight_segments(line)
            )

        for line in self.components.get("doors", MultiLineString()).geoms:
            segments.extend(
                NetworkSegment(segment, 100.0, SegmentType.DOOR, False)
                for segment in iter_straight_segments(line)
            )
        for line in self.components.get("windows", MultiLineString()).geoms:
            segments.extend(
                NetworkSegment(segment, 100.0, SegmentType.WINDOW, False)
                for segment in iter_straight_segments(line)
            )
        for line in self.components.get("balconies", MultiLineString()).geoms:
            segments.extend(
                NetworkSegment(segment, 100.0, SegmentType.WINDOW, False)
                for segment in iter_straight_segments(line)
            )

        if not segments:
            raise RuntimeError("No semantic network segments available for calibration.")
        return segments

    def generate_rooms(self) -> list[Polygon]:
        self._require_geometry()
        self.all_segments = self.collect_network_segments()
        result = generate_rooms_from_segments(self.all_segments, structural_thickness_threshold=300.0)
        self.unified_network = result.unified_network
        self.room_groups = result.room_groups
        self.rooms = result.rooms
        if not self.rooms:
            raise RuntimeError("Room generation produced no valid rooms.")
        return self.rooms

    def _load_reference_axes(self, doc):
        self.axis_layers = pick_dxf_axis_layers(doc, self.axis_layers)
        if not self.axis_layers:
            self.reference_axes = ReferenceAxisGrid()
            return

        self.axis_geometries = read_dxf_geometries_from_layers(
            doc,
            self.axis_layers,
            visible_only=self.visible_only,
        )
        self.axis_linework = geometries_to_linework(self.axis_geometries)
        self.reference_axes = ReferenceAxisGrid.from_lines(self.axis_linework)

    def _load_wall_geometries(self, doc):
        self.wall_layers = pick_dxf_wall_layers(doc, self.wall_layers)
        self.wall_geometries = read_dxf_geometries_from_layers(
            doc,
            self.wall_layers,
            visible_only=self.visible_only,
        )
        self.wall_linework = geometries_to_linework(self.wall_geometries)
        if not self.wall_linework:
            raise RuntimeError("No wall geometry was loaded from DXF wall layers.")

    def _extract_wall_axes(self):
        repair_options = {
            key: self.wall_axis_options[key]
            for key in ("axis_tolerance", "snap_tolerance")
            if key in self.wall_axis_options
        }
        if "min_feature_len" in self.wall_axis_options:
            repair_options["min_segment_length"] = self.wall_axis_options["min_feature_len"]

        repaired_edges = repair_wall_linework(self.wall_linework, **repair_options)
        thicknesses = self.wall_thicknesses or infer_wall_thicknesses(repaired_edges)
        self.wall_thicknesses = thicknesses
        self.wall_axes = extract_wall_axes_from_linework(
            self.wall_linework,
            thickness_candidates=thicknesses,
            **self.wall_axis_options,
        )
        if not self.wall_axes:
            raise RuntimeError("Wall axis extraction produced no centerlines.")

    def _align_wall_axes_to_reference_axes(self):
        if self.reference_axes.is_empty:
            return
        aligned_axes: list[tuple[LineString, float]] = []
        for line, thickness in self.wall_axes:
            for segment in iter_straight_segments(line):
                aligned = self.reference_axes.align_line(segment, self.reference_axis_snap_tolerance)
                if aligned.length > 0:
                    aligned_axes.append((aligned, thickness))
        self.wall_axes = aligned_axes

    def _build_wall_polygon(self):
        self.wall_polygon = build_wall_polygon_from_raw_lines(
            self.wall_linework,
            wall_thicknesses=self.wall_thicknesses,
        )

    def _load_openings(self, doc):
        if not self.opening_layers:
            return

        self.opening_geometries = read_dxf_geometries_from_layers(
            doc,
            self.opening_layers,
            visible_only=self.visible_only,
        )
        self.opening_clusters = cluster_opening_geometries(
            self.opening_geometries,
            **self.opening_cluster_options,
        )
        self.opening_embedments = infer_opening_embedments(
            self.wall_axes,
            self.opening_clusters,
            **self.embedment_options,
        )
        self.components = _components_from_embedments(self.opening_embedments)

    def _align_openings_to_reference_axes(self):
        if self.reference_axes.is_empty or not self.opening_embedments:
            return

        self.opening_embedments = [
            OpeningEmbedment(
                opening_type=embedment.opening_type,
                embed_line=self.reference_axes.align_line(
                    embedment.embed_line,
                    self.reference_axis_snap_tolerance,
                ),
                cluster_index=embedment.cluster_index,
                confidence=embedment.confidence,
                reason=f"{embedment.reason}|axis_aligned",
            )
            for embedment in self.opening_embedments
        ]
        self.components = _components_from_embedments(self.opening_embedments)

    def _calibrate_skeleton_topology(self):
        if not self.wall_axes and not self.opening_embedments:
            return

        result = calibrate_skeleton_topology(
            self.wall_axes,
            self.opening_embedments,
            **self.topology_calibration_options,
        )
        self.wall_axes = result.wall_axes
        self.opening_embedments = result.opening_embedments
        self.components = _components_from_embedments(self.opening_embedments)

    def _require_geometry(self):
        if not self._geometry_built:
            raise RuntimeError("Geometry is not built. Run build_geometry() first.")


class ReferenceAxisGrid:
    def __init__(self, x_axes: Sequence[float] = (), y_axes: Sequence[float] = ()):
        self.x_axes = sorted(float(value) for value in x_axes)
        self.y_axes = sorted(float(value) for value in y_axes)

    @property
    def is_empty(self) -> bool:
        return not self.x_axes and not self.y_axes

    @classmethod
    def from_lines(cls, lines: Sequence[LineString], axis_tolerance: float = 8.0) -> ReferenceAxisGrid:
        x_axes: list[float] = []
        y_axes: list[float] = []
        for line in lines:
            coords = list(line.coords)
            if len(coords) < 2:
                continue
            start = coords[0]
            end = coords[-1]
            dx = abs(end[0] - start[0])
            dy = abs(end[1] - start[1])
            if dx <= axis_tolerance and dy > axis_tolerance:
                x_axes.append((start[0] + end[0]) / 2.0)
            elif dy <= axis_tolerance and dx > axis_tolerance:
                y_axes.append((start[1] + end[1]) / 2.0)

        return cls(_dedupe_sorted_values(x_axes), _dedupe_sorted_values(y_axes))

    def align_line(self, line: LineString, snap_tolerance: float) -> LineString:
        coords = list(line.coords)
        if len(coords) < 2:
            return line

        start = coords[0]
        end = coords[-1]
        dx = abs(end[0] - start[0])
        dy = abs(end[1] - start[1])
        if dx >= dy:
            y = self._nearest_within((start[1] + end[1]) / 2.0, self.y_axes, snap_tolerance)
            if y is None:
                y = (start[1] + end[1]) / 2.0
            x1 = start[0]
            x2 = end[0]
            return LineString([(x1, y), (x2, y)])

        x = self._nearest_within((start[0] + end[0]) / 2.0, self.x_axes, snap_tolerance)
        if x is None:
            x = (start[0] + end[0]) / 2.0
        y1 = start[1]
        y2 = end[1]
        return LineString([(x, y1), (x, y2)])

    def _nearest_within(
        self,
        value: float,
        candidates: Sequence[float],
        snap_tolerance: float,
    ) -> float | None:
        if not candidates:
            return None
        nearest = min(candidates, key=lambda candidate: abs(candidate - value))
        if abs(nearest - value) > snap_tolerance:
            return None
        return nearest


def _dedupe_sorted_values(values: Sequence[float], tolerance: float = 5.0) -> list[float]:
    if not values:
        return []

    groups: list[list[float]] = []
    for value in sorted(values):
        if not groups or abs(value - groups[-1][-1]) > tolerance:
            groups.append([value])
        else:
            groups[-1].append(value)
    return [sum(group) / len(group) for group in groups]


def _components_from_embedments(embedments: Sequence[OpeningEmbedment]) -> dict[str, MultiLineString]:
    by_component: dict[str, list[LineString]] = {"doors": [], "windows": [], "balconies": []}
    for embedment in embedments:
        lines = list(iter_lines(embedment.embed_line))
        if embedment.opening_type == OPENING_DOOR:
            by_component["doors"].extend(lines)
        elif embedment.opening_type == OPENING_BALCONY:
            by_component["balconies"].extend(lines)
        else:
            by_component["windows"].extend(lines)

    return {
        name: MultiLineString(lines) if lines else MultiLineString() for name, lines in by_component.items()
    }


def _empty_components() -> dict[str, MultiLineString]:
    return {"doors": MultiLineString(), "windows": MultiLineString(), "balconies": MultiLineString()}
