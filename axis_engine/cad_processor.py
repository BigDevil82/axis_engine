from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from shapely.geometry import LineString, MultiLineString, MultiPolygon, Polygon
from shapely.ops import polygonize

from axis_engine.geometry_utils import iter_lines, iter_straight_segments
from axis_engine.linework_axis_extractor import (
    extract_wall_axes_from_linework,
    infer_wall_thicknesses,
    repair_wall_linework,
)
from axis_engine.opening_clustering import OpeningCluster, cluster_opening_segments
from axis_engine.opening_embedment import (
    OPENING_BALCONY,
    OPENING_DOOR,
    OpeningEmbedment,
    infer_opening_embedments,
)
from axis_engine.raw_wall_polygon_builder import build_wall_polygon_from_raw_lines
from cad_tests.dxf_utils import (
    pick_dxf_wall_layers,
    read_dxf,
    read_dxf_line_segments_from_layers,
    read_dxf_segments_from_layers,
)

from .line_network_calibrator import LineNetworkCalibrator, NetworkSegment, SegmentType
from .rect_decomposer import RectangularDecomposer


@dataclass
class LayoutArtifacts:
    wall_lines: list[LineString]
    wall_centerlines: list[tuple[LineString, float]]
    wall_polygon: MultiPolygon
    opening_lines: list[LineString]
    opening_clusters: list[OpeningCluster]
    opening_embedments: list[OpeningEmbedment]
    components: dict[str, MultiLineString]


class CADLayoutProcessor:
    def __init__(
        self,
        source_path: str | Path,
        wall_layers: Sequence[str] | None = None,
        opening_layers: Sequence[str] | None = None,
        wall_thicknesses: Sequence[float] | None = None,
        visible_only: bool = False,
        wall_axis_options: dict | None = None,
        opening_cluster_options: dict | None = None,
        embedment_options: dict | None = None,
    ):
        self.source_path = Path(source_path)
        if self.source_path.suffix.lower() != ".dxf":
            raise ValueError("CADLayoutProcessor now only supports DXF input.")

        self.wall_layers = list(wall_layers) if wall_layers else None
        self.opening_layers = list(opening_layers or ["WINDOW"])
        self.wall_thicknesses = list(wall_thicknesses) if wall_thicknesses else None
        self.visible_only = visible_only
        self.wall_axis_options = dict(wall_axis_options or {})
        self.opening_cluster_options = dict(opening_cluster_options or {})
        self.embedment_options = dict(embedment_options or {})

        self.wall_polygon = MultiPolygon()
        self.wall_centerlines: list[tuple[LineString, float]] = []
        self.components: dict[str, MultiLineString] = {}
        self.opening_clusters: list[OpeningCluster] = []
        self.opening_embedments: list[OpeningEmbedment] = []
        self.wall_lines: list[LineString] = []
        self.opening_lines: list[LineString] = []
        self.all_segments: list[NetworkSegment] = []
        self.unified_network = MultiLineString()
        self.rooms: list[Polygon] = []
        self.room_groups: list[list[Polygon]] = []

    def build_geometry(self) -> LayoutArtifacts:
        doc = read_dxf(self.source_path)

        wall_layers = pick_dxf_wall_layers(doc, self.wall_layers)
        self.wall_layers = wall_layers
        self.wall_lines = read_dxf_line_segments_from_layers(
            doc,
            wall_layers,
            visible_only=self.visible_only,
        )

        repair_options = {
            key: self.wall_axis_options[key]
            for key in ("axis_tolerance", "snap_tolerance")
            if key in self.wall_axis_options
        }
        if "min_feature_len" in self.wall_axis_options:
            repair_options["min_segment_length"] = self.wall_axis_options["min_feature_len"]
        repaired_edges = repair_wall_linework(self.wall_lines, **repair_options)

        thicknesses = self.wall_thicknesses or infer_wall_thicknesses(repaired_edges)
        self.wall_thicknesses = thicknesses

        self.wall_centerlines = extract_wall_axes_from_linework(
            self.wall_lines,
            thickness_candidates=thicknesses,
            **self.wall_axis_options,
        )
        self.wall_polygon = build_wall_polygon_from_raw_lines(
            self.wall_lines,
            wall_thicknesses=thicknesses,
        )

        self.components = {
            "doors": MultiLineString(),
            "windows": MultiLineString(),
            "balconies": MultiLineString(),
        }

        if self.opening_layers:
            opening_segments = read_dxf_segments_from_layers(
                doc,
                self.opening_layers,
                visible_only=self.visible_only,
            )
            self.opening_lines = [segment.line for segment in opening_segments]
            self.opening_clusters = cluster_opening_segments(
                opening_segments,
                **self.opening_cluster_options,
            )
            self.opening_embedments = infer_opening_embedments(
                self.wall_centerlines,
                self.opening_clusters,
                **self.embedment_options,
            )
            self._load_opening_embedments_as_components()
        else:
            self.opening_lines = []
            self.opening_clusters = []
            self.opening_embedments = []

        return LayoutArtifacts(
            wall_lines=self.wall_lines,
            wall_centerlines=self.wall_centerlines,
            wall_polygon=self.wall_polygon,
            opening_lines=self.opening_lines,
            opening_clusters=self.opening_clusters,
            opening_embedments=self.opening_embedments,
            components=self.components,
        )

    def extract_centerlines(self) -> list[tuple[LineString, float]]:
        if not self.wall_centerlines:
            raise RuntimeError("No wall centerlines found. Run build_geometry() first.")
        return self.wall_centerlines

    def collect_network_segments(self) -> list[NetworkSegment]:
        segments: list[NetworkSegment] = []
        for line, thickness in self.wall_centerlines:
            segments.extend(
                NetworkSegment(
                    geometry=segment,
                    thickness=thickness,
                    seg_type=SegmentType.WALL,
                    is_structural=thickness >= 180.0,
                )
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
        return segments

    def generate_rooms(self) -> list[Polygon]:
        self.all_segments = self.collect_network_segments()
        calibrator = LineNetworkCalibrator(structural_thickness_threshold=300.0)
        self.unified_network = calibrator.calibrate(self.all_segments)

        polys = list(polygonize(self.unified_network))
        decomposer = RectangularDecomposer()
        valid_rooms: list[Polygon] = []
        self.room_groups = []
        for polygon in polys:
            sub_rects = decomposer.decompose(polygon)
            valid_rooms.extend(sub_rects)
            self.room_groups.append(sub_rects)
        self.rooms = valid_rooms
        return self.rooms

    def _load_opening_embedments_as_components(self):
        by_component: dict[str, list[LineString]] = {
            "doors": [],
            "windows": [],
            "balconies": [],
        }
        for embedment in self.opening_embedments:
            lines = list(iter_lines(embedment.embed_line))
            if embedment.opening_type == OPENING_DOOR:
                by_component["doors"].extend(lines)
            elif embedment.opening_type == OPENING_BALCONY:
                by_component["balconies"].extend(lines)
            else:
                by_component["windows"].extend(lines)

        for component_name, lines in by_component.items():
            self.components[component_name] = MultiLineString(lines) if lines else MultiLineString()

