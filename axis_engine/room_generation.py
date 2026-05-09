from __future__ import annotations

from dataclasses import dataclass

from shapely.geometry import MultiLineString, Polygon
from shapely.ops import polygonize

from axis_engine.line_network_calibrator import LineNetworkCalibrator, NetworkSegment
from axis_engine.rect_decomposer import RectangularDecomposer


@dataclass
class RoomGenerationResult:
    unified_network: MultiLineString
    rooms: list[Polygon]
    room_groups: list[list[Polygon]]


def generate_rooms_from_segments(
    segments: list[NetworkSegment],
    structural_thickness_threshold: float = 300.0,
) -> RoomGenerationResult:
    calibrator = LineNetworkCalibrator(structural_thickness_threshold=structural_thickness_threshold)
    unified_network = calibrator.calibrate(segments)

    polys = list(polygonize(unified_network))
    decomposer = RectangularDecomposer()
    valid_rooms: list[Polygon] = []
    room_groups: list[list[Polygon]] = []
    for polygon in polys:
        sub_rects = decomposer.decompose(polygon)
        valid_rooms.extend(sub_rects)
        room_groups.append(sub_rects)

    return RoomGenerationResult(
        unified_network=unified_network,
        rooms=valid_rooms,
        room_groups=room_groups,
    )

