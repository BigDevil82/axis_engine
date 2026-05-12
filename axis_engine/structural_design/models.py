from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from shapely.geometry import LineString, Polygon


class BeamKind(str, Enum):
    PERIMETER = "perimeter"
    COUPLING = "coupling"
    SLAB_DIVIDER = "slab_divider"


@dataclass(frozen=True)
class ShearWall:
    axis: LineString
    thickness: float
    source: str = "dominant_wall_thickness"


@dataclass(frozen=True)
class Beam:
    axis: LineString
    kind: BeamKind
    reason: str
    related_ids: tuple[int, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class SlabRegion:
    inner_boundary: LineString
    recovered_polygon: Polygon
    source_polygon_index: int
    hole_index: int


@dataclass(frozen=True)
class StructuralDesignResult:
    shear_walls: list[ShearWall]
    beams: list[Beam]
    slab_regions: list[SlabRegion]
    dominant_wall_thickness: float | None
