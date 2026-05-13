from __future__ import annotations

from dataclasses import dataclass

from shapely.geometry import LineString

from axis_engine.structural_design.models import Beam, ShearWall


@dataclass(frozen=True)
class SkeletonLayerNames:
    wall: str = "AE_WALL_AXIS"
    opening: str = "AE_OPENING_AXIS"


@dataclass(frozen=True)
class DesignLayerNames:
    shear_wall: str = "AE_SHEAR_WALL"
    beam: str = "AE_BEAM"


@dataclass(frozen=True)
class SkeletonDxfData:
    wall_axes: list[tuple[LineString, float]]
    opening_lines: list[LineString]


@dataclass(frozen=True)
class DesignDxfData:
    shear_walls: list[ShearWall]
    beam_lines: list[LineString]
    beams: list[Beam]
