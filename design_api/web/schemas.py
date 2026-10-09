"""HTTP request and response schemas for the design API."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class GeometryPayload(BaseModel):
    id: str | None = None
    layer: str = ""
    geom_type: str
    params: dict[str, Any]
    block_path: list[str] = Field(default_factory=list)


class AxisLinePayload(BaseModel):
    id: str | None = None
    line: list[list[float]]


class WallAxisPayload(AxisLinePayload):
    thickness: float = 200.0


class OpeningPayload(AxisLinePayload):
    opening_type: str = "opening"
    cluster_index: int = 0
    confidence: float = 1.0
    reason: str = "manual"


class ShearWallPayload(AxisLinePayload):
    thickness: float = 200.0
    source: str = "manual"


class BeamPayload(AxisLinePayload):
    kind: str = "perimeter"
    reason: str = "manual"
    related_ids: list[int] = Field(default_factory=list)


class PolygonPayload(BaseModel):
    exterior: list[list[float]]
    interiors: list[list[list[float]]] = Field(default_factory=list)


class SlabRegionPayload(BaseModel):
    id: str
    polygon: PolygonPayload
    inner_boundary: list[list[float]]
    source_polygon_index: int
    hole_index: int
    area: float


class ExtractSkeletonRequest(BaseModel):
    wall_geometries: list[GeometryPayload] = Field(default_factory=list)
    opening_geometries: list[GeometryPayload] = Field(default_factory=list)
    axis_geometries: list[GeometryPayload] = Field(default_factory=list)
    options: dict[str, Any] = Field(default_factory=dict)


class NormalizeSkeletonRequest(BaseModel):
    wall_axes: list[WallAxisPayload] = Field(default_factory=list)
    opening_embedments: list[OpeningPayload] = Field(default_factory=list)
    axis_lines: list[AxisLinePayload] = Field(default_factory=list)
    options: dict[str, Any] = Field(default_factory=dict)


class DesignStructureRequest(NormalizeSkeletonRequest):
    pass


class NormalizeStructureRequest(BaseModel):
    shear_walls: list[ShearWallPayload] = Field(default_factory=list)
    beams: list[BeamPayload] = Field(default_factory=list)
    options: dict[str, Any] = Field(default_factory=dict)


class SkeletonResponse(BaseModel):
    wall_axes: list[WallAxisPayload]
    opening_embedments: list[OpeningPayload]
    axis_lines: list[AxisLinePayload]
    slab_regions: list[SlabRegionPayload]
    isolated_points: list[list[float]]
    diagnostics: dict[str, Any]


class StructureResponse(BaseModel):
    shear_walls: list[ShearWallPayload]
    beams: list[BeamPayload]
    slab_regions: list[SlabRegionPayload]
    diagnostics: dict[str, Any]
