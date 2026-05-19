from __future__ import annotations

import logging
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import ezdxf
from ezdxf.math import bulge_to_arc
from shapely.geometry import LineString, Point

logging.getLogger("ezdxf").setLevel(logging.ERROR)

Point2D = tuple[float, float]
Transform2D = tuple[float, float, float, float, float, float]
IDENTITY_TRANSFORM: Transform2D = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


@dataclass(frozen=True)
class DxfGeometry:
    """A faithful 2D primitive read from DXF after block transforms are applied."""

    geom_type: str
    params: dict[str, Any]
    layer: str
    source_type: str
    block_path: tuple[str, ...] = field(default_factory=tuple)

    @property
    def start(self) -> Point2D | None:
        return self.params.get("start")

    @property
    def end(self) -> Point2D | None:
        return self.params.get("end")

    @property
    def center(self) -> Point2D | None:
        return self.params.get("center")

    def as_linework(self, curve_tolerance: float = 10.0) -> list[LineString]:
        """Return a local linework view for algorithms that only consume segments."""
        if self.geom_type == "LINE":
            return [LineString([self.params["start"], self.params["end"]])]

        if self.geom_type == "ARC":
            return [LineString(_arc_points(self.params, curve_tolerance))]

        if self.geom_type == "CIRCLE":
            return [LineString(_circle_points(self.params, curve_tolerance))]

        if self.geom_type == "ELLIPSE":
            points = self.params.get("points", ())
            return [LineString(points)] if len(points) >= 2 else []

        if self.geom_type in {"POLYLINE", "LWPOLYLINE"}:
            points = list(self.params.get("points", ()))
            if self.params.get("closed") and points:
                points.append(points[0])
            return [LineString(points)] if len(points) >= 2 else []

        return []

    def representative_line(self, curve_tolerance: float = 20.0) -> LineString:
        lines = self.as_linework(curve_tolerance)
        if lines:
            return max(lines, key=lambda line: line.length)
        point = self.center or self.start or (0.0, 0.0)
        return LineString([point, point])


def read_dxf(path: str | Path):
    return ezdxf.readfile(Path(path))


def list_dxf_layer_names(doc) -> list[str]:
    return [layer.dxf.name for layer in doc.layers]


def pick_dxf_wall_layers(doc, explicit_layers: Sequence[str] | None = None) -> list[str]:
    if explicit_layers:
        return list(explicit_layers)

    candidates = []
    for name in list_dxf_layer_names(doc):
        lower_name = name.lower()
        if name.upper() == "AXIS_WALL":
            continue
        if "墙" in name or "wall" in lower_name:
            candidates.append(name)

    if not candidates:
        raise RuntimeError("DXF 中未找到包含 '墙' 或 'wall' 的墙体图层，请用 --wall-layer 显式指定。")

    return candidates


def pick_dxf_axis_layers(doc, explicit_layers: Sequence[str] | None = None) -> list[str]:
    if explicit_layers:
        return list(explicit_layers)

    names = list_dxf_layer_names(doc)
    if "AXIS_WALL" in names:
        return ["AXIS_WALL"]

    candidates = []
    for name in names:
        lower_name = name.lower()
        if "轴" in name or "axis" in lower_name:
            candidates.append(name)
    return candidates


def read_dxf_geometries_from_layers(
    doc,
    layer_names: Iterable[str],
    min_length: float = 1.0,
    visible_only: bool = False,
) -> list[DxfGeometry]:
    layer_set = set(layer_names)
    geometries: list[DxfGeometry] = []

    for entity in doc.modelspace():
        for expanded, transform, effective_layer, block_path in _iter_entity_and_nested_virtuals(
            entity,
            doc=doc,
            visible_only=visible_only,
        ):
            if effective_layer not in layer_set:
                continue
            if getattr(expanded.dxf, "invisible", 0):
                continue
            for geometry in _entity_geometries(expanded, transform, effective_layer, block_path):
                if _geometry_length(geometry) >= min_length:
                    geometries.append(geometry)

    return geometries


def geometries_to_linework(
    geometries: Iterable[DxfGeometry],
    curve_tolerance: float = 10.0,
    min_length: float = 1.0,
) -> list[LineString]:
    lines: list[LineString] = []
    for geometry in geometries:
        for line in geometry.as_linework(curve_tolerance):
            coords = list(line.coords)
            for start, end in zip(coords, coords[1:]):
                segment = LineString([start, end])
                if segment.length >= min_length:
                    lines.append(segment)
    return lines


def _iter_entity_and_nested_virtuals(
    entity,
    max_depth: int = 8,
    doc=None,
    visible_only: bool = False,
    in_block: bool = False,
    transform: Transform2D = IDENTITY_TRANSFORM,
    visited_blocks: frozenset[str] = frozenset(),
    parent_layer: str | None = None,
    block_path: tuple[str, ...] = (),
):
    own_layer = entity.dxf.layer
    effective_layer = parent_layer if (own_layer == "0" and parent_layer is not None) else own_layer
    if visible_only and not _is_entity_visible(entity, doc, effective_layer):
        return

    yield entity, transform, effective_layer, block_path

    if max_depth <= 0 or entity.dxftype() != "INSERT" or doc is None:
        return

    try:
        block_name = entity.dxf.name
        if block_name in visited_blocks or block_name.startswith("*"):
            return
        block = doc.blocks[block_name]
    except Exception:
        return

    next_transform = _compose_transform(transform, _insert_transform(entity))
    next_visited = visited_blocks | {block_name}
    next_block_path = block_path + (block_name,)

    for virtual_entity in block:
        if virtual_entity.dxftype() in ("ATTDEF", "SEQEND"):
            continue
        yield from _iter_entity_and_nested_virtuals(
            virtual_entity,
            max_depth=max_depth - 1,
            doc=doc,
            visible_only=visible_only,
            in_block=True,
            transform=next_transform,
            visited_blocks=next_visited,
            parent_layer=effective_layer,
            block_path=next_block_path,
        )


def _entity_geometries(
    entity,
    transform: Transform2D,
    layer: str,
    block_path: tuple[str, ...],
) -> list[DxfGeometry]:
    entity_type = entity.dxftype()

    if entity_type == "LINE":
        return [
            DxfGeometry(
                "LINE",
                {"start": _apply_transform(_xy(entity.dxf.start), transform), "end": _apply_transform(_xy(entity.dxf.end), transform)},
                layer,
                entity_type,
                block_path,
            )
        ]

    if entity_type == "ARC":
        return [_arc_geometry(entity, transform, layer, block_path)]

    if entity_type == "CIRCLE":
        center = _apply_transform(_xy(entity.dxf.center), transform)
        scale = _average_scale(transform)
        return [
            DxfGeometry(
                "CIRCLE",
                {"center": center, "radius": abs(float(entity.dxf.radius) * scale)},
                layer,
                entity_type,
                block_path,
            )
        ]

    if entity_type == "ELLIPSE":
        points = tuple(_apply_transform(_xy(point), transform) for point in entity.flattening(_local_curve_tolerance(transform)))
        return [DxfGeometry("ELLIPSE", {"points": points}, layer, entity_type, block_path)]

    if entity_type == "LWPOLYLINE":
        points = [
            ((float(x), float(y)), float(bulge))
            for x, y, _start_width, _end_width, bulge in entity.get_points("xyseb")
        ]
        return _polyline_geometries(points, bool(entity.closed), transform, layer, entity_type, block_path)

    if entity_type == "POLYLINE":
        points = [(_xy(vertex.dxf.location), float(vertex.dxf.get("bulge", 0.0))) for vertex in entity.vertices]
        return _polyline_geometries(points, bool(entity.is_closed), transform, layer, entity_type, block_path)

    return []


def _arc_geometry(entity, transform: Transform2D, layer: str, block_path: tuple[str, ...]) -> DxfGeometry:
    center_raw = _xy(entity.dxf.center)
    radius = float(entity.dxf.radius)
    start_angle = math.radians(float(entity.dxf.start_angle))
    end_angle = math.radians(float(entity.dxf.end_angle))
    sweep_angle = (end_angle - start_angle) % math.tau
    if _transform_determinant(transform) < 0:
        sweep_angle = -sweep_angle
    start_raw = (center_raw[0] + radius * math.cos(start_angle), center_raw[1] + radius * math.sin(start_angle))
    end_raw = (center_raw[0] + radius * math.cos(end_angle), center_raw[1] + radius * math.sin(end_angle))
    return DxfGeometry(
        "ARC",
        {
            "center": _apply_transform(center_raw, transform),
            "radius": abs(radius * _average_scale(transform)),
            "start": _apply_transform(start_raw, transform),
            "end": _apply_transform(end_raw, transform),
            "start_angle": start_angle + _rotation_angle(transform),
            "end_angle": end_angle + _rotation_angle(transform),
            "sweep_angle": sweep_angle,
        },
        layer,
        "ARC",
        block_path,
    )


def _polyline_geometries(
    points: Sequence[tuple[Point2D, float]],
    closed: bool,
    transform: Transform2D,
    layer: str,
    source_type: str,
    block_path: tuple[str, ...],
) -> list[DxfGeometry]:
    if len(points) < 2:
        return []

    geometries: list[DxfGeometry] = []
    pairs = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        pairs.append((points[-1], points[0]))

    for (start, bulge), (end, _next_bulge) in pairs:
        if abs(bulge) < 1e-12:
            geometries.append(
                DxfGeometry(
                    "LINE",
                    {"start": _apply_transform(start, transform), "end": _apply_transform(end, transform)},
                    layer,
                    source_type,
                    block_path,
                )
            )
            continue

        center, start_angle, end_angle, radius = bulge_to_arc(start, end, bulge)
        sweep_angle = 4.0 * math.atan(bulge)
        if _transform_determinant(transform) < 0:
            sweep_angle = -sweep_angle
        center_xy = (float(center.x), float(center.y))
        geometries.append(
            DxfGeometry(
                "ARC",
                {
                    "center": _apply_transform(center_xy, transform),
                    "radius": abs(float(radius) * _average_scale(transform)),
                    "start": _apply_transform(start, transform),
                    "end": _apply_transform(end, transform),
                    "start_angle": float(start_angle) + _rotation_angle(transform),
                    "end_angle": float(end_angle) + _rotation_angle(transform),
                    "sweep_angle": sweep_angle,
                },
                layer,
                source_type,
                block_path,
            )
        )

    return geometries


def _arc_points(params: dict[str, Any], tolerance: float) -> list[Point2D]:
    center = params["center"]
    radius = float(params["radius"])
    start = params["start"]
    end = params["end"]
    start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
    if "sweep_angle" in params:
        span = float(params["sweep_angle"])
    else:
        end_angle = math.atan2(end[1] - center[1], end[0] - center[0])
        span = (end_angle - start_angle) % math.tau

    segment_count = max(6, int(math.ceil(radius * abs(span) / max(tolerance, 0.001))))
    points = [
        (
            center[0] + radius * math.cos(start_angle + span * index / segment_count),
            center[1] + radius * math.sin(start_angle + span * index / segment_count),
        )
        for index in range(segment_count + 1)
    ]
    points[0] = start
    points[-1] = end
    return points


def _circle_points(params: dict[str, Any], tolerance: float) -> list[Point2D]:
    center = params["center"]
    radius = float(params["radius"])
    segment_count = max(16, int(math.ceil(math.tau * radius / max(tolerance, 0.001))))
    return [
        (
            center[0] + radius * math.cos(math.tau * index / segment_count),
            center[1] + radius * math.sin(math.tau * index / segment_count),
        )
        for index in range(segment_count + 1)
    ]


def _geometry_length(geometry: DxfGeometry) -> float:
    if geometry.geom_type == "LINE":
        return Point(geometry.params["start"]).distance(Point(geometry.params["end"]))
    return sum(line.length for line in geometry.as_linework())


def _xy(point) -> Point2D:
    return (float(point[0]), float(point[1]))


def _local_curve_tolerance(transform: Transform2D, model_tolerance: float = 10.0) -> float:
    scale = max(_average_scale(transform), 1.0)
    return max(0.001, model_tolerance / scale)


def _average_scale(transform: Transform2D) -> float:
    a, b, c, d, _e, _f = transform
    return (math.hypot(a, c) + math.hypot(b, d)) / 2.0


def _transform_determinant(transform: Transform2D) -> float:
    a, b, c, d, _e, _f = transform
    return a * d - b * c


def _rotation_angle(transform: Transform2D) -> float:
    a, _b, c, _d, _e, _f = transform
    return math.atan2(c, a)


def _insert_transform(entity) -> Transform2D:
    insert = _xy(entity.dxf.insert)
    x_scale = float(entity.dxf.get("xscale", 1.0))
    y_scale = float(entity.dxf.get("yscale", 1.0))
    rotation = math.radians(float(entity.dxf.get("rotation", 0.0)))
    c = math.cos(rotation)
    s = math.sin(rotation)
    return (c * x_scale, -s * y_scale, s * x_scale, c * y_scale, insert[0], insert[1])


def _apply_transform(point: Point2D, transform: Transform2D) -> Point2D:
    a, b, c, d, e, f = transform
    x, y = point
    return (a * x + b * y + e, c * x + d * y + f)


def _compose_transform(parent: Transform2D, child: Transform2D) -> Transform2D:
    pa, pb, pc, pd, pe, pf = parent
    ca, cb, cc, cd, ce, cf = child
    return (
        pa * ca + pb * cc,
        pa * cb + pb * cd,
        pc * ca + pd * cc,
        pc * cb + pd * cd,
        pa * ce + pb * cf + pe,
        pc * ce + pd * cf + pf,
    )


def _is_entity_visible(entity, doc, effective_layer: str) -> bool:
    if bool(entity.dxf.get("invisible", 0)):
        return False

    layer_name = effective_layer
    if doc is None or layer_name not in doc.layers:
        return True

    layer = doc.layers.get(layer_name)
    return not layer.is_off() and not layer.is_frozen()
