from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from collections.abc import Iterable, Sequence
from pathlib import Path

import ezdxf
from ezdxf.math import bulge_to_arc
from shapely.geometry import LineString


logging.getLogger("ezdxf").setLevel(logging.ERROR)

Point2D = tuple[float, float]
Transform2D = tuple[float, float, float, float, float, float]
IDENTITY_TRANSFORM: Transform2D = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


@dataclass(frozen=True)
class DxfLineSegment:
    line: LineString
    source_type: str
    is_arc: bool = False


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


def read_dxf_line_segments_from_layers(
    doc,
    layer_names: Iterable[str],
    min_length: float = 1.0,
    visible_only: bool = False,
) -> list[LineString]:
    return [
        segment.line
        for segment in read_dxf_segments_from_layers(
            doc,
            layer_names,
            min_length=min_length,
            visible_only=visible_only,
        )
    ]


def read_dxf_segments_from_layers(
    doc,
    layer_names: Iterable[str],
    min_length: float = 1.0,
    visible_only: bool = False,
) -> list[DxfLineSegment]:
    layer_set = set(layer_names)
    segments: list[DxfLineSegment] = []

    def collect(entity, selected_by_parent: bool, transform: Transform2D):
        if not selected_by_parent and entity.dxf.layer not in layer_set:
            return

        for start, end, is_arc, source_type in _entity_segments(entity, transform):
            line = LineString([start, end])
            if line.length >= min_length:
                segments.append(DxfLineSegment(line=line, source_type=source_type, is_arc=is_arc))

    for entity in doc.modelspace():
        for expanded_entity, selected_by_parent, transform in _iter_entity_and_nested_virtuals(
            entity,
            layer_set,
            doc=doc,
            visible_only=visible_only,
        ):
            collect(expanded_entity, selected_by_parent, transform)

    return segments


def read_dxf_line_segment_groups_from_layers(
    doc,
    layer_names: Iterable[str],
    min_length: float = 1.0,
    visible_only: bool = False,
) -> list[list[LineString]]:
    layer_set = set(layer_names)
    groups: list[list[LineString]] = []

    for entity in doc.modelspace():
        entity_group: list[LineString] = []
        for expanded_entity, selected_by_parent, transform in _iter_entity_and_nested_virtuals(
            entity,
            layer_set,
            doc=doc,
            visible_only=visible_only,
        ):
            if not selected_by_parent and expanded_entity.dxf.layer not in layer_set:
                continue

            for start, end, _is_arc, _source_type in _entity_segments(expanded_entity, transform):
                line = LineString([start, end])
                if line.length >= min_length:
                    entity_group.append(line)

        if entity_group:
            groups.append(entity_group)

    return groups


def _iter_entity_and_nested_virtuals(
    entity,
    selected_layers: set[str] | None = None,
    parent_selected: bool = False,
    max_depth: int = 8,
    doc=None,
    visible_only: bool = False,
    in_block: bool = False,
    transform: Transform2D = IDENTITY_TRANSFORM,
    visited_blocks: frozenset[str] = frozenset(),
):
    selected_layers = selected_layers or set()
    if visible_only and not _is_entity_visible(entity, doc, in_block):
        return

    selected = parent_selected or entity.dxf.layer in selected_layers
    yield entity, parent_selected, transform

    if max_depth <= 0 or entity.dxftype() != "INSERT":
        return

    if doc is None:
        return

    try:
        block_name = entity.dxf.name
        if block_name in visited_blocks:
            return
        block = doc.blocks[block_name]
    except Exception:
        return

    next_transform = _compose_transform(transform, _insert_transform(entity))
    next_visited = visited_blocks | {block_name}
    for virtual_entity in block:
        yield from _iter_entity_and_nested_virtuals(
            virtual_entity,
            selected_layers=selected_layers,
            parent_selected=selected,
            max_depth=max_depth - 1,
            doc=doc,
            visible_only=visible_only,
            in_block=True,
            transform=next_transform,
            visited_blocks=next_visited,
        )


def _entity_segments(
    entity,
    transform: Transform2D = IDENTITY_TRANSFORM,
) -> list[tuple[tuple[float, float], tuple[float, float], bool, str]]:
    entity_type = entity.dxftype()

    if entity_type == "LINE":
        return [
            (
                _apply_transform(_xy(entity.dxf.start), transform),
                _apply_transform(_xy(entity.dxf.end), transform),
                False,
                entity_type,
            )
        ]

    if entity_type in {"ARC", "CIRCLE"}:
        sagitta = _local_curve_tolerance(transform)
        return _tag_segments(
            _segments_from_points([_apply_transform(_xy(point), transform) for point in entity.flattening(sagitta)], False),
            is_arc=True,
            source_type=entity_type,
        )

    if entity_type == "ELLIPSE":
        distance = _local_curve_tolerance(transform)
        return _tag_segments(
            _segments_from_points([_apply_transform(_xy(point), transform) for point in entity.flattening(distance)], False),
            is_arc=True,
            source_type=entity_type,
        )

    if entity_type == "LWPOLYLINE":
        points = [((float(x), float(y)), float(bulge)) for x, y, _start_width, _end_width, bulge in entity.get_points("xyseb")]
        return _segments_from_bulged_points(points, bool(entity.closed), transform, source_type=entity_type)

    if entity_type == "POLYLINE":
        points = [(_xy(vertex.dxf.location), float(vertex.dxf.get("bulge", 0.0))) for vertex in entity.vertices]
        return _segments_from_bulged_points(points, bool(entity.is_closed), transform, source_type=entity_type)

    return []


def _segments_from_points(points: Sequence[tuple[float, float]], closed: bool):
    segments = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        segments.append((points[-1], points[0]))
    return segments


def _segments_from_bulged_points(
    points: Sequence[tuple[tuple[float, float], float]],
    closed: bool,
    transform: Transform2D = IDENTITY_TRANSFORM,
    source_type: str = "POLYLINE",
):
    if len(points) < 2:
        return []

    segments = []
    pairs = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        pairs.append((points[-1], points[0]))

    for (start, bulge), (end, _next_bulge) in pairs:
        if abs(bulge) < 1e-12:
            segments.append((_apply_transform(start, transform), _apply_transform(end, transform), False, source_type))
            continue

        arc_points = _bulge_arc_points(start, end, bulge, transform)
        segments.extend(
            _tag_segments(
                _transform_segments(_segments_from_points(arc_points, False), transform),
                is_arc=True,
                source_type=source_type,
            )
        )

    return segments


def _bulge_arc_points(
    start: tuple[float, float],
    end: tuple[float, float],
    bulge: float,
    transform: Transform2D = IDENTITY_TRANSFORM,
):
    center, start_angle, end_angle, radius = bulge_to_arc(start, end, bulge)
    angle_span = (end_angle - start_angle) % (math.tau)
    max_segment_length = _local_curve_tolerance(transform) * 8.0
    segments = max(4, int(math.ceil(radius * angle_span / max_segment_length)))
    return [
        (
            float(center.x + radius * math.cos(start_angle + angle_span * index / segments)),
            float(center.y + radius * math.sin(start_angle + angle_span * index / segments)),
        )
        for index in range(segments + 1)
    ]


def _xy(point) -> tuple[float, float]:
    return (float(point[0]), float(point[1]))


def _transform_segments(
    segments: Sequence[tuple[Point2D, Point2D]],
    transform: Transform2D,
) -> list[tuple[Point2D, Point2D]]:
    return [(_apply_transform(start, transform), _apply_transform(end, transform)) for start, end in segments]


def _tag_segments(
    segments: Sequence[tuple[Point2D, Point2D]],
    is_arc: bool,
    source_type: str,
) -> list[tuple[Point2D, Point2D, bool, str]]:
    return [(start, end, is_arc, source_type) for start, end in segments]


def _local_curve_tolerance(transform: Transform2D, model_tolerance: float = 10.0) -> float:
    a, b, c, d, _e, _f = transform
    x_scale = math.hypot(a, c)
    y_scale = math.hypot(b, d)
    scale = max(x_scale, y_scale, 1.0)
    return max(0.001, model_tolerance / scale)


def _insert_transform(entity) -> Transform2D:
    insert = _xy(entity.dxf.insert)
    x_scale = float(entity.dxf.get("xscale", 1.0))
    y_scale = float(entity.dxf.get("yscale", 1.0))
    rotation = math.radians(float(entity.dxf.get("rotation", 0.0)))
    c = math.cos(rotation)
    s = math.sin(rotation)
    return (
        c * x_scale,
        -s * y_scale,
        s * x_scale,
        c * y_scale,
        insert[0],
        insert[1],
    )


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


def _is_entity_visible(entity, doc, in_block: bool) -> bool:
    if bool(entity.dxf.get("invisible", 0)):
        return False

    layer_name = entity.dxf.layer
    if in_block and layer_name == "0":
        return True

    if doc is None or layer_name not in doc.layers:
        return True

    layer = doc.layers.get(layer_name)
    return not layer.is_off() and not layer.is_frozen()
