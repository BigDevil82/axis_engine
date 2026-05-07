from __future__ import annotations

from collections.abc import Iterable, Sequence
from math import cos, sin
from pathlib import Path
from time import sleep

import pythoncom
import pywintypes
import win32com.client
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import polygonize, unary_union


Point2D = tuple[float, float]
Transform2D = tuple[float, float, float, float, float, float]
IDENTITY_TRANSFORM: Transform2D = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def get_active_document():
    """Return the active AutoCAD document via COM."""
    pythoncom.CoInitialize()
    acad = win32com.client.GetActiveObject("AutoCAD.Application")
    return acad.ActiveDocument


def list_layer_names(doc) -> list[str]:
    return [layer.Name for layer in doc.Layers]


def pick_wall_layers(doc, explicit_layers: Sequence[str] | None = None) -> list[str]:
    if explicit_layers:
        return list(explicit_layers)

    candidates = []
    for name in list_layer_names(doc):
        lower_name = name.lower()
        upper_name = name.upper()
        if upper_name == "AXIS_WALL":
            continue
        if "墙" in name or "wall" in lower_name:
            candidates.append(name)

    if not candidates:
        raise RuntimeError("未找到包含 '墙' 或 'wall' 的墙体图层，请用 --wall-layer 显式指定。")

    return candidates


def read_line_segments_from_layers(doc, layer_names: Iterable[str], min_length: float = 1.0) -> list[LineString]:
    layer_set = set(layer_names)
    lines: list[LineString] = []

    for entity in doc.ModelSpace:
        for start, end in _entity_segments(entity, doc, layer_set):
            line = LineString([start, end])
            if line.length >= min_length:
                lines.append(line)

    return lines


def build_wall_polygon_from_lines(lines: Sequence[LineString]) -> MultiPolygon:
    if not lines:
        return MultiPolygon([])

    merged_lines = unary_union(lines)
    polygons = [poly for poly in polygonize(merged_lines) if poly.area > 0]
    return MultiPolygon(polygons)


def clear_layer(doc, layer_name: str) -> int:
    ensure_layer(doc, layer_name)
    layer = _com_retry(lambda: doc.Layers.Item(layer_name))
    if getattr(layer, "Lock", False):
        layer.Lock = False

    entities = [entity for entity in doc.ModelSpace if _com_retry(lambda item=entity: item.Layer) == layer_name]
    for entity in entities:
        _com_retry(entity.Delete)
    return len(entities)


def ensure_layer(doc, layer_name: str, color_index: int = 1):
    try:
        layer = _com_retry(lambda: doc.Layers.Item(layer_name))
    except Exception:
        layer = _com_retry(lambda: doc.Layers.Add(layer_name))

    _com_retry(lambda: setattr(layer, "Color", color_index))
    return layer


def draw_centerlines(doc, centerlines, layer_name: str = "AXIS_WALL", color_index: int = 1) -> int:
    layer = ensure_layer(doc, layer_name, color_index=color_index)
    if getattr(layer, "Lock", False):
        layer.Lock = False
    _com_retry(lambda: setattr(doc, "ActiveLayer", layer))

    count = 0
    for line, _thickness in centerlines:
        coords = list(line.coords)
        for start, end in zip(coords, coords[1:]):
            if LineString([start, end]).length <= 0:
                continue
            _com_retry(lambda: doc.ModelSpace.AddLine(_point3d(start), _point3d(end)))
            count += 1

    _com_retry(lambda: doc.Regen(1))
    return count


def draw_polygons(doc, geometry, layer_name: str = "polygon", color_index: int = 3) -> int:
    layer = ensure_layer(doc, layer_name, color_index=color_index)
    if getattr(layer, "Lock", False):
        layer.Lock = False
    _com_retry(lambda: setattr(doc, "ActiveLayer", layer))

    polygons = []
    if geometry is None or geometry.is_empty:
        return 0
    if isinstance(geometry, Polygon):
        polygons = [geometry]
    elif isinstance(geometry, MultiPolygon):
        polygons = list(geometry.geoms)
    else:
        return 0

    count = 0
    for polygon in polygons:
        count += _draw_ring_lines(doc, polygon.exterior.coords)
        for interior in polygon.interiors:
            count += _draw_ring_lines(doc, interior.coords)

    _com_retry(lambda: doc.Regen(1))
    return count


def _entity_segments(
    entity,
    doc,
    layer_names: set[str],
    parent_selected: bool = False,
    transform: Transform2D = IDENTITY_TRANSFORM,
    visited_blocks: frozenset[str] = frozenset(),
) -> list[tuple[Point2D, Point2D]]:
    try:
        entity_name = _com_retry(lambda: entity.EntityName)
        entity_layer = _com_retry(lambda: entity.Layer)
    except Exception:
        return []
    selected = parent_selected or entity_layer in layer_names

    if entity_name == "AcDbLine":
        if not selected:
            return []
        return [
            (
                _apply_transform(_xy(_com_retry(lambda: entity.StartPoint)), transform),
                _apply_transform(_xy(_com_retry(lambda: entity.EndPoint)), transform),
            )
        ]

    if entity_name in {"AcDbPolyline", "AcDb2dPolyline"}:
        if not selected:
            return []
        return [
            (_apply_transform(start, transform), _apply_transform(end, transform))
            for start, end in _polyline_segments(entity)
        ]

    if entity_name == "AcDbBlockReference":
        return _block_reference_segments(entity, doc, layer_names, selected, transform, visited_blocks)

    return []


def _block_reference_segments(
    entity,
    doc,
    layer_names: set[str],
    parent_selected: bool,
    parent_transform: Transform2D,
    visited_blocks: frozenset[str],
) -> list[tuple[Point2D, Point2D]]:
    block_name = _block_name(entity)
    if not block_name or block_name in visited_blocks:
        return []

    try:
        block = _com_retry(lambda: doc.Blocks.Item(block_name))
    except Exception:
        return []

    transform = _compose_transform(parent_transform, _block_transform(entity))
    segments: list[tuple[Point2D, Point2D]] = []
    next_visited = visited_blocks | {block_name}
    for child in block:
        segments.extend(_entity_segments(child, doc, layer_names, parent_selected, transform, next_visited))
    return segments


def _polyline_segments(entity) -> list[tuple[Point2D, Point2D]]:
    coords = list(_com_retry(lambda: entity.Coordinates))
    stride = 2 if len(coords) % 2 == 0 else 3
    points = [tuple(coords[i : i + 2]) for i in range(0, len(coords), stride)]

    segments = list(zip(points, points[1:]))
    if _com_retry(lambda: getattr(entity, "Closed", False)) and len(points) > 2:
        segments.append((points[-1], points[0]))
    return segments


def _xy(point) -> Point2D:
    return (float(point[0]), float(point[1]))


def _point3d(point: Sequence[float]):
    return win32com.client.VARIANT(
        pythoncom.VT_ARRAY | pythoncom.VT_R8,
        [float(point[0]), float(point[1]), 0.0],
    )


def _draw_ring_lines(doc, coords) -> int:
    points = list(coords)
    if len(points) < 4:
        return 0

    if points[0] == points[-1]:
        points = points[:-1]

    flat_coords = []
    for point in points:
        flat_coords.extend([float(point[0]), float(point[1])])

    polyline = _com_retry(lambda: doc.ModelSpace.AddLightWeightPolyline(_double_array(flat_coords)))
    _com_retry(lambda: setattr(polyline, "Closed", True))
    return 1


def _double_array(values: Sequence[float]):
    return win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, list(values))


def _block_name(entity) -> str | None:
    for attr in ("EffectiveName", "Name"):
        try:
            return _com_retry(lambda attr=attr: getattr(entity, attr))
        except Exception:
            continue
    return None


def _block_transform(entity) -> Transform2D:
    insert = _xy(_com_retry(lambda: entity.InsertionPoint))
    x_scale = float(_com_retry(lambda: entity.XScaleFactor))
    y_scale = float(_com_retry(lambda: entity.YScaleFactor))
    rotation = float(_com_retry(lambda: entity.Rotation))
    c = cos(rotation)
    s = sin(rotation)
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


def document_name(doc) -> str:
    try:
        return Path(doc.FullName).name
    except Exception:
        return doc.Name


def _com_retry(func, attempts: int = 20, delay: float = 0.1):
    last_error = None
    for _ in range(attempts):
        try:
            return func()
        except (pywintypes.com_error, AttributeError) as exc:
            last_error = exc
            pythoncom.PumpWaitingMessages()
            sleep(delay)
    raise last_error
