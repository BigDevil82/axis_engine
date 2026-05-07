from __future__ import annotations

import logging
from collections.abc import Iterable, Sequence
from pathlib import Path

import ezdxf
from shapely.geometry import LineString


logging.getLogger("ezdxf").setLevel(logging.ERROR)


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


def read_dxf_line_segments_from_layers(doc, layer_names: Iterable[str], min_length: float = 1.0) -> list[LineString]:
    layer_set = set(layer_names)
    lines: list[LineString] = []

    def collect(entity, selected_by_parent: bool):
        if not selected_by_parent and entity.dxf.layer not in layer_set:
            return

        for start, end in _entity_segments(entity):
            line = LineString([start, end])
            if line.length >= min_length:
                lines.append(line)

    for entity in doc.modelspace():
        for expanded_entity, selected_by_parent in _iter_entity_and_nested_virtuals(entity, layer_set):
            collect(expanded_entity, selected_by_parent)

    return lines


def read_dxf_line_segment_groups_from_layers(
    doc,
    layer_names: Iterable[str],
    min_length: float = 1.0,
) -> list[list[LineString]]:
    layer_set = set(layer_names)
    groups: list[list[LineString]] = []

    for entity in doc.modelspace():
        entity_group: list[LineString] = []
        for expanded_entity, selected_by_parent in _iter_entity_and_nested_virtuals(entity, layer_set):
            if not selected_by_parent and expanded_entity.dxf.layer not in layer_set:
                continue

            for start, end in _entity_segments(expanded_entity):
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
):
    selected_layers = selected_layers or set()
    selected = parent_selected or entity.dxf.layer in selected_layers
    yield entity, parent_selected

    if max_depth <= 0 or entity.dxftype() != "INSERT":
        return

    try:
        virtual_entities = list(entity.virtual_entities())
    except Exception:
        return

    for virtual_entity in virtual_entities:
        yield from _iter_entity_and_nested_virtuals(
            virtual_entity,
            selected_layers=selected_layers,
            parent_selected=selected,
            max_depth=max_depth - 1,
        )


def _entity_segments(entity) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    entity_type = entity.dxftype()

    if entity_type == "LINE":
        return [(_xy(entity.dxf.start), _xy(entity.dxf.end))]

    if entity_type == "LWPOLYLINE":
        points = [(float(x), float(y)) for x, y in entity.get_points("xy")]
        return _segments_from_points(points, bool(entity.closed))

    if entity_type == "POLYLINE":
        points = [_xy(vertex.dxf.location) for vertex in entity.vertices]
        return _segments_from_points(points, bool(entity.is_closed))

    return []


def _segments_from_points(points: Sequence[tuple[float, float]], closed: bool):
    segments = list(zip(points, points[1:]))
    if closed and len(points) > 2:
        segments.append((points[-1], points[0]))
    return segments


def _xy(point) -> tuple[float, float]:
    return (float(point[0]), float(point[1]))
