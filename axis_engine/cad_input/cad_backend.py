from __future__ import annotations

import math

from shapely.geometry import LineString

from axis_engine.dxf_utils import DxfGeometry


class ActiveCadInputSource:
    def __init__(self):
        self.document = _active_autocad_document()
        self._geometry_cache: list[DxfGeometry] | None = None

    def list_layer_names(self) -> list[str]:
        return [layer.Name for layer in self.document.Layers]

    def read_geometries_from_layers(
        self,
        layer_names,
        min_length: float = 1.0,
        visible_only: bool = False,
    ) -> list[DxfGeometry]:
        layer_set = set(layer_names)
        if not visible_only:
            return [
                geometry
                for geometry in self._all_geometries()
                if geometry.layer in layer_set and _geometry_length(geometry) >= min_length
            ]

        geometries: list[DxfGeometry] = []
        for entity in list(self.document.ModelSpace):
            for expanded, temporary, effective_layer in _iter_visible_entity_views(
                entity,
                visible_only=visible_only,
            ):
                if effective_layer not in layer_set:
                    if temporary:
                        _delete_if_temporary(expanded)
                    continue
                geometries.extend(_entity_geometries(expanded, IDENTITY_TRANSFORM, effective_layer))
                if temporary:
                    _delete_if_temporary(expanded)
        return [geometry for geometry in geometries if _geometry_length(geometry) >= min_length]

    def _all_geometries(self) -> list[DxfGeometry]:
        if self._geometry_cache is not None:
            return self._geometry_cache

        geometries: list[DxfGeometry] = []
        for entity in list(self.document.ModelSpace):
            for expanded, temporary, effective_layer in _iter_visible_entity_views(
                entity,
                visible_only=False,
            ):
                geometries.extend(_entity_geometries(expanded, IDENTITY_TRANSFORM, effective_layer))
                if temporary:
                    _delete_if_temporary(expanded)
        self._geometry_cache = geometries
        return geometries


Transform2D = tuple[float, float, float, float, float, float]
IDENTITY_TRANSFORM: Transform2D = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def _active_autocad_document():
    try:
        import pythoncom
        import win32com.client
    except ImportError as exc:
        raise RuntimeError("AutoCAD COM 读取需要 pywin32。") from exc

    pythoncom.CoInitialize()
    try:
        app = win32com.client.GetActiveObject("AutoCAD.Application")
    except Exception:
        app = win32com.client.Dispatch("AutoCAD.Application")
    if app.Documents.Count == 0:
        raise RuntimeError("当前 AutoCAD 没有打开任何图形。")
    return app.ActiveDocument


def _iter_visible_entity_views(
    entity,
    *,
    visible_only: bool = False,
    parent_layer: str | None = None,
    temporary: bool = False,
    max_depth: int = 8,
):
    try:
        own_layer = str(entity.Layer)
        if visible_only and not bool(entity.Visible):
            return
    except Exception:
        return

    effective_layer = parent_layer if own_layer == "0" and parent_layer is not None else own_layer
    if str(getattr(entity, "ObjectName", "")) != "AcDbBlockReference":
        yield entity, temporary, effective_layer
        return

    if max_depth <= 0:
        if temporary:
            _delete_if_temporary(entity)
        return

    try:
        exploded = list(entity.Explode())
    except Exception:
        if temporary:
            _delete_if_temporary(entity)
        return
    for child in exploded:
        yield from _iter_visible_entity_views(
            child,
            visible_only=visible_only,
            parent_layer=effective_layer,
            temporary=True,
            max_depth=max_depth - 1,
        )
    if temporary:
        _delete_if_temporary(entity)


def _entity_geometries(entity, transform: Transform2D, layer: str) -> list[DxfGeometry]:
    object_name = str(getattr(entity, "ObjectName", ""))
    if object_name == "AcDbLine":
        return [
            DxfGeometry(
                "LINE",
                {"start": _apply_transform(_xy(entity.StartPoint), transform), "end": _apply_transform(_xy(entity.EndPoint), transform)},
                layer,
                object_name,
            )
        ]

    if object_name == "AcDbArc":
        center_raw = _xy(entity.Center)
        center = _apply_transform(center_raw, transform)
        radius = abs(float(entity.Radius) * _average_scale(transform))
        start_angle = float(entity.StartAngle)
        end_angle = float(entity.EndAngle)
        sweep_angle = (end_angle - start_angle) % math.tau
        start = _apply_transform(
            (center_raw[0] + math.cos(start_angle) * float(entity.Radius), center_raw[1] + math.sin(start_angle) * float(entity.Radius)),
            transform,
        )
        end = _apply_transform(
            (center_raw[0] + math.cos(end_angle) * float(entity.Radius), center_raw[1] + math.sin(end_angle) * float(entity.Radius)),
            transform,
        )
        return [
            DxfGeometry(
                "ARC",
                {
                    "center": center,
                    "radius": radius,
                    "start_angle": start_angle,
                    "end_angle": end_angle,
                    "sweep_angle": sweep_angle,
                    "start": start,
                    "end": end,
                },
                layer,
                object_name,
            )
        ]

    if object_name == "AcDbCircle":
        return [
            DxfGeometry(
                "CIRCLE",
                {"center": _apply_transform(_xy(entity.Center), transform), "radius": abs(float(entity.Radius) * _average_scale(transform))},
                layer,
                object_name,
            )
        ]

    if object_name in {"AcDbPolyline", "AcDb2dPolyline", "AcDb3dPolyline"}:
        return _polyline_geometries(entity, transform, layer, object_name)

    return []


def _polyline_geometries(entity, transform: Transform2D, layer: str, source_type: str) -> list[DxfGeometry]:
    points = [_apply_transform(point, transform) for point in _polyline_points(entity)]
    if len(points) < 2:
        return []
    if bool(getattr(entity, "Closed", False)):
        points = points + [points[0]]
    return [
        DxfGeometry(
            "LINE",
            {"start": start, "end": end},
            layer,
            source_type,
        )
        for start, end in zip(points, points[1:])
        if LineString([start, end]).length > 0
    ]


def _polyline_points(entity) -> list[tuple[float, float]]:
    coords = list(entity.Coordinates)
    object_name = str(getattr(entity, "ObjectName", ""))
    step = 3 if object_name == "AcDb3dPolyline" else 2
    points = []
    for index in range(0, len(coords), step):
        if index + 1 < len(coords):
            points.append((float(coords[index]), float(coords[index + 1])))
    return points


def _geometry_length(geometry: DxfGeometry) -> float:
    return geometry.representative_line().length


def _xy(point) -> tuple[float, float]:
    return (float(point[0]), float(point[1]))


def _apply_transform(point: tuple[float, float], transform: Transform2D) -> tuple[float, float]:
    a, b, c, d, e, f = transform
    x, y = point
    return (a * x + b * y + e, c * x + d * y + f)


def _average_scale(transform: Transform2D) -> float:
    a, b, c, d, _e, _f = transform
    sx = math.hypot(a, c)
    sy = math.hypot(b, d)
    return (sx + sy) / 2.0


def _delete_if_temporary(entity):
    try:
        entity.Delete()
    except Exception:
        pass
