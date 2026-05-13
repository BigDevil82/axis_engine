from __future__ import annotations

from collections.abc import Iterable, Sequence

from shapely.geometry import LineString

from axis_engine.geometry_utils import iter_straight_segments
from axis_engine.opening_embedment import OpeningEmbedment
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall, StructuralDesignResult
from axis_engine.dxf_io.common import (
    line_endpoints_3d,
    opening_lines,
    read_thickness_from_com_entity,
    thickness_to_lineweight,
)
from axis_engine.dxf_io.models import DesignDxfData, DesignLayerNames, SkeletonDxfData, SkeletonLayerNames


def write_skeleton(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments_or_lines: Iterable[OpeningEmbedment | LineString],
    layers: SkeletonLayerNames,
):
    document = active_autocad_document()
    _prepare_layers(document, (layers.wall, layers.opening))
    _clear_layers(document, (layers.wall, layers.opening))

    for line, thickness in wall_axes:
        for segment in iter_straight_segments(line):
            _add_line(document, segment, layer=layers.wall, color=1, lineweight=thickness_to_lineweight(thickness))

    for line in opening_lines(opening_embedments_or_lines):
        for segment in iter_straight_segments(line):
            _add_line(document, segment, layer=layers.opening, color=3, lineweight=50)

    document.Regen(1)


def read_skeleton(
    layers: SkeletonLayerNames,
    default_wall_thickness: float = 200.0,
) -> SkeletonDxfData:
    document = active_autocad_document()
    wall_axes = [
        (line, read_thickness_from_com_entity(entity, default_wall_thickness))
        for entity, line in _iter_layer_lines(document, layers.wall)
    ]
    opening_axis_lines = [line for _entity, line in _iter_layer_lines(document, layers.opening)]
    return SkeletonDxfData(wall_axes=wall_axes, opening_lines=opening_axis_lines)


def write_design(
    design_result: StructuralDesignResult | None,
    shear_walls: Sequence[ShearWall] | None,
    beams: Sequence[Beam | LineString] | None,
    layers: DesignLayerNames,
):
    if design_result is not None:
        shear_walls = design_result.shear_walls
        beams = design_result.beams

    document = active_autocad_document()
    _prepare_layers(document, (layers.shear_wall, layers.beam))
    _clear_layers(document, (layers.shear_wall, layers.beam))

    for wall in shear_walls or ():
        for segment in iter_straight_segments(wall.axis):
            _add_line(document, segment, layer=layers.shear_wall, color=1, lineweight=thickness_to_lineweight(wall.thickness))

    for beam in beams or ():
        line = beam.axis if isinstance(beam, Beam) else beam
        for segment in iter_straight_segments(line):
            _add_line(document, segment, layer=layers.beam, color=5, lineweight=70)

    document.Regen(1)


def read_design(
    layers: DesignLayerNames,
    default_shear_wall_thickness: float = 200.0,
) -> DesignDxfData:
    document = active_autocad_document()
    shear_walls = [
        ShearWall(axis=line, thickness=read_thickness_from_com_entity(entity, default_shear_wall_thickness), source="manual_cad")
        for entity, line in _iter_layer_lines(document, layers.shear_wall)
    ]
    beams = [
        Beam(axis=line, kind=BeamKind.PERIMETER, reason="manual_cad")
        for _entity, line in _iter_layer_lines(document, layers.beam)
    ]
    return DesignDxfData(shear_walls=shear_walls, beam_lines=[beam.axis for beam in beams], beams=beams)


def active_autocad_document():
    try:
        import pythoncom
        import win32com.client
    except ImportError as exc:
        raise RuntimeError("AutoCAD COM 读写需要 pywin32。") from exc

    pythoncom.CoInitialize()
    try:
        app = win32com.client.GetActiveObject("AutoCAD.Application")
    except Exception:
        app = win32com.client.Dispatch("AutoCAD.Application")
    if app.Documents.Count == 0:
        raise RuntimeError("当前 AutoCAD 没有打开任何图形。")
    return app.ActiveDocument


def _prepare_layers(document, layer_names: Iterable[str]):
    for layer_name in layer_names:
        if not _layer_exists(document, layer_name):
            document.Layers.Add(layer_name)


def _layer_exists(document, layer_name: str) -> bool:
    try:
        document.Layers.Item(layer_name)
        return True
    except Exception:
        return False


def _clear_layers(document, layer_names: Iterable[str]):
    layer_set = set(layer_names)
    for entity in list(document.ModelSpace):
        try:
            if entity.Layer in layer_set:
                entity.Delete()
        except Exception:
            continue


def _add_line(document, line: LineString, *, layer: str, color: int, lineweight: int):
    start, end = line_endpoints_3d(line)
    entity = document.ModelSpace.AddLine(start, end)
    entity.Layer = layer
    entity.Color = color
    try:
        entity.Lineweight = lineweight
    except Exception:
        pass


def _iter_layer_lines(document, layer_name: str):
    for entity in list(document.ModelSpace):
        try:
            if entity.Layer != layer_name:
                continue
            line = _entity_line(entity)
        except Exception:
            continue
        if line is not None and line.length > 0:
            yield entity, line


def _entity_line(entity) -> LineString | None:
    object_name = str(getattr(entity, "ObjectName", ""))
    if object_name == "AcDbLine":
        return LineString([_xy(entity.StartPoint), _xy(entity.EndPoint)])
    if object_name in {"AcDbPolyline", "AcDb2dPolyline", "AcDb3dPolyline"}:
        points = _polyline_points(entity)
        if len(points) >= 2:
            return LineString(points)
    return None


def _polyline_points(entity) -> list[tuple[float, float]]:
    coords = list(entity.Coordinates)
    step = 3 if len(coords) % 3 == 0 and str(getattr(entity, "ObjectName", "")) == "AcDb3dPolyline" else 2
    points = []
    for index in range(0, len(coords), step):
        if index + 1 < len(coords):
            points.append((float(coords[index]), float(coords[index + 1])))
    return points


def _xy(point) -> tuple[float, float]:
    return (float(point[0]), float(point[1]))
