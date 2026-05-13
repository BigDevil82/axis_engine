from __future__ import annotations

from collections.abc import Iterable, Sequence
from pathlib import Path

import ezdxf
from shapely.geometry import LineString

from axis_engine.geometry_utils import iter_straight_segments
from axis_engine.opening_embedment import OpeningEmbedment
from axis_engine.structural_design.models import Beam, BeamKind, ShearWall, StructuralDesignResult
from axis_engine.dxf_io.common import APPID, opening_lines, read_thickness_from_entity, thickness_to_lineweight
from axis_engine.dxf_io.models import DesignDxfData, DesignLayerNames, SkeletonDxfData, SkeletonLayerNames


def write_skeleton(
    source_path: str | Path,
    output_path: str | Path,
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments_or_lines: Iterable[OpeningEmbedment | LineString],
    layers: SkeletonLayerNames,
):
    doc = _read_or_create_doc(source_path)
    _prepare_doc(doc, (layers.wall, layers.opening))
    _clear_layers(doc, (layers.wall, layers.opening))

    for line, thickness in wall_axes:
        for segment in iter_straight_segments(line):
            _add_line(doc, segment, layer=layers.wall, color=1, lineweight=thickness_to_lineweight(thickness))

    for line in opening_lines(opening_embedments_or_lines):
        for segment in iter_straight_segments(line):
            _add_line(doc, segment, layer=layers.opening, color=3, lineweight=50)

    doc.saveas(Path(output_path))


def read_skeleton(
    path: str | Path,
    layers: SkeletonLayerNames,
    default_wall_thickness: float = 200.0,
) -> SkeletonDxfData:
    doc = ezdxf.readfile(Path(path))
    wall_axes = [
        (line, read_thickness_from_entity(entity, default_wall_thickness))
        for entity, line in _iter_layer_lines(doc, layers.wall)
    ]
    opening_axis_lines = [line for _entity, line in _iter_layer_lines(doc, layers.opening)]
    return SkeletonDxfData(wall_axes=wall_axes, opening_lines=opening_axis_lines)


def write_design(
    source_path: str | Path,
    output_path: str | Path,
    design_result: StructuralDesignResult | None,
    shear_walls: Sequence[ShearWall] | None,
    beams: Sequence[Beam | LineString] | None,
    layers: DesignLayerNames,
):
    if design_result is not None:
        shear_walls = design_result.shear_walls
        beams = design_result.beams

    doc = _read_or_create_doc(source_path)
    _prepare_doc(doc, (layers.shear_wall, layers.beam))
    _clear_layers(doc, (layers.shear_wall, layers.beam))

    for wall in shear_walls or ():
        for segment in iter_straight_segments(wall.axis):
            _add_line(
                doc,
                segment,
                layer=layers.shear_wall,
                color=1,
                lineweight=thickness_to_lineweight(wall.thickness),
            )

    for beam in beams or ():
        line = beam.axis if isinstance(beam, Beam) else beam
        for segment in iter_straight_segments(line):
            _add_line(doc, segment, layer=layers.beam, color=5, lineweight=70)

    doc.saveas(Path(output_path))


def read_design(
    path: str | Path,
    layers: DesignLayerNames,
    default_shear_wall_thickness: float = 200.0,
) -> DesignDxfData:
    doc = ezdxf.readfile(Path(path))
    shear_walls = [
        ShearWall(axis=line, thickness=read_thickness_from_entity(entity, default_shear_wall_thickness), source="manual_dxf")
        for entity, line in _iter_layer_lines(doc, layers.shear_wall)
    ]
    beams = [
        Beam(axis=line, kind=_read_beam_kind(entity), reason="manual_dxf")
        for entity, line in _iter_layer_lines(doc, layers.beam)
    ]
    return DesignDxfData(shear_walls=shear_walls, beam_lines=[beam.axis for beam in beams], beams=beams)


def _read_or_create_doc(path: str | Path):
    path = Path(path)
    if path.exists():
        return ezdxf.readfile(path)
    return ezdxf.new("R2018")


def _prepare_doc(doc, layer_names: Iterable[str]):
    _repair_invalid_block_names(doc)
    _discard_axis_engine_xdata(doc)
    for layer_name in layer_names:
        if not doc.layers.has_entry(layer_name):
            doc.layers.add(layer_name)


def _repair_invalid_block_names(doc):
    repaired_names: dict[str, str] = {}
    for block_record in doc.block_records:
        if _valid_symbol_name(block_record.dxf.get("name", "")):
            continue
        name = f"_AE_REPAIRED_BLOCK_{block_record.dxf.handle}"
        block_record.dxf.name = name
        repaired_names[block_record.dxf.handle] = name

    for entity in doc.entitydb.values():
        if entity.dxftype() != "BLOCK" or _valid_symbol_name(entity.dxf.get("name", "")):
            continue
        owner = entity.dxf.get("owner", "")
        entity.dxf.name = repaired_names.get(owner, f"_AE_REPAIRED_BLOCK_{entity.dxf.handle}")


def _valid_symbol_name(name: str | None) -> bool:
    return bool(name and str(name).strip())


def _clear_layers(doc, layer_names: Iterable[str]):
    layer_set = set(layer_names)
    modelspace = doc.modelspace()
    for entity in list(modelspace):
        if entity.dxf.layer in layer_set:
            modelspace.delete_entity(entity)


def _discard_axis_engine_xdata(doc):
    for entity in doc.modelspace():
        try:
            entity.discard_xdata(APPID)
        except Exception:
            continue
    try:
        doc.appids.discard(APPID)
    except Exception:
        pass


def _add_line(doc, line: LineString, *, layer: str, color: int, lineweight: int):
    coords = list(line.coords)
    if len(coords) < 2:
        return
    start = coords[0]
    end = coords[-1]
    doc.modelspace().add_line(
        (float(start[0]), float(start[1])),
        (float(end[0]), float(end[1])),
        dxfattribs={"layer": layer, "color": color, "lineweight": lineweight},
    )


def _iter_layer_lines(doc, layer_name: str):
    for entity in doc.modelspace():
        if entity.dxf.layer != layer_name:
            continue
        entity_type = entity.dxftype()
        if entity_type == "LINE":
            yield entity, LineString([_xy(entity.dxf.start), _xy(entity.dxf.end)])
        elif entity_type in {"LWPOLYLINE", "POLYLINE"}:
            for line in _polyline_segments(entity):
                yield entity, line


def _polyline_segments(entity) -> Iterable[LineString]:
    if entity.dxftype() == "LWPOLYLINE":
        points = [(float(x), float(y)) for x, y, *_rest in entity.get_points("xy")]
        closed = bool(entity.closed)
    else:
        points = [_xy(vertex.dxf.location) for vertex in entity.vertices]
        closed = bool(entity.is_closed)

    if closed and points:
        points = points + [points[0]]
    for start, end in zip(points, points[1:]):
        line = LineString([start, end])
        if line.length > 0:
            yield line


def _read_beam_kind(entity) -> BeamKind:
    try:
        xdata = entity.get_xdata(APPID)
    except Exception:
        xdata = ()
    for code, value in xdata:
        if code != 1000:
            continue
        try:
            return BeamKind(str(value))
        except ValueError:
            continue
    return BeamKind.PERIMETER


def _xy(point) -> tuple[float, float]:
    return (float(point[0]), float(point[1]))
