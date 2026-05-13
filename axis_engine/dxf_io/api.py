from __future__ import annotations

from collections.abc import Iterable, Sequence
from pathlib import Path

from shapely.geometry import LineString

from axis_engine.opening_embedment import OpeningEmbedment
from axis_engine.structural_design.models import Beam, ShearWall, StructuralDesignResult
from axis_engine.dxf_io.models import DesignLayerNames, SkeletonLayerNames
from . import cad_backend, dxf_backend


def write_skeleton_axes(
    wall_axes: Sequence[tuple[LineString, float]],
    opening_embedments_or_lines: Iterable[OpeningEmbedment | LineString],
    *,
    backend: str = "dxf",
    source_path: str | Path | None = None,
    output_path: str | Path | None = None,
    layers: SkeletonLayerNames = SkeletonLayerNames(),
):
    if backend == "dxf":
        if source_path is None:
            raise ValueError("backend='dxf' requires source_path.")
        dxf_backend.write_skeleton(
            source_path=source_path,
            output_path=output_path or source_path,
            wall_axes=wall_axes,
            opening_embedments_or_lines=opening_embedments_or_lines,
            layers=layers,
        )
        return
    if backend == "cad":
        cad_backend.write_skeleton(wall_axes, opening_embedments_or_lines, layers)
        return
    raise ValueError(f"Unsupported IO backend: {backend}")


def read_skeleton_axes(
    *,
    backend: str = "dxf",
    path: str | Path | None = None,
    layers: SkeletonLayerNames = SkeletonLayerNames(),
    default_wall_thickness: float = 200.0,
):
    if backend == "dxf":
        if path is None:
            raise ValueError("backend='dxf' requires path.")
        return dxf_backend.read_skeleton(path, layers, default_wall_thickness)
    if backend == "cad":
        return cad_backend.read_skeleton(layers, default_wall_thickness)
    raise ValueError(f"Unsupported IO backend: {backend}")


def write_design_axes(
    design_result: StructuralDesignResult | None = None,
    *,
    shear_walls: Sequence[ShearWall] | None = None,
    beams: Sequence[Beam | LineString] | None = None,
    backend: str = "dxf",
    source_path: str | Path | None = None,
    output_path: str | Path | None = None,
    layers: DesignLayerNames = DesignLayerNames(),
):
    if backend == "dxf":
        if source_path is None:
            raise ValueError("backend='dxf' requires source_path.")
        dxf_backend.write_design(
            source_path=source_path,
            output_path=output_path or source_path,
            design_result=design_result,
            shear_walls=shear_walls,
            beams=beams,
            layers=layers,
        )
        return
    if backend == "cad":
        cad_backend.write_design(design_result, shear_walls, beams, layers)
        return
    raise ValueError(f"Unsupported IO backend: {backend}")


def read_design_axes(
    *,
    backend: str = "dxf",
    path: str | Path | None = None,
    layers: DesignLayerNames = DesignLayerNames(),
    default_shear_wall_thickness: float = 200.0,
):
    if backend == "dxf":
        if path is None:
            raise ValueError("backend='dxf' requires path.")
        return dxf_backend.read_design(path, layers, default_shear_wall_thickness)
    if backend == "cad":
        return cad_backend.read_design(layers, default_shear_wall_thickness)
    raise ValueError(f"Unsupported IO backend: {backend}")
