from __future__ import annotations

from pathlib import Path

from axis_engine.cad_input.cad_backend import ActiveCadInputSource
from axis_engine.cad_input.dxf_backend import DxfInputSource


def open_cad_source(backend: str = "dxf", path: str | Path | None = None):
    if backend == "dxf":
        if path is None:
            raise ValueError("backend='dxf' requires path.")
        return DxfInputSource(path)
    if backend == "cad":
        return ActiveCadInputSource()
    raise ValueError(f"Unsupported CAD input backend: {backend}")
