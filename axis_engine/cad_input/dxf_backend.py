from __future__ import annotations

from pathlib import Path

from axis_engine.dxf_utils import (
    list_dxf_layer_names,
    read_dxf,
    read_dxf_geometries_from_layers,
)


class DxfInputSource:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.doc = read_dxf(self.path)

    def list_layer_names(self) -> list[str]:
        return list_dxf_layer_names(self.doc)

    def read_geometries_from_layers(
        self,
        layer_names,
        min_length: float = 1.0,
        visible_only: bool = False,
    ):
        return read_dxf_geometries_from_layers(
            self.doc,
            layer_names,
            min_length=min_length,
            visible_only=visible_only,
        )
