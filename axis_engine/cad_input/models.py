from __future__ import annotations

from typing import Protocol

from axis_engine.dxf_utils import DxfGeometry


class CadInputSource(Protocol):
    def list_layer_names(self) -> list[str]:
        ...

    def read_geometries_from_layers(
        self,
        layer_names,
        min_length: float = 1.0,
        visible_only: bool = False,
    ) -> list[DxfGeometry]:
        ...
