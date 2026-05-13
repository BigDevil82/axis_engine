from __future__ import annotations

from collections.abc import Sequence


def pick_wall_layers(layer_names: Sequence[str], explicit_layers: Sequence[str] | None = None) -> list[str]:
    if explicit_layers:
        return list(explicit_layers)

    candidates = []
    for name in layer_names:
        lower_name = name.lower()
        if name.upper() == "AXIS_WALL":
            continue
        if "墙" in name or "wall" in lower_name:
            candidates.append(name)

    if not candidates:
        raise RuntimeError("CAD 输入中未找到包含 '墙' 或 'wall' 的墙体图层，请显式指定 wall layer。")
    return candidates


def pick_axis_layers(layer_names: Sequence[str], explicit_layers: Sequence[str] | None = None) -> list[str]:
    if explicit_layers:
        return list(explicit_layers)

    if "AXIS_WALL" in layer_names:
        return ["AXIS_WALL"]

    return [name for name in layer_names if "轴" in name or "axis" in name.lower()]
