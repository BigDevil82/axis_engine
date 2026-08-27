"""Export raw DXF primitives into the payload accepted by skeleton extraction."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_input import open_cad_source
from axis_engine.dxf_utils import DxfGeometry


DEFAULT_DXF = PROJECT_ROOT / "tmp" / "demo.dxf"
DEFAULT_OUTPUT = Path(__file__).with_name("demo_input.json")


def parse_args():
    parser = argparse.ArgumentParser(description="导出建筑图纸 API 原始输入。")
    parser.add_argument("--dxf", type=Path, default=DEFAULT_DXF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--wall-layer", action="append", dest="wall_layers", default=["WALL"])
    parser.add_argument("--opening-layer", action="append", dest="opening_layers", default=["WINDOW"])
    parser.add_argument("--axis-layer", action="append", dest="axis_layers", default=["DOTE"])
    return parser.parse_args()


def main():
    args = parse_args()
    source = open_cad_source("dxf", args.dxf)
    payload = {
        "wall_geometries": _read(source, args.wall_layers),
        "opening_geometries": _read(source, args.opening_layers),
        "axis_geometries": _read(source, args.axis_layers),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"墙体图元: {len(payload['wall_geometries'])}")
    print(f"门窗图元: {len(payload['opening_geometries'])}")
    print(f"轴网图元: {len(payload['axis_geometries'])}")
    print(f"输入 JSON: {args.output}")


def _read(source, layers: list[str]) -> list[dict]:
    return [_geometry_payload(geometry) for geometry in source.read_geometries_from_layers(layers)]


def _geometry_payload(geometry: DxfGeometry) -> dict:
    return {
        "layer": geometry.layer,
        "geom_type": geometry.geom_type,
        "params": geometry.params,
        "block_path": list(geometry.block_path),
    }


if __name__ == "__main__":
    main()
