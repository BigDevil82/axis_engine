from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.wall_centerline import extract_mixed_thickness_walls

from cad_tests.autocad_utils import (
    build_wall_polygon_from_lines,
    clear_layer,
    document_name,
    draw_centerlines,
    get_active_document,
    list_layer_names,
    pick_wall_layers,
    read_line_segments_from_layers,
)


def parse_args():
    parser = argparse.ArgumentParser(description="从当前 AutoCAD 活动文档读取墙体图层并绘制墙体轴线。")
    parser.add_argument(
        "--wall-layer",
        action="append",
        dest="wall_layers",
        help="墙体图层名。可重复传入两次；不传时自动选择名称包含 '墙' 或 'wall' 的图层。",
    )
    parser.add_argument("--axis-layer", default="AXIS_WALL", help="轴线输出图层名。")
    parser.add_argument("--min-length", type=float, default=1.0, help="忽略短于该长度的图元线段。")
    parser.add_argument("--max-iterations", type=int, default=3, help="混合墙厚剥离提取最大迭代次数。")
    parser.add_argument("--list-layers", action="store_true", help="仅列出当前文档图层后退出。")
    return parser.parse_args()


def main():
    args = parse_args()
    doc = get_active_document()

    print(f"当前文档: {document_name(doc)}")

    if args.list_layers:
        for layer_name in list_layer_names(doc):
            print(layer_name)
        return

    wall_layers = pick_wall_layers(doc, args.wall_layers)
    print(f"墙体图层: {', '.join(wall_layers)}")

    wall_lines = read_line_segments_from_layers(doc, wall_layers, min_length=args.min_length)
    print(f"读取墙体线段: {len(wall_lines)}")
    if not wall_lines:
        raise RuntimeError("墙体图层中没有可用 LINE/LWPOLYLINE 图元。")

    wall_polygon = build_wall_polygon_from_lines(wall_lines)
    polygon_count = len(wall_polygon.geoms) if not wall_polygon.is_empty else 0
    print(f"生成闭合墙体区域: {polygon_count}")
    if wall_polygon.is_empty:
        raise RuntimeError("无法由墙体图元 polygonize 出闭合墙体区域，请检查墙线是否闭合。")

    centerlines = extract_mixed_thickness_walls(wall_polygon, max_iterations=args.max_iterations)
    print(f"提取墙体轴线: {len(centerlines)}")

    deleted_count = clear_layer(doc, args.axis_layer)
    drawn_count = draw_centerlines(doc, centerlines, layer_name=args.axis_layer)
    print(f"清理旧轴线: {deleted_count}")
    print(f"绘制新轴线到 {args.axis_layer}: {drawn_count}")


if __name__ == "__main__":
    main()
