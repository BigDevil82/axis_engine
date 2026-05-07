from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import perf_counter

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.wall_centerline import extract_mixed_thickness_walls
from axis_engine.raw_wall_polygon_builder import build_wall_polygon_from_raw_lines

from cad_tests.autocad_utils import (
    build_wall_polygon_from_lines,
    clear_layer,
    draw_centerlines,
    draw_polygons,
    get_active_document,
)
from cad_tests.dxf_utils import (
    list_dxf_layer_names,
    pick_dxf_wall_layers,
    read_dxf,
    read_dxf_line_segments_from_layers,
)


DEFAULT_DXF_PATH = r"E:\Common\Desktop\test\ai-structures\case3\test.dxf"


def parse_args():
    parser = argparse.ArgumentParser(description="从 DXF 读取墙体图层并将墙体轴线绘制到当前 AutoCAD 文档。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH, help="输入 DXF 文件路径。")
    parser.add_argument(
        "--wall-layer",
        action="append",
        dest="wall_layers",
        help="墙体图层名。可重复传入；不传时自动选择名称包含 '墙' 或 'wall' 的图层。",
    )
    parser.add_argument("--axis-layer", default="AXIS_WALL", help="轴线输出图层名。")
    parser.add_argument("--min-length", type=float, default=1.0, help="忽略短于该长度的图元线段。")
    parser.add_argument("--max-iterations", type=int, default=3, help="混合墙厚剥离提取最大迭代次数。")
    parser.add_argument(
        "--raw-wall",
        action="store_true",
        help="按原始正交线段做墙边配对重构，适用于 WALL 这类非墙体轮廓图层。",
    )
    parser.add_argument(
        "--wall-thickness",
        action="append",
        type=float,
        dest="wall_thicknesses",
        help="原始墙线配对时使用的墙厚，可重复传入；默认 100 和 200。",
    )
    parser.add_argument("--snap-tolerance", type=float, default=3.0, help="原始墙线端点/坐标吸附容差。")
    parser.add_argument("--thickness-tolerance", type=float, default=15.0, help="原始墙线墙厚配对容差。")
    parser.add_argument("--min-overlap", type=float, default=150.0, help="原始墙线平行边最小重叠长度。")
    parser.add_argument("--list-layers", action="store_true", help="仅列出 DXF 图层后退出。")
    parser.add_argument("--no-draw", action="store_true", help="只提取轴线，不写回 AutoCAD。")
    parser.add_argument("--draw-polygons", action="store_true", help="将用于轴线提取的墙体多边形绘制回 AutoCAD。")
    parser.add_argument("--polygon-layer", default="polygon", help="墙体多边形输出图层名。")
    return parser.parse_args()


def main():
    args = parse_args()
    total_start = perf_counter()

    dxf_start = perf_counter()
    doc = read_dxf(args.dxf)
    print(f"DXF: {Path(args.dxf)}")
    print(f"读取 DXF 耗时: {perf_counter() - dxf_start:.2f}s")

    if args.list_layers:
        for layer_name in list_dxf_layer_names(doc):
            print(layer_name)
        return

    wall_layers = pick_dxf_wall_layers(doc, args.wall_layers)
    print(f"墙体图层: {', '.join(wall_layers)}")

    line_start = perf_counter()
    wall_lines = read_dxf_line_segments_from_layers(doc, wall_layers, min_length=args.min_length)
    print(f"读取墙体线段: {len(wall_lines)}")
    print(f"解析墙体线段耗时: {perf_counter() - line_start:.2f}s")
    if not wall_lines:
        raise RuntimeError("DXF 墙体图层中没有可用 LINE/LWPOLYLINE/POLYLINE 图元。")

    polygon_start = perf_counter()
    if args.raw_wall:
        wall_polygon = build_wall_polygon_from_raw_lines(
            wall_lines,
            wall_thicknesses=args.wall_thicknesses or (100.0, 200.0),
            snap_tolerance=args.snap_tolerance,
            thickness_tolerance=args.thickness_tolerance,
            min_overlap=args.min_overlap,
        )
    else:
        wall_polygon = build_wall_polygon_from_lines(wall_lines)
    polygon_count = len(wall_polygon.geoms) if not wall_polygon.is_empty else 0
    print(f"生成闭合墙体区域: {polygon_count}")
    print(f"构建墙体区域耗时: {perf_counter() - polygon_start:.2f}s")
    if wall_polygon.is_empty:
        raise RuntimeError("无法由墙体图元 polygonize 出闭合墙体区域，请检查墙线是否闭合。")

    axis_start = perf_counter()
    centerlines = extract_mixed_thickness_walls(wall_polygon, max_iterations=args.max_iterations)
    print(f"提取墙体轴线: {len(centerlines)}")
    print(f"提取轴线耗时: {perf_counter() - axis_start:.2f}s")

    if not args.no_draw:
        draw_start = perf_counter()
        acad_doc = get_active_document()
        if args.draw_polygons:
            deleted_polygon_count = clear_layer(acad_doc, args.polygon_layer)
            drawn_polygon_count = draw_polygons(acad_doc, wall_polygon, layer_name=args.polygon_layer)
            print(f"清理旧多边形: {deleted_polygon_count}")
            print(f"绘制墙体多边形到 {args.polygon_layer}: {drawn_polygon_count}")
        deleted_count = clear_layer(acad_doc, args.axis_layer)
        drawn_count = draw_centerlines(acad_doc, centerlines, layer_name=args.axis_layer)
        print(f"清理旧轴线: {deleted_count}")
        print(f"绘制新轴线到 {args.axis_layer}: {drawn_count}")
        print(f"写回 AutoCAD 耗时: {perf_counter() - draw_start:.2f}s")

    print(f"总耗时: {perf_counter() - total_start:.2f}s")


if __name__ == "__main__":
    main()
