from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path
from time import perf_counter

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from shapely.geometry import MultiLineString

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.linework_axis_extractor import (
    extract_wall_axes_from_linework,
    infer_wall_thicknesses,
    repair_wall_linework,
)
from axis_engine.raw_wall_polygon_builder import build_wall_polygon_from_raw_lines

from cad_tests.dxf_utils import (
    pick_dxf_wall_layers,
    read_dxf,
    read_dxf_line_segment_groups_from_layers,
    read_dxf_line_segments_from_layers,
)
from cad_tests.plot_windows_from_dxf import _cluster_bounds, cluster_line_segments

DEFAULT_DXF_PATH = r"E:\Common\Desktop\test\ai-structures\case3\test.dxf"
DEFAULT_OUTPUT_PATH = r"E:\Common\Desktop\test\ai-structures\case3\wall_axes_linework.png"


def parse_args():
    parser = argparse.ArgumentParser(description="从 DXF 原始墙线直接提取墙体轴线，并用 matplotlib 可视化。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH, help="输入 DXF 文件路径。")
    parser.add_argument(
        "--wall-layer",
        action="append",
        dest="wall_layers",
        help="墙体图层名。可重复传入；不传时自动选择名称包含 '墙' 或 'wall' 的图层。",
    )
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH, help="输出 PNG 路径。")
    parser.add_argument(
        "--wall-thickness",
        action="append",
        type=float,
        dest="wall_thicknesses",
        help="要尝试的墙厚，可重复传入；不传时从线网自动推断多个候选墙厚。",
    )
    parser.add_argument("--snap-tolerance", type=float, default=3.0)
    parser.add_argument("--axis-tolerance", type=float, default=5.0)
    parser.add_argument("--thickness-tolerance-ratio", type=float, default=0.10)
    parser.add_argument("--thickness-tolerance-abs", type=float, default=12.0)
    parser.add_argument("--min-overlap", type=float, default=30.0)
    parser.add_argument("--min-feature-len", type=float, default=20.0)
    parser.add_argument("--min-score", type=float, default=1.35)
    parser.add_argument("--alignment-tolerance", type=float, default=80.0)
    parser.add_argument("--connection-tolerance", type=float, default=160.0)
    parser.add_argument("--opening-layer", action="append", dest="opening_layers")
    parser.add_argument("--opening-cluster-distance", type=float, default=120.0)
    parser.add_argument("--opening-min-cluster-lines", type=int, default=2)
    parser.add_argument("--opening-bbox-padding", type=float, default=0.0)
    parser.add_argument("--opening-max-bbox-size", type=float, default=5000.0)
    parser.add_argument("--show-polygons", action="store_true", help="额外绘制原始线段重构的辅助多边形。")
    parser.add_argument("--show", action="store_true", help="保存后弹出 matplotlib 窗口。")
    return parser.parse_args()


def main():
    args = parse_args()
    start = perf_counter()

    doc = read_dxf(args.dxf)
    wall_layers = pick_dxf_wall_layers(doc, args.wall_layers)
    wall_lines = read_dxf_line_segments_from_layers(doc, wall_layers)
    print(f"DXF: {Path(args.dxf)}")
    print(f"墙体图层: {', '.join(wall_layers)}")
    print(f"原始墙体线段: {len(wall_lines)}")

    repaired_edges = repair_wall_linework(
        wall_lines,
        axis_tolerance=args.axis_tolerance,
        snap_tolerance=args.snap_tolerance,
        min_segment_length=args.min_feature_len,
    )
    thicknesses = args.wall_thicknesses or infer_wall_thicknesses(
        repaired_edges, min_overlap=max(80.0, args.min_overlap)
    )
    print(f"归一化正交边: {len(repaired_edges)}")
    print(f"墙厚候选: {', '.join(f'{item:g}' for item in thicknesses)}")

    axes = extract_wall_axes_from_linework(
        wall_lines,
        thickness_candidates=thicknesses,
        axis_tolerance=args.axis_tolerance,
        snap_tolerance=args.snap_tolerance,
        thickness_tolerance_ratio=args.thickness_tolerance_ratio,
        thickness_tolerance_abs=args.thickness_tolerance_abs,
        min_overlap=args.min_overlap,
        min_feature_len=args.min_feature_len,
        min_score=args.min_score,
        alignment_tolerance=args.alignment_tolerance,
        connection_tolerance=args.connection_tolerance,
    )
    print(f"提取轴线: {len(axes)}")
    thickness_counts = Counter(thickness for _line, thickness in axes)
    if thickness_counts:
        summary = ", ".join(
            f"{thickness:g}: {count}" for thickness, count in sorted(thickness_counts.items())
        )
        print(f"轴线墙厚分布: {summary}")

    opening_lines = []
    opening_bboxes = []
    if args.opening_layers:
        opening_lines = read_dxf_line_segments_from_layers(doc, args.opening_layers)
        opening_clusters = cluster_line_segments(
            opening_lines,
            distance=args.opening_cluster_distance,
            min_lines=args.opening_min_cluster_lines,
            max_bbox_size=args.opening_max_bbox_size,
        )
        opening_bboxes = [_cluster_bounds(cluster, args.opening_bbox_padding) for cluster in opening_clusters]
        print(f"门窗图层: {', '.join(args.opening_layers)}")
        print(f"门窗线段: {len(opening_lines)}")
        print(f"门窗包围盒: {len(opening_bboxes)}")

    wall_polygon = None
    if args.show_polygons:
        wall_polygon = build_wall_polygon_from_raw_lines(
            wall_lines,
            wall_thicknesses=thicknesses,
            snap_tolerance=args.snap_tolerance,
            min_overlap=max(100.0, args.min_overlap),
        )
        polygon_count = len(wall_polygon.geoms) if not wall_polygon.is_empty else 0
        print(f"辅助墙体多边形: {polygon_count}")

    plot_result(
        wall_lines,
        axes,
        wall_polygon,
        Path(args.output),
        show=args.show,
        opening_lines=opening_lines,
        opening_bboxes=opening_bboxes,
    )
    print(f"输出图片: {Path(args.output)}")
    print(f"总耗时: {perf_counter() - start:.2f}s")


def plot_result(
    wall_lines,
    axes,
    wall_polygon,
    output_path: Path,
    show: bool = False,
    opening_lines=None,
    opening_bboxes=None,
):
    fig, ax = plt.subplots(figsize=(16, 10))

    if wall_polygon is not None and not wall_polygon.is_empty:
        for polygon in wall_polygon.geoms:
            x, y = polygon.exterior.xy
            ax.fill(x, y, color="#d9d9d9", alpha=0.35, zorder=1)
            ax.plot(x, y, color="#808080", linewidth=1, zorder=2)
            for interior in polygon.interiors:
                ix, iy = interior.xy
                ax.plot(ix, iy, color="#504747", linewidth=0.4, zorder=2)

    for line in wall_lines:
        x, y = line.xy
        ax.plot(x, y, color="#9aa0a6", linewidth=0.5, alpha=1, zorder=3)

    colors = {
        50.0: "#7b1fa2",
        100.0: "#1976d2",
        150.0: "#00796b",
        200.0: "#d32f2f",
        250.0: "#f57c00",
        300.0: "#455a64",
    }
    wall_label_added = False
    for line, thickness in axes:
        x, y = line.xy
        color = colors.get(float(thickness), "#d32f2f")
        label = "wall axis" if not wall_label_added else None
        ax.plot(x, y, color=color, linewidth=1.6, zorder=10, label=label)
        wall_label_added = True

    opening_line_label_added = False
    for line in opening_lines or []:
        x, y = line.xy
        label = "door/window linework" if not opening_line_label_added else None
        ax.plot(x, y, color="#2e7d32", linewidth=0.9, alpha=0.9, zorder=11, label=label)
        opening_line_label_added = True

    bbox_label_added = False
    for minx, miny, maxx, maxy in opening_bboxes or []:
        label = "door/window bbox" if not bbox_label_added else None
        rect = patches.Rectangle(
            (minx, miny),
            maxx - minx,
            maxy - miny,
            fill=False,
            edgecolor="#ff00ff",
            linewidth=1.0,
            linestyle="--",
            alpha=0.9,
            zorder=12,
            label=label,
        )
        ax.add_patch(rect)
        bbox_label_added = True

    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Wall Axes With Door/Window Linework And Bounding Boxes")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.2)
    ax.tick_params(labelsize=7)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220)
    if show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
