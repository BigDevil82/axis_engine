from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Counter

import matplotlib.patches as patches
from shapely.geometry import MultiLineString
from shapely.ops import linemerge, unary_union

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_processor import CADLayoutProcessor
from axis_engine.dxf_io import read_skeleton_axes, write_skeleton_axes
from axis_engine.opening_clustering import cluster_bounds
from axis_engine.opening_embedment import OpeningEmbedment, unmatched_opening_indices
from axis_engine.geometry_utils import iter_lines
from cad_tests.cli_common import (
    DEFAULT_DXF_PATH,
    DEFAULT_OPENINGS_OUTPUT_PATH,
    ensure_dxf_exists,
)
from cad_tests.plot_common import create_axes, save_and_maybe_show

DEFAULT_OUTPUT_PATH = DEFAULT_OPENINGS_OUTPUT_PATH


def parse_args():
    parser = argparse.ArgumentParser(description="门窗聚类与嵌入线提取最小验证。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH)
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--wall-layer", action="append", dest="wall_layers")
    parser.add_argument("--axis-layer", action="append", dest="axis_layers")
    parser.add_argument("--opening-layer", action="append", dest="opening_layers", default=["WINDOW"])
    parser.add_argument("--write", action="store_true", help="提取骨架后写入编辑图层，并展示写入结果。")
    parser.add_argument("--read", action="store_true", help="从编辑图层读取骨架并展示。")
    parser.add_argument("--backend", choices=("dxf", "cad"), default="dxf", help="读写后端：dxf 操作 --dxf 文件，cad 操作当前 AutoCAD 图形。")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.write and args.read:
        raise ValueError("--write 和 --read 不能同时使用。")

    if args.read:
        dxf_path = ensure_dxf_exists(args.dxf) if args.backend == "dxf" else None
        data = read_skeleton_axes(backend=args.backend, path=dxf_path)
        print_skeleton_summary(data.wall_axes, data.opening_lines)
        plot_result(
            [],
            data.wall_axes,
            [],
            [],
            data.opening_lines,
            Path(args.output),
            show=args.show,
        )
        print(f"从 {'当前 AutoCAD 图形' if args.backend == 'cad' else dxf_path} 读取编辑图层。")
        print(f"输出图片: {args.output}")
        return

    dxf_path = ensure_dxf_exists(args.dxf)
    processor = CADLayoutProcessor(
        dxf_path,
        wall_layers=args.wall_layers,
        axis_layers=args.axis_layers,
        opening_layers=args.opening_layers,
    )
    artifacts = processor.build_geometry()

    if args.write:
        if args.backend == "cad":
            write_skeleton_axes(
                artifacts.wall_axes,
                artifacts.opening_embedments,
                backend="cad",
            )
            print_skeleton_summary(artifacts.wall_axes, [item.embed_line for item in artifacts.opening_embedments])
            print("已写入当前 AutoCAD 图形。")
            plot_wall_axes = artifacts.wall_axes
            plot_openings = artifacts.opening_embedments
        else:
            write_skeleton_axes(
                artifacts.wall_axes,
                artifacts.opening_embedments,
                backend="dxf",
                source_path=dxf_path,
            )
            data = read_skeleton_axes(backend="dxf", path=dxf_path)
            print_skeleton_summary(data.wall_axes, data.opening_lines)
            print(f"已写入 DXF 编辑图层: {dxf_path}")
            plot_wall_axes = data.wall_axes
            plot_openings = data.opening_lines
        plot_result(
            artifacts.axis_linework,
            plot_wall_axes,
            [],
            [],
            plot_openings,
            Path(args.output),
            show=args.show,
        )
        print(f"输出图片: {args.output}")
        return

    print_extraction_summary(args, artifacts)

    plot_result(
        artifacts.axis_linework,
        artifacts.wall_axes,
        artifacts.opening_geometries,
        artifacts.opening_clusters,
        artifacts.opening_embedments,
        Path(args.output),
        show=args.show,
    )
    print(f"输出图片: {args.output}")


def print_extraction_summary(args, artifacts):
    thickness_lengths = {}
    for line, thickness in artifacts.wall_axes:
        thickness_lengths[thickness] = thickness_lengths.get(thickness, 0.0) + line.length
    unmatched = unmatched_opening_indices(len(artifacts.opening_clusters), artifacts.opening_embedments)

    if thickness_lengths:
        summary = ", ".join(
            f"{thick:g}: {length:.1f}"
            for thick, length in sorted(thickness_lengths.items(), key=lambda x: x[1], reverse=True)
        )
        print(f"轴线墙厚长度分布: {summary}")
    print(f"门窗图层: {', '.join(args.opening_layers)}")
    print(f"参考轴线图元: {len(artifacts.axis_geometries)}")
    print(f"门窗原始图元: {len(artifacts.opening_geometries)}")
    print(f"门窗聚类组: {len(artifacts.opening_clusters)}")
    print(f"门窗嵌入线: {len(artifacts.opening_embedments)}")
    print(f"未匹配门窗: {len(unmatched)}")


def print_skeleton_summary(wall_axes, opening_lines):
    thickness_lengths = {}
    for line, thickness in wall_axes:
        thickness_lengths[thickness] = thickness_lengths.get(thickness, 0.0) + line.length
    if thickness_lengths:
        summary = ", ".join(
            f"{thick:g}: {length:.1f}"
            for thick, length in sorted(thickness_lengths.items(), key=lambda x: x[1], reverse=True)
        )
        print(f"轴线墙厚长度分布: {summary}")
    print(f"墙轴线: {len(wall_axes)}")
    print(f"门窗嵌入线: {len(opening_lines)}")


def plot_result(
    axis_linework,
    wall_axes,
    opening_geometries,
    opening_clusters,
    opening_embedments,
    output_path: Path,
    show: bool = False,
):
    fig, ax = create_axes((14, 10))

    buffered_network = build_buffered_network(wall_axes, opening_embedments, 150)
    if not buffered_network.is_empty:
        geoms = list(buffered_network.geoms) if hasattr(buffered_network, "geoms") else [buffered_network]
        for polygon in geoms:
            print("buffered network polygon:")
            # x, y = polygon.exterior.xy
            # ax.fill(x, y, color="gray", alpha=0.45, linewidth=0, zorder=-2, label=None)
            import random

            for interior in polygon.interiors:
                x, y = interior.xy
                ax.fill(
                    x,
                    y,
                    color=f"#{random.randint(0, 0xFFFFFF):06x}",
                    alpha=0.4,
                    linewidth=0,
                    zorder=-1,
                    label=None,
                )

    axis_label_added = False
    for line in axis_linework:
        x, y = line.xy
        ax.plot(
            x,
            y,
            color="#1565c0",
            linewidth=0.8,
            alpha=0.75,
            linestyle="--",
            zorder=0,
            dash_capstyle="butt",
            dash_joinstyle="miter",
            label="reference axis" if not axis_label_added else None,
        )
        axis_label_added = True

    primary_thickness = dominant_wall_thickness(wall_axes)
    wall_labels = set()
    for line, thickness in wall_axes:
        x, y = line.xy
        is_primary = thickness == primary_thickness
        label = "primary wall axis" if is_primary else "secondary wall axis"
        ax.plot(
            x,
            y,
            color="red" if is_primary else "#7e57c2",
            linewidth=2.0 if is_primary else 1.5,
            zorder=1 if is_primary else 2,
            solid_capstyle="butt",
            solid_joinstyle="miter",
            label=label if label not in wall_labels else None,
        )
        wall_labels.add(label)

    raw_style = {
        "LINE": {"color": "#455a64", "linewidth": 0.45, "alpha": 0.7, "label": "raw line"},
        "ARC": {"color": "#ef6c00", "linewidth": 0.8, "alpha": 0.9, "label": "raw arc"},
        "CIRCLE": {"color": "#1e88e5", "linewidth": 0.7, "alpha": 0.75, "label": "raw circle"},
        "ELLIPSE": {"color": "#5e35b1", "linewidth": 0.7, "alpha": 0.75, "label": "raw ellipse"},
    }
    raw_labels = set()
    # for geometry in opening_geometries:
    #     style = raw_style.get(
    #         geometry.geom_type,
    #         {"color": "#607d8b", "linewidth": 0.45, "alpha": 0.65, "label": "raw geometry"},
    #     )
    #     label = style["label"] if style["label"] not in raw_labels else None
    #     raw_labels.add(style["label"])
    #     for line in geometry.as_linework(curve_tolerance=8.0):
    #         x, y = line.xy
    #         ax.plot(
    #             x,
    #             y,
    #             color=style["color"],
    #             linewidth=style["linewidth"],
    #             alpha=style["alpha"],
    #             zorder=3,
    #             label=label,
    #         )
    #         label = None

    # for idx, cluster in enumerate(opening_clusters, start=1):
    #     minx, miny, maxx, maxy = cluster_bounds(cluster.lines)
    #     rect = patches.Rectangle(
    #         (minx, miny),
    #         maxx - minx,
    #         maxy - miny,
    #         fill=False,
    #         edgecolor="#8e24aa",
    #         linewidth=1.0,
    #         linestyle="--",
    #         zorder=5,
    #     )
    #     ax.add_patch(rect)
    #     ax.text(minx, maxy, str(idx), fontsize=6, color="#8e24aa")

    color_map = {"door": "#00acc1", "window": "#43a047", "balcony": "#f57c00"}
    seen_labels = set()
    for embedment in opening_embedments:
        opening_type = embedment.opening_type if isinstance(embedment, OpeningEmbedment) else "opening"
        color = color_map.get(opening_type, "#00acc1")
        label = None
        if opening_type not in seen_labels:
            label = f"{opening_type} embedment"
            seen_labels.add(opening_type)
        geometry = embedment.embed_line if isinstance(embedment, OpeningEmbedment) else embedment
        for line in iter_lines(geometry):
            x, y = line.xy
            ax.plot(
                x,
                y,
                color=color,
                linewidth=2.2,
                zorder=8,
                solid_capstyle="butt",
                solid_joinstyle="miter",
                label=label,
            )
            label = None

    ax.set_title("Opening Clusters And Embedments")
    if seen_labels or raw_labels or axis_label_added or wall_labels:
        ax.legend(loc="upper right")
    save_and_maybe_show(fig, output_path, show=show)


def build_buffered_network(wall_axes, opening_embedments, buffer_distance: float = 100.0):
    lines = [line for line, _thickness in wall_axes if line.length > 0]
    for embedment in opening_embedments:
        geometry = embedment.embed_line if isinstance(embedment, OpeningEmbedment) else embedment
        lines.extend(line for line in iter_lines(geometry) if line.length > 0)

    if not lines:
        return MultiLineString().buffer(0)

    network = linemerge(unary_union(lines))
    return network.buffer(buffer_distance, cap_style="square", join_style="mitre")


def dominant_wall_thickness(wall_axes) -> float | None:
    thickness_lengths = {}
    for line, thickness in wall_axes:
        thickness_lengths[thickness] = thickness_lengths.get(thickness, 0.0) + line.length
    if not thickness_lengths:
        return None
    return max(thickness_lengths.items(), key=lambda item: item[1])[0]


if __name__ == "__main__":
    main()
