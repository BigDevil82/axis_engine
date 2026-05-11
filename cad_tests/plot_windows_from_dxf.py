from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.patches as patches

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_processor import CADLayoutProcessor
from axis_engine.opening_clustering import cluster_bounds
from axis_engine.opening_embedment import unmatched_opening_indices
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
    parser.add_argument("--opening-layer", action="append", dest="opening_layers", default=["WINDOW"])
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    dxf_path = ensure_dxf_exists(args.dxf)
    processor = CADLayoutProcessor(dxf_path, opening_layers=args.opening_layers)
    artifacts = processor.build_geometry()
    unmatched = unmatched_opening_indices(len(artifacts.opening_clusters), artifacts.opening_embedments)

    print(f"门窗图层: {', '.join(args.opening_layers)}")
    print(f"门窗原始图元: {len(artifacts.opening_geometries)}")
    print(f"门窗聚类组: {len(artifacts.opening_clusters)}")
    print(f"门窗嵌入线: {len(artifacts.opening_embedments)}")
    print(f"未匹配门窗: {len(unmatched)}")

    plot_result(
        artifacts.wall_axes,
        artifacts.opening_clusters,
        artifacts.opening_embedments,
        Path(args.output),
        show=args.show,
    )
    print(f"输出图片: {args.output}")


def plot_result(wall_axes, opening_clusters, opening_embedments, output_path: Path, show: bool = False):
    fig, ax = create_axes((14, 10))

    for line, _thickness in wall_axes:
        x, y = line.xy
        ax.plot(x, y, color="#b0bec5", linewidth=1.0, zorder=1)

    for idx, cluster in enumerate(opening_clusters, start=1):
        minx, miny, maxx, maxy = cluster_bounds(cluster.lines)
        rect = patches.Rectangle(
            (minx, miny),
            maxx - minx,
            maxy - miny,
            fill=False,
            edgecolor="#8e24aa",
            linewidth=1.0,
            linestyle="--",
            zorder=5,
        )
        ax.add_patch(rect)
        ax.text(minx, maxy, str(idx), fontsize=6, color="#8e24aa")

    color_map = {"door": "#00acc1", "window": "#43a047", "balcony": "#f57c00"}
    seen_labels = set()
    for embedment in opening_embedments:
        color = color_map.get(embedment.opening_type, "#00acc1")
        label = None
        if embedment.opening_type not in seen_labels:
            label = f"{embedment.opening_type} embedment"
            seen_labels.add(embedment.opening_type)
        for line in iter_lines(embedment.embed_line):
            x, y = line.xy
            ax.plot(x, y, color=color, linewidth=2.2, zorder=8, label=label)
            label = None

    ax.set_title("Opening Clusters And Embedments")
    if seen_labels:
        ax.legend(loc="upper right")
    save_and_maybe_show(fig, output_path, show=show)


if __name__ == "__main__":
    main()

