from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_processor import CADLayoutProcessor
from cad_tests.cli_common import (
    DEFAULT_DXF_PATH,
    DEFAULT_WALL_AXES_OUTPUT_PATH,
    ensure_dxf_exists,
)
from cad_tests.plot_common import create_axes, save_and_maybe_show

DEFAULT_OUTPUT_PATH = DEFAULT_WALL_AXES_OUTPUT_PATH


def parse_args():
    parser = argparse.ArgumentParser(description="DXF 墙轴线提取与最小可视化验证。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH)
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--wall-layer", action="append", dest="wall_layers")
    parser.add_argument("--opening-layer", action="append", dest="opening_layers")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    dxf_path = ensure_dxf_exists(args.dxf)
    processor = CADLayoutProcessor(
        dxf_path,
        wall_layers=args.wall_layers,
        opening_layers=args.opening_layers,
    )
    artifacts = processor.build_geometry()
    thickness_counts = Counter(thickness for _line, thickness in artifacts.wall_axes)
    print(f"墙体原始图元: {len(artifacts.wall_geometries)}")
    print(f"墙体线性工作视图: {len(artifacts.wall_linework)}")
    print(f"提取墙轴线: {len(artifacts.wall_axes)}")
    if thickness_counts:
        summary = ", ".join(f"{thick:g}: {count}" for thick, count in sorted(thickness_counts.items()))
        print(f"轴线墙厚分布: {summary}")
    print(f"门窗组: {len(artifacts.opening_clusters)}")
    print(f"门窗嵌入线: {len(artifacts.opening_embedments)}")
    plot_result(artifacts.wall_linework, artifacts.wall_axes, Path(args.output), show=args.show)
    print(f"输出图片: {args.output}")


def plot_result(wall_linework, axes, output_path: Path, show: bool = False):
    fig, ax = create_axes((16, 10))
    for line in wall_linework:
        x, y = line.xy
        ax.plot(x, y, color="#9aa0a6", linewidth=0.5, alpha=1, zorder=3)

    axis_label_added = False
    for line, _thickness in axes:
        x, y = line.xy
        ax.plot(
            x,
            y,
            color="red",
            linewidth=1.6,
            zorder=10,
            label="wall axis" if not axis_label_added else None,
        )
        axis_label_added = True

    ax.set_title("Wall Axis Extraction (DXF)")
    if axis_label_added:
        ax.legend(loc="upper right")
    save_and_maybe_show(fig, output_path, show=show)


if __name__ == "__main__":
    main()

