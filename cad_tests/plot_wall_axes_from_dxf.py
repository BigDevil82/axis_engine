from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_processor import CADLayoutProcessor

DEFAULT_DXF_PATH = r"E:\Common\Desktop\test\ai-structures\case3\test.dxf"
DEFAULT_OUTPUT_PATH = r"E:\Common\Desktop\test\ai-structures\case3\wall_axes_linework.png"


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
    processor = CADLayoutProcessor(
        args.dxf,
        wall_layers=args.wall_layers,
        opening_layers=args.opening_layers,
    )
    artifacts = processor.build_geometry()
    thickness_counts = Counter(thickness for _line, thickness in artifacts.wall_centerlines)
    print(f"墙体原始线段: {len(artifacts.wall_lines)}")
    print(f"提取墙轴线: {len(artifacts.wall_centerlines)}")
    if thickness_counts:
        summary = ", ".join(f"{thick:g}: {count}" for thick, count in sorted(thickness_counts.items()))
        print(f"轴线墙厚分布: {summary}")
    print(f"门窗组: {len(artifacts.opening_clusters)}")
    print(f"门窗嵌入线: {len(artifacts.opening_embedments)}")
    plot_result(artifacts.wall_lines, artifacts.wall_centerlines, Path(args.output), show=args.show)
    print(f"输出图片: {args.output}")


def plot_result(wall_lines, axes, output_path: Path, show: bool = False):
    fig, ax = plt.subplots(figsize=(16, 10))
    for line in wall_lines:
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

    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Wall Axis Extraction (DXF)")
    ax.grid(True, linestyle="--", alpha=0.2)
    if axis_label_added:
        ax.legend(loc="upper right")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()

