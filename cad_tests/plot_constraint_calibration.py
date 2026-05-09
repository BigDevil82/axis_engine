from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_processor import CADLayoutProcessor
from axis_engine.constraint_network_calibrator import (
    ConstraintCalibrationOptions,
    calibrate_orthogonal_segments,
)
from axis_engine.line_network_calibrator import NetworkSegment, SegmentType
from cad_tests.cli_common import (
    DEFAULT_CALIBRATION_OUTPUT_PATH,
    DEFAULT_DXF_PATH,
    ensure_dxf_exists,
)
from cad_tests.plot_common import create_axes, save_and_maybe_show

DEFAULT_OUTPUT_PATH = DEFAULT_CALIBRATION_OUTPUT_PATH


def parse_args():
    parser = argparse.ArgumentParser(description="试验基于约束的正交线网校准算法，并绘制结果。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH)
    parser.add_argument("--opening-layer", action="append", dest="opening_layers", default=["WINDOW"])
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--eps-axis", type=float, default=200.0)
    parser.add_argument("--tau-join", type=float, default=120.0)
    parser.add_argument("--tau-node", type=float, default=200.0)
    parser.add_argument("--tau-span", type=float, default=300.0)
    parser.add_argument("--tau-extend", type=float, default=200.0)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    dxf_path = ensure_dxf_exists(args.dxf)
    processor = CADLayoutProcessor(dxf_path, opening_layers=args.opening_layers)
    processor.build_geometry()
    segments = processor.collect_network_segments()

    options = ConstraintCalibrationOptions(
        eps_axis=args.eps_axis,
        tau_join=args.tau_join,
        tau_node=args.tau_node,
        tau_span=args.tau_span,
        tau_extend=args.tau_extend,
    )
    result = calibrate_orthogonal_segments(segments, options)
    print(f"输入语义线段: {len(segments)}")
    print(f"接受连接约束: {len(result.accepted_candidates)}")
    print(f"校准后线段: {len(result.adjusted_segments)}")
    print(f"语义拓扑线段: {len(result.topology_segments)}")
    print(f"拓扑子线段: {len(result.topology.geoms) if hasattr(result.topology, 'geoms') else 1}")

    plot_result(segments, result.topology_segments, Path(args.output), show=args.show)
    print(f"输出图片: {args.output}")


def plot_result(
    raw_segments: list[NetworkSegment],
    calibrated: list[NetworkSegment],
    output_path: Path,
    show: bool = False,
):
    fig, ax = create_axes((18, 10))

    for segment in raw_segments:
        x, y = segment.geometry.xy
        color = "#9aa0a6"
        linewidth = 4
        alpha = 0.35
        if segment.seg_type == SegmentType.WALL:
            color = "#b0b0b0"
        elif segment.seg_type == SegmentType.DOOR:
            color = "#7e57c2"
        elif segment.seg_type == SegmentType.WINDOW:
            color = "#26a69a"
        ax.plot(x, y, color=color, linewidth=linewidth, alpha=alpha, zorder=1)

    for segment in calibrated:
        line = segment.geometry
        # if line.length < 200.0:
        #     continue
        x, y = line.xy
        color = "#d32f2f"
        linewidth = 1.6
        if segment.seg_type == SegmentType.WALL:
            color = "#d32f2f"
            linewidth = 1.8
        elif segment.seg_type == SegmentType.DOOR:
            color = "#5e35b1"
        elif segment.seg_type == SegmentType.WINDOW:
            color = "#00897b"
        ax.plot(x, y, color=color, linewidth=linewidth, zorder=5)

    # plot end points, collect first
    xs = []
    ys = []
    for segment in calibrated:
        line = segment.geometry
        x, y = line.xy
        xs.extend([x[0], x[-1]])
        ys.extend([y[0], y[-1]])
    ax.scatter(xs, ys, color="#45d32f", s=5, zorder=10, label="calibrated endpoints")

    ax.set_title("Constraint-Calibrated Orthogonal Network")
    save_and_maybe_show(fig, output_path, show=show)

if __name__ == "__main__":
    main()
