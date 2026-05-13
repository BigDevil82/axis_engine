from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.cad_processor import CADLayoutProcessor
from axis_engine.dxf_io import read_design_axes, write_design_axes
from axis_engine.structural_design import BeamKind, StructuralDesigner
from axis_engine.structural_design.designer import StructuralDesignOptions, dominant_wall_thickness
from axis_engine.structural_design.slab_division import SlabDivisionOptions
from cad_tests.cli_common import DEFAULT_DXF_PATH, ensure_dxf_exists
from cad_tests.plot_common import create_axes, save_and_maybe_show

DEFAULT_OUTPUT_PATH = str(Path(DEFAULT_DXF_PATH).with_name("structural_design.png"))


def parse_args():
    parser = argparse.ArgumentParser(description="结构布置结果最小可视化验证。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH)
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH)
    parser.add_argument("--wall-layer", action="append", dest="wall_layers")
    parser.add_argument("--axis-layer", action="append", dest="axis_layers")
    parser.add_argument("--opening-layer", action="append", dest="opening_layers")
    parser.add_argument(
        "--source-backend",
        choices=("dxf", "cad"),
        default="dxf",
        help="原始输入来源：dxf 读取 --dxf，cad 读取当前 AutoCAD 图形。",
    )
    parser.add_argument(
        "--slab-max-edge", type=float, default=6000.0, help="楼板边长超过该值时尝试布置板内分割梁。"
    )
    parser.add_argument("--slab-min-split", type=float, default=1000.0, help="板内分割梁最小有效长度。")
    parser.add_argument(
        "--slab-min-area-ratio", type=float, default=0.25, help="分割后较小区域/较大区域的最小面积比。"
    )
    parser.add_argument("--write", action="store_true", help="生成结构布置后写入编辑图层，并展示写入结果。")
    parser.add_argument("--read", action="store_true", help="从编辑图层读取剪力墙和梁并展示。")
    parser.add_argument(
        "--backend",
        choices=("dxf", "cad"),
        default="dxf",
        help="读写后端：dxf 操作 --dxf 文件，cad 操作当前 AutoCAD 图形。",
    )
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.write and args.read:
        raise ValueError("--write 和 --read 不能同时使用。")

    if args.read:
        dxf_path = ensure_dxf_exists(args.dxf) if args.backend == "dxf" else None
        data = read_design_axes(backend=args.backend, path=dxf_path)
        result = design_result_from_dxf_data(data)
        artifacts = empty_plot_artifacts()
        print_design_summary(result)
        plot_result(artifacts, result, Path(args.output), show=args.show)
        print(f"从 {'当前 AutoCAD 图形' if args.backend == 'cad' else dxf_path} 读取编辑图层。")
        print(f"输出图片: {args.output}")
        return

    dxf_path = ensure_dxf_exists(args.dxf) if args.source_backend == "dxf" else Path(args.dxf)
    processor = CADLayoutProcessor(
        dxf_path,
        wall_layers=args.wall_layers,
        axis_layers=args.axis_layers,
        opening_layers=args.opening_layers,
        source_backend=args.source_backend,
    )
    artifacts = processor.build_geometry()
    designer_options = StructuralDesignOptions(
        slab_division_options=SlabDivisionOptions(
            max_edge_length=args.slab_max_edge,
            min_split_length=args.slab_min_split,
            min_area_ratio=args.slab_min_area_ratio,
        )
    )
    result = StructuralDesigner(designer_options).design(artifacts)

    if args.write:
        if args.backend == "cad":
            write_design_axes(result, backend="cad")
            print("已写入当前 AutoCAD 图形。")
        else:
            write_design_axes(result, backend="dxf", source_path=dxf_path)
            data = read_design_axes(backend="dxf", path=dxf_path)
            result = design_result_from_dxf_data(data, slab_regions=result.slab_regions)
            print(f"已写入 DXF 编辑图层: {dxf_path}")
        print_design_summary(result)
        plot_result(artifacts, result, Path(args.output), show=args.show)
        print(f"输出图片: {args.output}")
        return

    print_design_summary(result)
    plot_result(artifacts, result, Path(args.output), show=args.show)
    print(f"输出图片: {args.output}")


def design_result_from_dxf_data(data, slab_regions=None):
    beams = list(data.beams)
    return SimpleNamespace(
        shear_walls=data.shear_walls,
        beams=beams,
        slab_regions=list(slab_regions or []),
        dominant_wall_thickness=dominant_wall_thickness(
            [(wall.axis, wall.thickness) for wall in data.shear_walls]
        ),
    )


def empty_plot_artifacts():
    return SimpleNamespace(axis_linework=[], wall_axes=[])


def print_design_summary(result):
    beam_counts = Counter(beam.kind.value for beam in result.beams)
    beam_summary = ", ".join(f"{k}: {v}" for k, v in sorted(beam_counts.items()))
    print(f"主剪力墙墙厚: {result.dominant_wall_thickness}")
    print(f"剪力墙: {len(result.shear_walls)}")
    print(f"梁: {len(result.beams)} ({beam_summary})")
    print(f"楼板空间: {len(result.slab_regions)}")


def plot_result(artifacts, result, output_path: Path, show: bool = False):
    fig, ax = create_axes((16, 10))

    for region in result.slab_regions:
        polygon = region.recovered_polygon
        if polygon.is_empty:
            continue
        x, y = polygon.exterior.xy
        ax.fill(x, y, color="#e8f5e9", alpha=0.35, linewidth=0, zorder=-3)

    axis_label_added = False
    for line in artifacts.axis_linework:
        x, y = line.xy
        ax.plot(
            x,
            y,
            color="#90a4ae",
            linewidth=0.6,
            linestyle="--",
            alpha=0.65,
            zorder=-1,
            dash_capstyle="butt",
            dash_joinstyle="miter",
            label="reference axis" if not axis_label_added else None,
        )
        axis_label_added = True

    wall_label_added = False
    for line, _thickness in artifacts.wall_axes:
        x, y = line.xy
        ax.plot(
            x,
            y,
            color="#cfd8dc",
            linewidth=1.0,
            zorder=1,
            solid_capstyle="butt",
            solid_joinstyle="miter",
            label="skeleton wall" if not wall_label_added else None,
        )
        wall_label_added = True

    shear_label_added = False
    for wall in result.shear_walls:
        x, y = wall.axis.xy
        ax.plot(
            x,
            y,
            color="#d32f2f",
            linewidth=2.8,
            zorder=5,
            solid_capstyle="butt",
            solid_joinstyle="miter",
            label="shear wall" if not shear_label_added else None,
        )
        shear_label_added = True

    beam_styles = {
        BeamKind.PERIMETER: ("#1976d2", 2.2, "perimeter beam"),
        BeamKind.COUPLING: ("#f57c00", 2.4, "coupling beam"),
        BeamKind.SLAB_DIVIDER: ("#7b1fa2", 1.8, "slab divider"),
    }
    seen_beam_labels = set()
    for beam in result.beams:
        color, width, label = beam_styles.get(beam.kind, ("#455a64", 2.0, beam.kind.value))
        x, y = beam.axis.xy
        ax.plot(
            x,
            y,
            color=color,
            linewidth=width,
            zorder=8,
            solid_capstyle="butt",
            solid_joinstyle="miter",
            label=label if label not in seen_beam_labels else None,
        )
        seen_beam_labels.add(label)

    ax.set_title("Structural Design")
    ax.legend(loc="upper right")
    save_and_maybe_show(fig, output_path, show=show)


if __name__ == "__main__":
    main()
