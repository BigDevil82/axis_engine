import argparse
import sys
from collections import Counter
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from shapely.geometry import LineString

# 确保脚本能找到项目根目录下的包
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from cad_tests.dxf_utils import (
    _entity_segments,
    _iter_entity_and_nested_virtuals,
    read_dxf,
    read_dxf_line_segments_from_layers,
)
from axis_engine.opening_clustering import cluster_bounds as _cluster_bounds
from axis_engine.opening_clustering import cluster_line_segments

DEFAULT_DXF_PATH = r"E:\Common\Desktop\test\ai-structures\case3\test.dxf"


def main():
    parser = argparse.ArgumentParser(description="递归提取并绘制 DXF 文件中指定图层（如 WINDOW）的线段。")
    parser.add_argument("--dxf", default=DEFAULT_DXF_PATH, help="输入 DXF 文件路径。")
    parser.add_argument("--layer", default="WINDOW", help="要提取和绘制的图层名称。")
    parser.add_argument("--output", default="window_layer_plot.png", help="输出图片路径。")
    parser.add_argument("--no-show", action="store_true", help="不弹出预览窗口，仅保存图片。")
    parser.add_argument(
        "--diagnose", action="store_true", help="按模型空间根实体输出块展开后的线段数量和坐标范围。"
    )
    parser.add_argument(
        "--include-hidden",
        action="store_true",
        help="兼容旧参数；当前默认会递归进入所有块以匹配 CAD 图层隔离效果。",
    )
    parser.add_argument(
        "--respect-layer-visibility", action="store_true", help="严格跳过关闭/冻结图层上的实体。"
    )
    parser.add_argument("--bbox", action="store_true", help="对门窗线元聚类并绘制每个聚类的包围盒。")
    parser.add_argument("--cluster-distance", type=float, default=120.0, help="线元聚类的最大间距。")
    parser.add_argument("--min-cluster-lines", type=int, default=2, help="保留为门窗组所需的最少线段数。")
    parser.add_argument("--bbox-padding", type=float, default=0.0, help="包围盒向外扩展距离。")
    parser.add_argument("--max-bbox-size", type=float, default=5000.0, help="过滤过大聚类包围盒的最大宽/高。")

    args = parser.parse_args()

    dxf_path = Path(args.dxf)
    if not dxf_path.exists():
        print(f"找不到 DXF 文件: {dxf_path}")
        return

    print(f"正在读取 DXF 文件: {dxf_path}")
    doc = read_dxf(str(dxf_path))

    print(f"正在提取图层 '{args.layer}' 的线段（支持嵌套块的递归读取）...")
    # read_dxf_line_segments_from_layers 内部通过 _iter_entity_and_nested_virtuals
    # 自动处理并解包块（嵌套块）和图层过滤
    visible_only = args.respect_layer_visibility and not args.include_hidden
    lines = read_dxf_line_segments_from_layers(doc, [args.layer], visible_only=visible_only)
    print(f"共提取到 {len(lines)} 条线段。")

    if args.diagnose:
        _print_diagnostics(doc, args.layer, visible_only=visible_only)

    if not lines:
        print(f"图层 '{args.layer}' 内没有找到任何可绘制线段。")
        return

    print("正在生成绘图...")
    fig, ax = plt.subplots(figsize=(12, 10))

    # 遍历并绘制线段
    for line in lines:
        x, y = line.xy
        ax.plot(x, y, color="#1976d2", linewidth=1.0, alpha=0.8)

    clusters = []
    if args.bbox:
        clusters = cluster_line_segments(
            lines,
            distance=args.cluster_distance,
            min_lines=args.min_cluster_lines,
            max_bbox_size=args.max_bbox_size,
        )
        print(f"门窗线元聚类: {len(clusters)} 组。")
        for index, cluster in enumerate(clusters, start=1):
            minx, miny, maxx, maxy = _cluster_bounds(cluster, args.bbox_padding)
            rect = patches.Rectangle(
                (minx, miny),
                maxx - minx,
                maxy - miny,
                fill=False,
                edgecolor="#d32f2f",
                linewidth=1.1,
                alpha=0.9,
            )
            ax.add_patch(rect)
            ax.text(
                minx,
                maxy + args.bbox_padding * 0.35,
                str(index),
                color="#d32f2f",
                fontsize=6,
                ha="left",
                va="bottom",
            )

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"Layer '{args.layer}' Contents")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.tick_params(labelsize=8)

    fig.tight_layout()

    output_path = Path(args.output).resolve()
    fig.savefig(output_path, dpi=200)
    print(f"图像已保存至: {output_path}")

    if not args.no_show:
        plt.show()
    plt.close(fig)


def _print_diagnostics(doc, layer: str, visible_only: bool):
    print("\n诊断信息：按模型空间根实体统计展开结果")
    print(
        "index | root_type | root_layer | block_name | lines | "
        "insert(x,y) | scale(x,y) | rotation | extent(xmin,xmax,ymin,ymax) | expanded_types"
    )

    layer_set = {layer}
    rows = []
    for index, entity in enumerate(doc.modelspace()):
        group_lines: list[LineString] = []
        expanded_types = Counter()

        for expanded_entity, selected_by_parent, transform in _iter_entity_and_nested_virtuals(
            entity,
            layer_set,
            doc=doc,
            visible_only=visible_only,
        ):
            if not selected_by_parent and expanded_entity.dxf.layer not in layer_set:
                continue

            expanded_types[expanded_entity.dxftype()] += 1
            for start, end, _is_arc, _source_type in _entity_segments(expanded_entity, transform):
                line = LineString([start, end])
                if line.length >= 1.0:
                    group_lines.append(line)

        if not group_lines:
            continue

        xs = [x for line in group_lines for x, _y in line.coords]
        ys = [y for line in group_lines for _x, y in line.coords]
        block_name = entity.dxf.get("name", "") if entity.dxftype() == "INSERT" else ""
        insert = entity.dxf.get("insert", None) if entity.dxftype() == "INSERT" else None
        insert_xy = (float(insert[0]), float(insert[1])) if insert is not None else None
        scale_xy = (
            (
                float(entity.dxf.get("xscale", 1.0)),
                float(entity.dxf.get("yscale", 1.0)),
            )
            if entity.dxftype() == "INSERT"
            else None
        )
        rotation = float(entity.dxf.get("rotation", 0.0)) if entity.dxftype() == "INSERT" else None
        extent = (min(xs), max(xs), min(ys), max(ys))
        rows.append(
            (
                len(group_lines),
                index,
                entity.dxftype(),
                entity.dxf.layer,
                block_name,
                insert_xy,
                scale_xy,
                rotation,
                extent,
                expanded_types,
            )
        )

    for (
        line_count,
        index,
        root_type,
        root_layer,
        block_name,
        insert_xy,
        scale_xy,
        rotation,
        extent,
        expanded_types,
    ) in sorted(rows, reverse=True)[:30]:
        extent_text = ", ".join(f"{value:.1f}" for value in extent)
        insert_text = "-" if insert_xy is None else f"({insert_xy[0]:.1f},{insert_xy[1]:.1f})"
        scale_text = "-" if scale_xy is None else f"({scale_xy[0]:.3g},{scale_xy[1]:.3g})"
        rotation_text = "-" if rotation is None else f"{rotation:.3g}"
        print(
            f"{index:>5} | {root_type:<9} | {root_layer:<10} | {block_name:<18} | "
            f"{line_count:>5} | {insert_text:<19} | {scale_text:<13} | {rotation_text:<8} | "
            f"({extent_text}) | {dict(expanded_types)}"
        )


if __name__ == "__main__":
    main()
