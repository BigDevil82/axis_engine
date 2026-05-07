import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt

COMPONENT_KEYS = ("walls", "doors", "windows")
COMPONENT_COLORS = {
    "walls": "#1f77b4",
    "doors": "#ff7f0e",
    "windows": "#2ca02c",
}


def get_point(line_item: dict, point_key: str):
    """Return (x, y) for a point key, tolerant of key case."""
    point = line_item.get(point_key) or line_item.get(point_key.lower())
    if not isinstance(point, dict):
        return None

    x = point.get("X") if "X" in point else point.get("x")
    y = point.get("Y") if "Y" in point else point.get("y")
    if x is None or y is None:
        return None
    return float(x), float(y)


def extract_segments(data: dict, component_key: str):
    """Extract valid line segments from one component list."""
    segments = []
    for item in data.get(component_key, []):
        if not isinstance(item, dict):
            continue
        p1 = get_point(item, "StartPoint")
        p2 = get_point(item, "EndPoint")
        if p1 is None or p2 is None:
            continue
        segments.append((p1, p2))
    return segments


def plot_components(json_path: Path, output_path: Path = None):
    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    fig, ax = plt.subplots(figsize=(10, 8))
    has_any_segment = False

    for key in COMPONENT_KEYS:
        segments = extract_segments(data, key)
        if not segments:
            continue

        has_any_segment = True
        color = COMPONENT_COLORS[key]
        for (x1, y1), (x2, y2) in segments:
            ax.plot([x1, x2], [y1, y2], color=color, linewidth=1.2)

        ax.plot([], [], color=color, label=f"{key} ({len(segments)})")

    if not has_any_segment:
        raise ValueError("JSON 中没有可绘制的 walls/doors/windows 线段数据")

    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Building Components (walls / doors / windows)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.2)
    plt.tight_layout()

    if output_path is not None:
        fig.savefig(output_path, dpi=200)
        print(f"Saved figure to: {output_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="读取包含 walls/doors/windows 线段的 JSON 并绘图")
    parser.add_argument("json_path", type=Path, help="输入 JSON 文件路径")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="保存图片路径（例如 output.png）；不传则直接弹窗显示",
    )
    args = parser.parse_args()

    plot_components(args.json_path, args.save)


if __name__ == "__main__":
    main()
