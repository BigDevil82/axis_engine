from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt


def create_axes(figsize: tuple[int, int]):
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.2)
    return fig, ax


def save_and_maybe_show(fig, output_path: Path, show: bool = False, dpi: int = 220):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    if show:
        plt.show()
    plt.close(fig)

