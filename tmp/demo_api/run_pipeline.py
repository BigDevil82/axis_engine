"""Run the complete public API pipeline and render every returned design stage."""

from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from axis_engine.structural_design.models import Beam, BeamKind, ShearWall
from cad_tests.plot_structural_design_from_dxf import plot_result as plot_structure
from cad_tests.plot_windows_from_dxf import plot_result as plot_skeleton
from design_api.serialization import geometries_from_payload, lines_from_payload, openings_from_payload, wall_axes_from_payload


DEFAULT_INPUT = Path(__file__).with_name("demo_input.json")
DEFAULT_OUTPUT_DIR = Path(__file__).with_name("output")
DEFAULT_API_URL = "http://218.93.206.72:10187/api/v1"


def parse_args():
    parser = argparse.ArgumentParser(description="调用结构设计 API 并保存真实图纸结果。")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    return parser.parse_args()


def main():
    args = parse_args()
    raw = _read_json(args.input)
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)

    extracted = _post(args.api_url, "/skeleton/extract", raw)
    _write_json(output / "01_skeleton_extracted.json", extracted)
    _plot_skeleton(raw, extracted, output / "01_skeleton_extracted.png")

    normalized_skeleton = _post(args.api_url, "/skeleton/normalize", _skeleton_request(extracted))
    _write_json(output / "02_skeleton_normalized.json", normalized_skeleton)
    _plot_skeleton(raw, normalized_skeleton, output / "02_skeleton_normalized.png")

    designed = _post(args.api_url, "/structure/design", _skeleton_request(normalized_skeleton))
    _write_json(output / "03_structure_designed.json", designed)
    _plot_structure(normalized_skeleton, designed, output / "03_structure_designed.png")

    normalized_structure = _post(args.api_url, "/structure/normalize", _structure_request(designed))
    _write_json(output / "04_structure_normalized.json", normalized_structure)
    _plot_structure(normalized_skeleton, normalized_structure, output / "04_structure_normalized.png")

    for name, result in (
        ("骨架提取", extracted),
        ("骨架校准", normalized_skeleton),
        ("结构生成", designed),
        ("结构校准", normalized_structure),
    ):
        print(f"{name}: {json.dumps(result['diagnostics'], ensure_ascii=False)}")
    print(f"结果目录: {output}")


def _post(base_url: str, path: str, payload: dict) -> dict:
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(request, timeout=300) as response:
        return json.loads(response.read())


def _skeleton_request(result: dict) -> dict:
    return {
        "wall_axes": result["wall_axes"],
        "opening_embedments": result["opening_embedments"],
        "axis_lines": result["axis_lines"],
    }


def _structure_request(result: dict) -> dict:
    return {"shear_walls": result["shear_walls"], "beams": result["beams"]}


def _plot_skeleton(raw: dict, result: dict, output: Path) -> None:
    plot_skeleton(
        lines_from_payload(result["axis_lines"]),
        wall_axes_from_payload(result["wall_axes"]),
        geometries_from_payload(raw["opening_geometries"]),
        [],
        openings_from_payload(result["opening_embedments"]),
        output,
    )


def _plot_structure(skeleton: dict, result: dict, output: Path) -> None:
    artifacts = SimpleNamespace(
        axis_linework=lines_from_payload(skeleton["axis_lines"]),
        wall_axes=wall_axes_from_payload(skeleton["wall_axes"]),
    )
    design = SimpleNamespace(
        shear_walls=[
            ShearWall(axis=_line(item["line"]), thickness=item["thickness"], source=item["source"])
            for item in result["shear_walls"]
        ],
        beams=[
            Beam(
                axis=_line(item["line"]),
                kind=BeamKind(item["kind"]),
                reason=item["reason"],
                related_ids=tuple(item["related_ids"]),
            )
            for item in result["beams"]
        ],
        slab_regions=[],
    )
    plot_structure(artifacts, design, output)


def _line(points):
    from shapely.geometry import LineString

    return LineString(points)


def _read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
