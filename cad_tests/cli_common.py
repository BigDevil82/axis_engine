from __future__ import annotations

from pathlib import Path

CASE_ROOT = Path(r"E:\Common\Desktop\test\ai-structures\case3")
DEFAULT_DXF_PATH = str(CASE_ROOT / "test.dxf")
DEFAULT_WALL_AXES_OUTPUT_PATH = str(CASE_ROOT / "wall_axes_linework.png")
DEFAULT_OPENINGS_OUTPUT_PATH = str(CASE_ROOT / "opening_embedments.png")
DEFAULT_CALIBRATION_OUTPUT_PATH = str(CASE_ROOT / "constraint_calibrated_network.png")


def normalize_dxf_path(path_str: str) -> Path:
    path = Path(path_str).expanduser()
    return path.resolve() if not path.is_absolute() else path


def ensure_dxf_exists(path_str: str) -> Path:
    dxf_path = normalize_dxf_path(path_str)
    if not dxf_path.exists():
        raise FileNotFoundError(f"DXF file not found: {dxf_path}")
    return dxf_path

