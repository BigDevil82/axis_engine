from __future__ import annotations

from pathlib import Path

DEFAULT_DXF_PATH = r"E:\Common\Desktop\test\ai-structures\case3\test.dxf"


def normalize_dxf_path(path_str: str) -> Path:
    path = Path(path_str).expanduser()
    return path.resolve() if not path.is_absolute() else path


def ensure_dxf_exists(path_str: str) -> Path:
    dxf_path = normalize_dxf_path(path_str)
    if not dxf_path.exists():
        raise FileNotFoundError(f"DXF file not found: {dxf_path}")
    return dxf_path

