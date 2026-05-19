from axis_engine.structural_design.models import (
    Beam,
    BeamKind,
    ShearWall,
    SlabRegion,
    StructuralDesignResult,
)
from axis_engine.structural_design.slab_division import SlabDivisionOptions
from axis_engine.structural_design.shear_wall_layout import ShearWallLayoutOptions

__all__ = [
    "Beam",
    "BeamKind",
    "ShearWall",
    "SlabRegion",
    "SlabDivisionOptions",
    "ShearWallLayoutOptions",
    "StructuralDesignOptions",
    "StructuralDesignResult",
    "StructuralDesigner",
]


def __getattr__(name):
    if name in {"StructuralDesignOptions", "StructuralDesigner"}:
        from axis_engine.structural_design.designer import StructuralDesignOptions, StructuralDesigner

        return {
            "StructuralDesignOptions": StructuralDesignOptions,
            "StructuralDesigner": StructuralDesigner,
        }[name]
    raise AttributeError(name)
