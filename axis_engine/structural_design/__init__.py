from axis_engine.structural_design.models import (
    Beam,
    BeamKind,
    ShearWall,
    SlabRegion,
    StructuralDesignResult,
)

__all__ = [
    "Beam",
    "BeamKind",
    "ShearWall",
    "SlabRegion",
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
