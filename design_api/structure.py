from __future__ import annotations

from typing import Any

from axis_engine.structural_design.designer import StructuralDesigner, StructuralDesignOptions
from axis_engine.structural_design.slab_division import SlabDivisionOptions, slab_regions_from_structural_lines
from axis_engine.structural_design.spur_pruner import prune_structural_spurs
from axis_engine.structural_design.shear_wall_layout import ShearWallLayoutOptions

from design_api.serialization import (
    beam_to_payload,
    beams_from_payload,
    shear_wall_to_payload,
    shear_walls_from_payload,
    slab_region_to_payload,
)
from design_api.skeleton import artifacts_from_skeleton_payload


def design_structure(payload: dict[str, Any]) -> dict[str, Any]:
    options = dict(payload.get("options", {}))
    artifacts = artifacts_from_skeleton_payload(payload)
    result = StructuralDesigner(_design_options(options)).design(artifacts)
    return _structure_response(
        result.shear_walls,
        result.beams,
        result.slab_regions,
        {
            "dominant_wall_thickness": result.dominant_wall_thickness,
            "source": "generated",
        },
    )


def normalize_structure(payload: dict[str, Any]) -> dict[str, Any]:
    options = dict(payload.get("options", {}))
    shear_walls = shear_walls_from_payload(payload.get("shear_walls", ()))
    beams = beams_from_payload(payload.get("beams", ()))
    prune_options = dict(options.get("structural_prune", {}))
    pruned = prune_structural_spurs(shear_walls, beams, **prune_options)
    shear_walls = pruned.shear_walls
    beams = pruned.beams
    slab_regions = slab_regions_from_structural_lines(
        shear_walls,
        beams,
        float(options.get("buffer_distance", 150.0)),
    )
    return _structure_response(
        shear_walls,
        beams,
        slab_regions,
        {"source": "normalized"},
    )


def _design_options(options: dict[str, Any]) -> StructuralDesignOptions:
    slab_options = options.get("slab_division_options", {})
    shear_options = options.get("shear_wall_layout_options", {})
    return StructuralDesignOptions(
        buffer_distance=float(options.get("buffer_distance", 150.0)),
        coupling_const_tolerance=float(options.get("coupling_const_tolerance", 10.0)),
        coupling_min_gap=float(options.get("coupling_min_gap", 300.0)),
        coupling_max_gap=float(options.get("coupling_max_gap", 5000.0)),
        coupling_near_parallel_wall_distance=float(options.get("coupling_near_parallel_wall_distance", 600.0)),
        coupling_near_parallel_wall_overlap_ratio=float(options.get("coupling_near_parallel_wall_overlap_ratio", 0.30)),
        structural_spur_length=float(options.get("structural_spur_length", 300.0)),
        structural_stitch_gap_distance=float(options.get("structural_stitch_gap_distance", 300.0)),
        structural_stitch_probe_width=float(options.get("structural_stitch_probe_width", 5.0)),
        structural_stitch_min_beam_length=float(options.get("structural_stitch_min_beam_length", 400.0)),
        balcony_beam_edge_tolerance=float(options.get("balcony_beam_edge_tolerance", 120.0)),
        balcony_beam_min_edge_length=float(options.get("balcony_beam_min_edge_length", 800.0)),
        balcony_beam_opening_overlap_ratio=float(options.get("balcony_beam_opening_overlap_ratio", 0.55)),
        shear_wall_layout_options=ShearWallLayoutOptions(**shear_options),
        min_element_length=float(options.get("min_element_length", 1.0)),
        slab_division_options=SlabDivisionOptions(**slab_options),
    )


def _structure_response(shear_walls, beams, slab_regions, diagnostics):
    beam_counts: dict[str, int] = {}
    for beam in beams:
        beam_counts[beam.kind.value] = beam_counts.get(beam.kind.value, 0) + 1
    diagnostics = {
        **diagnostics,
        "shear_wall_count": len(shear_walls),
        "beam_count": len(beams),
        "beam_counts": beam_counts,
        "slab_region_count": len(slab_regions),
    }
    return {
        "shear_walls": [
            shear_wall_to_payload(wall, index)
            for index, wall in enumerate(shear_walls)
        ],
        "beams": [
            beam_to_payload(beam, index)
            for index, beam in enumerate(beams)
        ],
        "slab_regions": [
            slab_region_to_payload(region, index)
            for index, region in enumerate(slab_regions)
        ],
        "diagnostics": diagnostics,
    }
