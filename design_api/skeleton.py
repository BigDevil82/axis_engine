from __future__ import annotations

from typing import Any

from shapely.geometry import MultiLineString, MultiPolygon

from axis_engine.cad_processor import (
    LayoutArtifacts,
    ReferenceAxisGrid,
    infer_core_wall_axis_lines,
    normalize_reference_axis_linework,
)
from axis_engine.dxf_utils import geometries_to_linework
from axis_engine.linework_axis_extractor import (
    extract_wall_axes_from_linework,
    infer_wall_thicknesses,
    repair_wall_linework,
)
from axis_engine.opening_clustering import cluster_opening_geometries
from axis_engine.opening_embedment import infer_opening_embedments
from axis_engine.raw_wall_polygon_builder import build_wall_polygon_from_raw_lines
from axis_engine.skeleton_spur_pruner import prune_skeleton_spurs
from axis_engine.skeleton_topology_calibrator import calibrate_skeleton_topology
from axis_engine.structural_design.skeleton_spaces import build_buffered_network, extract_slab_regions

from design_api.serialization import (
    geometries_from_payload,
    lines_from_payload,
    opening_to_payload,
    openings_from_payload,
    slab_region_to_payload,
    wall_axes_from_payload,
    wall_axis_to_payload,
)


def extract_skeleton(payload: dict[str, Any]) -> dict[str, Any]:
    options = dict(payload.get("options", {}))
    wall_geometries = geometries_from_payload(payload.get("wall_geometries", ()))
    opening_geometries = geometries_from_payload(payload.get("opening_geometries", ()))
    axis_geometries = geometries_from_payload(payload.get("axis_geometries", ()))

    wall_linework = geometries_to_linework(wall_geometries)
    axis_linework = geometries_to_linework(axis_geometries)
    reference_axes = ReferenceAxisGrid.from_lines(axis_linework)
    axis_linework = normalize_reference_axis_linework(axis_linework, reference_axes)

    wall_axis_options = dict(options.get("wall_axis", {}))
    repair_options = _repair_options(wall_axis_options)
    repaired_edges = repair_wall_linework(wall_linework, **repair_options)
    wall_thicknesses = options.get("wall_thicknesses") or infer_wall_thicknesses(repaired_edges)
    wall_axes = extract_wall_axes_from_linework(
        wall_linework,
        thickness_candidates=wall_thicknesses,
        **wall_axis_options,
    )

    axis_linework, reference_axes = _augment_reference_axes(
        axis_linework,
        wall_axes,
        options,
    )
    wall_axes = _align_wall_axes(wall_axes, reference_axes, options)
    wall_polygon = build_wall_polygon_from_raw_lines(wall_linework, wall_thicknesses=wall_thicknesses)

    opening_clusters = cluster_opening_geometries(
        opening_geometries,
        **dict(options.get("opening_cluster", {})),
    )
    opening_embedments = infer_opening_embedments(
        wall_axes,
        opening_clusters,
        **dict(options.get("opening_embedment", {})),
    )
    opening_embedments = _align_openings(opening_embedments, reference_axes, options)

    normalized = normalize_skeleton(
        {
            "wall_axes": [
                {"line": wall_axis_to_payload(line, thickness, index)["line"], "thickness": thickness}
                for index, (line, thickness) in enumerate(wall_axes)
            ],
            "opening_embedments": [
                opening_to_payload(opening, index)
                for index, opening in enumerate(opening_embedments)
            ],
            "axis_lines": [
                {"line": [[float(x), float(y)] for x, y in line.coords]}
                for line in axis_linework
            ],
            "options": options,
        }
    )
    normalized["diagnostics"].update(
        {
            "wall_geometry_count": len(wall_geometries),
            "opening_geometry_count": len(opening_geometries),
            "axis_geometry_count": len(axis_geometries),
            "opening_cluster_count": len(opening_clusters),
            "wall_thicknesses": list(wall_thicknesses),
            "wall_polygon_area": wall_polygon.area if not wall_polygon.is_empty else 0.0,
        }
    )
    return normalized


def normalize_skeleton(payload: dict[str, Any]) -> dict[str, Any]:
    options = dict(payload.get("options", {}))
    wall_axes = wall_axes_from_payload(payload.get("wall_axes", ()))
    opening_embedments = openings_from_payload(payload.get("opening_embedments", ()))
    axis_lines = lines_from_payload(payload.get("axis_lines", ()))
    reference_axes = ReferenceAxisGrid.from_lines(axis_lines)
    axis_lines = normalize_reference_axis_linework(axis_lines, reference_axes)

    wall_axes = _align_wall_axes(wall_axes, reference_axes, options)
    opening_embedments = _align_openings(opening_embedments, reference_axes, options)

    topology_options = dict(options.get("topology_calibration", {}))
    if wall_axes or opening_embedments:
        calibrated = calibrate_skeleton_topology(wall_axes, opening_embedments, **topology_options)
        wall_axes = calibrated.wall_axes
        opening_embedments = calibrated.opening_embedments

    wall_axes = _align_wall_axes(wall_axes, reference_axes, options)
    opening_embedments = _align_openings(opening_embedments, reference_axes, options)

    prune_options = dict(options.get("spur_prune", {}))
    if wall_axes or opening_embedments:
        pruned = prune_skeleton_spurs(wall_axes, opening_embedments, **prune_options)
        wall_axes = pruned.wall_axes
        opening_embedments = pruned.opening_embedments

    slab_regions = _skeleton_slab_regions(wall_axes, opening_embedments, options)
    return {
        "wall_axes": [
            wall_axis_to_payload(line, thickness, index)
            for index, (line, thickness) in enumerate(wall_axes)
        ],
        "opening_embedments": [
            opening_to_payload(opening, index)
            for index, opening in enumerate(opening_embedments)
        ],
        "axis_lines": [
            {"id": f"axis_line_{index}", "line": [[float(x), float(y)] for x, y in line.coords]}
            for index, line in enumerate(axis_lines)
        ],
        "slab_regions": [
            slab_region_to_payload(region, index)
            for index, region in enumerate(slab_regions)
        ],
        "diagnostics": {
            "wall_axis_count": len(wall_axes),
            "opening_embedment_count": len(opening_embedments),
            "axis_line_count": len(axis_lines),
            "slab_region_count": len(slab_regions),
        },
    }


def artifacts_from_skeleton_payload(payload: dict[str, Any]) -> LayoutArtifacts:
    wall_axes = wall_axes_from_payload(payload.get("wall_axes", ()))
    opening_embedments = openings_from_payload(payload.get("opening_embedments", ()))
    axis_lines = lines_from_payload(payload.get("axis_lines", ()))
    return LayoutArtifacts(
        axis_geometries=[],
        axis_linework=axis_lines,
        wall_geometries=[],
        wall_linework=[],
        wall_axes=wall_axes,
        wall_polygon=MultiPolygon(),
        opening_geometries=[],
        opening_clusters=[],
        opening_embedments=opening_embedments,
        components=_components_from_openings(opening_embedments),
    )


def _repair_options(wall_axis_options: dict[str, Any]) -> dict[str, Any]:
    repair_options = {
        key: wall_axis_options[key]
        for key in ("axis_tolerance", "snap_tolerance")
        if key in wall_axis_options
    }
    if "min_feature_len" in wall_axis_options:
        repair_options["min_segment_length"] = wall_axis_options["min_feature_len"]
    return repair_options


def _augment_reference_axes(axis_linework, wall_axes, options):
    core_options = dict(options.get("core_axis", {}))
    inferred = infer_core_wall_axis_lines(wall_axes, **core_options)
    if inferred:
        axis_linework = list(axis_linework) + inferred
    reference_axes = ReferenceAxisGrid.from_lines(axis_linework)
    return normalize_reference_axis_linework(axis_linework, reference_axes), reference_axes


def _align_wall_axes(wall_axes, reference_axes: ReferenceAxisGrid, options: dict[str, Any]):
    if reference_axes.is_empty:
        return wall_axes
    tolerance = float(options.get("reference_axis_snap_tolerance", 100.0))
    aligned = []
    for line, thickness in wall_axes:
        snapped = reference_axes.align_line(line, tolerance)
        if snapped.length > 0:
            aligned.append((snapped, thickness))
    return aligned


def _align_openings(opening_embedments, reference_axes: ReferenceAxisGrid, options: dict[str, Any]):
    if reference_axes.is_empty:
        return opening_embedments
    from axis_engine.opening_embedment import OpeningEmbedment

    tolerance = float(options.get("reference_axis_snap_tolerance", 100.0))
    return [
        OpeningEmbedment(
            opening_type=opening.opening_type,
            embed_line=reference_axes.align_line(opening.embed_line, tolerance),
            cluster_index=opening.cluster_index,
            confidence=opening.confidence,
            reason=f"{opening.reason}|axis_aligned",
        )
        for opening in opening_embedments
    ]


def _skeleton_slab_regions(wall_axes, opening_embedments, options: dict[str, Any]):
    buffer_distance = float(options.get("skeleton_slab_buffer", options.get("buffer_distance", 150.0)))
    buffered = build_buffered_network(wall_axes, opening_embedments, buffer_distance)
    return extract_slab_regions(buffered, buffer_distance)


def _components_from_openings(opening_embedments):
    groups = {"doors": [], "windows": [], "balconies": []}
    for opening in opening_embedments:
        if opening.opening_type == "door":
            groups["doors"].append(opening.embed_line)
        elif opening.opening_type == "balcony":
            groups["balconies"].append(opening.embed_line)
        else:
            groups["windows"].append(opening.embed_line)
    return {key: MultiLineString(lines) if lines else MultiLineString() for key, lines in groups.items()}
