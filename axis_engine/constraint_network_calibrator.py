from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral
from typing import Iterable, Sequence

from shapely.geometry import LineString, MultiLineString, Point
from shapely.ops import unary_union
from shapely.strtree import STRtree

from axis_engine.line_network_calibrator import NetworkSegment, SegmentType


@dataclass(frozen=True)
class ConstraintCalibrationOptions:
    eps_axis: float = 4.0
    tau_join: float = 12.0
    tau_node: float = 8.0
    tau_span: float = 5.0
    tau_extend: float = 5.0
    ambiguity_margin: float = 3.0
    min_length: float = 1.0


@dataclass(frozen=True)
class OrthoSegment:
    index: int
    orientation: str
    c: float
    u0: float
    u1: float
    seg_type: SegmentType
    thickness: float
    is_structural: bool
    length: float

    def endpoint_var(self, at_start: bool) -> tuple[int, str]:
        return (self.index, "u0" if at_start else "u1")

    @property
    def c_var(self) -> tuple[int, str]:
        return (self.index, "c")


@dataclass(frozen=True)
class Candidate:
    source: int
    source_at_start: bool
    target: int
    target_at_start: bool | None
    kind: str
    cost: float
    priority: int


@dataclass(frozen=True)
class CalibrationResult:
    adjusted_segments: list[LineString]
    topology: MultiLineString
    topology_segments: list[NetworkSegment]
    accepted_candidates: list[Candidate]


class ConstraintNetworkCalibrator:
    def __init__(self, options: ConstraintCalibrationOptions | None = None):
        self.options = options or ConstraintCalibrationOptions()

    def calibrate(self, segments: Sequence[NetworkSegment]) -> CalibrationResult:
        ortho = canonicalize_segments(segments, self.options.min_length)
        dsu = WeightedUnionFind()
        for seg in ortho:
            dsu.add(seg.c_var, seg.c, _var_weight(seg, "c"))
            dsu.add((seg.index, "u0"), seg.u0, _var_weight(seg, "u"))
            dsu.add((seg.index, "u1"), seg.u1, _var_weight(seg, "u"))

        for cluster in _axis_clusters([seg for seg in ortho if seg.orientation == "H"], self.options.eps_axis):
            _union_support_cluster(dsu, cluster)
        for cluster in _axis_clusters([seg for seg in ortho if seg.orientation == "V"], self.options.eps_axis):
            _union_support_cluster(dsu, cluster)

        candidates = generate_connection_candidates(ortho, self.options)
        accepted = select_non_conflicting_candidates(candidates, self.options)
        for cand in accepted:
            _add_candidate_constraints(dsu, ortho, cand)

        solved = dsu.solve()
        adjusted_segments = rebuild_network_segments(ortho, solved, self.options.min_length)
        adjusted = [segment.geometry for segment in adjusted_segments]
        topology_segments = build_semantic_topology(adjusted_segments, self.options.min_length)
        topology = MultiLineString([segment.geometry for segment in topology_segments])
        return CalibrationResult(adjusted, topology, topology_segments, accepted)


def calibrate_orthogonal_segments(
    segments: Sequence[NetworkSegment],
    options: ConstraintCalibrationOptions | None = None,
) -> CalibrationResult:
    return ConstraintNetworkCalibrator(options).calibrate(segments)


def canonicalize_segments(
    segments: Sequence[NetworkSegment],
    min_length: float = 1.0,
) -> list[OrthoSegment]:
    result: list[OrthoSegment] = []
    index = 0
    for seg in segments:
        for line in _explode_lines(seg.geometry):
            coords = list(line.coords)
            if len(coords) < 2:
                continue
            x1, y1 = coords[0]
            x2, y2 = coords[-1]
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            if max(dx, dy) < min_length:
                continue
            if dx >= dy:
                c = (y1 + y2) / 2.0
                u0, u1 = sorted((x1, x2))
                orientation = "H"
            else:
                c = (x1 + x2) / 2.0
                u0, u1 = sorted((y1, y2))
                orientation = "V"
            result.append(
                OrthoSegment(
                    index=index,
                    orientation=orientation,
                    c=c,
                    u0=u0,
                    u1=u1,
                    seg_type=seg.seg_type,
                    thickness=seg.thickness,
                    is_structural=seg.is_structural,
                    length=u1 - u0,
                )
            )
            index += 1
    return result


def generate_connection_candidates(
    segs: Sequence[OrthoSegment],
    options: ConstraintCalibrationOptions,
) -> list[Candidate]:
    candidates: list[Candidate] = []
    target_lines = [_segment_line(seg) for seg in segs]
    tree = STRtree(target_lines)

    for source in segs:
        for at_start, endpoint in ((True, source.u0), (False, source.u1)):
            point = _endpoint_point(source, at_start)
            query_geom = Point(point).buffer(max(options.tau_join, options.tau_node) + options.tau_span)
            local_candidates = []
            for item in tree.query(query_geom):
                target_index = int(item) if isinstance(item, Integral) else target_lines.index(item)
                target = segs[target_index]
                if target.index == source.index:
                    continue
                cand = _candidate_between(source, at_start, target, options)
                if cand is not None:
                    local_candidates.append(cand)
            candidates.extend(local_candidates)
    return candidates


def select_non_conflicting_candidates(
    candidates: Sequence[Candidate],
    options: ConstraintCalibrationOptions,
) -> list[Candidate]:
    by_endpoint: dict[tuple[int, bool], list[Candidate]] = {}
    for cand in candidates:
        by_endpoint.setdefault((cand.source, cand.source_at_start), []).append(cand)

    accepted: list[Candidate] = []
    used_endpoint_targets: set[tuple[int, bool]] = set()
    for endpoint, items in by_endpoint.items():
        ranked = sorted(items, key=lambda item: (item.priority, item.cost))
        if not ranked:
            continue
        best = ranked[0]
        if len(ranked) > 1:
            second = ranked[1]
            if second.cost - best.cost < options.ambiguity_margin and second.priority == best.priority:
                continue
        target_endpoint = None
        if best.target_at_start is not None:
            target_endpoint = (best.target, best.target_at_start)
            if target_endpoint in used_endpoint_targets:
                continue
        accepted.append(best)
        if target_endpoint is not None:
            used_endpoint_targets.add(target_endpoint)
    return accepted


def rebuild_segments(
    segs: Sequence[OrthoSegment],
    solved: dict[tuple[int, str], float],
    min_length: float,
) -> list[LineString]:
    lines = []
    for seg in segs:
        c = solved[seg.c_var]
        u0 = solved[(seg.index, "u0")]
        u1 = solved[(seg.index, "u1")]
        if abs(u1 - u0) < min_length:
            continue
        a, b = sorted((u0, u1))
        if seg.orientation == "H":
            lines.append(LineString([(a, c), (b, c)]))
        else:
            lines.append(LineString([(c, a), (c, b)]))
    return lines


def rebuild_network_segments(
    segs: Sequence[OrthoSegment],
    solved: dict[tuple[int, str], float],
    min_length: float,
) -> list[NetworkSegment]:
    result = []
    for seg in segs:
        c = solved[seg.c_var]
        u0 = solved[(seg.index, "u0")]
        u1 = solved[(seg.index, "u1")]
        if abs(u1 - u0) < min_length:
            continue
        a, b = sorted((u0, u1))
        if seg.orientation == "H":
            line = LineString([(a, c), (b, c)])
        else:
            line = LineString([(c, a), (c, b)])
        result.append(
            NetworkSegment(
                geometry=line,
                thickness=seg.thickness,
                seg_type=seg.seg_type,
                is_structural=seg.is_structural,
            )
        )
    return result


def build_semantic_topology(
    segments: Sequence[NetworkSegment],
    min_length: float = 1.0,
    tolerance: float = 1e-6,
) -> list[NetworkSegment]:
    edges = [_semantic_edge(segment) for segment in segments if segment.geometry.length >= min_length]
    if not edges:
        return []

    breakpoints = {index: {edge.u0, edge.u1} for index, edge in enumerate(edges)}
    blocked_points: dict[tuple[str, float], set[float]] = {}

    horizontal = [(index, edge) for index, edge in enumerate(edges) if edge.orientation == "H"]
    vertical = [(index, edge) for index, edge in enumerate(edges) if edge.orientation == "V"]
    for h_index, h_edge in horizontal:
        for v_index, v_edge in vertical:
            if not _within(v_edge.c, h_edge.u0, h_edge.u1, tolerance):
                continue
            if not _within(h_edge.c, v_edge.u0, v_edge.u1, tolerance):
                continue
            breakpoints[h_index].add(v_edge.c)
            breakpoints[v_index].add(h_edge.c)
            blocked_points.setdefault(("H", h_edge.c), set()).add(v_edge.c)
            blocked_points.setdefault(("V", v_edge.c), set()).add(h_edge.c)

    intervals_by_key: dict[tuple, list[tuple[float, float]]] = {}
    for index, edge in enumerate(edges):
        points = sorted(_dedupe_sorted(breakpoints[index], tolerance))
        key = (
            edge.orientation,
            edge.c,
            edge.seg_type,
            round(float(edge.thickness), 6),
            bool(edge.is_structural),
        )
        for start, end in zip(points, points[1:]):
            if end - start >= min_length:
                intervals_by_key.setdefault(key, []).append((start, end))

    merged_segments: list[NetworkSegment] = []
    for key, intervals in intervals_by_key.items():
        orientation, c, seg_type, thickness, is_structural = key
        blocked = blocked_points.get((orientation, c), set())
        for start, end in _merge_intervals_without_cross_nodes(intervals, blocked, tolerance, min_length):
            if orientation == "H":
                line = LineString([(start, c), (end, c)])
            else:
                line = LineString([(c, start), (c, end)])
            merged_segments.append(
                NetworkSegment(
                    geometry=line,
                    thickness=float(thickness),
                    seg_type=seg_type,
                    is_structural=is_structural,
                )
            )

    return merged_segments


def build_planar_topology(lines: Sequence[LineString], min_length: float = 1.0) -> MultiLineString:
    if not lines:
        return MultiLineString([])
    unioned = unary_union(lines)
    result = []
    geoms = unioned.geoms if hasattr(unioned, "geoms") else [unioned]
    for geom in geoms:
        if isinstance(geom, LineString) and geom.length >= min_length:
            result.append(geom)
    return MultiLineString(result)


@dataclass(frozen=True)
class _SemanticEdge:
    orientation: str
    c: float
    u0: float
    u1: float
    seg_type: SegmentType
    thickness: float
    is_structural: bool


def _semantic_edge(segment: NetworkSegment) -> _SemanticEdge:
    line = segment.geometry
    coords = list(line.coords)
    x1, y1 = coords[0]
    x2, y2 = coords[-1]
    if abs(x2 - x1) >= abs(y2 - y1):
        return _SemanticEdge(
            orientation="H",
            c=(y1 + y2) / 2.0,
            u0=min(x1, x2),
            u1=max(x1, x2),
            seg_type=segment.seg_type,
            thickness=segment.thickness,
            is_structural=segment.is_structural,
        )
    return _SemanticEdge(
        orientation="V",
        c=(x1 + x2) / 2.0,
        u0=min(y1, y2),
        u1=max(y1, y2),
        seg_type=segment.seg_type,
        thickness=segment.thickness,
        is_structural=segment.is_structural,
    )


def _merge_intervals_without_cross_nodes(
    intervals: Sequence[tuple[float, float]],
    blocked_points: set[float],
    tolerance: float,
    min_length: float,
) -> list[tuple[float, float]]:
    if not intervals:
        return []
    ordered = sorted((min(a, b), max(a, b)) for a, b in intervals)
    merged: list[tuple[float, float]] = []
    current_start, current_end = ordered[0]
    for start, end in ordered[1:]:
        if start <= current_end + tolerance and not _has_blocked_between(blocked_points, current_end, start, tolerance):
            current_end = max(current_end, end)
            continue
        if current_end - current_start >= min_length:
            merged.append((current_start, current_end))
        current_start, current_end = start, end
    if current_end - current_start >= min_length:
        merged.append((current_start, current_end))
    return merged


def _has_blocked_between(blocked_points: set[float], start: float, end: float, tolerance: float) -> bool:
    low = min(start, end) - tolerance
    high = max(start, end) + tolerance
    return any(low <= point <= high for point in blocked_points)


def _within(value: float, start: float, end: float, tolerance: float) -> bool:
    return min(start, end) - tolerance <= value <= max(start, end) + tolerance


def _dedupe_sorted(values: Iterable[float], tolerance: float) -> list[float]:
    ordered = sorted(values)
    if not ordered:
        return []
    result = [ordered[0]]
    for value in ordered[1:]:
        if abs(value - result[-1]) <= tolerance:
            result[-1] = (result[-1] + value) / 2.0
        else:
            result.append(value)
    return result


def _candidate_between(
    source: OrthoSegment,
    source_at_start: bool,
    target: OrthoSegment,
    options: ConstraintCalibrationOptions,
) -> Candidate | None:
    source_u = source.u0 if source_at_start else source.u1
    if source.orientation != target.orientation:
        gap_axis = abs(source_u - target.c)
        gap_span = _distance_to_interval(source.c, target.u0, target.u1)
        if gap_axis <= options.tau_join and gap_span <= options.tau_span + options.tau_extend:
            target_at_start = None
            if abs(source.c - target.u0) <= options.tau_node:
                target_at_start = True
            elif abs(source.c - target.u1) <= options.tau_node:
                target_at_start = False
            priority = 1 if target_at_start is not None else 2
            return Candidate(
                source=source.index,
                source_at_start=source_at_start,
                target=target.index,
                target_at_start=target_at_start,
                kind="orthogonal",
                cost=gap_axis + max(0.0, gap_span - options.tau_span),
                priority=priority,
            )
        return None

    if abs(source.c - target.c) > options.eps_axis:
        return None
    costs = [
        (abs(source_u - target.u0), True),
        (abs(source_u - target.u1), False),
    ]
    cost, target_at_start = min(costs, key=lambda item: item[0])
    if cost <= options.tau_node:
        return Candidate(
            source=source.index,
            source_at_start=source_at_start,
            target=target.index,
            target_at_start=target_at_start,
            kind="collinear_endpoint",
            cost=cost,
            priority=1,
        )
    if _distance_to_interval(source_u, target.u0, target.u1) <= options.tau_span:
        return Candidate(
            source=source.index,
            source_at_start=source_at_start,
            target=target.index,
            target_at_start=None,
            kind="collinear_span",
            cost=abs(source.c - target.c),
            priority=3,
        )
    return None


def _add_candidate_constraints(
    dsu: "WeightedUnionFind",
    segs: Sequence[OrthoSegment],
    cand: Candidate,
):
    source = segs[cand.source]
    target = segs[cand.target]
    source_u_var = source.endpoint_var(cand.source_at_start)
    if source.orientation != target.orientation:
        dsu.union(source_u_var, target.c_var)
        if cand.target_at_start is not None:
            dsu.union(source.c_var, target.endpoint_var(cand.target_at_start))
    else:
        dsu.union(source.c_var, target.c_var)
        if cand.target_at_start is not None:
            dsu.union(source_u_var, target.endpoint_var(cand.target_at_start))


def _union_support_cluster(dsu: "WeightedUnionFind", cluster: Sequence[OrthoSegment]):
    if len(cluster) < 2:
        return
    first = cluster[0].c_var
    for seg in cluster[1:]:
        dsu.union(first, seg.c_var)


def _axis_clusters(segs: Sequence[OrthoSegment], eps_axis: float) -> list[list[OrthoSegment]]:
    if not segs:
        return []
    sorted_segs = sorted(segs, key=lambda item: item.c)
    clusters = [[sorted_segs[0]]]
    for seg in sorted_segs[1:]:
        if abs(seg.c - clusters[-1][-1].c) <= eps_axis:
            clusters[-1].append(seg)
        else:
            clusters.append([seg])
    return clusters


def _explode_lines(geometry) -> Iterable[LineString]:
    if isinstance(geometry, LineString):
        coords = list(geometry.coords)
        for start, end in zip(coords, coords[1:]):
            line = LineString([start, end])
            if line.length > 0:
                yield line
    elif isinstance(geometry, MultiLineString):
        for line in geometry.geoms:
            yield from _explode_lines(line)
    elif hasattr(geometry, "geoms"):
        for geom in geometry.geoms:
            yield from _explode_lines(geom)


def _segment_line(seg: OrthoSegment) -> LineString:
    if seg.orientation == "H":
        return LineString([(seg.u0, seg.c), (seg.u1, seg.c)])
    return LineString([(seg.c, seg.u0), (seg.c, seg.u1)])


def _endpoint_point(seg: OrthoSegment, at_start: bool) -> tuple[float, float]:
    u = seg.u0 if at_start else seg.u1
    if seg.orientation == "H":
        return (u, seg.c)
    return (seg.c, u)


def _distance_to_interval(value: float, start: float, end: float) -> float:
    if value < start:
        return start - value
    if value > end:
        return value - end
    return 0.0


def _var_weight(seg: OrthoSegment, var_kind: str) -> float:
    type_weight = 5.0 if seg.seg_type == SegmentType.WALL else 1.0
    if seg.is_structural:
        type_weight *= 2.0
    length_weight = max(seg.length, 1.0)
    if var_kind == "u":
        length_weight = max(length_weight * 0.35, 1.0)
    return type_weight * length_weight


def _weighted_median(items: Sequence[tuple[float, float]]) -> float:
    if not items:
        return 0.0
    ordered = sorted(items, key=lambda item: item[0])
    total = sum(weight for _value, weight in ordered)
    accum = 0.0
    for value, weight in ordered:
        accum += weight
        if accum >= total / 2.0:
            return value
    return ordered[-1][0]


class WeightedUnionFind:
    def __init__(self):
        self.parent = {}
        self.rank = {}
        self.values: dict[tuple[int, str], tuple[float, float]] = {}

    def add(self, item, value: float, weight: float):
        if item not in self.parent:
            self.parent[item] = item
            self.rank[item] = 0
        self.values[item] = (float(value), float(weight))

    def find(self, item):
        if self.parent[item] != item:
            self.parent[item] = self.find(self.parent[item])
        return self.parent[item]

    def union(self, first, second):
        root_a = self.find(first)
        root_b = self.find(second)
        if root_a == root_b:
            return
        if self.rank[root_a] < self.rank[root_b]:
            root_a, root_b = root_b, root_a
        self.parent[root_b] = root_a
        if self.rank[root_a] == self.rank[root_b]:
            self.rank[root_a] += 1

    def solve(self) -> dict[tuple[int, str], float]:
        groups: dict[object, list[tuple[float, float]]] = {}
        for item, value_weight in self.values.items():
            groups.setdefault(self.find(item), []).append(value_weight)
        solved_group = {root: _weighted_median(items) for root, items in groups.items()}
        return {item: solved_group[self.find(item)] for item in self.values}
