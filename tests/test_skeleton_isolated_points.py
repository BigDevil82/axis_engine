from __future__ import annotations

import unittest

from shapely.geometry import LineString

from axis_engine.skeleton_diagnostics import find_skeleton_isolated_points
from design_api.skeleton import extract_skeleton, normalize_skeleton
from design_api.web.schemas import SkeletonResponse


class SkeletonIsolatedPointsTests(unittest.TestCase):
    def test_closed_network_has_no_isolated_points(self):
        wall_axes = [
            (LineString([(0, 0), (1000, 0)]), 200.0),
            (LineString([(1000, 0), (1000, 1000)]), 200.0),
            (LineString([(1000, 1000), (0, 1000)]), 200.0),
            (LineString([(0, 1000), (0, 0)]), 200.0),
        ]

        isolated = find_skeleton_isolated_points(wall_axes, [])

        self.assertEqual(isolated, [])

    def test_normalized_skeleton_returns_dangling_endpoints(self):
        result = normalize_skeleton(
            {
                "wall_axes": [
                    {"line": [[0, 0], [1000, 0]], "thickness": 200},
                    {"line": [[1000, 0], [1000, 1000]], "thickness": 200},
                ]
            }
        )

        self.assertEqual(result["isolated_points"], [[0.0, 0.0], [1000.0, 1000.0]])
        self.assertEqual(result["diagnostics"]["isolated_point_count"], 2)
        self.assertEqual(SkeletonResponse.model_validate(result).isolated_points, result["isolated_points"])

    def test_extracted_skeleton_returns_dangling_endpoints(self):
        result = extract_skeleton(
            {
                "wall_geometries": [
                    _line([0, -100], [1000, -100]),
                    _line([0, 100], [1000, 100]),
                    _line([900, 0], [900, 1000]),
                    _line([1100, 0], [1100, 1000]),
                ]
            }
        )

        self.assertEqual(len(result["isolated_points"]), 2)
        self.assertEqual(result["diagnostics"]["isolated_point_count"], 2)


def _line(start: list[float], end: list[float]) -> dict:
    return {"geom_type": "LINE", "params": {"start": start, "end": end}}


if __name__ == "__main__":
    unittest.main()
