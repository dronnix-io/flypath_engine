"""Compatibility check for the shared grid against the QGIS reference."""

import json
import math
import os
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine.grid import find_optimal_direction, generate_grid  # noqa: E402
from flypath_engine.grid import GridCapacityError, _scan_columns  # noqa: E402
from shapely.geometry import LineString, MultiPolygon, Polygon  # noqa: E402


def test_manual_directions_match_qgis_fixture():
    fixture = json.loads((ROOT / "tests/fixtures/plugin-direction.json").read_text())
    profile = json.loads((ROOT / "profiles/mini3pro-v1.json").read_text())
    survey_area = {
        "exterior": [
            {"latitude_deg": latitude, "longitude_deg": longitude}
            for latitude, longitude in fixture["polygon"]
        ],
        "holes": [],
    }
    for case in fixture["cases"]:
        result = generate_grid(
            survey_area,
            altitude_m=80,
            shot_spacing_m=16,
            side_overlap=.7,
            direction_deg=case["direction"],
            margin_m=0,
            camera=profile["camera"],
        )
        actual = result["waypoints"]
        assert len(actual) == len(case["waypoints"]), case["direction"]
        for point, (expected_latitude, expected_longitude) in zip(actual, case["waypoints"]):
            error_m = math.hypot(
                (point["latitude_deg"] - expected_latitude) * 111_320,
                (point["longitude_deg"] - expected_longitude)
                * 111_320 * math.cos(math.radians(expected_latitude)),
            )
            assert error_m <= .05, (case["direction"], error_m)

    line_spacing = 80 * profile["camera"]["sensor_width_mm"] \
        / profile["camera"]["focal_length_mm"] * .3
    assert find_optimal_direction(survey_area, line_spacing) == fixture["autoDirection"]


def test_scan_fallback_keeps_narrow_components_and_holes():
    left = Polygon([(0, 5), (2, 0), (4, 0), (4, 10), (2, 10)],
                   holes=[[(2.5, 1), (3.5, 1), (3.5, 3), (2.5, 3)]])
    right = Polygon([(20, 0), (22, 0), (22, 10), (20, 10)])
    area = MultiPolygon([left, right])
    columns = _scan_columns(area, 30, 1, fallback_if_empty=True)
    assert len(columns) == 2
    assert all(segments for _, segments in columns)
    assert columns[0][0] < 4 and 20 < columns[1][0] < 22
    assert all(area.covers(LineString([(x, low), (x, high)]))
               for x, segments in columns for low, high in segments)
    try:
        _scan_columns(MultiPolygon([left, right]), 30, 1, max_segments=2, fallback_if_empty=True)
    except GridCapacityError:
        pass
    else:
        assert False, "segment capacity was ignored"
    try:
        _scan_columns(MultiPolygon([left, right]), 30, 1, max_steps=2, fallback_if_empty=True)
    except GridCapacityError:
        pass
    else:
        assert False, "scan capacity was ignored"


if __name__ == "__main__":
    test_manual_directions_match_qgis_fixture()
    print("PASS  test_manual_directions_match_qgis_fixture")
