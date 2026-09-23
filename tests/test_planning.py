"""Acceptance checks for the public first-phase planning boundary."""

import copy
import json
import math
from pathlib import Path
import sys
import time

from pyproj import Transformer
from shapely.geometry import LineString, Polygon
from shapely.ops import transform

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine import PlanningError, plan_2d  # noqa: E402
from flypath_engine.measurements import route_distance_m  # noqa: E402


def _request():
    request = json.loads((ROOT / "tests/fixtures/reported-mission-126-input.json").read_text())
    request["split"]["enabled"] = True
    return request


def _full(request=None):
    request = copy.deepcopy(request or _request())
    request["capture"] = {"mode": "full_auto", "front_overlap_ratio": .7}
    return request


def _error(request, code, field=None):
    try:
        plan_2d(request)
    except PlanningError as exc:
        assert exc.code == code, exc.as_dict()
        if field:
            assert exc.field == field, exc.as_dict()
    else:
        raise AssertionError(f"Expected {code}")


def test_contract_versions_numbers_and_unsupported_features():
    for field in ("contract_version", "profile_version"):
        request = _request()
        request[field] = 99
        _error(request, "unsupported_version", field)
    request = _request()
    request["speed_m_s"] = True
    _error(request, "invalid_number", "speed_m_s")
    request = _request()
    request["survey_area"]["exterior"][0]["latitude_deg"] = "51"
    _error(request, "invalid_number", "survey_area.exterior.0.latitude_deg")
    request = _request()
    request["cross_hatch"] = "yes"
    _error(request, "invalid_boolean", "cross_hatch")
    request = _request()
    request["locations"] = {"shared": {"hom": {}}}
    _error(request, "unknown_field", "locations.shared.hom")


def test_direction_is_normalized_before_geometry_and_is_deterministic():
    request = _request()
    request["direction"]["value_deg"] = 250
    result = plan_2d(request)
    normalized = copy.deepcopy(request)
    normalized["direction"]["value_deg"] = 70
    assert result == plan_2d(normalized)
    assert result["resolved_direction"]["value_deg"] == 70


def test_cross_hatch_and_reverse_preserve_current_2d_workflows():
    normal = plan_2d(_request())
    cross_request = _request()
    cross_request["cross_hatch"] = True
    crossed = plan_2d(cross_request)
    assert len(crossed["route"]["strips"]) > len(normal["route"]["strips"])
    reverse_request = _request()
    reverse_request["reverse_route"] = True
    reversed_result = plan_2d(reverse_request)
    normal_positions = [waypoint["position"] for waypoint in normal["route"]["waypoints"]]
    reversed_positions = [waypoint["position"] for waypoint in reversed_result["route"]["waypoints"]]
    assert reversed_positions == normal_positions[::-1]


def test_hole_strips_retain_boundaries_after_full_auto_densification():
    request = _full()
    request["survey_area"] = {
        "exterior": [
            {"latitude_deg": 51.000, "longitude_deg": -114.010},
            {"latitude_deg": 51.000, "longitude_deg": -113.990},
            {"latitude_deg": 51.015, "longitude_deg": -113.990},
            {"latitude_deg": 51.015, "longitude_deg": -114.010},
        ],
        "holes": [[
            {"latitude_deg": 51.005, "longitude_deg": -114.004},
            {"latitude_deg": 51.005, "longitude_deg": -113.996},
            {"latitude_deg": 51.010, "longitude_deg": -113.996},
            {"latitude_deg": 51.010, "longitude_deg": -114.004},
        ]],
    }
    result = plan_2d(request)
    assert len(result["route"]["strips"]) > 2
    hole = Polygon([(point["longitude_deg"], point["latitude_deg"])
                    for point in request["survey_area"]["holes"][0]])
    project = Transformer.from_crs("EPSG:4326", "EPSG:32612", always_xy=True).transform
    hole_interior = transform(project, hole).buffer(-.2)
    for strip in result["route"]["strips"]:
        start = result["route"]["waypoints"][strip["start_waypoint_index"]]
        end = result["route"]["waypoints"][strip["end_waypoint_index"]]
        assert "survey_strip_start" in start["roles"]
        assert "survey_strip_end" in end["roles"]
        line = LineString([
            (result["route"]["waypoints"][index]["position"]["longitude_deg"],
             result["route"]["waypoints"][index]["position"]["latitude_deg"])
            for index in range(strip["start_waypoint_index"], strip["end_waypoint_index"] + 1)
        ])
        assert transform(project, line).intersection(hole_interior).is_empty


def test_full_auto_split_seams_get_fresh_startup_and_photo_actions():
    request = _full()
    request["split"]["requested_flights"] = 3
    request["split"]["max_waypoints_per_flight"] = 200
    result = plan_2d(request)
    assert len(result["flights"]) == 3
    for flight in result["flights"]:
        assert [action["type"] for action in flight["actions"][:3]] == [
            "rotate_camera", "hover", "take_photo"
        ]
        assert flight["startup_wait_seconds"] == 3
        assert flight["photo_count"] == sum(
            action["type"] == "take_photo" for action in flight["actions"]
        )
    for first, second in zip(result["flights"], result["flights"][1:]):
        assert first["route_end_waypoint_index"] == second["route_start_waypoint_index"]
    assert result["statistics"]["photo_count"] == len(result["route"]["waypoints"]) + 2


def test_waypoint_and_battery_limits_block_or_add_flights():
    disabled = _full()
    disabled["split"].update(enabled=False, max_waypoints_per_flight=30)
    blocked = plan_2d(disabled)
    assert not blocked["validation"]["export_allowed"]
    assert {error["code"] for error in blocked["validation"]["errors"]} >= {
        "splitting_required_waypoint_limit", "waypoint_limit_exceeded"
    }

    enabled = _full()
    enabled["split"]["max_waypoints_per_flight"] = 30
    planned = plan_2d(enabled)
    assert planned["validation"]["export_allowed"]
    assert all(flight["waypoint_count"] <= 30 for flight in planned["flights"])

    time_limited = _request()
    time_limited["speed_m_s"] = 1
    by_time = plan_2d(time_limited)
    assert len(by_time["flights"]) > 1
    assert all(flight["known_estimated_seconds"] <= flight["usable_battery_seconds"]
               for flight in by_time["flights"])

    impossible = _request()
    impossible["locations"] = {"shared": {
        "launch": {"latitude_deg": 40, "longitude_deg": -100},
        "home": {"latitude_deg": 40, "longitude_deg": -100},
    }}
    impossible_result = plan_2d(impossible)
    assert not impossible_result["validation"]["export_allowed"]
    assert "flight_cannot_fit" in {error["code"] for error in impossible_result["validation"]["errors"]}

    disabled_home = copy.deepcopy(impossible)
    disabled_home["split"]["enabled"] = False
    disabled_result = plan_2d(disabled_home)
    assert "splitting_required_battery_limit" in {
        error["code"] for error in disabled_result["validation"]["errors"]
    }


def test_two_flight_boundary_minimizes_the_worst_known_time():
    request = _full()
    request["split"].update(requested_flights=2, max_waypoints_per_flight=200)
    result = plan_2d(request)
    assert len(result["flights"]) == 2
    points = [waypoint["position"] for waypoint in result["route"]["waypoints"]]
    interval = result["capture"]["profile_interval_s"]
    allowance = max(2.5, interval)

    def seconds(start, end):
        return route_distance_m(points[start:end + 1]) / request["speed_m_s"] + (end - start + 1) * allowance + 3

    legal_worst = [max(seconds(0, seam), seconds(seam, len(points) - 1))
                   for seam in range(1, len(points) - 1)]
    actual = max(flight["known_estimated_seconds"] for flight in result["flights"])
    assert math.isclose(actual, min(legal_worst), abs_tol=1e-9)


def test_semi_auto_rounds_each_flight_route_and_excludes_home_travel():
    request = _request()
    request["split"]["requested_flights"] = 2
    request["locations"] = {"shared": {
        "launch": {"latitude_deg": 51.1, "longitude_deg": -114.1},
        "home": {"latitude_deg": 51.1, "longitude_deg": -114.1},
    }}
    result = plan_2d(request)
    interval = result["capture"]["profile_interval_s"]
    expected = sum(math.floor((flight["route_distance_m"] / request["speed_m_s"]) / interval)
                   for flight in result["flights"])
    assert result["statistics"]["photo_count"] == expected
    for flight in result["flights"]:
        assert flight["actions"] == [{
            "type": "rotate_camera",
            "waypoint_index": flight["route_start_waypoint_index"],
            "pitch_deg": -90,
        }]
        assert flight["photo_count"] == math.floor(
            (flight["route_distance_m"] / request["speed_m_s"]) / interval
        )
    assert result["statistics"]["estimates_complete"]
    assert result["statistics"]["total_distance_m"] > result["statistics"]["route_distance_m"]


def test_missing_and_per_flight_locations_are_explicit():
    missing = plan_2d(_request())
    assert missing["statistics"]["total_distance_m"] is None
    assert not missing["statistics"]["estimates_complete"]
    assert missing["validation"]["export_allowed"]

    no_return_request = _request()
    no_return_request["finish_action"] = "hover"
    no_return = plan_2d(no_return_request)
    assert no_return["statistics"]["recovery_distance_m"] == 0
    assert no_return["statistics"]["total_distance_m"] is None

    first_request = _request()
    first_request["finish_action"] = "return_to_first_waypoint"
    first = plan_2d(first_request)
    first_flight = first["flights"][0]
    first_points = [waypoint["position"] for waypoint in first["route"]["waypoints"]]
    expected_recovery = route_distance_m([
        first_points[first_flight["route_end_waypoint_index"]],
        first_points[first_flight["route_start_waypoint_index"]],
    ])
    assert math.isclose(first_flight["recovery_distance_m"], expected_recovery)
    assert math.isclose(first["statistics"]["recovery_distance_m"], expected_recovery)
    assert first["statistics"]["total_distance_m"] is None  # launch remains unknown

    request = _full()
    request["split"]["requested_flights"] = 2
    request["split"]["max_waypoints_per_flight"] = 30
    request["locations"] = {"per_flight": [{}, {}]}
    result = plan_2d(request)
    assert not result["validation"]["export_allowed"]
    assert "per_flight_locations_need_review" in {
        error["code"] for error in result["validation"]["errors"]
    }


def test_profile_interval_and_per_flight_locations_are_authoritative():
    interval_request = _full()
    interval_request["drone_profile_id"] = "air3s"
    interval_result = plan_2d(interval_request)
    assert interval_result["capture"]["profile_interval_s"] == 5
    assert all(
        flight["capture_allowance_seconds"] == flight["photo_count"] * 5
        for flight in interval_result["flights"]
    )
    overridden = copy.deepcopy(interval_request)
    overridden["capture"]["interval_s"] = 2
    _error(overridden, "profile_interval_override", "capture.interval_s")

    request = _request()
    request["split"]["requested_flights"] = 2
    request["locations"] = {"per_flight": [
        {
            "launch": {"latitude_deg": 51.02, "longitude_deg": -114.04},
            "home": {"latitude_deg": 51.021, "longitude_deg": -114.041},
        },
        {
            "launch": {"latitude_deg": 51.00, "longitude_deg": -114.01},
            "home": {"latitude_deg": 50.999, "longitude_deg": -114.009},
        },
    ]}
    result = plan_2d(request)
    assert len(result["flights"]) == 2
    assert result["statistics"]["estimates_complete"]
    assert result["validation"]["export_allowed"]
    for index, flight in enumerate(result["flights"]):
        assert flight["outbound_distance_m"] > 0
        assert flight["recovery_distance_m"] > 0
        assert math.isclose(
            flight["total_distance_m"],
            flight["route_distance_m"]
            + flight["outbound_distance_m"]
            + flight["recovery_distance_m"],
        )
    assert math.isclose(
        result["statistics"]["total_distance_m"],
        sum(flight["total_distance_m"] for flight in result["flights"]),
    )


def test_large_full_auto_plan_is_bounded():
    request = _full()
    request["capture"]["front_overlap_ratio"] = .95
    request["split"]["max_waypoints_per_flight"] = 200
    started = time.monotonic()
    result = plan_2d(request)
    elapsed = time.monotonic() - started
    assert len(result["route"]["waypoints"]) > 500
    assert elapsed < 3, elapsed

    over_capacity = _full()
    over_capacity["capture"]["front_overlap_ratio"] = .9999
    over_capacity["cross_hatch"] = True
    _error(over_capacity, "capacity_exceeded", "survey_area")


def test_antimeridian_and_invalid_geometry():
    request = _request()
    request["survey_area"] = {"exterior": [
        {"latitude_deg": 0, "longitude_deg": 179.99},
        {"latitude_deg": 0, "longitude_deg": -179.99},
        {"latitude_deg": .01, "longitude_deg": -179.99},
        {"latitude_deg": .01, "longitude_deg": 179.99},
    ], "holes": []}
    result = plan_2d(request)
    assert 2_400_000 < result["statistics"]["survey_area_m2"] < 2_500_000

    request["survey_area"] = {"exterior": [
        {"latitude_deg": 10, "longitude_deg": 179.998},
        {"latitude_deg": 10, "longitude_deg": -179.998},
        {"latitude_deg": 10.004, "longitude_deg": -179.998},
        {"latitude_deg": 10.004, "longitude_deg": 179.998},
    ], "holes": [[
        {"latitude_deg": 10.001, "longitude_deg": -179.999},
        {"latitude_deg": 10.001, "longitude_deg": 179.999},
        {"latitude_deg": 10.003, "longitude_deg": 179.999},
        {"latitude_deg": 10.003, "longitude_deg": -179.999},
    ]]}
    with_hole = plan_2d(request)
    assert 140_000 < with_hole["statistics"]["survey_area_m2"] < 150_000

    invalid = _request()
    invalid["survey_area"]["exterior"] = [
        {"latitude_deg": 0, "longitude_deg": 0},
        {"latitude_deg": 1, "longitude_deg": 1},
        {"latitude_deg": 0, "longitude_deg": 1},
        {"latitude_deg": 1, "longitude_deg": 0},
    ]
    _error(invalid, "invalid_geometry", "survey_area")


if __name__ == "__main__":
    functions = [value for name, value in sorted(globals().items())
                 if name.startswith("test_") and callable(value)]
    for function in functions:
        function()
        print(f"PASS  {function.__name__}")
