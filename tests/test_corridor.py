"""Contract-2 corridor geometry, capture, splitting, and terrain replay."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys

import pytest
from pyproj import Transformer

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from flypath_engine import PlanningError, corridor_sample_points, plan_corridor
from flypath_engine.precision import number
from flypath_engine.measurements import route_distance_m


def request():
    return {
        "contract_version": 2, "operation": "plan_corridor", "mapping_style": "corridor",
        "profile_version": 1, "drone_profile_id": "mini4pro",
        "centre_line": [
            {"latitude_deg": 51.0, "longitude_deg": -114.0},
            {"latitude_deg": 51.0006, "longitude_deg": -114.0},
            {"latitude_deg": 51.0006, "longitude_deg": -113.9994},
        ],
        "width_m": 90.0, "altitude_m": 40.0, "speed_m_s": 5.0,
        "side_overlap_ratio": .5, "capture": {"mode": "semi_auto"},
        "turn_style": "curved", "finish_action": "hover",
        "split": {"enabled": True, "requested_flights": 1, "max_waypoints_per_flight": 200},
    }


def test_symmetric_offsets_and_bends():
    result = plan_corridor(request())
    offsets = result["corridor"]["pass_offsets_m"]
    assert offsets == [-offset for offset in reversed(offsets)]
    assert len(offsets) >= 2
    assert len(result["route"]["strips"]) == len(offsets)
    assert all(strip["end_waypoint_index"] - strip["start_waypoint_index"] >= 2
               for strip in result["route"]["strips"])
    assert result["validation"]["export_allowed"]
    assert result["statistics"]["survey_area_m2"] > 0


def test_full_auto_photos_and_reverse():
    original = request()
    original["capture"] = {"mode": "full_auto", "front_overlap_ratio": .7}
    forward = plan_corridor(original)
    reverse = deepcopy(original)
    reverse["reverse_route"] = True
    backward = plan_corridor(reverse)
    assert len(forward["route"]["waypoints"]) > len(plan_corridor(request())["route"]["waypoints"])
    assert [item["position"] for item in backward["route"]["waypoints"]] == [
        item["position"] for item in reversed(forward["route"]["waypoints"])]
    assert forward["statistics"]["photo_count"] == sum(
        action["type"] == "take_photo" for flight in forward["flights"] for action in flight["actions"])


def test_hard_break_and_split():
    value = request()
    value["break_indices"] = [1]
    value["split"] = {"enabled": True, "requested_flights": 3, "max_waypoints_per_flight": 4}
    result = plan_corridor(value)
    groups = [index for index, waypoint in enumerate(result["route"]["waypoints"])
              if "corridor_group_start" in waypoint["roles"]]
    assert len(groups) == 2
    assert len(result["flights"]) >= 3
    assert all(not (flight["route_start_waypoint_index"] < groups[1] <= flight["route_end_waypoint_index"])
               for flight in result["flights"])
    reversed_value = deepcopy(value)
    reversed_value["reverse_route"] = True
    backward = plan_corridor(reversed_value)
    assert [waypoint["position"] for waypoint in backward["route"]["waypoints"]] == [
        waypoint["position"] for waypoint in reversed(result["route"]["waypoints"])]
    assert backward["corridor"]["pass_offsets_m"] == list(reversed(result["corridor"]["pass_offsets_m"]))


def test_terrain_samples_and_exact_replay():
    value = request()
    value["terrain_follow"] = {"tolerance_m": 2.0}
    samples = corridor_sample_points(value)
    assert len(samples) > len(plan_corridor(request())["route"]["waypoints"])
    with pytest.raises(PlanningError, match="One elevation"):
        plan_corridor(value)
    value["terrain_follow"]["elevations_m"] = [100 + (index // 4) * 3 for index in range(len(samples))]
    result = plan_corridor(value)
    assert all("ground_elevation_m" in waypoint for waypoint in result["route"]["waypoints"])
    assert result == plan_corridor(json.loads(json.dumps(value)))
    assert len(result["route"]["waypoints"]) > len(plan_corridor(request())["route"]["waypoints"])


def test_invalid_input_and_capacity():
    value = request()
    value["centre_line"].append(value["centre_line"][0])
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "invalid_geometry"
    value = request()
    value["contract_version"] = 1
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "unsupported_version"
    value = request()
    value["width_m"] = 5000
    value["side_overlap_ratio"] = .99
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "capacity_exceeded"

def test_terrain_input_and_contract_context():
    value = request()
    value["terrain_follow"] = {"tolerance_m": 2.0}
    samples = corridor_sample_points(value)
    for invalid in (True, float("nan")):
        bad = deepcopy(value)
        bad["terrain_follow"]["elevations_m"] = [100.0] * len(samples)
        bad["terrain_follow"]["elevations_m"][0] = invalid
        with pytest.raises(PlanningError) as error:
            plan_corridor(bad)
        assert error.value.code == "invalid_number"
    assert number(1.123456789) == 1.123456789
    with pytest.raises(PlanningError):
        plan_corridor(value)
    assert number(1.123456789) == 1.123456789


def test_break_and_capacity_validation():
    for breaks in ([0], [2], [True], [1, 1]):
        value = request()
        value["break_indices"] = breaks
        with pytest.raises(PlanningError) as error:
            plan_corridor(value)
        assert error.value.code in ("invalid_break_indices", "number_out_of_range", "invalid_number")
    value = request()
    value["split"]["requested_flights"] = 41
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "capacity_exceeded"
    value = request()
    value["split"] = {"enabled": False, "max_waypoints_per_flight": 2}
    result = plan_corridor(value)
    assert not result["validation"]["export_allowed"]
    assert any(issue["code"] == "splitting_required_waypoint_limit" for issue in result["validation"]["errors"])


def test_single_pass_and_full_auto_terrain():
    value = request()
    value["width_m"] = 10.0
    assert plan_corridor(value)["corridor"]["pass_offsets_m"] == [0.0]
    value["capture"] = {"mode": "full_auto", "front_overlap_ratio": .7}
    base = plan_corridor(value)
    value["terrain_follow"] = {"tolerance_m": 2.0}
    samples = corridor_sample_points(value)
    assert samples == [waypoint["position"] for waypoint in base["route"]["waypoints"]]
    value["terrain_follow"]["elevations_m"] = list(range(len(samples)))
    terrain = plan_corridor(value)
    assert [waypoint["position"] for waypoint in terrain["route"]["waypoints"]] == samples
    assert [waypoint["ground_elevation_m"] for waypoint in terrain["route"]["waypoints"]] == list(range(len(samples)))


def test_disconnected_offset_and_terrain_sample_capacity():
    value = request()
    value["centre_line"] = [
        {"latitude_deg": 51.0, "longitude_deg": -114.0},
        {"latitude_deg": 51.0005, "longitude_deg": -114.0},
        {"latitude_deg": 51.0005, "longitude_deg": -113.99995},
        {"latitude_deg": 51.0, "longitude_deg": -113.99995},
    ]
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "invalid_offset"
    value = request()
    value["centre_line"] = [
        {"latitude_deg": latitude, "longitude_deg": -114.0} for latitude in (51.0, 52.0, 53.0)]
    value["width_m"] = 10.0
    value["terrain_follow"] = {"tolerance_m": 2.0}
    with pytest.raises(PlanningError) as error:
        corridor_sample_points(value)
    assert error.value.code == "capacity_exceeded"


def test_requested_flight_goes_to_longest_estimated_group():
    value = request()
    # Short first group has many photo stops; the longer second group has few.
    value["centre_line"] = [
        {"latitude_deg": 51.0 + index * .00003,
         "longitude_deg": -114.0 + (index % 2) * .00001}
        for index in range(20)
    ] + [{"latitude_deg": 51.0017, "longitude_deg": -114.0}]
    value["break_indices"] = [19]
    value["width_m"] = 10.0
    value["capture"] = {"mode": "full_auto", "front_overlap_ratio": .2}
    unsplit = plan_corridor(value)
    assert len(unsplit["flights"]) == 2
    assert unsplit["flights"][0]["route_distance_m"] < unsplit["flights"][1]["route_distance_m"]
    assert unsplit["flights"][0]["known_estimated_seconds"] > unsplit["flights"][1]["known_estimated_seconds"]
    value["split"]["requested_flights"] = 3
    result = plan_corridor(value)
    starts = [index for index, waypoint in enumerate(result["route"]["waypoints"])
              if "corridor_group_start" in waypoint["roles"]]
    assert len(starts) == 2
    assert len(result["flights"]) == 3
    assert sum(flight["route_end_waypoint_index"] < starts[1] for flight in result["flights"]) == 2


def test_shared_consumer_2000_waypoint_boundary():
    value = request()
    value["width_m"] = 10.0
    value["centre_line"] = value["centre_line"][:2]
    value["capture"] = {"mode": "full_auto", "front_overlap_ratio": .7}
    spacing = plan_corridor(value)["capture"]["shot_spacing_m"]
    forward = Transformer.from_crs("EPSG:4326", "EPSG:32612", always_xy=True)
    inverse = Transformer.from_crs("EPSG:32612", "EPSG:4326", always_xy=True)
    easting, northing = forward.transform(-114.0, 51.0)

    def endpoint(distance):
        longitude, latitude = inverse.transform(easting, northing + distance)
        return {"latitude_deg": latitude, "longitude_deg": longitude}

    value["centre_line"][1] = endpoint(1998.5 * spacing)
    assert len(plan_corridor(value)["route"]["waypoints"]) == 2_000
    value["centre_line"][1] = endpoint(1999.5 * spacing)
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "capacity_exceeded"

    value["capture"] = {"mode": "semi_auto"}
    value["terrain_follow"] = {"tolerance_m": 2.0}  # 10 m lookup interval
    value["centre_line"][1] = endpoint(1999 * 10 - .5)
    assert len(corridor_sample_points(value)) == 2_000
    value["centre_line"][1] = endpoint(2001 * 10)
    with pytest.raises(PlanningError) as error:
        corridor_sample_points(value)
    assert error.value.code == "capacity_exceeded"


def test_requested_flights_split_long_corridor():
    value = request()
    value["width_m"] = 10.0
    value["centre_line"] = value["centre_line"][:2]
    value["centre_line"][1]["latitude_deg"] = 51.01
    value["capture"] = {"mode": "full_auto", "front_overlap_ratio": .7}
    value["split"]["requested_flights"] = 8
    result = plan_corridor(value)
    assert len(result["flights"]) == 8
    assert result["validation"]["export_allowed"]


def test_collapsed_corridor_band_has_planning_error():
    value = request()
    value["width_m"] = .001
    value["centre_line"] = value["centre_line"][:2]
    forward = Transformer.from_crs("EPSG:4326", "EPSG:32612", always_xy=True)
    inverse = Transformer.from_crs("EPSG:32612", "EPSG:4326", always_xy=True)
    easting, northing = forward.transform(-114.0, 51.0)
    longitude, latitude = inverse.transform(easting, northing + .01)
    value["centre_line"][1] = {"latitude_deg": latitude, "longitude_deg": longitude}
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "invalid_geometry"
    assert error.value.field == "width_m"

def test_antimeridian_terrain_samples_follow_short_legs():
    value = request()
    value.update(centre_line=[
        {"latitude_deg": 0.0, "longitude_deg": 179.9999},
        {"latitude_deg": .001, "longitude_deg": 179.9999},
    ], altitude_m=80.0, width_m=300.0, side_overlap_ratio=.7,
        terrain_follow={"tolerance_m": 5.0})
    samples = corridor_sample_points(value)
    assert len(samples) == 48
    assert all(abs(point["longitude_deg"]) > 179 for point in samples)
    assert all(route_distance_m([first, second]) <= 25.01
               for first, second in zip(samples, samples[1:]))
    value["terrain_follow"]["elevations_m"] = [100.0] * len(samples)
    result = plan_corridor(value)
    assert result["validation"]["export_allowed"]
    assert all(abs(waypoint["position"]["longitude_deg"]) > 179
               for waypoint in result["route"]["waypoints"])


def test_fingerprint():
    cases = {
        "semi": "c8ea2b534d4d281e5f735ecf7176b84af1c16bd7837d9de3e6302c8ed73153e2",
        "full": "65fd1640ec5c3a33296084287a519009a48517dbd5c7f9e3365f2fe8f1b4c350",
        "break_reverse": "e75757de551c2de28005b7d2051748d9ad889791c73ab27e1e98bd25d4b9f9cf",
        "terrain": "b24038153cee41ca64b602195a49ecb23facd1c65479c8de697acfcdc1940daf",
    }
    for name, expected in cases.items():
        value = request()
        if name == "full":
            value["capture"] = {"mode": "full_auto", "front_overlap_ratio": .7}
        elif name == "break_reverse":
            value["break_indices"] = [1]
            value["reverse_route"] = True
            value["split"].update(requested_flights=3, max_waypoints_per_flight=4)
        elif name == "terrain":
            value["terrain_follow"] = {"tolerance_m": 2.0}
            samples = corridor_sample_points(value)
            value["terrain_follow"]["elevations_m"] = [100 + (index // 4) * 3 for index in range(len(samples))]
        digest = hashlib.sha256(json.dumps(plan_corridor(value), sort_keys=True, separators=(",", ":"),
                                           allow_nan=False).encode()).hexdigest()
        assert digest == expected, name
