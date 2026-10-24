"""Acceptance checks for the oblique planning boundary (grid passes, tilted camera)."""

import copy
import json
import math
from pathlib import Path
import sys

import pytest
from pyproj import Geod

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine import PlanningError, plan_2d, plan_oblique  # noqa: E402

GEOD = Geod(ellps="WGS84")
AREA = {"exterior": [
    {"latitude_deg": 51.05, "longitude_deg": -114.09},
    {"latitude_deg": 51.05, "longitude_deg": -114.087},
    {"latitude_deg": 51.0515, "longitude_deg": -114.087},
    {"latitude_deg": 51.0515, "longitude_deg": -114.09},
]}


def _request(mode="full_auto", **overrides):
    request = {
        "contract_version": 1,
        "operation": "plan_oblique",
        "mapping_style": "oblique",
        "profile_version": 1,
        "drone_profile_id": "mini4pro",
        "survey_area": copy.deepcopy(AREA),
        "altitude_m": 60,
        "speed_m_s": 5,
        "side_overlap_ratio": .7,
        "margin_m": 0,
        "direction": {"mode": "manual", "value_deg": 0},
        "capture": {"mode": mode},
        "gimbal_pitch_deg": -45,
        "turn_style": "curved",
        "finish_action": "return_to_home",
        "split": {"enabled": True, "requested_flights": 1, "max_waypoints_per_flight": 200},
    }
    if mode == "full_auto":
        request["capture"]["front_overlap_ratio"] = .8
    request.update(overrides)
    return request


def _centroid(points):
    return (sum(point["longitude_deg"] for point in points) / len(points),
            sum(point["latitude_deg"] for point in points) / len(points))


def _pass_points(result, survey_pass):
    rows = result["route"]["waypoints"]
    return [row["position"] for row in rows[survey_pass["start_waypoint_index"]:survey_pass["end_waypoint_index"] + 1]]


def test_five_passes_nadir_first_then_four_looks():
    result = plan_oblique(_request())
    passes = result["oblique"]["passes"]
    assert [p["kind"] for p in passes] == ["nadir", "forward", "right", "back", "left"]
    assert passes[0]["gimbal_pitch_deg"] == -90 and passes[0]["heading_deg"] is None
    assert all(p["gimbal_pitch_deg"] == -45 for p in passes[1:])
    looks = [p["look_bearing_deg"] for p in passes[1:]]
    assert [(look - looks[0]) % 360 for look in looks] == [0, 90, 180, 270]
    assert result["operation"] == "plan_oblique" and result["mapping_style"] == "oblique"
    assert result["engine_version"] == "1.4.0"
    assert result["validation"]["export_allowed"] is True
    json.dumps(result)


def test_without_nadir_only_four_oblique_passes():
    result = plan_oblique(_request(include_nadir=False))
    assert [p["kind"] for p in result["oblique"]["passes"]] == ["forward", "right", "back", "left"]
    assert all(row["heading_deg"] is not None for row in result["route"]["waypoints"])


def test_pitch_is_user_defined_and_drives_offset_and_slant():
    for pitch in (-30, -45, -60, -75):
        oblique = plan_oblique(_request(gimbal_pitch_deg=pitch))["oblique"]
        tilt = math.radians(90 + pitch)
        assert oblique["gimbal_pitch_deg"] == pitch
        assert oblique["offset_m"] == pytest.approx(60 * math.tan(tilt))
        assert oblique["slant_distance_m"] == pytest.approx(60 / math.cos(tilt))


@pytest.mark.parametrize("pitch", [-90, -86, -19, 0, 10])
def test_pitch_outside_range_fails(pitch):
    with pytest.raises(PlanningError) as error:
        plan_oblique(_request(gimbal_pitch_deg=pitch))
    assert error.value.code == "number_out_of_range" and error.value.field == "gimbal_pitch_deg"


def test_each_oblique_pass_moves_against_its_look_by_the_offset():
    result = plan_oblique(_request(direction={"mode": "manual", "value_deg": 0}))
    area_lon, area_lat = _centroid(AREA["exterior"])
    nadir = result["oblique"]["passes"][0]
    nadir_lon, nadir_lat = _centroid(_pass_points(result, nadir))
    for survey_pass in result["oblique"]["passes"][1:]:
        lon, lat = _centroid(_pass_points(result, survey_pass))
        azimuth, _back, distance = GEOD.inv(nadir_lon, nadir_lat, lon, lat)
        # The pass sits behind the camera: opposite the heading, by the offset.
        assert abs(((azimuth - survey_pass["heading_deg"]) % 360) - 180) < 8
        assert distance == pytest.approx(result["oblique"]["offset_m"], rel=.25)
    assert GEOD.inv(area_lon, area_lat, nadir_lon, nadir_lat)[2] < 15


def test_waypoints_carry_pass_heading_and_flights_tilt_the_camera():
    result = plan_oblique(_request())
    rows = result["route"]["waypoints"]
    for index, survey_pass in enumerate(result["oblique"]["passes"]):
        span = rows[survey_pass["start_waypoint_index"]:survey_pass["end_waypoint_index"] + 1]
        assert {row["pass_index"] for row in span} == {index}
        assert {row["heading_deg"] for row in span} == {survey_pass["heading_deg"]}
        assert "pass_start" in span[0]["roles"] and "pass_end" in span[-1]["roles"]
    for flight in result["flights"]:
        pass_index = rows[flight["route_start_waypoint_index"]]["pass_index"]
        assert rows[flight["route_end_waypoint_index"]]["pass_index"] == pass_index
        pitch = result["oblique"]["passes"][pass_index]["gimbal_pitch_deg"]
        assert flight["actions"][0] == {"type": "rotate_camera", "waypoint_index": flight["route_start_waypoint_index"],
                                        "pitch_deg": pitch}
        assert flight["actions"][1]["type"] == "hover"
        photos = [action for action in flight["actions"] if action["type"] == "take_photo"]
        assert len(photos) == flight["waypoint_count"]


def test_flights_follow_each_other_and_cover_the_route():
    result = plan_oblique(_request(split={"enabled": True, "requested_flights": 1, "max_waypoints_per_flight": 40}))
    rows = result["route"]["waypoints"]
    flights = result["flights"]
    assert flights[0]["route_start_waypoint_index"] == 0
    assert flights[-1]["route_end_waypoint_index"] == len(rows) - 1
    for previous, flight in zip(flights, flights[1:]):
        start = flight["route_start_waypoint_index"]
        if start == previous["route_end_waypoint_index"] + 1:
            assert "pass_start" in rows[start]["roles"]
        else:
            assert start == previous["route_end_waypoint_index"]
    assert all(flight["waypoint_count"] <= 40 for flight in flights)
    assert len(flights) > 5
    assert result["statistics"]["battery_count"] == len(flights)
    # A seam inside a pass is photographed by both of its flights, as in plan_2d.
    seams = sum(flight["route_start_waypoint_index"] == previous["route_end_waypoint_index"]
                for previous, flight in zip(flights, flights[1:]))
    assert result["statistics"]["photo_count"] == len(rows) + seams


def test_requested_flights_beyond_passes_adds_flights():
    result = plan_oblique(_request(split={"enabled": True, "requested_flights": 8, "max_waypoints_per_flight": 200}))
    assert len(result["flights"]) == 8


def test_unsplit_pass_over_the_waypoint_limit_needs_splitting():
    result = plan_oblique(_request(split={"enabled": False, "max_waypoints_per_flight": 30}))
    codes = [error["code"] for error in result["validation"]["errors"]]
    assert codes == ["splitting_required_waypoint_limit"]
    assert len(result["flights"]) == 5


def test_semi_auto_uses_speed_and_interval_and_estimates_photos():
    result = plan_oblique(_request("semi_auto"))
    assert result["capture"]["mode"] == "semi_auto"
    assert result["statistics"]["photo_count_kind"] == "estimate"
    for flight in result["flights"]:
        assert [action["type"] for action in flight["actions"]] == ["rotate_camera"]


def test_oblique_spacing_uses_the_slant_distance():
    result = plan_oblique(_request())
    nadir, forward = result["oblique"]["passes"][:2]
    ratio = result["oblique"]["slant_distance_m"] / 60
    assert forward["line_spacing_m"] == pytest.approx(nadir["line_spacing_m"] * ratio)
    assert forward["shot_spacing_m"] == pytest.approx(nadir["shot_spacing_m"] * ratio)


def test_nadir_pass_matches_plan_2d():
    oblique = plan_oblique(_request(split={"enabled": False, "max_waypoints_per_flight": 200}))
    request = _request(split={"enabled": False, "requested_flights": 1, "max_waypoints_per_flight": 200})
    for key in ("gimbal_pitch_deg",):
        request.pop(key)
    request.update(operation="plan_2d", mapping_style="2d")
    flat = plan_2d(request)
    nadir = oblique["oblique"]["passes"][0]
    assert _pass_points(oblique, nadir) == [row["position"] for row in flat["route"]["waypoints"]]


def test_deterministic():
    assert plan_oblique(_request()) == plan_oblique(_request())


@pytest.mark.parametrize("overrides, code, field", [
    ({"terrain_follow": True}, "unsupported_feature", "terrain_follow"),
    ({"include_nadir": "yes"}, "invalid_boolean", "include_nadir"),
    ({"mapping_style": "2d"}, "unsupported_feature", "mapping_style"),
    ({"operation": "plan_2d"}, "unsupported_operation", "operation"),
    ({"cross_hatch": True}, "unknown_field", "request.cross_hatch"),
    ({"locations": {"per_flight": [{}]}}, "unsupported_feature", "locations.per_flight"),
])
def test_invalid_requests_fail(overrides, code, field):
    with pytest.raises(PlanningError) as error:
        plan_oblique(_request(**overrides))
    assert (error.value.code, error.value.field) == (code, field)
