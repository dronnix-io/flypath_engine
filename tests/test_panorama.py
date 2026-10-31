"""Acceptance checks for panorama planning: small outward-facing laps around each point."""

import copy
import json
import math
from pathlib import Path
import sys

from pyproj import Geod
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine import PlanningError, plan_panorama  # noqa: E402

GEOD = Geod(ellps="WGS84")
CENTRE = {"latitude_deg": 51.05, "longitude_deg": -114.09}


def _request(**overrides):
    request = {
        "contract_version": 2,
        "operation": "plan_panorama",
        "mapping_style": "panorama",
        "profile_version": 1,
        "drone_profile_id": "mini4pro",
        "points": [dict(CENTRE)],
        "altitude_m": 60,
        "speed_m_s": 2,
        "capture": {"mode": "full_auto", "overlap_ratio": .3},
        "finish_action": "return_to_home",
    }
    request.update(overrides)
    return request


def _rows(result, point=0):
    return [(row["gimbal_pitch_deg"], row["photo_count"], row["radius_m"])
            for row in result["panorama"]["points"][point]["rows"]]


def test_sphere_rows_from_horizon_to_nadir():
    result = plan_panorama(_request())
    assert result["operation"] == "plan_panorama" and result["mapping_style"] == "panorama"
    assert result["engine_version"] == "2.1.0" and result["contract_version"] == 2
    assert _rows(result) == [(0.0, 8, 3.0), (-30.0, 8, 4.0), (-60.0, 7, 5.0), (-90.0, 1, 0.0)]
    assert result["statistics"]["photo_count"] == result["statistics"]["waypoint_count"] == 24
    assert result["statistics"]["photo_count_kind"] == "actions"
    assert result["validation"] == {"export_allowed": True, "errors": []}
    json.dumps(result, allow_nan=False)


def test_laps_face_outward_and_never_share_a_position():
    result = plan_panorama(_request())
    waypoints = result["route"]["waypoints"]
    positions = {(row["position"]["latitude_deg"], row["position"]["longitude_deg"]) for row in waypoints}
    assert len(positions) == len(waypoints)
    for record in result["panorama"]["points"][0]["rows"][:-1]:
        for index in range(record["start_waypoint_index"], record["end_waypoint_index"] + 1):
            row = waypoints[index]
            bearing, _back, distance = GEOD.inv(CENTRE["longitude_deg"], CENTRE["latitude_deg"],
                                                row["position"]["longitude_deg"], row["position"]["latitude_deg"])
            assert distance == pytest.approx(record["radius_m"], abs=.01)
            # The heading points away from the centre, the way the camera looks.
            assert (row["heading_deg"] - bearing + 180) % 360 - 180 == pytest.approx(0, abs=.01)
    nadir = waypoints[-1]
    assert nadir["position"] == CENTRE and "nadir" in nadir["roles"]


def test_every_photo_holds_still_and_each_row_tilts_the_camera():
    result = plan_panorama(_request())
    actions = result["flights"][0]["actions"]
    for record in result["panorama"]["points"][0]["rows"]:
        start = record["start_waypoint_index"]
        assert [action for action in actions if action["waypoint_index"] == start] == [
            {"type": "rotate_camera", "waypoint_index": start, "pitch_deg": record["gimbal_pitch_deg"]},
            {"type": "hover", "waypoint_index": start, "duration_s": 2.0},
            {"type": "take_photo", "waypoint_index": start},
        ]
        for index in range(start + 1, record["end_waypoint_index"] + 1):
            assert [action["type"] for action in actions if action["waypoint_index"] == index] == [
                "hover", "take_photo"]


def test_more_overlap_takes_more_photos_and_no_nadir_drops_the_last_row():
    tight = plan_panorama(_request(capture={"mode": "full_auto", "overlap_ratio": .5}))
    assert tight["statistics"]["photo_count"] > 24
    without = plan_panorama(_request(include_nadir=False))
    assert _rows(without) == [(0.0, 8, 3.0), (-30.0, 8, 4.0), (-60.0, 7, 5.0)]
    assert "nadir" not in without["route"]["waypoints"][-1]["roles"]


def test_contract_one_matches_contract_two_layout():
    first = plan_panorama(_request(contract_version=1))
    second = plan_panorama(_request())
    assert first["contract_version"] == 1 and _rows(first) == _rows(second)


@pytest.mark.parametrize("change,code", [
    ({"capture": {"mode": "semi_auto"}}, "invalid_capture_mode"),
    ({"capture": {"mode": "full_auto", "overlap_ratio": .9}}, "number_out_of_range"),
    ({"capture": {"mode": "full_auto", "overlap_ratio": .3, "interval_s": 2}}, "unknown_field"),
    ({"altitude_m": 0}, "number_out_of_range"),
    ({"speed_m_s": 50}, "speed_out_of_profile_range"),
    ({"terrain_follow": True}, "unsupported_feature"),
    ({"include_nadir": 1}, "invalid_boolean"),
    ({"finish_action": "orbit"}, "invalid_finish_action"),
    ({"drone_profile_id": "m4e"}, "profile_not_found"),
    ({"radius_m": 5}, "unknown_field"),
    ({"points": []}, "invalid_points"),
    ({"points": dict(CENTRE)}, "invalid_points"),
    ({"points": [dict(CENTRE)] * 51}, "invalid_points"),
    ({"points": [{"latitude_deg": 91, "longitude_deg": 0}]}, "number_out_of_range"),
    ({"split": {"max_waypoints_per_flight": 1}}, "number_out_of_range"),
    ({"turn_style": "zigzag"}, "invalid_turn_style"),
])
def test_invalid_requests_are_rejected(change, code):
    request = _request(**change)
    saved = copy.deepcopy(request)
    with pytest.raises(PlanningError) as error:
        plan_panorama(request)
    assert error.value.code == code
    assert request == saved


def test_plans_are_deterministic():
    assert plan_panorama(_request()) == plan_panorama(_request())
    assert math.isfinite(plan_panorama(_request())["statistics"]["known_estimated_seconds"])


def _east(metres):
    """A point due east of the centre, at contract 2's nine decimal places."""
    longitude, latitude, _back = GEOD.fwd(CENTRE["longitude_deg"], CENTRE["latitude_deg"], 90, metres)
    return {"latitude_deg": round(latitude, 9), "longitude_deg": round(longitude, 9)}


def test_points_are_flown_in_order_each_with_its_own_panorama():
    points = [dict(CENTRE), _east(40), _east(80)]
    result = plan_panorama(_request(points=points))
    records = result["panorama"]["points"]
    assert [record["centre"] for record in records] == points
    assert [(record["start_waypoint_index"], record["end_waypoint_index"]) for record in records] == [
        (0, 23), (24, 47), (48, 71)]
    assert all(_rows(result, index) == _rows(result) for index in range(3))
    waypoints = result["route"]["waypoints"]
    assert [row["point_index"] for row in waypoints] == [0] * 24 + [1] * 24 + [2] * 24
    assert "point_start" in waypoints[24]["roles"] and "point_end" in waypoints[23]["roles"]
    assert waypoints[47]["position"] == points[1]          # each panorama ends straight down over its point
    assert len(result["route"]["strips"]) == result["statistics"]["strip_count"] == 12
    # 72 waypoints fit one flight under the default limit of 200.
    assert result["statistics"]["battery_count"] == 1 and result["validation"]["export_allowed"]


def test_points_too_close_for_their_laps_are_rejected():
    with pytest.raises(PlanningError) as error:
        plan_panorama(_request(points=[dict(CENTRE), _east(12)]))
    assert error.value.code == "points_too_close" and error.value.field == "points[1]"
    assert plan_panorama(_request(points=[dict(CENTRE), _east(15.1)]))["panorama"]["min_point_spacing_m"] == 15


def test_flights_split_between_panoramas_never_inside_one():
    points = [dict(CENTRE), _east(40), _east(80), _east(120)]
    result = plan_panorama(_request(points=points, split={"max_waypoints_per_flight": 50}))
    flights = result["flights"]
    assert [(item["route_start_waypoint_index"], item["route_end_waypoint_index"]) for item in flights] == [
        (0, 47), (48, 95)]
    for item in flights:
        start = item["route_start_waypoint_index"]
        assert "point_start" in result["route"]["waypoints"][start]["roles"]
        assert item["actions"][0] == {"type": "rotate_camera", "waypoint_index": start, "pitch_deg": 0.0}
    assert result["statistics"]["battery_count"] == 2
    assert result["statistics"]["photo_count"] == sum(item["photo_count"] for item in flights) == 96
    # The transit between the two flights is not flown by either.
    assert result["statistics"]["route_distance_m"] < sum(
        GEOD.inv(a["position"]["longitude_deg"], a["position"]["latitude_deg"],
                 b["position"]["longitude_deg"], b["position"]["latitude_deg"])[2]
        for a, b in zip(result["route"]["waypoints"], result["route"]["waypoints"][1:]))


def test_one_panorama_above_the_waypoint_limit_blocks_export():
    result = plan_panorama(_request(split={"max_waypoints_per_flight": 20}))
    assert not result["validation"]["export_allowed"]
    assert result["validation"]["errors"][0]["code"] == "waypoint_limit_exceeded"


AREA = {"exterior": [
    {"latitude_deg": 51.05, "longitude_deg": -114.09}, {"latitude_deg": 51.05, "longitude_deg": -114.08},
    {"latitude_deg": 51.055, "longitude_deg": -114.08}, {"latitude_deg": 51.055, "longitude_deg": -114.09},
], "holes": []}


def _area_request(**overrides):
    request = _request(survey_area=AREA, speed_m_s=8, altitude_m=180)
    request.pop("points")
    request.update(overrides)
    return request


def _metres(first, second):
    return GEOD.inv(first["longitude_deg"], first["latitude_deg"], second["longitude_deg"], second["latitude_deg"])[2]


def test_area_points_sit_on_a_hexagon_grid_one_altitude_apart():
    result = plan_panorama(_area_request())
    panorama = result["panorama"]
    assert panorama["layout"] == "area" and panorama["point_spacing_m"] == 180
    centres = [record["centre"] for record in panorama["points"]]
    assert len(centres) == 11
    for centre in centres:
        assert 51.05 <= centre["latitude_deg"] <= 51.055 and -114.09 <= centre["longitude_deg"] <= -114.08
    # Every point's nearest neighbour is one spacing away, as on a hexagon grid.
    for centre in centres:
        nearest = min(_metres(centre, other) for other in centres if other is not centre)
        assert nearest == pytest.approx(180, abs=1)
    # Rows of 4, 3 and 4 points run back and forth: west to east, east to west, west to east.
    longitudes = [centre["longitude_deg"] for centre in centres]
    assert longitudes[:4] == sorted(longitudes[:4])
    assert longitudes[4:7] == sorted(longitudes[4:7], reverse=True)
    assert longitudes[7:] == sorted(longitudes[7:])
    assert result["statistics"]["survey_area_m2"] == pytest.approx(390_000, rel=.01)
    assert result["statistics"]["battery_count"] == 2 and result["validation"]["export_allowed"]


def test_area_spacing_follows_the_request_and_small_areas_get_one_point():
    assert len(plan_panorama(_area_request(point_spacing_m=300))["panorama"]["points"]) == 7
    single = plan_panorama(_area_request(point_spacing_m=2000))["panorama"]["points"]
    assert len(single) == 1
    centre = single[0]["centre"]
    assert 51.05 <= centre["latitude_deg"] <= 51.055 and -114.09 <= centre["longitude_deg"] <= -114.08


@pytest.mark.parametrize("change,code", [
    ({"point_spacing_m": 10}, "points_too_close"),
    ({"point_spacing_m": 30}, "too_many_points"),
    ({"point_spacing_m": 0}, "number_out_of_range"),
    ({"points": [dict(CENTRE)]}, "invalid_points"),
    ({"survey_area": {"exterior": [dict(CENTRE)], "holes": []}}, "invalid_geometry"),
])
def test_invalid_area_requests_are_rejected(change, code):
    with pytest.raises(PlanningError) as error:
        plan_panorama(_area_request(**change))
    assert error.value.code == code


def test_point_spacing_needs_an_area():
    with pytest.raises(PlanningError) as error:
        plan_panorama(_request(point_spacing_m=100))
    assert error.value.code == "unsupported_feature"


def test_turn_style_is_echoed_and_keeps_the_route():
    curved = plan_panorama(_request())
    straight = plan_panorama(_request(turn_style="straight"))
    assert curved["turn_style"] == "curved" and straight["turn_style"] == "straight"
    assert straight["route"] == curved["route"] and straight["flights"] == curved["flights"]
