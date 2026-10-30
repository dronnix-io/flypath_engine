"""Acceptance checks for panorama planning: small outward-facing laps around one point."""

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
        "centre": dict(CENTRE),
        "altitude_m": 60,
        "speed_m_s": 2,
        "capture": {"mode": "full_auto", "overlap_ratio": .3},
        "finish_action": "return_to_home",
    }
    request.update(overrides)
    return request


def _rows(result):
    return [(row["gimbal_pitch_deg"], row["photo_count"], row["radius_m"]) for row in result["panorama"]["rows"]]


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
    for record in result["panorama"]["rows"][:-1]:
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
    for record in result["panorama"]["rows"]:
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
