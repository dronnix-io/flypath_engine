"""Acceptance checks for the orbit planning boundary (one ring, oblique capture)."""

import copy
import json
import math
from pathlib import Path
import sys

from pyproj import Geod

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine import PlanningError, plan_orbit  # noqa: E402

GEOD = Geod(ellps="WGS84")
CENTRE = {"latitude_deg": 51.05, "longitude_deg": -114.09}


def _request(mode="full_auto", **overrides):
    request = {
        "contract_version": 1,
        "operation": "plan_orbit",
        "mapping_style": "orbit",
        "profile_version": 1,
        "drone_profile_id": "mini4pro",
        "centre": dict(CENTRE),
        "radius_m": 40,
        "altitude_m": 30,
        "gimbal_pitch_deg": -35,
        "direction": "clockwise",
        "speed_m_s": 3,
        "capture": {"mode": mode},
        "finish_action": "return_to_home",
        "split": {"enabled": False, "max_waypoints_per_flight": 200},
    }
    if mode == "full_auto":
        request["capture"]["side_overlap_ratio"] = .8
    request.update(overrides)
    return request


def _error(request, code, field=None):
    try:
        plan_orbit(request)
    except PlanningError as exc:
        assert exc.code == code, exc.as_dict()
        if field:
            assert exc.field == field, exc.as_dict()
    else:
        raise AssertionError(f"expected {code}")


def _distance_and_bearing_to_centre(waypoint):
    position = waypoint["position"]
    azimuth, _back, distance = GEOD.inv(position["longitude_deg"], position["latitude_deg"],
                                        CENTRE["longitude_deg"], CENTRE["latitude_deg"])
    return distance, azimuth


def _angle_gap(a, b):
    return abs((a - b + 180) % 360 - 180)


def test_full_auto_ring_faces_the_centre_with_a_photo_per_waypoint():
    result = plan_orbit(_request())
    waypoints = result["route"]["waypoints"]
    assert len(waypoints) >= 12
    assert result["validation"]["export_allowed"], result["validation"]
    for waypoint in waypoints:
        distance, bearing = _distance_and_bearing_to_centre(waypoint)
        assert abs(distance - 40) < 0.01, distance
        assert _angle_gap(waypoint["heading_deg"], bearing) < 1e-6
        assert -180 < waypoint["heading_deg"] <= 180
    actions = result["flights"][0]["actions"]
    assert actions[0] == {"type": "rotate_camera", "waypoint_index": 0, "pitch_deg": -35}
    assert actions[1]["type"] == "hover"
    photos = [action["waypoint_index"] for action in actions if action["type"] == "take_photo"]
    assert photos == list(range(len(waypoints)))
    assert result["statistics"]["photo_count"] == len(waypoints)
    assert result["mapping_style"] == "orbit" and result["turn_style"] == "curved"
    json.dumps(result)


def test_side_overlap_sets_the_photo_spacing():
    low = plan_orbit(_request(capture={"mode": "full_auto", "side_overlap_ratio": .6}))
    high = plan_orbit(_request(capture={"mode": "full_auto", "side_overlap_ratio": .9}))
    assert len(high["route"]["waypoints"]) > len(low["route"]["waypoints"])
    slant = math.hypot(40, 30)
    footprint = slant * 9.6 / 6.9
    assert high["capture"]["shot_spacing_m"] <= footprint * (1 - .9) + 1e-9


def test_direction_controls_the_order_around_the_ring():
    def bearings(direction):
        waypoints = plan_orbit(_request(direction=direction))["route"]["waypoints"]
        return [(_distance_and_bearing_to_centre(w)[1] + 180) % 360 for w in waypoints[:3]]
    clockwise = bearings("clockwise")
    counter = bearings("counterclockwise")
    assert _angle_gap(clockwise[0], 0) < 1e-6 and _angle_gap(counter[0], 0) < 1e-6
    assert 0 < (clockwise[1] - clockwise[0]) % 360 < 180
    assert 0 < (counter[0] - counter[1]) % 360 < 180


def test_semi_auto_closes_the_loop_without_photo_actions():
    result = plan_orbit(_request(mode="semi_auto"))
    waypoints = result["route"]["waypoints"]
    first, last = waypoints[0]["position"], waypoints[-1]["position"]
    assert abs(first["latitude_deg"] - last["latitude_deg"]) < 1e-9
    assert abs(first["longitude_deg"] - last["longitude_deg"]) < 1e-9
    actions = result["flights"][0]["actions"]
    assert actions == [{"type": "rotate_camera", "waypoint_index": 0, "pitch_deg": -35}]
    distance = result["statistics"]["route_distance_m"]
    assert result["statistics"]["photo_count"] == math.floor(distance / (3 * 2.0))
    assert result["statistics"]["photo_count_kind"] == "estimate"
    assert 0 <= result["capture"]["side_overlap_ratio"] < 1


def test_waypoint_limit_blocks_export_instead_of_splitting():
    result = plan_orbit(_request(split={"enabled": False, "max_waypoints_per_flight": 10}))
    codes = [error["code"] for error in result["validation"]["errors"]]
    assert "waypoint_limit_exceeded" in codes
    assert not result["validation"]["export_allowed"]


def test_invalid_requests_are_rejected_with_stable_codes():
    _error(_request(split={"enabled": True, "max_waypoints_per_flight": 99}), "unsupported_feature", "split.enabled")
    _error(_request(terrain_follow=True), "unsupported_feature", "terrain_follow")
    _error(_request(direction="sideways"), "invalid_direction", "direction")
    _error(_request(radius_m=2), "number_out_of_range", "radius_m")
    _error(_request(gimbal_pitch_deg=10), "number_out_of_range", "gimbal_pitch_deg")
    _error(_request(speed_m_s=40), "speed_out_of_profile_range", "speed_m_s")
    _error(_request(mapping_style="2d"), "unsupported_feature", "mapping_style")
    _error(_request(operation="plan_2d"), "unsupported_operation", "operation")
    _error(_request(drone_profile_id="nope"), "profile_not_found", "drone_profile_id")
    request = _request(mode="semi_auto")
    request["capture"]["side_overlap_ratio"] = .8
    _error(request, "unsupported_feature", "capture.side_overlap_ratio")
    unknown = _request()
    unknown["orbit_height"] = 3
    _error(unknown, "unknown_field", "request.orbit_height")


def test_results_are_deterministic():
    request = _request()
    assert plan_orbit(copy.deepcopy(request)) == plan_orbit(copy.deepcopy(request))


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print(f"PASS  {test.__name__}")
