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


def _seam_m(waypoints):
    first, last = waypoints[0]["position"], waypoints[-1]["position"]
    return GEOD.inv(last["longitude_deg"], last["latitude_deg"],
                    first["longitude_deg"], first["latitude_deg"])[2]


def test_semi_auto_ends_short_of_the_start_without_photo_actions():
    result = plan_orbit(_request(mode="semi_auto"))
    waypoints = result["route"]["waypoints"]
    for waypoint in waypoints:
        assert abs(_distance_and_bearing_to_centre(waypoint)[0] - 40) < 0.01
    # Two separate waypoints, no further apart than one photo spacing (3 m/s x 2 s).
    assert 0.5 < _seam_m(waypoints) <= 3 * 2.0 + 1e-6
    actions = result["flights"][0]["actions"]
    assert actions == [{"type": "rotate_camera", "waypoint_index": 0, "pitch_deg": -35}]
    distance = result["statistics"]["route_distance_m"]
    assert result["statistics"]["photo_count"] == math.floor(distance / (3 * 2.0)) + 1
    assert result["statistics"]["photo_count_kind"] == "estimate"
    assert 0 <= result["capture"]["side_overlap_ratio"] < 1


def test_semi_auto_gap_never_exceeds_a_ring_segment():
    result = plan_orbit(_request(mode="semi_auto", speed_m_s=12))
    waypoints = result["route"]["waypoints"]
    segment = GEOD.inv(waypoints[0]["position"]["longitude_deg"], waypoints[0]["position"]["latitude_deg"],
                       waypoints[1]["position"]["longitude_deg"], waypoints[1]["position"]["latitude_deg"])[2]
    assert _seam_m(waypoints) <= segment + 1e-6
    assert abs(_seam_m(waypoints) - segment) < 0.01    # 24 m photo spacing is wider than a segment


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



RINGS = [
    {"radius_m": 40, "altitude_m": 30, "gimbal_pitch_deg": -35},
    {"radius_m": 55, "altitude_m": 45, "gimbal_pitch_deg": -50},
    {"radius_m": 30, "altitude_m": 60, "gimbal_pitch_deg": -70},
]


def _multi(rings=RINGS, mode="full_auto", **overrides):
    request = _request(mode, **overrides)
    for field in ("radius_m", "altitude_m", "gimbal_pitch_deg"):
        request.pop(field)
    request["rings"] = copy.deepcopy(rings)
    return request


def test_rings_fly_in_order_as_one_flight_facing_the_centre():
    result = plan_orbit(_multi())
    waypoints = result["route"]["waypoints"]
    rings = result["orbit"]["rings"]
    assert len(result["flights"]) == 1 and result["statistics"]["strip_count"] == 3
    assert [ring["index"] for ring in rings] == [0, 1, 2]
    assert rings[0]["start_waypoint_index"] == 0
    assert rings[-1]["end_waypoint_index"] == len(waypoints) - 1
    for ring, request_ring in zip(rings, RINGS):
        assert rings[ring["index"] - 1]["end_waypoint_index"] + 1 == ring["start_waypoint_index"] or ring["index"] == 0
        for waypoint in waypoints[ring["start_waypoint_index"]:ring["end_waypoint_index"] + 1]:
            distance, bearing = _distance_and_bearing_to_centre(waypoint)
            assert abs(distance - request_ring["radius_m"]) < 0.01
            assert _angle_gap(waypoint["heading_deg"], bearing) < 1e-6
            assert waypoint["altitude_m"] == request_ring["altitude_m"]
            assert waypoint["ring_index"] == ring["index"]
        # Every ring starts at the start bearing (north).
        start = waypoints[ring["start_waypoint_index"]]
        assert _angle_gap((_distance_and_bearing_to_centre(start)[1] + 180) % 360, 0) < 1e-6
    assert "route_start" in waypoints[0]["roles"] and "route_end" in waypoints[-1]["roles"]
    json.dumps(result)


def test_each_ring_tilts_the_camera_and_full_auto_photographs_every_waypoint():
    result = plan_orbit(_multi())
    actions = result["flights"][0]["actions"]
    rings = result["orbit"]["rings"]
    tilts = [(action["waypoint_index"], action["pitch_deg"]) for action in actions if action["type"] == "rotate_camera"]
    assert tilts == [(ring["start_waypoint_index"], RINGS[ring["index"]]["gimbal_pitch_deg"]) for ring in rings]
    hovers = [action["waypoint_index"] for action in actions if action["type"] == "hover"]
    assert hovers == [ring["start_waypoint_index"] for ring in rings]
    photos = [action["waypoint_index"] for action in actions if action["type"] == "take_photo"]
    assert photos == list(range(len(result["route"]["waypoints"])))
    assert result["statistics"]["photo_count"] == len(photos)
    assert result["flights"][0]["startup_wait_seconds"] == 3 * 3.0


def test_semi_auto_rings_tilt_only_and_count_interval_photos_over_the_whole_flight():
    result = plan_orbit(_multi(mode="semi_auto"))
    actions = result["flights"][0]["actions"]
    assert {action["type"] for action in actions} == {"rotate_camera"}
    assert len(actions) == 3
    distance = result["statistics"]["route_distance_m"]
    assert result["statistics"]["photo_count"] == math.floor(distance / (3 * 2.0)) + 1
    assert result["capture"]["side_overlap_ratio"] == min(ring["side_overlap_ratio"] for ring in result["orbit"]["rings"])


def test_transitions_count_the_climb_between_rings():
    same_place = [dict(RINGS[0]), dict(RINGS[0], altitude_m=90)]
    result = plan_orbit(_multi(same_place, mode="semi_auto"))
    single = plan_orbit(_request("semi_auto"))
    ring_distance = single["statistics"]["route_distance_m"]
    first_end = result["route"]["waypoints"][result["orbit"]["rings"][0]["end_waypoint_index"]]["position"]
    second_start = result["route"]["waypoints"][result["orbit"]["rings"][1]["start_waypoint_index"]]["position"]
    across = GEOD.inv(first_end["longitude_deg"], first_end["latitude_deg"],
                      second_start["longitude_deg"], second_start["latitude_deg"])[2]
    expected = 2 * ring_distance + math.hypot(across, 60)
    assert abs(result["statistics"]["route_distance_m"] - expected) < 0.01


def test_one_ring_in_the_list_matches_the_single_ring_route():
    listed = plan_orbit(_multi([RINGS[0]]))
    single = plan_orbit(_request())
    assert [w["position"] for w in listed["route"]["waypoints"]] == [w["position"] for w in single["route"]["waypoints"]]
    assert [w["heading_deg"] for w in listed["route"]["waypoints"]] == [w["heading_deg"] for w in single["route"]["waypoints"]]
    assert listed["statistics"]["photo_count"] == single["statistics"]["photo_count"]


def test_single_ring_requests_keep_the_original_result_shape():
    result = plan_orbit(_request())
    assert "rings" not in result["orbit"] and "radius_m" in result["orbit"]
    assert all("altitude_m" not in w and "ring_index" not in w for w in result["route"]["waypoints"])


def test_ring_limits_and_mixed_requests_are_rejected():
    _error(_multi(RINGS + [RINGS[0]]), "invalid_rings", "rings")
    _error(_multi([]), "invalid_rings", "rings")
    mixed = _multi()
    mixed["radius_m"] = 40
    _error(mixed, "invalid_rings", "rings")
    _error(_multi([dict(RINGS[0], radius_m=2)]), "number_out_of_range", "rings[0].radius_m")
    _error(_multi([RINGS[0], dict(RINGS[1], gimbal_pitch_deg=5)]), "number_out_of_range", "rings[1].gimbal_pitch_deg")
    _error(_multi([dict(RINGS[0], heading=3)]), "unknown_field", "rings[0].heading")


def test_ring_waypoints_count_together_against_the_limit():
    total = len(plan_orbit(_multi())["route"]["waypoints"])
    limit = total - 1
    result = plan_orbit(_multi(split={"enabled": False, "max_waypoints_per_flight": limit}))
    # Each ring fits on its own; together they exceed the limit.
    assert all(ring["waypoint_count"] < limit for ring in result["orbit"]["rings"])
    assert "waypoint_limit_exceeded" in [error["code"] for error in result["validation"]["errors"]]
    assert not result["validation"]["export_allowed"]

if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print(f"PASS  {test.__name__}")
