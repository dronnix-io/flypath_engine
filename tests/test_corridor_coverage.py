"""Bend coverage and hard-break geometry for new corridor plans."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest
from pyproj import Transformer
from shapely.geometry import LineString

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from flypath_engine import PlanningError, plan_corridor
from flypath_engine.profiles import drone_profile


def _request(points, *, breaks=(), full=False):
    inverse = Transformer.from_crs("EPSG:32612", "EPSG:4326", always_xy=True)
    line = []
    for x, y in points:
        lon, lat = inverse.transform(x, y)
        line.append({"latitude_deg": lat, "longitude_deg": lon})
    return {
        "contract_version": 2, "operation": "plan_corridor", "mapping_style": "corridor",
        "profile_version": 1, "drone_profile_id": "mini4pro", "centre_line": line,
        "width_m": 90, "altitude_m": 40, "speed_m_s": 5, "side_overlap_ratio": .5,
        "capture": ({"mode": "full_auto", "front_overlap_ratio": .7} if full
                    else {"mode": "semi_auto"}),
        "split": {"enabled": True, "requested_flights": 1, "max_waypoints_per_flight": 200},
        "break_indices": list(breaks),
    }


def _strip(result, index):
    segment = result["route"]["strips"][index]
    return [row["position"] for row in result["route"]["waypoints"][
        segment["start_waypoint_index"]:segment["end_waypoint_index"] + 1]]


@pytest.mark.parametrize("angle,expected_factor", [(90, 1), (60, 3 ** -.5)])
def test_outer_pass_reaches_sharp_miter_with_camera_height(angle, expected_factor):
    import math

    radians = math.radians(angle)
    origin = (500_000, 5_650_000)
    bend = (500_000, 5_650_100)
    # The outgoing heading forms the requested interior angle with the return
    # direction at the bend.
    end = (bend[0] + 100 * math.sin(radians), bend[1] - 100 * math.cos(radians))
    result = plan_corridor(_request([origin, bend, end], full=True))
    camera = drone_profile("mini4pro")[1]["camera"]
    width = 40 * camera["sensor_width_mm"] / camera["focal_length_mm"]
    height = 40 * camera["sensor_height_mm"] / camera["focal_length_mm"]
    required_offset = 45 - height / 2 * expected_factor
    assert max(result["corridor"]["pass_offsets_m"]) >= required_offset - .01
    assert result["validation"]["export_allowed"]
    # Check the actual generated outside miter against the corner of the
    # projected corridor band. A full-auto photo occurs at this waypoint.
    positive_offset = max(result["corridor"]["pass_offsets_m"])
    forward = Transformer.from_crs("EPSG:4326", "EPSG:32612", always_xy=True)
    def miter(distance):
        # Incoming tangent is north; outgoing tangent is the chosen angle.
        # Their left-offset lines intersect here.
        return (bend[0] - distance,
                bend[1] + distance * (1 + math.cos(radians)) / math.sin(radians))
    planned = miter(positive_offset)
    target = miter(45)
    strip = _strip(result, len(result["corridor"]["pass_offsets_m"]) - 1)
    projected = [forward.transform(p["longitude_deg"], p["latitude_deg"])
                 for p in strip]
    actual = min(projected, key=lambda p: math.dist(p, planned))
    assert math.dist(actual, planned) < .02
    delta = (target[0] - actual[0], target[1] - actual[1])
    assert abs(delta[0]) <= width / 2 + .02
    assert abs(delta[1]) <= height / 2 + .02
    assert width > height


@pytest.mark.parametrize("full", [False, True])
def test_break_cuts_full_offsets_without_losing_the_bend(full):
    points = [(500_000, 5_650_000), (500_000, 5_650_100),
              (500_100, 5_650_100)]
    unbroken = plan_corridor(_request(points, full=full))
    value = _request(points, breaks=[1], full=full)
    broken = plan_corridor(value)
    count = len(unbroken["corridor"]["pass_offsets_m"])
    for index in range(count):
        first = _strip(broken, index)
        second = _strip(broken, count + index)
        if index % 2:
            first, second = first[::-1], second[::-1]
        assert first[-1] == second[0]
        continuous = _strip(unbroken, index)
        if index % 2:
            continuous.reverse()
        whole = LineString([(p["longitude_deg"], p["latitude_deg"])
                            for p in first + second[1:]])
        expected = LineString([(p["longitude_deg"], p["latitude_deg"])
                               for p in continuous])
        assert whole.hausdorff_distance(expected) < 1e-8
        assert abs(whole.length - expected.length) < 1e-8
    backward = deepcopy(value)
    backward["reverse_route"] = True
    reverse = plan_corridor(backward)
    assert [row["position"] for row in reverse["route"]["waypoints"]] == [
        row["position"] for row in reversed(broken["route"]["waypoints"])]
    starts = [row["index"] for row in broken["route"]["waypoints"]
              if "corridor_group_start" in row["roles"]]
    assert len(starts) == 2
    assert all(not (flight["route_start_waypoint_index"] < starts[1]
                    <= flight["route_end_waypoint_index"])
               for flight in broken["flights"])


def test_offset_that_folds_back_at_a_break_is_rejected():
    value = _request([(500_000, 5_650_000), (500_000, 5_650_100),
                      (500_010, 5_650_100), (500_010, 5_650_000)], breaks=[1])
    with pytest.raises(PlanningError) as error:
        plan_corridor(value)
    assert error.value.code == "invalid_offset"
