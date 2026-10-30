"""Versioned panorama planning: photos all around one point for a stitched panorama.

DJI Fly missions cannot turn the drone in place, so the drone flies a very
small circle around the panorama point, facing outward, and takes one photo at
each heading. Each camera row is one lap at its own gimbal pitch, a little
wider than the lap before so no two waypoints share a position. A straight-down
photo over the point ends the mission. A few metres of movement does not show
in a panorama of the surroundings.

Rows run from the horizon (0 degrees) down towards straight down, evenly
spaced so neighbouring rows overlap by the requested ratio. Photos in a row are
spaced so neighbours overlap by the same ratio at the row's top edge, where the
photos are narrowest in heading. Every waypoint holds still for a moment
before its photo; each row also tilts the camera at its first waypoint.
"""

import math

from .measurements import _WGS84, route_distance_m
from .orbit import FINISH_ACTIONS, _flight_record, _recovery, _signed_degrees
from .planning import (
    CONTRACT_VERSION, ENGINE_VERSION, MAX_ALTITUDE_M, MAX_WAYPOINT_LIMIT, PlanningError,
    _ensure_finite_json, _issue, _keys, _number, _optional_bool, _position, _version,
)
from .precision import number, versioned
from .profiles import PROFILE_VERSION, drone_profile


MIN_OVERLAP = 0.2
MAX_OVERLAP = 0.7
MIN_ROW_PHOTOS = 3
FIRST_LAP_RADIUS_M = 3.0         # the first row's lap around the point
LAP_GAP_M = 1.0                  # each later row's lap is this much wider
SHOT_SETTLE_S = 1.0              # hold still before each photo
ROW_SETTLE_S = 2.0               # longer where the camera tilts to a new row


@versioned
def plan_panorama(request):
    """Return one deterministic, JSON-compatible panorama planning result."""
    if not isinstance(request, dict):
        raise PlanningError("invalid_request", "Planning request must be an object.")
    _keys(request, {
        "contract_version", "operation", "source_mission_id", "mapping_style",
        "drone_profile_id", "profile_version", "centre", "altitude_m", "speed_m_s",
        "capture", "include_nadir", "finish_action", "terrain_follow",
    }, "request")
    if request.get("operation", "plan_panorama") != "plan_panorama":
        raise PlanningError("unsupported_operation", "Only plan_panorama is supported here.", "operation")
    if request.get("mapping_style", "panorama") != "panorama":
        raise PlanningError("unsupported_feature", "plan_panorama only plans panorama missions.", "mapping_style")
    _version(request.get("contract_version"), CONTRACT_VERSION, "contract_version")
    _version(request.get("profile_version"), PROFILE_VERSION, "profile_version")
    if request.get("terrain_follow") not in (None, False):
        raise PlanningError("unsupported_feature", "terrain_follow is not supported for panorama missions.",
                            "terrain_follow")

    profile_id = request.get("drone_profile_id")
    try:
        profile_name, profile = drone_profile(profile_id)
    except (KeyError, TypeError):
        raise PlanningError("profile_not_found", "Drone profile is not available.", "drone_profile_id") from None
    aircraft, camera = profile["aircraft"], profile["camera"]

    centre = _position(request.get("centre"), "centre")
    altitude = _number(request.get("altitude_m"), "altitude_m", minimum=0, maximum=MAX_ALTITUDE_M, exclusive=True)
    speed = _number(request.get("speed_m_s"), "speed_m_s", minimum=0, exclusive=True)
    if not aircraft["min_speed_ms"] <= speed <= aircraft["max_speed_ms"]:
        raise PlanningError("speed_out_of_profile_range", "Speed is outside the drone profile range.", "speed_m_s")
    include_nadir = _optional_bool(request.get("include_nadir", True), "include_nadir")
    finish_action = request.get("finish_action", "return_to_home")
    if finish_action not in FINISH_ACTIONS:
        raise PlanningError("invalid_finish_action", "Finish action is not supported.", "finish_action")

    capture = request.get("capture")
    if not isinstance(capture, dict) or capture.get("mode") != "full_auto":
        raise PlanningError("invalid_capture_mode", "Panorama capture mode must be full_auto.", "capture.mode")
    _keys(capture, {"mode", "overlap_ratio"}, "capture")
    overlap = _number(capture.get("overlap_ratio"), "capture.overlap_ratio",
                      minimum=MIN_OVERLAP, maximum=MAX_OVERLAP)
    interval = _number(camera.get("min_shoot_interval_s"), "drone_profile.camera.min_shoot_interval_s",
                       minimum=0, exclusive=True)
    width = _number(camera.get("sensor_width_mm"), "drone_profile.camera.sensor_width_mm", minimum=0, exclusive=True)
    height = _number(camera.get("sensor_height_mm"), "drone_profile.camera.sensor_height_mm",
                     minimum=0, exclusive=True)
    focal = _number(camera.get("focal_length_mm"), "drone_profile.camera.focal_length_mm", minimum=0, exclusive=True)
    horizontal_fov = number(math.degrees(2 * math.atan(width / (2 * focal))))
    vertical_fov = number(math.degrees(2 * math.atan(height / (2 * focal))))

    rows = _rows(horizontal_fov, vertical_fov, overlap, include_nadir)
    waypoints, actions, edges, row_records = [], [], [], []
    previous = None
    for row_index, row in enumerate(rows):
        start = len(waypoints)
        for photo in range(row["photo_count"]):
            bearing = 360.0 * photo / row["photo_count"]
            if row["radius_m"]:
                longitude, latitude, _back = _WGS84.fwd(centre["longitude_deg"], centre["latitude_deg"],
                                                        bearing, row["radius_m"])
                position = {"latitude_deg": latitude, "longitude_deg": longitude}
            else:
                position = dict(centre)             # the straight-down photo
            index = len(waypoints)
            roles = {"nadir"} if row["nadir"] else {"panorama_point"}
            if photo == 0:
                roles.add("row_start")
                actions.append({"type": "rotate_camera", "waypoint_index": index,
                                "pitch_deg": row["gimbal_pitch_deg"]})
            if photo == row["photo_count"] - 1:
                roles.add("row_end")
            actions.append({"type": "hover", "waypoint_index": index,
                            "duration_s": ROW_SETTLE_S if photo == 0 else SHOT_SETTLE_S})
            actions.append({"type": "take_photo", "waypoint_index": index})
            if previous is not None:
                edges.append(route_distance_m([previous, position]))
            previous = position
            waypoints.append({"index": index, "position": position, "roles": roles,
                              "heading_deg": _signed_degrees(bearing)})
        row_records.append({
            "index": row_index, "gimbal_pitch_deg": row["gimbal_pitch_deg"],
            "photo_count": row["photo_count"], "radius_m": row["radius_m"],
            "start_waypoint_index": start, "end_waypoint_index": len(waypoints) - 1,
        })
    waypoints[0]["roles"].add("route_start")
    waypoints[-1]["roles"].add("route_end")
    for waypoint in waypoints:
        waypoint["roles"] = sorted(waypoint["roles"])

    usable_seconds = _number(aircraft.get("battery_safe_min"), "drone_profile.aircraft.battery_safe_min",
                             minimum=0, exclusive=True) * 60
    common = {"speed": speed, "mode": "full_auto", "usable_seconds": usable_seconds}
    photo_count = len(waypoints)
    hover_seconds = sum(action["duration_s"] for action in actions if action["type"] == "hover")
    last = len(waypoints) - 1
    flight = _flight_record(0, 0, last, actions, sum(edges),
                            _recovery(waypoints[0]["position"], waypoints[last]["position"], finish_action),
                            photo_count, photo_count * max(2.5, interval), hover_seconds, common)
    blocking = []
    if len(waypoints) > MAX_WAYPOINT_LIMIT:
        blocking.append(_issue("waypoint_limit_exceeded",
                               "The panorama needs more waypoints than the limit; lower the overlap."))
    if flight["known_estimated_seconds"] > usable_seconds:
        blocking.append(_issue("battery_time_exceeded", "The panorama exceeds the usable battery-time limit."))

    first = rows[0]
    result = {
        "contract_version": CONTRACT_VERSION,
        "engine_version": ENGINE_VERSION,
        "profile_version": PROFILE_VERSION,
        "operation": "plan_panorama",
        "mapping_style": "panorama",
        "drone_profile_id": profile_id,
        "drone_profile_name": profile_name,
        "turn_style": "curved",
        "finish_action": finish_action,
        "panorama": {
            "centre": centre,
            "altitude_m": altitude,
            "overlap_ratio": overlap,
            "include_nadir": include_nadir,
            "horizontal_fov_deg": horizontal_fov,
            "vertical_fov_deg": vertical_fov,
            "rows": row_records,
        },
        "route": {"waypoints": waypoints,
                  "strips": [{"index": record["index"], "start_waypoint_index": record["start_waypoint_index"],
                              "end_waypoint_index": record["end_waypoint_index"]} for record in row_records]},
        "flights": [flight],
        "statistics": {
            "survey_area_m2": 0.0,
            "route_distance_m": flight["route_distance_m"],
            "outbound_distance_m": None,
            "recovery_distance_m": flight["recovery_distance_m"],
            "known_distance_m": flight["known_distance_m"],
            "total_distance_m": None,
            "known_estimated_seconds": flight["known_estimated_seconds"],
            "total_estimated_seconds": None,
            "photo_count": photo_count,
            "photo_count_kind": "actions",
            "strip_count": len(row_records),
            "waypoint_count": len(waypoints),
            "battery_count": 1,
            "estimates_complete": False,
        },
        "capture": {
            "mode": "full_auto",
            "profile_interval_s": interval,
            # The spacing between photos along the first (horizon) lap.
            "shot_spacing_m": 2 * math.pi * first["radius_m"] / first["photo_count"],
            "side_overlap_ratio": overlap,
        },
        "assumptions": [
            {"code": "small_lap_parallax", "message": "Laps of a few metres around the point are treated as one viewpoint."},
            {"code": "straight_segment_estimate", "message": "Distance and time use straight segments between waypoints."},
            {"code": "unmodeled_operations", "message": "Takeoff, climb, landing, and turning delays are not modeled."},
        ],
        "warnings": [_issue("travel_estimate_incomplete", "Flight 1 is missing launch or recovery location travel.")],
        "validation": {"export_allowed": not blocking, "errors": blocking},
    }
    _ensure_finite_json(result)
    return result


def _rows(horizontal_fov, vertical_fov, overlap, include_nadir):
    """Camera rows from the horizon down: pitch, photo count and lap radius.

    The pitches split the 90 degrees from the horizon to straight down into
    equal steps no larger than the photo height less its overlap. With the
    nadir photo, the last step ends at -90; without it, the lowest row stops
    one step short."""
    step = vertical_fov * (1 - overlap)
    intervals = max(1, math.ceil(number(90.0 / step)))
    rows = []
    for row in range(intervals + 1 if include_nadir else intervals):
        pitch = number(-90.0 * row / intervals) + 0.0     # 0.0, never -0.0
        nadir = row == intervals
        if nadir:
            count = 1
        else:
            # The row's top edge is where neighbouring photos are narrowest in heading.
            top = min(0.0, pitch + vertical_fov / 2)
            count = max(MIN_ROW_PHOTOS, math.ceil(number(
                360.0 * math.cos(math.radians(top)) / (horizontal_fov * (1 - overlap)))))
        rows.append({"gimbal_pitch_deg": pitch, "photo_count": count, "nadir": nadir,
                     "radius_m": 0.0 if nadir else FIRST_LAP_RADIUS_M + LAP_GAP_M * row})
    return rows
