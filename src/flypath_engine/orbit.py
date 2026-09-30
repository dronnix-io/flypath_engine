"""Versioned orbit planning: one ring around a centre point for oblique 3D capture.

The drone flies a circle of a given radius around a centre point, with its
heading pointed at the centre and the gimbal tilted to an oblique pitch, so the
photos can feed 3D model generation. Results reuse the plan_2d shape (route,
flights, statistics, capture, validation) and add a heading per waypoint.
"""

import math

from .measurements import _WGS84, route_distance_m
from .planning import (
    CONTRACT_VERSION, ENGINE_VERSION, MAX_ALTITUDE_M, MAX_WAYPOINT_LIMIT, PlanningError,
    _ensure_finite_json, _integer, _issue, _keys, _number, _position, _version,
)
from .profiles import PROFILE_VERSION, drone_profile


MIN_RADIUS_M = 5
MAX_RADIUS_M = 2_000
MIN_RING_WAYPOINTS = 12          # enough for a curved DJI path to trace a circle
SEMI_AUTO_SEGMENT_M = 15.0       # target arc length between semi-auto waypoints
MAX_SEMI_AUTO_WAYPOINTS = 72
STARTUP_WAIT_S = 3.0
DIRECTIONS = ("clockwise", "counterclockwise")
FINISH_ACTIONS = ("return_to_home", "return_to_first_waypoint", "hover", "land")


def plan_orbit(request):
    """Return one deterministic, JSON-compatible orbit planning result."""
    if not isinstance(request, dict):
        raise PlanningError("invalid_request", "Planning request must be an object.")
    _keys(request, {
        "contract_version", "operation", "source_mission_id", "mapping_style",
        "drone_profile_id", "profile_version", "centre", "radius_m", "altitude_m",
        "gimbal_pitch_deg", "direction", "start_bearing_deg", "speed_m_s",
        "capture", "finish_action", "split", "terrain_follow",
    }, "request")
    if request.get("operation", "plan_orbit") != "plan_orbit":
        raise PlanningError("unsupported_operation", "Only plan_orbit is supported here.", "operation")
    if request.get("mapping_style", "orbit") != "orbit":
        raise PlanningError("unsupported_feature", "plan_orbit only plans orbit missions.", "mapping_style")
    _version(request.get("contract_version"), CONTRACT_VERSION, "contract_version")
    _version(request.get("profile_version"), PROFILE_VERSION, "profile_version")
    if request.get("terrain_follow") not in (None, False):
        raise PlanningError("unsupported_feature", "terrain_follow is not supported for orbit missions.", "terrain_follow")

    profile_id = request.get("drone_profile_id")
    try:
        profile_name, profile = drone_profile(profile_id)
    except (KeyError, TypeError):
        raise PlanningError("profile_not_found", "Drone profile is not available.", "drone_profile_id") from None
    aircraft, camera = profile["aircraft"], profile["camera"]

    centre = _position(request.get("centre"), "centre")
    radius = _number(request.get("radius_m"), "radius_m", minimum=MIN_RADIUS_M, maximum=MAX_RADIUS_M)
    altitude = _number(request.get("altitude_m"), "altitude_m", minimum=0, maximum=MAX_ALTITUDE_M, exclusive=True)
    pitch = _number(request.get("gimbal_pitch_deg"), "gimbal_pitch_deg", minimum=-90, maximum=0)
    speed = _number(request.get("speed_m_s"), "speed_m_s", minimum=0, exclusive=True)
    if not aircraft["min_speed_ms"] <= speed <= aircraft["max_speed_ms"]:
        raise PlanningError("speed_out_of_profile_range", "Speed is outside the drone profile range.", "speed_m_s")
    direction = request.get("direction", "clockwise")
    if direction not in DIRECTIONS:
        raise PlanningError("invalid_direction", "Direction must be clockwise or counterclockwise.", "direction")
    start_bearing = _number(request.get("start_bearing_deg", 0), "start_bearing_deg") % 360
    finish_action = request.get("finish_action", "return_to_home")
    if finish_action not in FINISH_ACTIONS:
        raise PlanningError("invalid_finish_action", "Finish action is not supported.", "finish_action")

    split = request.get("split")
    if not isinstance(split, dict):
        raise PlanningError("invalid_split", "split must be an object.", "split")
    _keys(split, {"enabled", "max_waypoints_per_flight"}, "split")
    if split.get("enabled", False) is not False:
        raise PlanningError("unsupported_feature", "Splitting is not supported for orbit missions.", "split.enabled")
    waypoint_limit = _integer(split.get("max_waypoints_per_flight"), "split.max_waypoints_per_flight",
                              minimum=2, maximum=MAX_WAYPOINT_LIMIT)

    capture = request.get("capture")
    if not isinstance(capture, dict) or capture.get("mode") not in ("semi_auto", "full_auto"):
        raise PlanningError("invalid_capture_mode", "Capture mode must be semi_auto or full_auto.", "capture.mode")
    _keys(capture, {"mode", "interval_s", "side_overlap_ratio"}, "capture")
    mode = capture["mode"]
    interval = _number(camera.get("min_shoot_interval_s"), "drone_profile.camera.min_shoot_interval_s",
                       minimum=0, exclusive=True)
    if "interval_s" in capture and _number(capture["interval_s"], "capture.interval_s",
                                           minimum=0, exclusive=True) != interval:
        raise PlanningError("profile_interval_override", "Capture interval is owned by the drone profile.",
                            "capture.interval_s")

    # Image width seen across the ring: the camera looks at the centre, so
    # neighbouring photos overlap across the image width at the slant distance.
    slant = math.hypot(radius, altitude)
    footprint = slant * _number(camera.get("sensor_width_mm"), "drone_profile.camera.sensor_width_mm",
                                minimum=0, exclusive=True) / _number(
        camera.get("focal_length_mm"), "drone_profile.camera.focal_length_mm", minimum=0, exclusive=True)
    circumference = 2 * math.pi * radius

    if mode == "full_auto":
        overlap = _number(capture.get("side_overlap_ratio"), "capture.side_overlap_ratio",
                          minimum=0, maximum=1, maximum_exclusive=True)
        spacing = max(footprint * (1 - overlap), .5)
        count = max(MIN_RING_WAYPOINTS, math.ceil(circumference / spacing))
        closing = False
    else:
        if "side_overlap_ratio" in capture:
            raise PlanningError("unsupported_feature", "Semi-auto overlap follows speed and the photo interval.",
                                "capture.side_overlap_ratio")
        count = min(MAX_SEMI_AUTO_WAYPOINTS,
                    max(MIN_RING_WAYPOINTS, math.ceil(circumference / SEMI_AUTO_SEGMENT_M)))
        closing = True

    step = 360.0 / count
    sign = 1 if direction == "clockwise" else -1
    bearings = [start_bearing + sign * step * index for index in range(count)]
    if closing:
        bearings.append(start_bearing + sign * 360.0)
    points, headings = [], []
    for bearing in bearings:
        longitude, latitude, back = _WGS84.fwd(centre["longitude_deg"], centre["latitude_deg"],
                                               bearing % 360, radius)
        points.append({"latitude_deg": latitude, "longitude_deg": longitude})
        headings.append(_signed_degrees(back))

    edges = [route_distance_m(points[index:index + 2]) for index in range(len(points) - 1)]
    route_distance = sum(edges)
    if mode == "full_auto":
        shot_spacing = circumference / count
        achieved_overlap = overlap
        photo_count = count
    else:
        shot_spacing = max(speed * interval, .5)
        achieved_overlap = max(0.0, 1 - shot_spacing / footprint)
        photo_count = math.floor(route_distance / shot_spacing)

    usable_seconds = _number(aircraft.get("battery_safe_min"), "drone_profile.aircraft.battery_safe_min",
                             minimum=0, exclusive=True) * 60
    photo_allowance = max(2.5, interval)
    last = len(points) - 1
    if finish_action == "return_to_first_waypoint":
        recovery = route_distance_m([points[last], points[0]])
    elif finish_action == "return_to_home":
        recovery = None
    else:
        recovery = 0.0
    known_distance = route_distance + (recovery or 0)
    capture_seconds = photo_count * photo_allowance if mode == "full_auto" else 0.0
    startup_seconds = STARTUP_WAIT_S if mode == "full_auto" else 0.0
    known_seconds = known_distance / speed + capture_seconds + startup_seconds

    if mode == "full_auto":
        actions = [
            {"type": "rotate_camera", "waypoint_index": 0, "pitch_deg": pitch},
            {"type": "hover", "waypoint_index": 0, "duration_s": STARTUP_WAIT_S},
        ] + [{"type": "take_photo", "waypoint_index": index} for index in range(count)]
    else:
        actions = [{"type": "rotate_camera", "waypoint_index": 0, "pitch_deg": pitch}]

    flight = {
        "index": 0,
        "route_start_waypoint_index": 0,
        "route_end_waypoint_index": last,
        "waypoint_count": len(points),
        "actions": actions,
        "route_distance_m": route_distance,
        "outbound_distance_m": None,
        "recovery_distance_m": recovery,
        "known_distance_m": known_distance,
        "total_distance_m": None,
        "capture_allowance_seconds": capture_seconds,
        "startup_wait_seconds": startup_seconds,
        "known_estimated_seconds": known_seconds,
        "total_estimated_seconds": None,
        "usable_battery_seconds": usable_seconds,
        "estimates_complete": False,
        "photo_count": photo_count,
        "photo_count_kind": "actions" if mode == "full_auto" else "estimate",
    }

    warnings = [_issue("travel_estimate_incomplete", "Flight 1 is missing launch or recovery location travel.")]
    blocking = []
    if len(points) > waypoint_limit:
        blocking.append(_issue("waypoint_limit_exceeded",
                               "The orbit needs more waypoints than the limit; lower the overlap or the radius."))
    if known_seconds > usable_seconds:
        blocking.append(_issue("battery_time_exceeded", "The orbit exceeds the usable battery-time limit."))

    roles = [{"orbit_point"} for _ in points]
    roles[0].add("route_start")
    roles[last].add("route_end")
    waypoints = [
        {"index": index, "position": point, "roles": sorted(roles[index]), "heading_deg": headings[index]}
        for index, point in enumerate(points)
    ]
    result = {
        "contract_version": CONTRACT_VERSION,
        "engine_version": ENGINE_VERSION,
        "profile_version": PROFILE_VERSION,
        "operation": "plan_orbit",
        "mapping_style": "orbit",
        "drone_profile_id": profile_id,
        "drone_profile_name": profile_name,
        "turn_style": "curved",
        "finish_action": finish_action,
        "orbit": {
            "centre": centre,
            "radius_m": radius,
            "altitude_m": altitude,
            "gimbal_pitch_deg": pitch,
            "direction": direction,
            "start_bearing_deg": start_bearing,
            "slant_distance_m": slant,
            "angular_step_deg": step,
        },
        "route": {
            "waypoints": waypoints,
            "strips": [{"index": 0, "start_waypoint_index": 0, "end_waypoint_index": last}],
        },
        "flights": [flight],
        "statistics": {
            "survey_area_m2": math.pi * radius * radius,
            "route_distance_m": route_distance,
            "outbound_distance_m": None,
            "recovery_distance_m": recovery,
            "known_distance_m": known_distance,
            "total_distance_m": None,
            "known_estimated_seconds": known_seconds,
            "total_estimated_seconds": None,
            "photo_count": photo_count,
            "photo_count_kind": flight["photo_count_kind"],
            "strip_count": 1,
            "waypoint_count": len(points),
            "battery_count": 1,
            "estimates_complete": False,
        },
        "capture": {
            "mode": mode,
            "profile_interval_s": interval,
            "shot_spacing_m": shot_spacing,
            "side_overlap_ratio": achieved_overlap,
        },
        "assumptions": [
            {"code": "centre_at_takeoff_level", "message": "Overlap uses the slant distance to the centre at takeoff ground level."},
            {"code": "straight_segment_estimate", "message": "Distance and time use straight segments between waypoints."},
            {"code": "unmodeled_operations", "message": "Takeoff, climb, landing, and additional turn delays are not modeled."},
        ],
        "warnings": warnings,
        "validation": {"export_allowed": not blocking, "errors": blocking},
    }
    _ensure_finite_json(result)
    return result


def _signed_degrees(value):
    """Normalise an angle to DJI's heading range (-180, 180]."""
    value = (value + 180.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 else value
