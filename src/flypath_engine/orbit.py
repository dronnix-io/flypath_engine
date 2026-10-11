"""Versioned orbit planning: rings around a centre point for oblique 3D capture.

The drone flies a circle of a given radius around a centre point, with its
heading pointed at the centre and the gimbal tilted to an oblique pitch, so the
photos can feed 3D model generation. Results reuse the plan_2d shape (route,
flights, statistics, capture, validation) and add a heading per waypoint.

A request can also list up to three rings around the same centre, each with its
own radius, altitude and camera pitch. They are flown in order as one flight:
from the last waypoint of a ring straight to the first waypoint of the next.
A request without ``rings`` is a single ring and keeps the v1.2.0 result shape.
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
MAX_RINGS = 3
MIN_RING_WAYPOINTS = 12          # enough for a curved DJI path to trace a circle
SEMI_AUTO_SEGMENT_M = 15.0       # target arc length between semi-auto waypoints
MAX_SEMI_AUTO_WAYPOINTS = 72
STARTUP_WAIT_S = 3.0
DIRECTIONS = ("clockwise", "counterclockwise")
FINISH_ACTIONS = ("return_to_home", "return_to_first_waypoint", "hover", "land")
RING_FIELDS = ("radius_m", "altitude_m", "gimbal_pitch_deg")


def plan_orbit(request):
    """Return one deterministic, JSON-compatible orbit planning result."""
    if not isinstance(request, dict):
        raise PlanningError("invalid_request", "Planning request must be an object.")
    _keys(request, {
        "contract_version", "operation", "source_mission_id", "mapping_style",
        "drone_profile_id", "profile_version", "centre", "radius_m", "altitude_m",
        "gimbal_pitch_deg", "rings", "direction", "start_bearing_deg", "speed_m_s",
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
    rings = _rings(request)
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
    if mode == "full_auto":
        overlap = _number(capture.get("side_overlap_ratio"), "capture.side_overlap_ratio",
                          minimum=0, maximum=1, maximum_exclusive=True)
    else:
        if "side_overlap_ratio" in capture:
            raise PlanningError("unsupported_feature", "Semi-auto overlap follows speed and the photo interval.",
                                "capture.side_overlap_ratio")
        overlap = None
    lens = (_number(camera.get("sensor_width_mm"), "drone_profile.camera.sensor_width_mm",
                    minimum=0, exclusive=True),
            _number(camera.get("focal_length_mm"), "drone_profile.camera.focal_length_mm",
                    minimum=0, exclusive=True))
    sign = 1 if direction == "clockwise" else -1
    prefix = "rings[%d]." if "rings" in request else ""
    planned = [_plan_ring(ring, centre, lens, overlap, speed, interval, start_bearing, sign,
                          prefix % index if prefix else "")
               for index, ring in enumerate(rings)]

    usable_seconds = _number(aircraft.get("battery_safe_min"), "drone_profile.aircraft.battery_safe_min",
                             minimum=0, exclusive=True) * 60
    common = {
        "centre": centre, "direction": direction, "start_bearing": start_bearing, "mode": mode,
        "speed": speed, "interval": interval, "finish_action": finish_action,
        "waypoint_limit": waypoint_limit, "usable_seconds": usable_seconds,
        "profile_id": profile_id, "profile_name": profile_name,
    }
    if "rings" in request:
        result = _multi_ring_result(planned, common)
    else:
        result = _single_ring_result(planned[0], common)
    _ensure_finite_json(result)
    return result


def _rings(request):
    """The validated rings: the request's ``rings`` list, or its single ring."""
    if "rings" not in request:
        return [{field: request.get(field) for field in RING_FIELDS}]
    if any(field in request for field in RING_FIELDS):
        raise PlanningError("invalid_rings", "Give either rings or a single radius, altitude and pitch.", "rings")
    rings = request["rings"]
    if not isinstance(rings, list) or not 1 <= len(rings) <= MAX_RINGS:
        raise PlanningError("invalid_rings", "An orbit has 1 to %d rings." % MAX_RINGS, "rings")
    for index, ring in enumerate(rings):
        if not isinstance(ring, dict):
            raise PlanningError("invalid_rings", "Each ring must be an object.", "rings[%d]" % index)
        _keys(ring, set(RING_FIELDS), "rings[%d]" % index)
    return rings


def _plan_ring(ring, centre, lens, overlap, speed, interval, start_bearing, sign, field=""):
    """Waypoints, headings and capture figures for one ring."""
    radius = _number(ring.get("radius_m"), field + "radius_m", minimum=MIN_RADIUS_M, maximum=MAX_RADIUS_M)
    altitude = _number(ring.get("altitude_m"), field + "altitude_m", minimum=0, maximum=MAX_ALTITUDE_M,
                       exclusive=True)
    pitch = _number(ring.get("gimbal_pitch_deg"), field + "gimbal_pitch_deg", minimum=-90, maximum=0)

    # Image width seen across the ring: the camera looks at the centre, so
    # neighbouring photos overlap across the image width at the slant distance.
    slant = math.hypot(radius, altitude)
    footprint = slant * lens[0] / lens[1]     # sensor width / focal length, in v1.2.0's order
    circumference = 2 * math.pi * radius

    if overlap is not None:
        spacing = max(footprint * (1 - overlap), .5)
        count = max(MIN_RING_WAYPOINTS, math.ceil(circumference / spacing))
        step = 360.0 / count
        sweep = step * (count - 1)
    else:
        count = min(MAX_SEMI_AUTO_WAYPOINTS,
                    max(MIN_RING_WAYPOINTS, math.ceil(circumference / SEMI_AUTO_SEGMENT_M)))
        step = 360.0 / count
        # The ring stops short of the start instead of closing on it. The gap
        # is no wider than one photo spacing, so the last and first photos
        # still overlap as much as any other neighbouring pair.
        seam = min(step, 360.0 * max(speed * interval, .5) / circumference)
        sweep = 360.0 - seam

    bearings = [start_bearing + sign * sweep * index / (count - 1) for index in range(count)]
    points, headings = [], []
    for bearing in bearings:
        longitude, latitude, back = _WGS84.fwd(centre["longitude_deg"], centre["latitude_deg"],
                                               bearing % 360, radius)
        points.append({"latitude_deg": latitude, "longitude_deg": longitude})
        headings.append(_signed_degrees(back))

    edges = [route_distance_m(points[index:index + 2]) for index in range(len(points) - 1)]
    route_distance = sum(edges)
    if overlap is not None:
        shot_spacing = circumference / count
        achieved_overlap = overlap
        photo_count = count
    else:
        shot_spacing = max(speed * interval, .5)
        achieved_overlap = max(0.0, 1 - shot_spacing / footprint)
        photo_count = math.floor(route_distance / shot_spacing) + 1   # the first shot is at the start
    return {
        "radius": radius, "altitude": altitude, "pitch": pitch, "slant": slant,
        "angular_step": sweep / (count - 1), "count": count, "points": points,
        "headings": headings, "route_distance": route_distance, "shot_spacing": shot_spacing,
        "overlap": achieved_overlap, "photo_count": photo_count,
    }


def _ring_actions(start, ring, full_auto):
    """The camera tilt at a ring's first waypoint, plus full-auto capture."""
    if not full_auto:
        return [{"type": "rotate_camera", "waypoint_index": start, "pitch_deg": ring["pitch"]}]
    return [
        {"type": "rotate_camera", "waypoint_index": start, "pitch_deg": ring["pitch"]},
        {"type": "hover", "waypoint_index": start, "duration_s": STARTUP_WAIT_S},
    ] + [{"type": "take_photo", "waypoint_index": start + index} for index in range(ring["count"])]


def _recovery(first, last, finish_action, height_change=0.0):
    if finish_action == "return_to_first_waypoint":
        return math.hypot(route_distance_m([last, first]), height_change)
    if finish_action == "return_to_home":
        return None
    return 0.0


def _flight_and_checks(points, actions, route_distance, recovery, photo_count, capture_seconds,
                       startup_seconds, common):
    """The single flight record and its blocking checks."""
    known_distance = route_distance + (recovery or 0)
    known_seconds = known_distance / common["speed"] + capture_seconds + startup_seconds
    full_auto = common["mode"] == "full_auto"
    flight = {
        "index": 0,
        "route_start_waypoint_index": 0,
        "route_end_waypoint_index": len(points) - 1,
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
        "usable_battery_seconds": common["usable_seconds"],
        "estimates_complete": False,
        "photo_count": photo_count,
        "photo_count_kind": "actions" if full_auto else "estimate",
    }
    blocking = []
    if len(points) > common["waypoint_limit"]:
        blocking.append(_issue("waypoint_limit_exceeded",
                               "The orbit needs more waypoints than the limit; lower the overlap or the radius."))
    if known_seconds > common["usable_seconds"]:
        blocking.append(_issue("battery_time_exceeded", "The orbit exceeds the usable battery-time limit."))
    return flight, blocking


def _result(common, orbit, waypoints, strips, flight, blocking, survey_area, capture):
    return {
        "contract_version": CONTRACT_VERSION,
        "engine_version": ENGINE_VERSION,
        "profile_version": PROFILE_VERSION,
        "operation": "plan_orbit",
        "mapping_style": "orbit",
        "drone_profile_id": common["profile_id"],
        "drone_profile_name": common["profile_name"],
        "turn_style": "curved",
        "finish_action": common["finish_action"],
        "orbit": orbit,
        "route": {"waypoints": waypoints, "strips": strips},
        "flights": [flight],
        "statistics": {
            "survey_area_m2": survey_area,
            "route_distance_m": flight["route_distance_m"],
            "outbound_distance_m": None,
            "recovery_distance_m": flight["recovery_distance_m"],
            "known_distance_m": flight["known_distance_m"],
            "total_distance_m": None,
            "known_estimated_seconds": flight["known_estimated_seconds"],
            "total_estimated_seconds": None,
            "photo_count": flight["photo_count"],
            "photo_count_kind": flight["photo_count_kind"],
            "strip_count": len(strips),
            "waypoint_count": flight["waypoint_count"],
            "battery_count": 1,
            "estimates_complete": False,
        },
        "capture": capture,
        "assumptions": [
            {"code": "centre_at_takeoff_level", "message": "Overlap uses the slant distance to the centre at takeoff ground level."},
            {"code": "straight_segment_estimate", "message": "Distance and time use straight segments between waypoints."},
            {"code": "unmodeled_operations", "message": "Takeoff, climb, landing, and additional turn delays are not modeled."},
        ],
        "warnings": [_issue("travel_estimate_incomplete", "Flight 1 is missing launch or recovery location travel.")],
        "validation": {"export_allowed": not blocking, "errors": blocking},
    }


def _single_ring_result(ring, common):
    """The v1.2.0 single-ring result, unchanged."""
    points, last = ring["points"], len(ring["points"]) - 1
    full_auto = common["mode"] == "full_auto"
    recovery = _recovery(points[0], points[last], common["finish_action"])
    capture_seconds = ring["photo_count"] * max(2.5, common["interval"]) if full_auto else 0.0
    startup_seconds = STARTUP_WAIT_S if full_auto else 0.0
    flight, blocking = _flight_and_checks(
        points, _ring_actions(0, ring, full_auto), ring["route_distance"], recovery,
        ring["photo_count"], capture_seconds, startup_seconds, common)
    roles = [{"orbit_point"} for _ in points]
    roles[0].add("route_start")
    roles[last].add("route_end")
    waypoints = [
        {"index": index, "position": point, "roles": sorted(roles[index]), "heading_deg": ring["headings"][index]}
        for index, point in enumerate(points)
    ]
    orbit = {
        "centre": common["centre"],
        "radius_m": ring["radius"],
        "altitude_m": ring["altitude"],
        "gimbal_pitch_deg": ring["pitch"],
        "direction": common["direction"],
        "start_bearing_deg": common["start_bearing"],
        "slant_distance_m": ring["slant"],
        "angular_step_deg": ring["angular_step"],
    }
    capture = {
        "mode": common["mode"],
        "profile_interval_s": common["interval"],
        "shot_spacing_m": ring["shot_spacing"],
        "side_overlap_ratio": ring["overlap"],
    }
    return _result(common, orbit, waypoints, [{"index": 0, "start_waypoint_index": 0, "end_waypoint_index": last}],
                   flight, blocking, math.pi * ring["radius"] * ring["radius"], capture)


def _multi_ring_result(rings, common):
    """Rings flown in order as one flight, each starting at the start bearing."""
    full_auto = common["mode"] == "full_auto"
    waypoints, actions, strips, ring_records = [], [], [], []
    route_distance = 0.0
    for ring_index, ring in enumerate(rings):
        start = len(waypoints)
        if ring_index:
            # Transition: straight from the previous ring's last waypoint to
            # this ring's first, climbing or descending to its altitude.
            previous = rings[ring_index - 1]
            route_distance += math.hypot(route_distance_m([previous["points"][-1], ring["points"][0]]),
                                         ring["altitude"] - previous["altitude"])
        route_distance += ring["route_distance"]
        for offset, point in enumerate(ring["points"]):
            roles = {"orbit_point"}
            if offset == 0:
                roles.add("ring_start")
            if offset == len(ring["points"]) - 1:
                roles.add("ring_end")
            waypoints.append({"index": start + offset, "position": point, "roles": roles,
                              "heading_deg": ring["headings"][offset], "altitude_m": ring["altitude"],
                              "ring_index": ring_index})
        end = len(waypoints) - 1
        actions += _ring_actions(start, ring, full_auto)
        strips.append({"index": ring_index, "start_waypoint_index": start, "end_waypoint_index": end})
        ring_records.append({
            "index": ring_index, "radius_m": ring["radius"], "altitude_m": ring["altitude"],
            "gimbal_pitch_deg": ring["pitch"], "slant_distance_m": ring["slant"],
            "angular_step_deg": ring["angular_step"], "start_waypoint_index": start,
            "end_waypoint_index": end, "waypoint_count": ring["count"],
            "shot_spacing_m": ring["shot_spacing"], "side_overlap_ratio": ring["overlap"],
        })
    waypoints[0]["roles"].add("route_start")
    waypoints[-1]["roles"].add("route_end")
    for waypoint in waypoints:
        waypoint["roles"] = sorted(waypoint["roles"])

    points = [waypoint["position"] for waypoint in waypoints]
    recovery = _recovery(points[0], points[-1], common["finish_action"],
                         rings[-1]["altitude"] - rings[0]["altitude"])
    if full_auto:
        photo_count = sum(ring["photo_count"] for ring in rings)
        shot_spacing = min(ring["shot_spacing"] for ring in rings)
    else:
        # Interval capture runs for the whole flight, transitions included.
        shot_spacing = rings[0]["shot_spacing"]
        photo_count = math.floor(route_distance / shot_spacing) + 1
    capture_seconds = photo_count * max(2.5, common["interval"]) if full_auto else 0.0
    startup_seconds = STARTUP_WAIT_S * len(rings) if full_auto else 0.0
    flight, blocking = _flight_and_checks(points, actions, route_distance, recovery, photo_count,
                                          capture_seconds, startup_seconds, common)
    orbit = {
        "centre": common["centre"],
        "direction": common["direction"],
        "start_bearing_deg": common["start_bearing"],
        "rings": ring_records,
    }
    capture = {
        "mode": common["mode"],
        "profile_interval_s": common["interval"],
        "shot_spacing_m": shot_spacing,
        # The lowest ring overlap, so the reported figure is never optimistic.
        "side_overlap_ratio": min(ring["overlap"] for ring in rings),
    }
    largest = max(ring["radius"] for ring in rings)
    return _result(common, orbit, waypoints, strips, flight, blocking, math.pi * largest * largest, capture)


def _signed_degrees(value):
    """Normalise an angle to DJI's heading range (-180, 180]."""
    value = (value + 180.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 else value
