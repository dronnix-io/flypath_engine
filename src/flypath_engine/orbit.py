"""Versioned orbit planning: rings around a centre point for oblique 3D capture.

The drone flies a circle of a given radius around a centre point, with its
heading pointed at the centre and the gimbal tilted to an oblique pitch, so the
photos can feed 3D model generation. Results reuse the plan_2d shape (route,
flights, statistics, capture, validation) and add a heading per waypoint.

A request can also list up to three rings around the same centre, each with its
own radius, altitude and camera pitch. They are flown in order: from the last
waypoint of a ring straight to the first waypoint of the next. With splitting
enabled, the route is divided into flights, whole rings first, and a ring is
only cut into parts when it does not fit one flight on its own. A request
without ``rings`` and without splitting keeps the v1.2.0 result shape.
"""

import itertools
import math

from .measurements import _WGS84, route_distance_m
from .planning import (
    CONTRACT_VERSION, ENGINE_VERSION, MAX_ALTITUDE_M, MAX_WAYPOINT_LIMIT, PlanningError,
    _balanced_boundaries, _complete_sum, _ensure_finite_json, _integer, _issue, _keys, _number,
    _position, _version,
)
from .profiles import PROFILE_VERSION, drone_profile
from .precision import versioned, number


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


@versioned
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
    _keys(split, {"enabled", "requested_flights", "max_waypoints_per_flight"}, "split")
    split_enabled = split.get("enabled", False)
    if type(split_enabled) is not bool:
        raise PlanningError("invalid_split", "split.enabled must be a boolean.", "split.enabled")
    requested_flights = _integer(split.get("requested_flights", 1), "split.requested_flights", minimum=1)
    if not split_enabled and requested_flights != 1:
        raise PlanningError("invalid_split", "requested_flights needs splitting enabled.",
                            "split.requested_flights")
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
        "split_enabled": split_enabled, "requested_flights": requested_flights,
    }
    if "rings" in request or split_enabled:
        result = _rings_result(planned, common)
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
    slant = number(math.hypot(radius, altitude))
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
        "headings": headings, "edges": edges, "route_distance": route_distance,
        "shot_spacing": shot_spacing,
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


def _flight_record(index, start, end, actions, route_distance, recovery, photo_count,
                   capture_seconds, startup_seconds, common):
    """One flight over route waypoints start to end, inclusive."""
    known_distance = route_distance + (recovery or 0)
    known_seconds = known_distance / common["speed"] + capture_seconds + startup_seconds
    full_auto = common["mode"] == "full_auto"
    return {
        "index": index,
        "route_start_waypoint_index": start,
        "route_end_waypoint_index": end,
        "waypoint_count": end - start + 1,
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


def _flight_and_checks(points, actions, route_distance, recovery, photo_count, capture_seconds,
                       startup_seconds, common):
    """The single-ring flight record and its blocking checks (v1.2.0)."""
    flight = _flight_record(0, 0, len(points) - 1, actions, route_distance, recovery, photo_count,
                            capture_seconds, startup_seconds, common)
    known_seconds = flight["known_estimated_seconds"]
    blocking = []
    if len(points) > common["waypoint_limit"]:
        blocking.append(_issue("waypoint_limit_exceeded",
                               "The orbit needs more waypoints than the limit; lower the overlap or the radius."))
    if known_seconds > common["usable_seconds"]:
        blocking.append(_issue("battery_time_exceeded", "The orbit exceeds the usable battery-time limit."))
    return flight, blocking


def _result(common, orbit, waypoints, strips, flights, blocking, survey_area, capture):
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
        "flights": flights,
        "statistics": {
            "survey_area_m2": survey_area,
            "route_distance_m": sum(flight["route_distance_m"] for flight in flights),
            "outbound_distance_m": None,
            "recovery_distance_m": _complete_sum(flight["recovery_distance_m"] for flight in flights),
            "known_distance_m": sum(flight["known_distance_m"] for flight in flights),
            "total_distance_m": None,
            "known_estimated_seconds": sum(flight["known_estimated_seconds"] for flight in flights),
            "total_estimated_seconds": None,
            "photo_count": sum(flight["photo_count"] for flight in flights),
            "photo_count_kind": flights[0]["photo_count_kind"],
            "strip_count": len(strips),
            "waypoint_count": len(waypoints),
            "battery_count": len(flights),
            "estimates_complete": False,
        },
        "capture": capture,
        "assumptions": [
            {"code": "centre_at_takeoff_level", "message": "Overlap uses the slant distance to the centre at takeoff ground level."},
            {"code": "straight_segment_estimate", "message": "Distance and time use straight segments between waypoints."},
            {"code": "unmodeled_operations", "message": "Takeoff, climb, landing, and additional turn delays are not modeled."},
        ],
        "warnings": [_issue("travel_estimate_incomplete",
                            "Flight %d is missing launch or recovery location travel." % (flight["index"] + 1))
                     for flight in flights],
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
                   [flight], blocking, math.pi * ring["radius"] * ring["radius"], capture)


def _rings_result(rings, common):
    """Rings flown in order, each starting at the start bearing, in one or more flights."""
    full_auto = common["mode"] == "full_auto"
    waypoints, edges, strips, ring_records, ring_of = [], [], [], [], []
    for ring_index, ring in enumerate(rings):
        start = len(waypoints)
        if ring_index:
            # Transition: straight from the previous ring's last waypoint to
            # this ring's first, climbing or descending to its altitude.
            previous = rings[ring_index - 1]
            edges.append(math.hypot(route_distance_m([previous["points"][-1], ring["points"][0]]),
                                    ring["altitude"] - previous["altitude"]))
        edges.extend(ring["edges"])
        for offset, point in enumerate(ring["points"]):
            roles = {"orbit_point"}
            if offset == 0:
                roles.add("ring_start")
            if offset == len(ring["points"]) - 1:
                roles.add("ring_end")
            waypoints.append({"index": start + offset, "position": point, "roles": roles,
                              "heading_deg": ring["headings"][offset], "altitude_m": ring["altitude"],
                              "ring_index": ring_index})
            ring_of.append(ring_index)
        end = len(waypoints) - 1
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

    if full_auto:
        shot_spacing = min(ring["shot_spacing"] for ring in rings)
    else:
        # Interval capture runs for the whole flight, transitions included.
        shot_spacing = rings[0]["shot_spacing"]
    route = {"points": [waypoint["position"] for waypoint in waypoints], "edges": edges,
             "rings": rings, "ring_of": ring_of,
             "ring_starts": {strip["start_waypoint_index"] for strip in strips},
             "shot_spacing": shot_spacing}

    blocking = []
    boundaries = [(0, len(waypoints) - 1)]
    if common["split_enabled"]:
        split = _split_boundaries(strips, route, common)
        if split is None:
            blocking.append(_issue("flight_cannot_fit",
                                   "No split fits the waypoint and usable battery-time limits."))
        else:
            boundaries = split
    flights = [_ring_flight(index, start, end, route, common)
               for index, (start, end) in enumerate(boundaries)]
    if not common["split_enabled"]:
        whole = flights[0]
        if whole["waypoint_count"] > common["waypoint_limit"]:
            blocking.append(_issue(
                "splitting_required_waypoint_limit",
                "The orbit needs %d waypoints, more than the limit of %d. Enable splitting, raise the "
                "waypoint limit, or lower the overlap." % (whole["waypoint_count"], common["waypoint_limit"])))
        if whole["known_estimated_seconds"] > common["usable_seconds"]:
            blocking.append(_issue(
                "splitting_required_battery_limit",
                "The orbit needs about %d minutes of flight, more than the %d of one battery. "
                "Enable splitting." % (math.ceil(whole["known_estimated_seconds"] / 60),
                                       common["usable_seconds"] // 60)))
    elif not blocking:
        for flight in flights:
            if flight["waypoint_count"] > common["waypoint_limit"]:
                blocking.append(_issue("waypoint_limit_exceeded",
                                       "Flight %d exceeds the waypoint limit." % (flight["index"] + 1)))
            if flight["known_estimated_seconds"] > common["usable_seconds"]:
                blocking.append(_issue("battery_time_exceeded",
                                       "Flight %d exceeds the usable battery-time limit." % (flight["index"] + 1)))
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
    return _result(common, orbit, waypoints, strips, flights, blocking, math.pi * largest * largest, capture)


def _ring_flight(index, start, end, route, common):
    """One flight over route waypoints start to end.

    The camera tilts to the ring's pitch where the flight starts and where each
    ring starts; full auto also hovers there and photographs every waypoint."""
    full_auto = common["mode"] == "full_auto"
    rings, ring_of, points = route["rings"], route["ring_of"], route["points"]
    route_distance = sum(route["edges"][start:end])
    recovery = _recovery(points[start], points[end], common["finish_action"],
                         rings[ring_of[end]]["altitude"] - rings[ring_of[start]]["altitude"])
    actions, tilts = [], 0
    for waypoint_index in range(start, end + 1):
        if waypoint_index == start or waypoint_index in route["ring_starts"]:
            tilts += 1
            actions.append({"type": "rotate_camera", "waypoint_index": waypoint_index,
                            "pitch_deg": rings[ring_of[waypoint_index]]["pitch"]})
            if full_auto:
                actions.append({"type": "hover", "waypoint_index": waypoint_index, "duration_s": STARTUP_WAIT_S})
        if full_auto:
            actions.append({"type": "take_photo", "waypoint_index": waypoint_index})
    if full_auto:
        photo_count = end - start + 1
        capture_seconds = photo_count * max(2.5, common["interval"])
        startup_seconds = STARTUP_WAIT_S * tilts
    else:
        photo_count = math.floor(route_distance / route["shot_spacing"]) + 1
        capture_seconds = startup_seconds = 0.0
    return _flight_record(index, start, end, actions, route_distance, recovery, photo_count,
                          capture_seconds, startup_seconds, common)


def _split_boundaries(strips, route, common):
    """Flight boundaries: whole rings grouped where they fit, else rings cut into parts.

    A flight of whole rings starts at a ring's first waypoint. Parts of one
    ring share their seam waypoint, like plan_2d flights. Returns None when no
    split fits the waypoint and usable battery-time limits."""
    limit, budget = common["waypoint_limit"], common["usable_seconds"]
    requested = common["requested_flights"]

    def longest(boundaries):
        """The longest flight in seconds, or None when a flight does not fit."""
        seconds = 0.0
        for start, end in boundaries:
            flight = _ring_flight(0, start, end, route, common)
            if flight["waypoint_count"] > limit or flight["known_estimated_seconds"] > budget:
                return None
            seconds = max(seconds, flight["known_estimated_seconds"])
        return seconds

    rings = len(strips)
    for count in range(requested, rings + 1):
        best = None
        for cuts in itertools.combinations(range(1, rings), count - 1):
            groups = (0,) + cuts + (rings,)
            boundaries = [(strips[groups[i]]["start_waypoint_index"],
                           strips[groups[i + 1] - 1]["end_waypoint_index"]) for i in range(count)]
            seconds = longest(boundaries)
            if seconds is not None and (best is None or seconds < best[0] - 1e-9):
                best = (seconds, boundaries)
        if best is not None:
            return best[1]

    # A ring does not fit one flight, or more flights were requested than
    # there are rings: each ring flies on its own, cut into as few parts as fit.
    edges = [strip["end_waypoint_index"] - strip["start_waypoint_index"] for strip in strips]
    if requested > sum(edges):
        raise PlanningError("too_many_requested_flights",
                            "requested_flights exceeds the available route edges.", "split.requested_flights")
    parts = []
    for strip, ring_edges in zip(strips, edges):
        for count in range(1, ring_edges + 1):
            ring_parts = _ring_parts(strip, count, route, common)
            if ring_parts is not None:
                parts.append(ring_parts)
                break
        else:
            return None
    while sum(len(ring_parts) for ring_parts in parts) < requested:
        # One more part for the ring whose longest flight is longest.
        _seconds, index = max((longest(ring_parts), index) for index, ring_parts in enumerate(parts)
                              if len(ring_parts) < edges[index])
        parts[index] = _ring_parts(strips[index], len(parts[index]) + 1, route, common)
        if parts[index] is None:
            return None
    return [boundary for ring_parts in parts for boundary in ring_parts]


def _ring_parts(strip, count, route, common):
    """One ring cut into count balanced flights that share their seams."""
    start, end = strip["start_waypoint_index"], strip["end_waypoint_index"]
    found = _balanced_boundaries(
        route["points"][start:end + 1], route["edges"][start:end], count, common["waypoint_limit"],
        common["speed"], common["mode"], max(2.5, common["interval"]), common["usable_seconds"],
        common["finish_action"], None)
    if not found:
        return None
    return [(start + first, start + last) for first, last in found]


def _signed_degrees(value):
    """Normalise an angle to DJI's heading range (-180, 180]."""
    value = (value + 180.0) % 360.0 - 180.0
    return 180.0 if value == -180.0 else value
