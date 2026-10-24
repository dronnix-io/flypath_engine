"""Versioned oblique planning: grid passes with a tilted camera for 3D models.

The survey area is flown as up to five grid passes. An optional nadir pass looks
straight down, like plan_2d. Four oblique passes tilt the camera to the
requested pitch and face it one way each: along the flight lines, to their
right, back along them, and to their left. The drone holds that heading for the
whole pass, so it flies some lines backwards or sideways.

Each oblique pass is moved away from the area, against the way the camera
faces, by altitude / tan(|pitch|): the centre of every photo then lands inside
the area, so its edges are seen from all four sides without flying far past
them. Line and photo spacing use the slant distance to the ground at the photo
centre, which keeps the overlap at least as high as requested.

Every pass flies as its own flights; a flight never spans two passes. Results
reuse the plan_2d shape and add the pass, heading, and camera pitch of each
waypoint.
"""

import math

from shapely import affinity

from . import grid
from .measurements import _WGS84, survey_area_m2
from .planning import (
    CONTRACT_VERSION, DIRECTION_CONVENTION, ENGINE_VERSION, MAX_ALTITUDE_M, MAX_MARGIN_M,
    MAX_ROUTE_WAYPOINTS, MAX_WAYPOINT_LIMIT, PlanningError, _balanced_boundaries, _complete_sum,
    _edge_distances, _ensure_finite_json, _flight_location, _flight_result, _integer, _issue, _keys,
    _locations, _number, _route_metadata, _validate_area, _version,
)
from .profiles import PROFILE_VERSION, drone_profile


MIN_PITCH_DEG = -85
MAX_PITCH_DEG = -20
NADIR_PITCH_DEG = -90
OBLIQUE_PASSES = 4
MAX_OBLIQUE_ROUTE_WAYPOINTS = (OBLIQUE_PASSES + 1) * MAX_ROUTE_WAYPOINTS
FINISH_ACTIONS = ("return_to_home", "return_to_first_waypoint", "hover", "land")
# Names of the oblique passes, by the side the camera faces relative to the
# flight lines' bearing.
LOOKS = ("forward", "right", "back", "left")


def plan_oblique(request):
    """Return one deterministic, JSON-compatible oblique planning result."""
    if not isinstance(request, dict):
        raise PlanningError("invalid_request", "Planning request must be an object.")
    _keys(request, {
        "contract_version", "operation", "source_mission_id", "mapping_style", "survey_area",
        "drone_profile_id", "profile_version", "altitude_m", "speed_m_s", "side_overlap_ratio",
        "margin_m", "direction", "capture", "gimbal_pitch_deg", "include_nadir", "turn_style",
        "finish_action", "split", "locations", "terrain_follow",
    }, "request")
    if request.get("operation", "plan_oblique") != "plan_oblique":
        raise PlanningError("unsupported_operation", "Only plan_oblique is supported here.", "operation")
    if request.get("mapping_style", "oblique") != "oblique":
        raise PlanningError("unsupported_feature", "plan_oblique only plans oblique missions.", "mapping_style")
    _version(request.get("contract_version"), CONTRACT_VERSION, "contract_version")
    _version(request.get("profile_version"), PROFILE_VERSION, "profile_version")
    if request.get("terrain_follow") not in (None, False):
        raise PlanningError("unsupported_feature", "terrain_follow is not supported for oblique missions.",
                            "terrain_follow")

    profile_id = request.get("drone_profile_id")
    try:
        profile_name, profile = drone_profile(profile_id)
    except (KeyError, TypeError):
        raise PlanningError("profile_not_found", "Drone profile is not available.", "drone_profile_id") from None
    aircraft, camera = profile["aircraft"], profile["camera"]

    altitude = _number(request.get("altitude_m"), "altitude_m", minimum=0, maximum=MAX_ALTITUDE_M, exclusive=True)
    speed = _number(request.get("speed_m_s"), "speed_m_s", minimum=0, exclusive=True)
    if not aircraft["min_speed_ms"] <= speed <= aircraft["max_speed_ms"]:
        raise PlanningError("speed_out_of_profile_range", "Speed is outside the drone profile range.", "speed_m_s")
    side_overlap = _number(request.get("side_overlap_ratio"), "side_overlap_ratio", minimum=0, maximum=1,
                           maximum_exclusive=True)
    margin = _number(request.get("margin_m", 0), "margin_m", minimum=0, maximum=MAX_MARGIN_M)
    pitch = _number(request.get("gimbal_pitch_deg"), "gimbal_pitch_deg", minimum=MIN_PITCH_DEG,
                    maximum=MAX_PITCH_DEG)
    include_nadir = request.get("include_nadir", True)
    if type(include_nadir) is not bool:
        raise PlanningError("invalid_boolean", "Value must be a boolean.", "include_nadir")

    capture = request.get("capture")
    if not isinstance(capture, dict) or capture.get("mode") not in ("semi_auto", "full_auto"):
        raise PlanningError("invalid_capture_mode", "Capture mode must be semi_auto or full_auto.", "capture.mode")
    _keys(capture, {"mode", "interval_s", "front_overlap_ratio"}, "capture")
    mode = capture["mode"]
    interval = _number(camera.get("min_shoot_interval_s"), "drone_profile.camera.min_shoot_interval_s",
                       minimum=0, exclusive=True)
    if "interval_s" in capture and _number(capture["interval_s"], "capture.interval_s",
                                           minimum=0, exclusive=True) != interval:
        raise PlanningError("profile_interval_override", "Capture interval is owned by the drone profile.",
                            "capture.interval_s")
    front_overlap = None
    if mode == "full_auto":
        front_overlap = _number(capture.get("front_overlap_ratio"), "capture.front_overlap_ratio",
                                minimum=0, maximum=1, maximum_exclusive=True)

    split = request.get("split")
    if not isinstance(split, dict) or type(split.get("enabled")) is not bool:
        raise PlanningError("invalid_split", "split.enabled must be a boolean.", "split.enabled")
    _keys(split, {"enabled", "requested_flights", "max_waypoints_per_flight"}, "split")
    requested_flights = _integer(split.get("requested_flights", 1), "split.requested_flights", minimum=1)
    waypoint_limit = _integer(split.get("max_waypoints_per_flight"), "split.max_waypoints_per_flight",
                              minimum=2, maximum=MAX_WAYPOINT_LIMIT)
    finish_action = request.get("finish_action", "return_to_home")
    if finish_action not in FINISH_ACTIONS:
        raise PlanningError("invalid_finish_action", "Finish action is not supported.", "finish_action")
    turn_style = request.get("turn_style", "curved")
    if turn_style not in ("curved", "straight"):
        raise PlanningError("invalid_turn_style", "Turn style must be curved or straight.", "turn_style")
    raw_locations = request.get("locations")
    if isinstance(raw_locations, dict) and "per_flight" in raw_locations:
        raise PlanningError("unsupported_feature", "Oblique missions use shared launch and home locations.",
                            "locations.per_flight")
    locations = _locations(raw_locations, requested_flights)
    survey_area = request.get("survey_area")
    _validate_area(survey_area)

    direction = request.get("direction")
    if not isinstance(direction, dict) or direction.get("mode") not in ("manual", "automatic"):
        raise PlanningError("invalid_direction", "Direction mode must be manual or automatic.", "direction.mode")
    _keys(direction, {"mode", "convention", "value_deg"}, "direction")
    if direction.get("convention", DIRECTION_CONVENTION) != DIRECTION_CONVENTION:
        raise PlanningError("unsupported_direction_convention", "Direction convention is not supported.",
                            "direction.convention")

    lens = {key: _number(camera.get(key), "drone_profile.camera." + key, minimum=0, exclusive=True)
            for key in ("sensor_width_mm", "sensor_height_mm", "focal_length_mm")}
    if direction["mode"] == "manual":
        resolved_direction = _number(direction.get("value_deg"), "direction.value_deg") % 180
    else:
        nadir_spacing = max(altitude * lens["sensor_width_mm"] / lens["focal_length_mm"] * (1 - side_overlap), .5)
        try:
            resolved_direction = grid.find_optimal_direction(
                survey_area, nadir_spacing, max_scan_steps=MAX_ROUTE_WAYPOINTS)
        except grid.GridCapacityError as exc:
            raise PlanningError("capacity_exceeded", str(exc), "survey_area") from None
        except ValueError as exc:
            raise PlanningError("invalid_geometry", str(exc), "survey_area") from None

    # The camera tilt from straight down, the distance from the drone to the
    # ground at the photo centre, and how far that centre lies ahead of it.
    tilt = math.radians(90 + pitch)
    slant = altitude / math.cos(tilt)
    offset = altitude * math.tan(tilt)

    try:
        passes = _plan_passes(survey_area, resolved_direction, altitude, slant, offset, pitch, include_nadir,
                              lens, camera, side_overlap, front_overlap, speed, interval, margin, mode)
        area = survey_area_m2(survey_area)
    except grid.GridCapacityError as exc:
        raise PlanningError("capacity_exceeded", str(exc), "survey_area") from None
    except (KeyError, TypeError, ValueError) as exc:
        raise PlanningError("invalid_geometry", str(exc), "survey_area") from None
    if sum(len(survey_pass["points"]) for survey_pass in passes) > MAX_OBLIQUE_ROUTE_WAYPOINTS:
        raise PlanningError("capacity_exceeded", "Planned route exceeds the engine waypoint capacity.",
                            "survey_area")

    waypoints, strips, points = _route(passes)
    edges = _edge_distances(points)
    usable_seconds = _number(aircraft.get("battery_safe_min"), "drone_profile.aircraft.battery_safe_min",
                             minimum=0, exclusive=True) * 60
    photo_allowance = max(2.5, interval)
    budget = (speed, mode, photo_allowance, usable_seconds, finish_action, locations)

    blocking = []
    per_pass = []
    for survey_pass in passes:
        boundaries = (_pass_boundaries(survey_pass, points, edges, waypoint_limit, budget)
                      if split["enabled"] else [(survey_pass["start"], survey_pass["end"])])
        if boundaries is None:
            if not any(error["code"] == "flight_cannot_fit" for error in blocking):
                blocking.append(_issue("flight_cannot_fit",
                                       "No valid split fits the waypoint and usable battery-time limits."))
            boundaries = [(survey_pass["start"], survey_pass["end"])]
        per_pass.append(boundaries)
    if split["enabled"] and not blocking:
        per_pass = _add_requested_flights(per_pass, passes, points, edges, waypoint_limit, budget,
                                          requested_flights)

    flights, warnings = [], []
    for boundaries, survey_pass in zip(per_pass, passes):
        for start, end in boundaries:
            index = len(flights)
            flight = _flight_result(index, start, end, points, edges, speed, mode, interval, photo_allowance,
                                    usable_seconds, finish_action, _flight_location(locations, index),
                                    pitch_deg=survey_pass["pitch"])
            flights.append(flight)
            if not flight["estimates_complete"]:
                warnings.append(_issue("travel_estimate_incomplete",
                                       f"Flight {index + 1} is missing launch or recovery location travel."))
    for flight in flights:
        number = flight["index"] + 1
        if flight["waypoint_count"] > waypoint_limit:
            if not split["enabled"]:
                if not any(error["code"] == "splitting_required_waypoint_limit" for error in blocking):
                    blocking.append(_issue("splitting_required_waypoint_limit",
                                           "Enable splitting to satisfy the waypoint limit."))
            elif not any(error["code"] == "flight_cannot_fit" for error in blocking):
                blocking.append(_issue("waypoint_limit_exceeded", f"Flight {number} exceeds the waypoint limit."))
        if flight["known_estimated_seconds"] > usable_seconds:
            if not split["enabled"]:
                if not any(error["code"] == "splitting_required_battery_limit" for error in blocking):
                    blocking.append(_issue("splitting_required_battery_limit",
                                           "Enable splitting to satisfy the usable battery-time limit."))
            elif not any(error["code"] == "flight_cannot_fit" for error in blocking):
                blocking.append(_issue("battery_time_exceeded",
                                       f"Flight {number} exceeds the usable battery-time limit."))

    complete = all(flight["estimates_complete"] for flight in flights)
    result = {
        "contract_version": CONTRACT_VERSION,
        "engine_version": ENGINE_VERSION,
        "profile_version": PROFILE_VERSION,
        "operation": "plan_oblique",
        "mapping_style": "oblique",
        "drone_profile_id": profile_id,
        "drone_profile_name": profile_name,
        "resolved_direction": {"convention": DIRECTION_CONVENTION, "value_deg": resolved_direction},
        "turn_style": turn_style,
        "finish_action": finish_action,
        "oblique": {
            "gimbal_pitch_deg": pitch,
            "include_nadir": include_nadir,
            "slant_distance_m": slant,
            "offset_m": offset,
            "passes": [_pass_record(survey_pass) for survey_pass in passes],
        },
        "route": {"waypoints": waypoints, "strips": strips},
        "flights": flights,
        "statistics": {
            "survey_area_m2": area,
            "route_distance_m": sum(f["route_distance_m"] for f in flights),
            "outbound_distance_m": _complete_sum(f["outbound_distance_m"] for f in flights),
            "recovery_distance_m": _complete_sum(f["recovery_distance_m"] for f in flights),
            "known_distance_m": sum(f["known_distance_m"] for f in flights),
            "total_distance_m": sum(f["total_distance_m"] for f in flights) if complete else None,
            "known_estimated_seconds": sum(f["known_estimated_seconds"] for f in flights),
            "total_estimated_seconds": sum(f["total_estimated_seconds"] for f in flights) if complete else None,
            "photo_count": sum(f["photo_count"] for f in flights),
            "photo_count_kind": "actions" if mode == "full_auto" else "estimate",
            "strip_count": len(strips),
            "waypoint_count": len(points),
            "battery_count": len(flights),
            "estimates_complete": complete,
        },
        "capture": {
            "mode": mode,
            "profile_interval_s": interval,
            # The shortest pass spacing, so the reported figure is never optimistic.
            "shot_spacing_m": min(survey_pass["shot_spacing"] for survey_pass in passes),
        },
        "assumptions": [
            {"code": "photo_centre_at_takeoff_level",
             "message": "Oblique spacing and pass offsets use the slant distance to takeoff ground level."},
            {"code": "straight_segment_estimate",
             "message": "Distance and time use straight waypoint segments for every turn style."},
            {"code": "unmodeled_operations",
             "message": "Takeoff, climb, landing, and additional turn delays are not modeled."},
            {"code": "battery_reserve_separate",
             "message": "The profile usable-time budget is applied without a blanket time multiplier."},
        ],
        "warnings": warnings,
        "validation": {"export_allowed": not blocking, "errors": blocking},
    }
    _ensure_finite_json(result)
    return result


def _plan_passes(survey_area, direction, altitude, slant, offset, pitch, include_nadir, lens, camera,
                 side_overlap, front_overlap, speed, interval, margin, mode):
    """The grid of every pass: nadir first when included, then the four looks."""
    # Flight lines run along this grid bearing (clockwise from grid north); the
    # engine's direction convention is counterclockwise.
    line_bearing = (-direction) % 360
    plans = []
    if include_nadir:
        plans.append({"kind": "nadir", "look": None, "area": survey_area, "pitch": NADIR_PITCH_DEG,
                      "direction": direction, "distance": altitude})
    for index, look in enumerate(LOOKS):
        bearing = (line_bearing + 90 * index) % 360
        plans.append({
            "kind": look, "look": bearing, "pitch": pitch,
            # Lines run along the camera, so the image width spans the lines
            # as in a nadir grid; the right and left looks use the cross grid.
            "direction": direction if index % 2 == 0 else (direction + 90) % 180,
            "area": _shifted_area(survey_area, (bearing + 180) % 360, offset),
            "distance": slant,
        })
    passes = []
    for plan in plans:
        if mode == "full_auto":
            along = plan["distance"] * lens["sensor_height_mm"] / lens["focal_length_mm"]
            shot_spacing = max(along * (1 - front_overlap), .5)
            densify = shot_spacing
        else:
            shot_spacing = max(speed * interval, .5)
            densify = None
        generated = grid.generate_grid(
            plan["area"], altitude_m=plan["distance"], shot_spacing_m=shot_spacing,
            side_overlap=side_overlap, direction_deg=plan["direction"], margin_m=margin,
            camera=camera, densify_spacing=densify, include_metadata=True,
            max_waypoints=MAX_ROUTE_WAYPOINTS,
        )
        heading = None if plan["look"] is None else _true_heading(plan["area"], plan["look"])
        passes.append({
            "kind": plan["kind"], "look": plan["look"], "heading": heading, "pitch": plan["pitch"],
            "direction": plan["direction"], "points": generated["waypoints"], "strips": generated["strips"],
            "line_spacing": generated["line_spacing_m"], "shot_spacing": shot_spacing,
        })
    return passes


def _route(passes):
    """Waypoints and strips of all passes in order, with pass indices and roles."""
    waypoints, strips, points = [], [], []
    for pass_index, survey_pass in enumerate(passes):
        start = len(points)
        roles, pass_strips = _route_metadata(survey_pass["points"], survey_pass["strips"])
        roles[0].discard("route_start")
        roles[-1].discard("route_end")
        roles[0].add("pass_start")
        roles[-1].add("pass_end")
        for offset, point in enumerate(survey_pass["points"]):
            waypoints.append({"index": start + offset, "position": point, "roles": roles[offset],
                              "pass_index": pass_index, "heading_deg": survey_pass["heading"]})
            points.append(point)
        for strip in pass_strips:
            strips.append({**strip, "index": len(strips), "pass_index": pass_index,
                           "start_waypoint_index": strip["start_waypoint_index"] + start,
                           "end_waypoint_index": strip["end_waypoint_index"] + start})
        survey_pass["start"], survey_pass["end"] = start, len(points) - 1
    waypoints[0]["roles"].add("route_start")
    waypoints[-1]["roles"].add("route_end")
    for waypoint in waypoints:
        waypoint["roles"] = sorted(waypoint["roles"])
    return waypoints, strips, points


def _pass_boundaries(survey_pass, points, edges, limit, budget, count=None):
    """The fewest balanced flights (or exactly count) that fit one pass."""
    start, end = survey_pass["start"], survey_pass["end"]
    speed, mode, allowance, usable, finish_action, locations = budget
    available = max(1, end - start)
    first = count or (math.ceil((end - start) / (limit - 1)) if end > start else 1)
    for flights in range(first, (count or available) + 1):
        found = _balanced_boundaries(points[start:end + 1], edges[start:end], flights, limit, speed, mode,
                                     allowance, usable, finish_action, locations)
        if found:
            return [(start + low, start + high) for low, high in found]
    return None


def _add_requested_flights(per_pass, passes, points, edges, limit, budget, requested):
    """Add flights, one at a time to the pass with the longest flight, until the
    route has at least the requested number."""
    speed, mode, allowance, _usable, _finish, _locations = budget
    available = sum(max(1, survey_pass["end"] - survey_pass["start"]) for survey_pass in passes)
    if requested > available:
        raise PlanningError("too_many_requested_flights", "requested_flights exceeds the available route edges.",
                            "split.requested_flights")

    def longest(boundaries):
        return max(sum(edges[start:end]) / speed + ((end - start + 1) * allowance + 3 if mode == "full_auto" else 0)
                   for start, end in boundaries)

    while sum(len(boundaries) for boundaries in per_pass) < requested:
        _seconds, index = max((longest(boundaries), index) for index, boundaries in enumerate(per_pass)
                              if len(boundaries) < max(1, passes[index]["end"] - passes[index]["start"]))
        more = _pass_boundaries(passes[index], points, edges, limit, budget, count=len(per_pass[index]) + 1)
        if more is None:
            break
        per_pass[index] = more
    return per_pass


def _pass_record(survey_pass):
    return {
        "kind": survey_pass["kind"],
        "look_bearing_deg": survey_pass["look"],
        "heading_deg": survey_pass["heading"],
        "gimbal_pitch_deg": survey_pass["pitch"],
        "grid_direction_deg": survey_pass["direction"],
        "start_waypoint_index": survey_pass["start"],
        "end_waypoint_index": survey_pass["end"],
        "waypoint_count": survey_pass["end"] - survey_pass["start"] + 1,
        "line_spacing_m": survey_pass["line_spacing"],
        "shot_spacing_m": survey_pass["shot_spacing"],
    }


def _shifted_area(survey_area, bearing_deg, distance_m):
    """The survey area moved distance_m along a grid bearing."""
    metric, inverse = grid._metric_polygon(survey_area)
    angle = math.radians(bearing_deg)
    moved = affinity.translate(metric, xoff=distance_m * math.sin(angle), yoff=distance_m * math.cos(angle))
    polygons = list(moved.geoms) if moved.geom_type == "MultiPolygon" else [moved]
    parts = [{"exterior": _wgs84_ring(polygon.exterior.coords, inverse),
              "holes": [_wgs84_ring(ring.coords, inverse) for ring in polygon.interiors]}
             for polygon in polygons]
    return parts[0] if len(parts) == 1 else {"parts": parts}


def _wgs84_ring(coordinates, inverse):
    ring = []
    for x, y in list(coordinates)[:-1]:          # shapely repeats the first point
        longitude, latitude = inverse.transform(x, y)
        ring.append({"latitude_deg": latitude, "longitude_deg": (longitude + 180) % 360 - 180})
    return ring


def _true_heading(survey_area, grid_bearing):
    """The geographic heading (clockwise from true north, in DJI's (-180, 180])
    of a grid bearing at the area's centre."""
    metric, inverse = grid._metric_polygon(survey_area)
    centre = metric.centroid
    angle = math.radians(grid_bearing)
    start = inverse.transform(centre.x, centre.y)
    end = inverse.transform(centre.x + 100 * math.sin(angle), centre.y + 100 * math.cos(angle))
    azimuth = _WGS84.inv(start[0], start[1], end[0], end[1])[0]
    azimuth = (azimuth + 180.0) % 360.0 - 180.0
    return 180.0 if azimuth == -180.0 else round(azimuth, 6)
