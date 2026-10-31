"""Versioned panorama planning: photos all around one or more points, for stitched panoramas.

DJI Fly missions cannot turn the drone in place, so at each point the drone
flies very small laps, facing outward, and takes one photo at each heading.
Each camera row is one lap at its own gimbal pitch, a little wider than the lap
before so no two waypoints share a position. A straight-down photo over the
point ends that point's panorama. A few metres of movement does not show in a
panorama of the surroundings.

Rows run from the horizon (0 degrees) down towards straight down, evenly
spaced so neighbouring rows overlap by the requested ratio. Photos in a row are
spaced so neighbours overlap by the same ratio at the row's top edge, where the
photos are narrowest in heading. Every waypoint holds still for a moment
before its photo; each row also tilts the camera at its first waypoint.

Instead of points, a request can give a survey area. Its panorama points are
placed on a hexagon grid (as Spexi's hexagons), one row at a time, the rows
flown back and forth. The default spacing is the altitude.

Points are flown in the order given, straight from one panorama to the next.
When the route needs more than one flight, it is cut between points, never in
the middle of a panorama: each flight takes as many whole panoramas as fit the
waypoint limit and one battery.
"""

import math

from shapely.geometry import Point

from . import grid
from .measurements import _WGS84, route_distance_m, survey_area_m2
from .orbit import FINISH_ACTIONS, _flight_record, _recovery, _signed_degrees
from .planning import (
    CONTRACT_VERSION, ENGINE_VERSION, MAX_ALTITUDE_M, MAX_WAYPOINT_LIMIT, PlanningError,
    _ensure_finite_json, _integer, _issue, _keys, _number, _optional_bool,
    _position, _validate_area, _version,
)
from .precision import number, versioned
from .profiles import PROFILE_VERSION, drone_profile


MAX_POINTS = 50
MIN_OVERLAP = 0.2
MAX_OVERLAP = 0.7
MIN_ROW_PHOTOS = 3
FIRST_LAP_RADIUS_M = 3.0         # the first row's lap around the point
LAP_GAP_M = 1.0                  # each later row's lap is this much wider
LAP_CLEARANCE_M = 5.0            # the least gap between the outer laps of two points
AUTO_SPACING_PER_ALTITUDE = 1.0  # area points: spacing as a multiple of the altitude
MAX_POINT_SPACING_M = 5_000
SHOT_SETTLE_S = 1.0              # hold still before each photo
ROW_SETTLE_S = 2.0               # longer where the camera tilts to a new row


@versioned
def plan_panorama(request):
    """Return one deterministic, JSON-compatible panorama planning result."""
    if not isinstance(request, dict):
        raise PlanningError("invalid_request", "Planning request must be an object.")
    _keys(request, {
        "contract_version", "operation", "source_mission_id", "mapping_style",
        "drone_profile_id", "profile_version", "points", "survey_area", "point_spacing_m",
        "altitude_m", "speed_m_s", "capture", "include_nadir", "finish_action", "split",
        "turn_style", "terrain_follow",
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

    if ("points" in request) == ("survey_area" in request):
        raise PlanningError("invalid_points", "Give either points or a survey_area.", "points")
    altitude = _number(request.get("altitude_m"), "altitude_m", minimum=0, maximum=MAX_ALTITUDE_M, exclusive=True)
    survey_area = request.get("survey_area")
    point_spacing = None
    if survey_area is None:
        if "point_spacing_m" in request:
            raise PlanningError("unsupported_feature", "point_spacing_m needs a survey_area.", "point_spacing_m")
        points = request.get("points")
        if not isinstance(points, list) or not 1 <= len(points) <= MAX_POINTS:
            raise PlanningError("invalid_points", "A panorama mission has 1 to %d points." % MAX_POINTS, "points")
        points = [_position(point, "points[%d]" % index) for index, point in enumerate(points)]
    else:
        _validate_area(survey_area)
        point_spacing = (AUTO_SPACING_PER_ALTITUDE * altitude if request.get("point_spacing_m") is None
                         else _number(request["point_spacing_m"], "point_spacing_m",
                                      minimum=0, exclusive=True, maximum=MAX_POINT_SPACING_M))
    speed = _number(request.get("speed_m_s"), "speed_m_s", minimum=0, exclusive=True)
    if not aircraft["min_speed_ms"] <= speed <= aircraft["max_speed_ms"]:
        raise PlanningError("speed_out_of_profile_range", "Speed is outside the drone profile range.", "speed_m_s")
    include_nadir = _optional_bool(request.get("include_nadir", True), "include_nadir")
    finish_action = request.get("finish_action", "return_to_home")
    if finish_action not in FINISH_ACTIONS:
        raise PlanningError("invalid_finish_action", "Finish action is not supported.", "finish_action")
    turn_style = request.get("turn_style", "curved")
    if turn_style not in ("curved", "straight"):
        raise PlanningError("invalid_turn_style", "Turn style must be curved or straight.", "turn_style")
    split = request.get("split", {"max_waypoints_per_flight": MAX_WAYPOINT_LIMIT})
    if not isinstance(split, dict):
        raise PlanningError("invalid_split", "split must be an object.", "split")
    _keys(split, {"max_waypoints_per_flight"}, "split")
    waypoint_limit = _integer(split.get("max_waypoints_per_flight"), "split.max_waypoints_per_flight",
                              minimum=2, maximum=MAX_WAYPOINT_LIMIT)

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
    outer = max(row["radius_m"] for row in rows)
    spacing = 2 * outer + LAP_CLEARANCE_M
    if survey_area is not None:
        if point_spacing < spacing:
            raise PlanningError("points_too_close",
                                "Point spacing must be at least %g m, so the laps around the points do not "
                                "overlap." % spacing, "point_spacing_m")
        try:
            points = _area_points(survey_area, point_spacing)
            area_m2 = survey_area_m2(survey_area)
        except (KeyError, TypeError, ValueError) as exc:
            raise PlanningError("invalid_geometry", str(exc), "survey_area") from None
        if len(points) > MAX_POINTS:
            raise PlanningError("too_many_points",
                                "This area needs %d panorama points at %g m apart, more than %d; raise the "
                                "point spacing or the altitude." % (len(points), point_spacing, MAX_POINTS),
                                "point_spacing_m")
    for index in range(1, len(points)):
        for other in range(index):
            if route_distance_m([points[other], points[index]]) < spacing:
                raise PlanningError("points_too_close",
                                    "Points %d and %d are closer than %g m, so their laps would overlap."
                                    % (other + 1, index + 1, spacing), "points[%d]" % index)

    waypoints, actions, edges, point_records = [], [], [], []
    for point_index, centre in enumerate(points):
        point_start = len(waypoints)
        row_records = []
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
                if waypoints:
                    edges.append(route_distance_m([waypoints[-1]["position"], position]))
                waypoints.append({"index": index, "position": position, "roles": roles,
                                  "heading_deg": _signed_degrees(bearing), "point_index": point_index})
            row_records.append({
                "index": row_index, "gimbal_pitch_deg": row["gimbal_pitch_deg"],
                "photo_count": row["photo_count"], "radius_m": row["radius_m"],
                "start_waypoint_index": start, "end_waypoint_index": len(waypoints) - 1,
            })
        waypoints[point_start]["roles"].add("point_start")
        waypoints[-1]["roles"].add("point_end")
        point_records.append({"index": point_index, "centre": centre, "start_waypoint_index": point_start,
                              "end_waypoint_index": len(waypoints) - 1, "rows": row_records})
    waypoints[0]["roles"].add("route_start")
    waypoints[-1]["roles"].add("route_end")
    for waypoint in waypoints:
        waypoint["roles"] = sorted(waypoint["roles"])

    usable_seconds = _number(aircraft.get("battery_safe_min"), "drone_profile.aircraft.battery_safe_min",
                             minimum=0, exclusive=True) * 60
    common = {"speed": speed, "mode": "full_auto", "usable_seconds": usable_seconds}
    route = {"waypoints": waypoints, "actions": actions, "edges": edges, "interval": interval,
             "finish_action": finish_action, "common": common}

    def flight(index, first, last):
        start = point_records[first]["start_waypoint_index"]
        end = point_records[last]["end_waypoint_index"]
        return _panorama_flight(index, start, end, route)

    # Whole panoramas per flight, as many as fit the waypoint limit and one battery.
    groups, first = [], 0
    for last in range(1, len(points)):
        candidate = flight(0, first, last)
        if (candidate["waypoint_count"] > waypoint_limit
                or candidate["known_estimated_seconds"] > usable_seconds):
            groups.append((first, last - 1))
            first = last
    groups.append((first, len(points) - 1))
    flights = [flight(index, first, last) for index, (first, last) in enumerate(groups)]

    blocking = []
    per_point = len(waypoints) // len(points)
    if per_point > waypoint_limit:
        blocking.append(_issue("waypoint_limit_exceeded",
                               "One panorama needs %d waypoints, more than the limit of %d; lower the overlap "
                               "or raise the waypoint limit." % (per_point, waypoint_limit)))
    if any(item["known_estimated_seconds"] > usable_seconds for item in flights):
        blocking.append(_issue("battery_time_exceeded", "A panorama exceeds the usable battery-time limit."))

    # A strip is one row of one point's panorama.
    strips = [{"point_index": record["index"], "start_waypoint_index": row["start_waypoint_index"],
               "end_waypoint_index": row["end_waypoint_index"]}
              for record in point_records for row in record["rows"]]
    for index, strip in enumerate(strips):
        strip["index"] = index
    first_row = rows[0]
    result = {
        "contract_version": CONTRACT_VERSION,
        "engine_version": ENGINE_VERSION,
        "profile_version": PROFILE_VERSION,
        "operation": "plan_panorama",
        "mapping_style": "panorama",
        "drone_profile_id": profile_id,
        "drone_profile_name": profile_name,
        "turn_style": turn_style,
        "finish_action": finish_action,
        "panorama": {
            "altitude_m": altitude,
            "overlap_ratio": overlap,
            "include_nadir": include_nadir,
            "horizontal_fov_deg": horizontal_fov,
            "vertical_fov_deg": vertical_fov,
            "min_point_spacing_m": spacing,
            "layout": "points" if survey_area is None else "area",
            "point_spacing_m": point_spacing,
            "points": point_records,
        },
        "route": {"waypoints": waypoints, "strips": strips},
        "flights": flights,
        "statistics": {
            "survey_area_m2": 0.0 if survey_area is None else area_m2,
            "route_distance_m": math.fsum(item["route_distance_m"] for item in flights),
            "outbound_distance_m": None,
            "recovery_distance_m": _exact_complete_sum(item["recovery_distance_m"] for item in flights),
            "known_distance_m": math.fsum(item["known_distance_m"] for item in flights),
            "total_distance_m": None,
            "known_estimated_seconds": math.fsum(item["known_estimated_seconds"] for item in flights),
            "total_estimated_seconds": None,
            "photo_count": len(waypoints),
            "photo_count_kind": "actions",
            "strip_count": len(point_records) * len(rows),
            "waypoint_count": len(waypoints),
            "battery_count": len(flights),
            "estimates_complete": False,
        },
        "capture": {
            "mode": "full_auto",
            "profile_interval_s": interval,
            # The spacing between photos along the first (horizon) lap.
            "shot_spacing_m": 2 * math.pi * first_row["radius_m"] / first_row["photo_count"],
            "side_overlap_ratio": overlap,
        },
        "assumptions": [
            {"code": "small_lap_parallax", "message": "Laps of a few metres around a point are treated as one viewpoint."},
            {"code": "straight_segment_estimate", "message": "Distance and time use straight segments between waypoints."},
            {"code": "unmodeled_operations", "message": "Takeoff, climb, landing, and turning delays are not modeled."},
        ],
        "warnings": [_issue("travel_estimate_incomplete",
                            "Flight %d is missing launch or recovery location travel." % (item["index"] + 1))
                     for item in flights],
        "validation": {"export_allowed": not blocking, "errors": blocking},
    }
    _ensure_finite_json(result)
    return result


def _area_points(survey_area, spacing):
    """Panorama points on a hexagon grid inside the area, in flight order.

    The grid is centred on the area's centroid, in the area's UTM zone. Rows run
    west to east, each half a spacing along from the last, and are flown back
    and forth from south to north. An area too small to hold a grid point gets
    one point inside it."""
    metric, inverse = grid._metric_polygon(survey_area)
    centre = metric.centroid
    centre_x, centre_y = number(centre.x, 3), number(centre.y, 3)
    min_x, min_y, max_x, max_y = metric.bounds
    row_step = spacing * math.sqrt(3) / 2
    rows = []
    for row in range(math.floor((min_y - centre_y) / row_step), math.ceil((max_y - centre_y) / row_step) + 1):
        y = number(centre_y + row * row_step, 3)
        shift = spacing / 2 if row % 2 else 0.0
        columns = range(math.floor((min_x - centre_x - shift) / spacing),
                        math.ceil((max_x - centre_x - shift) / spacing) + 1)
        found = [(number(centre_x + shift + column * spacing, 3), y) for column in columns]
        found = [point for point in found if metric.covers(Point(point))]
        if found:
            rows.append(found if len(rows) % 2 == 0 else found[::-1])
    metric_points = [point for row in rows for point in row]
    if not metric_points:
        inside = metric.representative_point()
        metric_points = [(number(inside.x, 3), number(inside.y, 3))]
    positions = []
    for x, y in metric_points:
        longitude, latitude = inverse.transform(x, y)
        positions.append({"latitude_deg": latitude, "longitude_deg": longitude})
    return positions


def _exact_complete_sum(values):
    """The exact sum of the values, or None when any is unknown."""
    values = list(values)
    return None if any(value is None for value in values) else math.fsum(values)


def _panorama_flight(index, start, end, route):
    """One flight over route waypoints start to end: whole panoramas."""
    waypoints = route["waypoints"]
    actions = [action for action in route["actions"] if start <= action["waypoint_index"] <= end]
    photos = end - start + 1
    hover_seconds = math.fsum(action["duration_s"] for action in actions if action["type"] == "hover")
    recovery = _recovery(waypoints[start]["position"], waypoints[end]["position"], route["finish_action"])
    # math.fsum, not sum: Python 3.12 made sum() compensate for rounding, so
    # plain sums differ in the last digits between Python versions.
    return _flight_record(index, start, end, actions, math.fsum(route["edges"][start:end]), recovery,
                          photos, photos * max(2.5, route["interval"]), hover_seconds, route["common"])


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
