"""Contract-2 corridor planning from an open centre line."""

import math
from functools import wraps

from pyproj import CRS
from shapely.geometry import LineString
from shapely.ops import transform

from .flight_accounting import mission_statistics
from .measurements import route_distance_m, survey_area_m2
from .planning import (
    ENGINE_VERSION, MAX_ALTITUDE_M, MAX_WAYPOINT_LIMIT,
    PlanningError, _balanced_boundaries_solver, _edge_distances, _ensure_finite_json,
    _flight_location, _flight_result, _integer, _issue, _keys, _locations,
    _number, _optional_bool, _position, _version,
)
from .precision import Projection, canonical_planning, geometry, result_numbers, number
from .profiles import PROFILE_VERSION, drone_profile

MAX_CORRIDOR_FLIGHTS = 40
MAX_CORRIDOR_WAYPOINTS = 2_000


def _contract_2(operation):
    @wraps(operation)
    def wrapped(request):
        with canonical_planning():
            return operation(request)
    return wrapped


@_contract_2
def corridor_sample_points(request):
    """Return deterministic terrain lookup positions, including every semi-auto leg."""
    common = _prepare(request)
    groups, _offsets, _area = _geometry(common)
    points = [point for group in groups for point in _samples(group, common)]
    if len(points) > MAX_CORRIDOR_WAYPOINTS:
        raise PlanningError("capacity_exceeded", "Terrain sampling exceeds engine capacity.", "terrain_follow")
    return result_numbers(points)


@_contract_2
def plan_corridor(request):
    """Plan a corridor; supplied terrain samples are part of the replayable request."""
    common = _prepare(request)
    groups, offsets, area = _geometry(common)
    if len(groups) > MAX_CORRIDOR_FLIGHTS:
        raise PlanningError("capacity_exceeded", "Corridor exceeds 40 flight groups.", "break_indices")
    terrain = common["terrain"]
    samples = [point for group in groups for point in _samples(group, common)] if terrain else []
    if len(samples) > MAX_CORRIDOR_WAYPOINTS:
        raise PlanningError("capacity_exceeded", "Terrain sampling exceeds engine capacity.", "terrain_follow")
    elevations = None
    if terrain:
        elevations = terrain.get("elevations_m")
        if not isinstance(elevations, list) or len(elevations) != len(samples):
            raise PlanningError("terrain_samples_incomplete", "One elevation is required for every corridor sample.", "terrain_follow.elevations_m")
        elevations = [_number(value, f"terrain_follow.elevations_m.{index}") for index, value in enumerate(elevations)]

    route = []
    strips = []
    bounds = []
    sample_index = 0
    for group in groups:
        start = len(route)
        source = _samples(group, common) if terrain else group["points"]
        ground = elevations[sample_index:sample_index + len(source)] if terrain else None
        sample_index += len(source)
        if ground is not None and common["mode"] == "semi_auto":
            bend_positions = {(point["latitude_deg"], point["longitude_deg"]) for point in group["points"]}
            keep = {0, len(source) - 1}
            keep.update(index for index, point in enumerate(source)
                        if (point["latitude_deg"], point["longitude_deg"]) in bend_positions)
            last = 0
            for index in range(1, len(source) - 1):
                if abs(ground[index] - ground[last]) > terrain["tolerance_m"]:
                    keep.add(index)
                    last = index
                elif index in keep:
                    last = index
            indices = sorted(keep)
            source = [source[index] for index in indices]
            ground = [ground[index] for index in indices]
        for index, point in enumerate(source):
            roles = set()
            if index == 0:
                roles.add("corridor_group_start")
            if index == len(source) - 1:
                roles.add("corridor_group_end")
            item = {"index": len(route), "position": point, "roles": sorted(roles)}
            if ground is not None:
                item["ground_elevation_m"] = ground[index]
            route.append(item)
        end = len(route) - 1
        bounds.append((start, end))
        for survey_pass in group["passes"]:
            # Terrain may insert or remove route positions, so pass ranges are
            # located by their original endpoints in the retained sequence.
            first, last = survey_pass
            first_point, last_point = group["points"][first], group["points"][last]
            left = next(i for i in range(start, end + 1) if route[i]["position"] == first_point)
            right = next(i for i in range(left, end + 1) if route[i]["position"] == last_point)
            strips.append({"index": len(strips), "start_waypoint_index": left, "end_waypoint_index": right})
            route[left]["roles"] = sorted(set(route[left]["roles"]) | {"survey_strip_start"})
            route[right]["roles"] = sorted(set(route[right]["roles"]) | {"survey_strip_end"})
            for waypoint_index in range(left + 1, right):
                route[waypoint_index]["roles"] = sorted(set(route[waypoint_index]["roles"]) | {"survey_strip_interior"})
    if len(route) > MAX_CORRIDOR_WAYPOINTS:
        raise PlanningError("capacity_exceeded", "Corridor route exceeds engine waypoint capacity.", "centre_line")
    route[0]["roles"] = sorted(set(route[0]["roles"]) | {"route_start"})
    route[-1]["roles"] = sorted(set(route[-1]["roles"]) | {"route_end"})
    points = [waypoint["position"] for waypoint in route]
    flights, blocking, warnings = [], [], []
    requested = common["requested"]
    if requested > sum(max(1, end - start) for start, end in bounds):
        raise PlanningError("too_many_requested_flights", "Requested flights exceed route edges.", "split.requested_flights")
    per_group, solvers = [], []
    for start, end in bounds:
        local = points[start:end + 1]
        if not common["split_enabled"]:
            chosen = [(0, len(local) - 1)]
            solvers.append(None)
        else:
            solver = _balanced_boundaries_solver(
                local, _edge_distances(local), common["limit"], common["speed"],
                common["mode"], common["allowance"], common["budget"], common["finish"],
                common["locations"] if common["locations"] and common["locations"][0] == "shared" else None)
            solvers.append(solver)
            chosen = _best_split(local, common, 1, solver)
            if chosen is None:
                blocking.append(_issue("flight_cannot_fit", "No valid split fits the waypoint and battery limits."))
                chosen = [(0, len(local) - 1)]
        per_group.append([(start + low, start + high) for low, high in chosen])
    edges = _edge_distances(points)
    if common["split_enabled"]:
        while sum(map(len, per_group)) < requested:
            candidates = []
            for index, ((start, end), current) in enumerate(zip(bounds, per_group)):
                if len(current) < end - start:
                    location = _flight_location(common["locations"], 0)
                    longest = max(_flight_result(0, low, high, points, edges, common["speed"],
                                                 common["mode"], common["interval"], common["allowance"],
                                                 common["budget"], common["finish"], location)["known_estimated_seconds"]
                                  for low, high in current)
                    candidates.append((longest, -index, index))
            if not candidates:
                break
            index = max(candidates)[2]
            start, end = bounds[index]
            local = points[start:end + 1]
            target = requested - sum(map(len, per_group)) + len(per_group[index]) if len(candidates) == 1 else len(per_group[index]) + 1
            found = _best_split(local, common, target, solvers[index], exact=True)
            if found is None:
                blocking.append(_issue("flight_cannot_fit", "Requested flight count cannot fit."))
                break
            per_group[index] = [(start + low, start + high) for low, high in found]
    boundaries = [part for group in per_group for part in group]
    if len(boundaries) > MAX_CORRIDOR_FLIGHTS:
        raise PlanningError("capacity_exceeded", "Corridor exceeds 40 flights.", "split")
    for index, (start, end) in enumerate(boundaries):
        location = _flight_location(common["locations"], index)
        flight = _flight_result(index, start, end, points, edges, common["speed"], common["mode"], common["interval"], common["allowance"], common["budget"], common["finish"], location)
        flights.append(flight)
        if not flight["estimates_complete"]:
            warnings.append(_issue("travel_estimate_incomplete", f"Flight {index + 1} is missing launch or recovery location travel."))
        if flight["waypoint_count"] > common["limit"]:
            blocking.append(_issue("splitting_required_waypoint_limit" if not common["split_enabled"] else "waypoint_limit_exceeded", "Flight exceeds waypoint limit."))
        if flight["known_estimated_seconds"] > common["budget"]:
            blocking.append(_issue("splitting_required_battery_limit" if not common["split_enabled"] else "battery_time_exceeded", "Flight exceeds usable battery time."))
    result = {
        "contract_version": 2, "engine_version": ENGINE_VERSION,
        "profile_version": PROFILE_VERSION, "drone_profile_id": common["profile_id"],
        "drone_profile_name": common["profile_name"], "operation": "plan_corridor",
        "mapping_style": "corridor", "turn_style": common["turn"], "finish_action": common["finish"],
        "corridor": {"width_m": common["width"], "pass_offsets_m": offsets,
                     "break_indices": common["breaks"]},
        "route": {"waypoints": route, "strips": strips}, "flights": flights,
        "statistics": mission_statistics(flights, area, len(strips), len(route)),
        "capture": {"mode": common["mode"], "profile_interval_s": common["interval"],
                    "shot_spacing_m": common["shot_spacing"]},
        "assumptions": [
            {"code": "straight_segment_estimate", "message": "Distance and time use straight waypoint segments for every turn style."},
            {"code": "unmodeled_operations", "message": "Takeoff, climb, landing, and additional turn delays are not modeled."},
            {"code": "battery_reserve_separate", "message": "The profile usable-time budget is applied without a blanket time multiplier."},
        ], "warnings": warnings, "validation": {"export_allowed": not blocking, "errors": blocking},
    }
    _ensure_finite_json(result)
    return result_numbers(result)


def _best_split(points, common, minimum, solve, exact=False):
    maximum = min(MAX_CORRIDOR_FLIGHTS, max(1, len(points) - 1))
    lower = max(minimum, math.ceil((len(points) - 1) / (common["limit"] - 1)))
    if lower > MAX_CORRIDOR_FLIGHTS:
        raise PlanningError("capacity_exceeded", "Corridor requires more than 40 flights.", "split")
    for count in range(lower, (minimum if exact else maximum) + 1):
        found = solve(count)
        if found:
            return found
    return None


def _prepare(request):
    if not isinstance(request, dict):
        raise PlanningError("invalid_request", "Planning request must be an object.")
    _keys(request, {"contract_version", "operation", "mapping_style", "source_mission_id", "centre_line", "width_m", "altitude_m", "speed_m_s", "side_overlap_ratio", "capture", "turn_style", "finish_action", "split", "reverse_route", "break_indices", "locations", "drone_profile_id", "profile_version", "terrain_follow"}, "request")
    _version(request.get("contract_version"), 2, "contract_version")
    if request.get("operation") != "plan_corridor":
        raise PlanningError("unsupported_operation", "Only plan_corridor is supported.", "operation")
    if request.get("mapping_style") != "corridor":
        raise PlanningError("unsupported_feature", "Mapping style must be corridor.", "mapping_style")
    _version(request.get("profile_version"), PROFILE_VERSION, "profile_version")
    try:
        profile_name, profile = drone_profile(request.get("drone_profile_id"))
    except (KeyError, TypeError):
        raise PlanningError("profile_not_found", "Drone profile is not available.", "drone_profile_id") from None
    aircraft, camera = profile["aircraft"], profile["camera"]
    altitude = _number(request.get("altitude_m"), "altitude_m", minimum=0, maximum=MAX_ALTITUDE_M, exclusive=True)
    width = _number(request.get("width_m"), "width_m", minimum=0, maximum=5000, exclusive=True)
    speed = _number(request.get("speed_m_s"), "speed_m_s", minimum=0, exclusive=True)
    if not aircraft["min_speed_ms"] <= speed <= aircraft["max_speed_ms"]:
        raise PlanningError("speed_out_of_profile_range", "Speed is outside drone profile range.", "speed_m_s")
    overlap = _number(request.get("side_overlap_ratio"), "side_overlap_ratio", minimum=0, maximum=1, maximum_exclusive=True)
    capture = request.get("capture")
    if not isinstance(capture, dict) or capture.get("mode") not in ("semi_auto", "full_auto"):
        raise PlanningError("invalid_capture_mode", "Capture mode must be semi_auto or full_auto.", "capture.mode")
    _keys(capture, {"mode", "interval_s", "front_overlap_ratio"}, "capture")
    interval = _number(camera["min_shoot_interval_s"], "drone_profile.camera.min_shoot_interval_s", minimum=0, exclusive=True)
    if "interval_s" in capture and _number(capture["interval_s"], "capture.interval_s", minimum=0, exclusive=True) != interval:
        raise PlanningError("profile_interval_override", "Capture interval is owned by drone profile.", "capture.interval_s")
    footprint = altitude * camera["sensor_width_mm"] / camera["focal_length_mm"]
    shot_spacing = speed * interval
    if capture["mode"] == "full_auto":
        front = _number(capture.get("front_overlap_ratio"), "capture.front_overlap_ratio", minimum=0, maximum=1, maximum_exclusive=True)
        shot_spacing = max(altitude * camera["sensor_height_mm"] / camera["focal_length_mm"] * (1 - front), .5)
    split = request.get("split")
    if not isinstance(split, dict) or type(split.get("enabled")) is not bool:
        raise PlanningError("invalid_split", "split.enabled must be a boolean.", "split.enabled")
    _keys(split, {"enabled", "requested_flights", "max_waypoints_per_flight"}, "split")
    requested = _integer(split.get("requested_flights", 1), "split.requested_flights", minimum=1)
    if requested > MAX_CORRIDOR_FLIGHTS:
        raise PlanningError("capacity_exceeded", "Corridor supports at most 40 flights.", "split.requested_flights")
    limit = _integer(split.get("max_waypoints_per_flight"), "split.max_waypoints_per_flight", minimum=2, maximum=MAX_WAYPOINT_LIMIT)
    if not split["enabled"] and requested != 1:
        raise PlanningError("invalid_split", "Requested flights requires splitting.", "split.requested_flights")
    turn = request.get("turn_style", "curved")
    if turn not in ("curved", "straight"):
        raise PlanningError("invalid_turn_style", "Turn style must be curved or straight.", "turn_style")
    finish = request.get("finish_action", "return_to_home")
    if finish not in ("return_to_home", "return_to_first_waypoint", "hover", "land"):
        raise PlanningError("invalid_finish_action", "Finish action is unsupported.", "finish_action")
    line = request.get("centre_line")
    if not isinstance(line, list) or not 2 <= len(line) <= 500:
        raise PlanningError("invalid_geometry", "Centre line needs 2 to 500 positions.", "centre_line")
    line = [_position(point, f"centre_line.{index}") for index, point in enumerate(line)]
    if len({(point["latitude_deg"], point["longitude_deg"]) for point in line}) != len(line):
        raise PlanningError("invalid_geometry", "Centre line contains repeated positions.", "centre_line")
    breaks = request.get("break_indices", [])
    if not isinstance(breaks, list) or len(breaks) > len(line) - 2:
        raise PlanningError("invalid_break_indices", "Break indices must be an array of interior vertices.", "break_indices")
    breaks = [_integer(value, f"break_indices.{index}", minimum=1, maximum=len(line) - 2) for index, value in enumerate(breaks)]
    if breaks != sorted(set(breaks)):
        raise PlanningError("invalid_break_indices", "Break indices must be unique and ascending.", "break_indices")
    terrain = request.get("terrain_follow")
    if terrain is not None:
        if not isinstance(terrain, dict):
            raise PlanningError("invalid_terrain", "terrain_follow must be an object.", "terrain_follow")
        _keys(terrain, {"tolerance_m", "elevations_m"}, "terrain_follow")
        terrain = {**terrain, "tolerance_m": _number(terrain.get("tolerance_m"), "terrain_follow.tolerance_m", minimum=0, maximum=100, exclusive=True)}
    locations = _locations(request.get("locations"), requested)
    if locations and locations[0] == "per_flight":
        raise PlanningError("unsupported_feature", "Per-flight locations are unsupported for corridors.", "locations.per_flight")
    return {"line": line, "breaks": breaks, "width": width, "altitude": altitude, "speed": speed,
            "spacing": max(footprint * (1 - overlap), .5), "footprint": footprint,
            "mode": capture["mode"], "interval": interval, "shot_spacing": shot_spacing,
            "allowance": max(2.5, interval), "limit": limit, "requested": requested,
            "split_enabled": split["enabled"], "reverse": _optional_bool(request.get("reverse_route", False), "reverse_route"),
            "turn": turn, "finish": finish, "locations": locations,
            "budget": aircraft["battery_safe_min"] * 60, "terrain": terrain,
            "profile_id": request["drone_profile_id"], "profile_name": profile_name}


def _geometry(common):
    from shapely.geometry import Point
    from shapely.ops import substring

    line = common["line"]
    longitudes = [point["longitude_deg"] for point in line]
    if max(longitudes) - min(longitudes) > 3:
        raise PlanningError("invalid_geometry", "Corridor must fit a local projected zone.", "centre_line")
    centre_lon = sum(longitudes) / len(line)
    centre_lat = sum(point["latitude_deg"] for point in line) / len(line)
    if abs(centre_lat) > 80 or max(point["latitude_deg"] for point in line) - min(point["latitude_deg"] for point in line) > 3:
        raise PlanningError("invalid_geometry", "Corridor must fit a local projected zone.", "centre_line")
    zone = min(60, max(1, int((centre_lon + 180) / 6) + 1))
    utm = CRS.from_epsg((32600 if centre_lat >= 0 else 32700) + zone)
    forward, inverse = Projection("EPSG:4326", utm, 3), Projection(utm, "EPSG:4326", 9)
    xy = [forward.transform(point["longitude_deg"], point["latitude_deg"]) for point in line]
    if len(set(xy)) != len(xy):
        raise PlanningError("invalid_geometry", "Centre line contains coincident projected positions.", "centre_line")
    centre = geometry(LineString(xy))
    if centre.is_empty or not centre.is_simple or centre.length < .001 or not all(math.isfinite(v) for v in centre.bounds):
        raise PlanningError("invalid_geometry", "Centre line must be a simple open line.", "centre_line")
    # At a miter, the outside corner is farther along either leg than the
    # straight-edge inset. A photo at the bend must reach that corner along
    # its shorter, along-track footprint dimension.
    camera = drone_profile(common["profile_id"])[1]["camera"]
    footprint_height = (common["footprint"] * camera["sensor_height_mm"]
                        / camera["sensor_width_mm"])
    along_reach = footprint_height / 2
    if common["mode"] == "semi_auto":
        # An interval photo can be half an interval away from a bend.
        along_reach -= common["shot_spacing"] / 2
    inset = common["footprint"] / 2
    for before, bend, after in zip(xy, xy[1:], xy[2:]):
        incoming = (bend[0] - before[0], bend[1] - before[1])
        outgoing = (after[0] - bend[0], after[1] - bend[1])
        first, second = math.hypot(*incoming), math.hypot(*outgoing)
        if min(first, second) < .001:
            raise PlanningError("invalid_geometry", "Centre line has a collapsed segment.", "centre_line")
        cosine = max(-1.0, min(1.0, -sum(a * b for a, b in zip(incoming, outgoing)) / (first * second)))
        if cosine > -1:
            if along_reach <= 0:
                raise PlanningError("invalid_offset", "Capture interval is too long to cover a corridor bend.", "capture")
            inset = min(inset, along_reach * math.sqrt((1 - cosine) / (1 + cosine)))
    extent = max(0.0, common["width"] / 2 - inset)
    count = max(1, math.ceil(2 * extent / common["spacing"])) if extent else 0
    offsets = [(-extent + index * 2 * extent / count) for index in range(count + 1)] if count else [0.0]
    if len(offsets) * len(line) > MAX_CORRIDOR_WAYPOINTS:
        raise PlanningError("capacity_exceeded", "Corridor exceeds engine waypoint capacity.", "width_m")
    groups = []
    total_points = 0
    band = centre.buffer(common["width"] / 2, cap_style=2, join_style=2, mitre_limit=2)
    if band.is_empty or band.area <= 0:
        raise PlanningError("invalid_geometry", "Corridor width is too small for projected precision.", "width_m")
    geographic_band = transform(inverse.transform, band)
    polygons = geographic_band.geoms if geographic_band.geom_type == "MultiPolygon" else [geographic_band]
    area = survey_area_m2({"parts": [{"exterior": [
        {"latitude_deg": y, "longitude_deg": x} for x, y in polygon.exterior.coords],
        "holes": [[{"latitude_deg": y, "longitude_deg": x} for x, y in hole.coords]
                  for hole in polygon.interiors]} for polygon in polygons]})
    cuts = [0, *common["breaks"], len(line) - 1]
    curves = []
    for offset in offsets:
        curve = centre if offset == 0 else centre.offset_curve(offset, join_style=2, mitre_limit=2)
        if curve.geom_type != "LineString" or curve.is_empty or not curve.is_simple or len(curve.coords) < 2:
            raise PlanningError("invalid_offset", "Corridor offset is disconnected or self-intersecting.", "centre_line")
        coords = list(curve.coords)
        for endpoint, neighbour, label in ((xy[0], xy[1], "start"),
                                           (xy[-1], xy[-2], "end")):
            dx, dy = neighbour[0] - endpoint[0], neighbour[1] - endpoint[1]
            length = math.hypot(dx, dy)
            sign = 1 if label == "start" else -1
            expected = (endpoint[0] - sign * offset * dy / length,
                        endpoint[1] + sign * offset * dx / length)
            if min(math.dist(coords[0], expected), math.dist(coords[-1], expected)) > .02:
                raise PlanningError("invalid_offset", f"Corridor offset does not reach its {label}.", "centre_line")
        if math.dist(coords[0], xy[0]) > math.dist(coords[-1], xy[0]):
            coords.reverse()
            curve = LineString(coords)
        distances = [0.0]
        for index in common["breaks"]:
            bend = xy[index]
            incoming = (bend[0] - xy[index - 1][0], bend[1] - xy[index - 1][1])
            outgoing = (xy[index + 1][0] - bend[0], xy[index + 1][1] - bend[1])
            cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            candidate = None
            if abs(cross) > 1e-8:
                first = (bend[0] - offset * incoming[1] / math.hypot(*incoming),
                         bend[1] + offset * incoming[0] / math.hypot(*incoming))
                second = (bend[0] - offset * outgoing[1] / math.hypot(*outgoing),
                          bend[1] + offset * outgoing[0] / math.hypot(*outgoing))
                delta = (second[0] - first[0], second[1] - first[1])
                factor = (delta[0] * outgoing[1] - delta[1] * outgoing[0]) / cross
                candidate = Point(first[0] + factor * incoming[0],
                                  first[1] + factor * incoming[1])
                if curve.distance(candidate) > .02:
                    candidate = None
            if candidate is None:
                candidate = Point(bend)
                nearest = curve.interpolate(curve.project(candidate))
                # A loop can place two equally near pieces on opposite sides
                # of a break. Choosing either one silently changes the group.
                distance = candidate.distance(nearest)
                for left, right in zip(coords, coords[1:]):
                    segment = LineString([left, right])
                    other = segment.interpolate(segment.project(candidate))
                    if abs(candidate.distance(other) - distance) <= .02 and other.distance(nearest) > .02:
                        raise PlanningError("invalid_offset", "Corridor break has an ambiguous offset.", "break_indices")
            position = curve.project(candidate)
            if position <= distances[-1] + .001 or position >= curve.length - .001:
                raise PlanningError("invalid_offset", "Corridor break does not divide its offset in order.", "break_indices")
            distances.append(position)
        distances.append(curve.length)
        pieces = [substring(curve, start, end) for start, end in zip(distances, distances[1:])]
        if any(piece.geom_type != "LineString" or len(piece.coords) < 2 for piece in pieces):
            raise PlanningError("invalid_offset", "Corridor break creates an empty offset.", "break_indices")
        curves.append([list(piece.coords) for piece in pieces])
    for group_index in range(len(cuts) - 1):
        points, passes = [], []
        for pass_index, offset in enumerate(offsets):
            coords = curves[pass_index][group_index][:]
            if pass_index % 2:
                coords.reverse()
            start = len(points)
            for coord_index, coord in enumerate(coords):
                if coord_index and common["mode"] == "full_auto":
                    previous = coords[coord_index - 1]
                    steps = max(1, math.ceil(math.dist(previous, coord) / common["shot_spacing"]))
                    if total_points + len(points) + steps > MAX_CORRIDOR_WAYPOINTS:
                        raise PlanningError("capacity_exceeded", "Corridor exceeds engine waypoint capacity.", "centre_line")
                    for step in range(1, steps):
                        fraction = step / steps
                        lon, lat = inverse.transform(previous[0] + (coord[0] - previous[0]) * fraction,
                                                     previous[1] + (coord[1] - previous[1]) * fraction)
                        points.append({"latitude_deg": lat, "longitude_deg": lon})
                lon, lat = inverse.transform(*coord)
                points.append({"latitude_deg": lat, "longitude_deg": lon})
            passes.append((start, len(points) - 1))
            if total_points + len(points) > MAX_CORRIDOR_WAYPOINTS:
                raise PlanningError("capacity_exceeded", "Corridor exceeds engine waypoint capacity.", "centre_line")
        if common["reverse"]:
            points.reverse()
            size = len(points)
            passes = [(size - 1 - end, size - 1 - start) for start, end in reversed(passes)]
        groups.append({"points": points, "passes": passes})
        total_points += len(points)
    if common["reverse"]:
        groups.reverse()
    return groups, [number(value) for value in (reversed(offsets) if common["reverse"] else offsets)], area


def _samples(group, common):
    points = group["points"]
    if common["mode"] == "full_auto":
        # Photo positions are generated independently below; terrain never
        # changes the number or placement of these positions.
        return points
    if not common["terrain"]:
        return points
    spacing = min(30.0, max(8.0, common["terrain"]["tolerance_m"] * 5))
    result = [points[0]]
    for first, second in zip(points, points[1:]):
        distance = route_distance_m([first, second])
        steps = max(1, math.ceil(distance / spacing))
        if len(result) + steps > MAX_CORRIDOR_WAYPOINTS:
            raise PlanningError("capacity_exceeded", "Terrain sampling exceeds engine capacity.", "terrain_follow")
        for step in range(1, steps):
            fraction = step / steps
            longitude_delta = (second["longitude_deg"] - first["longitude_deg"] + 180) % 360 - 180
            longitude = first["longitude_deg"] + fraction * longitude_delta
            longitude = (longitude + 180) % 360 - 180
            result.append({"latitude_deg": round(first["latitude_deg"] + fraction * (second["latitude_deg"] - first["latitude_deg"]), 9),
                           "longitude_deg": round(longitude, 9)})
        result.append(second)
    return result
