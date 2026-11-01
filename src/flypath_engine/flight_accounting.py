"""Internal flight records and mission totals for the planning operations."""

def flight_record(index, start, end, actions, route_distance, outbound, recovery,
                  photo_count, capture_seconds, startup_seconds, speed, budget, mode):
    """Account for one inclusive route portion after its actions and travel are chosen."""
    complete = outbound is not None and recovery is not None
    known_distance = route_distance + (outbound or 0) + (recovery or 0)
    known_seconds = known_distance / speed + capture_seconds + startup_seconds
    return {
        "index": index,
        "route_start_waypoint_index": start,
        "route_end_waypoint_index": end,
        "waypoint_count": end - start + 1,
        "actions": actions,
        "route_distance_m": route_distance,
        "outbound_distance_m": outbound,
        "recovery_distance_m": recovery,
        "known_distance_m": known_distance,
        "total_distance_m": known_distance if complete else None,
        "capture_allowance_seconds": capture_seconds,
        "startup_wait_seconds": startup_seconds,
        "known_estimated_seconds": known_seconds,
        "total_estimated_seconds": known_seconds if complete else None,
        "usable_battery_seconds": budget,
        "estimates_complete": complete,
        "photo_count": photo_count,
        "photo_count_kind": "actions" if mode == "full_auto" else "estimate",
    }


def _complete_sum(values, add=sum):
    values = list(values)
    return add(values) if all(value is not None for value in values) else None


def mission_statistics(flights, survey_area, strip_count, waypoint_count, add=sum):
    """Aggregate flight records; panorama passes math.fsum for stable sums."""
    complete = all(flight["estimates_complete"] for flight in flights)
    return {
        "survey_area_m2": survey_area,
        "route_distance_m": add(flight["route_distance_m"] for flight in flights),
        "outbound_distance_m": _complete_sum((flight["outbound_distance_m"] for flight in flights), add),
        "recovery_distance_m": _complete_sum((flight["recovery_distance_m"] for flight in flights), add),
        "known_distance_m": add(flight["known_distance_m"] for flight in flights),
        "total_distance_m": add(flight["total_distance_m"] for flight in flights) if complete else None,
        "known_estimated_seconds": add(flight["known_estimated_seconds"] for flight in flights),
        "total_estimated_seconds": add(flight["total_estimated_seconds"] for flight in flights) if complete else None,
        "photo_count": sum(flight["photo_count"] for flight in flights),
        "photo_count_kind": flights[0]["photo_count_kind"],
        "strip_count": strip_count,
        "waypoint_count": waypoint_count,
        "battery_count": len(flights),
        "estimates_complete": complete,
    }
