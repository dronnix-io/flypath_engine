# Shared planning engine contract

This contract defines the boundary used by the FlyPath website and QGIS plugin.
It describes values and behavior independently of Python, JavaScript,
QGIS, map rendering, persistence, or KMZ serialization.

Engine release `v0.4.0` implements contract version 1 through
`plan_2d(request)`. Both consumers adapt this result. See
[integration status](integration-status.md) for verification and remaining
public limits.

## Versioning

Every request contains `contract_version`. Every result contains
`contract_version`, `engine_version`, and `profile_version`.

- Additive optional fields may keep the contract version.
- Changed meaning, units, defaults, ordering, or required fields increments it.
- Mission sync rejects unsupported versions and asks the user to update.
- Updating software never regenerates a saved route. Regeneration is explicit.

## Coordinates and numbers

- Positions use objects: `{ "latitude_deg": 51, "longitude_deg": -114 }`.
- Latitude is -90 through 90; longitude is -180 through 180.
- Arrays never rely on ambiguous `[x, y]` coordinate ordering.
- Distance uses metres, time seconds, angles degrees, and area square metres.
- Direction uses the plugin's established convention: counterclockwise from
  local grid North, normalized to `[0, 180)` for bidirectional survey lines.
  Zero is north-south; 90 is east-west. Requests and results name this
  convention explicitly; geographic strip bearings remain separate outputs.
- Inputs and outputs are finite JSON numbers. NaN and infinity are rejected.
- Identical versioned inputs produce deterministically ordered outputs.

## First-phase request

The target `plan_2d` request contains:

- contract and drone-profile versions
- survey exterior ring and optional interior rings
- altitude, speed, side overlap, and margin
- manual direction with an explicit convention, or automatic direction mode
- semi-auto interval or full-auto capture behavior
- turn style and finish action
- split enablement, requested flight count, and waypoint limit

Contract version 1 uses `drone_profile_id`, `profile_version: 1`, and direction
convention `plugin_grid_ccw_from_north`. Capture mode is `semi_auto` or
`full_auto`; full-auto also requires `front_overlap_ratio`. The profile owns
the capture interval. A legacy `capture.interval_s` is accepted only when it
exactly matches the profile value.

Optional `locations` contains either `shared: {launch, home}` or a
`per_flight` array whose length equals `split.requested_flights`. If automatic
splitting would change a route with per-flight locations, export is blocked
until those associations are reviewed.

`cross_hatch` and `reverse_route` preserve current 2D workflows. Terrain and
corridor planning fail explicitly as unsupported. Unknown fields fail so
misspelled planning input cannot be ignored.

Compatibility limits are 500 metres altitude, 1,000 metres margin, 200
waypoints per flight, and 7,961 generated route waypoints. Requests above
engine capacity fail with `capacity_exceeded`; values are never clamped.

Corridor planning and terrain following are outside the first phase.

## First-phase result

The target result contains:

- resolved direction and ordered survey strips
- ordered route waypoints with explicit roles
- flights referencing contiguous route portions
- photo actions attached to route positions
- route, recovery, and total estimated flight distance
- survey area, flight time, photo count, strip count, waypoint count, and
  battery count
- calculation assumptions
- warnings with stable codes and readable messages

Each flight references inclusive route waypoint indices. Adjacent flights
share their seam index. Actions are ordered per flight. Semi-auto contains a
first-waypoint camera rotation but no hover or photo commands. Full-auto
contains camera rotation, the first 3-second hover, and photo actions. A seam
therefore has a photo in both adjacent flights. Missing launch or required
home travel sets complete totals to `null` while retaining named known totals.

## Ownership

The shared Python core owns coordinate normalization, direction, 2D route generation, ordering,
splitting, photo actions, geographic measurements, estimates, and planning
warnings.

The website and plugin own UI state, map drawing, persistence, sync,
authentication, and KMZ serialization. KMZ writers serialize core decisions;
they do not recalculate routes, splits, photo actions, or estimates.

## Errors

Invalid requests raise `PlanningError`; `as_dict()` contains a stable code,
field path where applicable, and display message. Flyable plans that cannot be
exported because splitting, waypoint, battery, or location-association rules
are unmet return `validation.export_allowed: false` with structured errors.
Invalid geometry, unsupported versions, missing profiles, and invalid values
fail explicitly. A partial flyable route is never returned as success.
