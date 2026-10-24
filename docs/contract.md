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

## Orbit request

Engine release `v1.2.0` adds `plan_orbit(request)` for one ring around a centre
point, for oblique capture of a 3D model. It is an additive operation under
contract version 1. The request contains:

- `operation: "plan_orbit"` and `mapping_style: "orbit"`
- contract and drone-profile versions, and `drone_profile_id`
- `centre` position, `radius_m` (5 to 2,000), and `altitude_m`
- `gimbal_pitch_deg` (-90 to 0) and `direction` (`clockwise` or
  `counterclockwise`); optional `start_bearing_deg`, default 0 (north)
- `speed_m_s` within the profile range, and `finish_action`
- `capture.mode` `semi_auto` or `full_auto`; full-auto also requires
  `side_overlap_ratio` between neighbouring photos
- `split` with `enabled: false` and a waypoint limit (`v1.3.0` adds
  splitting, see below)

Terrain following fails as `unsupported_feature`. Unknown fields
fail as with `plan_2d`.

## Orbit result

The result uses the `plan_2d` shape (route, flights, statistics, capture,
validation, assumptions, warnings) with these differences:

- `operation`, `mapping_style: "orbit"`, `turn_style: "curved"`, and an `orbit`
  block echoing the centre, radius, altitude, pitch, direction, start bearing,
  slant distance, and angular step
- every route waypoint carries `heading_deg` in (-180, 180], facing the centre
- one strip and one flight; statistics report the circle area as
  `survey_area_m2`

Full-auto spaces waypoints by the side overlap of the image width at the slant
distance, with at least 12 waypoints and a photo action at each one, plus the
first camera rotation and 3-second hover. Semi-auto places a waypoint about
every 15 metres of the ring (12 to 72) and contains only the camera rotation.
The ring stops short of the start by at most one photo spacing, so the first
and last waypoints are separate. A ring above the waypoint limit or the usable
battery time returns `validation.export_allowed: false`.

## Multi-ring orbits

Engine release `v1.3.0` adds an optional `rings` list to the `plan_orbit`
request, still under contract version 1. It replaces the single `radius_m`,
`altitude_m`, and `gimbal_pitch_deg` fields (sending both fails as
`invalid_rings`) and holds 1 to 3 rings, each with its own `radius_m`,
`altitude_m`, and `gimbal_pitch_deg`. All rings share the centre, direction,
start bearing, speed, capture settings, finish action, and waypoint limit.

The rings are flown in order, as one flight unless the request enables
splitting. Each ring starts at the start bearing; the route goes straight from the last waypoint of a ring to the first
waypoint of the next, and that transition counts its climb or descent in the
distance and time. The result differs from the single-ring shape as follows:

- the `orbit` block holds the centre, direction, start bearing, and a `rings`
  list with each ring's radius, altitude, pitch, slant distance, angular step,
  waypoint range, waypoint count, shot spacing, and side overlap
- every route waypoint also carries its `altitude_m` and `ring_index`; ring
  boundaries carry the `ring_start` and `ring_end` roles
- one strip per ring, and `survey_area_m2` uses the largest ring
- each ring's first waypoint has its own camera rotation; full-auto adds the
  3-second hover there and a photo at every waypoint
- `capture.side_overlap_ratio` reports the lowest ring overlap and full-auto
  `capture.shot_spacing_m` the shortest ring spacing; semi-auto interval photos
  are estimated over the whole flight, transitions included
- the waypoint and battery limits apply to each flight

A request without `rings` and without splitting keeps the `v1.2.0` result
exactly, so saved single-ring orbits still verify.

### Splitting orbits

`v1.3.0` also accepts `split.enabled: true` for `plan_orbit`, with the
optional `split.requested_flights` (default 1) as in `plan_2d`. A split request
uses the multi-ring result shape, even for one ring. Flights are chosen as
follows:

- whole rings are grouped into the fewest flights, at least
  `requested_flights`, that fit the waypoint limit and the usable battery
  time; among those, the grouping with the shortest longest flight wins
- a flight of whole rings starts at a ring's first waypoint, so it shares no
  waypoint with the flight before it
- when a ring does not fit one flight, or more flights are requested than
  there are rings, every ring flies on its own and is cut into the fewest
  balanced parts that fit; parts of one ring share their seam waypoint, as
  `plan_2d` flights do
- every flight turns the camera to its ring's pitch at its first waypoint;
  full-auto adds the 3-second hover there and a photo at every waypoint, so a
  shared seam has a photo in both flights

Without splitting, a multi-ring route that exceeds the waypoint limit or one
battery returns `splitting_required_waypoint_limit` or
`splitting_required_battery_limit`. A split that cannot fit returns
`flight_cannot_fit`, and more requested flights than route edges fail as
`too_many_requested_flights`.

## Oblique request

Engine release `v1.4.0` adds `plan_oblique(request)` for oblique photography
of an area, for 3D models. It is an additive operation under contract version
1. The request is the `plan_2d` request with these differences:

- `operation: "plan_oblique"` and `mapping_style: "oblique"`
- `gimbal_pitch_deg` (-85 to -20), the camera pitch of the oblique passes
- optional `include_nadir`, default `true`, to fly a nadir pass first
- optional `pattern`, `full` (default) or `fast`
- no `cross_hatch` or `reverse_route`; per-flight `locations` and terrain
  following fail as `unsupported_feature`

## Oblique result

The result uses the `plan_2d` shape with these differences:

- `operation`, `mapping_style: "oblique"`, and an `oblique` block with the
  pitch, `include_nadir`, the slant distance to the photo centre
  (`altitude / sin(|pitch|)`), the pass offset (`altitude / tan(|pitch|)`),
  and a `passes` list
- the passes are flown in order: the nadir pass (`kind` `nadir`) when
  included, then the oblique passes of the pattern. Each records its kind,
  look bearing (grid, clockwise from grid north, or `null`), true
  `heading_deg` (or `null`), pitch, grid direction, waypoint range, line
  spacing, and shot spacing
- `full` flies four oblique passes whose camera faces along the flight lines,
  to their right, back along them, and to their left (`kind` `forward`,
  `right`, `back`, `left`). Each is the grid of the survey area moved against
  its look bearing by the pass offset, with lines along the look bearing; the
  drone holds the pass heading
- `fast` flies two crossing grids (`kind` `grid`, `cross_grid`) with the
  drone facing along its lines, so neighbouring lines photograph opposite
  sides. Each is the grid of the survey area swept by the pass offset both
  ways along its lines
- the nadir pass is the `plan_2d` grid; oblique passes space lines and photos
  from the slant distance instead of the altitude
- every route waypoint carries `pass_index` and `heading_deg`: the held pass
  heading of a `full` pass, or `null` (face along the path) on the nadir and
  `fast` passes; pass boundaries carry the `pass_start` and `pass_end` roles,
  and strips carry `pass_index`
- a flight never spans two passes. Without splitting, every pass is one
  flight; with splitting, every pass is cut into the fewest balanced flights
  that fit, and more are added to the passes with the longest flights until
  there are at least `requested_flights`. A pass's first flight starts at its
  `pass_start` waypoint; flights inside a pass share their seam, as in
  `plan_2d`
- every flight turns the camera to its pass pitch at its first waypoint;
  full-auto adds the 3-second hover there and a photo at every waypoint
- `capture.shot_spacing_m` reports the shortest pass spacing, and
  `survey_area_m2` the drawn area

Without splitting, a pass above the waypoint limit or one battery returns
`splitting_required_waypoint_limit` or `splitting_required_battery_limit`.

## Ownership

The shared Python core owns coordinate normalization, direction, 2D, oblique and orbit route generation, ordering,
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
