# Shared planning integration status

Release `v0.4.0`, contract version 1, adds the versioned `plan_2d`
boundary for 2D routes, time-balanced flight splitting, explicit capture
actions, profile-owned limits, known and complete estimates, warnings, and
structured validation.

The `v1.2.0` candidate adds `plan_orbit` under contract version 1. Both
consumer adapters use its ring, centre-facing headings, capture actions,
overlap, and statistics. Candidate integration adds orbit mission persistence
and synchronization, with one centre point plus radius and camera tilt.

Release `v1.3.0` adds up to three rings per orbit, each with its own radius,
altitude, and camera tilt, and orbit splitting into flights, whole rings
first. A request without rings and without splitting keeps the `v1.2.0`
result exactly. The QGIS plugin plans and exports multi-ring and split orbits;
mission synchronization still carries one ring flown as one flight, so the
website side of rings and splitting remains to be added. The plugin build
requires the public v1.3.0 tag at its pinned commit before release.

Release `v1.4.0` adds `plan_oblique` under contract version 1: an optional
nadir pass and four oblique passes over a survey area, each oblique pass
facing one side with a user-set camera pitch and moved out from the area by
the pass offset. The QGIS plugin plans and exports oblique missions as one
KMZ per flight; mission synchronization does not carry them yet, so the
website side remains to be added. The plugin build requires the public
v1.4.0 tag at its pinned commit before release.

## Public integration

- The QGIS plugin vendors the pinned engine source and remains offline-capable.
- The web adapter and plugin consume the same planning request and result.
- Released engine profiles are the planning authority for both consumers.
- Saved routes remain unchanged until explicit regeneration.
- Preview, statistics, splitting, and supported exports consume one planning
  result per generated route.
- Terrain and corridor planning remain consumer-owned legacy paths.

## Verification

- All engine test scripts pass across the supported Python matrix.
- `tools/check_consumer_parity.py` compares eight semi/full-auto,
  manual/automatic, and cross-hatch combinations, plus four semi/full-auto
  and clockwise/counterclockwise orbit cases across both adapters.
- Split-flight KMZ coordinates and camera actions match within the documented
  serialization tolerance; orbit exports also compare waypoint headings.
- The packaged plugin was smoke-tested offline with QGIS 3.44.14 on Windows.

These checks do not replace plugin-manager installation tests, broader platform
coverage, controller validation, or flight testing on supported aircraft.

## Remaining public limits

1. Validate plugin installation across the supported QGIS/Python/platform
   matrix and test exported missions on supported controllers and aircraft.
2. Enterprise native mapping export remains blocked where it cannot represent
   the complete shared action contract.
3. Mission sync rejects polygon holes and multipart areas until the public sync
   contract can represent them losslessly.
4. Launch/home coordinates are supported by the engine contract; picker UI,
   terrain/corridor migration, and controller delivery remain later phases.
