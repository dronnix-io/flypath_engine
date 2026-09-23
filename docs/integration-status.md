# Shared planning integration status

Release `v0.4.0`, contract version 1, adds the versioned `plan_2d`
boundary for 2D routes, time-balanced flight splitting, explicit capture
actions, profile-owned limits, known and complete estimates, warnings, and
structured validation.

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
  manual/automatic, and cross-hatch combinations across both adapters.
- Split-flight KMZ coordinates and camera actions match within the documented
  serialization tolerance.
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
