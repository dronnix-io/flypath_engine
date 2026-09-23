# Calculation rules

These rules are implemented by contract-version-1 `plan_2d` in engine release
`v0.4.0`.

## Area and distance

- Measure polygon area and route segments on the WGS84 ellipsoid.
- Report survey area separately from camera coverage.
- Report route distance, recovery distance, and their total separately.
- Add recovery only when the finish action flies it and its destination is
  known. A first waypoint is not silently treated as home.

## Time

Estimated flight time is the sum of travel time, generated photo-stop time,
per-flight startup actions, and named operational allowances. Every allowance
is returned in the calculation assumptions. A blanket multiplier is accepted
only after flight-log validation and must not duplicate explicit delays.

## Photos

- Semi-auto counts captures only over route portions where interval capture is
  enabled. First-trigger, turns, connectors, recovery, and restart behavior must
  be explicit.
- Full-auto photo count equals generated photo actions.
- Photo estimates never increase merely because a travel-time safety allowance
  increased.

The current shared statistics function is deliberately narrower: it calculates
route travel time, full-auto waypoint stop time, route-based photo count, and a
battery estimate. It does not yet model recovery or per-flight startup/restart
allowances. Clients must not treat that partial result as the final contract.

`plan_2d` calculates every flight separately. Full-auto time adds the greater
of 2.5 seconds or the profile capture interval for each photo action, plus one
3-second startup hover per flight. Semi-auto photo estimates floor each
flight's route travel time divided by the profile interval and then sum the
per-flight counts. Outbound and recovery travel never increase semi-auto photo
counts.

Splits are contiguous and share one seam waypoint. The partition minimizes the
largest known estimated flight time for a candidate flight count; the engine
increases that count until waypoint and usable profile battery-time limits
pass. Missing travel is excluded from known time and makes complete
time/distance totals unknown. It does not by itself block export.

## Initial acceptance tolerances

- Direction interpretation: 0.01 degree at the engine boundary.
- Geographic strip bearing: 0.05 degree against an independent reference.
- Route waypoint position: 0.20 metre against an approved reference route.
- Distance: greater of 0.05 metre or one part per million.
- Area: greater of 1 square metre or one part per million.

Both products consume the same shared-core output, so product-to-product values must
be identical before display formatting.
