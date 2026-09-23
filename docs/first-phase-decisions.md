# First-phase shared planning: agreed decisions

Recorded: 2026-09-16

State: engine contract and consumer adapters implemented for `v0.4.0`;
broader platform validation remains pending.

## Problem Statement

FlyPath's website and QGIS plugin share engine v0.3.0, but still independently
decide capture placement, flight splitting, and parts of their estimates.
Consumer-owned aircraft values can also differ from released engine profiles.
Users need equivalent planning results for the same versioned inputs.

## Solution

Complete one shared Python planning engine for 2D missions without terrain.
Both products consume its planning decisions for preview, statistics, and export.
The web adapter calls the engine in its application process; the plugin calls
its bundled copy directly and remains offline-capable.

This spec records the decisions agreed so far. Unanswered behavior questions
remain explicit rather than inheriting either product's behavior silently.

## User Stories

1. As a planner, I want the same inputs and versions to produce equivalent
   results in both products, so that switching products does not change my mission.
2. As a planner, I want one engine-owned drone profile catalogue, so that camera
   and aircraft capabilities cannot drift between products.
3. As a mission owner, I want the generating profile and engine versions retained,
   so that my saved plan has traceable provenance.
4. As a product maintainer, I want one released aircraft and camera catalogue,
   so that consumer-local metadata cannot change planning rules.
5. As a pilot, I want flight splits balanced by estimated time, so that flights
   with different line lengths have more comparable workloads.
6. As a pilot, I want every flight to respect waypoint limits, so that balancing
   does not produce an unexecutable flight.
7. As a planner, I want split seams to preserve survey coverage, so that splitting
   cannot leave gaps in my survey.
8. As a pilot, I want to supply one launch/home location for all flights, so that
   estimates can account for operating from a fixed location.
9. As a pilot, I want to supply separate launch/home locations per flight, so that
   the engine can support relocating between flights.
10. As a pilot, I want outbound and applicable return travel included when their
    locations are known, so that estimates represent more than the survey route.
11. As a planner, I want missing travel estimates identified as incomplete, so
    that an unknown home location is not silently replaced by a survey waypoint.
12. As a planner, I want the plugin's existing full-auto placement method used
    by both products, so that photo locations and counts agree.
13. As a pilot, I want preview and export to consume the same generated photo
    actions, so that displayed photo totals describe the exported plan.
14. As a mission owner, I want saved routes preserved until explicit regeneration,
    so that profile or software updates do not silently replace my route.

## Implementation Decisions

- **Profile ownership — agreed:** released, versioned engine profiles are the
  single source of truth. Remove duplicated consumer-owned planning values as
  consumers migrate. Preserve mission references and profile-version
  provenance. Do not retain website planning overrides as a second authority.
  This does not mean deleting mission settings such as chosen altitude or speed.
- **Splitting — agreed:** balance estimated flight time, rather than line or
  waypoint counts alone. Preserve coverage and enforce waypoint limits. The
  exact balancing algorithm remains implementation work within the agreed
  timing model and limits.
- **Waypoint limits and split enablement — agreed:** follow the website's
  explicit split-enabled behavior. If splitting is disabled and one flight
  exceeds the waypoint limit, block export and explain that splitting is
  required. If splitting is enabled, increase the flight count as needed to
  enforce the limit. Never silently export an over-limit flight.
  The plugin currently has a minimum-flight count, not a disable control;
  setting that count to one does not disable automatic splitting. Record a
  plugin UI integration requirement: add an explicit "Enable splitting"
  checkbox (agreed). Off requests one flight and blocks export if its waypoint
  limit is exceeded; on uses the requested minimum and adds flights as needed.
  Default splitting to on for new missions in both products (agreed).
- **Existing split settings — agreed:** preserve reliable explicit website
  choices. Retain enabled splitting for identifiable legacy plugin missions,
  whose saved false value may only mean a minimum count of one. If origin or
  intent is ambiguous, require an explicit choice before regeneration; do not
  guess or automatically change saved routes.
- **Battery-time limits — agreed:** when splitting is enabled, add flights as
  needed so each estimated flight fits the engine profile's usable-time budget.
  When splitting is disabled and the flight exceeds that budget, or no valid
  split can fit, block export and explain why. Respect waypoint limits and the
  requested minimum flight count as well. Missing launch/home travel remains
  explicitly incomplete; passing the known-time check is not a claim that
  unknown travel fits the budget.
- **Unsupported saved planning versions — agreed:** preserve the saved route
  and allow viewing, but block unsupported regeneration/export with a message
  explaining the required update. Never silently substitute another profile
  or regenerate the route. The exact tested compatibility table and stable
  error codes remain implementation work.
- **Launch/home — agreed:** support both a shared location and locations per
  flight in the engine contract. Calculate outbound travel to the first waypoint
  and recovery travel when the finish action requires it and the destination is
  supplied. Never silently assume the first waypoint is home.
- **Missing locations — agreed:** expose that outbound/recovery estimates are incomplete
  when required locations are absent. Do not report an unknown contribution as
  a known zero or label route-only estimates as complete mission totals.
  Continue allowing export when a location is absent, subject to all other
  export checks. Enforce the battery-time limit against known estimated time
  while clearly stating that the complete flight budget cannot be verified.
- **UI sequencing — agreed:** defer launch/home map-picker UI in both products.
  Contract support comes first. Existing elevation-reference settings do not
  supply a geographic launch/home location.
- **Curved turns — agreed:** keep the current straight-segment basis for
  calculated distance and time; do not generate a modeled curved aircraft path
  in this phase. Preserve the selected turn style for display/export and expose
  the estimation assumption. Existing differences in exporter flags still need
  compatibility validation; this decision does not endorse inconsistent flags.
- **Full-auto placement — agreed:** reuse the plugin's existing engine method.
  Derive spacing from camera footprint and front overlap; place points along
  survey lines in the engine's metric coordinate system. Replace the website's
  independent spherical-distance and latitude/longitude interpolation method.
- **Capture ownership — agreed:** the engine returns explicit full-auto photo
  actions. Preview, export, and photo totals consume those actions; photo count
  equals the generated action count.
- **Full-auto split capture — agreed:** preserve current behavior at a shared
  split waypoint: the ending flight takes a photo there and the next flight
  takes a fresh photo when it starts there. Keep both actions and include both
  in per-flight and mission photo totals; do not deduplicate photos merely
  because their coordinates match.
- **Full-auto first-photo sequence — agreed:** use the website's explicit
  sequence at the first waypoint of every flight, including split restarts:
  rotate the camera downward, hover for 3 seconds, then take the first photo.
  The engine owns this ordered action sequence; both exporters serialize it.
  Include the 3-second wait exactly once per flight in estimated time, without
  counting it again as another startup allowance. This replaces the plugin's
  timed-rotation-only approach. Aircraft execution and camera timing constraints
  still require validation.
- **Full-auto per-photo estimate — agreed:** retain the current allowance of
  the greater of 2.5 seconds or the profile's minimum capture interval per
  generated photo action. This is an estimation allowance, not an instruction
  to export an additional hover at every photo. The agreed 3-second first-photo
  hover remains separate and is counted once per flight.
- **No blanket time multiplier — agreed:** do not apply a general percentage
  increase to travel time in this phase. Count explicit travel, photo
  allowances, and the agreed startup wait; keep the profile's battery reserve
  separate. Do not carry consumer-specific blanket multipliers into
  shared estimates.
- **Initial timing scope — agreed:** include known travel, the full-auto
  per-photo allowance, and the 3-second full-auto startup wait. Explicitly
  disclose that takeoff/climb, landing, and additional turn delays are not
  modeled in this phase. Add calibrated allowances later rather than inventing
  values or implying that the current estimate includes those operations.
- **Route start — agreed:** preserve the shared engine's existing corner-start
  behavior and make tie-breaking deterministic. Do not select a different start
  based on launch/home position. A future nearest-launch start option is a
  separate change.
- **Semi-auto capture — agreed:** the pilot manually starts interval photography
  for each flight and leaves it running through survey lines, turns, and
  connectors. Keep consumer semi-auto exports free of individual photo-trigger
  commands. Report photo counts as estimates, not guaranteed actions, because
  the engine does not know the actual manual start/stop time or first-trigger
  phase. Keep the current photo-estimate scope: count the survey route,
  including turns and connectors; exclude outbound and recovery travel even
  when those travel estimates are available. No separate outbound/recovery
  photo totals are requested. Additional flight-time safety allowances must not
  themselves increase estimated photo counts.
- **Semi-auto rounding — agreed:** estimate photos from each flight's survey
  route travel time divided by the profile interval, round down per flight,
  then sum those counts for the mission total. Do not round a combined duration
  instead. For example, two flight estimates of 10.7 each produce displayed
  counts of 10 and 10 and a mission total of 20. Actual first-trigger timing
  remains unknown because capture is manually controlled.
- **Semi-auto interval — agreed:** use the released engine profile's capture
  interval consistently in both products for photo estimates, spacing, and
  overlap calculations. Replace the plugin's separate hidden 2-second value
  and website-owned interval values as part of integration. Neither product
  currently exposes an editable mission interval; do not add that UI or a
  mission-level interval override in this phase. The pilot must configure the
  actual camera to match; this planning value does not start or configure
  interval capture on the aircraft. Adjustable mission intervals are deferred.
- **Architecture — existing agreement:** one versioned planning result owns
  routes, flights, actions, statistics, assumptions, warnings, and errors.
  Adapters retain UI, persistence, transfer, and serialization responsibilities.
  The HTTP planning endpoint remains website-only.

## Testing Decisions

The implementation authorization adopted the engine's public versioned
planning request/result as the primary acceptance boundary. Thin Django and
QGIS adapter parity checks remain part of consumer integration.

- Test observable outputs and invariants, not private helper structure.
- Reuse existing route, grid, measurement, statistics, profile, and direction
  fixtures as prior art; add reviewed expected results at the planning boundary.
- Cover uneven survey-line lengths, time-balanced splits, waypoint limits,
  coverage at seams, and deterministic results.
- Cover an over-limit route with splitting disabled (export blocked with an
  explanation) and enabled (flight count increased to satisfy the limit).
- Cover new-mission split defaults in both adapters, per-flight battery-time
  limits, increased flight counts, and an impossible-to-fit flight. Verify
  unknown launch/home travel remains identified as incomplete.
- Cover legacy website/plugin split choices and ambiguous origin requiring a
  choice before regeneration, without changing stored routes.
- Verify missing home/launch coordinates alone do not block export, incomplete
  estimates remain explicit, and a known over-budget flight still blocks it.
- Verify unsupported saved versions remain viewable without route mutation
  while unsupported regeneration/export is blocked with an update message.
- Verify existing starting-corner behavior and stable tie-breaking; adding a
  launch/home location must not silently choose a different route start.
- Cover shared and per-flight launch/home locations, missing locations, return
  finish actions, and finish actions that require no recovery travel.
- Verify turn-style selection is preserved without changing the straight-segment
  distance model, and that the result states this estimation assumption.
- Check full-auto placement against the plugin reference method and verify that
  totals equal generated actions. At a shared split waypoint, assert that each
  flight has its own capture action and that the mission total includes both.
- Verify each full-auto flight starts with camera rotation, a 3-second hover,
  and its first photo in that order. Assert that estimates include the wait
  once per flight and that both exporters preserve the sequence.
- Verify the per-photo allowance uses the greater of 2.5 seconds and the
  profile capture interval, separately from the once-per-flight startup wait,
  with no blanket travel-time multiplier or extra exported per-photo hover.
- Verify assumptions disclose unmodeled takeoff/climb, landing, and additional
  turn delays rather than presenting those contributions as known zeros.
- Check semi-auto counts are labeled estimates, include active capture through
  turns/connectors, exclude outbound/recovery travel, and do not imply exported
  individual photo-trigger commands.
- Verify per-flight semi-auto rounding and mission totals, including two
  fractional estimates whose rounded sum differs from rounding their total.
- Verify semi-auto photo estimates, spacing, and overlap use the same resolved
  engine-profile interval in both adapters, including a profile whose interval
  differs from the plugin's former hidden 2-second value.
- Compare both adapters with identical resolved profile versions and inputs.
- Verify saved routes remain unchanged until explicit regeneration and that
  preview, statistics, and export consume the same current planning result.
- Check profile migration preserves mission references and does not regenerate
  stored routes merely because profile ownership changes.

## Out of Scope

- Launch/home location-picker UI in this step.
- Editable per-mission photo intervals and their UI.
- Terrain and corridor migration into the engine.
- Controller delivery and broad dialog/browser UI rewrites.

## Further Notes

The discussed product-behavior choices are recorded above. Remaining design
and verification work includes:

- Implementing deterministic time balancing within the agreed limits.
- Validating exported action execution and camera timing constraints.
- Defining the exact tested contract compatibility table and stable error codes.
- Defining how per-flight locations remain associated when recalculation changes
  flight boundaries or flight count; do not silently attach a location to the
  wrong flight. Escalate a meaningful behavior conflict if implementation
  cannot resolve it within the agreed contract.
- Completing adapter, saved-route, offline QGIS, and real-aircraft validation.

Implementation must not interpret remaining technical details as approval to
preserve all current plugin or website behavior.
