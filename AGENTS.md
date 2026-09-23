# FlyPath planning engine

Visibility: **PUBLIC**. Every committed change and document must be safe for public disclosure.

## Scope

This Python 3.10+ package is the shared, framework-independent planning core used by the QGIS plugin and Django website. It depends on Shapely and pyproj and must remain free of QGIS, Django, browser, persistence, and KMZ concerns.

Read `docs/contract.md` before changing request/result fields or semantics, `docs/calculation-rules.md` before changing calculations, and `docs/integration-status.md` only when coordinating a release or consumer integration.

Important areas:

- `src/flypath_engine/`: planning, geometry, measurements, statistics, and profiles.
- `src/flypath_engine/profiles/drones.json`: versioned drone profiles.
- `tests/`: directly runnable test scripts and fixtures.
- `tools/check_consumer_parity.py`: local plugin/website adapter parity check.
- `docs/`: public contract, calculation rules, and release/integration notes.

## Commands

```powershell
python -m pip install .
python tests/test_route.py
python tests/test_grid.py
python tests/test_measurements.py
python tests/test_statistics.py
python tests/test_profiles.py
python tests/test_planning.py
```

With a website environment containing both consumers' dependencies, run:

```powershell
python tools/check_consumer_parity.py --website ../flypath.io --plugin ../FlyPath
```

No separate run, build, lint, or format command is documented.

## Conventions and safety

Keep inputs/results JSON-compatible, deterministic, explicitly versioned, and named with units. Additive optional fields may retain a contract version; changed meaning, units, defaults, ordering, or required fields require a version increment. Invalid or unsupported input must fail explicitly; never return a partial route as success. Updating software must not regenerate saved routes implicitly.

The engine owns planning decisions. Consumers own UI, persistence, authentication, sync, and KMZ serialization. Coordinate contract/profile changes with both consumers and report compatibility impact.

Normally leave `build/`, `dist/`, `*.egg-info/`, `.venv/`, `__pycache__/`, and generated wheels alone. Do not include private website code, internal documentation, infrastructure/deployment details, credentials, private URLs, production data, or environment-specific configuration.

