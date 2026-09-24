# FlyPath Engine

Shared Python planning engine used by the FlyPath Django website and QGIS
plugin. The package contains no Django, QGIS, UI, persistence, or KMZ code.

Version `1.0.1` retains the versioned `plan_2d(request)` contract for 2D routes,
time-balanced flights, explicit capture actions, complete/known estimates,
warnings, and export validation. The lower-level geometry, measurement,
statistics, split-helper, and profile interfaces remain available.

The website and plugin adapters currently consume `v1.0.0`; their `v1.0.1`
updates are pending. The plugin vendors the pinned release source. See
[integration status](docs/integration-status.md) for tested behavior and
remaining public limits.

The package is not published to PyPI. Consumers install the released Git tag
or pin its commit; CI-built wheel and source archives are validation artifacts.

Run the checks with a Python environment containing Shapely and pyproj:

```powershell
python tests/test_route.py
python tests/test_grid.py
python tests/test_measurements.py
python tests/test_statistics.py
python tests/test_profiles.py
python tests/test_planning.py
```

With the website virtual environment, compare both local adapters and their
consumer KMZ output without accessing the application database:

```powershell
python tools/check_consumer_parity.py --website ../flypath.io --plugin ../FlyPath
```

CI runs on Linux x64, Windows x64, and macOS ARM with Python 3.10 and
3.13 coverage. It also builds and installs the wheel and source distribution,
uploads those artifacts, and scans Git history for secrets. macOS Intel remains
part of the QGIS installation matrix but is not covered by the standard hosted
runner matrix.
