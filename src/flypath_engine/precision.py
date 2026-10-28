"""Contract 2 calculation precision; contract 1 retains its original arithmetic."""

from contextvars import ContextVar
from functools import wraps

from pyproj import Geod, Transformer
from shapely import set_precision


CANONICAL_CONTRACT_VERSION = 2
_canonical = ContextVar('canonical_planning', default=False)


def number(value, places=6):
    return round(value, places) if _canonical.get() else value


def geometry(value):
    return set_precision(value, .001) if _canonical.get() else value


def result_numbers(value, key=''):
    if isinstance(value, dict):
        return {name: result_numbers(item, name) for name, item in value.items()}
    if isinstance(value, list):
        return [result_numbers(item, key) for item in value]
    if type(value) is float:
        places = 9 if key in ('latitude_deg', 'longitude_deg') else 6
        return round(value, places)
    return value


def versioned(planner):
    @wraps(planner)
    def plan(request, **kwargs):
        canonical = isinstance(request, dict) and type(request.get('contract_version')) is int and request['contract_version'] == 2
        if canonical and kwargs.get('engine_version') not in (None, '2.0.0'):
            from .planning import PlanningError
            raise PlanningError('unsupported_version', 'Contract 2 requires engine 2.0.0.', 'engine_version')
        token = _canonical.set(canonical)
        try:
            result = planner({**request, 'contract_version': 1} if canonical else request, **kwargs)
            if canonical:
                result = result_numbers(result)
                result['contract_version'] = 2
            return result
        finally:
            _canonical.reset(token)
    return plan


class Geodesic:
    """Quantize native geodesic output before distance/count/split decisions."""

    def __init__(self):
        self._geod = Geod(ellps='WGS84')

    def fwd(self, *args):
        lon, lat, bearing = self._geod.fwd(*args)
        return number(lon, 9), number(lat, 9), number(bearing, 6)

    def inv(self, *args):
        first, second, distance = self._geod.inv(*args)
        return number(first, 6), number(second, 6), number(distance, 4)

    def polygon_area_perimeter(self, *args):
        area, perimeter = self._geod.polygon_area_perimeter(*args)
        return number(area, 3), number(perimeter, 4)


class Projection:
    def __init__(self, source, destination, places):
        self._transformer = Transformer.from_crs(source, destination, always_xy=True)
        self._places = places

    def transform(self, x, y):
        a, b = self._transformer.transform(x, y)
        if not _canonical.get():
            return a, b
        if isinstance(a, (list, tuple)):
            return [number(v, self._places) for v in a], [number(v, self._places) for v in b]
        return number(a, self._places), number(b, self._places)
