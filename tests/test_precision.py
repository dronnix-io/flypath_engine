"""Exact contract-2 replay fingerprints, shared by every CI operating system."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from flypath_engine import plan_2d, plan_oblique, plan_orbit, plan_panorama, PlanningError


def requests():
    orbit = {
        'contract_version': 2, 'profile_version': 1, 'operation': 'plan_orbit',
        'mapping_style': 'orbit', 'drone_profile_id': 'mini4pro',
        'centre': {'latitude_deg': 51., 'longitude_deg': -114.},
        'radius_m': 30., 'altitude_m': 30., 'gimbal_pitch_deg': -35.,
        'direction': 'clockwise', 'speed_m_s': 3., 'capture': {'mode': 'semi_auto'},
        'finish_action': 'return_to_home',
        'split': {'enabled': False, 'max_waypoints_per_flight': 200},
    }
    oblique = {
        'contract_version': 2, 'profile_version': 1, 'operation': 'plan_oblique',
        'mapping_style': 'oblique', 'drone_profile_id': 'mini4pro',
        'altitude_m': 60., 'speed_m_s': 5., 'side_overlap_ratio': .7, 'margin_m': 0.,
        'direction': {'mode': 'manual', 'value_deg': 0.},
        'capture': {'mode': 'semi_auto'}, 'finish_action': 'return_to_home',
        'turn_style': 'curved', 'gimbal_pitch_deg': -45., 'include_nadir': True,
        'split': {'enabled': False, 'requested_flights': 1, 'max_waypoints_per_flight': 200},
    }
    rng = random.Random(27)
    for index in range(80):
        lat, lon = rng.uniform(-65, 65), rng.uniform(-170, 170)
        r = deepcopy(orbit)
        r.update(centre={'latitude_deg': lat, 'longitude_deg': lon}, radius_m=rng.uniform(10, 150))
        yield f'orbit-{index}', r
        r = deepcopy(oblique)
        r['survey_area'] = {'exterior': [
            {'latitude_deg': lat + dy, 'longitude_deg': lon + dx}
            for dx, dy in ((0, 0), (.002, 0), (.002, .001), (0, .001))], 'holes': []}
        yield f'oblique-{index}', r
        if index < 8:
            for pattern in ('full', 'fast'):
                full = deepcopy(r)
                full.update(pattern=pattern, reverse_route=bool(index % 2))
                full['capture'] = {'mode': 'full_auto', 'front_overlap_ratio': .6}
                full['split'] = {'enabled': True, 'requested_flights': 5, 'max_waypoints_per_flight': 30}
                yield f'oblique-full-{pattern}-{index}', full
            flat = deepcopy(r)
            flat.update(operation='plan_2d', mapping_style='2d')
            for key in ('gimbal_pitch_deg', 'include_nadir'):
                flat.pop(key)
            flat['direction'] = {'mode': 'automatic'}
            yield f'2d-auto-{index}', flat
    # Straddle ring waypoint-count thresholds by one floating-point step.
    for count in (12, 13, 30, 72):
        radius = count * 15 / (2 * math.pi)
        for side, value in enumerate((math.nextafter(radius, 0), radius, math.nextafter(radius, math.inf))):
            for limit in (12, 13, 200):
                r = deepcopy(orbit)
                r['radius_m'] = value
                r['split'] = {'enabled': True, 'requested_flights': 2, 'max_waypoints_per_flight': limit}
                yield f'orbit-count-{count}-{side}-{limit}', r
    for mode in ('semi_auto', 'full_auto'):
        r = deepcopy(orbit)
        for key in ('radius_m', 'altitude_m', 'gimbal_pitch_deg'):
            r.pop(key)
        r['rings'] = [dict(radius_m=30., altitude_m=30., gimbal_pitch_deg=-35.),
                      dict(radius_m=45., altitude_m=60., gimbal_pitch_deg=-45.)]
        r['capture'] = {'mode': mode, **({'side_overlap_ratio': .8} if mode == 'full_auto' else {})}
        r['split'] = {'enabled': True, 'requested_flights': 3, 'max_waypoints_per_flight': 30}
        yield 'rings-' + mode, r
    rng = random.Random(29)
    for index in range(12):
        latitude, longitude = rng.uniform(-65, 65), rng.uniform(-170, 170)
        yield f'panorama-{index}', {
            'contract_version': 2, 'profile_version': 1, 'operation': 'plan_panorama',
            'mapping_style': 'panorama', 'drone_profile_id': ('mini4pro', 'air3', 'mavic4pro')[index % 3],
            'points': [{'latitude_deg': latitude + .0003 * step, 'longitude_deg': longitude}
                       for step in range(1 + index % 3)],
            'altitude_m': rng.uniform(20, 120), 'speed_m_s': 2., 'include_nadir': bool(index % 4),
            'split': {'max_waypoints_per_flight': (200, 40)[index % 2]},
            'capture': {'mode': 'full_auto', 'overlap_ratio': rng.uniform(.2, .7)},
            'finish_action': 'return_to_home',
        }
    rng = random.Random(31)
    for index in range(8):
        latitude, longitude = rng.uniform(-65, 65), rng.uniform(-170, 170)
        yield f'panorama-area-{index}', {
            'contract_version': 2, 'profile_version': 1, 'operation': 'plan_panorama',
            'mapping_style': 'panorama', 'drone_profile_id': 'mini4pro',
            'survey_area': {'exterior': [
                {'latitude_deg': latitude + dy, 'longitude_deg': longitude + dx}
                for dx, dy in ((0, 0), (.006, 0), (.006, .004), (0, .004))], 'holes': []},
            **({'point_spacing_m': rng.uniform(120, 300)} if index % 2 else {}),
            'altitude_m': rng.uniform(40, 90), 'speed_m_s': 8.,
            'capture': {'mode': 'full_auto', 'overlap_ratio': .3},
            'finish_action': 'return_to_home',
        }


PLANNERS = {'plan_2d': plan_2d, 'plan_orbit': plan_orbit, 'plan_oblique': plan_oblique,
            'plan_panorama': plan_panorama}


def fingerprint(result):
    return hashlib.sha256(json.dumps(result, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def test_platform_fingerprints():
    expected = json.loads((ROOT / 'tests/fixtures/contract-2-fingerprints.json').read_text())
    for name, request in requests():
        saved = deepcopy(request)
        result = PLANNERS[request['operation']](request)
        assert request == saved, name
        assert result['contract_version'] == 2, name
        assert fingerprint(result) == expected[name], name


def test_legacy_arithmetic_is_restored_after_success_and_failure():
    request = next(requests())[1]
    legacy = {**request, 'contract_version': 1}
    before = plan_orbit(legacy)
    plan_orbit(request)
    try:
        plan_orbit({**request, 'radius_m': -1})
    except PlanningError:
        pass
    else:
        raise AssertionError('Invalid radius accepted')
    assert plan_orbit(legacy) == before


if __name__ == '__main__':
    test_platform_fingerprints()
    test_legacy_arithmetic_is_restored_after_success_and_failure()
    print('Exact platform fingerprints and legacy isolation passed')
