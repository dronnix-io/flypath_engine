"""Compare local website/plugin adapters without opening either application's DB.

Run with the website virtual environment after installing this engine candidate.
"""

import argparse
import importlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
import xml.etree.ElementTree as ET
import zipfile


def compare_exports(website_kmz, plugin_kmz):
    """Compare flown decisions, ignoring XML dialect/formatting metadata."""
    for member in ('wpmz/template.kml', 'wpmz/waylines.wpml'):
        with zipfile.ZipFile(io.BytesIO(website_kmz)) as archive:
            website = ET.fromstring(archive.read(member))
        with zipfile.ZipFile(io.BytesIO(plugin_kmz)) as archive:
            plugin = ET.fromstring(archive.read(member))
        a, b = website.findall('.//{*}Placemark'), plugin.findall('.//{*}Placemark')
        assert len(a) == len(b)
        for left, right in zip(a, b):
            lc = left.find('.//{*}coordinates').text.strip().split(',')[:2]
            rc = right.find('.//{*}coordinates').text.strip().split(',')[:2]
            assert all(math.isclose(float(x), float(y), abs_tol=1e-8, rel_tol=0)
                       for x, y in zip(lc, rc))
            for tag in ('actionActuatorFunc', 'waypointTurnMode', 'useStraightLine',
                        'waypointHeadingMode', 'waypointHeadingAngleEnable'):
                assert [e.text for e in left.findall('.//{*}' + tag)] == [
                    e.text for e in right.findall('.//{*}' + tag)], tag
            for tag in ('hoverTime', 'gimbalPitchRotateAngle', 'waypointSpeed', 'executeHeight', 'height',
                        'waypointHeadingAngle'):
                assert [float(e.text) for e in left.findall('.//{*}' + tag)] == [
                    float(e.text) for e in right.findall('.//{*}' + tag)], tag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--website', type=Path, required=True)
    parser.add_argument('--plugin', type=Path, required=True)
    args = parser.parse_args()
    website, plugin = args.website.resolve(), args.plugin.resolve()
    sys.path[:0] = [str(website), str(plugin.parent)]
    os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'config.settings')
    import django
    django.setup()
    from missions.models import DroneConfig, PlannerPolicy
    from missions.shared_planning import plan_2d as website_plan, validate_saved_plan
    from missions.wpml import build_kmz_bundle
    adapter = importlib.import_module(f'{plugin.name}.planning_adapter')
    writers = importlib.import_module(f'{plugin.name}.wpml')
    registry = importlib.import_module(f'{plugin.name}.hardware').registry
    fixture = Path(__file__).resolve().parents[1] / 'tests/fixtures/reported-mission-126-input.json'
    area = json.loads(fixture.read_text())['survey_area']
    drone = DroneConfig(code='mini3pro',
                        is_available=True, wpml_verified=True)
    count = 0
    # Only catalogue lookup/policy storage are stubbed. Both real adapters,
    # installed/vendored engines, geometry and all planning decisions run.
    with patch.object(DroneConfig.objects, 'filter') as lookup, patch.object(
            PlannerPolicy, 'current', return_value=PlannerPolicy()):
        lookup.return_value.first.return_value = drone
        for full in (False, True):
            for automatic in (False, True):
                for cross_hatch in (False, True):
                    settings = {
                        'altitude': 80, 'speed': 8, 'side_overlap': 70,
                        'margin': 0, 'direction': 70, 'auto_direction': automatic,
                        'capture_mode': 'full' if full else 'semi',
                        'front_overlap': 75, 'finish_action': 'goHome',
                        'flight_path': 'curved', 'split_enabled': True,
                        'split_count': 1, 'split_max_wp': 70,
                        'cross_hatch': cross_hatch, 'reverse_route': False,
                        'terrain_follow': False, 'mapping_style': '2d',
                    }
                    payload = {
                        'drone_model': 'mini3pro', 'settings': settings,
                        'polygon': [[p['latitude_deg'], p['longitude_deg']]
                                    for p in area['exterior']],
                    }
                    request = adapter.build_request(
                        survey_area=area, drone_profile_id='mini3pro',
                        altitude_m=80, speed_m_s=8, side_overlap_ratio=.7,
                        margin_m=0, automatic_direction=automatic,
                        direction_deg=70, capture_mode=settings['capture_mode'],
                        front_overlap_ratio=.75, turn_style='curved',
                        finish_action='Return to Home', split_enabled=True,
                        requested_flights=1, max_waypoints_per_flight=70,
                        cross_hatch=cross_hatch,
                    )
                    website_result = website_plan(payload)
                    plugin_result = adapter.plan(request)
                    assert website_result['planning_result'] == plugin_result, (full, automatic, cross_hatch)
                    validate_saved_plan(request, plugin_result, website_result['waypoints'], payload)
                    export_payload = dict(payload, waypoints=website_result['waypoints'],
                                          planning_request=request, planning_result=plugin_result)
                    content, _, multiple = build_kmz_bundle(export_payload)
                    if multiple:
                        with zipfile.ZipFile(io.BytesIO(content)) as archive:
                            website_flights = [archive.read(name) for name in archive.namelist()]
                    else:
                        website_flights = [content]
                    flights = adapter.consume_result(request, plugin_result)
                    assert len(website_flights) == len(flights)
                    with tempfile.TemporaryDirectory(prefix='flypath-parity-') as directory:
                        assert Path(directory).resolve().parent == Path(tempfile.gettempdir()).resolve()
                        for index, (flight, website_kmz) in enumerate(zip(flights, website_flights)):
                            path = Path(directory) / f'{index}.kmz'
                            spec = writers.MissionSpec(
                                waypoints=flight['waypoints'], altitude_m=80, speed_ms=8,
                                finish_action='Return to Home', rc_lost_action='Return to Home',
                                capture_mode=settings['capture_mode'], actions=flight['actions'])
                            writers.write_mission(registry.get('DJI Mini 3 Pro'), spec, str(path))
                            compare_exports(website_kmz, path.read_bytes())
                    count += 1
        # Orbit adapters must preserve the same engine route, overlap, headings,
        # and actions; the centre is a singleton [latitude, longitude] in sync.
        for full in (False, True):
            for reverse in (False, True):
                altitude, pitch = (30, -35) if full else (10, 0)
                settings = {
                    'mapping_style': 'orbit', 'orbit_radius': 40, 'orbit_tilt': pitch,
                    'altitude': altitude, 'speed': 3, 'side_overlap': 90,
                    'capture_mode': 'full' if full else 'semi',
                    'finish_action': 'goHome', 'rc_lost_action': 'goBack',
                    'flight_path': 'curved', 'reverse_route': reverse,
                    'split_enabled': False, 'split_max_wp': 200,
                    'terrain_follow': False, 'cross_hatch': False,
                }
                payload = {'drone_model': 'mini3pro', 'settings': settings,
                           'polygon': [[51.05, -114.09]]}
                request = adapter.build_orbit_request(
                    centre=(-114.09, 51.05), drone_profile_id='mini3pro',
                    radius_m=40, altitude_m=altitude, gimbal_pitch_deg=pitch,
                    clockwise=not reverse, speed_m_s=3,
                    capture_mode=settings['capture_mode'], side_overlap_ratio=.9,
                    finish_action='Return to Home', max_waypoints_per_flight=200)
                website_result = website_plan(payload)
                plugin_result = adapter.plan(request)
                assert website_result['planning_result'] == plugin_result, (full, reverse)
                validate_saved_plan(request, plugin_result, website_result['waypoints'], payload)
                content, _, multiple = build_kmz_bundle(dict(
                    payload, waypoints=website_result['waypoints'],
                    planning_request=request, planning_result=plugin_result))
                assert not multiple, 'Orbits are a single flight.'
                flight = adapter.consume_result(request, plugin_result)[0]
                with tempfile.TemporaryDirectory(prefix='flypath-orbit-parity-') as directory:
                    path = Path(directory) / 'orbit.kmz'
                    spec = writers.MissionSpec(
                        waypoints=flight['waypoints'], altitude_m=altitude, speed_ms=3,
                        finish_action='Return to Home', rc_lost_action='Return to Home',
                        capture_mode=settings['capture_mode'], gimbal_pitch=pitch,
                        actions=flight['actions'], headings=flight['headings'])
                    writers.write_mission(registry.get('DJI Mini 3 Pro'), spec, str(path))
                    compare_exports(content, path.read_bytes())
                count += 1
    count += compare_corridors(website_plan, validate_saved_plan, build_kmz_bundle,
                               DroneConfig, PlannerPolicy, adapter, writers, registry)
    print(f'{count} consumer adapter parity cases passed')


def compare_corridors(website_plan, validate_saved_plan, build_kmz_bundle,
                      DroneConfig, PlannerPolicy, adapter, writers, registry):
    """Compare canonical Corridor decisions and both real KMZ serializers."""
    line = [{'latitude_deg': lat, 'longitude_deg': lon} for lat, lon in
            ((51., 8.), (51.001, 8.), (51.002, 8.001))]

    def elevation(point):
        return round(100 + (point['latitude_deg'] - 51) * 10_000
                     + (point['longitude_deg'] - 8) * 3_000, 3)

    count = 0
    drone = DroneConfig(code='mavic3pro', is_available=True, wpml_verified=True)
    with patch.object(DroneConfig.objects, 'filter') as lookup, patch.object(
            PlannerPolicy, 'current', return_value=PlannerPolicy()), patch(
            'missions.elevation.sample_elevations', side_effect=lambda points: [
                elevation({'latitude_deg': lat, 'longitude_deg': lon}) for lat, lon in points]):
        lookup.return_value.first.return_value = drone
        for full in (False, True):
            for reverse in (False, True):
                for terrain in (False, True):
                    for breaks in ([], [1]):
                        settings = {
                            'mapping_style': 'corridor', 'corridor_width': 120,
                            'corridor_break_indices': breaks, 'altitude': 40, 'speed': 5,
                            'side_overlap': 70, 'front_overlap': 70,
                            'capture_mode': 'full' if full else 'semi',
                            'finish_action': 'goHome', 'rc_lost_action': 'goBack',
                            'flight_path': 'curved', 'reverse_route': reverse,
                            'split_enabled': True, 'split_count': 2, 'split_max_wp': 70,
                            'terrain_follow': terrain, 'terrain_tolerance': 5,
                            'cross_hatch': False,
                        }
                        payload = {'drone_model': 'mavic3pro', 'settings': settings,
                                   'polygon': [[p['latitude_deg'], p['longitude_deg']] for p in line]}
                        request = adapter.build_corridor_request(
                            centre_line=line, width_m=120, break_indices=breaks,
                            drone_profile_id='mavic3pro', altitude_m=40, speed_m_s=5,
                            side_overlap_ratio=.7, capture_mode=settings['capture_mode'],
                            front_overlap_ratio=.7, turn_style='curved',
                            finish_action='Return to Home', split_enabled=True,
                            requested_flights=2, max_waypoints_per_flight=70,
                            reverse_route=reverse)
                        if terrain:
                            request['terrain_follow'] = {'tolerance_m': 5}
                            request['terrain_follow']['elevations_m'] = [
                                elevation(point) for point in adapter.corridor_sample_points(request)]
                        actual = website_plan(payload)
                        result = adapter.plan(request)
                        case = (full, reverse, terrain, breaks)
                        assert actual['planning_result'] == result, case
                        validate_saved_plan(request, result, actual['waypoints'], payload)
                        saved = dict(payload, waypoints=actual['waypoints'],
                                     planning_request=request, planning_result=result)
                        assert adapter.validate_mission_provenance(saved)[0], case
                        flights = adapter.consume_result(request, result)
                        for reference in ('shared', 'per_flight'):
                            terrain_flights = adapter.corridor_flights_with_heights(
                                request, result, takeoff_reference=reference)
                            for offset in (-5, 0, 5):
                                export = {**saved, 'settings': {**settings,
                                          'takeoff_reference': reference, 'launch_offset_m': offset}}
                                # Saved terrain is sufficient: exporting must never fetch or replan.
                                with patch('missions.elevation.sample_elevations',
                                           side_effect=AssertionError('export sampled terrain')):
                                    content, _, multiple = build_kmz_bundle(export)
                                if multiple:
                                    with zipfile.ZipFile(io.BytesIO(content)) as archive:
                                        website_flights = [archive.read(name) for name in archive.namelist()]
                                else:
                                    website_flights = [content]
                                assert len(website_flights) == len(flights), case
                                with tempfile.TemporaryDirectory(prefix='flypath-corridor-parity-') as directory:
                                    for index, (flight, website_kmz) in enumerate(zip(flights, website_flights)):
                                        heights = ([height - offset for height in terrain_flights[index]['heights']]
                                                   if terrain_flights else None)
                                        spec = writers.MissionSpec(
                                            waypoints=flight['waypoints'], altitude_m=40 - offset, speed_ms=5,
                                            finish_action='Return to Home', rc_lost_action='Return to Home',
                                            capture_mode=settings['capture_mode'], actions=flight['actions'],
                                            heights=heights)
                                        path = Path(directory) / f'{index}.kmz'
                                        writers.write_mission(registry.get('DJI Mavic 3 Pro'), spec, str(path))
                                        compare_exports(website_kmz, path.read_bytes())
                                count += 1
    return count


if __name__ == '__main__':
    main()
