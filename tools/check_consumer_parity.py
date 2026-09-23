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
            for tag in ('actionActuatorFunc', 'waypointTurnMode', 'useStraightLine'):
                assert [e.text for e in left.findall('.//{*}' + tag)] == [
                    e.text for e in right.findall('.//{*}' + tag)], tag
            for tag in ('hoverTime', 'gimbalPitchRotateAngle', 'waypointSpeed'):
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
    drone = DroneConfig(code='mini3pro', category='consumer',
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
    print(f'{count} consumer adapter parity cases passed')


if __name__ == '__main__':
    main()
