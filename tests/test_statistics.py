"""Shared mission-statistics rules used by every FlyPath client."""

from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine.statistics import mission_statistics  # noqa: E402


def test_semi_auto_statistics_match_plugin_rules():
    stats = mission_statistics(
        route_distance_m=24_190.247243,
        speed_m_s=5,
        capture_mode="semi",
        photo_interval_s=2,
        battery_minutes=24,
    )
    assert math.isclose(stats["distance_m"], 24_190.247243)
    assert math.isclose(stats["flight_seconds"], 4_838.0494486)
    assert stats["photo_count"] == 2_419
    assert stats["battery_count"] == 4

    full = mission_statistics(
        route_distance_m=1_000,
        speed_m_s=5,
        capture_mode="full",
        photo_interval_s=2,
        battery_minutes=24,
        waypoint_count=100,
    )
    assert full["photo_count"] == 100
    assert full["flight_seconds"] == 450


if __name__ == "__main__":
    test_semi_auto_statistics_match_plugin_rules()
    print("PASS  test_semi_auto_statistics_match_plugin_rules")
