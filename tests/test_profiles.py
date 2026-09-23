"""Shared drone profiles are complete enough for both clients."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from flypath_engine.profiles import drone_profile, load_drone_profiles  # noqa: E402


def test_shared_drone_profiles():
    profiles = load_drone_profiles()
    assert len(profiles) >= 7
    name, mini3 = drone_profile("mini3pro")
    assert name == "DJI Mini 3 Pro"
    assert mini3["camera"]["focal_length_mm"] == 6.9
    for profile in profiles.values():
        assert profile["aircraft"]["battery_time_min"] > 0
        assert profile["aircraft"]["battery_safe_min"] > 0
        assert profile["aircraft"]["default_speed_ms"] > 0
        assert profile["camera"]["sensor_width_mm"] > 0
        assert profile["camera"]["focal_length_mm"] > 0
        assert isinstance(profile["camera"]["continuous_trigger"], bool)

    profiles["DJI Mini 3 Pro"]["camera"]["focal_length_mm"] = -1
    assert drone_profile("mini3pro")[1]["camera"]["focal_length_mm"] == 6.9


if __name__ == "__main__":
    test_shared_drone_profiles()
    print("PASS  test_shared_drone_profiles")
