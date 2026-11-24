"""Shared flight partitioning keeps its choices without repeated location work."""

from pathlib import Path
import random
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from flypath_engine import planning


def test_shared_location_costs_are_computed_once():
    points = [{"latitude_deg": 51.0 + index * .0001, "longitude_deg": -114.0}
              for index in range(5)]
    location = {"launch": points[0], "home": points[-1]}
    locations = ("shared", location)
    with patch.object(planning, "_distance", wraps=planning._distance) as distance:
        boundaries = planning._balanced_boundaries(
            points, [10.0] * 4, 3, 4, 5.0, "semi_auto", 2.5,
            1_000.0, "return_to_home", locations,
        )
    assert boundaries is not None
    assert len(boundaries) == 3
    assert distance.call_count == 2 * len(points)


def test_equal_cost_partitions_keep_the_first_tie():
    points = [{} for _ in range(6)]
    boundaries = planning._balanced_boundaries(
        points, [1.0] * 5, 2, 4, 1.0, "semi_auto", 2.5,
        1_000.0, "hover", None,
    )
    assert boundaries == [(0, 2), (2, 5)]


@pytest.mark.parametrize("mode", ["semi_auto", "full_auto"])
@pytest.mark.parametrize("finish_action", ["hover", "return_to_home", "return_to_first_waypoint"])
@pytest.mark.parametrize("location_kind", [None, "shared", "per_flight"])
@pytest.mark.parametrize("budget", [15.0, 1_000.0])
def test_lazy_solver_matches_existing_balanced_partitions(mode, finish_action,
                                                           location_kind, budget):
    points = [{"latitude_deg": 51.0 + index * .0001,
               "longitude_deg": -114.0 + (index % 2) * .0001}
              for index in range(6)]
    edges = [3.0, 7.0, 2.0, 6.0, 4.0]
    location = {"launch": points[0], "home": points[-1]}
    locations = (("shared", location) if location_kind == "shared" else
                 ("per_flight", [location] * 5) if location_kind == "per_flight" else None)
    solve = planning._balanced_boundaries_solver(
        points, edges, 4, 5.0, mode, 2.5, budget, finish_action, locations)
    for count in (3, 1, 5, 2, 4):
        expected = planning._balanced_boundaries(
            points, edges, count, 4, 5.0, mode, 2.5,
            budget, finish_action, locations)
        assert solve(count) == expected


def test_lazy_solver_matches_reference_on_varied_small_routes():
    rng = random.Random(5902)
    for _ in range(100):
        size = rng.randrange(2, 14)
        points = [{"latitude_deg": 51 + index * .00005,
                   "longitude_deg": -114 + (index % 3) * .0001}
                  for index in range(size)]
        edges = [rng.uniform(.5, 10) for _ in range(size - 1)]
        limit = rng.randrange(2, min(7, size + 1))
        speed = rng.uniform(1, 10)
        mode = rng.choice(("semi_auto", "full_auto"))
        allowance = rng.uniform(1, 5)
        budget = rng.uniform(3, 80)
        finish = rng.choice(("hover", "return_to_home", "return_to_first_waypoint"))
        location = {"launch": points[0], "home": points[-1]}
        locations = rng.choice((None, ("shared", location),
                                ("per_flight", [location] * (size - 1))))
        solve = planning._balanced_boundaries_solver(
            points, edges, limit, speed, mode, allowance, budget, finish, locations)
        counts = list(range(1, size))
        rng.shuffle(counts)
        for count in counts:
            assert solve(count) == planning._balanced_boundaries(
                points, edges, count, limit, speed, mode, allowance,
                budget, finish, locations)
