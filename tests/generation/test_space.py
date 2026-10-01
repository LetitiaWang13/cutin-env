"""Search-space input boundaries; record constructors deliberately remain simple."""

import pytest

from cutin.generation import ParameterRange, SearchSpace, validate_search_space


@pytest.mark.parametrize(
    "bounds",
    [
        [2, 1],
        [True, 2],
        [0, float("nan")],
        [0, float("inf")],
        ["0", 2],
        [0, 10**1000],
        [-1e308, 1e308],
        [1],
        [1, 2, 3],
        2,
        {"low": 0, "high": 1},
    ],
)
def test_invalid_bounds(bounds):
    with pytest.raises(ValueError):
        SearchSpace.from_dict({"ranges": {"initial.ego.x_m": bounds}})


@pytest.mark.parametrize(
    "data",
    [
        [],
        {"unknown": 1},
        {"schema_version": 2},
        {"schema_version": True},
        {"ranges": []},
        {"ranges": {"seed": [0, 3]}},
        {"ranges": {"road.length_m": [100, 200]}},
        {"ranges": {"initial.ego.lane": [0, 2]}},
        {"ranges": {"baseline_policy.desired_speed_mps": [20, 30]}},
    ],
)
def test_bad_schema_and_disallowed_paths(data):
    with pytest.raises(ValueError):
        SearchSpace.from_dict(data)


def test_duplicate_paths_and_bad_python_records_fail_at_boundary():
    item = ParameterRange("initial.ego.x_m", 45, 55)
    for space in (SearchSpace((item, item)), SearchSpace((0,)), SearchSpace([item])):
        with pytest.raises(ValueError):
            validate_search_space(space)


def test_negative_proposal_bounds_are_syntax_not_scenario_validity():
    # They are not clamped: the candidate validator rejects invalid draws.
    assert (
        SearchSpace.from_dict({"ranges": {"initial.ego.speed_mps": [-1, 1]}})
        .ranges[0]
        .low
        == -1
    )
