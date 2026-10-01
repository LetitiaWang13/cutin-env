"""Deterministic proposal semantics, without simulation or filesystem dependencies."""

import json
from dataclasses import replace
from random import Random, getstate

import pytest

from cutin.core import ScenarioConfig
from cutin.generation import ParameterRange, RejectedSample, SearchSpace, sample
from cutin.generation.space import ALLOWED_PATHS


def space():
    return SearchSpace(
        (
            ParameterRange("initial.cut_in.x_m", 45, 80),
            ParameterRange("cut_in.start_time_s", 0, 3),
            ParameterRange("cut_in.lane_change_duration_s", 0.8, 3),
        )
    )


def test_identical_local_seeds_and_stream_progression():
    base = ScenarioConfig()
    a, b = Random(17), Random(17)
    first = sample(base, space(), a)
    second = sample(base, space(), a)
    assert first == sample(base, space(), b)
    assert second == sample(base, space(), b)
    assert first != second
    assert first.seed == second.seed == base.seed
    assert base == ScenarioConfig()


def test_input_order_is_irrelevant_and_draw_order_is_alphabetical():
    base = ScenarioConfig()
    normal = space()
    reversed_space = replace(normal, ranges=tuple(reversed(normal.ranges)))
    rng, reference = Random(5), Random(5)
    candidate = sample(base, reversed_space, rng)
    values = candidate.to_dict()
    for item in sorted(normal.ranges, key=lambda item: item.path):
        target = values
        for part in item.path.split("."):
            target = target[part]
        assert target == reference.uniform(item.low, item.high)
    assert rng.getstate() == reference.getstate()
    assert candidate == sample(base, normal, Random(5))


def test_sampling_never_uses_the_global_rng():
    before = getstate()
    sample(ScenarioConfig(), space(), Random(17))
    assert getstate() == before


def test_fixed_and_empty_spaces_consume_no_draws():
    rng = Random(7)
    state = rng.getstate()
    base = ScenarioConfig()
    assert sample(base, SearchSpace(), rng) == base
    fixed = SearchSpace((ParameterRange("initial.cut_in.x_m", 60, 60),))
    assert sample(base, fixed, rng).initial.cut_in.x_m == 60
    assert rng.getstate() == state
    variable = space()
    added_fixed = replace(
        variable, ranges=variable.ranges + (ParameterRange("initial.ego.x_m", 50, 50),)
    )
    assert sample(base, variable, Random(7)) == sample(base, added_fixed, Random(7))


@pytest.mark.parametrize("path", ALLOWED_PATHS)
def test_each_allowed_path_updates_only_the_selected_field(path):
    base = ScenarioConfig()
    value = {
        "initial.ego.x_m": 45,
        "initial.front.x_m": 120,
        "initial.cut_in.x_m": 60,
    }.get(path, 3)
    proposal = sample(
        base, SearchSpace((ParameterRange(path, value, value),)), Random(3)
    )
    expected = base.to_dict()
    target = expected
    parts = path.split(".")
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = float(value)
    assert proposal.to_dict() == expected
    assert (
        proposal.road == base.road and proposal.baseline_policy == base.baseline_policy
    )


def test_many_valid_proposals_stay_inside_declared_ranges():
    rng = Random(19)
    for _ in range(200):
        candidate = sample(ScenarioConfig(), space(), rng)
        assert 45 <= candidate.initial.cut_in.x_m <= 80
        assert 0 <= candidate.cut_in.start_time_s <= 3
        assert 0.8 <= candidate.cut_in.lane_change_duration_s <= 3


@pytest.mark.parametrize(
    "path, value",
    [
        ("initial.front.x_m", 40),
        ("initial.front.x_m", 55),
        ("initial.ego.x_m", 0),
        ("initial.cut_in.x_m", 3000),
        ("initial.cut_in.speed_mps", -1),
        ("cut_in.lane_change_duration_s", 0),
    ],
)
def test_invalid_candidate_is_an_explicit_rejection_not_a_retry(path, value):
    rng = Random(19)
    state = rng.getstate()
    with pytest.raises(RejectedSample):
        sample(
            ScenarioConfig(), SearchSpace((ParameterRange(path, value, value),)), rng
        )
    assert rng.getstate() == state  # Fixed invalid proposal never consumes randomness.


def test_rejection_consumes_exactly_one_variable_draw():
    rng, reference = Random(19), Random(19)
    reference.uniform(30, 40)
    with pytest.raises(RejectedSample, match="front must start ahead"):
        sample(
            ScenarioConfig(),
            SearchSpace((ParameterRange("initial.front.x_m", 30, 40),)),
            rng,
        )
    assert rng.getstate() == reference.getstate()


def test_invalid_base_or_space_is_not_a_candidate_rejection():
    rng = Random(19)
    state = rng.getstate()
    for base, search in [
        (replace(ScenarioConfig(), seed=-1), space()),
        (ScenarioConfig(), SearchSpace((ParameterRange("seed", 1, 2),))),
    ]:
        with pytest.raises(ValueError) as error:
            sample(base, search, rng)
        assert not isinstance(error.value, RejectedSample)
        assert rng.getstate() == state


def test_space_json_roundtrip_and_partial_horizon_are_allowed():
    original = space()
    assert (
        SearchSpace.from_dict(
            json.loads(json.dumps(original.to_dict(), allow_nan=False))
        )
        == original
    )
    candidate = sample(
        ScenarioConfig(),
        SearchSpace((ParameterRange("cut_in.start_time_s", 20, 20),)),
        Random(3),
    )
    assert candidate.cut_in.start_time_s == 20  # No invented must-finish constraint.
