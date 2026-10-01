"""Shared geometry and metric records do not depend on evaluation or simulation."""

import json
from dataclasses import FrozenInstanceError, asdict
from math import pi, sqrt

import pytest

from cutin.core import PairMetrics, Role, VehicleState
from cutin.core.geometry import half_extents


@pytest.mark.parametrize(
    "heading, expected",
    [
        (0, (2.5, 1)),
        (pi / 2, (1, 2.5)),
        (pi / 4, (7 / (2 * sqrt(2)), 7 / (2 * sqrt(2)))),
    ],
)
def test_projected_half_extents(heading, expected):
    state = VehicleState(Role.EGO, 0, 0, heading, 20, 5, 2)
    assert half_extents(state) == pytest.approx(expected)


def test_nullable_metric_record_is_immutable_and_json_compatible():
    pair = PairMetrics(Role.FRONT)
    assert json.loads(json.dumps(asdict(pair), allow_nan=False)) == {
        "other_role": "front",
        "min_gap_m": None,
        "min_gap_tick": None,
        "min_ttc_s": None,
        "min_ttc_tick": None,
    }
    with pytest.raises(FrozenInstanceError):
        pair.min_gap_m = 0
