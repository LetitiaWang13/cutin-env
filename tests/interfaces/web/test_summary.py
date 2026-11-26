"""Display summary: collision status, pair minima and warning flags."""

import pytest

from cutin.interfaces.web.inputs import (
    default_scenario,
    default_simulation,
)
from cutin.interfaces.web.summary import Warnings, observe_pair, summarize

WARNINGS = Warnings(gap_warning_m=2.0, ttc_warning_s=1.5)


@pytest.fixture
def record(make_record):
    def build(**kwargs):
        return make_record(default_scenario(), default_simulation(), **kwargs)

    return build


def test_horizon_run_without_collision_is_complete_not_safe(record):
    summary = summarize(record(), WARNINGS)
    assert summary["assessment_status"] == "complete"
    assert not summary["collision"] and not summary["ego_collision"]
    assert summary["physics_hz"] == 30 and summary["end_tick"] == 2
    front, cut_in = summary["pairs"]
    assert front["other_role"] == "front" and cut_in["other_role"] == "cut_in"
    assert front["min_gap_m"] == pytest.approx(55.0)
    assert front["min_ttc_s"] is None  # Equal speeds never close.
    assert cut_in["min_gap_m"] is None  # Never overlapping laterally.
    assert front["ttc_unavailable_counts"]["not_closing"] == 3
    assert cut_in["ttc_unavailable_counts"]["no_lateral_overlap"] == 3


def test_ego_collision_terminates_with_flags_and_warnings(record):
    summary = summarize(record(collide=True), WARNINGS)
    assert summary["termination_reason"] == "collision"
    assert summary["ego_collision"] and summary["first_ego_collision_tick"] == 2
    assert summary["collision_roles"] == ["ego", "cut_in"]


def test_non_ego_collision_is_reported_as_non_target(record):
    rec = record(collide=True)
    rec.frames[-1].ego.collided = False
    summary = summarize(rec, WARNINGS)
    assert summary["assessment_status"] == "non_target_collision"
    assert not summary["ego_collision"] and summary["collision"]


def test_gap_and_ttc_follow_the_longitudinal_geometry(make_vehicle):
    vehicle = make_vehicle
    ego = vehicle(0, y=4, speed=30)
    ahead = vehicle(20, y=4, speed=20)
    gap, ttc = observe_pair(ego, ahead)
    assert gap == pytest.approx(15.0) and ttc == pytest.approx(1.5)
    behind_other = vehicle(-20, y=4, speed=20)
    gap, ttc = observe_pair(ego, behind_other)
    assert gap == pytest.approx(15.0) and ttc is None  # Opening, not closing.
    assert observe_pair(ego, vehicle(20, y=0, speed=20)) == (None, None)


def test_warning_thresholds_only_change_interpretation(record):
    rec = record()
    strict = summarize(rec, Warnings(gap_warning_m=100, ttc_warning_s=1.5))
    loose = summarize(rec, Warnings(gap_warning_m=2, ttc_warning_s=1.5))
    assert strict["gap_warning"] and not loose["gap_warning"]
    assert strict["pairs"] == loose["pairs"]
