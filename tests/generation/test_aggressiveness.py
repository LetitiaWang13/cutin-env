"""Aggressiveness defaults resolve to nested evaluation profiles."""

import pytest

from cutin.config import GenerationConfig, RobustnessTargets
from cutin.generation.optim.translators.aggressiveness import AggressivenessTranslator


def test_default_aggressiveness_resolves_to_level_five_targets():
    translator = AggressivenessTranslator()

    targets = translator.translate(GenerationConfig().aggressiveness)

    assert isinstance(targets, RobustnessTargets)
    assert targets == translator.translate(5)


def test_higher_aggressiveness_levels_have_stricter_targets():
    translator = AggressivenessTranslator()
    profiles = [translator.translate(level) for level in range(1, 6)]

    for lower, higher in zip(profiles[:-1], profiles[1:], strict=True):
        assert higher.ttc_upper_s < lower.ttc_upper_s
        assert higher.lct_upper_s < lower.lct_upper_s


@pytest.mark.parametrize("level", [-1, 6, True, 5.0])
def test_unsupported_aggressiveness_reports_available_levels(level):
    with pytest.raises(ValueError, match=r"Available levels: 1, 2, 3, 4, 5\."):
        AggressivenessTranslator().translate(level)
