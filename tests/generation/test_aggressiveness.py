"""Aggressiveness defaults resolve to a usable evaluation profile."""

import pytest

from cutin.config import GenerationConfig, RobustnessTargets
from cutin.generation.optim.translators.aggressiveness import AggressivenessTranslator


def test_default_aggressiveness_resolves_to_level_five_targets():
    translator = AggressivenessTranslator()

    targets = translator.translate(GenerationConfig().aggressiveness)

    assert isinstance(targets, RobustnessTargets)
    assert targets == translator.translate(5)


def test_unsupported_aggressiveness_reports_available_levels():
    with pytest.raises(ValueError, match=r"Available levels: 5\."):
        AggressivenessTranslator().translate(-1)
