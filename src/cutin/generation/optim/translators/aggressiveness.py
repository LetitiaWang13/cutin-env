"""Resolve aggressiveness levels to TTC and LCT targets."""

from cutin.config import RobustnessTargets

# Starter profile for experiments. Both values are strict upper limits in seconds.
_PROFILES = {
    5: RobustnessTargets(ttc_upper_s=1.0, lct_upper_s=2.5),
}


class AggressivenessTranslator:
    def translate(self, aggressiveness: int) -> RobustnessTargets:
        """Return the targets for a configured aggressiveness level."""
        targets = _PROFILES.get(aggressiveness)
        if targets is None:
            levels = ", ".join(str(level) for level in sorted(_PROFILES))
            raise ValueError(
                f"Unsupported aggressiveness level {aggressiveness}. "
                f"Available levels: {levels}."
            )
        return targets
