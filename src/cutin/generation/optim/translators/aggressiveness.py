"""Resolve aggressiveness levels to TTC and LCT targets."""

from cutin.config import RobustnessTargets

# Experimental profiles in increasing aggressiveness. Both are strict upper limits.
_PROFILES = {
    1: RobustnessTargets(ttc_upper_s=11.2, lct_upper_s=4.2),
    2: RobustnessTargets(ttc_upper_s=8.0, lct_upper_s=3.7),
    3: RobustnessTargets(ttc_upper_s=4.9, lct_upper_s=3.4),
    4: RobustnessTargets(ttc_upper_s=3.0, lct_upper_s=3.1),
    5: RobustnessTargets(ttc_upper_s=2.0, lct_upper_s=2.8),
}


class AggressivenessTranslator:
    def translate(self, aggressiveness: int) -> RobustnessTargets:
        """Return the targets for a configured aggressiveness level."""
        targets = _PROFILES.get(aggressiveness) if type(aggressiveness) is int else None
        if targets is None:
            levels = ", ".join(str(level) for level in sorted(_PROFILES))
            raise ValueError(
                f"Unsupported aggressiveness level {aggressiveness}. "
                f"Available levels: {levels}."
            )
        return targets
