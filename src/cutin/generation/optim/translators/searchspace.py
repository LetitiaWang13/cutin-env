"""Fixed user conditions to tunable initial-state/controller dimensions."""

from cutin.config import GenerationConfig


class SearchSpaceTranslator:
    def translate(self, config: GenerationConfig):
        """Return search dimensions and ranges; parameterization is pending."""
        raise NotImplementedError
