"""Bounded, backend-independent empirical falsification."""

from cutin.falsification.config import SearchConfig
from cutin.falsification.search import SearchResult, SearchStopReason, search

__all__ = ["SearchConfig", "SearchResult", "SearchStopReason", "search"]
