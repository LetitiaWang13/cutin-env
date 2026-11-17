"""Bounded, backend-independent empirical falsification."""

from cutin_mvp.falsification.config import SearchConfig
from cutin_mvp.falsification.search import SearchResult, SearchStopReason, search

__all__ = ["SearchConfig", "SearchResult", "SearchStopReason", "search"]
