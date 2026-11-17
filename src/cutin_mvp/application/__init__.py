"""Thin experiment workflows with optional, explicit persistence."""

from cutin_mvp.application.storage import (
    load_scenario,
    load_search_space,
    save_run,
    save_search,
)
from cutin_mvp.application.workflows import (
    RunResult,
    SearchRunResult,
    run_scenario,
    search_scenarios,
)

__all__ = [
    "RunResult",
    "SearchRunResult",
    "load_scenario",
    "load_search_space",
    "run_scenario",
    "search_scenarios",
    "save_run",
    "save_search",
]
