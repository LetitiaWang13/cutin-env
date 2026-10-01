"""Thin experiment workflows with optional, explicit persistence."""

from cutin.application.storage import load_scenario, save_run
from cutin.application.workflows import RunResult, run_scenario

__all__ = ["RunResult", "load_scenario", "run_scenario", "save_run"]
