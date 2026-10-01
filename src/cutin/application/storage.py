"""Explicit scenario input and fresh-directory artifact output. No implicit I/O."""

import json
import shutil
import tomllib
from dataclasses import asdict, fields
from pathlib import Path

from cutin.application.workflows import RunResult, SearchRunResult
from cutin.core import ScenarioConfig
from cutin.generation import SearchSpace


def load_scenario(path: Path) -> ScenarioConfig:
    return ScenarioConfig.from_dict(_load_object(path))


def load_search_space(path: Path) -> SearchSpace:
    return SearchSpace.from_dict(_load_object(path))


def _load_object(path: Path) -> dict:
    if path.suffix.lower() == ".toml":
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    elif path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
    else:
        raise ValueError("input must be .toml or .json")
    if not isinstance(data, dict):
        raise ValueError("input must be an object")
    return data


def load_run_documents(directory: Path) -> dict:
    """Read saved JSON for replay without stepping or re-evaluating the run."""
    return {
        "log": _load_object(directory / "log.json"),
        "metrics": _load_object(directory / "metrics.json"),
    }


def save_run(result: RunResult, directory: Path) -> Path:
    """Save five views of one result; refuse any existing file or directory.

    Encode everything before claiming the output directory. On write failure,
    remove our newly created directory; the supplied result remains in memory.
    This is not a crash-atomic store or a mutable experiment database.
    """
    return _write_documents(_run_documents(result), directory)


def _run_documents(result: RunResult) -> dict:
    return {
        "scenario.json": result.log.scenario.to_dict(),
        "simulator.json": asdict(result.simulator_config),
        "evaluation.json": asdict(result.evaluation_config),
        "log.json": asdict(result.log),
        "metrics.json": asdict(result.metrics),
    }


def save_search(result: SearchRunResult, directory: Path) -> Path:
    search = result.search
    excluded = {"base", "space", "config", "best_log", "best_metrics"}
    summary = {
        field.name: getattr(search, field.name)
        for field in fields(search)
        if field.name not in excluded
    }
    summary.update(
        config=asdict(search.config),
        best_config=search.best_config.to_dict()
        if search.best_config is not None
        else None,
        best_metrics=asdict(search.best_metrics)
        if search.best_metrics is not None
        else None,
        best_run="best" if search.best_log is not None else None,
    )
    documents = {
        "base.json": search.base.to_dict(),
        "space.json": search.space.to_dict(),
        "search.json": summary,
        "simulator.json": asdict(result.simulator_config),
        "evaluation.json": asdict(result.evaluation_config),
    }
    if search.best_log is not None:
        best = RunResult(
            search.best_log,
            search.best_metrics,
            result.simulator_config,
            result.evaluation_config,
        )
        documents.update(
            {f"best/{name}": data for name, data in _run_documents(best).items()}
        )
    return _write_documents(documents, directory)


def _write_documents(documents: dict, directory: Path) -> Path:
    encoded = {
        name: json.dumps(data, indent=2, allow_nan=False) + "\n"
        for name, data in documents.items()
    }
    directory.mkdir(parents=True, exist_ok=False)
    try:
        for name, text in encoded.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
    except OSError:
        shutil.rmtree(directory)
        raise
    return directory
