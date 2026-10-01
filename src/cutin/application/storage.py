"""Explicit scenario input and fresh-directory artifact output. No implicit I/O."""

import json
import shutil
import tomllib
from dataclasses import asdict
from pathlib import Path

from cutin.application.workflows import RunResult
from cutin.core import ScenarioConfig


def load_scenario(path: Path) -> ScenarioConfig:
    if path.suffix.lower() == ".toml":
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    elif path.suffix.lower() == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))
    else:
        raise ValueError("scenario input must be .toml or .json")
    if not isinstance(data, dict):
        raise ValueError("scenario input must be an object")
    return ScenarioConfig.from_dict(data)


def save_run(result: RunResult, directory: Path) -> Path:
    """Save five views of one result; refuse any existing file or directory.

    Encode everything before claiming the output directory. On write failure,
    remove our newly created directory; the supplied result remains in memory.
    This is not a crash-atomic store or a mutable experiment database.
    """
    documents = {
        "scenario.json": result.log.scenario.to_dict(),
        "simulator.json": asdict(result.simulator_config),
        "evaluation.json": asdict(result.evaluation_config),
        "log.json": asdict(result.log),
        "metrics.json": asdict(result.metrics),
    }
    encoded = {
        name: json.dumps(data, indent=2, allow_nan=False) + "\n"
        for name, data in documents.items()
    }
    directory.mkdir(parents=True, exist_ok=False)
    try:
        for name, text in encoded.items():
            (directory / name).write_text(text, encoding="utf-8")
    except OSError:
        shutil.rmtree(directory)
        raise
    return directory
