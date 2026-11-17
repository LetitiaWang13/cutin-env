"""Run folders: one subfolder per saved run, named `<date-time>-<run name>`.

Each folder holds record.json (the full ScenarioRecord, enough to replay the run
without simulating again) and metrics.json (the Web summary). Existing folders
are never overwritten.
"""

import json
import shutil
from dataclasses import asdict
from pathlib import Path

from cutin.core.record import ScenarioRecord

FILES = ("record.json", "metrics.json")


def documents(record: ScenarioRecord, metrics: dict) -> dict:
    """Plain JSON-ready data; raises on NaN/inf instead of writing invalid JSON."""
    return json.loads(
        json.dumps({"record": asdict(record), "metrics": metrics}, allow_nan=False)
    )


def save_run(record: ScenarioRecord, metrics: dict, directory: Path) -> Path:
    data = documents(record, metrics)
    encoded = {
        "record.json": json.dumps(data["record"], indent=2) + "\n",
        "metrics.json": json.dumps(data["metrics"], indent=2) + "\n",
    }
    directory.mkdir(parents=True, exist_ok=False)
    try:
        for name, text in encoded.items():
            (directory / name).write_text(text, encoding="utf-8")
    except OSError:
        shutil.rmtree(directory)
        raise
    return directory


def load_run(directory: Path) -> dict:
    return {
        "record": json.loads((directory / "record.json").read_text("utf-8")),
        "metrics": json.loads((directory / "metrics.json").read_text("utf-8")),
    }
