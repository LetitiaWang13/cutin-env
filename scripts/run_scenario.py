"""Legacy diagnostic entry point; --output remains a single raw JSON log."""

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from cutin.application import load_scenario, run_scenario
from cutin.core import TerminationReason
from cutin.evaluation import EvaluationConfig
from cutin.interfaces.cli import print_result, run_with_viewer
from cutin.simulation import SimulatorConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario", nargs="?", type=Path, default=Path("examples/baseline.toml")
    )
    parser.add_argument("--output", type=Path, help="Optional JSON log; no overwrite")
    parser.add_argument(
        "--render", action="store_true", help="Native highway-env window"
    )
    args = parser.parse_args()
    scenario = load_scenario(args.scenario)
    result = (
        run_with_viewer(scenario, SimulatorConfig(), EvaluationConfig())
        if args.render
        else run_scenario(scenario)
    )
    print_result(result)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as output:
            json.dump(asdict(result.log), output, indent=2, allow_nan=False)
            output.write("\n")
        print("Saved:", args.output)
    if result.log.termination_reason == TerminationReason.ERROR:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
