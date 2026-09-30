"""Run the configured driving policies headlessly; metrics/visualization follow."""

import argparse
import json
import tomllib
from dataclasses import asdict
from pathlib import Path

from cutin.core import ScenarioConfig, TerminationReason
from cutin.simulation import SimulatorConfig, simulate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario", nargs="?", type=Path, default=Path("examples/baseline.toml")
    )
    parser.add_argument("--output", type=Path, help="Optional JSON log; no overwrite")
    args = parser.parse_args()
    scenario = ScenarioConfig.from_dict(tomllib.loads(args.scenario.read_text()))
    config = SimulatorConfig()
    log = simulate(scenario, config)
    print("Termination:", log.termination_reason)
    print("Cut-in reference/completion ticks:", log.cut_in_report)
    if log.entries:
        final = log.entries[-1].snapshot
        print(f"End: tick={final.tick}, time={final.tick / config.physics_hz:.3f} s")
        for state in final.vehicles:
            print(
                f"  {state.role}: x={state.x_m:.3f} m, y={state.y_m:.3f} m, "
                f"speed={state.speed_mps:.3f} m/s, collided={state.collided}"
            )
    if log.diagnostic:
        print("Diagnostic:", log.diagnostic)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as output:
            json.dump(asdict(log), output, indent=2, allow_nan=False)
            output.write("\n")
        print("Saved:", args.output)
    if log.termination_reason == TerminationReason.ERROR:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
