"""Exercise highway-env with raw actions; this is not the cut-in controller."""

import argparse
import json
import tomllib
from dataclasses import asdict
from pathlib import Path

from cutin.core import ControlAction, Role, ScenarioConfig
from cutin.simulation import SimulatorConfig, simulate


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "scenario", nargs="?", type=Path, default=Path("examples/baseline.toml")
    )
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument(
        "--acceleration",
        type=float,
        default=1.0,
        help="Raw cut-in acceleration in m/s^2",
    )
    parser.add_argument(
        "--steering", type=float, default=0.02, help="Raw cut-in steering in radians"
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional JSON log; existing files are not overwritten",
    )
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    scenario = ScenarioConfig.from_dict(tomllib.loads(args.scenario.read_text()))
    config = SimulatorConfig()
    actions = {role: ControlAction(0, 0) for role in Role}
    actions[Role.CUT_IN] = ControlAction(args.acceleration, args.steering)
    log = simulate(scenario, config, actions=actions, max_steps=args.steps)
    print("Raw-action probe only: scenario cut-in timing/policy is not executed.")
    print("Simulator:", log.simulator_settings)
    if log.entries:
        for label, snapshot in (
            ("Initial", log.entries[0].snapshot),
            ("Final", log.entries[-1].snapshot),
        ):
            print(
                f"{label}: tick={snapshot.tick}, "
                f"time={snapshot.tick / config.physics_hz:.3f} s"
            )
            for state in snapshot.vehicles:
                print(
                    f"  {state.role}: x={state.x_m:.3f} m, y={state.y_m:.3f} m, "
                    f"speed={state.speed_mps:.3f} m/s, collided={state.collided}"
                )
    print("Termination:", log.termination_reason)
    if log.diagnostic:
        print("Diagnostic:", log.diagnostic)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as output:
            json.dump(asdict(log), output, indent=2, allow_nan=False)
            output.write("\n")
        print("Saved:", args.output)
    if log.termination_reason == "error":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
