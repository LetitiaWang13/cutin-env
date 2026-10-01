"""Run configured driving policies, optionally render, and evaluate safety metrics."""

import argparse
import json
import tomllib
from dataclasses import asdict
from pathlib import Path

from cutin.core import ScenarioConfig, TerminationReason
from cutin.evaluation import evaluate
from cutin.simulation import SimulatorConfig, simulate


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
    scenario = ScenarioConfig.from_dict(tomllib.loads(args.scenario.read_text()))
    config = SimulatorConfig()
    if args.render:
        from cutin.simulation.adapters.highway_env import HighwayEnvAdapter

        adapter = HighwayEnvAdapter(scenario, render_mode="human")
        print("Colors: blue=ego, green=front, orange=cut-in. Close window to cancel.")

        def draw(snapshot):
            # Human mode already renders through Gym reset/step. Do not draw
            # each frame twice; this callback handles close/terminal hold only.
            if adapter.episode_finished and not adapter.closed:
                print("Run finished. Close the native window to continue.")
                while not adapter.closed:
                    adapter.render()
            return not adapter.closed

        log = simulate(
            scenario, config, adapter_factory=lambda _: adapter, on_step=draw
        )
    else:
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
    metrics = evaluate(log)
    print("Assessment:", metrics.assessment_status)
    print("Collision / ego collision:", metrics.collision, metrics.ego_collision)
    for pair in metrics.pairs:
        gap = "n/a" if pair.min_gap_m is None else f"{pair.min_gap_m:.3f} m"
        ttc = "n/a" if pair.min_ttc_s is None else f"{pair.min_ttc_s:.3f} s"
        print(f"  ego/{pair.other_role}: min_gap={gap}, min_ttc={ttc}")
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
