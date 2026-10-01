"""Run a cut-in experiment and optionally save its artifacts."""

import argparse
from functools import partial
from pathlib import Path

from cutin.application import RunResult, load_scenario, run_scenario, save_run
from cutin.core import ScenarioConfig, TerminationReason
from cutin.evaluation import EvaluationConfig
from cutin.evaluation.config import validate_evaluation_config
from cutin.simulation import SimulatorConfig, simulate


def run_with_viewer(
    scenario: ScenarioConfig,
    simulator_config: SimulatorConfig,
    evaluation_config: EvaluationConfig,
) -> RunResult:
    """Keep native UI/input in the interface, not the application workflow."""
    from cutin.simulation.adapters.highway_env import HighwayEnvAdapter

    adapter = HighwayEnvAdapter(scenario, render_mode="human")
    print("Colors: blue=ego, green=front, orange=cut-in. Close window to cancel.")

    def draw(snapshot):
        # Gym human reset/step already draws; do not render physics ticks twice.
        if adapter.episode_finished and not adapter.closed:
            print("Run finished. Close the native window to continue.")
            while not adapter.closed:
                adapter.render()
        return not adapter.closed

    try:
        return run_scenario(
            scenario,
            simulator_config,
            evaluation_config,
            simulate_fn=partial(
                simulate, adapter_factory=lambda _: adapter, on_step=draw
            ),
        )
    finally:
        adapter.close()


def print_result(result: RunResult) -> None:
    log, metrics = result.log, result.metrics
    print("Termination:", log.termination_reason)
    print("Cut-in reference/completion ticks:", log.cut_in_report)
    if log.entries:
        final = log.entries[-1].snapshot
        print(f"End: tick={final.tick}, time={final.tick / metrics.physics_hz:.3f} s")
        for state in final.vehicles:
            print(
                f"  {state.role}: x={state.x_m:.3f} m, y={state.y_m:.3f} m, "
                f"speed={state.speed_mps:.3f} m/s, collided={state.collided}"
            )
    if log.diagnostic:
        print("Diagnostic:", log.diagnostic)
    print("Assessment:", metrics.assessment_status)
    print("Collision / ego collision:", metrics.collision, metrics.ego_collision)
    for pair in metrics.pairs:
        gap = "n/a" if pair.min_gap_m is None else f"{pair.min_gap_m:.3f} m"
        ttc = "n/a" if pair.min_ttc_s is None else f"{pair.min_ttc_s:.3f} s"
        print(f"  ego/{pair.other_role}: min_gap={gap}, min_ttc={ttc}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="Simulate and evaluate a TOML/JSON scenario")
    run.add_argument("scenario", type=Path)
    run.add_argument("--output", type=Path, help="New run directory; never overwrite")
    run.add_argument("--render", action="store_true", help="Native highway-env window")
    run.add_argument("--gap-warning-m", type=float, default=2.0)
    run.add_argument("--ttc-warning-s", type=float, default=1.5)
    args = parser.parse_args(argv)
    try:
        if args.output and (args.output.exists() or args.output.is_symlink()):
            raise FileExistsError(f"output already exists: {args.output}")
        scenario = load_scenario(args.scenario)
        simulator = SimulatorConfig()
        evaluation = EvaluationConfig(args.gap_warning_m, args.ttc_warning_s)
        # Reject bad input before opening a viewer.
        validate_evaluation_config(evaluation)
        result = (
            run_with_viewer(scenario, simulator, evaluation)
            if args.render
            else run_scenario(scenario, simulator, evaluation)
        )
        print_result(result)
        if args.output:
            save_run(result, args.output)
            print("Saved:", args.output)
    except (OSError, ValueError, TypeError) as error:
        parser.error(str(error))
    if result.log.termination_reason == TerminationReason.ERROR:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
