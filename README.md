# Cut-in scenario generator

A course project for risky cut-in scenario generation using STL/robustness-based
falsification and Bayesian optimization. The repository separates a working MVP
from a replacement framework under development.

See [DEVELOPMENT_PLAN.md](DEVELOPMENT_PLAN.md) for component work blocks,
shared-interface decisions, and the next integration milestones.

## Status

- `src/cutin_mvp/`: working MVP, moved without changing its simulation/search logic.
- `tests/mvp/`: the corresponding MVP tests.
- `src/cutin/`: concrete scenario/record models, native IDM, four-phase cut-in,
  and the headless highway-env adapter are implemented. Generation, metrics, and
  CLI/Web remain explicit `NotImplementedError` skeletons.
- `tests/simulation/adapters/highway_env/`: real controller and adapter integration
  tests, including complete motion records and JSON export.
- `tests/test_skeleton.py`: public import and unimplemented-workflow smoke checks.

The old scaffold branch is not used by this implementation. The new package does
not import the MVP. Controller/execution settings and concrete scenario/record
schemas are defined. Generation, evaluation, and interface configuration sections
remain placeholders.

## New package structure

```text
src/cutin/
  run.py                              Minimal user workflow entry
  config.py                           User configuration sections
  core/
    scenario.py                       Resolved ScenarioConfig
    record.py                         ScenarioRecord
  generation/
    interface.py                      Generator interface
    optim/
      optim_generator.py              OptimGenerator; owns the optimization loop
      translators/
        aggressiveness.py             Aggressiveness -> evaluation settings
        searchspace.py                Conditions -> searchable parameters
      optimizers/
        interface.py                  Candidate proposal/feedback interface
        random/optimizer.py           RandomOptimizer skeleton
        bo/optimizer.py               BOOptimizer skeleton
  simulation/
    interface.py                      Simulator interface
    adapters/
      highway_env/
        adapter.py                    Backend translation and physics loop
        controllers/
          idm.py                      Per-vehicle native IDM wrapper
          cutin.py                    Four-phase constant-speed cut-in control
  evaluation/
    evaluator.py                      Evaluation composition/interface
    metrics/                          TTC, LCT, robustness skeletons
  interfaces/
    cli.py                            CLI placeholder
    web.py                            Web placeholder
```

`highway_env` uses an underscore so it is a valid Python package name. The intended
flow is `run.run(UserConfig) -> OptimGenerator.run()`. The generator will assemble
its translators, optimizer, simulator, and evaluator. Its optimization loop and
the adapter's physics loop have separate owners. Saving and playback belong at
the outer user-workflow boundary.

The generation flow still stops explicitly at the unimplemented generator. Direct
simulation is available through the adapter as shown below. No search strategy,
robustness formula, or candidate-selection policy is implemented. The new CLI is
not registered yet.

## Run a concrete scenario

The direct API is:

```python
adapter = HighwayEnvAdapter(simulation_config)
record = adapter.run(scenario_config)
```

See [examples/run_simulation.py](examples/run_simulation.py) for a complete runnable
construction, with no placeholder inputs:

```sh
uv run --locked python examples/run_simulation.py > /tmp/cutin-record.json
```

The example emits a headless 12-second episode: three native vehicles on three
straight lanes, a 2-second warmup, a gap-triggered merge, and native IDM following.
The adapter creates fresh road/vehicle/controller state per call and does not
render or write files. The example explicitly serializes `dataclasses.asdict(record)`;
`json.loads` reloads ordinary data, not a typed record or live simulator checkpoint.

### Scenario and execution inputs

`core/scenario.py` contains `RoadConfig`, `InitialVehicleState`, and `ScenarioConfig`.
Initial vehicle x is its world-coordinate centre; lane determines y and initial
heading is zero. Relative offsets/percentages must be resolved before simulation.
Ego/front initial speeds and desired IDM cruise speeds remain independent. Cut-in
initial speed is its target until FOLLOW. Cut-in targets ego's initial lane.

`SimulationConfig` holds backend and fixed physics/control rates. The current
concrete adapter is selected explicitly and executes highway-env; backend routing
for the generator is not implemented. Physics Hz must be an integer multiple of
controller Hz. The horizon ends at the first physics tick at or beyond `duration_s`.

### Record semantics

`core/record.py` contains `VehicleState`, `Frame`, and `ScenarioRecord`:

- Frames include tick zero, every physics tick, and the terminal frame. Time is
  `tick / record.simulation.physics_hz`.
- States include actual dimensions, heading, speed, and backend collision flags.
- Action fields describe the interval ending at that frame. They are None at
  tick zero, and otherwise reflect the backend action used during the step.
- Cut-in phase is observed after stepping. Completion can be recorded before the
  next control decision, so the first FOLLOW frame may still contain MERGE actions.
- The record carries scenario/execution settings, backend/controller versions, and
  actual prepare/merge start/completion times. Unobserved events are None.
- Current termination reasons are `horizon` and `collision`; no metric assessment
  is implied. An unfinished maneuver remains unfinished at the horizon.

There is no generic input validation, error recovery, or road-exit assessment in
this slice; scenes are assumed to fit the finite road, and backend exceptions
propagate. No TTC/LCT/robustness is stored or computed yet. Repeated execution is
tested in the same code/dependency environment; cross-version/platform bitwise
identity is not promised. Record-based playback has not been implemented yet.

## Controller slice

`IDMConfig` and `CutInConfig` live in `src/cutin/config.py`. `IDMController` wraps an
existing native `IDMVehicle`, applies parameters per instance, and disables MOBIL.
Its defaults are illustrative, not safety-calibrated. Native IDM also observes
lane speed limits, so road settings must agree with the requested cruise speeds.

`CutInController` uses an existing native `IDMVehicle` and an ego reference on the
same straight, parallel-lane road:

- WARMUP: keep the source lane and track the initial speed for 2 seconds by default.
- PREPARE: maintain that speed until the requested longitudinal bumper gap is
  reached or crossed between control ticks. Crossings during warmup do not latch.
  An unreachable target stays in PREPARE until the episode ends.
- MERGE: track a quintic lateral reference while keeping the initial speed target.
- FOLLOW: after the reference ends and the actual lateral position/heading meet
  completion tolerances, delegate to native IDM on the destination lane, using
  the separately configured `follow` settings. The vehicle is never replaced.

The controller exposes phase and preparation/merge start/completion times for the
record model. Reference duration is not a guarantee of actual LCT.
Collided vehicles do not transition to a successful completion.

All cut-in state transitions live in `update_phase()`. `act()` calls it with
`control_tick=True`, then dispatches actions by phase. The adapter advances
`Road.step()` once per physics tick and calls `update_phase(time_s)` with the
default `control_tick=False` to record completion without initiating a maneuver
between control decisions. It must not additionally call `Road.act()`, which
would overwrite custom actions. FOLLOW commands start on the next control tick
after observed completion.

`HighwayEnvAdapter.run()` uses this scheduling directly and copies backend values
into plain record data. The controller tests also exercise it without the adapter,
so policy behavior and record capture can be tested separately.

## Run the working MVP

```sh
uv sync --locked
uv run --locked cutin-mvp --help
uv run --locked cutin-mvp run examples/mvp/baseline.toml
uv run --locked cutin-mvp web --port 8000 --runs-directory runs
```

The scripts in `scripts/` still operate on the MVP. Its TOML examples live in
`examples/mvp/`; the new `examples/run_simulation.py` stays at the examples root.
MVP examples and saved artifacts retain their formats; the new framework does
not load or convert them.
The [historical MVP README](docs/mvp/README.md) contains the longer command and
artifact reference, with migration substitutions noted at the top. The archived
[design](docs/mvp/DESIGN.md), [handover](docs/mvp/HANDOVER.md), and
[implementation plan](docs/mvp/PLAN.md) are kept alongside it.

## Checks

```sh
uv run --locked pytest -q
uv run --locked ruff check .
uv run --locked ruff format --check .
uv build
```

No new dependencies are required. Both packages ship in the existing `cutin`
distribution; the working console command is now named `cutin-mvp`.

## License

[MIT](LICENSE). Dependencies retain their own licenses. Empirical falsification
and specification satisfaction are not proofs of general safety.

> Recap: The MVP remains runnable under cutin_mvp/cutin-mvp. The new package executes concrete scenarios into per-tick JSON-compatible records using native IDM and four-phase cut-in control. Evaluation, optimization, and playback remain pending.
