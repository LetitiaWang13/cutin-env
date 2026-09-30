# Cut-in

An incremental cut-in scenario generator/falsification project. See DESIGN.md
for architecture and PLAN.md for delivery order.

## Current scope

The foundation, a headless highway-env adapter, and a fixed-action runner are
implemented. There are no driving controllers, evaluator, CLI command, or
visualization application yet. All numerical defaults are illustrative, not safety-calibrated.
Only highway-env adapter/translation modules import the backend. Core and the
runner's public imports remain backend-independent.
Runtime dependencies are highway-env and NumPy (plus their FOSS dependencies).

```sh
uv sync --locked
uv run pytest
uv run ruff check .
uv run ruff format --check .
uv build
```

## Run real physics locally

From this repository directory:

```sh
uv sync --locked
uv run python scripts/probe_highway_env.py
# Optional: save resolved settings, initial/terminal states, and applied actions.
uv run python scripts/probe_highway_env.py --output runs/probe.json
# Coast for the whole 10-second horizon, without steering/acceleration.
uv run python scripts/probe_highway_env.py --steps 300 --acceleration 0 --steering 0
```

The default probe advances 30 physics ticks (one simulated second). Ego/front
coast; cut-in receives raw acceleration 1 m/s^2 and steering 0.02 rad. It prints
initial/final states and termination. No display server or window is needed.
The step-limited probe reports `cancelled`, not a completed safety assessment.
Collisions/road exit stop it early. Existing output files are never overwritten.

**This is not the scenario controller:** cut-in start time, lane-change duration,
and car-following policy are not executed yet. The next step implements them.
JSON export records the scenario, simulator settings/backend version, integer
ticks, actual bounded actions, and per-vehicle collision flags. Metadata is plain
JSON-compatible data, not a backend object or an event framework.

## Try the configuration locally

From this repository directory:

```sh
uv run python - <<'PY'
import tomllib
from dataclasses import replace
from pathlib import Path

from cutin.core import ScenarioConfig, validate_scenario

scenario = ScenarioConfig.from_dict(
    tomllib.loads(Path("examples/baseline.toml").read_text())
)
print("Initial conditions:", scenario.initial)
print("Cut-in:", scenario.cut_in)

changed = replace(scenario, cut_in=replace(scenario.cut_in, start_time_s=3.0))
validate_scenario(changed)
print("Changed cut-in:", changed.cut_in)

try:
    ScenarioConfig.from_dict({"cut_in": {"lane_change_duration_s": 0}})
except ValueError as error:
    print("Validation error:", error)
PY
```

## Reading the code

- `core/scenario_config.py`: scenario records plus a single `validate_scenario()` boundary.
  `from_dict()` parses TOML/JSON values and validates automatically. Python-created
  or modified records must be validated explicitly before execution. Constructors
  do not perform hidden checks. Unknown fields and invalid external values fail.
- `core/state.py`: vehicle roles/states and `StateSnapshot(tick, vehicles)`.
  Integer tick is the authoritative simulation clock; seconds = tick / physics_hz.
  `vehicles` is a flat tuple, supporting multiple vehicles per role.
  `snapshot.vehicles_for_role(Role.FRONT)` returns all matches as a tuple, or `()`.
  Producers supply tuples; there are no container wrappers or constructor hooks.
- `core/action.py`: acceleration/steering requests and role-keyed applied actions.
- `core/log.py`: scenario, entries, `termination_reason` (TerminationReason enum),
  optional diagnostic, and resolved simulator settings.
  No event framework or per-constructor lifecycle validation.

TOML is for hand-edited scenarios; `to_dict()` expands defaults into JSON-compatible
values for future API/artifact output. Core performs no file I/O. Three-vehicle
restrictions live only in scenario validation, not in generic snapshots/logs.
The simulation component is now split into:

- `simulation/config.py`: SimulatorConfig (frequencies and generic actuator limits).
- `simulation/runner.py`: `simulate()` validates inputs, supplies dt, bounds generic
  actions, decides termination, assembles the log, and closes the adapter. It runs
  zero/fixed actions for now; controller scheduling is the next slice.
- `simulation/adapters/highway_env_translation.py`: `build_scene()` translates a
  validated scenario into backend Road/Vehicle objects; `translate_actions()` maps
  action fields and enforces backend speed limits; `collect_snapshot()` copies
  backend state into our records. No Translator classes or config framework.
- `simulation/adapters/highway_env.py`: HighwayEnvAdapter coordinates those
  translations and calls highway-env physics. It accepts `step(actions, dt_s)`
  and has no SimulatorConfig dependency. It counts actual physics ticks; the
  runner uses returned ticks rather than maintaining another clock.

The probe is now only argument parsing, calling the same `simulate()` entry
point, and displaying/exporting the returned log, not a second simulation loop.
The runner's small Protocol allows fake-adapter tests without backend objects.

The probe supports highway-env's default 5 m x 2 m vehicle model and speeds up
to 40 m/s. Unsupported dimensions/speeds and nonintegral tick horizons fail
explicitly. There is no AbstractEnv wrapper, random traffic, reward, viewer,
controller framework, or backend registry.

Coordinates use SI units: x follows the road, lane centres are
`lane * lane_width_m`, and heading is radians from positive x. Positions are
vehicle centres. `start_time_s` begins the lateral reference at the next control
tick; PREPARE is deferred. Requested lane-change duration is not guaranteed
physical completion time.
