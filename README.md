# Cut-in

An incremental cut-in scenario generator/falsification project. See DESIGN.md
for architecture and PLAN.md for delivery order.

## Current scope

The foundation and a real headless highway-env stepping probe are implemented.
There is no controlled scenario runner, evaluator, CLI command, or visualization
application yet. All numerical defaults are illustrative, not safety-calibrated.
Only the highway adapter imports the backend; core remains dependency-free.
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
uv run python scripts/probe_highway.py
# Optional: save resolved settings, initial/terminal states, and applied actions.
uv run python scripts/probe_highway.py --output runs/probe.json
# Coast for the whole 10-second horizon, without steering/acceleration.
uv run python scripts/probe_highway.py --steps 300 --acceleration 0 --steering 0
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
The adapter validates scenario/settings at initialization. `simulation/config.py`
contains SimulatorConfig; `simulation/adapters/highway.py` builds a RoadNetwork,
Road, and three base Vehicles directly. It exposes snapshot(), step(actions)
returning LogEntry, conservative road_exited(), settings metadata, and close().
Each step advances exactly 1 / physics_hz seconds, clamps actuator/speed bounds,
and logs the action actually used by the backend. A future runner uses returned
ticks for controller scheduling, without a second clock.

The probe supports highway-env's default 5 m x 2 m vehicle model and speeds up
to 40 m/s. Unsupported dimensions/speeds and nonintegral tick horizons fail
explicitly. There is no AbstractEnv wrapper, random traffic, reward, viewer,
controller framework, or backend registry.

Coordinates use SI units: x follows the road, lane centres are
`lane * lane_width_m`, and heading is radians from positive x. Positions are
vehicle centres. `start_time_s` begins the lateral reference at the next control
tick; PREPARE is deferred. Requested lane-change duration is not guaranteed
physical completion time.
